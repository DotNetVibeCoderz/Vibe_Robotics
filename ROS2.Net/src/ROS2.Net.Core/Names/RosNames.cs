namespace ROS2.Net;

/// <summary>Thrown when a node, namespace, topic or service name breaks the ROS 2 naming rules.</summary>
public sealed class InvalidRosNameException(string message) : ArgumentException(message);

/// <summary>
/// ROS 2 name validation, expansion and remapping, following the
/// <see href="https://design.ros2.org/articles/topic_and_service_names.html">Topic and Service Names</see> design.
/// </summary>
public static class RosNames
{
    /// <summary>Validates a node base name: alphanumerics and underscores, not starting with a digit.</summary>
    public static void ValidateNodeName(string name)
    {
        if (string.IsNullOrEmpty(name))
        {
            throw new InvalidRosNameException("Node name must not be empty.");
        }

        if (char.IsDigit(name[0]))
        {
            throw new InvalidRosNameException($"Node name '{name}' must not start with a number.");
        }

        foreach (var c in name)
        {
            if (!(char.IsAsciiLetterOrDigit(c) || c == '_'))
            {
                throw new InvalidRosNameException($"Node name '{name}' may only contain alphanumerics and '_'.");
            }
        }
    }

    /// <summary>Normalizes a namespace: ensures a leading slash and strips a trailing one.</summary>
    public static string NormalizeNamespace(string? ns)
    {
        if (string.IsNullOrEmpty(ns) || ns == "/")
        {
            return "/";
        }

        if (ns[0] != '/')
        {
            ns = "/" + ns;
        }

        ns = ns.TrimEnd('/');
        ValidateFullyQualifiedName(ns, "namespace");
        return ns;
    }

    /// <summary>Fully qualified node name, e.g. <c>/robot1/talker</c>.</summary>
    public static string FullyQualifiedNodeName(string ns, string name) =>
        ns == "/" ? "/" + name : ns + "/" + name;

    /// <summary>
    /// Expands a topic or service name relative to a node: <c>~</c> is the private namespace
    /// (<c>/ns/node</c>), relative names are prefixed with the node namespace, and <c>{node}</c> /
    /// <c>{ns}</c> substitutions are applied.
    /// </summary>
    public static string ExpandTopicName(string name, string nodeName, string nodeNamespace)
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            throw new InvalidRosNameException("Topic or service name must not be empty.");
        }

        var ns = NormalizeNamespace(nodeNamespace);
        name = name.Replace("{node}", nodeName, StringComparison.Ordinal)
                   .Replace("{ns}", ns == "/" ? string.Empty : ns, StringComparison.Ordinal)
                   .Replace("{namespace}", ns == "/" ? string.Empty : ns, StringComparison.Ordinal);

        string expanded;
        if (name[0] == '/')
        {
            expanded = name;
        }
        else if (name[0] == '~')
        {
            var priv = FullyQualifiedNodeName(ns, nodeName);
            expanded = name.Length == 1 ? priv : name[1] == '/' ? priv + name[1..] : throw new InvalidRosNameException($"'~' must be followed by '/' in '{name}'.");
        }
        else
        {
            expanded = (ns == "/" ? "/" : ns + "/") + name;
        }

        ValidateFullyQualifiedName(expanded, "topic");
        return expanded;
    }

    /// <summary>Validates a fully qualified topic, service or namespace name.</summary>
    public static void ValidateFullyQualifiedName(string name, string what = "name")
    {
        if (name.Length == 0 || name[0] != '/')
        {
            throw new InvalidRosNameException($"The {what} '{name}' must be absolute.");
        }

        if (name.Length > 1 && name[^1] == '/')
        {
            throw new InvalidRosNameException($"The {what} '{name}' must not end with '/'.");
        }

        if (name.Contains("//", StringComparison.Ordinal))
        {
            throw new InvalidRosNameException($"The {what} '{name}' must not contain '//'.");
        }

        var tokenStart = true;
        foreach (var c in name)
        {
            if (c == '/')
            {
                tokenStart = true;
                continue;
            }

            if (!(char.IsAsciiLetterOrDigit(c) || c == '_'))
            {
                throw new InvalidRosNameException($"The {what} '{name}' contains the invalid character '{c}'.");
            }

            if (tokenStart && char.IsDigit(c))
            {
                throw new InvalidRosNameException($"A token of the {what} '{name}' starts with a number.");
            }

            tokenStart = false;
        }
    }

    /// <summary>True when the name is hidden (a token starts with an underscore), e.g. <c>/_action/status</c>.</summary>
    public static bool IsHidden(string name) =>
        name.Split('/', StringSplitOptions.RemoveEmptyEntries).Any(t => t.StartsWith('_'));

    /// <summary>
    /// Applies remapping rules (<c>from:=to</c>) to a fully qualified name. Rules are matched against
    /// the expanded name and against the name as written.
    /// </summary>
    public static string Remap(string expandedName, string originalName, string nodeName, string nodeNamespace, IReadOnlyList<RemapRule> rules)
    {
        foreach (var rule in rules)
        {
            if (rule.NodeName is { } target && target != nodeName)
            {
                continue;
            }

            if (rule.From.StartsWith("__", StringComparison.Ordinal))
            {
                continue; // __node / __ns are handled when the node is created.
            }

            var from = rule.From;
            if (from == originalName || ExpandTopicName(from, nodeName, nodeNamespace) == expandedName)
            {
                return ExpandTopicName(rule.To, nodeName, nodeNamespace);
            }
        }

        return expandedName;
    }
}

/// <summary>A <c>--ros-args -r [node:]from:=to</c> remapping rule.</summary>
public sealed record RemapRule(string From, string To, string? NodeName = null)
{
    /// <summary>Parses <c>from:=to</c> or <c>node:from:=to</c>.</summary>
    public static RemapRule Parse(string text)
    {
        var sep = text.IndexOf(":=", StringComparison.Ordinal);
        if (sep <= 0)
        {
            throw new ArgumentException($"Remap rule '{text}' must have the form from:=to.", nameof(text));
        }

        var from = text[..sep];
        var to = text[(sep + 2)..];
        string? node = null;
        var colon = from.IndexOf(':');
        if (colon > 0)
        {
            node = from[..colon];
            from = from[(colon + 1)..];
        }

        return new RemapRule(from, to, node);
    }
}
