using System.Globalization;
using YamlDotNet.RepresentationModel;

namespace ROS2.Net;

/// <summary>
/// Reads ROS 2 parameter files:
/// <code>
/// /robot1/controller:        # or "controller", "/**", "/robot1/*"
///   ros__parameters:
///     gain: 1.5
///     limits: { max_speed: 2.0 }   # becomes "limits.max_speed"
/// </code>
/// </summary>
public static class ParameterYaml
{
    /// <summary>Parameters that apply to the node <paramref name="fullyQualifiedNodeName"/>, in file order (later entries win).</summary>
    public static IReadOnlyList<Parameter> Load(string yaml, string fullyQualifiedNodeName)
    {
        var stream = new YamlStream();
        stream.Load(new StringReader(yaml));
        var result = new List<Parameter>();
        foreach (var doc in stream.Documents)
        {
            if (doc.RootNode is not YamlMappingNode root)
            {
                continue;
            }

            Collect(root, string.Empty, fullyQualifiedNodeName, result);
        }

        return result;
    }

    public static IReadOnlyList<Parameter> LoadFile(string path, string fullyQualifiedNodeName) =>
        Load(File.ReadAllText(path), fullyQualifiedNodeName);

    /// <summary>Every node's parameters in the file, keyed by node pattern.</summary>
    public static IReadOnlyDictionary<string, IReadOnlyList<Parameter>> LoadAll(string yaml)
    {
        var stream = new YamlStream();
        stream.Load(new StringReader(yaml));
        var all = new Dictionary<string, IReadOnlyList<Parameter>>(StringComparer.Ordinal);
        foreach (var doc in stream.Documents)
        {
            if (doc.RootNode is YamlMappingNode root)
            {
                CollectAll(root, string.Empty, all);
            }
        }

        return all;
    }

    /// <summary>Serializes parameters as a ROS 2 parameter file for <paramref name="fullyQualifiedNodeName"/>.</summary>
    public static string Dump(string fullyQualifiedNodeName, IEnumerable<Parameter> parameters)
    {
        var sb = new System.Text.StringBuilder();
        sb.Append(fullyQualifiedNodeName).AppendLine(":");
        sb.AppendLine("  ros__parameters:");
        foreach (var p in parameters.OrderBy(p => p.Name, StringComparer.Ordinal))
        {
            sb.Append("    ").Append(p.Name).Append(": ").AppendLine(FormatValue(p.Value));
        }

        return sb.ToString();
    }

    private static string FormatValue(ParameterValue v) => v.Type switch
    {
        ParameterType.String => Quote(v.AsString()),
        ParameterType.StringArray => "[" + string.Join(", ", v.AsStringArray().Select(Quote)) + "]",
        _ => v.ToString(),
    };

    private static string Quote(string s) => "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"") + "\"";

    private static void Collect(YamlMappingNode node, string path, string fqn, List<Parameter> result)
    {
        foreach (var (keyNode, value) in node.Children)
        {
            var key = ((YamlScalarNode)keyNode).Value ?? string.Empty;
            if (key == "ros__parameters")
            {
                if (Matches(path, fqn) && value is YamlMappingNode parameters)
                {
                    Flatten(parameters, string.Empty, result);
                }

                continue;
            }

            if (value is YamlMappingNode child)
            {
                var childPath = path.Length == 0 ? key : path + "/" + key.TrimStart('/');
                Collect(child, childPath, fqn, result);
            }
        }
    }

    private static void CollectAll(YamlMappingNode node, string path, Dictionary<string, IReadOnlyList<Parameter>> all)
    {
        foreach (var (keyNode, value) in node.Children)
        {
            var key = ((YamlScalarNode)keyNode).Value ?? string.Empty;
            if (key == "ros__parameters" && value is YamlMappingNode parameters)
            {
                var list = new List<Parameter>();
                Flatten(parameters, string.Empty, list);
                all[path] = list;
            }
            else if (value is YamlMappingNode child)
            {
                CollectAll(child, path.Length == 0 ? key : path + "/" + key.TrimStart('/'), all);
            }
        }
    }

    /// <summary>Matches a node pattern (<c>/**</c>, <c>/ns/*</c>, <c>node</c>, <c>/ns/node</c>) against a fully qualified node name.</summary>
    public static bool Matches(string pattern, string fqn)
    {
        if (pattern is "/**" or "**")
        {
            return true;
        }

        var p = pattern.StartsWith('/') ? pattern : "/" + pattern;
        if (p == fqn)
        {
            return true;
        }

        // A bare node name matches that node in any namespace only when written without a namespace.
        if (!pattern.StartsWith('/') && !pattern.Contains('/') && fqn.EndsWith("/" + pattern, StringComparison.Ordinal))
        {
            return fqn.Count(c => c == '/') == 1;
        }

        return WildcardMatch(p.Split('/', StringSplitOptions.RemoveEmptyEntries), 0, fqn.Split('/', StringSplitOptions.RemoveEmptyEntries), 0);
    }

    private static bool WildcardMatch(string[] p, int pi, string[] n, int ni)
    {
        if (pi == p.Length)
        {
            return ni == n.Length;
        }

        if (p[pi] == "**")
        {
            for (var k = ni; k <= n.Length; k++)
            {
                if (WildcardMatch(p, pi + 1, n, k))
                {
                    return true;
                }
            }

            return false;
        }

        return ni < n.Length && (p[pi] == "*" || p[pi] == n[ni]) && WildcardMatch(p, pi + 1, n, ni + 1);
    }

    private static void Flatten(YamlMappingNode node, string prefix, List<Parameter> result)
    {
        foreach (var (keyNode, value) in node.Children)
        {
            var name = prefix + ((YamlScalarNode)keyNode).Value;
            switch (value)
            {
                case YamlMappingNode map:
                    Flatten(map, name + ".", result);
                    break;
                case YamlSequenceNode seq:
                    result.Add(new Parameter(name, FromSequence(seq)));
                    break;
                case YamlScalarNode scalar:
                    result.Add(new Parameter(name, FromScalar(scalar)));
                    break;
            }
        }
    }

    private static ParameterValue FromSequence(YamlSequenceNode seq)
    {
        var items = seq.Children.OfType<YamlScalarNode>().Select(FromScalar).ToList();
        if (items.Count == 0)
        {
            return ParameterValue.From(Array.Empty<string>());
        }

        if (items.All(i => i.Type == ParameterType.Bool))
        {
            return ParameterValue.From(items.Select(i => i.AsBool()).ToArray());
        }

        if (items.All(i => i.Type == ParameterType.Integer))
        {
            return ParameterValue.From(items.Select(i => i.AsInteger()).ToArray());
        }

        if (items.All(i => i.Type is ParameterType.Integer or ParameterType.Double))
        {
            return ParameterValue.From(items.Select(i => i.AsDouble()).ToArray());
        }

        return ParameterValue.From(items.Select(i => i.ToString()).ToArray());
    }

    private static ParameterValue FromScalar(YamlScalarNode scalar)
    {
        var text = scalar.Value ?? string.Empty;
        if (scalar.Style is YamlDotNet.Core.ScalarStyle.SingleQuoted or YamlDotNet.Core.ScalarStyle.DoubleQuoted)
        {
            return ParameterValue.From(text);
        }

        return ParseScalar(text);
    }

    /// <summary>YAML 1.2 core-schema scalar resolution: bool, int, float, else string.</summary>
    public static ParameterValue ParseScalar(string text)
    {
        var t = text.Trim();
        switch (t)
        {
            case "true" or "True" or "TRUE":
                return ParameterValue.From(true);
            case "false" or "False" or "FALSE":
                return ParameterValue.From(false);
            case ".inf" or ".Inf" or "+.inf":
                return ParameterValue.From(double.PositiveInfinity);
            case "-.inf" or "-.Inf":
                return ParameterValue.From(double.NegativeInfinity);
            case ".nan" or ".NaN":
                return ParameterValue.From(double.NaN);
        }

        if (long.TryParse(t, NumberStyles.AllowLeadingSign, CultureInfo.InvariantCulture, out var l))
        {
            return ParameterValue.From(l);
        }

        if (t.StartsWith("0x", StringComparison.OrdinalIgnoreCase) && long.TryParse(t[2..], NumberStyles.HexNumber, CultureInfo.InvariantCulture, out var hex))
        {
            return ParameterValue.From(hex);
        }

        if (t.Length > 0 && (char.IsDigit(t[0]) || t[0] is '-' or '+' or '.') &&
            double.TryParse(t, NumberStyles.Float, CultureInfo.InvariantCulture, out var d))
        {
            return ParameterValue.From(d);
        }

        return ParameterValue.From(t.Length >= 2 && (t[0] == '"' || t[0] == '\'') && t[^1] == t[0] ? t[1..^1] : t);
    }

    /// <summary>Parses a scalar or a flow sequence such as <c>[1, 2, 3]</c>.</summary>
    public static ParameterValue ParseScalarOrList(string text)
    {
        var t = text.Trim();
        if (t.StartsWith('[') && t.EndsWith(']'))
        {
            var stream = new YamlStream();
            stream.Load(new StringReader("v: " + t));
            if (stream.Documents[0].RootNode is YamlMappingNode m && m.Children[new YamlScalarNode("v")] is YamlSequenceNode seq)
            {
                return FromSequence(seq);
            }
        }

        return ParseScalar(t);
    }
}
