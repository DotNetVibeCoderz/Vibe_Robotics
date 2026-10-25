using System.Collections.Concurrent;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Text;

namespace ROS2.Net;

/// <summary>Runtime metadata of a message type.</summary>
public sealed class MessageTypeInfo
{
    internal MessageTypeInfo(string name, string ddsName, string definition, IReadOnlyList<string> dependencies, Type clrType, Func<IRosMessage> factory)
    {
        Name = name;
        DdsName = ddsName;
        Definition = definition;
        Dependencies = dependencies;
        ClrType = clrType;
        Factory = factory;
    }

    /// <summary>ROS 2 type name, e.g. <c>sensor_msgs/msg/Imu</c>.</summary>
    public string Name { get; }

    /// <summary>DDS type name, e.g. <c>sensor_msgs::msg::dds_::Imu_</c>.</summary>
    public string DdsName { get; }

    public string Definition { get; }

    public IReadOnlyList<string> Dependencies { get; }

    public Type ClrType { get; }

    /// <summary>Creates a default instance.</summary>
    public Func<IRosMessage> Factory { get; }

    /// <summary>Metadata for a generated type.</summary>
    public static MessageTypeInfo Of<T>()
        where T : class, IRosMessage<T>, new() =>
        new(T.TypeName, T.DdsTypeName, T.Definition, T.Dependencies, typeof(T), static () => new T());

    /// <summary>Metadata for a type whose name is only known at runtime (action wire wrappers).</summary>
    public static MessageTypeInfo Create(string name, Type clrType, Func<IRosMessage> factory, string definition = "", IReadOnlyList<string>? dependencies = null) =>
        new(name, ToDdsName(name), definition, dependencies ?? [], clrType, factory);

    /// <summary><c>pkg/msg/Name</c> → <c>pkg::msg::dds_::Name_</c>.</summary>
    public static string ToDdsName(string rosName)
    {
        var parts = rosName.Split('/');
        return parts.Length == 3 ? $"{parts[0]}::{parts[1]}::dds_::{parts[2]}_" : rosName;
    }

    /// <summary><c>pkg::msg::dds_::Name_</c> → <c>pkg/msg/Name</c>. Returns the input when it is already a ROS name.</summary>
    public static string FromDdsName(string ddsName)
    {
        var parts = ddsName.Split("::");
        if (parts.Length == 4 && parts[2] == "dds_" && parts[3].EndsWith('_'))
        {
            return $"{parts[0]}/{parts[1]}/{parts[3][..^1]}";
        }

        return ddsName;
    }

    public override string ToString() => Name;
}

/// <summary>Runtime metadata of a service type.</summary>
public sealed record ServiceTypeInfo(string Name, MessageTypeInfo Request, MessageTypeInfo Response);

/// <summary>Runtime metadata of an action type.</summary>
public sealed record ActionTypeInfo(string Name, MessageTypeInfo Goal, MessageTypeInfo Result, MessageTypeInfo Feedback);

/// <summary>
/// Process-wide registry of known interface types. Generated assemblies register their types from a
/// module initializer, so anything referenced (and loaded) is resolvable by name — which is what
/// <c>topic echo</c>, bag playback and rosbridge need when only a type string is available.
/// </summary>
public static class RosTypeRegistry
{
    private static readonly ConcurrentDictionary<string, MessageTypeInfo> s_messages = new(StringComparer.Ordinal);
    private static readonly ConcurrentDictionary<Type, MessageTypeInfo> s_byClr = new();
    private static readonly ConcurrentDictionary<string, ServiceTypeInfo> s_services = new(StringComparer.Ordinal);
    private static readonly ConcurrentDictionary<string, ActionTypeInfo> s_actions = new(StringComparer.Ordinal);

    public static IEnumerable<MessageTypeInfo> Messages => s_messages.Values.Distinct().OrderBy(m => m.Name, StringComparer.Ordinal);

    public static IEnumerable<ServiceTypeInfo> Services => s_services.Values.OrderBy(s => s.Name, StringComparer.Ordinal);

    public static IEnumerable<ActionTypeInfo> Actions => s_actions.Values.OrderBy(a => a.Name, StringComparer.Ordinal);

    public static void RegisterMessage<T>()
        where T : class, IRosMessage<T>, new() => Register(MessageTypeInfo.Of<T>());

    public static void Register(MessageTypeInfo info)
    {
        s_messages[info.Name] = info;
        s_messages[info.DdsName] = info;
        s_messages[ShortName(info.Name)] = info;
        s_byClr[info.ClrType] = info;
    }

    public static void RegisterService<TRequest, TResponse>()
        where TRequest : class, IRosMessage<TRequest>, IRosServiceRequest<TRequest, TResponse>, new()
        where TResponse : class, IRosMessage<TResponse>, new()
    {
        var req = MessageTypeInfo.Of<TRequest>();
        var res = MessageTypeInfo.Of<TResponse>();
        Register(req);
        Register(res);
        var info = new ServiceTypeInfo(TRequest.ServiceTypeName, req, res);
        s_services[info.Name] = info;
        s_services[ShortName(info.Name)] = info;
    }

    public static void RegisterAction<TGoal, TResult, TFeedback>()
        where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
        where TResult : class, IRosMessage<TResult>, new()
        where TFeedback : class, IRosMessage<TFeedback>, new()
    {
        var goal = MessageTypeInfo.Of<TGoal>();
        var result = MessageTypeInfo.Of<TResult>();
        var feedback = MessageTypeInfo.Of<TFeedback>();
        Register(goal);
        Register(result);
        Register(feedback);
        var info = new ActionTypeInfo(TGoal.ActionTypeName, goal, result, feedback);
        s_actions[info.Name] = info;
        s_actions[ShortName(info.Name)] = info;
    }

    /// <summary>Looks a message up by <c>pkg/msg/Name</c>, <c>pkg/Name</c> or its DDS name.</summary>
    public static bool TryGetMessage(string name, [System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out MessageTypeInfo? info) =>
        s_messages.TryGetValue(name, out info);

    public static bool TryGetMessage(Type clrType, [System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out MessageTypeInfo? info) =>
        s_byClr.TryGetValue(clrType, out info);

    public static bool TryGetService(string name, [System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out ServiceTypeInfo? info) =>
        s_services.TryGetValue(name, out info);

    public static bool TryGetAction(string name, [System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out ActionTypeInfo? info) =>
        s_actions.TryGetValue(name, out info);

    /// <summary>Creates a message instance by type name, or null when the type is unknown.</summary>
    public static IRosMessage? CreateMessage(string name) =>
        TryGetMessage(name, out var info) ? info.Factory() : null;

    /// <summary>Runs the module initializer of an assembly of generated types, registering them.</summary>
    public static void EnsureLoaded(Assembly assembly) =>
        RuntimeHelpers.RunModuleConstructor(assembly.ManifestModule.ModuleHandle);

    /// <summary>
    /// Builds the self-contained <c>ros2msg</c> definition used by rosbag2 MCAP schemas and type
    /// introspection: the type's own text followed by every transitive dependency.
    /// </summary>
    public static string GetFullDefinition(string name)
    {
        if (!TryGetMessage(name, out var root))
        {
            return string.Empty;
        }

        var sb = new StringBuilder(root.Definition.TrimEnd('\n'));
        var seen = new HashSet<string>(StringComparer.Ordinal) { root.Name };
        var queue = new Queue<string>(root.Dependencies);
        while (queue.Count > 0)
        {
            var dep = queue.Dequeue();
            if (!seen.Add(dep) || !TryGetMessage(dep, out var info))
            {
                continue;
            }

            sb.Append('\n').Append('=', 80).Append('\n');
            sb.Append("MSG: ").Append(ShortName(info.Name)).Append('\n');
            sb.Append(info.Definition.TrimEnd('\n'));
            foreach (var d in info.Dependencies)
            {
                queue.Enqueue(d);
            }
        }

        return sb.Append('\n').ToString();
    }

    /// <summary><c>pkg/msg/Name</c> → <c>pkg/Name</c>.</summary>
    public static string ShortName(string fullName)
    {
        var parts = fullName.Split('/');
        return parts.Length == 3 ? parts[0] + "/" + parts[2] : fullName;
    }
}
