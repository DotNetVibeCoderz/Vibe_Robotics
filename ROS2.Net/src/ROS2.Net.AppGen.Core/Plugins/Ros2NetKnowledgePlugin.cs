using System.ComponentModel;
using System.Reflection;
using System.Text;
using Microsoft.SemanticKernel;

namespace ROS2.Net.AppGen.Plugins;

/// <summary>
/// ROS2.Net reference knowledge for the assistant: the library's own documentation (embedded at build
/// time from <c>docs/en</c>, so it stays in sync), the message type catalog reflected from the generated
/// interface types, and the package map.
/// </summary>
public sealed class Ros2NetKnowledgePlugin
{
    private static readonly Lazy<Dictionary<string, string>> s_docs = new(LoadDocs);

    /// <summary>Topic keyword → documentation page(s).</summary>
    private static readonly (string[] Keys, string[] Pages)[] s_topics =
    [
        (["start", "getting", "install", "hello", "quick"], ["getting-started"]),
        (["core", "node", "publisher", "subscription", "topic", "service", "action", "parameter", "lifecycle", "executor", "qos", "timer", "context", "composition", "logging"], ["concepts"]),
        (["msg", "message", "interface", "codegen", "generator", "rosidl", "srv"], ["interfaces"]),
        (["rtps", "dds", "network", "fastdds", "cyclone"], ["rtps"]),
        (["rosbridge", "websocket", "browser"], ["rosbridge"]),
        (["hosting", "di", "dependency", "host", "worker", "appsettings"], ["hosting"]),
        (["security", "sros2", "keystore", "enclave"], ["security"]),
        (["tf", "tf2", "transform", "frame"], ["tf2"]),
        (["bag", "rosbag", "mcap", "record", "playback", "dataset"], ["rosbag2"]),
        (["navigation", "nav2", "planner", "costmap", "controller_server", "amcl", "ekf", "behavior", "bt", "path"], ["navigation"]),
        (["slam", "mapping", "loop"], ["slam"]),
        (["perception", "pointcloud", "point", "cloud", "image", "camera", "lidar", "detection", "yolo", "onnx_vision"], ["perception"]),
        (["native", "rust"], ["native"]),
        (["control", "ros2_control", "pid", "diffdrive", "trajectory", "hardware", "gripper"], ["control"]),
        (["manipulation", "arm", "urdf", "ik", "kinematics", "moveit", "rrt", "pick"], ["manipulation"]),
        (["simulation", "simulator", "sim", "world", "scenario", "turtlesim", "robot"], ["simulation"]),
        (["ai", "rl", "reinforcement", "gym", "onnx", "llm", "commander", "anomaly"], ["ai"]),
        (["avalonia", "desktop", "wpf", "control_ui"], ["avalonia"]),
        (["blazor", "web", "razor", "component"], ["blazor"]),
        (["cli", "ros2net"], ["cli"]),
        (["template", "dotnet new"], ["templates"]),
    ];

    [KernelFunction("list_reference_topics")]
    [Description("Lists the ROS2.Net documentation topics available through get_api_reference.")]
    public string ListReferenceTopics() =>
        "Topics: " + string.Join(", ", s_docs.Value.Keys.Order(StringComparer.Ordinal)) +
        "\nAlso: packages (get_packages), message types (list_message_types / get_message_type).";

    [KernelFunction("get_api_reference")]
    [Description("Returns the official ROS2.Net documentation for a topic, with accurate API names and code samples. Topics include: concepts (nodes, pub/sub, services, actions, parameters, lifecycle, executors, QoS), interfaces, simulation, navigation, slam, tf2, rosbag2, perception, control, manipulation, ai, hosting, avalonia, blazor, rtps, rosbridge, security, cli. ALWAYS read the relevant page before writing code that uses a ROS2.Net library.")]
    public string GetApiReference([Description("Topic keyword or page name, e.g. 'simulation', 'navigation', 'blazor', 'parameters'")] string topic)
    {
        var key = topic.Trim().ToLowerInvariant();
        var docs = s_docs.Value;
        var pages = new List<string>();
        if (docs.ContainsKey(key))
        {
            pages.Add(key);
        }
        else
        {
            foreach (var (keys, ps) in s_topics)
            {
                if (keys.Any(k => key.Contains(k, StringComparison.Ordinal) || k.Contains(key, StringComparison.Ordinal)))
                {
                    pages.AddRange(ps);
                }
            }
        }

        pages = pages.Distinct().Where(docs.ContainsKey).Take(2).ToList();
        if (pages.Count == 0)
        {
            return $"No page matches '{topic}'. {ListReferenceTopics()}";
        }

        var sb = new StringBuilder();
        foreach (var p in pages)
        {
            sb.AppendLine($"===== docs/en/{p}.md =====").AppendLine(docs[p]);
        }

        var text = sb.ToString();
        return text.Length > 30_000 ? text[..30_000] + "\n… [truncated]" : text;
    }

    [KernelFunction("get_packages")]
    [Description("Lists the ROS2.Net NuGet packages and what each provides, with the namespaces to import.")]
    public string GetPackages() => """
        ROS2.Net (umbrella: Core + Msgs + Rmw.Rtps + Rmw.Rosbridge + Tf2 + Bag + Hosting)
        ROS2.Net.Core — namespace ROS2.Net: Ros2Context, Node, Publisher<T>, Subscription<T>, services, actions, parameters, LifecycleNode, executors, QosProfile, RosTime. In-process middleware built in.
        ROS2.Net.Msgs — namespaces ROS2.Net.Msgs.<Package>, e.g. ROS2.Net.Msgs.GeometryMsgs (Twist, Pose...), SensorMsgs, NavMsgs, StdMsgs (alias it: using StdMsgs = ROS2.Net.Msgs.StdMsgs;), StdSrvs, Tf2Msgs, VisualizationMsgs, TrajectoryMsgs, DiagnosticMsgs, ExampleInterfaces, Turtlesim, Nav2Msgs, ControlMsgs. Helpers: ROS2.Net.Msgs.MessageExtensions (QuaternionFromYaw, Yaw(), Twist2D, WithStamp).
        ROS2.Net.Rmw.Rtps — options.UseRtps(): talk to ROS 2 over DDS/RTPS (using ROS2.Net.Rmw.Rtps).
        ROS2.Net.Rmw.Rosbridge — options.UseRosbridge("ws://host:9090").
        ROS2.Net.Hosting — services.AddRos2(...), AddRos2Node<T>() (namespace ROS2.Net.Hosting).
        ROS2.Net.Tf2 — TfBuffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster, Transform3d (namespace ROS2.Net.Tf2).
        ROS2.Net.Bag — BagRecorder, BagPlayer, BagReader, BagInfo, BagExporter (namespace ROS2.Net.Bag).
        ROS2.Net.Navigation — OccupancyGridMap, Costmap2D, planners, controllers, BT, Amcl, Ekf2D, OnlineSlam, Nav2-style server nodes.
        ROS2.Net.Perception (+ .Onnx, .Native) — PointCloud, filters, segmentation, ICP, images, camera models, detectors.
        ROS2.Net.Control — ControllerManager, PID, DiffDriveController, JointTrajectoryController, hardware interfaces.
        ROS2.Net.Manipulation — UrdfModel, KinematicChain, IkSolver, PlanningScene, RrtConnectPlanner, MoveGroup.
        ROS2.Net.Simulation — SimWorld, Scenarios, robots, sensors, SimulationNode, TurtlesimNode (namespace ROS2.Net.Simulation).
        ROS2.Net.AI — RobotGymEnv, RlTrainer, OnnxModelRunner, NaturalLanguageCommanderNode, anomaly detectors.
        ROS2.Net.Avalonia — Avalonia controls: TeleopJoystick, LaserScanView, OccupancyGridView, SimWorldView, PlotView, ...; RosUi helpers; RosTheme.
        ROS2.Net.Blazor — Blazor components (namespace ROS2.Net.Blazor.Components), AddRos2Blazor().
        All packages target net10.0. Use the version configured in the generator settings.
        """;

    private static readonly Lazy<Dictionary<string, string>> s_api = new(() => LoadResources("api/"));

    [KernelFunction("get_type_api")]
    [Description("Returns the exact public API (constructors with parameter names and defaults, properties, methods, events) of ROS2.Net types whose name contains the given text, e.g. 'SimulationNode', 'SimWorld', 'AStarPlanner', 'TeleopJoystick', 'IkSolver', 'BagRecorder', 'Node'. Use it to get signatures right before writing code.")]
    public string GetTypeApi([Description("Type name or part of it (case-insensitive)")] string typeName)
    {
        var needle = typeName.Trim();
        var sb = new StringBuilder();
        var matches = 0;
        foreach (var (assembly, text) in s_api.Value.OrderBy(kv => kv.Key, StringComparer.Ordinal))
        {
            var lines = text.Replace("\r\n", "\n").Split('\n');
            for (var i = 0; i < lines.Length; i++)
            {
                var line = lines[i];
                if (!(line.StartsWith("class ", StringComparison.Ordinal) || line.StartsWith("struct ", StringComparison.Ordinal) ||
                      line.StartsWith("enum ", StringComparison.Ordinal) || line.StartsWith("interface ", StringComparison.Ordinal)))
                {
                    continue;
                }

                var fullName = line.Split(' ')[1];
                var simple = fullName[(fullName.LastIndexOf('.') + 1)..];
                var exact = simple.Equals(needle, StringComparison.OrdinalIgnoreCase);
                if (!exact && !simple.Contains(needle, StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }

                if (++matches > 12)
                {
                    break;
                }

                sb.AppendLine($"// {assembly}").AppendLine(line);
                for (var j = i + 1; j < lines.Length && lines[j].StartsWith("  ", StringComparison.Ordinal); j++)
                {
                    var member = lines[j];
                    if (member.Contains("<Clone>$", StringComparison.Ordinal) || member.Contains(" GetHashCode(", StringComparison.Ordinal) || member.Contains(" Equals(", StringComparison.Ordinal))
                    {
                        continue;
                    }

                    sb.AppendLine(member);
                }
            }
        }

        if (matches == 0)
        {
            return $"No ROS2.Net type matches '{typeName}'. Message types are described by get_message_type.";
        }

        var result = sb.ToString();
        return result.Length > 20_000 ? result[..20_000] + "\n… [truncated: be more specific]" : result;
    }

    [KernelFunction("list_message_types")]
    [Description("Lists ROS message/service/action types available in ROS2.Net with their C# class names. Filter by ROS package (e.g. 'sensor_msgs') or keyword.")]
    public string ListMessageTypes([Description("ROS package or keyword, empty for all packages")] string filter = "")
    {
        EnsureTypesLoaded();
        var lines = RosTypeRegistry.Messages
            .Where(m => !m.Name.Contains("/srv/", StringComparison.Ordinal) && !m.Name.Contains("/action/", StringComparison.Ordinal))
            .Select(m => $"{m.Name} → {m.ClrType.FullName}")
            .Concat(RosTypeRegistry.Services.Select(s => $"{s.Name} → {s.Request.ClrType.FullName} / {s.Response.ClrType.Name}"))
            .Concat(RosTypeRegistry.Actions.Select(a => $"{a.Name} → {a.Goal.ClrType.FullName} / {a.Result.ClrType.Name} / {a.Feedback.ClrType.Name}"))
            .Distinct()
            .Where(l => string.IsNullOrWhiteSpace(filter) || l.Contains(filter, StringComparison.OrdinalIgnoreCase))
            .Order(StringComparer.Ordinal)
            .ToList();
        if (lines.Count == 0)
        {
            return $"No types match '{filter}'.";
        }

        if (string.IsNullOrWhiteSpace(filter))
        {
            var packages = lines.Select(l => l.Split('/')[0]).Distinct().ToList();
            return $"{lines.Count} types in packages: {string.Join(", ", packages)}. Call again with a package name to list its types.";
        }

        return string.Join('\n', lines.Take(300));
    }

    [KernelFunction("get_message_type")]
    [Description("Shows a ROS type's definition and its generated C# class with exact property names and types, e.g. 'sensor_msgs/msg/LaserScan', 'geometry_msgs/Twist', 'std_srvs/srv/SetBool', 'example_interfaces/action/Fibonacci'.")]
    public string GetMessageType(string rosType)
    {
        EnsureTypesLoaded();
        var name = rosType.Trim();
        var sb = new StringBuilder();
        if (RosTypeRegistry.TryGetService(name, out var srv))
        {
            sb.AppendLine($"Service {srv.Name}: request {srv.Request.ClrType.FullName}, response {srv.Response.ClrType.FullName}");
            sb.AppendLine($"Server: node.CreateService<{srv.Request.ClrType.Name}, {srv.Response.ClrType.Name}>(\"name\", req => new {srv.Response.ClrType.Name} {{ ... }});");
            sb.AppendLine($"Client: node.CreateClient<{srv.Request.ClrType.Name}, {srv.Response.ClrType.Name}>(\"name\"); await client.CallAsync(new() {{ ... }}, TimeSpan.FromSeconds(2));");
            Describe(sb, srv.Request);
            Describe(sb, srv.Response);
            return sb.ToString();
        }

        if (RosTypeRegistry.TryGetAction(name, out var act))
        {
            sb.AppendLine($"Action {act.Name}: goal {act.Goal.ClrType.FullName}, result {act.Result.ClrType.Name}, feedback {act.Feedback.ClrType.Name}");
            sb.AppendLine($"Server: node.CreateActionServer<{act.Goal.ClrType.Name}, {act.Result.ClrType.Name}, {act.Feedback.ClrType.Name}>(\"name\", async (goal, ct) => {{ goal.PublishFeedback(new()); return new {act.Result.ClrType.Name}(); }});");
            sb.AppendLine($"Client: node.CreateActionClient<{act.Goal.ClrType.Name}, {act.Result.ClrType.Name}, {act.Feedback.ClrType.Name}>(\"name\"); await client.SendGoalAndWaitAsync(goal, fb => ...);");
            Describe(sb, act.Goal);
            Describe(sb, act.Result);
            Describe(sb, act.Feedback);
            return sb.ToString();
        }

        if (RosTypeRegistry.TryGetMessage(name, out var msg))
        {
            Describe(sb, msg);
            return sb.ToString();
        }

        return $"Unknown type '{rosType}'. Use list_message_types with the package name.";
    }

    private static void Describe(StringBuilder sb, MessageTypeInfo info)
    {
        sb.AppendLine($"--- {info.Name} → C# {info.ClrType.FullName}");
        var props = info.ClrType.GetProperties(BindingFlags.Public | BindingFlags.Instance)
            .Where(p => p.GetCustomAttribute<System.Text.Json.Serialization.JsonPropertyNameAttribute>() is not null);
        foreach (var p in props)
        {
            var ros = p.GetCustomAttribute<System.Text.Json.Serialization.JsonPropertyNameAttribute>()!.Name;
            sb.AppendLine($"  {TypeName(p.PropertyType)} {p.Name}   // {ros}");
        }

        var consts = info.ClrType.GetFields(BindingFlags.Public | BindingFlags.Static).Where(f => f.IsLiteral).ToList();
        if (consts.Count > 0)
        {
            sb.AppendLine("  constants: " + string.Join(", ", consts.Select(c => $"{c.Name}={c.GetRawConstantValue()}")));
        }

        sb.AppendLine("  definition:");
        foreach (var line in info.Definition.Split('\n').Where(l => l.Trim().Length > 0 && !l.TrimStart().StartsWith('#')))
        {
            sb.AppendLine("    " + line.Trim());
        }
    }

    private static string TypeName(Type t) => t.IsArray ? TypeName(t.GetElementType()!) + "[]" : t.Namespace?.StartsWith("ROS2.Net.Msgs", StringComparison.Ordinal) == true ? t.FullName!.Replace("ROS2.Net.Msgs.", string.Empty, StringComparison.Ordinal) : t.Name switch
    {
        "Double" => "double",
        "Single" => "float",
        "Int32" => "int",
        "UInt32" => "uint",
        "Int64" => "long",
        "UInt64" => "ulong",
        "Int16" => "short",
        "UInt16" => "ushort",
        "Byte" => "byte",
        "SByte" => "sbyte",
        "Boolean" => "bool",
        "String" => "string",
        _ => t.Name,
    };

    private static void EnsureTypesLoaded()
    {
        RosTypeRegistry.EnsureLoaded(typeof(Ros2Context).Assembly);
        RosTypeRegistry.EnsureLoaded(typeof(ROS2.Net.Msgs.MessageExtensions).Assembly);
    }

    private static Dictionary<string, string> LoadDocs()
    {
        var docs = LoadResources("docs/");
        docs.Remove("README");
        return docs;
    }

    private static Dictionary<string, string> LoadResources(string prefix)
    {
        var asm = typeof(Ros2NetKnowledgePlugin).Assembly;
        var result = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (var name in asm.GetManifestResourceNames().Where(n => n.StartsWith(prefix, StringComparison.Ordinal)))
        {
            using var stream = asm.GetManifestResourceStream(name)!;
            using var reader = new StreamReader(stream);
            result[Path.GetFileNameWithoutExtension(name[prefix.Length..])] = reader.ReadToEnd();
        }

        return result;
    }
}
