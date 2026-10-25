using System.Globalization;

namespace ROS2.Net.AppGen;

/// <summary>
/// Maps a <see cref="RunTarget"/> to the environment variables and arguments generated apps read
/// through their <c>AppTarget</c> helper.
/// </summary>
public static class RunTargetEnvironment
{
    public const string Target = "ROS2NET_TARGET";
    public const string Rmw = "ROS2NET_RMW";
    public const string DomainId = "ROS_DOMAIN_ID";
    public const string RosbridgeUrl = "ROS2NET_ROSBRIDGE_URL";
    public const string RobotNamespace = "ROS2NET_ROBOT_NS";
    public const string LocalhostOnly = "ROS_LOCALHOST_ONLY";
    public const string NetworkInterface = "ROS2NET_NETWORK_INTERFACE";
    public const string Scenario = "ROS2NET_SIM_SCENARIO";
    public const string MaxLinear = "ROS2NET_MAX_LINEAR";
    public const string MaxAngular = "ROS2NET_MAX_ANGULAR";

    /// <summary>The default target: software only, in-process middleware, simulator.</summary>
    public static RunTarget DefaultSimulator => new() { Name = "Simulator (in-process)", Kind = RunTargetKind.Simulator, Rmw = "inprocess" };

    /// <summary>Environment variables for <paramref name="target"/>.</summary>
    public static Dictionary<string, string> ToEnvironment(RunTarget target)
    {
        ArgumentNullException.ThrowIfNull(target);
        var rmw = string.IsNullOrWhiteSpace(target.Rmw) ? (target.Kind == RunTargetKind.Simulator ? "inprocess" : "rtps") : target.Rmw.Trim().ToLowerInvariant();
        if (target.Kind == RunTargetKind.PhysicalRobot && rmw == "inprocess")
        {
            // An in-process middleware cannot reach a real robot.
            rmw = "rtps";
        }

        var env = new Dictionary<string, string>(StringComparer.Ordinal)
        {
            [Target] = target.Kind == RunTargetKind.PhysicalRobot ? "robot" : "sim",
            [Rmw] = rmw,
            [LocalhostOnly] = target.LocalhostOnly ? "1" : "0",
        };

        if (target.DomainId is { } d)
        {
            env[DomainId] = d.ToString(CultureInfo.InvariantCulture);
        }

        if (rmw == "rosbridge" && !string.IsNullOrWhiteSpace(target.RosbridgeUrl))
        {
            env[RosbridgeUrl] = target.RosbridgeUrl.Trim();
        }

        if (!string.IsNullOrWhiteSpace(target.RobotNamespace))
        {
            env[RobotNamespace] = NormalizeNamespace(target.RobotNamespace);
        }

        if (!string.IsNullOrWhiteSpace(target.NetworkInterface))
        {
            env[NetworkInterface] = target.NetworkInterface.Trim();
        }

        if (target.Kind == RunTargetKind.Simulator && !string.IsNullOrWhiteSpace(target.Scenario))
        {
            env[Scenario] = target.Scenario.Trim();
        }

        if (target.MaxLinearSpeed > 0)
        {
            env[MaxLinear] = target.MaxLinearSpeed.ToString("R", CultureInfo.InvariantCulture);
        }

        if (target.MaxAngularSpeed > 0)
        {
            env[MaxAngular] = target.MaxAngularSpeed.ToString("R", CultureInfo.InvariantCulture);
        }

        foreach (var (k, v) in target.ExtraEnvironment)
        {
            env[k] = v;
        }

        return env;
    }

    /// <summary>
    /// Command-line arguments for <paramref name="target"/>: a <c>--ros-args -r __ns:=...</c> remap when a
    /// namespace is set (nodes then live in the robot's namespace, like ROS 2 launch files do).
    /// </summary>
    public static IReadOnlyList<string> ToArguments(RunTarget target) =>
        string.IsNullOrWhiteSpace(target.RobotNamespace)
            ? []
            : ["--ros-args", "-r", "__ns:=" + NormalizeNamespace(target.RobotNamespace)];

    /// <summary>The run targets a first run offers.</summary>
    public static List<RunTarget> Defaults() =>
    [
        DefaultSimulator,
        new() { Name = "Simulator (RTPS on localhost, visible to ros2 CLI)", Kind = RunTargetKind.Simulator, Rmw = "rtps", LocalhostOnly = true, DomainId = 0 },
        new() { Name = "Robot over DDS/RTPS (LAN)", Kind = RunTargetKind.PhysicalRobot, Rmw = "rtps", DomainId = 0, MaxLinearSpeed = 0.3, MaxAngularSpeed = 1.0 },
        new() { Name = "Robot via rosbridge", Kind = RunTargetKind.PhysicalRobot, Rmw = "rosbridge", RosbridgeUrl = "ws://192.168.1.100:9090", MaxLinearSpeed = 0.3, MaxAngularSpeed = 1.0 },
    ];

    private static string NormalizeNamespace(string ns)
    {
        var s = ns.Trim().TrimEnd('/');
        return s.StartsWith('/') ? s : "/" + s;
    }
}
