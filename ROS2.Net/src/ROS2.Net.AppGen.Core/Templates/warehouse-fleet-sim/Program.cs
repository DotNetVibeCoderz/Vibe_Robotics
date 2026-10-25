// {{ProjectName}} — warehouse AMR fleet simulator (ROS2.Net template).
// Each robot has its own namespace (/robot1, /robot2, ...) with cmd_vel, odom, scan, imu and battery topics.
using ROS2.Net;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Simulation;

await using var context = await AppTarget.CreateContextAsync(args);

// Simulator: a 3-robot warehouse. Physical robot: drives the one real robot (ROS2NET_ROBOT_NS).
using var sim = AppTarget.StartSimulator(context, ScenarioKind.Warehouse, robots: 3);
Console.WriteLine(AppTarget.Describe());

// One reactive "brain" per robot: drive forward, turn away from obstacles in front.
var brains = context.CreateNode("fleet_brains");
var index = 0;
foreach (var ns in AppTarget.RobotNamespaces())
{
    var turn = (index++ % 2 == 0) ? 1.0 : -1.0;
    var cmd = brains.CreatePublisher<Twist>($"{ns}/cmd_vel");
    brains.CreateSubscription<LaserScan>($"{ns}/scan", scan =>
    {
        var front = ScanMath.MinRange(scan, -0.5, 0.5);
        var twist = new Twist();
        if (front < 0.6)
        {
            twist.Angular.Z = 1.2 * turn;
        }
        else
        {
            twist.Linear.X = Math.Min(0.4, (front - 0.3) * 0.5);
            twist.Angular.Z = 0.1 * turn;
        }

        cmd.Publish(AppTarget.Limit(twist)); // SAFETY: velocity limits for real robots
    }, QosProfile.SensorData);
}

var report = context.CreateNode("fleet_report");
report.CreateTimer(TimeSpan.FromSeconds(2), () =>
{
    if (AppTarget.World is not { } world)
    {
        return;
    }

    foreach (var r in world.Robots)
    {
        Console.WriteLine($"[{world.Time,6:0.0}s] {r.Name}: ({r.Pose.X:0.00}, {r.Pose.Y:0.00}) odometer {r.Odometer:0.0} m, collisions {r.CollisionCount}, battery {(r.Battery?.Percentage ?? 0):0.#}");
    }
});

Console.WriteLine("Warehouse fleet running — press Ctrl+C to stop.");
await context.SpinAsync();

static class ScanMath
{
    /// <summary>Minimum valid range in the angular sector [from, to] (radians, robot frame).</summary>
    public static double MinRange(LaserScan scan, double from, double to)
    {
        var min = double.PositiveInfinity;
        for (var i = 0; i < scan.Ranges.Length; i++)
        {
            double a = scan.AngleMin + (i * scan.AngleIncrement);
            a = Math.Atan2(Math.Sin(a), Math.Cos(a));
            var r = scan.Ranges[i];
            if (a >= from && a <= to && float.IsFinite(r) && r > scan.RangeMin)
            {
                min = Math.Min(min, (double)r);
            }
        }

        return min;
    }
}
