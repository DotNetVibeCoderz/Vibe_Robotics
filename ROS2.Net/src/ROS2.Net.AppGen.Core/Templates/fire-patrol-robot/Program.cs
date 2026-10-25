// {{ProjectName}} — fire-detection patrol (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Simulation;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await AppTarget.CreateContextAsync(args);
using var sim = AppTarget.StartSimulator(context, ScenarioKind.Warehouse);
Console.WriteLine(AppTarget.Describe());

// Checkpoints: simulator waypoints, or your site's coordinates in the odom frame.
var checkpoints = AppTarget.World is { } world
    ? new[] { "entrance", "charger", "loading_dock" }.Where(world.Waypoints.ContainsKey).Select(n => (n, (world.Waypoints[n].X, world.Waypoints[n].Y))).ToArray()
    : [("north", (3.0, 0.0)), ("east", (3.0, 3.0)), ("start", (0.0, 0.0))];
var rnd = new Random(5);

// Simulated hotspots (simulator only). A real robot reports its thermal camera's maximum on thermal/max_temperature.
var hotspots = AppTarget.IsSimulator
    ? Enumerable.Range(0, 3).Select(_ => (X: 3 + (rnd.NextDouble() * 18), Y: 2 + (rnd.NextDouble() * 12), Temp: 60 + (rnd.NextDouble() * 140))).ToList()
    : [];

var patrol = context.CreateNode("fire_patrol");
var cmd = patrol.CreatePublisher<Twist>(AppTarget.Topic("cmd_vel"));
var alerts = patrol.CreatePublisher<StdMsgs.String>("/alerts");
patrol.CreateSubscription<StdMsgs.String>("/alerts", a => Console.WriteLine("🔥 " + a.Data));
var target = 0;
var confirmUntil = 0.0;
patrol.CreateSubscription<ROS2.Net.Msgs.SensorMsgs.Temperature>(AppTarget.Topic("thermal/max_temperature"), t =>
{
    if (t.TemperatureValue > 60)
    {
        alerts.Publish(new StdMsgs.String { Data = $"thermal camera reports {t.TemperatureValue:0} °C" });
    }
});
patrol.CreateSubscription<Odometry>(AppTarget.Topic("odom"), odom =>
{
    var q = odom.Pose.Pose.Orientation;
    var p = (X: odom.Pose.Pose.Position.X, Y: odom.Pose.Pose.Position.Y, Theta: Math.Atan2(2 * ((q.W * q.Z) + (q.X * q.Y)), 1 - (2 * ((q.Y * q.Y) + (q.Z * q.Z)))));
    var now = RosTime.FromMsg(odom.Header.Stamp).Seconds;

    // "Thermal camera": anything within 2 m is sensed; temperature falls off with distance.
    foreach (var h in hotspots.ToList())
    {
        var d = Math.Sqrt(Math.Pow(h.X - p.X, 2) + Math.Pow(h.Y - p.Y, 2));
        if (d < 2.0)
        {
            var severity = h.Temp > 150 ? "CRITICAL" : h.Temp > 90 ? "HIGH" : "WATCH";
            alerts.Publish(new StdMsgs.String { Data = $"{severity}: {h.Temp:0} °C at ({h.X:0.0}, {h.Y:0.0}), robot at ({p.X:0.0}, {p.Y:0.0})" });
            hotspots.Remove(h);
            confirmUntil = now + 3;    // stop and confirm for 3 s
        }
    }

    if (now < confirmUntil)
    {
        cmd.Publish(new Twist());
        return;
    }

    var (name, goal) = checkpoints[target % checkpoints.Length];
    var dx = goal.X - p.X;
    var dy = goal.Y - p.Y;
    if (Math.Sqrt((dx * dx) + (dy * dy)) < 0.5)
    {
        Console.WriteLine($"[{now,5:0}s] checkpoint {name} clear");
        target++;
        return;
    }

    var error = Math.Atan2(Math.Sin(Math.Atan2(dy, dx) - p.Theta), Math.Cos(Math.Atan2(dy, dx) - p.Theta));
    cmd.Publish(AppTarget.Limit(new Twist { Linear = new Vector3 { X = Math.Abs(error) < 0.5 ? 0.45 : 0.05 }, Angular = new Vector3 { Z = 2.0 * error } }));
});

Console.WriteLine($"Patrolling {checkpoints.Length} checkpoints with {hotspots.Count} hidden hotspots — Ctrl+C to stop.");
await context.SpinAsync();
