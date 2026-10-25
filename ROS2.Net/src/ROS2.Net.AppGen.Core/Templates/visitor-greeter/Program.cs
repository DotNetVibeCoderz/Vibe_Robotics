// {{ProjectName}} — reception greeter robot (ROS2.Net template).
// A second simulated robot plays the visitor walking in and out of the lobby.
using ROS2.Net;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Simulation;
using SimPose = ROS2.Net.Simulation.Pose2D;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await AppTarget.CreateContextAsync(args);
// Simulator: robot1 greets, robot2 plays a visitor. Physical robot: the real robot greets real people.
using var sim = AppTarget.StartSimulator(context, ScenarioKind.Empty, robots: 2);
Console.WriteLine(AppTarget.Describe());
if (AppTarget.World is { } world)
{
    world.Robots[0].Teleport(new SimPose(2, 10, 0));        // greeter
    world.Robots[1].Teleport(new SimPose(8, 10, Math.PI));  // visitor
}

var greeterNs = AppTarget.RobotNamespaces()[0];

var greeter = context.CreateNode("greeter");
var speech = greeter.CreatePublisher<StdMsgs.String>("/greeter/speech");
var visitors = 0;
var present = false;
greeter.CreateSubscription<LaserScan>(greeterNs + "/scan", scan =>
{
    // Something within 1.5 m in front of the desk counts as a visitor.
    var near = false;
    for (var i = 0; i < scan.Ranges.Length; i++)
    {
        var a = scan.AngleMin + (i * scan.AngleIncrement);
        if (Math.Abs(Math.Atan2(Math.Sin(a), Math.Cos(a))) < 0.6 && scan.Ranges[i] < 1.5)
        {
            near = true;
            break;
        }
    }

    if (near && !present)
    {
        visitors++;
        var hello = DateTime.Now.Hour < 12 ? "Good morning" : "Good afternoon";
        speech.Publish(new StdMsgs.String { Data = $"{hello}! Welcome — you are visitor #{visitors} today." });
    }

    present = near;
}, QosProfile.SensorData);
greeter.CreateSubscription<StdMsgs.String>("/greeter/speech", s => Console.WriteLine("🤖 " + s.Data));

// The simulated visitor walks toward the desk and back, every ~16 s (simulator only).
if (AppTarget.IsPhysicalRobot)
{
    Console.WriteLine("Greeter on duty — Ctrl+C to stop.");
    await context.SpinAsync();
    return;
}

var visitor = context.CreateNode("visitor");
var walk = visitor.CreatePublisher<Twist>("/robot2/cmd_vel");
var t = 0.0;
visitor.CreateTimer(TimeSpan.FromMilliseconds(100), () =>
{
    t += 0.1;
    var phase = t % 16;
    walk.Publish(new Twist { Linear = new Vector3 { X = phase < 8 ? 0.6 : -0.6 } });
});

Console.WriteLine("Greeter on duty — Ctrl+C to stop.");
await context.SpinAsync();
