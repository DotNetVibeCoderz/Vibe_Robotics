// {{ProjectName}} — turtlesim playground (ROS2.Net template). Works with the stock ROS 2 turtlesim tutorials.
using ROS2.Net;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Simulation;
using TurtlePose = ROS2.Net.Msgs.Turtlesim.Pose;
using ROS2.Net.Msgs.Turtlesim;

await using var context = await AppTarget.CreateContextAsync(args);
// Simulator: the built-in turtlesim. Physical/external target: an existing turtlesim_node on the network.
using var turtlesim = AppTarget.IsSimulator ? new TurtlesimNode(context) : null;
turtlesim?.Start();
Console.WriteLine(AppTarget.Describe());

var node = context.CreateNode("playground");
using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);

// Spawn a second turtle, exactly like `ros2 service call /spawn turtlesim/srv/Spawn ...`.
var spawn = node.CreateClient<SpawnRequest, SpawnResponse>("/spawn");
await spawn.WaitForServiceAsync(TimeSpan.FromSeconds(5));
var spawned = await spawn.CallAsync(new SpawnRequest { X = 2, Y = 2, Theta = 0, Name = "turtle2" }, TimeSpan.FromSeconds(5));
Console.WriteLine($"Spawned {spawned.Name}");

var circle = node.CreatePublisher<Twist>("/turtle1/cmd_vel");
var square = node.CreatePublisher<Twist>("/turtle2/cmd_vel");
TurtlePose? lastPose = null;
node.CreateSubscription<TurtlePose>("/turtle1/pose", p => lastPose = p);

var tick = 0;
node.CreateTimer(TimeSpan.FromMilliseconds(100), () =>
{
    tick++;
    circle.Publish(new Twist { Linear = new Vector3 { X = 2.0 }, Angular = new Vector3 { Z = 1.0 } });

    // Square: 2 s straight, ~1.6 s turning 90°.
    var phase = tick % 36;
    square.Publish(phase < 20
        ? new Twist { Linear = new Vector3 { X = 1.0 } }
        : new Twist { Angular = new Vector3 { Z = Math.PI / 2 / 1.6 } });
});

node.CreateTimer(TimeSpan.FromSeconds(1), () =>
{
    foreach (var turtle in turtlesim?.World.Turtles ?? [])
    {
        Console.WriteLine($"{turtle.Name}: ({turtle.Pose.X:0.00}, {turtle.Pose.Y:0.00})");
    }

    if (lastPose is not null)
    {
        Console.WriteLine($"  /turtle1/pose says θ={lastPose.Theta:0.00} rad, v={lastPose.LinearVelocity:0.0} m/s");
    }
});

Console.WriteLine("Drawing — press Ctrl+C to stop.");
await context.WaitForShutdownAsync();
cts.Cancel();
await spin;
