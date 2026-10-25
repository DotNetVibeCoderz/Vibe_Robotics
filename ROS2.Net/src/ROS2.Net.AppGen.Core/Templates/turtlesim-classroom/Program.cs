// {{ProjectName}} — turtlesim classroom: closed-loop square and circle (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Simulation;
using TurtlePose = ROS2.Net.Msgs.Turtlesim.Pose;

await using var context = await AppTarget.CreateContextAsync(args);
// Simulator: built-in turtlesim. Other targets: an existing turtlesim_node on the network.
using var turtlesim = AppTarget.IsSimulator ? new TurtlesimNode(context) : null;
turtlesim?.Start();
Console.WriteLine(AppTarget.Describe());
using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);

var student = context.CreateNode("student");
var cmd = student.CreatePublisher<Twist>("/turtle1/cmd_vel");
TurtlePose? pose = null;
student.CreateSubscription<TurtlePose>("/turtle1/pose", p => pose = p);
while (pose is null)
{
    await Task.Delay(50);
}

async Task DriveTo(double x, double y)
{
    // Proportional controller: turn toward the target, then drive.
    while (true)
    {
        var p = pose!;
        var dx = x - p.X;
        var dy = y - p.Y;
        var distance = Math.Sqrt((dx * dx) + (dy * dy));
        if (distance < 0.05)
        {
            cmd.Publish(new Twist());
            return;
        }

        var error = Math.Atan2(Math.Sin(Math.Atan2(dy, dx) - p.Theta), Math.Cos(Math.Atan2(dy, dx) - p.Theta));
        cmd.Publish(new Twist
        {
            Linear = new Vector3 { X = Math.Abs(error) < 0.3 ? Math.Min(2.0, 1.5 * distance) : 0 },
            Angular = new Vector3 { Z = 4.0 * error },
        });
        await Task.Delay(30);
    }
}

Console.WriteLine("Lesson 1: a closed-loop square");
foreach (var (x, y) in new[] { (8.0, 3.0), (8.0, 8.0), (3.0, 8.0), (3.0, 3.0) })
{
    await DriveTo(x, y);
    Console.WriteLine($"  reached ({pose!.X:0.00}, {pose.Y:0.00})");
}

Console.WriteLine("Lesson 2: a circle — constant linear and angular velocity (r = v/ω = 1.5 m)");
for (var i = 0; i < 70; i++)
{
    cmd.Publish(new Twist { Linear = new Vector3 { X = 1.5 }, Angular = new Vector3 { Z = 1.0 } });
    await Task.Delay(100);
}

cmd.Publish(new Twist());
Console.WriteLine("Class dismissed!");
cts.Cancel();
await spin;
