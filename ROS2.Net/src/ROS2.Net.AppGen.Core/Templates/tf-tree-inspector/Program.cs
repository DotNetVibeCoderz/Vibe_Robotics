// {{ProjectName}} — inspect the TF tree like view_frames / tf2_echo (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Simulation;
using ROS2.Net.Tf2;

await using var context = await AppTarget.CreateContextAsync(args);
using var sim = AppTarget.StartSimulator(context, ScenarioKind.Warehouse, robots: 2, new SimulationNodeOptions { PublishMapToOdom = true });
Console.WriteLine(AppTarget.Describe());

// Simulated frames are prefixed (robot1/base_link); a real robot usually publishes plain base_link.
var frames = AppTarget.World is { } w ? w.Robots.Select(r => $"{r.Name}/base_link").ToArray() : new[] { "base_link" };
var fixedFrame = AppTarget.IsSimulator ? "map" : "odom";

var node = context.CreateNode("tf_inspector");
var buffer = new TfBuffer();
using var listener = new TransformListener(buffer, node);
using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(6));
var spin = context.SpinAsync(cts.Token);

await buffer.WaitForTransformAsync(fixedFrame, frames[0], RosTime.Zero, TimeSpan.FromSeconds(5));
Console.WriteLine(buffer.AllFramesAsString());
Console.WriteLine("Mermaid (paste into a Markdown viewer):");
Console.WriteLine(buffer.ToMermaid());

for (var i = 0; i < 4; i++)
{
    foreach (var frame in frames)
    {
        if (buffer.TryLookupTransform(fixedFrame, frame, RosTime.Zero, out var tf, out var error))
        {
            var (_, _, yaw) = tf.Rotation.ToRpy();
            Console.WriteLine($"{fixedFrame} → {frame}: t=({tf.Translation.X:0.00}, {tf.Translation.Y:0.00}, {tf.Translation.Z:0.00}) yaw={yaw * 180 / Math.PI:0.0}°");
        }
        else
        {
            Console.WriteLine($"{frame}: {error}");
        }
    }

    await Task.Delay(1000);
}

await spin;
