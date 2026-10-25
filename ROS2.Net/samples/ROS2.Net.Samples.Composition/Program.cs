// Component composition: several nodes in one container process. The in-process middleware
// already delivers by reference; with a network middleware (RTPS) set UseIntraProcessComms = true
// to keep zero-copy delivery between components, and the container becomes controllable with
// `ros2 component load /ComponentManager <assembly> <Type>`.
using ROS2.Net;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await Ros2Context.CreateAsync(new Ros2ContextOptions { Arguments = args });
using var container = new ComponentContainer(context);
container.RegisterAssembly(typeof(Producer).Assembly);
container.Load<Producer>();
container.Load(typeof(Producer).Assembly.GetName().Name!, "demo::Consumer");

foreach (var c in container.Components)
{
    Console.WriteLine($"[{c.UniqueId}] {c.FullNodeName} ({c.PluginName})");
}

using var stop = new CancellationTokenSource(TimeSpan.FromSeconds(3));
await context.SpinAsync(stop.Token);

[RosComponent("demo::Producer")]
public sealed class Producer : Node
{
    public Producer(Ros2Context context, NodeOptions options)
        : base(context, "producer", options)
    {
        var pub = CreatePublisher<StdMsgs.Int64>("counter");
        long i = 0;
        CreateTimer(TimeSpan.FromMilliseconds(250), () => pub.Publish(new StdMsgs.Int64 { Data = ++i }));
    }
}

[RosComponent("demo::Consumer")]
public sealed class Consumer : Node
{
    public Consumer(Ros2Context context, NodeOptions options)
        : base(context, "consumer", options)
    {
        CreateSubscription<StdMsgs.Int64>("counter", (m, info) =>
            Console.WriteLine($"consumer got {m.Data} (intra-process: {info.FromIntraProcess})"));
    }
}
