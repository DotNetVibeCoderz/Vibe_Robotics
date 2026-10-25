// A managed (lifecycle) node: only publishes while Active. Drive it from code or with
// `ros2 lifecycle set /lc_talker configure|activate|deactivate|shutdown`.
using ROS2.Net;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await Ros2Context.CreateAsync(new Ros2ContextOptions { Arguments = args });
using var node = new LifecycleTalker(context);
node.StateChanged += (from, to) => Console.WriteLine($"  {LifecycleNode.Label(from)} -> {LifecycleNode.Label(to)}");

using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);

await node.ConfigureAsync();
await node.ActivateAsync();
await Task.Delay(1500);
await node.DeactivateAsync();
await node.CleanupAsync();
await node.ShutdownAsync();

cts.Cancel();
await spin;

internal sealed class LifecycleTalker(Ros2Context context) : LifecycleNode(context, "lc_talker")
{
    private LifecyclePublisher<StdMsgs.String>? _publisher;
    private RosTimer? _timer;
    private int _count;

    protected override ValueTask<CallbackReturn> OnConfigure(LifecycleState previous)
    {
        _publisher = CreateLifecyclePublisher<StdMsgs.String>("lifecycle_chatter");
        _timer = CreateTimer(TimeSpan.FromMilliseconds(300), () =>
        {
            var msg = new StdMsgs.String { Data = $"Lifecycle HelloWorld #{++_count}" };
            Console.WriteLine(_publisher.IsActivated ? $"Publishing: {msg.Data}" : "Inactive, not publishing");
            _publisher.Publish(msg);
        });
        return ValueTask.FromResult(CallbackReturn.Success);
    }

    protected override ValueTask<CallbackReturn> OnCleanup(LifecycleState previous)
    {
        _timer?.Dispose();
        _publisher?.Dispose();
        return ValueTask.FromResult(CallbackReturn.Success);
    }
}
