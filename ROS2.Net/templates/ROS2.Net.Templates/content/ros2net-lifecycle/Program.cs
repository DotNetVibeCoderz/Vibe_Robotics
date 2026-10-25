// Ros2NetLifecycle — a managed ROS 2 node built with ROS2.Net (made by Gravicode Studios, led by Kang Fadhil).
//
// The node starts Unconfigured. Drive it from another terminal:
//   ros2net service call /sensor_driver/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"   # configure
//   ros2net service call /sensor_driver/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"   # activate
// or with stock ROS 2: `ros2 lifecycle set /sensor_driver configure`.
// Pass --autostart to configure and activate immediately.
using Microsoft.Extensions.Logging;
using ROS2.Net;
using ROS2.Net.Rmw.Rtps;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

var options = new Ros2ContextOptions { Arguments = args, HandleShutdownSignals = true }.UseRtps();
if (Environment.GetEnvironmentVariable("ROS2NET_RMW") is { Length: > 0 })
{
    options.Rmw = null; // honor ROS2NET_RMW when set
}

await using var context = await Ros2Context.CreateAsync(options);
using var node = new SensorDriver(context);
node.StateChanged += (from, to) => Console.WriteLine($"[{node.Name}] {LifecycleNode.Label(from)} -> {LifecycleNode.Label(to)}");

if (args.Contains("--autostart"))
{
    await node.ConfigureAsync();
    await node.ActivateAsync();
}

await context.SpinAsync();

/// <summary>Simulates a sensor driver: allocates resources on configure, streams only while active.</summary>
internal sealed class SensorDriver(Ros2Context context) : LifecycleNode(context, "sensor_driver")
{
    private LifecyclePublisher<StdMsgs.Float64>? _reading;
    private RosTimer? _timer;

    protected override ValueTask<CallbackReturn> OnConfigure(LifecycleState previous)
    {
        var rate = DeclareParameterIfMissing("rate_hz", 10.0);
        _reading = CreateLifecyclePublisher<StdMsgs.Float64>("reading", QosProfile.SensorData);
        _timer = CreateTimer(TimeSpan.FromSeconds(1 / rate), () =>
            _reading.Publish(new StdMsgs.Float64 { Data = Math.Sin(Clock.Now().Seconds) }));
        Logger.LogInformation("Configured at {Rate} Hz", rate);
        return ValueTask.FromResult(CallbackReturn.Success);
    }

    protected override ValueTask<CallbackReturn> OnActivate(LifecycleState previous)
    {
        Logger.LogInformation("Streaming on {Topic}", _reading!.TopicName);
        return ValueTask.FromResult(CallbackReturn.Success);
    }

    protected override ValueTask<CallbackReturn> OnDeactivate(LifecycleState previous)
    {
        Logger.LogInformation("Paused");
        return ValueTask.FromResult(CallbackReturn.Success);
    }

    protected override ValueTask<CallbackReturn> OnCleanup(LifecycleState previous)
    {
        _timer?.Dispose();
        _reading?.Dispose();
        return ValueTask.FromResult(CallbackReturn.Success);
    }

    protected override ValueTask<CallbackReturn> OnShutdown(LifecycleState previous)
    {
        Context.Shutdown("lifecycle shutdown");
        return ValueTask.FromResult(CallbackReturn.Success);
    }

    private double DeclareParameterIfMissing(string name, double fallback) =>
        HasParameter(name) ? GetParameter<double>(name) : DeclareParameter(name, fallback);
}
