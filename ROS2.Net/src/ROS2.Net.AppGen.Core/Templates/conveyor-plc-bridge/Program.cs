// {{ProjectName}} — conveyor PLC bridge with a lifecycle node (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Msgs.StdSrvs;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await AppTarget.CreateContextAsync(args);
using var conveyor = new ConveyorNode(context);

using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);

await conveyor.ConfigureAsync();
await conveyor.ActivateAsync();

// A supervisor: starts the belt, raises the speed through the parameter service, stops after a while.
var supervisor = context.CreateNode("supervisor");
var run = supervisor.CreateClient<SetBoolRequest, SetBoolResponse>("/conveyor/run");
using var parameters = new ParameterClient(supervisor, "/conveyor");
supervisor.CreateSubscription<StdMsgs.Int32>("/conveyor/items", m => Console.WriteLine($"items packed: {m.Data}"));
await run.WaitForServiceAsync(TimeSpan.FromSeconds(5));

Console.WriteLine((await run.CallAsync(new SetBoolRequest { Data = true }, TimeSpan.FromSeconds(2))).Message);
await Task.Delay(3000);
Console.WriteLine($"set speed 0.8 m/s: {(await parameters.SetAsync("speed_mps", 0.8)).Successful}");
Console.WriteLine($"set speed 5 m/s: {(await parameters.SetAsync("speed_mps", 5.0)).Reason}");
await Task.Delay(3000);
Console.WriteLine((await run.CallAsync(new SetBoolRequest { Data = false }, TimeSpan.FromSeconds(2))).Message);

await conveyor.DeactivateAsync();
await conveyor.ShutdownAsync();
cts.Cancel();
await spin;

/// <summary>The conveyor controller. Publishes only while active; the belt runs only after /conveyor/run true.</summary>
internal sealed class ConveyorNode(Ros2Context context) : LifecycleNode(context, "conveyor")
{
    private LifecyclePublisher<StdMsgs.Float64>? _speed;
    private LifecyclePublisher<StdMsgs.Int32>? _items;
    private bool _running;
    private double _travelled;
    private int _count;

    protected override ValueTask<CallbackReturn> OnConfigure(LifecycleState previous)
    {
        DeclareParameter("speed_mps", 0.4, new ParameterDescriptor
        {
            Description = "Belt speed",
            FloatingPointRange = new FloatingPointRange(0.0, 1.5),
        });
        DeclareParameter("item_spacing_m", 0.5);
        _speed = CreateLifecyclePublisher<StdMsgs.Float64>("~/speed");
        _items = CreateLifecyclePublisher<StdMsgs.Int32>("~/items");
        CreateService<SetBoolRequest, SetBoolResponse>("~/run", req =>
        {
            _running = req.Data;
            return new SetBoolResponse { Success = true, Message = _running ? "belt started" : "belt stopped" };
        });
        CreateTimer(TimeSpan.FromMilliseconds(100), () =>
        {
            var v = _running ? GetParameter<double>("speed_mps") : 0;
            _travelled += v * 0.1;
            var spacing = GetParameter<double>("item_spacing_m");
            if (_travelled >= spacing)
            {
                _travelled -= spacing;
                _items?.Publish(new StdMsgs.Int32 { Data = ++_count });
            }

            _speed?.Publish(new StdMsgs.Float64 { Data = v });
        });
        return ValueTask.FromResult(CallbackReturn.Success);
    }
}
