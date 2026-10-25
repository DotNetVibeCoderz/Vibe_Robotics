using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.ExampleInterfaces;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Navigation;
using ROS2.Net.Navigation.Control;
using ROS2.Net.Navigation.Nodes;
using ROS2.Net.Simulation;
using NavPose = ROS2.Net.Navigation.Pose2D;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Gallery.Web.Services;

/// <summary>
/// The live robot world behind every gallery page: a simulated warehouse with one TurtleBot-like
/// robot (topics at the root namespace: /cmd_vel, /odom, /scan, /map, /tf …), a Nav2-style stack,
/// demo servers, and a wandering autopilot that pages can hold while they drive the robot.
/// </summary>
public sealed class GalleryRuntime(Ros2Context context, ILogger<GalleryRuntime> logger) : IHostedService, IDisposable
{
    private readonly List<IDisposable> _owned = [];
    private readonly CancellationTokenSource _cts = new();
    private int _holds;
    private Publisher<Twist>? _autopilotPub;
    private volatile float[] _lastRanges = [];
    private double _lastAngleMin, _lastAngleIncrement;

    public Ros2Context Context => context;

    public SimWorld World { get; private set; } = default!;

    public SimRobot Robot { get; private set; } = default!;

    public SimulationNode Simulation { get; private set; } = default!;

    /// <summary>Ground-truth pose provider for the navigation servers.</summary>
    public IRobotPoseProvider PoseProvider { get; private set; } = default!;

    public string MapFrame => "map";

    /// <summary>True while no page holds the autopilot.</summary>
    public bool AutopilotActive => Volatile.Read(ref _holds) == 0;

    public Task StartAsync(CancellationToken cancellationToken)
    {
        var sensors = new RobotSensorOptions { LidarRange = 8, Camera = true, LidarBeams = 360 };
        World = Scenarios.Create(ScenarioKind.Warehouse, robots: 1, sensors: sensors);
        Robot = World.Robots[0];
        Robot.Namespace = "/";
        Robot.Parameters.MaxLinearSpeed = 0.6;
        Robot.Parameters.MaxAngularSpeed = 2.0;
        if (Robot.Battery is { } battery)
        {
            battery.DrainFactor = 4; // visible drain during a demo session
        }

        Simulation = new SimulationNode(context, World, new SimulationNodeOptions { PublishMapToOdom = true, ClockRateHz = 20 });
        Simulation.Start();

        PoseProvider = new DelegatePoseProvider(() =>
        {
            var p = Robot.Pose;
            return (new NavPose(p.X, p.Y, p.Theta), new Velocity2D(Robot.LinearVelocity, Robot.AngularVelocity));
        });

        // Nav2-style stack: planner + controller + BT navigator against the simulator's /map.
        _owned.Add(new PlannerServerNode(context, PoseProvider));
        _owned.Add(new ControllerServerNode(context, PoseProvider));
        _owned.Add(new BtNavigatorNode(context, PoseProvider));

        StartDemoServers();
        StartAutopilot();
        logger.LogInformation("Gallery runtime started: {World} with {Robot}", World.Name, Robot.Name);
        return Task.CompletedTask;
    }

    /// <summary>Stops the autopilot until the returned handle is disposed (pages that drive the robot hold it).</summary>
    public IDisposable HoldAutopilot()
    {
        if (Interlocked.Increment(ref _holds) == 1)
        {
            _autopilotPub?.Publish(new Twist());
        }

        return new Hold(this);
    }

    private sealed class Hold(GalleryRuntime runtime) : IDisposable
    {
        private int _released;

        public void Dispose()
        {
            if (Interlocked.Exchange(ref _released, 1) == 0)
            {
                Interlocked.Decrement(ref runtime._holds);
            }
        }
    }

    private void StartDemoServers()
    {
        var talker = context.CreateNode("talker");
        var chatter = talker.CreatePublisher<StdMsgs.String>("/chatter");
        var count = 0;
        talker.CreateTimer(TimeSpan.FromSeconds(0.5), () => chatter.Publish(new StdMsgs.String { Data = $"Hello ROS2.Net #{++count}" }));

        var adder = context.CreateNode("add_two_ints_server");
        adder.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("/add_two_ints", r =>
        {
            adder.Logger.LogInformation("add_two_ints: {A} + {B}", r.A, r.B);
            return new AddTwoIntsResponse { Sum = r.A + r.B };
        });

        var fib = context.CreateNode("fibonacci_server");
        fib.CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("/fibonacci", async (goal, ct) =>
        {
            var seq = new List<int> { 0, 1 };
            for (var i = 1; i < Math.Clamp(goal.Goal.Order, 1, 40); i++)
            {
                ct.ThrowIfCancellationRequested();
                seq.Add(seq[i] + seq[i - 1]);
                goal.PublishFeedback(new FibonacciFeedback { Sequence = seq.ToArray() });
                await Task.Delay(250, ct);
            }

            return new FibonacciResult { Sequence = seq.ToArray() };
        });

        var controller = context.CreateNode("controller", "/gallery");
        controller.DeclareParameter("max_speed", 0.5, new ParameterDescriptor
        {
            Description = "Linear speed limit [m/s]",
            FloatingPointRange = new FloatingPointRange(0, 2),
        });
        controller.DeclareParameter("robot_name", "waffle-01", "Fleet identifier", readOnly: true);
        controller.DeclareParameter("use_lidar", true, "Enable obstacle stop from /scan");
        controller.DeclareParameter("control_rate_hz", 50L, new ParameterDescriptor
        {
            Description = "Control loop frequency",
            IntegerRange = new IntegerRange(1, 500),
        });
        controller.DeclareParameter("waypoints", new[] { "dock", "aisle_3", "charger" }, "Patrol route");

        _owned.Add(new DemoLifecycleTalker(context));
    }

    private void StartAutopilot()
    {
        var node = context.CreateNode("autopilot");
        _autopilotPub = node.CreatePublisher<Twist>("/cmd_vel");
        var scan = node.CreateSubscription<LaserScan>("/scan", QosProfile.SensorData);
        _ = Task.Run(async () =>
        {
            await foreach (var s in scan.ReadAllAsync(_cts.Token))
            {
                _lastAngleMin = s.AngleMin;
                _lastAngleIncrement = s.AngleIncrement;
                _lastRanges = s.Ranges;
            }
        });
        node.CreateTimer(TimeSpan.FromMilliseconds(100), () =>
        {
            if (!AutopilotActive)
            {
                return;
            }

            _autopilotPub.Publish(Wander(_lastRanges, _lastAngleMin, _lastAngleIncrement));
        });
    }

    /// <summary>A reactive wander: drive forward, steer toward open space, turn away from close obstacles.</summary>
    internal static Twist Wander(float[] ranges, double angleMin, double increment)
    {
        if (ranges.Length == 0)
        {
            return new Twist();
        }

        double Sector(double from, double to)
        {
            var min = double.PositiveInfinity;
            for (var i = 0; i < ranges.Length; i++)
            {
                var a = ROS2.Net.Simulation.Angles.Normalize(angleMin + (i * increment));
                if (a >= from && a <= to && float.IsFinite(ranges[i]) && ranges[i] > 0.05f)
                {
                    min = Math.Min(min, ranges[i]);
                }
            }

            return double.IsInfinity(min) ? 8 : min;
        }

        var front = Sector(-0.45, 0.45);
        var left = Sector(0.45, 1.4);
        var right = Sector(-1.4, -0.45);
        if (front < 0.65)
        {
            return MessageExtensions.Twist2D(0.0, left > right ? 1.4 : -1.4);
        }

        var steer = Math.Clamp((left - right) * 0.35, -0.8, 0.8);
        var speed = Math.Clamp((front - 0.5) * 0.4, 0.12, 0.45);
        return MessageExtensions.Twist2D(speed, steer);
    }

    public Task StopAsync(CancellationToken cancellationToken)
    {
        _cts.Cancel();
        return Simulation?.StopAsync() ?? Task.CompletedTask;
    }

    public void Dispose()
    {
        foreach (var d in _owned)
        {
            d.Dispose();
        }

        _cts.Dispose();
    }
}

/// <summary>The lifecycle demo node driven from the Lifecycle page.</summary>
internal sealed class DemoLifecycleTalker : LifecycleNode
{
    private LifecyclePublisher<StdMsgs.String>? _pub;
    private RosTimer? _timer;
    private int _count;

    public DemoLifecycleTalker(Ros2Context context)
        : base(context, "lc_talker", new NodeOptions { Namespace = "/gallery" })
    {
    }

    protected override ValueTask<CallbackReturn> OnConfigure(LifecycleState previous)
    {
        _pub = CreateLifecyclePublisher<StdMsgs.String>("lifecycle_chatter");
        _timer = CreateTimer(TimeSpan.FromMilliseconds(500), () => _pub.Publish(new StdMsgs.String { Data = $"Lifecycle hello #{++_count}" }));
        return ValueTask.FromResult(CallbackReturn.Success);
    }

    protected override ValueTask<CallbackReturn> OnCleanup(LifecycleState previous)
    {
        _timer?.Dispose();
        _pub?.Dispose();
        return ValueTask.FromResult(CallbackReturn.Success);
    }

    protected override ValueTask<CallbackReturn> OnShutdown(LifecycleState previous) => OnCleanup(previous);
}
