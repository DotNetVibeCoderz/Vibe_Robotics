using System.Diagnostics;
using System.Runtime.InteropServices;
using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs.ControllerManagerMsgs;

namespace ROS2.Net.Control;

/// <summary>Strictness of a controller switch (<c>controller_manager_msgs/srv/SwitchController</c>).</summary>
public enum SwitchStrictness
{
    /// <summary>Do what is possible, skip the rest.</summary>
    BestEffort = 1,
    /// <summary>All requested changes succeed or nothing changes.</summary>
    Strict = 2,
}

/// <summary>
/// The ros2_control controller manager: loads controllers, claims interfaces for them and runs the
/// read → update → write loop on a dedicated high-priority thread. Exposes the standard
/// <c>~/list_controllers</c>, <c>~/list_hardware_interfaces</c>, <c>~/switch_controller</c>,
/// <c>~/load_controller</c>, <c>~/configure_controller</c> and <c>~/unload_controller</c> services, so
/// <c>ros2 control</c> works against it.
/// </summary>
public sealed partial class ControllerManager : Node
{
    private static readonly Dictionary<string, Func<ControllerBase>> s_builtinTypes = new(StringComparer.Ordinal)
    {
        ["joint_state_broadcaster/JointStateBroadcaster"] = () => new JointStateBroadcaster(),
        ["forward_command_controller/ForwardCommandController"] = () => new ForwardCommandController(),
        ["position_controllers/JointGroupPositionController"] = () => new ForwardCommandController(HardwareInterfaceNames.Position),
        ["velocity_controllers/JointGroupVelocityController"] = () => new ForwardCommandController(HardwareInterfaceNames.Velocity),
        ["effort_controllers/JointGroupEffortController"] = () => new ForwardCommandController(HardwareInterfaceNames.Effort),
        ["diff_drive_controller/DiffDriveController"] = () => new DiffDriveController(),
        ["ackermann_steering_controller/AckermannSteeringController"] = () => new AckermannSteeringController(),
        ["joint_trajectory_controller/JointTrajectoryController"] = () => new JointTrajectoryController(),
        ["position_controllers/GripperActionController"] = () => new GripperActionController(),
    };

    private readonly object _gate = new();
    private readonly Dictionary<string, Func<ControllerBase>> _types = new(s_builtinTypes, StringComparer.Ordinal);
    private readonly List<ControllerBase> _controllers = [];
    private ControllerBase[] _active = [];
    private Thread? _thread;
    private volatile bool _running;
    private long _cycles;
    private long _overruns;

    public ControllerManager(Ros2Context context, ResourceManager resources, string name = "controller_manager", NodeOptions? options = null)
        : base(context, name, options)
    {
        Resources = resources;
        UpdateRate = (int)DeclareParameter("update_rate", 100L);

        CreateService<ListControllersRequest, ListControllersResponse>("~/list_controllers", _ => new ListControllersResponse { Controller = DescribeControllers() });
        CreateService<ListHardwareInterfacesRequest, ListHardwareInterfacesResponse>("~/list_hardware_interfaces", _ => DescribeInterfaces());
        CreateService<LoadControllerRequest, LoadControllerResponse>("~/load_controller", req => new LoadControllerResponse { Ok = TryRun(() => LoadController(req.Name)) });
        CreateService<ConfigureControllerRequest, ConfigureControllerResponse>("~/configure_controller", req => new ConfigureControllerResponse { Ok = ConfigureController(req.Name) });
        CreateService<UnloadControllerRequest, UnloadControllerResponse>("~/unload_controller", req => new UnloadControllerResponse { Ok = UnloadController(req.Name) });
        CreateService<SwitchControllerRequest, SwitchControllerResponse>("~/switch_controller", req => new SwitchControllerResponse
        {
            Ok = SwitchController(
                [.. req.ActivateControllers, .. req.StartControllers],
                [.. req.DeactivateControllers, .. req.StopControllers],
                req.Strictness == SwitchControllerRequest.BEST_EFFORT ? SwitchStrictness.BestEffort : SwitchStrictness.Strict),
        });
    }

    public ResourceManager Resources { get; }

    /// <summary>Loop frequency in Hz (parameter <c>update_rate</c>).</summary>
    public int UpdateRate { get; }

    public IReadOnlyList<ControllerBase> Controllers
    {
        get
        {
            lock (_gate)
            {
                return _controllers.ToList();
            }
        }
    }

    /// <summary>Completed control cycles.</summary>
    public long CycleCount => Interlocked.Read(ref _cycles);

    /// <summary>Cycles that started later than one period after their deadline.</summary>
    public long OverrunCount => Interlocked.Read(ref _overruns);

    public bool IsRunning => _running;

    /// <summary>Registers a controller type under a plugin name, e.g. <c>my_pkg/MyController</c>.</summary>
    public void RegisterControllerType(string typeName, Func<ControllerBase> factory) => _types[typeName] = factory;

    public IReadOnlyCollection<string> ControllerTypes => _types.Keys;

    public ControllerBase? GetController(string name)
    {
        lock (_gate)
        {
            return _controllers.Find(c => c.Name == name);
        }
    }

    /// <summary>Loads a controller whose type comes from the <c>&lt;name&gt;.type</c> parameter (or <paramref name="typeName"/>).</summary>
    public ControllerBase LoadController(string name, string? typeName = null)
    {
        var key = name + ".type";
        if (typeName is null)
        {
            if (!HasParameter(key) && ParameterOverrides.ContainsKey(key))
            {
                DeclareParameter(key, ParameterValue.From(string.Empty));
            }

            typeName = HasParameter(key) ? GetParameter<string>(key) : throw new InvalidOperationException($"Parameter '{key}' is not set; cannot load controller '{name}'.");
        }

        if (!_types.TryGetValue(typeName, out var factory))
        {
            throw new InvalidOperationException($"Unknown controller type '{typeName}'.");
        }

        var controller = factory();
        controller.TypeName = typeName;
        return LoadController(controller, name);
    }

    /// <summary>Loads a controller instance.</summary>
    public T LoadController<T>(T controller, string name, IEnumerable<Parameter>? parameters = null)
        where T : ControllerBase
    {
        lock (_gate)
        {
            if (_controllers.Exists(c => c.Name == name))
            {
                throw new InvalidOperationException($"A controller named '{name}' is already loaded.");
            }
        }

        // Parameters written as "<controller>.<param>" on the manager are forwarded to the controller's node.
        var options = new NodeOptions { Namespace = Namespace };
        var prefix = name + ".";
        foreach (var (k, v) in ParameterOverrides)
        {
            if (k.StartsWith(prefix, StringComparison.Ordinal) && k != prefix + "type")
            {
                options.ParameterOverrides.Add(new Parameter(k[prefix.Length..], v));
            }
        }

        if (parameters is not null)
        {
            options.ParameterOverrides.AddRange(parameters);
        }

        if (string.IsNullOrEmpty(controller.TypeName))
        {
            controller.TypeName = controller.GetType().FullName!;
        }

        var node = Context.CreateNode(name, Namespace, options);
        controller.Initialize(node, name);
        lock (_gate)
        {
            _controllers.Add(controller);
        }

        Logger.LogInformation("Loaded controller '{Name}' ({Type})", name, controller.TypeName);
        return controller;
    }

    public bool ConfigureController(string name)
    {
        var c = GetController(name);
        if (c is null || c.Lifecycle is not (ControllerLifecycle.Unconfigured or ControllerLifecycle.Inactive))
        {
            return false;
        }

        if (c.Lifecycle == ControllerLifecycle.Inactive && c.OnCleanup() != CallbackReturn.Success)
        {
            return false;
        }

        var ok = Safe(c, c.OnConfigure) == CallbackReturn.Success;
        c.Lifecycle = ok ? ControllerLifecycle.Inactive : ControllerLifecycle.Unconfigured;
        return ok;
    }

    /// <summary>Loads, configures and activates in one call (like the <c>spawner</c>).</summary>
    public ControllerBase Spawn(string name, string? typeName = null)
    {
        var c = LoadController(name, typeName);
        if (!ConfigureController(name) || !SwitchController([name], [], SwitchStrictness.Strict))
        {
            throw new InvalidOperationException($"Controller '{name}' could not be started.");
        }

        return c;
    }

    /// <summary>Spawns an already constructed controller instance.</summary>
    public T Spawn<T>(T controller, string name, IEnumerable<Parameter>? parameters = null)
        where T : ControllerBase
    {
        LoadController(controller, name, parameters);
        if (!ConfigureController(name) || !SwitchController([name], [], SwitchStrictness.Strict))
        {
            throw new InvalidOperationException($"Controller '{name}' could not be started.");
        }

        return controller;
    }

    /// <summary>Deactivates and activates controllers atomically between two control cycles.</summary>
    public bool SwitchController(IReadOnlyList<string> activate, IReadOnlyList<string> deactivate, SwitchStrictness strictness = SwitchStrictness.Strict)
    {
        lock (_gate)
        {
            var toStop = new List<ControllerBase>();
            foreach (var n in deactivate.Distinct())
            {
                var c = _controllers.Find(x => x.Name == n);
                if (c is null || c.Lifecycle != ControllerLifecycle.Active)
                {
                    if (strictness == SwitchStrictness.Strict)
                    {
                        Logger.LogWarning("Cannot deactivate '{Name}': not active", n);
                        return false;
                    }

                    continue;
                }

                toStop.Add(c);
            }

            var toStart = new List<ControllerBase>();
            foreach (var n in activate.Distinct())
            {
                var c = _controllers.Find(x => x.Name == n);
                if (c is null || c.Lifecycle != ControllerLifecycle.Inactive)
                {
                    if (strictness == SwitchStrictness.Strict)
                    {
                        Logger.LogWarning("Cannot activate '{Name}': not inactive", n);
                        return false;
                    }

                    continue;
                }

                toStart.Add(c);
            }

            // Strict mode: check every claim up front, accounting for interfaces released by toStop.
            if (strictness == SwitchStrictness.Strict)
            {
                var released = toStop.Select(c => c.Name).ToHashSet(StringComparer.Ordinal);
                var wanted = new HashSet<string>(StringComparer.Ordinal);
                foreach (var c in toStart)
                {
                    foreach (var ci in ResolveCommands(c))
                    {
                        var holder = Resources.ClaimedBy(ci);
                        if (!Resources.HasCommandInterface(ci) || (holder is not null && !released.Contains(holder)) || !wanted.Add(ci))
                        {
                            Logger.LogWarning("Cannot activate '{Name}': command interface '{Interface}' unavailable", c.Name, ci);
                            return false;
                        }
                    }

                    if (ResolveStates(c).Any(s => !Resources.HasStateInterface(s)))
                    {
                        return false;
                    }
                }
            }

            foreach (var c in toStop)
            {
                Safe(c, c.OnDeactivate);
                Resources.Release(c.Name);
                c.Unassign();
                c.Lifecycle = ControllerLifecycle.Inactive;
            }

            var allOk = true;
            foreach (var c in toStart)
            {
                try
                {
                    var commands = Resources.Claim(c.Name, ResolveCommands(c));
                    var states = ResolveStates(c).Select(Resources.GetStateInterface).ToList();
                    c.Assign(commands, states);
                    if (Safe(c, c.OnActivate) != CallbackReturn.Success)
                    {
                        throw new InvalidOperationException("on_activate failed");
                    }

                    c.LastUpdateNs = long.MinValue;
                    c.Lifecycle = ControllerLifecycle.Active;
                }
                catch (Exception ex) when (ex is InterfaceClaimException or InvalidOperationException or KeyNotFoundException)
                {
                    Logger.LogWarning("Activating '{Name}' failed: {Reason}", c.Name, ex.Message);
                    Resources.Release(c.Name);
                    c.Unassign();
                    allOk = false;
                }
            }

            _active = _controllers.Where(c => c.Lifecycle == ControllerLifecycle.Active).ToArray();
            return allOk || strictness == SwitchStrictness.BestEffort;
        }
    }

    public bool UnloadController(string name)
    {
        lock (_gate)
        {
            var c = _controllers.Find(x => x.Name == name);
            if (c is null || c.Lifecycle == ControllerLifecycle.Active)
            {
                return false;
            }

            if (c.Lifecycle == ControllerLifecycle.Inactive)
            {
                Safe(c, c.OnCleanup);
            }

            c.Lifecycle = ControllerLifecycle.Finalized;
            _controllers.Remove(c);
            c.Dispose();
            return true;
        }
    }

    /// <summary>
    /// Runs one cycle: read hardware, update active controllers (each at its own rate), write hardware.
    /// Call it yourself for deterministic simulation, or use <see cref="Start"/>. Allocation-free.
    /// </summary>
    public void Update(RosTime time, RosDuration period)
    {
        lock (_gate)
        {
            Resources.ReadAll(time, period);
            foreach (var c in _active)
            {
                var controllerPeriod = period;
                if (c.UpdateRate > 0 && c.LastUpdateNs != long.MinValue)
                {
                    var elapsed = time.Nanoseconds - c.LastUpdateNs;
                    if (elapsed < (1_000_000_000L / c.UpdateRate) - (period.Nanoseconds / 2))
                    {
                        continue;
                    }

                    controllerPeriod = new RosDuration(elapsed);
                }

                c.LastUpdateNs = time.Nanoseconds;
                c.Update(time, controllerPeriod);
            }

            Resources.WriteAll(time, period);
        }

        Interlocked.Increment(ref _cycles);
    }

    /// <summary>Starts the control loop on a dedicated high-priority thread.</summary>
    public void Start()
    {
        if (_running)
        {
            return;
        }

        _running = true;
        _thread = new Thread(Loop) { IsBackground = true, Priority = ThreadPriority.Highest, Name = $"{Name} update loop" };
        _thread.Start();
    }

    public void Stop()
    {
        _running = false;
        _thread?.Join(TimeSpan.FromSeconds(2));
        _thread = null;
    }

    private void Loop()
    {
        if (OperatingSystem.IsWindows())
        {
            try
            {
                _ = TimeBeginPeriod(1);
            }
            catch (DllNotFoundException)
            {
            }
        }

        var clock = new RosClock(ClockType.SteadyTime);
        var periodTicks = Stopwatch.Frequency / Math.Max(1, UpdateRate);
        var next = Stopwatch.GetTimestamp();
        var last = clock.Now();
        var spinner = default(SpinWait);
        while (_running && !Context.IsShutdown)
        {
            next += periodTicks;
            while (true)
            {
                var remaining = next - Stopwatch.GetTimestamp();
                if (remaining <= 0)
                {
                    break;
                }

                if (remaining > Stopwatch.Frequency / 500)
                {
                    Thread.Sleep(1);
                }
                else
                {
                    spinner.SpinOnce(sleep1Threshold: -1);
                }
            }

            spinner.Reset();
            var lateBy = Stopwatch.GetTimestamp() - next;
            if (lateBy > periodTicks)
            {
                Interlocked.Increment(ref _overruns);
                next = Stopwatch.GetTimestamp(); // Re-anchor instead of bursting.
            }

            var now = clock.Now();
            Update(now, now - last);
            last = now;
        }
    }

    private IEnumerable<string> ResolveCommands(ControllerBase c) => c.CommandInterfaceConfiguration.Type switch
    {
        InterfaceConfigurationType.All => Resources.CommandInterfaces.Select(i => i.Name).Where(n => Resources.ClaimedBy(n) is null || Resources.ClaimedBy(n) == c.Name).ToList(),
        InterfaceConfigurationType.Individual => c.CommandInterfaceConfiguration.Names,
        _ => [],
    };

    private IEnumerable<string> ResolveStates(ControllerBase c) => c.StateInterfaceConfiguration.Type switch
    {
        InterfaceConfigurationType.All => Resources.StateInterfaces.Select(i => i.Name).ToList(),
        InterfaceConfigurationType.Individual => c.StateInterfaceConfiguration.Names,
        _ => [],
    };

    private CallbackReturn Safe(ControllerBase c, Func<CallbackReturn> f)
    {
        try
        {
            return f();
        }
        catch (Exception ex) when (ex is not OutOfMemoryException)
        {
            Logger.LogError(ex, "Controller '{Name}' threw", c.Name);
            return CallbackReturn.Error;
        }
    }

    private bool TryRun(Action a)
    {
        try
        {
            a();
            return true;
        }
        catch (Exception ex) when (ex is InvalidOperationException or ParameterException)
        {
            Logger.LogWarning("{Message}", ex.Message);
            return false;
        }
    }

    private ControllerState[] DescribeControllers()
    {
        lock (_gate)
        {
            return _controllers.Select(c => new ControllerState
            {
                Name = c.Name,
                Type = c.TypeName,
                State = c.Lifecycle.ToString().ToLowerInvariant(),
                ClaimedInterfaces = Resources.CommandInterfaces.Where(i => Resources.ClaimedBy(i.Name) == c.Name).Select(i => i.Name).ToArray(),
                RequiredCommandInterfaces = c.Lifecycle == ControllerLifecycle.Unconfigured ? [] : ResolveCommands(c).ToArray(),
                RequiredStateInterfaces = c.Lifecycle == ControllerLifecycle.Unconfigured ? [] : ResolveStates(c).ToArray(),
            }).ToArray();
        }
    }

    private ListHardwareInterfacesResponse DescribeInterfaces()
    {
        lock (_gate)
        {
            return new ListHardwareInterfacesResponse
            {
                CommandInterfaces = Resources.CommandInterfaces.Select(i => new Msgs.ControllerManagerMsgs.HardwareInterface { Name = i.Name, IsAvailable = true, IsClaimed = Resources.IsClaimed(i.Name) }).ToArray(),
                StateInterfaces = Resources.StateInterfaces.Select(i => new Msgs.ControllerManagerMsgs.HardwareInterface { Name = i.Name, IsAvailable = true }).ToArray(),
            };
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            Stop();
            lock (_gate)
            {
                foreach (var c in _controllers)
                {
                    c.Dispose();
                }

                _controllers.Clear();
                _active = [];
            }
        }

        base.Dispose(disposing);
    }

    [LibraryImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
    private static partial uint TimeBeginPeriod(uint period);
}
