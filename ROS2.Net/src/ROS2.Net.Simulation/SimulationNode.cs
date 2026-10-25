using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Msgs.RosgraphMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.StdSrvs;
using ROS2.Net.Msgs.Tf2Msgs;
using ROS2.Net.Msgs.Turtlesim;
using GeoPose = ROS2.Net.Msgs.GeometryMsgs.Pose;

namespace ROS2.Net.Simulation;

/// <summary>Options of <see cref="SimulationNode"/>.</summary>
public sealed class SimulationNodeOptions
{
    /// <summary>Step the world in real time from <see cref="SimulationNode.RunAsync"/>. Disable to step the world yourself.</summary>
    public bool RunWorld { get; set; } = true;

    /// <summary>Publish <c>/clock</c> so nodes with <c>use_sim_time</c> follow simulation time.</summary>
    public bool PublishClock { get; set; } = true;

    public double ClockRateHz { get; set; } = 100;

    public double OdomRateHz { get; set; } = 50;

    public double JointStateRateHz { get; set; } = 30;

    public double GroundTruthRateHz { get; set; } = 30;

    /// <summary>Publish odom→base_link (and map→odom when <see cref="PublishMapToOdom"/>) on <c>/tf</c>, sensor mounts on <c>/tf_static</c>.</summary>
    public bool PublishTf { get; set; } = true;

    /// <summary>
    /// Publish a perfect-localization map→odom transform (ground truth), so map-frame tools work
    /// without a localization node. Leave off when running AMCL/SLAM.
    /// </summary>
    public bool PublishMapToOdom { get; set; }

    /// <summary>Publish the ground-truth occupancy grid on <c>/map</c> (latched).</summary>
    public bool PublishMap { get; set; } = true;

    public double MapResolution { get; set; } = 0.05;

    public string MapFrame { get; set; } = "map";
}

/// <summary>
/// Exposes a <see cref="SimWorld"/> over ROS 2. Per robot (namespace <c>/robotN</c>, or <c>/</c>):
/// subscribes <c>cmd_vel</c>; publishes <c>odom</c>, <c>scan</c>, <c>imu</c>, <c>joint_states</c>,
/// <c>battery_state</c>, <c>gps/fix</c>, sonar ranges, <c>camera/image_raw</c> and
/// <c>ground_truth/pose</c>. Globally: <c>/tf</c>, <c>/tf_static</c>, <c>/clock</c>, <c>/map</c>, and
/// the services <c>/sim/reset</c>, <c>/sim/pause</c>, <c>/sim/spawn</c>, <c>/sim/kill</c> and
/// <c>&lt;ns&gt;/teleport</c>.
/// </summary>
/// <remarks>
/// Sensor data is published from the stepping thread. <c>cmd_vel</c> is read without the executor,
/// so driving works even when the node is not spun; spin the node for the services.
/// </remarks>
public sealed class SimulationNode : Node
{
    private readonly Dictionary<string, RobotBridge> _bridges = new(StringComparer.Ordinal);
    private readonly object _bridgeGate = new();
    private readonly Publisher<Clock> _clock;
    private readonly Publisher<TFMessage> _tf;
    private readonly Publisher<TFMessage> _tfStatic;
    private readonly Publisher<OccupancyGrid> _map;
    private double _clockAccumulator;
    private CancellationTokenSource? _runCts;
    private Task? _runTask;

    public SimulationNode(Ros2Context context, SimWorld world, SimulationNodeOptions? options = null, string name = "simulator", NodeOptions? nodeOptions = null)
        : base(context, name, nodeOptions)
    {
        World = world;
        SimOptions = options ?? new SimulationNodeOptions();
        _clock = CreatePublisher<Clock>("/clock", QosProfile.Default);
        _tf = CreatePublisher<TFMessage>("/tf", QosProfile.Default.KeepLast(100));
        _tfStatic = CreatePublisher<TFMessage>("/tf_static", QosProfile.Latched);
        _map = CreatePublisher<OccupancyGrid>("/map", QosProfile.Latched);

        foreach (var robot in world.Robots)
        {
            AddBridge(robot);
        }

        world.RobotsChanged += OnRobotsChanged;
        world.Stepped += OnStepped;
        world.WasReset += OnReset;

        CreateService<EmptyRequest, EmptyResponse>("/sim/reset", _ =>
        {
            World.Reset();
            return new EmptyResponse();
        });
        CreateService<SetBoolRequest, SetBoolResponse>("/sim/pause", req =>
        {
            World.Paused = req.Data;
            return new SetBoolResponse { Success = true, Message = req.Data ? "paused" : "running" };
        });
        CreateService<SpawnRequest, SpawnResponse>("/sim/spawn", req =>
        {
            var robotName = string.IsNullOrWhiteSpace(req.Name) ? NextRobotName() : req.Name;
            if (World.GetRobot(robotName) is not null)
            {
                Logger.LogWarning("A robot named {Name} already exists", robotName);
                return new SpawnResponse { Name = string.Empty };
            }

            World.AddRobot(RobotFactory.Create(RobotKind.DiffDrive, robotName, new Pose2D(req.X, req.Y, req.Theta)));
            return new SpawnResponse { Name = robotName };
        });
        CreateService<KillRequest, KillResponse>("/sim/kill", req =>
        {
            if (!World.RemoveRobot(req.Name))
            {
                Logger.LogWarning("No robot named {Name}", req.Name);
            }

            return new KillResponse();
        });

        if (SimOptions.PublishMap)
        {
            PublishMap();
        }
    }

    public SimWorld World { get; }

    public SimulationNodeOptions SimOptions { get; }

    /// <summary>Current simulation time as a ROS time.</summary>
    public RosTime SimTime => new((long)Math.Round(World.Time * 1e9));

    /// <summary>Publishes the ground-truth map on <c>/map</c>.</summary>
    public void PublishMap() => _map.Publish(World.ToOccupancyGrid(SimOptions.MapResolution, SimOptions.MapFrame));

    /// <summary>Starts stepping the world in the background (if <see cref="SimulationNodeOptions.RunWorld"/>).</summary>
    public void Start()
    {
        if (_runTask is not null || !SimOptions.RunWorld)
        {
            return;
        }

        _runCts = CancellationTokenSource.CreateLinkedTokenSource(Context.ShutdownToken);
        _runTask = Task.Run(() => World.RunAsync(_runCts.Token));
    }

    /// <summary>Stops background stepping.</summary>
    public async Task StopAsync()
    {
        if (_runCts is null || _runTask is null)
        {
            return;
        }

        _runCts.Cancel();
        try
        {
            await _runTask.ConfigureAwait(false);
        }
        catch (OperationCanceledException)
        {
        }

        _runCts.Dispose();
        _runCts = null;
        _runTask = null;
    }

    /// <summary>Steps the world (if enabled) and spins the node until cancelled.</summary>
    public async Task RunAsync(CancellationToken cancellationToken = default)
    {
        Start();
        try
        {
            await SpinAsync(cancellationToken).ConfigureAwait(false);
        }
        finally
        {
            await StopAsync().ConfigureAwait(false);
        }
    }

    private string NextRobotName()
    {
        for (var i = 1; ; i++)
        {
            if (World.GetRobot($"robot{i}") is null)
            {
                return $"robot{i}";
            }
        }
    }

    private void OnRobotsChanged(SimRobot robot, bool added)
    {
        if (added)
        {
            AddBridge(robot);
        }
        else
        {
            RobotBridge? bridge;
            lock (_bridgeGate)
            {
                _bridges.Remove(robot.Name, out bridge);
            }

            bridge?.Dispose();
        }

        PublishStaticTransforms();
    }

    private void AddBridge(SimRobot robot)
    {
        var bridge = new RobotBridge(this, robot);
        lock (_bridgeGate)
        {
            _bridges[robot.Name] = bridge;
        }

        PublishStaticTransforms();
    }

    private void OnReset(SimWorld world)
    {
        if (SimOptions.PublishMap)
        {
            PublishMap();
        }

        RobotBridge[] bridges;
        lock (_bridgeGate)
        {
            bridges = [.. _bridges.Values];
        }

        foreach (var b in bridges)
        {
            b.OnReset();
        }
    }

    private void OnStepped(SimWorld world)
    {
        if (IsDisposed)
        {
            return;
        }

        var stamp = SimTime;
        if (SimOptions.PublishClock)
        {
            _clockAccumulator += world.StepSize;
            if (_clockAccumulator + 1e-9 >= 1.0 / SimOptions.ClockRateHz)
            {
                _clockAccumulator = 0;
                _clock.Publish(new Clock { ClockValue = stamp.ToMsg() });
            }
        }

        RobotBridge[] bridges;
        lock (_bridgeGate)
        {
            bridges = [.. _bridges.Values];
        }

        List<TransformStamped>? transforms = null;
        foreach (var b in bridges)
        {
            b.OnStep(world, stamp, ref transforms);
        }

        if (transforms is { Count: > 0 } && SimOptions.PublishTf)
        {
            _tf.Publish(new TFMessage { Transforms = [.. transforms] });
        }
    }

    /// <summary>Publishes every robot's sensor mounts on <c>/tf_static</c> as one latched message.</summary>
    public void PublishStaticTransforms()
    {
        if (!SimOptions.PublishTf)
        {
            return;
        }

        var list = new List<TransformStamped>();
        foreach (var r in World.Robots)
        {
            var baseFrame = r.FramePrefix + "base_link";
            list.Add(MakeTransform(SimTime, r.FramePrefix + "base_footprint", baseFrame, new Pose2D(0, 0, 0), 0.01));
            foreach (var s in r.Sensors)
            {
                list.Add(MakeTransform(SimTime, baseFrame, r.FramePrefix + s.FrameSuffix, s.Mount, s.MountHeight));
            }
        }

        _tfStatic.Publish(new TFMessage { Transforms = [.. list] });
    }

    internal static TransformStamped MakeTransform(RosTime stamp, string parent, string child, Pose2D pose, double z = 0)
    {
        var t = new TransformStamped { ChildFrameId = child };
        t.Header.FrameId = parent;
        stamp.CopyTo(t.Header.Stamp);
        t.Transform.Translation.X = pose.X;
        t.Transform.Translation.Y = pose.Y;
        t.Transform.Translation.Z = z;
        t.Transform.Rotation = MessageExtensions.QuaternionFromYaw(pose.Theta);
        return t;
    }

    internal static GeoPose ToPose(Pose2D p) => new()
    {
        Position = new Point { X = p.X, Y = p.Y },
        Orientation = MessageExtensions.QuaternionFromYaw(p.Theta),
    };

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            World.RobotsChanged -= OnRobotsChanged;
            World.Stepped -= OnStepped;
            World.WasReset -= OnReset;
            _runCts?.Cancel();
            lock (_bridgeGate)
            {
                foreach (var b in _bridges.Values)
                {
                    b.Dispose();
                }

                _bridges.Clear();
            }
        }

        base.Dispose(disposing);
    }

    /// <summary>Topics and services of one robot.</summary>
    private sealed class RobotBridge : IDisposable
    {
        private readonly SimulationNode _node;
        private readonly SimRobot _robot;
        private readonly Subscription<Twist> _cmd;
        private readonly Publisher<Odometry> _odom;
        private readonly Publisher<JointState> _joints;
        private readonly Publisher<PoseStamped> _groundTruth;
        private readonly Publisher<LaserScan>? _scan;
        private readonly Publisher<Imu>? _imu;
        private readonly Publisher<NavSatFix>? _gps;
        private readonly Publisher<BatteryState>? _battery;
        private readonly Publisher<Image>? _camera;
        private readonly List<(RangeSensor Sensor, Publisher<Msgs.SensorMsgs.Range> Publisher, long Last)> _ranges = [];
        private readonly ServiceServer<TeleportAbsoluteRequest, TeleportAbsoluteResponse> _teleport;
        private long _lastScan, _lastImu, _lastGps, _lastBattery, _lastCamera;
        private double _odomAcc, _jointAcc, _gtAcc;

        public RobotBridge(SimulationNode node, SimRobot robot)
        {
            _node = node;
            _robot = robot;
            var ns = robot.Namespace == "/" ? string.Empty : robot.Namespace.TrimEnd('/');
            string T(string topic) => ns + "/" + topic;

            _cmd = node.CreateSubscription<Twist>(T("cmd_vel"), QosProfile.Default);
            _odom = node.CreatePublisher<Odometry>(T("odom"));
            _joints = node.CreatePublisher<JointState>(T("joint_states"));
            _groundTruth = node.CreatePublisher<PoseStamped>(T("ground_truth/pose"));
            if (robot.Lidar is not null)
            {
                _scan = node.CreatePublisher<LaserScan>(T("scan"), QosProfile.SensorData);
            }

            if (robot.Imu is not null)
            {
                _imu = node.CreatePublisher<Imu>(T("imu"), QosProfile.SensorData);
            }

            if (robot.Gps is not null)
            {
                _gps = node.CreatePublisher<NavSatFix>(T("gps/fix"), QosProfile.SensorData);
            }

            if (robot.Battery is not null)
            {
                _battery = node.CreatePublisher<BatteryState>(T("battery_state"));
            }

            if (robot.Camera is not null)
            {
                _camera = node.CreatePublisher<Image>(T("camera/image_raw"), QosProfile.SensorData);
            }

            foreach (var r in robot.RangeSensors)
            {
                _ranges.Add((r, node.CreatePublisher<Msgs.SensorMsgs.Range>(T(r.Name), QosProfile.SensorData), 0));
            }

            _teleport = node.CreateService<TeleportAbsoluteRequest, TeleportAbsoluteResponse>(T("teleport"), req =>
            {
                lock (node.World.SyncRoot)
                {
                    robot.Teleport(new Pose2D(req.X, req.Y, req.Theta));
                }

                return new TeleportAbsoluteResponse();
            });
        }

        private string Frame(string suffix) => _robot.FramePrefix + suffix;

        public void OnReset()
        {
            _lastScan = _lastImu = _lastGps = _lastBattery = _lastCamera = 0;
            for (var i = 0; i < _ranges.Count; i++)
            {
                _ranges[i] = _ranges[i] with { Last = 0 };
            }
        }

        public void OnStep(SimWorld world, RosTime stamp, ref List<TransformStamped>? transforms)
        {
            // Commands: newest wins; drained without the executor.
            Twist? latest = null;
            while (_cmd.TryTake(out var t))
            {
                latest = t;
            }

            if (latest is not null)
            {
                _robot.SetCommand(latest.Linear.X, latest.Linear.Y, latest.Angular.Z, world.Time);
            }

            var dt = world.StepSize;
            var opts = _node.SimOptions;
            if (Due(ref _odomAcc, dt, opts.OdomRateHz))
            {
                PublishOdom(stamp, ref transforms);
            }

            if (Due(ref _jointAcc, dt, opts.JointStateRateHz))
            {
                PublishJoints(stamp);
            }

            if (Due(ref _gtAcc, dt, opts.GroundTruthRateHz))
            {
                var gt = new PoseStamped { Pose = ToPose(_robot.Pose) };
                gt.Header.FrameId = opts.MapFrame;
                stamp.CopyTo(gt.Header.Stamp);
                _groundTruth.Publish(gt);
            }

            if (_robot.Lidar is { } lidar && _scan is not null && lidar.Sequence != _lastScan)
            {
                _lastScan = lidar.Sequence;
                _scan.Publish(MakeScan(lidar, stamp));
            }

            if (_robot.Imu is { } imu && _imu is not null && imu.Sequence != _lastImu)
            {
                _lastImu = imu.Sequence;
                _imu.Publish(MakeImu(imu, stamp));
            }

            if (_robot.Gps is { } gps && _gps is not null && gps.Sequence != _lastGps)
            {
                _lastGps = gps.Sequence;
                _gps.Publish(MakeFix(gps, stamp));
            }

            if (_robot.Battery is { } bat && _battery is not null && bat.Sequence != _lastBattery)
            {
                _lastBattery = bat.Sequence;
                _battery.Publish(MakeBattery(bat, stamp));
            }

            if (_robot.Camera is { } cam && _camera is not null && cam.Sequence != _lastCamera)
            {
                _lastCamera = cam.Sequence;
                var img = new Image
                {
                    Width = (uint)cam.Width,
                    Height = (uint)cam.Height,
                    Encoding = "rgb8",
                    Step = (uint)(cam.Width * 3),
                    Data = (byte[])cam.Pixels.Clone(),
                };
                img.Header.FrameId = Frame(cam.FrameSuffix);
                stamp.CopyTo(img.Header.Stamp);
                _camera.Publish(img);
            }

            for (var i = 0; i < _ranges.Count; i++)
            {
                var (sensor, pub, last) = _ranges[i];
                if (sensor.Sequence == last)
                {
                    continue;
                }

                _ranges[i] = (sensor, pub, sensor.Sequence);
                var msg = new Msgs.SensorMsgs.Range
                {
                    RadiationType = Msgs.SensorMsgs.Range.ULTRASOUND,
                    FieldOfView = (float)sensor.FieldOfView,
                    MinRange = (float)sensor.MinRange,
                    MaxRange = (float)sensor.MaxRange,
                    RangeValue = (float)sensor.Range,
                };
                msg.Header.FrameId = Frame(sensor.FrameSuffix);
                stamp.CopyTo(msg.Header.Stamp);
                pub.Publish(msg);
            }
        }

        private static bool Due(ref double accumulator, double dt, double rate)
        {
            if (rate <= 0)
            {
                return false;
            }

            accumulator += dt;
            if (accumulator + 1e-9 >= 1.0 / rate)
            {
                accumulator = 0;
                return true;
            }

            return false;
        }

        private void PublishOdom(RosTime stamp, ref List<TransformStamped>? transforms)
        {
            var r = _robot;
            var odom = new Odometry { ChildFrameId = Frame("base_link") };
            odom.Header.FrameId = Frame("odom");
            stamp.CopyTo(odom.Header.Stamp);
            odom.Pose.Pose = ToPose(r.OdomPose);
            odom.Twist.Twist.Linear.X = r.OdomLinearVelocity;
            odom.Twist.Twist.Linear.Y = r.OdomLateralVelocity;
            odom.Twist.Twist.Angular.Z = r.OdomAngularVelocity;
            foreach (var i in new[] { 0, 7, 35 })
            {
                odom.Pose.Covariance[i] = 1e-3;
                odom.Twist.Covariance[i] = 1e-3;
            }

            foreach (var i in new[] { 14, 21, 28 })
            {
                odom.Pose.Covariance[i] = 1e6;
                odom.Twist.Covariance[i] = 1e6;
            }

            _odom.Publish(odom);

            transforms ??= [];
            transforms.Add(MakeTransform(stamp, Frame("odom"), Frame("base_link"), r.OdomPose));
            if (_node.SimOptions.PublishMapToOdom)
            {
                // map→odom = truth ∘ odom⁻¹, so map→base_link equals the ground truth.
                var odomInv = Inverse(r.OdomPose);
                transforms.Add(MakeTransform(stamp, _node.SimOptions.MapFrame, Frame("odom"), r.Pose.Compose(odomInv)));
            }
        }

        private static Pose2D Inverse(Pose2D p)
        {
            var t = new Vec2(-p.X, -p.Y).Rotate(-p.Theta);
            return new Pose2D(t.X, t.Y, Angles.Normalize(-p.Theta));
        }

        private void PublishJoints(RosTime stamp)
        {
            var r = _robot;
            var names = new List<string> { "wheel_left_joint", "wheel_right_joint" };
            var positions = new List<double> { r.LeftWheelPosition, r.RightWheelPosition };
            if (r.Kind == RobotKind.Ackermann)
            {
                names.Add("steering_joint");
                positions.Add(r.SteeringAngle);
            }

            if (r.Arm is { } arm)
            {
                names.AddRange(arm.JointNames);
                positions.AddRange(arm.Positions);
            }

            var js = new JointState { Name = [.. names], Position = [.. positions] };
            stamp.CopyTo(js.Header.Stamp);
            _joints.Publish(js);
        }

        private LaserScan MakeScan(LidarSensor lidar, RosTime stamp)
        {
            var inc = lidar.AngleIncrement;
            var scan = new LaserScan
            {
                AngleMin = (float)lidar.AngleMin,
                AngleMax = (float)(lidar.AngleMin + (inc * (lidar.Beams - 1))),
                AngleIncrement = (float)inc,
                TimeIncrement = 0,
                ScanTime = (float)(1.0 / lidar.RateHz),
                RangeMin = (float)lidar.RangeMin,
                RangeMax = (float)lidar.RangeMax,
                Ranges = (float[])lidar.Ranges.Clone(),
            };
            scan.Header.FrameId = Frame(lidar.FrameSuffix);
            stamp.CopyTo(scan.Header.Stamp);
            return scan;
        }

        private Imu MakeImu(ImuSensor imu, RosTime stamp)
        {
            var m = new Imu { Orientation = MessageExtensions.QuaternionFromYaw(imu.Yaw) };
            m.Header.FrameId = Frame(imu.FrameSuffix);
            stamp.CopyTo(m.Header.Stamp);
            m.AngularVelocity.Z = imu.AngularVelocityZ;
            var (ax, ay, az) = imu.LinearAcceleration;
            m.LinearAcceleration.X = ax;
            m.LinearAcceleration.Y = ay;
            m.LinearAcceleration.Z = az;
            var o2 = imu.OrientationNoiseStdDev * imu.OrientationNoiseStdDev;
            var g2 = imu.GyroNoiseStdDev * imu.GyroNoiseStdDev;
            var a2 = imu.AccelNoiseStdDev * imu.AccelNoiseStdDev;
            m.OrientationCovariance[0] = m.OrientationCovariance[4] = m.OrientationCovariance[8] = o2;
            m.AngularVelocityCovariance[0] = m.AngularVelocityCovariance[4] = m.AngularVelocityCovariance[8] = g2;
            m.LinearAccelerationCovariance[0] = m.LinearAccelerationCovariance[4] = m.LinearAccelerationCovariance[8] = a2;
            return m;
        }

        private NavSatFix MakeFix(GpsSensor gps, RosTime stamp)
        {
            var fix = new NavSatFix
            {
                Latitude = gps.Latitude,
                Longitude = gps.Longitude,
                Altitude = gps.Altitude,
                PositionCovarianceType = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
            };
            fix.Status.Status = NavSatStatus.STATUS_FIX;
            fix.Status.Service = NavSatStatus.SERVICE_GPS;
            var v = gps.NoiseStdDev * gps.NoiseStdDev;
            fix.PositionCovariance[0] = fix.PositionCovariance[4] = v;
            fix.PositionCovariance[8] = v * 2.25;
            fix.Header.FrameId = Frame(gps.FrameSuffix);
            stamp.CopyTo(fix.Header.Stamp);
            return fix;
        }

        private BatteryState MakeBattery(BatterySensor bat, RosTime stamp)
        {
            var b = new BatteryState
            {
                Voltage = (float)bat.Voltage,
                Temperature = 25f,
                Current = (float)bat.Current,
                Charge = (float)bat.ChargeAh,
                Capacity = (float)bat.CapacityAh,
                DesignCapacity = (float)bat.CapacityAh,
                Percentage = (float)bat.Percentage,
                PowerSupplyStatus = bat.IsCharging
                    ? (bat.Percentage >= 0.999 ? BatteryState.POWER_SUPPLY_STATUS_FULL : BatteryState.POWER_SUPPLY_STATUS_CHARGING)
                    : BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
                PowerSupplyHealth = BatteryState.POWER_SUPPLY_HEALTH_GOOD,
                PowerSupplyTechnology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
                Present = true,
                Location = _robot.Name,
                SerialNumber = "SIM-" + _robot.Name,
            };
            b.Header.FrameId = Frame(bat.FrameSuffix);
            stamp.CopyTo(b.Header.Stamp);
            return b;
        }

        public void Dispose()
        {
            _cmd.Dispose();
            _odom.Dispose();
            _joints.Dispose();
            _groundTruth.Dispose();
            _scan?.Dispose();
            _imu?.Dispose();
            _gps?.Dispose();
            _battery?.Dispose();
            _camera?.Dispose();
            foreach (var (_, p, _) in _ranges)
            {
                p.Dispose();
            }

            _teleport.Dispose();
        }
    }
}
