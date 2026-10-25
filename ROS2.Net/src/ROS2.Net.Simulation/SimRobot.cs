namespace ROS2.Net.Simulation;

/// <summary>Drive type of a simulated robot.</summary>
public enum RobotKind
{
    /// <summary>Two driven wheels (TurtleBot-like). Commands: linear.x, angular.z.</summary>
    DiffDrive,
    /// <summary>Car-like steering. Commands: linear.x and angular.z (yaw rate, converted to a steering angle).</summary>
    Ackermann,
    /// <summary>Holonomic base. Commands: linear.x, linear.y, angular.z.</summary>
    Omni,
}

/// <summary>Tunable physical and noise parameters of a robot.</summary>
public sealed class RobotParameters
{
    /// <summary>Collision radius (m).</summary>
    public double Radius { get; set; } = 0.105;

    /// <summary>Footprint length/width for rendering (m).</summary>
    public double Length { get; set; } = 0.14;

    public double Width { get; set; } = 0.18;

    public double Height { get; set; } = 0.19;

    public double MaxLinearSpeed { get; set; } = 0.22;

    public double MaxAngularSpeed { get; set; } = 2.84;

    /// <summary>Linear acceleration limit (m/s²).</summary>
    public double MaxLinearAcceleration { get; set; } = 2.5;

    /// <summary>Angular acceleration limit (rad/s²).</summary>
    public double MaxAngularAcceleration { get; set; } = 10;

    public double WheelRadius { get; set; } = 0.033;

    public double WheelSeparation { get; set; } = 0.160;

    /// <summary>Ackermann wheelbase (m).</summary>
    public double Wheelbase { get; set; } = 0.30;

    /// <summary>Ackermann maximum steering angle (rad).</summary>
    public double MaxSteeringAngle { get; set; } = 0.5;

    /// <summary>Relative standard deviation of wheel slip (true motion vs encoder).</summary>
    public double WheelSlip { get; set; } = 0.01;

    /// <summary>Relative standard deviation of encoder noise in odometry.</summary>
    public double EncoderNoise { get; set; } = 0.002;

    /// <summary>A command older than this is treated as zero (s).</summary>
    public double CommandTimeout { get; set; } = 0.5;

    public SimColor Color { get; set; } = SimColor.Robot;

    public static RobotParameters TurtleBot3Burger() => new();

    public static RobotParameters TurtleBot3Waffle() => new()
    {
        Radius = 0.22, Length = 0.281, Width = 0.306, Height = 0.141,
        MaxLinearSpeed = 0.26, MaxAngularSpeed = 1.82, WheelSeparation = 0.287,
    };

    public static RobotParameters Car() => new()
    {
        Radius = 0.25, Length = 0.5, Width = 0.3, Height = 0.2,
        MaxLinearSpeed = 2.0, MaxAngularSpeed = 2.0, MaxLinearAcceleration = 3, WheelRadius = 0.05,
        WheelSeparation = 0.25, Wheelbase = 0.33, MaxSteeringAngle = 0.5, Color = new SimColor(220, 70, 60),
    };

    public static RobotParameters OmniBase() => new()
    {
        Radius = 0.2, Length = 0.36, Width = 0.36, Height = 0.15,
        MaxLinearSpeed = 0.8, MaxAngularSpeed = 2.0, WheelRadius = 0.05, Color = new SimColor(120, 80, 200),
    };
}

/// <summary>
/// A simulated mobile robot: ground-truth pose, kinematics, collision, odometry and sensors.
/// Subclasses implement the drive kinematics.
/// </summary>
public abstract class SimRobot
{
    private readonly object _commandGate = new();
    private double _cmdVx, _cmdVy, _cmdWz, _cmdTime = double.NegativeInfinity;
    private readonly Vec2[] _trail;
    private int _trailCount, _trailHead;
    private double _trailAccum;
    private readonly List<SimSensor> _sensors = [];

    protected SimRobot(string name, RobotKind kind, Pose2D pose, RobotParameters? parameters)
    {
        Name = name;
        Kind = kind;
        Parameters = parameters ?? new RobotParameters();
        Pose = StartPose = pose;
        OdomPose = new Pose2D(0, 0, 0);
        _trail = new Vec2[600];
        Namespace = "/" + name;
    }

    public string Name { get; }

    /// <summary>ROS namespace, e.g. <c>/robot1</c>. Use <c>/</c> for a single robot without a namespace.</summary>
    public string Namespace { get; set; }

    /// <summary>TF frame prefix derived from the namespace (<c>robot1/</c>, or empty for <c>/</c>).</summary>
    public string FramePrefix => Namespace == "/" ? string.Empty : Namespace.Trim('/') + "/";

    public RobotKind Kind { get; }

    public RobotParameters Parameters { get; }

    /// <summary>Ground-truth pose in the world frame.</summary>
    public Pose2D Pose { get; internal set; }

    public Pose2D StartPose { get; set; }

    /// <summary>Pose estimated from wheel odometry, in the odom frame (starts at the origin).</summary>
    public Pose2D OdomPose { get; private set; }

    /// <summary>Actual body-frame velocities.</summary>
    public double LinearVelocity { get; protected set; }

    public double LateralVelocity { get; protected set; }

    public double AngularVelocity { get; protected set; }

    /// <summary>Velocities as measured by odometry.</summary>
    public double OdomLinearVelocity { get; private set; }

    public double OdomLateralVelocity { get; private set; }

    public double OdomAngularVelocity { get; private set; }

    /// <summary>Body-frame linear acceleration of the last step (for the IMU).</summary>
    public Vec2 LinearAcceleration { get; private set; }

    /// <summary>True while touching an obstacle or another robot.</summary>
    public bool Bumper { get; private set; }

    public int CollisionCount { get; private set; }

    /// <summary>Accumulated wheel rotation (rad) for joint states: left, right.</summary>
    public double LeftWheelPosition { get; protected set; }

    public double RightWheelPosition { get; protected set; }

    /// <summary>Ackermann steering angle (rad).</summary>
    public double SteeringAngle { get; protected set; }

    public SimArm? Arm { get; set; }

    public IReadOnlyList<SimSensor> Sensors => _sensors;

    public LidarSensor? Lidar => _sensors.OfType<LidarSensor>().FirstOrDefault();

    public ImuSensor? Imu => _sensors.OfType<ImuSensor>().FirstOrDefault();

    public GpsSensor? Gps => _sensors.OfType<GpsSensor>().FirstOrDefault();

    public BatterySensor? Battery => _sensors.OfType<BatterySensor>().FirstOrDefault();

    public CameraSensor? Camera => _sensors.OfType<CameraSensor>().FirstOrDefault();

    public IEnumerable<RangeSensor> RangeSensors => _sensors.OfType<RangeSensor>();

    /// <summary>Total distance driven (m).</summary>
    public double Odometer { get; private set; }

    public T AddSensor<T>(T sensor)
        where T : SimSensor
    {
        _sensors.Add(sensor);
        return sensor;
    }

    public void RemoveSensor(SimSensor sensor) => _sensors.Remove(sensor);

    /// <summary>Sets the velocity command (like a <c>geometry_msgs/Twist</c> on cmd_vel). Thread-safe.</summary>
    public void SetCommand(double linearX, double linearY, double angularZ, double worldTime)
    {
        lock (_commandGate)
        {
            _cmdVx = linearX;
            _cmdVy = linearY;
            _cmdWz = angularZ;
            _cmdTime = worldTime;
        }
    }

    public void Stop() => SetCommand(0, 0, 0, double.NegativeInfinity);

    /// <summary>Recent ground-truth positions, oldest first (for trails).</summary>
    public Vec2[] CopyTrail()
    {
        var result = new Vec2[_trailCount];
        for (var i = 0; i < _trailCount; i++)
        {
            result[i] = _trail[(_trailHead - _trailCount + i + _trail.Length) % _trail.Length];
        }

        return result;
    }

    /// <summary>Teleports the robot (odometry is left untouched, as when a robot is picked up).</summary>
    public void Teleport(Pose2D pose)
    {
        Pose = pose;
        LinearVelocity = LateralVelocity = AngularVelocity = 0;
    }

    internal void Reset()
    {
        Pose = StartPose;
        OdomPose = new Pose2D(0, 0, 0);
        LinearVelocity = LateralVelocity = AngularVelocity = 0;
        OdomLinearVelocity = OdomLateralVelocity = OdomAngularVelocity = 0;
        LeftWheelPosition = RightWheelPosition = SteeringAngle = 0;
        LinearAcceleration = Vec2.Zero;
        Bumper = false;
        CollisionCount = 0;
        Odometer = 0;
        _trailCount = _trailHead = 0;
        Stop();
        foreach (var s in _sensors)
        {
            s.Reset();
        }

        Arm?.Reset();
        OnReset();
    }

    /// <summary>Clears drive-internal state (ramped velocities) on reset.</summary>
    protected virtual void OnReset()
    {
    }

    /// <summary>
    /// Kinematics: from the command, compute the true body velocities and the velocities the wheel
    /// encoders report (which differ by slip). Must not allocate.
    /// </summary>
    protected abstract void ComputeVelocities(double cmdVx, double cmdVy, double cmdWz, double dt, SimRandom rng,
        out double vx, out double vy, out double wz, out double odomVx, out double odomVy, out double odomWz);

    /// <summary>Moves a velocity toward a target, respecting an acceleration limit.</summary>
    protected static double Ramp(double current, double target, double maxAccel, double dt)
    {
        var step = maxAccel * dt;
        return current + Math.Clamp(target - current, -step, step);
    }

    internal void Step(SimWorld world, double dt)
    {
        double cvx, cvy, cwz;
        lock (_commandGate)
        {
            var fresh = world.Time - _cmdTime <= Parameters.CommandTimeout;
            cvx = fresh ? _cmdVx : 0;
            cvy = fresh ? _cmdVy : 0;
            cwz = fresh ? _cmdWz : 0;
        }

        var p = Parameters;
        cvx = Math.Clamp(cvx, -p.MaxLinearSpeed, p.MaxLinearSpeed);
        cvy = Math.Clamp(cvy, -p.MaxLinearSpeed, p.MaxLinearSpeed);
        cwz = Math.Clamp(cwz, -p.MaxAngularSpeed, p.MaxAngularSpeed);

        var prevV = new Vec2(LinearVelocity, LateralVelocity);
        ComputeVelocities(cvx, cvy, cwz, dt, world.Random, out var vx, out var vy, out var wz, out var ovx, out var ovy, out var owz);

        // Friction scales achievable motion on slippery terrain.
        var traction = world.Friction;
        vx *= traction;
        vy *= traction;

        LinearVelocity = vx;
        LateralVelocity = vy;
        AngularVelocity = wz;
        LinearAcceleration = (new Vec2(vx, vy) - prevV) * (1 / dt);

        var before = Pose;
        Pose = Integrate(Pose, vx, vy, wz, dt);
        Bumper = world.ResolveCollisions(this, before);
        if (Bumper)
        {
            CollisionCount++;
            var moved = Pose.Position - before.Position;
            var local = moved.Rotate(-before.Theta) * (1 / dt);
            LinearVelocity = local.X;
            LateralVelocity = local.Y;
        }

        Odometer += (Pose.Position - before.Position).Length;

        OdomLinearVelocity = ovx;
        OdomLateralVelocity = ovy;
        OdomAngularVelocity = owz;
        OdomPose = Integrate(OdomPose, ovx, ovy, owz, dt);

        _trailAccum += (Pose.Position - before.Position).Length;
        if (_trailAccum >= 0.05 || _trailCount == 0)
        {
            _trailAccum = 0;
            _trail[_trailHead] = Pose.Position;
            _trailHead = (_trailHead + 1) % _trail.Length;
            _trailCount = Math.Min(_trailCount + 1, _trail.Length);
        }

        Arm?.Step(dt);
    }

    /// <summary>Exact integration of a constant body twist over <paramref name="dt"/>.</summary>
    internal static Pose2D Integrate(Pose2D p, double vx, double vy, double wz, double dt)
    {
        double dx, dy;
        var dth = wz * dt;
        if (Math.Abs(dth) < 1e-9)
        {
            dx = vx * dt;
            dy = vy * dt;
        }
        else
        {
            var s = Math.Sin(dth) / wz;
            var c = (1 - Math.Cos(dth)) / wz;
            dx = (vx * s) - (vy * c);
            dy = (vx * c) + (vy * s);
        }

        var (sin, cos) = Math.SinCos(p.Theta);
        return new Pose2D(p.X + (cos * dx) - (sin * dy), p.Y + (sin * dx) + (cos * dy), Angles.Normalize(p.Theta + dth));
    }

    public override string ToString() => $"{Kind} '{Name}' at {Pose}";
}

/// <summary>Differential-drive robot (TurtleBot3-like by default).</summary>
public sealed class DiffDriveRobot(string name, Pose2D pose, RobotParameters? parameters = null)
    : SimRobot(name, RobotKind.DiffDrive, pose, parameters ?? RobotParameters.TurtleBot3Burger())
{
    private double _v, _w;

    protected override void OnReset() => _v = _w = 0;

    protected override void ComputeVelocities(double cmdVx, double cmdVy, double cmdWz, double dt, SimRandom rng,
        out double vx, out double vy, out double wz, out double odomVx, out double odomVy, out double odomWz)
    {
        var p = Parameters;
        _v = Ramp(_v, cmdVx, p.MaxLinearAcceleration, dt);
        _w = Ramp(_w, cmdWz, p.MaxAngularAcceleration, dt);

        // Wheel speeds (rad/s) the motors achieve.
        var half = p.WheelSeparation / 2;
        var wl = (_v - (_w * half)) / p.WheelRadius;
        var wr = (_v + (_w * half)) / p.WheelRadius;
        LeftWheelPosition += wl * dt;
        RightWheelPosition += wr * dt;

        // True ground speed of each wheel includes slip; encoders see rotation plus a little noise.
        var gl = wl * p.WheelRadius * (1 + rng.Gaussian(p.WheelSlip));
        var gr = wr * p.WheelRadius * (1 + rng.Gaussian(p.WheelSlip));
        vx = (gl + gr) / 2;
        wz = (gr - gl) / p.WheelSeparation;
        vy = 0;

        var el = wl * p.WheelRadius * (1 + rng.Gaussian(p.EncoderNoise));
        var er = wr * p.WheelRadius * (1 + rng.Gaussian(p.EncoderNoise));
        odomVx = (el + er) / 2;
        odomWz = (er - el) / p.WheelSeparation;
        odomVy = 0;
    }
}

/// <summary>Car-like robot with Ackermann steering (bicycle model).</summary>
public sealed class AckermannRobot(string name, Pose2D pose, RobotParameters? parameters = null)
    : SimRobot(name, RobotKind.Ackermann, pose, parameters ?? RobotParameters.Car())
{
    private double _v;

    protected override void OnReset() => _v = 0;

    /// <summary>Interpret angular.z as a steering angle (rad) instead of a yaw rate.</summary>
    public bool AngularIsSteeringAngle { get; set; }

    protected override void ComputeVelocities(double cmdVx, double cmdVy, double cmdWz, double dt, SimRandom rng,
        out double vx, out double vy, out double wz, out double odomVx, out double odomVy, out double odomWz)
    {
        var p = Parameters;
        _v = Ramp(_v, cmdVx, p.MaxLinearAcceleration, dt);
        double targetSteer;
        if (AngularIsSteeringAngle)
        {
            targetSteer = cmdWz;
        }
        else if (Math.Abs(cmdVx) > 1e-3)
        {
            targetSteer = Math.Atan(cmdWz * p.Wheelbase / cmdVx);
        }
        else
        {
            targetSteer = SteeringAngle;
        }

        targetSteer = Math.Clamp(targetSteer, -p.MaxSteeringAngle, p.MaxSteeringAngle);
        SteeringAngle = Ramp(SteeringAngle, targetSteer, 3.0, dt);

        var wheelRate = _v / p.WheelRadius;
        LeftWheelPosition += wheelRate * dt;
        RightWheelPosition += wheelRate * dt;

        vx = _v * (1 + rng.Gaussian(p.WheelSlip));
        vy = 0;
        wz = vx * Math.Tan(SteeringAngle) / p.Wheelbase;

        odomVx = _v * (1 + rng.Gaussian(p.EncoderNoise));
        odomVy = 0;
        odomWz = odomVx * Math.Tan(SteeringAngle + rng.Gaussian(0.002)) / p.Wheelbase;
    }
}

/// <summary>Holonomic (mecanum/omni-wheel) base.</summary>
public sealed class OmniRobot(string name, Pose2D pose, RobotParameters? parameters = null)
    : SimRobot(name, RobotKind.Omni, pose, parameters ?? RobotParameters.OmniBase())
{
    private double _vx, _vy, _w;

    protected override void OnReset() => _vx = _vy = _w = 0;

    protected override void ComputeVelocities(double cmdVx, double cmdVy, double cmdWz, double dt, SimRandom rng,
        out double vx, out double vy, out double wz, out double odomVx, out double odomVy, out double odomWz)
    {
        var p = Parameters;
        _vx = Ramp(_vx, cmdVx, p.MaxLinearAcceleration, dt);
        _vy = Ramp(_vy, cmdVy, p.MaxLinearAcceleration, dt);
        _w = Ramp(_w, cmdWz, p.MaxAngularAcceleration, dt);
        LeftWheelPosition += (_vx - _vy) / p.WheelRadius * dt;
        RightWheelPosition += (_vx + _vy) / p.WheelRadius * dt;

        vx = _vx * (1 + rng.Gaussian(p.WheelSlip));
        vy = _vy * (1 + rng.Gaussian(p.WheelSlip));
        wz = _w * (1 + rng.Gaussian(p.WheelSlip));
        odomVx = _vx * (1 + rng.Gaussian(p.EncoderNoise));
        odomVy = _vy * (1 + rng.Gaussian(p.EncoderNoise));
        odomWz = _w * (1 + rng.Gaussian(p.EncoderNoise));
    }
}

/// <summary>
/// A kinematic arm placeholder: joints move toward targets at limited speed. Only joint states are
/// simulated (for rendering and joint_states), not dynamics.
/// </summary>
public sealed class SimArm
{
    private readonly double[] _initial;

    public SimArm(IReadOnlyList<string> jointNames, double maxJointVelocity = 1.0, IReadOnlyList<double>? linkLengths = null)
    {
        JointNames = jointNames.ToArray();
        Positions = new double[JointNames.Length];
        Targets = new double[JointNames.Length];
        _initial = new double[JointNames.Length];
        MaxJointVelocity = maxJointVelocity;
        LinkLengths = linkLengths?.ToArray() ?? Enumerable.Repeat(0.2, JointNames.Length).ToArray();
    }

    public static SimArm TwoDof() => new(["shoulder_joint", "elbow_joint"], 1.5, [0.25, 0.2]);

    public static SimArm SixDof() => new(["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"], 1.0, [0.15, 0.25, 0.22, 0.08, 0.08, 0.05]);

    public string[] JointNames { get; }

    public double[] Positions { get; }

    public double[] Targets { get; }

    public double[] LinkLengths { get; }

    public double MaxJointVelocity { get; set; }

    /// <summary>Sets joint targets by index (thread-safe enough for UI use: each double is written atomically).</summary>
    public void SetTargets(ReadOnlySpan<double> targets)
    {
        for (var i = 0; i < Math.Min(targets.Length, Targets.Length); i++)
        {
            Targets[i] = targets[i];
        }
    }

    internal void Step(double dt)
    {
        var step = MaxJointVelocity * dt;
        for (var i = 0; i < Positions.Length; i++)
        {
            Positions[i] += Math.Clamp(Targets[i] - Positions[i], -step, step);
        }
    }

    internal void Reset()
    {
        _initial.CopyTo(Positions, 0);
        _initial.CopyTo(Targets, 0);
    }
}
