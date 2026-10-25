using ROS2.Net.Msgs;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Msgs.Tf2Msgs;

namespace ROS2.Net.Control;

/// <summary>Planar odometry integrator for a differential drive (second-order Runge–Kutta).</summary>
public sealed class DiffDriveOdometry
{
    private double _lastLeft = double.NaN;
    private double _lastRight = double.NaN;

    public DiffDriveOdometry(double wheelSeparation, double leftWheelRadius, double rightWheelRadius)
    {
        WheelSeparation = wheelSeparation;
        LeftWheelRadius = leftWheelRadius;
        RightWheelRadius = rightWheelRadius;
    }

    public double WheelSeparation { get; set; }

    public double LeftWheelRadius { get; set; }

    public double RightWheelRadius { get; set; }

    public double X { get; private set; }

    public double Y { get; private set; }

    public double Heading { get; private set; }

    /// <summary>Linear velocity (m/s) of the last update.</summary>
    public double Linear { get; private set; }

    /// <summary>Angular velocity (rad/s) of the last update.</summary>
    public double Angular { get; private set; }

    public void Reset(double x = 0, double y = 0, double heading = 0)
    {
        X = x;
        Y = y;
        Heading = heading;
        _lastLeft = double.NaN;
        _lastRight = double.NaN;
    }

    /// <summary>Updates from wheel positions (rad). The first call only latches the positions.</summary>
    public void UpdateFromPositions(double leftPosition, double rightPosition, double dt)
    {
        if (double.IsNaN(_lastLeft))
        {
            _lastLeft = leftPosition;
            _lastRight = rightPosition;
            return;
        }

        var dl = (leftPosition - _lastLeft) * LeftWheelRadius;
        var dr = (rightPosition - _lastRight) * RightWheelRadius;
        _lastLeft = leftPosition;
        _lastRight = rightPosition;
        Integrate((dl + dr) * 0.5, (dr - dl) / WheelSeparation, dt);
    }

    /// <summary>Updates from body velocities (open loop).</summary>
    public void UpdateOpenLoop(double linear, double angular, double dt) => Integrate(linear * dt, angular * dt, dt);

    /// <summary>Integrates a displacement <paramref name="ds"/> and rotation <paramref name="dtheta"/>.</summary>
    public void Integrate(double ds, double dtheta, double dt)
    {
        var mid = Heading + (dtheta * 0.5);
        X += ds * Math.Cos(mid);
        Y += ds * Math.Sin(mid);
        Heading = Math.IEEERemainder(Heading + dtheta, 2 * Math.PI);
        if (dt > 0)
        {
            Linear = ds / dt;
            Angular = dtheta / dt;
        }
    }
}

/// <summary>Velocity, acceleration and jerk-free limiting of a scalar command, like <c>SpeedLimiter</c>.</summary>
public sealed class SpeedLimiter(double maxVelocity = double.PositiveInfinity, double maxAcceleration = double.PositiveInfinity)
{
    public double MaxVelocity { get; set; } = maxVelocity;

    public double MinVelocity { get; set; } = -maxVelocity;

    public double MaxAcceleration { get; set; } = maxAcceleration;

    public double Limit(double command, double previous, double dt)
    {
        var v = Math.Clamp(command, double.IsNaN(MinVelocity) ? -MaxVelocity : MinVelocity, MaxVelocity);
        if (dt > 0 && !double.IsPositiveInfinity(MaxAcceleration))
        {
            var dv = MaxAcceleration * dt;
            v = Math.Clamp(v, previous - dv, previous + dv);
        }

        return v;
    }
}

/// <summary>Shared plumbing of mobile base controllers: cmd_vel input with timeout, odometry and tf output.</summary>
public abstract class MobileBaseControllerBase : ControllerBase
{
    private static readonly int[] s_diagonal = [0, 7, 14, 21, 28, 35];
    private readonly RealtimeBuffer<TimedTwist> _command = new();
    private Publisher<Odometry>? _odomPublisher;
    private Publisher<TFMessage>? _tfPublisher;
    private long _lastPublishNs = long.MinValue;

    protected sealed class TimedTwist(double linear, double angular, long receivedNs)
    {
        public double Linear { get; } = linear;
        public double Angular { get; } = angular;
        public long ReceivedNs { get; } = receivedNs;
    }

    protected string OdomFrame { get; private set; } = "odom";

    protected string BaseFrame { get; private set; } = "base_link";

    protected bool EnableOdomTf { get; private set; } = true;

    protected double CmdVelTimeout { get; private set; } = 0.5;

    protected double PublishRate { get; private set; } = 50;

    protected bool OpenLoop { get; private set; }

    public SpeedLimiter LinearLimiter { get; } = new();

    public SpeedLimiter AngularLimiter { get; } = new();

    protected double LastLinear { get; set; }

    protected double LastAngular { get; set; }

    protected override void OnInit()
    {
        Node.DeclareParameter("odom_frame_id", "odom");
        Node.DeclareParameter("base_frame_id", "base_link");
        Node.DeclareParameter("enable_odom_tf", true);
        Node.DeclareParameter("cmd_vel_timeout", 0.5);
        Node.DeclareParameter("publish_rate", 50.0);
        Node.DeclareParameter("open_loop", false);
        Node.DeclareParameter("use_stamped_vel", false);
        Node.DeclareParameter("linear.x.max_velocity", double.PositiveInfinity);
        Node.DeclareParameter("linear.x.min_velocity", double.NaN);
        Node.DeclareParameter("linear.x.max_acceleration", double.PositiveInfinity);
        Node.DeclareParameter("angular.z.max_velocity", double.PositiveInfinity);
        Node.DeclareParameter("angular.z.min_velocity", double.NaN);
        Node.DeclareParameter("angular.z.max_acceleration", double.PositiveInfinity);
    }

    protected internal override CallbackReturn OnConfigure()
    {
        OdomFrame = Node.GetParameter<string>("odom_frame_id");
        BaseFrame = Node.GetParameter<string>("base_frame_id");
        EnableOdomTf = Node.GetParameter<bool>("enable_odom_tf");
        CmdVelTimeout = Node.GetParameter<double>("cmd_vel_timeout");
        PublishRate = Node.GetParameter<double>("publish_rate");
        OpenLoop = Node.GetParameter<bool>("open_loop");
        LinearLimiter.MaxVelocity = Node.GetParameter<double>("linear.x.max_velocity");
        LinearLimiter.MinVelocity = Node.GetParameter<double>("linear.x.min_velocity");
        LinearLimiter.MaxAcceleration = Node.GetParameter<double>("linear.x.max_acceleration");
        AngularLimiter.MaxVelocity = Node.GetParameter<double>("angular.z.max_velocity");
        AngularLimiter.MinVelocity = Node.GetParameter<double>("angular.z.min_velocity");
        AngularLimiter.MaxAcceleration = Node.GetParameter<double>("angular.z.max_acceleration");

        if (Node.GetParameter<bool>("use_stamped_vel"))
        {
            Node.CreateSubscription<TwistStamped>("~/cmd_vel", m => SetCommand(m.Twist.Linear.X, m.Twist.Angular.Z));
        }
        else
        {
            Node.CreateSubscription<Twist>("~/cmd_vel", m => SetCommand(m.Linear.X, m.Angular.Z));
        }

        _odomPublisher ??= Node.CreatePublisher<Odometry>("~/odom");
        _tfPublisher ??= Node.CreatePublisher<TFMessage>("/tf", QosProfile.Default.KeepLast(100));
        return CallbackReturn.Success;
    }

    /// <summary>Sets the body velocity command directly (same as receiving cmd_vel).</summary>
    public void SetCommand(double linear, double angular) =>
        _command.Write(new TimedTwist(linear, angular, RosTime.Now().Nanoseconds));

    /// <summary>The current command after timeout and limits.</summary>
    protected (double Linear, double Angular) ReadCommand(RosTime time, double dt)
    {
        var cmd = _command.Read();
        double v = 0, w = 0;
        if (cmd is not null && (RosTime.Now().Nanoseconds - cmd.ReceivedNs) / 1e9 <= CmdVelTimeout)
        {
            v = cmd.Linear;
            w = cmd.Angular;
        }

        v = LinearLimiter.Limit(v, LastLinear, dt);
        w = AngularLimiter.Limit(w, LastAngular, dt);
        LastLinear = v;
        LastAngular = w;
        return (v, w);
    }

    /// <summary>Publishes odometry and the odom→base tf at <see cref="PublishRate"/>.</summary>
    protected void PublishOdometry(RosTime time, double x, double y, double heading, double linear, double angular)
    {
        if (PublishRate > 0 && _lastPublishNs != long.MinValue && time.Nanoseconds - _lastPublishNs < 1e9 / PublishRate)
        {
            return;
        }

        _lastPublishNs = time.Nanoseconds;
        var stamp = RosTime.Now();
        var q = MessageExtensions.QuaternionFromYaw(heading);
        var odom = new Odometry { ChildFrameId = BaseFrame };
        odom.Header.Stamp = stamp.ToMsg();
        odom.Header.FrameId = OdomFrame;
        odom.Pose.Pose.Position.X = x;
        odom.Pose.Pose.Position.Y = y;
        odom.Pose.Pose.Orientation = q;
        odom.Twist.Twist.Linear.X = linear;
        odom.Twist.Twist.Angular.Z = angular;
        foreach (var i in s_diagonal)
        {
            odom.Pose.Covariance[i] = 0.001;
            odom.Twist.Covariance[i] = 0.001;
        }

        _odomPublisher!.Publish(odom);

        if (EnableOdomTf)
        {
            var tf = new TransformStamped { ChildFrameId = BaseFrame };
            tf.Header.Stamp = stamp.ToMsg();
            tf.Header.FrameId = OdomFrame;
            tf.Transform.Translation.X = x;
            tf.Transform.Translation.Y = y;
            tf.Transform.Rotation = q;
            _tfPublisher!.Publish(new TFMessage { Transforms = [tf] });
        }
    }
}

/// <summary>
/// Differential drive controller like <c>diff_drive_controller</c>: <c>~/cmd_vel</c> (Twist, or TwistStamped
/// with <c>use_stamped_vel</c>) → wheel velocity commands; wheel positions → <c>~/odom</c> and the odom→base tf.
/// Parameters: <c>left_wheel_names</c>, <c>right_wheel_names</c>, <c>wheel_separation</c>, <c>wheel_radius</c>,
/// multipliers, limits (<c>linear.x.*</c>, <c>angular.z.*</c>), <c>cmd_vel_timeout</c>, frames.
/// </summary>
public sealed class DiffDriveController : MobileBaseControllerBase
{
    private string[] _left = [];
    private string[] _right = [];
    private int[] _leftState = [];
    private int[] _rightState = [];
    private int _leftCount;
    private double _radius;
    private double _separation;
    private double _leftMul = 1;
    private double _rightMul = 1;

    public DiffDriveOdometry Odometry { get; private set; } = new(0.5, 0.1, 0.1);

    public override InterfaceConfiguration CommandInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_left.Concat(_right).Select(j => j + "/" + HardwareInterfaceNames.Velocity));

    public override InterfaceConfiguration StateInterfaceConfiguration =>
        OpenLoop ? InterfaceConfiguration.None : InterfaceConfiguration.Individual(_left.Concat(_right).Select(j => j + "/" + HardwareInterfaceNames.Position));

    protected override void OnInit()
    {
        base.OnInit();
        Node.DeclareParameter("left_wheel_names", Array.Empty<string>());
        Node.DeclareParameter("right_wheel_names", Array.Empty<string>());
        Node.DeclareParameter("wheel_separation", 0.0);
        Node.DeclareParameter("wheel_radius", 0.0);
        Node.DeclareParameter("wheel_separation_multiplier", 1.0);
        Node.DeclareParameter("left_wheel_radius_multiplier", 1.0);
        Node.DeclareParameter("right_wheel_radius_multiplier", 1.0);
    }

    protected internal override CallbackReturn OnConfigure()
    {
        _left = Node.GetParameter<string[]>("left_wheel_names");
        _right = Node.GetParameter<string[]>("right_wheel_names");
        _radius = Node.GetParameter<double>("wheel_radius");
        _separation = Node.GetParameter<double>("wheel_separation") * Node.GetParameter<double>("wheel_separation_multiplier");
        _leftMul = Node.GetParameter<double>("left_wheel_radius_multiplier");
        _rightMul = Node.GetParameter<double>("right_wheel_radius_multiplier");
        if (_left.Length == 0 || _left.Length != _right.Length || _radius <= 0 || _separation <= 0)
        {
            return CallbackReturn.Failure;
        }

        Odometry = new DiffDriveOdometry(_separation, _radius * _leftMul, _radius * _rightMul);
        return base.OnConfigure();
    }

    protected internal override CallbackReturn OnActivate()
    {
        _leftCount = _left.Length;
        _leftState = _left.Select(j => FindState(j + "/" + HardwareInterfaceNames.Position)).ToArray();
        _rightState = _right.Select(j => FindState(j + "/" + HardwareInterfaceNames.Position)).ToArray();
        Odometry.Reset(Odometry.X, Odometry.Y, Odometry.Heading);
        LastLinear = 0;
        LastAngular = 0;
        return CallbackReturn.Success;
    }

    protected internal override CallbackReturn OnDeactivate()
    {
        foreach (var c in CommandInterfaces)
        {
            c.Value = 0;
        }

        return CallbackReturn.Success;
    }

    public override ControllerReturn Update(RosTime time, RosDuration period)
    {
        var dt = period.TotalSeconds;
        var (v, w) = ReadCommand(time, dt);

        if (OpenLoop)
        {
            Odometry.UpdateOpenLoop(v, w, dt);
        }
        else
        {
            var states = StateInterfaces;
            double l = 0, r = 0;
            for (var i = 0; i < _leftState.Length; i++)
            {
                l += states[_leftState[i]].Value;
                r += states[_rightState[i]].Value;
            }

            Odometry.UpdateFromPositions(l / _leftState.Length, r / _rightState.Length, dt);
        }

        var leftVel = (v - (w * _separation * 0.5)) / (_radius * _leftMul);
        var rightVel = (v + (w * _separation * 0.5)) / (_radius * _rightMul);
        var commands = CommandInterfaces;
        for (var i = 0; i < commands.Count; i++)
        {
            commands[i].Value = i < _leftCount ? leftVel : rightVel;
        }

        PublishOdometry(time, Odometry.X, Odometry.Y, Odometry.Heading, Odometry.Linear, Odometry.Angular);
        return ControllerReturn.Ok;
    }
}

/// <summary>
/// Ackermann steering controller using the bicycle model: <c>~/cmd_vel</c> → traction wheel velocity and
/// steering angle; traction velocity + steering position → odometry.
/// Parameters: <c>traction_joints</c>, <c>steering_joints</c>, <c>wheelbase</c>, <c>wheel_radius</c>, <c>max_steering_angle</c>.
/// </summary>
public sealed class AckermannSteeringController : MobileBaseControllerBase
{
    private string[] _traction = [];
    private string[] _steering = [];
    private double _wheelbase;
    private double _radius;
    private double _maxSteer;
    private int[] _tractionState = [];
    private int[] _steeringState = [];

    public DiffDriveOdometry Odometry { get; } = new(1, 1, 1);

    public override InterfaceConfiguration CommandInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_traction.Select(j => j + "/" + HardwareInterfaceNames.Velocity)
            .Concat(_steering.Select(j => j + "/" + HardwareInterfaceNames.Position)));

    public override InterfaceConfiguration StateInterfaceConfiguration =>
        OpenLoop ? InterfaceConfiguration.None : InterfaceConfiguration.Individual(_traction.Select(j => j + "/" + HardwareInterfaceNames.Velocity)
            .Concat(_steering.Select(j => j + "/" + HardwareInterfaceNames.Position)));

    protected override void OnInit()
    {
        base.OnInit();
        Node.DeclareParameter("traction_joints", Array.Empty<string>());
        Node.DeclareParameter("steering_joints", Array.Empty<string>());
        Node.DeclareParameter("wheelbase", 0.0);
        Node.DeclareParameter("wheel_radius", 0.0);
        Node.DeclareParameter("max_steering_angle", 0.6);
    }

    protected internal override CallbackReturn OnConfigure()
    {
        _traction = Node.GetParameter<string[]>("traction_joints");
        _steering = Node.GetParameter<string[]>("steering_joints");
        _wheelbase = Node.GetParameter<double>("wheelbase");
        _radius = Node.GetParameter<double>("wheel_radius");
        _maxSteer = Node.GetParameter<double>("max_steering_angle");
        return _traction.Length == 0 || _steering.Length == 0 || _wheelbase <= 0 || _radius <= 0 ? CallbackReturn.Failure : base.OnConfigure();
    }

    protected internal override CallbackReturn OnActivate()
    {
        _tractionState = _traction.Select(j => FindState(j + "/" + HardwareInterfaceNames.Velocity)).ToArray();
        _steeringState = _steering.Select(j => FindState(j + "/" + HardwareInterfaceNames.Position)).ToArray();
        return CallbackReturn.Success;
    }

    /// <summary>Steering angle (rad) for a body command; zero when not moving forward.</summary>
    public static double SteeringFor(double linear, double angular, double wheelbase) =>
        Math.Abs(linear) < 1e-6 ? 0 : Math.Atan(wheelbase * angular / linear);

    public override ControllerReturn Update(RosTime time, RosDuration period)
    {
        var dt = period.TotalSeconds;
        var (v, w) = ReadCommand(time, dt);
        var steer = Math.Clamp(SteeringFor(v, w, _wheelbase), -_maxSteer, _maxSteer);

        double speed, angle;
        if (OpenLoop)
        {
            speed = v;
            angle = steer;
        }
        else
        {
            var states = StateInterfaces;
            double ts = 0, ss = 0;
            foreach (var i in _tractionState)
            {
                ts += states[i].Value;
            }

            foreach (var i in _steeringState)
            {
                ss += states[i].Value;
            }

            speed = ts / _tractionState.Length * _radius;
            angle = ss / _steeringState.Length;
        }

        var yawRate = speed * Math.Tan(angle) / _wheelbase;
        Odometry.Integrate(speed * dt, yawRate * dt, dt);

        var commands = CommandInterfaces;
        for (var i = 0; i < commands.Count; i++)
        {
            commands[i].Value = i < _traction.Length ? v / _radius : steer;
        }

        PublishOdometry(time, Odometry.X, Odometry.Y, Odometry.Heading, Odometry.Linear, Odometry.Angular);
        return ControllerReturn.Ok;
    }
}
