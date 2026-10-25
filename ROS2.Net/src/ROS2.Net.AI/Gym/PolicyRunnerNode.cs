using ROS2.Net.Msgs;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Simulation;
using Pose2D = ROS2.Net.Simulation.Pose2D;

namespace ROS2.Net.AI.Gym;

/// <summary>Options of <see cref="PolicyRunnerNode"/>; must match the training configuration.</summary>
public sealed class PolicyRunnerOptions
{
    public string ScanTopic { get; set; } = "scan";

    public string OdomTopic { get; set; } = "odom";

    /// <summary>Goal in the odom frame (geometry_msgs/PoseStamped).</summary>
    public string GoalTopic { get; set; } = "goal_pose";

    public string CmdVelTopic { get; set; } = "cmd_vel";

    public int Sectors { get; set; } = 8;

    public double LidarRange { get; set; } = 3.5;

    public double MaxGoalDistance { get; set; } = 3.0;

    public double ControlPeriod { get; set; } = 0.2;

    public double ForwardSpeed { get; set; } = 0.22;

    public double TurnSpeed { get; set; } = 1.5;

    public double GoalTolerance { get; set; } = 0.3;
}

/// <summary>
/// Deploys a trained discrete policy: builds the <see cref="NavigationObservation"/> from
/// <c>scan</c> and <c>odom</c>, and drives <c>cmd_vel</c> toward the latest <c>goal_pose</c>.
/// The control loop is a node timer, so spin the node.
/// </summary>
public sealed class PolicyRunnerNode : Node
{
    private readonly Func<ReadOnlySpan<float>, int> _policy;
    private readonly PolicyRunnerOptions _options;
    private readonly Publisher<Twist> _cmd;
    private readonly float[] _observation;
    private LaserScan? _scan;
    private Odometry? _odom;
    private Vec2? _goal;

    public PolicyRunnerNode(Ros2Context context, Func<ReadOnlySpan<float>, int> policy, PolicyRunnerOptions? options = null, string name = "policy_runner", NodeOptions? nodeOptions = null)
        : base(context, name, nodeOptions)
    {
        _policy = policy;
        _options = options ?? new PolicyRunnerOptions();
        _observation = new float[NavigationObservation.Size(_options.Sectors)];
        _cmd = CreatePublisher<Twist>(_options.CmdVelTopic);
        CreateSubscription<LaserScan>(_options.ScanTopic, m => _scan = m, QosProfile.SensorData);
        CreateSubscription<Odometry>(_options.OdomTopic, m => _odom = m);
        CreateSubscription<PoseStamped>(_options.GoalTopic, m =>
        {
            _goal = new Vec2(m.Pose.Position.X, m.Pose.Position.Y);
            GoalReached = false;
        });
        CreateTimer(TimeSpan.FromSeconds(_options.ControlPeriod), Control);
    }

    /// <summary>True once the current goal is within tolerance.</summary>
    public bool GoalReached { get; private set; }

    /// <summary>Sets the goal directly (odom frame).</summary>
    public void SetGoal(double x, double y)
    {
        _goal = new Vec2(x, y);
        GoalReached = false;
    }

    private void Control()
    {
        if (_goal is not { } goal || _odom is not { } odom)
        {
            return;
        }

        var p = odom.Pose.Pose;
        var pose = new Pose2D(p.Position.X, p.Position.Y, p.Orientation.Yaw());
        if ((goal - pose.Position).Length <= _options.GoalTolerance)
        {
            if (!GoalReached)
            {
                GoalReached = true;
                _cmd.Publish(new Twist());
            }

            return;
        }

        var scan = _scan;
        NavigationObservation.Fill(_observation, scan?.Ranges ?? [], scan?.AngleMin ?? 0, scan?.AngleIncrement ?? 0, _options.LidarRange,
            _options.Sectors, pose, goal, _options.MaxGoalDistance * 1.5);
        var action = _policy(_observation);
        var (v, w) = NavigationObservation.DefaultAction(action, _options.ForwardSpeed, _options.TurnSpeed);
        _cmd.Publish(MessageExtensions.Twist2D(v, w));
    }
}
