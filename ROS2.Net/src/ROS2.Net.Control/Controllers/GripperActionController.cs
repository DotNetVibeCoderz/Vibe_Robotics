using ROS2.Net.Msgs.Ros2netControlMsgs;

namespace ROS2.Net.Control;

/// <summary>
/// Gripper controller with a <c>~/gripper_cmd</c> action (<c>ros2net_control_msgs/action/GripperCommand</c>, the
/// same fields as control_msgs/GripperCommand but flattened): commands a position, reports reached/stalled.
/// A stall is velocity below <c>stall_velocity_threshold</c> for <c>stall_timeout</c> seconds, or effort above the
/// goal's <c>max_effort</c>. Parameters: <c>joint</c>, <c>goal_tolerance</c>, <c>allow_stalling</c>,
/// <c>stall_velocity_threshold</c>, <c>stall_timeout</c>.
/// </summary>
public sealed class GripperActionController : ControllerBase
{
    private readonly RealtimeBuffer<Goal> _pending = new();
    private string _joint = string.Empty;
    private double _tolerance;
    private bool _allowStalling;
    private double _stallVelocity;
    private double _stallTimeout;
    private int _effortState = -1;
    private int _velocityState = -1;
    private Goal? _active;
    private double _stillSince = double.NaN;
    private double _position;
    private double _effort;

    internal sealed class Goal(double position, double maxEffort)
    {
        public double Position { get; } = position;
        public double MaxEffort { get; } = maxEffort;
        public TaskCompletionSource<GripperCommandResult> Done { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
    }

    public override InterfaceConfiguration CommandInterfaceConfiguration => InterfaceConfiguration.Individual(_joint + "/" + HardwareInterfaceNames.Position);

    public override InterfaceConfiguration StateInterfaceConfiguration => InterfaceConfiguration.Individual(StateNames());

    public double Position => Volatile.Read(ref _position);

    private IEnumerable<string> StateNames()
    {
        yield return _joint + "/" + HardwareInterfaceNames.Position;
        yield return _joint + "/" + HardwareInterfaceNames.Velocity;
    }

    protected override void OnInit()
    {
        Node.DeclareParameter("joint", string.Empty);
        Node.DeclareParameter("goal_tolerance", 0.01);
        Node.DeclareParameter("allow_stalling", false);
        Node.DeclareParameter("stall_velocity_threshold", 0.001);
        Node.DeclareParameter("stall_timeout", 1.0);
    }

    protected internal override CallbackReturn OnConfigure()
    {
        _joint = Node.GetParameter<string>("joint");
        _tolerance = Node.GetParameter<double>("goal_tolerance");
        _allowStalling = Node.GetParameter<bool>("allow_stalling");
        _stallVelocity = Node.GetParameter<double>("stall_velocity_threshold");
        _stallTimeout = Node.GetParameter<double>("stall_timeout");
        if (_joint.Length == 0)
        {
            return CallbackReturn.Failure;
        }

        Node.CreateActionServer<GripperCommandGoal, GripperCommandResult, GripperCommandFeedback>("~/gripper_cmd", async (handle, ct) =>
        {
            var goal = new Goal(handle.Goal.Position, handle.Goal.MaxEffort);
            _pending.Write(goal);
            while (!goal.Done.Task.IsCompleted)
            {
                ct.ThrowIfCancellationRequested();
                handle.PublishFeedback(new GripperCommandFeedback { Position = Position, Effort = Volatile.Read(ref _effort) });
                await Task.WhenAny(goal.Done.Task, Task.Delay(50, ct)).ConfigureAwait(false);
            }

            var result = await goal.Done.Task.ConfigureAwait(false);
            if (!result.ReachedGoal && !(result.Stalled && _allowStalling))
            {
                handle.Abort();
            }

            return result;
        });
        return CallbackReturn.Success;
    }

    protected internal override CallbackReturn OnActivate()
    {
        _velocityState = FindState(_joint + "/" + HardwareInterfaceNames.Velocity);
        _effortState = FindState(_joint + "/" + HardwareInterfaceNames.Effort);
        CommandInterfaces[0].Value = StateInterfaces[0].Value;
        return CallbackReturn.Success;
    }

    /// <summary>Commands the gripper directly and awaits the outcome.</summary>
    public Task<GripperCommandResult> CommandAsync(double position, double maxEffort = 0)
    {
        var goal = new Goal(position, maxEffort);
        _pending.Write(goal);
        return goal.Done.Task;
    }

    public override ControllerReturn Update(RosTime time, RosDuration period)
    {
        var position = StateInterfaces[0].Value;
        var velocity = _velocityState >= 0 ? StateInterfaces[_velocityState].Value : 0;
        var effort = _effortState >= 0 ? StateInterfaces[_effortState].Value : 0;
        Volatile.Write(ref _position, position);
        Volatile.Write(ref _effort, effort);

        if (_pending.Take() is { } incoming)
        {
            _active?.Done.TrySetResult(new GripperCommandResult { Position = position, Effort = effort, ReachedGoal = false });
            _active = incoming;
            _stillSince = double.NaN;
            CommandInterfaces[0].Value = incoming.Position;
        }

        var goal = _active;
        if (goal is null)
        {
            return ControllerReturn.Ok;
        }

        var now = time.Seconds;
        if (Math.Abs(position - goal.Position) <= _tolerance)
        {
            Complete(goal, position, effort, reached: true, stalled: false);
            return ControllerReturn.Ok;
        }

        var overEffort = goal.MaxEffort > 0 && Math.Abs(effort) >= goal.MaxEffort;
        if (Math.Abs(velocity) < _stallVelocity || overEffort)
        {
            if (double.IsNaN(_stillSince))
            {
                _stillSince = now;
            }

            if (overEffort || now - _stillSince >= _stallTimeout)
            {
                // Hold where it stalled so it does not keep squeezing.
                CommandInterfaces[0].Value = position;
                Complete(goal, position, effort, reached: false, stalled: true);
            }
        }
        else
        {
            _stillSince = double.NaN;
        }

        return ControllerReturn.Ok;
    }

    private void Complete(Goal goal, double position, double effort, bool reached, bool stalled)
    {
        _active = null;
        goal.Done.TrySetResult(new GripperCommandResult { Position = position, Effort = effort, ReachedGoal = reached, Stalled = stalled });
    }
}
