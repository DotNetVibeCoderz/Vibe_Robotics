using ROS2.Net.Msgs.ControlMsgs;
using ROS2.Net.Msgs.TrajectoryMsgs;

namespace ROS2.Net.Control;

/// <summary>
/// A trajectory sampled with splines: quintic where positions, velocities and accelerations are given,
/// cubic Hermite with positions and velocities, linear with positions only (as joint_trajectory_controller does).
/// </summary>
public sealed class SampledTrajectory
{
    private readonly double[] _times;
    private readonly double[][] _pos;
    private readonly double[][]? _vel;
    private readonly double[][]? _acc;
    private int _segment;

    /// <param name="times">Seconds from trajectory start, strictly increasing.</param>
    /// <param name="positions">Positions per point.</param>
    /// <param name="velocities">Velocities per point, or null.</param>
    /// <param name="accelerations">Accelerations per point, or null (requires velocities).</param>
    public SampledTrajectory(double[] times, double[][] positions, double[][]? velocities, double[][]? accelerations)
    {
        _times = times;
        _pos = positions;
        _vel = velocities;
        _acc = accelerations;
        JointCount = positions.Length > 0 ? positions[0].Length : 0;
    }

    public int JointCount { get; }

    public double Duration => _times.Length == 0 ? 0 : _times[^1];

    public int PointCount => _times.Length;

    /// <summary>The final point's positions.</summary>
    public double[] FinalPositions => _pos[^1];

    /// <summary>Builds a trajectory for the controller's joints from a message, prepending the current state at t=0 when needed.</summary>
    public static SampledTrajectory FromMessage(JointTrajectory traj, IReadOnlyList<string> controllerJoints, double[] currentPositions, double[] currentVelocities)
    {
        var map = new int[controllerJoints.Count];
        for (var i = 0; i < controllerJoints.Count; i++)
        {
            map[i] = Array.IndexOf(traj.JointNames, controllerJoints[i]);
            if (map[i] < 0)
            {
                throw new ArgumentException($"Trajectory has no joint '{controllerJoints[i]}'.");
            }
        }

        var points = traj.Points;
        var prepend = points.Length == 0 || RosDuration.FromMsg(points[0].TimeFromStart).TotalSeconds > 1e-9;
        var n = points.Length + (prepend ? 1 : 0);
        var times = new double[n];
        var pos = new double[n][];
        var hasVel = points.All(p => p.Velocities.Length == traj.JointNames.Length);
        var hasAcc = hasVel && points.All(p => p.Accelerations.Length == traj.JointNames.Length);
        var vel = hasVel ? new double[n][] : null;
        var acc = hasAcc ? new double[n][] : null;
        var k = 0;
        if (prepend)
        {
            pos[0] = (double[])currentPositions.Clone();
            if (vel is not null)
            {
                vel[0] = (double[])currentVelocities.Clone();
            }

            if (acc is not null)
            {
                acc[0] = new double[controllerJoints.Count];
            }

            k = 1;
        }

        double last = 0;
        foreach (var p in points)
        {
            var t = RosDuration.FromMsg(p.TimeFromStart).TotalSeconds;
            if (k > 0 && t <= last)
            {
                throw new ArgumentException("Trajectory point times must be strictly increasing.");
            }

            if (p.Positions.Length != traj.JointNames.Length)
            {
                throw new ArgumentException("Every trajectory point needs one position per joint.");
            }

            times[k] = t;
            last = t;
            pos[k] = map.Select(m => p.Positions[m]).ToArray();
            if (vel is not null)
            {
                vel[k] = map.Select(m => p.Velocities[m]).ToArray();
            }

            if (acc is not null)
            {
                acc[k] = map.Select(m => p.Accelerations[m]).ToArray();
            }

            k++;
        }

        return new SampledTrajectory(times, pos, vel, acc);
    }

    /// <summary>Samples at <paramref name="t"/> seconds into the trajectory. Allocation-free.</summary>
    public void Sample(double t, double[] position, double[] velocity, double[] acceleration)
    {
        if (_times.Length == 1 || t >= _times[^1])
        {
            var last = _pos[^1];
            for (var j = 0; j < JointCount; j++)
            {
                position[j] = last[j];
                velocity[j] = 0;
                acceleration[j] = 0;
            }

            return;
        }

        t = Math.Max(0, t);
        if (_segment >= _times.Length - 1 || t < _times[_segment])
        {
            _segment = 0;
        }

        while (_segment < _times.Length - 2 && t >= _times[_segment + 1])
        {
            _segment++;
        }

        var s = _segment;
        var T = _times[s + 1] - _times[s];
        var tau = t - _times[s];
        for (var j = 0; j < JointCount; j++)
        {
            var p0 = _pos[s][j];
            var p1 = _pos[s + 1][j];
            if (_acc is not null)
            {
                Quintic(p0, _vel![s][j], _acc[s][j], p1, _vel[s + 1][j], _acc[s + 1][j], T, tau, out position[j], out velocity[j], out acceleration[j]);
            }
            else if (_vel is not null)
            {
                Cubic(p0, _vel[s][j], p1, _vel[s + 1][j], T, tau, out position[j], out velocity[j], out acceleration[j]);
            }
            else
            {
                var slope = (p1 - p0) / T;
                position[j] = p0 + (slope * tau);
                velocity[j] = slope;
                acceleration[j] = 0;
            }
        }
    }

    private static void Cubic(double p0, double v0, double p1, double v1, double T, double t, out double p, out double v, out double a)
    {
        var c2 = ((3 * (p1 - p0)) - (((2 * v0) + v1) * T)) / (T * T);
        var c3 = ((2 * (p0 - p1)) + ((v0 + v1) * T)) / (T * T * T);
        p = p0 + (v0 * t) + (c2 * t * t) + (c3 * t * t * t);
        v = v0 + (2 * c2 * t) + (3 * c3 * t * t);
        a = (2 * c2) + (6 * c3 * t);
    }

    private static void Quintic(double p0, double v0, double a0, double p1, double v1, double a1, double T, double t, out double p, out double v, out double a)
    {
        var T2 = T * T;
        var T3 = T2 * T;
        var T4 = T3 * T;
        var T5 = T4 * T;
        var c3 = ((20 * (p1 - p0)) - (((8 * v1) + (12 * v0)) * T) - (((3 * a0) - a1) * T2)) / (2 * T3);
        var c4 = ((30 * (p0 - p1)) + (((14 * v1) + (16 * v0)) * T) + (((3 * a0) - (2 * a1)) * T2)) / (2 * T4);
        var c5 = ((12 * (p1 - p0)) - ((6 * (v1 + v0)) * T) - ((a0 - a1) * T2)) / (2 * T5);
        var t2 = t * t;
        var t3 = t2 * t;
        p = p0 + (v0 * t) + (0.5 * a0 * t2) + (c3 * t3) + (c4 * t3 * t) + (c5 * t3 * t2);
        v = v0 + (a0 * t) + (3 * c3 * t2) + (4 * c4 * t3) + (5 * c5 * t3 * t);
        a = a0 + (6 * c3 * t) + (12 * c4 * t2) + (20 * c5 * t3);
    }
}

/// <summary>
/// joint_trajectory_controller: follows trajectories from the <c>~/follow_joint_trajectory</c> action
/// (control_msgs/FollowJointTrajectory) or the <c>~/joint_trajectory</c> topic, on position (and optionally
/// velocity, with feed-forward + P gain) command interfaces, checking path and goal tolerances. A new goal
/// preempts the running one. Parameters: <c>joints</c>, <c>command_interfaces</c>, <c>state_interfaces</c>,
/// <c>constraints.goal_time</c>, <c>constraints.stopped_velocity_tolerance</c>, <c>constraints.&lt;joint&gt;.goal</c>,
/// <c>constraints.&lt;joint&gt;.trajectory</c>, <c>gains.&lt;joint&gt;.p</c>.
/// </summary>
public sealed class JointTrajectoryController : ControllerBase
{
    private readonly RealtimeBuffer<ActiveGoal> _pending = new();
    private readonly object _feedbackGate = new();
    private string[] _joints = [];
    private string[] _commandTypes = [HardwareInterfaceNames.Position];
    private string[] _stateTypes = [HardwareInterfaceNames.Position, HardwareInterfaceNames.Velocity];
    private double[] _goalTol = [];
    private double[] _pathTol = [];
    private double[] _gains = [];
    private double _goalTime;
    private double _stoppedVel;
    private int[] _posCmd = [];
    private int[] _velCmd = [];
    private int[] _posState = [];
    private int[] _velState = [];
    private double[] _actualPos = [];
    private double[] _actualVel = [];
    private double[] _desiredPos = [];
    private double[] _desiredVel = [];
    private double[] _desiredAcc = [];
    private ActiveGoal? _active;
    private long _startNs;

    /// <summary>An executing trajectory and how it ends.</summary>
    internal sealed class ActiveGoal(SampledTrajectory trajectory, double[] goalTolerance, double[] pathTolerance, double goalTime)
    {
        public SampledTrajectory Trajectory { get; } = trajectory;
        public double[] GoalTolerance { get; } = goalTolerance;
        public double[] PathTolerance { get; } = pathTolerance;
        public double GoalTime { get; } = goalTime;
        public long StartNs { get; set; } = long.MinValue;
        public TaskCompletionSource<(int Code, string Message)> Done { get; } = new(TaskCreationOptions.RunContinuationsAsynchronously);
    }

    public IReadOnlyList<string> Joints => _joints;

    /// <summary>True while a trajectory is executing.</summary>
    public bool IsExecuting => _active is not null;

    public override InterfaceConfiguration CommandInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_commandTypes.SelectMany(t => _joints.Select(j => j + "/" + t)));

    public override InterfaceConfiguration StateInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_stateTypes.SelectMany(t => _joints.Select(j => j + "/" + t)));

    protected override void OnInit()
    {
        Node.DeclareParameter("joints", Array.Empty<string>());
        Node.DeclareParameter("command_interfaces", new[] { HardwareInterfaceNames.Position });
        Node.DeclareParameter("state_interfaces", new[] { HardwareInterfaceNames.Position, HardwareInterfaceNames.Velocity });
        Node.DeclareParameter("constraints.goal_time", 0.0);
        Node.DeclareParameter("constraints.stopped_velocity_tolerance", 0.01);
    }

    protected internal override CallbackReturn OnConfigure()
    {
        _joints = Node.GetParameter<string[]>("joints");
        _commandTypes = Node.GetParameter<string[]>("command_interfaces");
        _stateTypes = Node.GetParameter<string[]>("state_interfaces");
        if (_joints.Length == 0 || !_stateTypes.Contains(HardwareInterfaceNames.Position) || _commandTypes.Except([HardwareInterfaceNames.Position, HardwareInterfaceNames.Velocity]).Any())
        {
            return CallbackReturn.Failure;
        }

        _goalTime = Node.GetParameter<double>("constraints.goal_time");
        _stoppedVel = Node.GetParameter<double>("constraints.stopped_velocity_tolerance");
        _goalTol = _joints.Select(j => Node.HasParameter($"constraints.{j}.goal") ? Node.GetParameter<double>($"constraints.{j}.goal") : Node.DeclareParameter($"constraints.{j}.goal", 0.0)).ToArray();
        _pathTol = _joints.Select(j => Node.HasParameter($"constraints.{j}.trajectory") ? Node.GetParameter<double>($"constraints.{j}.trajectory") : Node.DeclareParameter($"constraints.{j}.trajectory", 0.0)).ToArray();
        _gains = _joints.Select(j => Node.HasParameter($"gains.{j}.p") ? Node.GetParameter<double>($"gains.{j}.p") : Node.DeclareParameter($"gains.{j}.p", 5.0)).ToArray();

        var n = _joints.Length;
        _actualPos = new double[n];
        _actualVel = new double[n];
        _desiredPos = new double[n];
        _desiredVel = new double[n];
        _desiredAcc = new double[n];

        Node.CreateSubscription<JointTrajectory>("~/joint_trajectory", m =>
        {
            try
            {
                Submit(m, null, null, _goalTime);
            }
            catch (ArgumentException)
            {
                // Invalid topic trajectories are ignored, as upstream does.
            }
        });
        Node.CreateActionServer<FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback>("~/follow_joint_trajectory", ExecuteAsync);
        return CallbackReturn.Success;
    }

    protected internal override CallbackReturn OnActivate()
    {
        _posCmd = _joints.Select(j => FindCommand(j + "/" + HardwareInterfaceNames.Position)).ToArray();
        _velCmd = _joints.Select(j => FindCommand(j + "/" + HardwareInterfaceNames.Velocity)).ToArray();
        _posState = _joints.Select(j => FindState(j + "/" + HardwareInterfaceNames.Position)).ToArray();
        _velState = _joints.Select(j => FindState(j + "/" + HardwareInterfaceNames.Velocity)).ToArray();

        // Hold the current position.
        ReadState();
        for (var i = 0; i < _joints.Length; i++)
        {
            _desiredPos[i] = _actualPos[i];
            if (_posCmd[i] >= 0)
            {
                CommandInterfaces[_posCmd[i]].Value = _actualPos[i];
            }

            if (_velCmd[i] >= 0)
            {
                CommandInterfaces[_velCmd[i]].Value = 0;
            }
        }

        return CallbackReturn.Success;
    }

    protected internal override CallbackReturn OnDeactivate()
    {
        Interlocked.Exchange(ref _active, null)?.Done.TrySetResult((FollowJointTrajectoryResult.INVALID_GOAL, "controller deactivated"));
        return CallbackReturn.Success;
    }

    private void ReadState()
    {
        var states = StateInterfaces;
        for (var i = 0; i < _joints.Length; i++)
        {
            _actualPos[i] = states[_posState[i]].Value;
            _actualVel[i] = _velState[i] >= 0 ? states[_velState[i]].Value : 0;
        }
    }

    internal ActiveGoal Submit(JointTrajectory trajectory, JointTolerance[]? goalTolerances, JointTolerance[]? pathTolerances, double goalTime)
    {
        double[] pos, vel;
        lock (_feedbackGate)
        {
            pos = (double[])_actualPos.Clone();
            vel = (double[])_actualVel.Clone();
        }

        var sampled = SampledTrajectory.FromMessage(trajectory, _joints, pos, vel);
        var goalTol = Override(_goalTol, goalTolerances);
        var pathTol = Override(_pathTol, pathTolerances);
        var goal = new ActiveGoal(sampled, goalTol, pathTol, goalTime) { StartNs = trajectory.Header.Stamp.Sec == 0 && trajectory.Header.Stamp.Nanosec == 0 ? long.MinValue : RosTime.FromMsg(trajectory.Header.Stamp).Nanoseconds };
        _pending.Write(goal);
        return goal;
    }

    private double[] Override(double[] defaults, JointTolerance[]? tolerances)
    {
        var result = (double[])defaults.Clone();
        if (tolerances is null)
        {
            return result;
        }

        foreach (var t in tolerances)
        {
            var i = Array.IndexOf(_joints, t.Name);
            if (i >= 0 && t.Position != 0)
            {
                result[i] = t.Position < 0 ? 0 : t.Position;
            }
        }

        return result;
    }

    private async Task<FollowJointTrajectoryResult> ExecuteAsync(ServerGoalHandle<FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback> handle, CancellationToken ct)
    {
        var g = handle.Goal;
        if (Lifecycle != ControllerLifecycle.Active)
        {
            return new FollowJointTrajectoryResult { ErrorCode = FollowJointTrajectoryResult.INVALID_GOAL, ErrorString = "controller is not active" };
        }

        ActiveGoal goal;
        try
        {
            var goalTime = g.GoalTimeTolerance.Sec != 0 || g.GoalTimeTolerance.Nanosec != 0 ? RosDuration.FromMsg(g.GoalTimeTolerance).TotalSeconds : _goalTime;
            goal = Submit(g.Trajectory, g.GoalTolerance, g.PathTolerance, goalTime);
        }
        catch (ArgumentException ex)
        {
            return new FollowJointTrajectoryResult { ErrorCode = FollowJointTrajectoryResult.INVALID_JOINTS, ErrorString = ex.Message };
        }

        while (!goal.Done.Task.IsCompleted)
        {
            if (ct.IsCancellationRequested)
            {
                // Stop where we are: replace the trajectory with a hold.
                HoldPosition();
                goal.Done.TrySetResult((FollowJointTrajectoryResult.SUCCESSFUL, "canceled"));
                break;
            }

            handle.PublishFeedback(BuildFeedback());
            await Task.WhenAny(goal.Done.Task, Task.Delay(50, CancellationToken.None)).ConfigureAwait(false);
        }

        var (code, message) = await goal.Done.Task.ConfigureAwait(false);
        if (code != FollowJointTrajectoryResult.SUCCESSFUL && !ct.IsCancellationRequested)
        {
            handle.Abort();
        }

        return new FollowJointTrajectoryResult { ErrorCode = code, ErrorString = message };
    }

    private void HoldPosition()
    {
        double[] pos;
        lock (_feedbackGate)
        {
            pos = (double[])_actualPos.Clone();
        }

        var traj = new SampledTrajectory([0.0], [pos], null, null);
        _pending.Write(new ActiveGoal(traj, new double[_joints.Length], new double[_joints.Length], 0));
    }

    private FollowJointTrajectoryFeedback BuildFeedback()
    {
        var fb = new FollowJointTrajectoryFeedback { JointNames = _joints };
        fb.Header.Stamp = RosTime.Now().ToMsg();
        lock (_feedbackGate)
        {
            fb.Desired.Positions = (double[])_desiredPos.Clone();
            fb.Desired.Velocities = (double[])_desiredVel.Clone();
            fb.Actual.Positions = (double[])_actualPos.Clone();
            fb.Actual.Velocities = (double[])_actualVel.Clone();
            fb.Error.Positions = _desiredPos.Zip(_actualPos, (d, a) => d - a).ToArray();
            fb.Error.Velocities = _desiredVel.Zip(_actualVel, (d, a) => d - a).ToArray();
        }

        return fb;
    }

    public override ControllerReturn Update(RosTime time, RosDuration period)
    {
        lock (_feedbackGate)
        {
            ReadState();

            if (_pending.Take() is { } incoming)
            {
                var previous = _active;
                _active = incoming;
                _startNs = incoming.StartNs == long.MinValue || incoming.StartNs < time.Nanoseconds - 1_000_000_000_000 ? time.Nanoseconds : incoming.StartNs;
                previous?.Done.TrySetResult((FollowJointTrajectoryResult.INVALID_GOAL, "preempted by a new goal"));
            }

            var goal = _active;
            if (goal is null)
            {
                return ControllerReturn.Ok;
            }

            var t = (time.Nanoseconds - _startNs) / 1e9;
            if (t < 0)
            {
                return ControllerReturn.Ok; // Scheduled in the future.
            }

            goal.Trajectory.Sample(t, _desiredPos, _desiredVel, _desiredAcc);
            var commands = CommandInterfaces;
            for (var i = 0; i < _joints.Length; i++)
            {
                if (_posCmd[i] >= 0)
                {
                    commands[_posCmd[i]].Value = _desiredPos[i];
                }

                if (_velCmd[i] >= 0)
                {
                    commands[_velCmd[i]].Value = _desiredVel[i] + (_gains[i] * (_desiredPos[i] - _actualPos[i]));
                }
            }

            var duration = goal.Trajectory.Duration;
            if (t < duration)
            {
                for (var i = 0; i < _joints.Length; i++)
                {
                    if (goal.PathTolerance[i] > 0 && Math.Abs(_desiredPos[i] - _actualPos[i]) > goal.PathTolerance[i])
                    {
                        Finish(goal, FollowJointTrajectoryResult.PATH_TOLERANCE_VIOLATED, $"joint '{_joints[i]}' left its path tolerance");
                        return ControllerReturn.Ok;
                    }
                }

                return ControllerReturn.Ok;
            }

            var withinGoal = true;
            for (var i = 0; i < _joints.Length; i++)
            {
                var tol = goal.GoalTolerance[i] > 0 ? goal.GoalTolerance[i] : 0.01;
                if (Math.Abs(goal.Trajectory.FinalPositions[i] - _actualPos[i]) > tol || (_velState[i] >= 0 && Math.Abs(_actualVel[i]) > _stoppedVel))
                {
                    withinGoal = false;
                    break;
                }
            }

            if (withinGoal)
            {
                Finish(goal, FollowJointTrajectoryResult.SUCCESSFUL, string.Empty);
            }
            else if (t > duration + Math.Max(goal.GoalTime, 0) + (goal.GoalTime <= 0 ? 1.0 : 0))
            {
                Finish(goal, FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED, "goal tolerance not reached in time");
            }

            return ControllerReturn.Ok;
        }
    }

    private void Finish(ActiveGoal goal, int code, string message)
    {
        // Keep commanding the final point; only the goal bookkeeping ends.
        goal.Done.TrySetResult((code, message));
        if (code != FollowJointTrajectoryResult.SUCCESSFUL)
        {
            _active = null;
            for (var i = 0; i < _joints.Length; i++)
            {
                _desiredPos[i] = _actualPos[i];
                if (_posCmd[i] >= 0)
                {
                    CommandInterfaces[_posCmd[i]].Value = _actualPos[i];
                }

                if (_velCmd[i] >= 0)
                {
                    CommandInterfaces[_velCmd[i]].Value = 0;
                }
            }
        }
    }

    /// <summary>Executes a trajectory directly (without the action) and awaits its outcome.</summary>
    public async Task<(int Code, string Message)> ExecuteAsync(JointTrajectory trajectory, CancellationToken cancellationToken = default)
    {
        var goal = Submit(trajectory, null, null, _goalTime);
        return await goal.Done.Task.WaitAsync(cancellationToken).ConfigureAwait(false);
    }
}
