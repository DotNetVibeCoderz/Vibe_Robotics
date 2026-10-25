using ROS2.Net.Msgs.ControlMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.TrajectoryMsgs;

namespace ROS2.Net.Manipulation;

/// <summary>A planned motion: joint waypoints and the timed trajectory to execute.</summary>
public sealed record MotionPlan(bool Success, IReadOnlyList<double[]> Waypoints, JointTrajectory Trajectory, string Message = "");

/// <summary>
/// A MoveIt <c>MoveGroupInterface</c>-like facade for one kinematic chain: plans with RRT-Connect, IK and
/// Cartesian interpolation against a <see cref="PlanningScene"/>, time-parameterizes the result and executes it
/// through a <c>control_msgs/FollowJointTrajectory</c> action (e.g. ROS2.Net's or ros2_control's
/// joint_trajectory_controller). The current state comes from <c>/joint_states</c>.
/// </summary>
public sealed class MoveGroup : IDisposable
{
    private readonly ActionClient<FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback> _execute;
    private readonly Subscription<JointState> _jointStates;
    private readonly double[] _current;
    private volatile bool _haveState;

    public MoveGroup(Node node, KinematicChain chain, PlanningScene? scene = null, string controllerAction = "/joint_trajectory_controller/follow_joint_trajectory")
    {
        Node = node;
        Chain = chain;
        Scene = scene ?? new PlanningScene(chain.Model);
        _current = new double[chain.Dof];
        _execute = node.CreateActionClient<FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback>(controllerAction);
        _jointStates = node.CreateSubscription<JointState>("/joint_states", OnJointState);
        Planner = new RrtConnectPlanner(chain, Scene);
        Cartesian = new CartesianPlanner(chain, Scene);
    }

    public Node Node { get; }

    public KinematicChain Chain { get; }

    public PlanningScene Scene { get; }

    public RrtConnectPlanner Planner { get; set; }

    public CartesianPlanner Cartesian { get; }

    /// <summary>Fraction of the velocity limits used when timing trajectories.</summary>
    public double MaxVelocityScaling { get; set; } = 0.5;

    public double MaxAccelerationScaling { get; set; } = 0.5;

    /// <summary>Acceleration limits per joint (rad/s²); URDF has none, so a default is used.</summary>
    public double[] AccelerationLimits { get; set; } = [];

    public VelocityProfile Profile { get; set; } = VelocityProfile.Trapezoidal;

    /// <summary>True once a joint state covering every chain joint has been received.</summary>
    public bool HasCurrentState => _haveState;

    /// <summary>Latest chain positions from /joint_states (or zeros).</summary>
    public double[] CurrentJointValues
    {
        get
        {
            lock (_current)
            {
                return (double[])_current.Clone();
            }
        }
    }

    /// <summary>Overrides the current state (e.g. offline planning).</summary>
    public void SetCurrentJointValues(double[] q)
    {
        lock (_current)
        {
            Array.Copy(q, _current, _current.Length);
        }

        _haveState = true;
    }

    public Pose3 CurrentPose => Chain.ForwardKinematics(CurrentJointValues);

    private void OnJointState(JointState msg)
    {
        var found = 0;
        lock (_current)
        {
            for (var i = 0; i < Chain.Dof; i++)
            {
                var k = Array.IndexOf(msg.Name, Chain.JointNames[i]);
                if (k >= 0 && k < msg.Position.Length)
                {
                    _current[i] = msg.Position[k];
                    found++;
                }
            }
        }

        if (found == Chain.Dof)
        {
            _haveState = true;
        }
    }

    public MotionPlan PlanToJointTarget(double[] target, double[]? start = null)
    {
        var from = start ?? CurrentJointValues;
        var plan = Planner.Plan(from, target);
        return plan.Success
            ? new MotionPlan(true, plan.Waypoints, Time(plan.Waypoints))
            : new MotionPlan(false, [], new JointTrajectory(), plan.Message);
    }

    /// <summary>Solves IK for a collision-free configuration at <paramref name="pose"/>, then plans to it.</summary>
    public MotionPlan PlanToPose(Pose3 pose, double[]? start = null, bool positionOnly = false)
    {
        var from = start ?? CurrentJointValues;
        var ik = new IkSolver(Chain, new IkOptions
        {
            PositionOnly = positionOnly,
            PositionTolerance = 1e-5,
            OrientationTolerance = 1e-4,
            Validator = q => Scene.IsStateValid(Chain, q),
        });
        var solution = ik.Solve(pose, from);
        return solution.Success
            ? PlanToJointTarget(solution.Solution, from)
            : new MotionPlan(false, [], new JointTrajectory(), $"no collision-free IK solution (position error {solution.PositionError:0.####} m)");
    }

    /// <summary>A straight-line tip motion through <paramref name="poses"/>.</summary>
    public MotionPlan PlanCartesian(IReadOnlyList<Pose3> poses, double[]? start = null)
    {
        var r = Cartesian.Plan(start ?? CurrentJointValues, poses);
        return r.Complete
            ? new MotionPlan(true, r.Waypoints, Time(r.Waypoints))
            : new MotionPlan(false, r.Waypoints, Time(r.Waypoints), $"only {r.Fraction:P0} of the Cartesian path is feasible");
    }

    /// <summary>Times waypoints with the configured profile and scaling.</summary>
    public JointTrajectory Time(IReadOnlyList<double[]> waypoints)
    {
        var acc = AccelerationLimits.Length == Chain.Dof ? AccelerationLimits : Enumerable.Repeat(3.0, Chain.Dof).ToArray();
        return TimeParameterization.Compute(Chain.JointNames, waypoints, Chain.VelocityLimits, acc, Profile, 0.05, MaxVelocityScaling, MaxAccelerationScaling);
    }

    /// <summary>Executes a trajectory on the controller and waits for the outcome.</summary>
    public async Task<bool> ExecuteAsync(JointTrajectory trajectory, CancellationToken cancellationToken = default)
    {
        if (!await _execute.WaitForServerAsync(TimeSpan.FromSeconds(5), cancellationToken).ConfigureAwait(false))
        {
            throw new InvalidOperationException($"Trajectory controller '{_execute.ActionName}' is not available.");
        }

        var result = await _execute.SendGoalAndWaitAsync(new FollowJointTrajectoryGoal { Trajectory = trajectory }, null, cancellationToken).ConfigureAwait(false);
        return result.Succeeded && result.Result.ErrorCode == FollowJointTrajectoryResult.SUCCESSFUL;
    }

    public async Task<bool> MoveToJointTargetAsync(double[] target, CancellationToken cancellationToken = default)
    {
        var plan = PlanToJointTarget(target);
        return plan.Success && await ExecuteAsync(plan.Trajectory, cancellationToken).ConfigureAwait(false);
    }

    public async Task<bool> MoveToPoseAsync(Pose3 pose, CancellationToken cancellationToken = default)
    {
        var plan = PlanToPose(pose);
        return plan.Success && await ExecuteAsync(plan.Trajectory, cancellationToken).ConfigureAwait(false);
    }

    public void Dispose()
    {
        _execute.Dispose();
        _jointStates.Dispose();
    }
}

/// <summary>One step of a pick-and-place sequence.</summary>
public sealed record PickPlaceStep(string Name, MotionPlan? Plan, double? GripperPosition = null);

/// <summary>
/// A pick-and-place pipeline: pre-grasp → approach (Cartesian) → close gripper → attach → retreat (Cartesian) →
/// pre-place → place (Cartesian) → open → detach → retreat. Grasps approach along the tool's +Z axis.
/// </summary>
public sealed class PickAndPlace(MoveGroup group, Func<double, CancellationToken, Task<bool>> gripper)
{
    public MoveGroup Group { get; } = group;

    /// <summary>Distance (m) of the pre-grasp/pre-place pose before the grasp along the approach axis.</summary>
    public double ApproachDistance { get; set; } = 0.10;

    /// <summary>Lift (m) along world +Z after grasping and placing.</summary>
    public double RetreatDistance { get; set; } = 0.10;

    public double OpenPosition { get; set; } = 0.08;

    public double ClosedPosition { get; set; } = 0.0;

    /// <summary>Plans the full sequence without moving. Each motion starts where the previous ended.</summary>
    public IReadOnlyList<PickPlaceStep> Plan(Pose3 graspPose, Pose3 placePose)
    {
        var steps = new List<PickPlaceStep>();
        var q = Group.CurrentJointValues;

        MotionPlan Add(string name, MotionPlan plan)
        {
            steps.Add(new PickPlaceStep(name, plan));
            if (!plan.Success)
            {
                throw new InvalidOperationException($"Pick-and-place step '{name}' failed: {plan.Message}");
            }

            q = plan.Waypoints[^1];
            return plan;
        }

        var approachAxis = graspPose.Rotation.Rotate(Vec3.UnitZ);
        var preGrasp = graspPose with { Position = graspPose.Position - (approachAxis * ApproachDistance) };
        var lift = new Vec3(0, 0, RetreatDistance);
        var placeAxis = placePose.Rotation.Rotate(Vec3.UnitZ);
        var prePlace = placePose with { Position = placePose.Position - (placeAxis * ApproachDistance) };

        steps.Add(new PickPlaceStep("open", null, OpenPosition));
        Add("pre-grasp", Group.PlanToPose(preGrasp, q));
        Add("approach", Group.PlanCartesian([graspPose], q));
        steps.Add(new PickPlaceStep("grasp", null, ClosedPosition));
        Add("retreat", Group.PlanCartesian([graspPose with { Position = graspPose.Position + lift }], q));
        Add("pre-place", Group.PlanToPose(prePlace, q));
        Add("place", Group.PlanCartesian([placePose], q));
        steps.Add(new PickPlaceStep("release", null, OpenPosition));
        Add("post-place", Group.PlanCartesian([placePose with { Position = placePose.Position + lift }], q));
        return steps;
    }

    /// <summary>Plans and executes the sequence; attaches <paramref name="objectId"/> to the tip while carried.</summary>
    public async Task<bool> ExecuteAsync(Pose3 graspPose, Pose3 placePose, string? objectId = null, CancellationToken cancellationToken = default)
    {
        foreach (var step in Plan(graspPose, placePose))
        {
            if (step.GripperPosition is { } g)
            {
                if (!await gripper(g, cancellationToken).ConfigureAwait(false) && g != ClosedPosition)
                {
                    return false;
                }

                if (objectId is not null)
                {
                    var state = Group.Scene.StateFor(Group.Chain, Group.CurrentJointValues);
                    if (step.Name == "grasp")
                    {
                        Group.Scene.Attach(objectId, Group.Chain.TipLink, state);
                    }
                    else if (step.Name == "release")
                    {
                        Group.Scene.Detach(objectId, state);
                    }
                }

                continue;
            }

            if (!await Group.ExecuteAsync(step.Plan!.Trajectory, cancellationToken).ConfigureAwait(false))
            {
                return false;
            }
        }

        return true;
    }
}
