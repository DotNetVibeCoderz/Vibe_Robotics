using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.Nav2Msgs;
using ROS2.Net.Navigation.Behaviors;
using ROS2.Net.Navigation.BehaviorTrees;
using ROS2.Net.Navigation.Control;
using NavPath = ROS2.Net.Msgs.NavMsgs.Path;

namespace ROS2.Net.Navigation.Nodes;

/// <summary>
/// The Nav2 BT navigator: serves <c>navigate_to_pose</c> (<c>nav2_msgs/action/NavigateToPose</c>)
/// by ticking a behavior tree whose leaves call the planner server
/// (<c>compute_path_to_pose</c>) and controller server (<c>follow_path</c>) actions — so it works with
/// ROS2.Net's servers and with stock Nav2 ones. Recoveries (Spin, Wait, BackUp) run locally on
/// <c>/cmd_vel</c>; ClearEntireCostmap invokes <see cref="ClearCostmaps"/>.
/// </summary>
public sealed class BtNavigatorNode : Node
{
    private readonly IRobotPoseProvider _pose;
    private readonly ActionClient<ComputePathToPoseGoal, ComputePathToPoseResult, ComputePathToPoseFeedback> _planner;
    private readonly ActionClient<FollowPathGoal, FollowPathResult, FollowPathFeedback> _controller;
    private readonly Publisher<Twist> _cmdVel;
    private readonly SemaphoreSlim _single = new(1, 1);
    private int _recoveries;

    public BtNavigatorNode(Ros2Context context, IRobotPoseProvider pose, string? behaviorTreeXml = null, string name = "bt_navigator", NodeOptions? options = null)
        : base(context, name, options)
    {
        _pose = pose;
        BehaviorTreeXml = behaviorTreeXml ?? NavTrees.NavigateToPoseWithReplanningAndRecovery;
        GlobalFrame = DeclareParameter("global_frame", "map");
        TickPeriod = TimeSpan.FromMilliseconds(DeclareParameter("bt_loop_duration", 10L) * 10);
        _planner = CreateActionClient<ComputePathToPoseGoal, ComputePathToPoseResult, ComputePathToPoseFeedback>("compute_path_to_pose");
        _controller = CreateActionClient<FollowPathGoal, FollowPathResult, FollowPathFeedback>("follow_path");
        _cmdVel = CreatePublisher<Twist>("/cmd_vel");
        Factory = CreateFactory();
        CreateActionServer<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback>("navigate_to_pose", ExecuteAsync);
    }

    /// <summary>The default tree, used when a goal does not name one.</summary>
    public string BehaviorTreeXml { get; set; }

    public string GlobalFrame { get; }

    public TimeSpan TickPeriod { get; }

    /// <summary>The node factory: register additional BT nodes here before goals arrive.</summary>
    public BtFactory Factory { get; }

    /// <summary>Called by ClearEntireCostmap nodes with the node's <c>service_name</c> port.</summary>
    public Action<string>? ClearCostmaps { get; set; }

    private async Task<NavigateToPoseResult> ExecuteAsync(ServerGoalHandle<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback> goal, CancellationToken ct)
    {
        await _single.WaitAsync(ct).ConfigureAwait(false);
        try
        {
            var xml = string.IsNullOrWhiteSpace(goal.Goal.BehaviorTree) ? BehaviorTreeXml
                : File.Exists(goal.Goal.BehaviorTree) ? await File.ReadAllTextAsync(goal.Goal.BehaviorTree, ct).ConfigureAwait(false)
                : goal.Goal.BehaviorTree;
            var tree = Factory.CreateTreeFromText(xml);
            tree.Blackboard.Set("goal", goal.Goal.Pose);
            _recoveries = 0;
            var start = Clock.Now();
            var target = Pose2D.FromMsg(goal.Goal.Pose.Pose);
            var feedback = new NavigateToPoseFeedback();
            var status = await tree.TickWhileRunningAsync(TickPeriod, ct, () =>
            {
                if (!_pose.TryGetPose(out var pose, out var vel))
                {
                    return;
                }

                feedback.CurrentPose = pose.ToPoseStamped(GlobalFrame, Clock.Now());
                feedback.NavigationTime = (Clock.Now() - start).ToMsg();
                feedback.NumberOfRecoveries = (short)_recoveries;
                feedback.DistanceRemaining = (float)(tree.Blackboard.TryGet<NavPath>("path", out var path) && path.Poses.Length > 0
                    ? RemainingAlong(path, pose)
                    : pose.DistanceTo(target));
                var speed = Math.Max(0.05, Math.Abs(vel.Linear));
                feedback.EstimatedTimeRemaining = RosDuration.FromSeconds(feedback.DistanceRemaining / speed).ToMsg();
                goal.PublishFeedback(feedback);
            }).ConfigureAwait(false);

            if (status != NodeStatus.Success)
            {
                Logger.LogWarning("Navigation failed.");
                goal.Abort();
            }

            return new NavigateToPoseResult();
        }
        finally
        {
            _single.Release();
        }
    }

    private static double RemainingAlong(NavPath path, Pose2D pose)
    {
        var poses = path.Poses;
        var closest = 0;
        var best = double.MaxValue;
        for (var i = 0; i < poses.Length; i++)
        {
            var d = pose.DistanceSquaredTo(Pose2D.FromMsg(poses[i].Pose));
            if (d < best)
            {
                best = d;
                closest = i;
            }
        }

        var total = Math.Sqrt(best);
        for (var i = closest + 1; i < poses.Length; i++)
        {
            total += Pose2D.FromMsg(poses[i - 1].Pose).DistanceTo(Pose2D.FromMsg(poses[i].Pose));
        }

        return total;
    }

    private BtFactory CreateFactory()
    {
        var f = new BtFactory();
        f.RegisterAsyncAction("ComputePathToPose", ComputePathAsync);
        f.Register("FollowPath", c => new FollowPathBtNode(c, this));
        f.RegisterAction("ClearEntireCostmap", n =>
        {
            Interlocked.Increment(ref _recoveries);
            ClearCostmaps?.Invoke(n.GetInput("service_name", string.Empty));
            return NodeStatus.Success;
        });
        f.RegisterCondition("GoalUpdated", _ => false);
        f.RegisterCondition("GoalReached", n =>
            n.Blackboard.TryGet<PoseStamped>("goal", out var g) && _pose.TryGetPose(out var p, out _) &&
            p.DistanceTo(Pose2D.FromMsg(g.Pose)) < n.GetInput("goal_reached_tol", 0.25));
        f.RegisterAsyncAction("Spin", (n, ct) => RunRecoveryAsync(new SpinBehavior(n.GetInput("spin_dist", 1.57)), ct));
        f.RegisterAsyncAction("Wait", (n, ct) => RunRecoveryAsync(new WaitBehavior(n.GetInput("wait_duration", 1.0)), ct));
        f.RegisterAsyncAction("BackUp", (n, ct) => RunRecoveryAsync(new BackUpBehavior(n.GetInput("backup_dist", 0.3), n.GetInput("backup_speed", 0.05)), ct));
        return f;
    }

    private async Task<NodeStatus> ComputePathAsync(BtNode node, CancellationToken ct)
    {
        if (!node.Blackboard.TryGet<PoseStamped>(StripBraces(node.Config.Ports.GetValueOrDefault("goal", "{goal}")), out var target))
        {
            return NodeStatus.Failure;
        }

        try
        {
            var result = await _planner.SendGoalAndWaitAsync(new ComputePathToPoseGoal
            {
                Goal = target,
                PlannerId = node.GetInput("planner_id", string.Empty),
            }, null, ct).ConfigureAwait(false);
            if (!result.Succeeded || result.Result.Path.Poses.Length == 0)
            {
                return NodeStatus.Failure;
            }

            node.SetOutput("path", result.Result.Path);
            return NodeStatus.Success;
        }
        catch (Exception ex) when (ex is GoalRejectedException or ServiceTimeoutException)
        {
            return NodeStatus.Failure;
        }
    }

    private async Task<NodeStatus> RunRecoveryAsync(IRecoveryBehavior behavior, CancellationToken ct)
    {
        Interlocked.Increment(ref _recoveries);
        if (!_pose.TryGetPose(out var pose, out _))
        {
            return NodeStatus.Failure;
        }

        behavior.Start(pose);
        var dt = 0.05;
        try
        {
            while (true)
            {
                _pose.TryGetPose(out pose, out _);
                var status = behavior.Update(pose, dt, out var cmd);
                _cmdVel.Publish(cmd.ToTwist());
                if (status != BehaviorStatus.Running)
                {
                    return status == BehaviorStatus.Succeeded ? NodeStatus.Success : NodeStatus.Failure;
                }

                await Task.Delay(TimeSpan.FromSeconds(dt), ct).ConfigureAwait(false);
            }
        }
        finally
        {
            _cmdVel.Publish(new Twist());
        }
    }

    private static string StripBraces(string s) => s.Length > 2 && s[0] == '{' && s[^1] == '}' ? s[1..^1] : s;

    /// <summary>FollowPath leaf: sends the path to the controller server and re-sends it when replanning updates <c>{path}</c>.</summary>
    private sealed class FollowPathBtNode(BtNodeConfig config, BtNavigatorNode owner) : BtNode(config)
    {
        private ClientGoalHandle<FollowPathGoal, FollowPathResult, FollowPathFeedback>? _handle;
        private Task<ClientGoalHandle<FollowPathGoal, FollowPathResult, FollowPathFeedback>>? _sending;
        private Task<ActionResult<FollowPathResult>>? _result;
        private NavPath? _sentPath;

        protected override NodeStatus OnTick()
        {
            var path = GetInput<NavPath?>("path", null);
            if (path is null)
            {
                return NodeStatus.Failure;
            }

            if (_sending is null || (!ReferenceEquals(path, _sentPath) && _sending.IsCompleted))
            {
                _sentPath = path;
                _result = null;
                _sending = owner._controller.SendGoalAsync(new FollowPathGoal
                {
                    Path = path,
                    ControllerId = GetInput("controller_id", string.Empty),
                });
            }

            if (!_sending.IsCompleted)
            {
                return NodeStatus.Running;
            }

            if (!_sending.IsCompletedSuccessfully)
            {
                Reset();
                return NodeStatus.Failure;
            }

            _handle = _sending.Result;
            _result ??= _handle.GetResultAsync();
            if (!_result.IsCompleted)
            {
                return NodeStatus.Running;
            }

            var ok = _result.IsCompletedSuccessfully && _result.Result.Succeeded && ReferenceEquals(path, _sentPath);
            var aborted = _result.IsCompletedSuccessfully && _result.Result.Status == GoalStatus.Aborted;
            if (!ok && !aborted && !ReferenceEquals(path, _sentPath))
            {
                return NodeStatus.Running; // superseded by a new path; next tick sends it
            }

            Reset();
            return ok ? NodeStatus.Success : NodeStatus.Failure;
        }

        protected override void OnHalt()
        {
            if (_handle is { } h)
            {
                _ = h.CancelAsync();
            }

            Reset();
        }

        private void Reset()
        {
            _handle = null;
            _sending = null;
            _result = null;
            _sentPath = null;
        }
    }
}
