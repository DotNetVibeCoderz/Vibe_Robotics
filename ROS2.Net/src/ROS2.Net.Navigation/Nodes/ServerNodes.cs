using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.Nav2Msgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Navigation.Control;
using ROS2.Net.Navigation.Costmaps;
using ROS2.Net.Navigation.Maps;
using ROS2.Net.Navigation.Planning;
using NavPath = ROS2.Net.Msgs.NavMsgs.Path;

namespace ROS2.Net.Navigation.Nodes;

/// <summary>
/// Serves a static map like <c>nav2_map_server</c>: latched <c>/map</c>, <c>/map_metadata</c> and
/// the <c>~/map</c> (<c>nav_msgs/srv/GetMap</c>) service.
/// </summary>
public sealed class MapServerNode : Node
{
    private readonly Publisher<OccupancyGrid> _map;
    private readonly Publisher<MapMetaData> _metadata;
    private OccupancyGridMap _current;

    public MapServerNode(Ros2Context context, OccupancyGridMap map, string frameId = "map", string name = "map_server", NodeOptions? options = null)
        : base(context, name, options)
    {
        FrameId = DeclareParameter("frame_id", frameId);
        _current = map;
        _map = CreatePublisher<OccupancyGrid>("/map", QosProfile.Latched);
        _metadata = CreatePublisher<MapMetaData>("/map_metadata", QosProfile.Latched);
        CreateService<GetMapRequest, GetMapResponse>("~/map", _ => new GetMapResponse { Map = _current.ToMsg(FrameId, Clock.Now()) });
        Publish();
    }

    public string FrameId { get; }

    public OccupancyGridMap Map => _current;

    /// <summary>Loads a map_server YAML file.</summary>
    public static MapServerNode FromFile(Ros2Context context, string yamlPath, string frameId = "map") =>
        new(context, OccupancyGridMap.LoadMapServer(yamlPath), frameId);

    /// <summary>Replaces and republishes the map.</summary>
    public void SetMap(OccupancyGridMap map)
    {
        _current = map;
        Publish();
    }

    private void Publish()
    {
        var msg = _current.ToMsg(FrameId, Clock.Now());
        _map.Publish(msg);
        _metadata.Publish(msg.Info);
    }
}

/// <summary>Options shared by the planner and controller servers' costmaps.</summary>
public sealed record CostmapOptions
{
    public double InflationRadius { get; init; } = 0.55;

    public double CostScalingFactor { get; init; } = 3.0;

    public double RobotRadius { get; init; } = 0.22;

    public bool TrackUnknown { get; init; } = true;

    /// <summary>Topic of the static map; the costmap is (re)built when a map arrives.</summary>
    public string MapTopic { get; init; } = "/map";
}

/// <summary>
/// The Nav2 planner server: serves <c>compute_path_to_pose</c>
/// (<c>nav2_msgs/action/ComputePathToPose</c>) with pluggable planners and publishes <c>/plan</c>.
/// Its global costmap follows <c>/map</c> unless one is supplied.
/// </summary>
public sealed class PlannerServerNode : Node
{
    private readonly IRobotPoseProvider _pose;
    private readonly Publisher<NavPath> _plan;
    private readonly CostmapOptions _costmapOptions;
    private readonly Dictionary<string, IGlobalPlanner> _planners = new(StringComparer.Ordinal);
    private Costmap2D? _costmap;

    public PlannerServerNode(Ros2Context context, IRobotPoseProvider pose, Costmap2D? costmap = null, CostmapOptions? costmapOptions = null, string name = "planner_server", NodeOptions? options = null)
        : base(context, name, options)
    {
        _pose = pose;
        _costmap = costmap;
        _costmapOptions = costmapOptions ?? new CostmapOptions();
        FrameId = DeclareParameter("global_frame", "map");
        DefaultPlannerId = DeclareParameter("default_planner_id", "GridBased");
        _planners[DefaultPlannerId] = new AStarPlanner();
        _plan = CreatePublisher<NavPath>("/plan");
        if (costmap is null)
        {
            CreateSubscription<OccupancyGrid>(_costmapOptions.MapTopic, msg => SetMap(OccupancyGridMap.FromMsg(msg)), QosProfile.Latched);
        }

        CreateActionServer<ComputePathToPoseGoal, ComputePathToPoseResult, ComputePathToPoseFeedback>("compute_path_to_pose", ExecuteAsync);
    }

    public string FrameId { get; }

    public string DefaultPlannerId { get; }

    public Costmap2D? Costmap => _costmap;

    /// <summary>Registers (or replaces) a planner under an id, e.g. <c>"SmacHybrid"</c>.</summary>
    public void AddPlanner(string id, IGlobalPlanner planner) => _planners[id] = planner;

    /// <summary>Rebuilds the global costmap from a map.</summary>
    public void SetMap(OccupancyGridMap map) => _costmap = BuildCostmap(map, _costmapOptions);

    internal static Costmap2D BuildCostmap(OccupancyGridMap map, CostmapOptions o)
    {
        var c = Costmap2D.FromMap(map, o.TrackUnknown)
            .AddLayer(new StaticLayer(map, o.TrackUnknown))
            .AddLayer(new ObstacleLayer())
            .AddLayer(new InflationLayer(o.InflationRadius, o.CostScalingFactor, o.RobotRadius));
        c.Update();
        return c;
    }

    private Task<ComputePathToPoseResult> ExecuteAsync(ServerGoalHandle<ComputePathToPoseGoal, ComputePathToPoseResult, ComputePathToPoseFeedback> goal, CancellationToken ct)
    {
        var started = System.Diagnostics.Stopwatch.GetTimestamp();
        var result = new ComputePathToPoseResult();
        var costmap = _costmap;
        if (costmap is null)
        {
            Logger.LogWarning("No costmap yet; cannot plan.");
            goal.Abort();
            return Task.FromResult(result);
        }

        Pose2D start;
        if (goal.Goal.UseStart)
        {
            start = Pose2D.FromMsg(goal.Goal.Start.Pose);
        }
        else if (!_pose.TryGetPose(out start, out _))
        {
            Logger.LogWarning("Robot pose unavailable; cannot plan.");
            goal.Abort();
            return Task.FromResult(result);
        }

        var id = string.IsNullOrEmpty(goal.Goal.PlannerId) ? DefaultPlannerId : goal.Goal.PlannerId;
        if (!_planners.TryGetValue(id, out var planner))
        {
            Logger.LogWarning("Unknown planner id '{Planner}'.", id);
            goal.Abort();
            return Task.FromResult(result);
        }

        var plan = planner.CreatePlan(costmap, start, Pose2D.FromMsg(goal.Goal.Goal.Pose), ct);
        if (!plan.Success)
        {
            Logger.LogWarning("Planner {Planner} failed: {Error}", id, plan.Error);
            goal.Abort();
            return Task.FromResult(result);
        }

        result.Path = PathUtils.ToMsg(plan.Poses, FrameId, Clock.Now());
        result.PlanningTime = RosDuration.FromTimeSpan(System.Diagnostics.Stopwatch.GetElapsedTime(started)).ToMsg();
        _plan.Publish(result.Path);
        return Task.FromResult(result);
    }
}

/// <summary>
/// The Nav2 controller server: serves <c>follow_path</c> (<c>nav2_msgs/action/FollowPath</c>),
/// running the selected controller at <c>controller_frequency</c> and publishing <c>/cmd_vel</c>.
/// A new goal preempts the running one, as in Nav2.
/// </summary>
public sealed class ControllerServerNode : Node
{
    private readonly IRobotPoseProvider _pose;
    private readonly Publisher<Twist> _cmdVel;
    private readonly Dictionary<string, IController> _controllers = new(StringComparer.Ordinal);
    private readonly object _gate = new();
    private CancellationTokenSource? _active;

    public ControllerServerNode(Ros2Context context, IRobotPoseProvider pose, Costmap2D? costmap = null, string name = "controller_server", NodeOptions? options = null)
        : base(context, name, options)
    {
        _pose = pose;
        Costmap = costmap;
        ControllerFrequency = DeclareParameter("controller_frequency", 20.0);
        DefaultControllerId = DeclareParameter("default_controller_id", "FollowPath");
        _controllers[DefaultControllerId] = new RegulatedPurePursuitController();
        GoalChecker = new GoalChecker(
            DeclareParameter("goal_checker.xy_goal_tolerance", 0.25),
            DeclareParameter("goal_checker.yaw_goal_tolerance", 0.25));
        ProgressChecker = new ProgressChecker(
            DeclareParameter("progress_checker.required_movement_radius", 0.5),
            DeclareParameter("progress_checker.movement_time_allowance", 10.0));
        _cmdVel = CreatePublisher<Twist>("/cmd_vel");
        CreateActionServer<FollowPathGoal, FollowPathResult, FollowPathFeedback>("follow_path", ExecuteAsync);
    }

    public double ControllerFrequency { get; }

    public string DefaultControllerId { get; }

    /// <summary>Local costmap used by controllers for obstacle scoring (optional).</summary>
    public Costmap2D? Costmap { get; set; }

    public GoalChecker GoalChecker { get; }

    public ProgressChecker ProgressChecker { get; }

    public void AddController(string id, IController controller) => _controllers[id] = controller;

    public IController GetController(string id) => _controllers[id];

    private async Task<FollowPathResult> ExecuteAsync(ServerGoalHandle<FollowPathGoal, FollowPathResult, FollowPathFeedback> goal, CancellationToken ct)
    {
        var result = new FollowPathResult();
        var path = PathUtils.FromMsg(goal.Goal.Path);
        var id = string.IsNullOrEmpty(goal.Goal.ControllerId) ? DefaultControllerId : goal.Goal.ControllerId;
        if (path.Count == 0 || !_controllers.TryGetValue(id, out var controller))
        {
            goal.Abort();
            return result;
        }

        // Preempt the previous goal.
        var mine = CancellationTokenSource.CreateLinkedTokenSource(ct);
        lock (_gate)
        {
            _active?.Cancel();
            _active = mine;
        }

        controller.SetPlan(path);
        GoalChecker.Reset();
        ProgressChecker.Reset();
        var target = path[^1];
        var period = TimeSpan.FromSeconds(1.0 / ControllerFrequency);
        var feedback = new FollowPathFeedback();
        try
        {
            while (true)
            {
                if (mine.IsCancellationRequested)
                {
                    if (ct.IsCancellationRequested)
                    {
                        goal.Canceled();
                    }
                    else
                    {
                        goal.Abort();
                    }

                    return result;
                }

                if (_pose.TryGetPose(out var pose, out var velocity))
                {
                    if (GoalChecker.IsGoalReached(pose, target))
                    {
                        return result;
                    }

                    if (!ProgressChecker.Check(pose, Clock.Now().Seconds))
                    {
                        Logger.LogWarning("Failed to make progress.");
                        goal.Abort();
                        return result;
                    }

                    var cmd = controller.ComputeVelocity(pose, velocity, Costmap);
                    _cmdVel.Publish(cmd.ToTwist());
                    feedback.DistanceToGoal = (float)pose.DistanceTo(target);
                    feedback.Speed = (float)Math.Abs(velocity.Linear);
                    goal.PublishFeedback(feedback);
                }

                try
                {
                    await Task.Delay(period, mine.Token).ConfigureAwait(false);
                }
                catch (OperationCanceledException)
                {
                }
            }
        }
        finally
        {
            _cmdVel.Publish(new Twist());
            lock (_gate)
            {
                if (_active == mine)
                {
                    _active = null;
                }
            }

            mine.Dispose();
        }
    }
}
