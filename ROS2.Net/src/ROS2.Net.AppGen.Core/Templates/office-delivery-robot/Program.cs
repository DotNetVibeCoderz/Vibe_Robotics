// {{ProjectName}} — office delivery robot with a Nav2-style stack (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.Nav2Msgs;
using ROS2.Net.Navigation.Control;
using ROS2.Net.Navigation.Nodes;
using ROS2.Net.Simulation;
using NavPose = ROS2.Net.Navigation.Pose2D;

await using var context = await AppTarget.CreateContextAsync(args);
var defaultScenario = ScenarioKind.Office;

// Nav2-style stack. Simulator: ground-truth poses and the simulator's /map. Physical robot: poses from
// the robot's odometry (add AMCL/SLAM corrections for real deployments) and the robot's map server on /map.
using var sim = AppTarget.StartSimulator(context, defaultScenario, robots: 1, new SimulationNodeOptions { PublishMapToOdom = true });
Console.WriteLine(AppTarget.Describe());
var navTools = context.CreateNode("nav_tools");
IRobotPoseProvider pose = AppTarget.World is { } simWorld
    ? new DelegatePoseProvider(() =>
    {
        var r = simWorld.Robots[0];
        return (new NavPose(r.Pose.X, r.Pose.Y, r.Pose.Theta), new Velocity2D(r.LinearVelocity, r.AngularVelocity));
    })
    : new OdometryPoseProvider(navTools, AppTarget.Topic("odom"));
var planner = new PlannerServerNode(context, pose);

// The controller publishes /cmd_vel: remap it into the robot's namespace.
var controller = new ControllerServerNode(context, pose, options: new NodeOptions().WithRemap("/cmd_vel", AppTarget.Topic("cmd_vel")));
var navigator = new BtNavigatorNode(context, pose);
double Odometer() => AppTarget.World?.Robots[0].Odometer ?? 0;


// Destinations: the simulator's office rooms, or your building's coordinates (in the map frame).
var rooms = AppTarget.World?.Waypoints.ToDictionary(w => w.Key, w => (w.Value.X, w.Value.Y, w.Value.Theta))
    ?? new Dictionary<string, (double X, double Y, double Theta)> { ["room_ne"] = (12, 9.5, 1.57), ["room_sw"] = (4, 2.5, 0) };
var dispatcher = context.CreateNode("dispatcher");
var nav = dispatcher.CreateActionClient<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback>("/navigate_to_pose");
using var cts = CancellationTokenSource.CreateLinkedTokenSource(context.ShutdownToken);
var spin = context.SpinAsync(cts.Token, new MultiThreadedExecutor());

await nav.WaitForServerAsync(TimeSpan.FromSeconds(10));
var route = new[] { "room_ne", "room_sw", "room_se", "corridor_end" }.Where(rooms.ContainsKey);
foreach (var stop in route)
{
    var wp = rooms[stop];
    Console.WriteLine($"Delivering to {stop}...");
    await Nav.GoToAsync(nav, stop, wp.X, wp.Y, wp.Theta, cts.Token);
}

Console.WriteLine($"All deliveries done ({Odometer():0.0} m driven in simulation).");
cts.Cancel();
await spin;

static class Nav
{
    /// <summary>Sends a NavigateToPose goal and waits for the result.</summary>
    public static async Task<bool> GoToAsync(ActionClient<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback> client, string name, double x, double y, double yaw, CancellationToken ct)
    {
        var goal = new NavigateToPoseGoal();
        goal.Pose.Header.FrameId = "map";
        goal.Pose.Pose.Position.X = x;
        goal.Pose.Pose.Position.Y = y;
        goal.Pose.Pose.Orientation = MessageExtensions.QuaternionFromYaw(yaw);
        var lastReport = DateTime.MinValue;
        var result = await client.SendGoalAndWaitAsync(goal, fb =>
        {
            if (DateTime.UtcNow - lastReport > TimeSpan.FromSeconds(2))
            {
                lastReport = DateTime.UtcNow;
                Console.WriteLine($"  → {name}: {fb.DistanceRemaining:0.0} m to go");
            }
        }, ct);
        Console.WriteLine($"  {name}: {result.Status}");
        return result.Succeeded;
    }
}
