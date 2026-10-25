// {{ProjectName}} — cleaning-robot coverage planner (ROS2.Net template).
using ROS2.Net.Navigation.Control;
using ROS2.Net.Navigation.Costmaps;
using ROS2.Net.Navigation.Maps;
using ROS2.Net.Navigation.Planning;
using ROS2.Net.Simulation;
using NavPose = ROS2.Net.Navigation.Pose2D;

if (AppTarget.IsPhysicalRobot)
{
    Console.WriteLine("This app is a simulation study (lock-step physics, ground truth). Run it with a Simulator target.");
    return;
}

var world = Scenarios.Create(AppTarget.Scenario(ScenarioKind.Office), robots: 1);
var robot = world.Robots[0];
var map = OccupancyGridMap.FromMsg(world.ToOccupancyGrid(0.05));
var costmap = Costmap2D.FromMap(map)
    .AddLayer(new StaticLayer(map))
    .AddLayer(new InflationLayer(inflationRadius: 0.45, costScalingFactor: 4, inscribedRadius: 0.18));
costmap.Update();

// Boustrophedon lanes every 0.6 m over cells that are well clear of obstacles.
var lanes = new List<NavPose>();
var lane = 0;
for (var y = 0.6; y < map.WorldHeight - 0.4; y += 0.6, lane++)
{
    var row = new List<NavPose>();
    for (var x = 0.4; x < map.WorldWidth - 0.4; x += 0.3)
    {
        var wx = map.Origin.X + x;
        var wy = map.Origin.Y + y;
        if (costmap.GetCostAt(wx, wy) < 50)
        {
            row.Add(new NavPose(wx, wy, lane % 2 == 0 ? 0 : Math.PI));
        }
    }

    if (lane % 2 == 1)
    {
        row.Reverse();
    }

    lanes.AddRange(row.Where((_, i) => i % 4 == 0));
}

Console.WriteLine($"Coverage route: {lanes.Count} waypoints over {map.WorldWidth:0.0} x {map.WorldHeight:0.0} m");

var planner = new AStarPlanner();
var controller = new RegulatedPurePursuitController();
var goalChecker = new GoalChecker(0.25, Math.PI);
var visited = 0;
var totalPath = 0.0;
foreach (var waypoint in lanes)
{
    var start = new NavPose(robot.Pose.X, robot.Pose.Y, robot.Pose.Theta);
    if (start.DistanceTo(waypoint) < 0.3)
    {
        continue;
    }

    var plan = planner.CreatePlan(costmap, start, waypoint);
    if (!plan.Success)
    {
        continue;
    }

    totalPath += plan.Length;
    controller.SetPlan(plan.Poses);
    goalChecker.Reset();
    var deadline = world.Time + 60;
    while (world.Time < deadline)
    {
        var pose = new NavPose(robot.Pose.X, robot.Pose.Y, robot.Pose.Theta);
        if (goalChecker.IsGoalReached(pose, waypoint))
        {
            visited++;
            break;
        }

        var cmd = controller.ComputeVelocity(pose, new Velocity2D(robot.LinearVelocity, robot.AngularVelocity), costmap);
        robot.SetCommand(cmd.Linear, 0, cmd.Angular, world.Time);
        world.StepFor(0.05);
    }
}

robot.Stop();
Console.WriteLine($"Visited {visited}/{lanes.Count} waypoints, planned {totalPath:0.0} m, drove {robot.Odometer:0.0} m in {world.Time / 60:0.0} simulated minutes (collisions: {robot.CollisionCount}).");
