// {{ProjectName}} — search & rescue mapping with online SLAM (ROS2.Net template). Lock-step simulation.
// For a real robot use ROS2.Net.Navigation's SlamNode on its /scan + /odom instead.
using ROS2.Net.Navigation.Slam;
using ROS2.Net.Simulation;
using NavPose = ROS2.Net.Navigation.Pose2D;

if (AppTarget.IsPhysicalRobot)
{
    Console.WriteLine("This app is a simulation study (lock-step physics, ground truth). Run it with a Simulator target.");
    return;
}

var world = Scenarios.Create(ScenarioKind.CorridorLoop, robots: 1, sensors: new RobotSensorOptions { LidarRange = 8 });
var robot = world.Robots[0];
var lidar = robot.Lidar!;
var slam = new OnlineSlam(new SlamOptions { Resolution = 0.05, MaxLaserRange = 8 });
slam.LoopClosed += n => Console.WriteLine($"[{world.Time,5:0}s] loop closure #{n}");

(double, double)[] Points()
{
    var pts = new List<(double, double)>(lidar.Beams);
    var ranges = lidar.Ranges;
    for (var i = 0; i < ranges.Length; i++)
    {
        var r = ranges[i];
        if (float.IsFinite(r) && r > lidar.RangeMin && r < lidar.RangeMax)
        {
            var a = lidar.AngleMin + (i * lidar.AngleIncrement);
            pts.Add((r * Math.Cos(a), r * Math.Sin(a)));
        }
    }

    return pts.ToArray();
}

double Sector(double center)
{
    var min = double.PositiveInfinity;
    for (var i = 0; i < lidar.Ranges.Length; i++)
    {
        var a = lidar.AngleMin + (i * lidar.AngleIncrement);
        if (Math.Abs(Math.Atan2(Math.Sin(a - center), Math.Cos(a - center))) < 0.3 && float.IsFinite(lidar.Ranges[i]))
        {
            min = Math.Min(min, lidar.Ranges[i]);
        }
    }

    return min;
}

while (world.Time < 240)
{
    // Left-wall following around the loop.
    var front = Sector(0);
    var left = Sector(Math.PI / 2);
    var (v, w) = front < 0.7 ? (0.05, -1.0) : (0.35, Math.Clamp((left - 0.9) * 1.5, -0.8, 0.8));
    robot.SetCommand(v, 0, w, world.Time);
    world.StepFor(0.1);

    var odom = robot.OdomPose;
    slam.Process(new NavPose(odom.X, odom.Y, odom.Theta), Points());
}

var map = slam.GetMap();
var path = Path.Combine(Environment.CurrentDirectory, "rescue_map");
slam.SaveMap(path);
var truth = robot.Pose;
var estimate = slam.Pose;
Console.WriteLine($"Mapped {map.WorldWidth:0.0} x {map.WorldHeight:0.0} m with {slam.Keyframes.Count} keyframes and {slam.LoopClosureCount} loop closures.");
Console.WriteLine($"Pose error: SLAM {Math.Sqrt(Math.Pow(estimate.X - truth.X, 2) + Math.Pow(estimate.Y - truth.Y, 2)):0.00} m vs odometry {Math.Sqrt(Math.Pow(robot.OdomPose.X - truth.X, 2) + Math.Pow(robot.OdomPose.Y - truth.Y, 2)):0.00} m");
Console.WriteLine($"Map saved to {path}.yaml/.pgm");
