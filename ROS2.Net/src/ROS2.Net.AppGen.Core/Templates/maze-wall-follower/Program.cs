// {{ProjectName}} — maze escape with the right-hand wall-following rule (ROS2.Net template).
// The world is stepped in lock-step (no real-time wait), so the run completes in seconds.
using ROS2.Net.Simulation;

if (AppTarget.IsPhysicalRobot)
{
    Console.WriteLine("This app is a simulation study (lock-step physics, ground truth). Run it with a Simulator target.");
    return;
}

var world = Scenarios.Create(ScenarioKind.Maze, robots: 1, seed: 7);
var robot = world.Robots[0];
var exit = world.Waypoints["exit"];
var lidar = robot.Lidar ?? throw new InvalidOperationException("The robot has no LiDAR.");
const double Wall = 0.55;

double Sector(double center, double halfWidth)
{
    var min = double.PositiveInfinity;
    var ranges = lidar.Ranges;
    for (var i = 0; i < ranges.Length; i++)
    {
        var a = lidar.AngleMin + (i * lidar.AngleIncrement);
        var d = Math.Atan2(Math.Sin(a - center), Math.Cos(a - center));
        if (Math.Abs(d) <= halfWidth && float.IsFinite(ranges[i]))
        {
            min = Math.Min(min, ranges[i]);
        }
    }

    return min;
}

var maxTime = 600.0;
while (world.Time < maxTime)
{
    var front = Sector(0, 0.35);
    var right = Sector(-Math.PI / 2, 0.35);
    double v, w;
    if (front < Wall)
    {
        (v, w) = (0.0, 1.2);                   // wall ahead: turn left
    }
    else if (right > Wall * 1.8)
    {
        (v, w) = (0.18, -1.0);                 // lost the wall: turn right to find it
    }
    else
    {
        (v, w) = (0.25, (Wall - right) * 2.0); // follow the wall at a constant distance
    }

    robot.SetCommand(v, 0, w, world.Time);
    world.StepFor(0.1);

    var dx = robot.Pose.X - exit.X;
    var dy = robot.Pose.Y - exit.Y;
    if (Math.Sqrt((dx * dx) + (dy * dy)) < 0.6)
    {
        Console.WriteLine($"Escaped the maze in {world.Time:0.0} s of simulated time after {robot.Odometer:0.0} m (collisions: {robot.CollisionCount}).");
        return;
    }

    if (world.StepCount % 3000 == 0)
    {
        Console.WriteLine($"t={world.Time:0}s at ({robot.Pose.X:0.0}, {robot.Pose.Y:0.0}), {Math.Sqrt((dx * dx) + (dy * dy)):0.0} m from the exit");
    }
}

Console.WriteLine("Time budget exhausted before reaching the exit.");
