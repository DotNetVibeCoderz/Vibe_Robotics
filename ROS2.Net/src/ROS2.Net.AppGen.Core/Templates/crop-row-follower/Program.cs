// {{ProjectName}} — crop row following (ROS2.Net template). Lock-step simulation.
using ROS2.Net.Simulation;

if (AppTarget.IsPhysicalRobot)
{
    Console.WriteLine("This app is a simulation study (lock-step physics, ground truth). Run it with a Simulator target.");
    return;
}

var world = Scenarios.Create(ScenarioKind.AgricultureRows, robots: 1);
var robot = world.Robots[0];
var lidar = robot.Lidar!;
var starts = world.Waypoints.Where(w => w.Key.EndsWith("_start", StringComparison.Ordinal)).OrderBy(w => w.Key, StringComparer.Ordinal).ToList();
if (starts.Count == 0)
{
    Console.WriteLine("The scenario has no rows.");
    return;
}

double Side(double center)
{
    var sum = 0.0;
    var n = 0;
    for (var i = 0; i < lidar.Ranges.Length; i++)
    {
        var a = lidar.AngleMin + (i * lidar.AngleIncrement);
        if (Math.Abs(Math.Atan2(Math.Sin(a - center), Math.Cos(a - center))) < 0.4 && float.IsFinite(lidar.Ranges[i]) && lidar.Ranges[i] < 2.0)
        {
            sum += lidar.Ranges[i];
            n++;
        }
    }

    return n == 0 ? 2.0 : sum / n;
}

var rowsDone = 0;
foreach (var (name, start) in starts.Take(4))
{
    robot.Teleport(start);
    var endName = name.Replace("_start", "_end", StringComparison.Ordinal);
    var end = world.Waypoints[endName];
    var t0 = world.Time;
    while (world.Time - t0 < 120)
    {
        var dx = end.X - robot.Pose.X;
        var dy = end.Y - robot.Pose.Y;
        if (Math.Sqrt((dx * dx) + (dy * dy)) < 0.5)
        {
            rowsDone++;
            break;
        }

        // Steer toward the end while balancing the distance to the plants on both sides.
        var heading = Math.Atan2(dy, dx);
        var headingError = Math.Atan2(Math.Sin(heading - robot.Pose.Theta), Math.Cos(heading - robot.Pose.Theta));
        var balance = Side(Math.PI / 2) - Side(-Math.PI / 2);
        robot.SetCommand(0.4, 0, (1.5 * headingError) + (0.6 * balance), world.Time);
        world.StepFor(0.05);
    }

    Console.WriteLine($"{name.Replace("_start", string.Empty, StringComparison.Ordinal)}: {(world.Time - t0):0.0} s, collisions so far {robot.CollisionCount}");
}

Console.WriteLine($"Finished {rowsDone} rows, {robot.Odometer:0} m driven.");
