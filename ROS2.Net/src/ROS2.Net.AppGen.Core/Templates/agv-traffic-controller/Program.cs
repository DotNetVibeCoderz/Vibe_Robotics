// {{ProjectName}} — AGV traffic control (ROS2.Net template). Lock-step simulation: a 5-minute shift runs in seconds.
using ROS2.Net.Simulation;

if (AppTarget.IsPhysicalRobot)
{
    Console.WriteLine("This app is a simulation study (lock-step physics, ground truth). Run it with a Simulator target.");
    return;
}

var world = Scenarios.Create(ScenarioKind.Warehouse, robots: 4);
var stops = world.Waypoints.Where(w => w.Key.StartsWith("shelf_", StringComparison.Ordinal)).Select(w => w.Value).ToArray();
var rnd = new Random(3);
var targets = world.Robots.ToDictionary(r => r.Name, _ => stops[rnd.Next(stops.Length)]);
var delivered = world.Robots.ToDictionary(r => r.Name, _ => 0);
var yields = 0;

while (world.Time < 300)
{
    foreach (var r in world.Robots)
    {
        var goal = targets[r.Name];
        var dx = goal.X - r.Pose.X;
        var dy = goal.Y - r.Pose.Y;
        if (Math.Sqrt((dx * dx) + (dy * dy)) < 0.4)
        {
            delivered[r.Name]++;
            targets[r.Name] = stops[rnd.Next(stops.Length)];
            continue;
        }

        // Right-of-way: yield to any robot with a lower name that is closer than 1.2 m and in front of us.
        var mustYield = world.Robots.Any(o => o != r && string.CompareOrdinal(o.Name, r.Name) < 0 && Distance(o, r) < 1.2 && InFront(r, o));
        if (mustYield)
        {
            yields++;
            r.SetCommand(0, 0, 0, world.Time);
            continue;
        }

        var bearing = Math.Atan2(dy, dx);
        var error = Math.Atan2(Math.Sin(bearing - r.Pose.Theta), Math.Cos(bearing - r.Pose.Theta));
        var front = r.Lidar is { } l ? l.Ranges.Take(20).Concat(l.Ranges.TakeLast(20)).Where(float.IsFinite).DefaultIfEmpty(9f).Min() : 9f;
        var v = Math.Abs(error) < 0.5 ? Math.Min(0.5, Math.Max(0, (front - 0.35) * 0.8)) : 0.05;
        r.SetCommand(v, 0, front < 0.5 ? 1.2 : 2.0 * error, world.Time);
    }

    world.StepFor(0.05);
}

Console.WriteLine($"Shift summary ({world.Time / 60:0} simulated minutes, {yields} right-of-way yields):");
foreach (var r in world.Robots)
{
    Console.WriteLine($"  {r.Name}: {delivered[r.Name]} deliveries, {r.Odometer:0} m, {r.CollisionCount} collisions");
}

static double Distance(SimRobot a, SimRobot b) => Math.Sqrt(Math.Pow(a.Pose.X - b.Pose.X, 2) + Math.Pow(a.Pose.Y - b.Pose.Y, 2));

static bool InFront(SimRobot self, SimRobot other)
{
    var bearing = Math.Atan2(other.Pose.Y - self.Pose.Y, other.Pose.X - self.Pose.X);
    return Math.Abs(Math.Atan2(Math.Sin(bearing - self.Pose.Theta), Math.Cos(bearing - self.Pose.Theta))) < 1.2;
}
