// {{ProjectName}} — pick-and-place cycle planning (ROS2.Net template).
using ROS2.Net.Manipulation;

var model = UrdfModel.LoadBuiltIn("ur5");
var arm = new KinematicChain(model, "base_link", "tool0");
var scene = new PlanningScene(model);
scene.AddBox("table", new Vec3(0.4, 0, -0.03), new Vec3(1.4, 1.4, 0.04));
scene.AddBox("fixture", new Vec3(0.45, 0.0, 0.15), new Vec3(0.08, 0.30, 0.30));

var ik = new IkSolver(arm, new IkOptions { Restarts = 10, Validator = q => scene.IsStateValid(arm, q) });
var rrt = new RrtConnectPlanner(arm, scene, new RrtOptions { Timeout = TimeSpan.FromSeconds(3) });
var vmax = arm.VelocityLimits.Select(v => v * 0.6).ToArray();
var amax = Enumerable.Repeat(3.0, arm.Dof).ToArray();

double[] Solve(string name, Pose3 pose, double[] seed)
{
    var r = ik.Solve(pose, seed);
    if (!r.Success)
    {
        throw new InvalidOperationException($"{name} is unreachable.");
    }

    return r.Solution;
}

var home = new double[] { 0, -1.57, 1.57, -1.57, -1.57, 0 };
var pick = Solve("pick", Pose3.FromXyzRpy(0.45, -0.30, 0.10, Math.PI, 0, 0), home);
var place = Solve("place", Pose3.FromXyzRpy(0.45, 0.30, 0.10, Math.PI, 0, 0), pick);

var total = 0.0;
foreach (var (from, to, label) in new[] { (home, pick, "home → pick"), (pick, place, "pick → place (over the fixture)"), (place, home, "place → home") })
{
    var plan = rrt.Plan(from, to);
    if (!plan.Success)
    {
        Console.WriteLine($"{label}: planning failed ({plan.Message})");
        return;
    }

    var trajectory = TimeParameterization.Compute(arm.JointNames, plan.Waypoints, vmax, amax, VelocityProfile.Trapezoidal);
    var duration = TimeParameterization.Duration(trajectory);
    total += duration;
    Console.WriteLine($"{label,-34} {plan.Waypoints.Count,3} waypoints {plan.PlanningTime.TotalMilliseconds,6:0} ms planning {duration,6:0.00} s motion");
}

Console.WriteLine($"Cycle time: {total + 1.0:0.00} s (including 2 × 0.5 s gripper actions) → {3600 / (total + 1.0):0} picks/hour");
