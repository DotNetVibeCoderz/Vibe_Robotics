// {{ProjectName}} — laboratory sample sorter (ROS2.Net template): IK + collision-free planning with a UR5 model.
using ROS2.Net.Manipulation;

var model = UrdfModel.LoadBuiltIn("ur5");
var arm = new KinematicChain(model, "base_link", "tool0");
var scene = new PlanningScene(model);
scene.AddBox("rack", new Vec3(0.45, 0.0, 0.05), new Vec3(0.20, 0.40, 0.10));
scene.AddBox("table", new Vec3(0.40, 0.0, -0.03), new Vec3(1.2, 1.2, 0.04));

var ik = new IkSolver(arm, new IkOptions { Restarts = 8, Validator = q => scene.IsStateValid(arm, q) });
var planner = new RrtConnectPlanner(arm, scene, new RrtOptions { Timeout = TimeSpan.FromSeconds(2) });

var home = new double[arm.Dof];
home[1] = -1.57;
home[2] = 1.57;

// Vial slots: 2 rows x 4 columns, approached from above (gripper pointing down).
var slots = new List<(string Name, Pose3 Pose)>();
for (var row = 0; row < 2; row++)
{
    for (var col = 0; col < 4; col++)
    {
        slots.Add(($"{(char)('A' + row)}{col + 1}", Pose3.FromXyzRpy(0.40 + (row * 0.08), -0.12 + (col * 0.08), 0.28, Math.PI, 0, 0)));
    }
}

Console.WriteLine($"{"slot",-5} {"IK",-4} {"plan",-5} {"waypoints",9} {"duration",9}");
var current = home;
foreach (var (name, pose) in slots)
{
    var solution = ik.Solve(pose, current);
    if (!solution.Success)
    {
        Console.WriteLine($"{name,-5} no   -");
        continue;
    }

    var plan = planner.Plan(current, solution.Solution);
    if (!plan.Success)
    {
        Console.WriteLine($"{name,-5} yes  no    {plan.Message}");
        continue;
    }

    var trajectory = TimeParameterization.Compute(arm.JointNames, plan.Waypoints, arm.VelocityLimits.Select(v => v * 0.5).ToArray(), Enumerable.Repeat(2.0, arm.Dof).ToArray());
    Console.WriteLine($"{name,-5} yes  yes   {plan.Waypoints.Count,9} {TimeParameterization.Duration(trajectory),8:0.00}s");
    current = solution.Solution;
}
