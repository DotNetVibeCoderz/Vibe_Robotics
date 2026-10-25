namespace ROS2.Net.Manipulation;

/// <summary>Result of a Cartesian path: the joint waypoints achieved and the fraction of the path that was feasible.</summary>
public sealed record CartesianPathResult(IReadOnlyList<double[]> Waypoints, double Fraction)
{
    public bool Complete => Fraction >= 1 - 1e-9;
}

/// <summary>
/// Straight-line motion of the tip between poses (linear position, slerp orientation), solved point by point with
/// IK seeded from the previous solution, like MoveIt's <c>computeCartesianPath</c>. Stops at the first
/// unreachable, colliding or discontinuous step.
/// </summary>
public sealed class CartesianPlanner(KinematicChain chain, PlanningScene? scene = null)
{
    public KinematicChain Chain { get; } = chain;

    public PlanningScene? Scene { get; } = scene;

    /// <summary>Maximum tip translation per step (m).</summary>
    public double EndEffectorStep { get; set; } = 0.01;

    /// <summary>Maximum tip rotation per step (rad).</summary>
    public double RotationStep { get; set; } = 0.05;

    /// <summary>Largest allowed joint change between consecutive steps (rad); guards against IK branch flips.</summary>
    public double JumpThreshold { get; set; } = 0.5;

    public CartesianPathResult Plan(double[] start, IReadOnlyList<Pose3> targets)
    {
        var ik = new IkSolver(Chain, new IkOptions { Restarts = 0, MaxIterations = 100, PositionTolerance = 1e-5, OrientationTolerance = 1e-4 });
        var waypoints = new List<double[]> { (double[])start.Clone() };
        var current = start;
        var from = Chain.ForwardKinematics(start);
        var totalSteps = 0;
        var doneSteps = 0;
        var segments = new List<(Pose3 A, Pose3 B, int Steps)>();
        foreach (var target in targets)
        {
            var steps = Math.Max(1, (int)Math.Ceiling(Math.Max((target.Position - from.Position).Length / EndEffectorStep, from.Rotation.AngleTo(target.Rotation) / RotationStep)));
            segments.Add((from, target, steps));
            totalSteps += steps;
            from = target;
        }

        foreach (var (a, b, steps) in segments)
        {
            for (var k = 1; k <= steps; k++)
            {
                var pose = Pose3.Interpolate(a, b, (double)k / steps);
                var r = ik.Solve(pose, current);
                if (!r.Success || MaxJump(current, r.Solution) > JumpThreshold || !(Scene?.IsStateValid(Chain, r.Solution) ?? true))
                {
                    return new CartesianPathResult(waypoints, totalSteps == 0 ? 1 : (double)doneSteps / totalSteps);
                }

                waypoints.Add(r.Solution);
                current = r.Solution;
                doneSteps++;
            }
        }

        return new CartesianPathResult(waypoints, 1);
    }

    private static double MaxJump(double[] a, double[] b)
    {
        var m = 0.0;
        for (var i = 0; i < a.Length; i++)
        {
            m = Math.Max(m, Math.Abs(a[i] - b[i]));
        }

        return m;
    }
}
