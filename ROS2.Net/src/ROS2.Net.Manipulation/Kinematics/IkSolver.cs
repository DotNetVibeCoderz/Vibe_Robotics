namespace ROS2.Net.Manipulation;

/// <summary>Numerical IK method.</summary>
public enum IkMethod
{
    /// <summary>Levenberg–Marquardt with adaptive damping (robust, the default).</summary>
    LevenbergMarquardt,
    /// <summary>Damped least squares (Jacobian pseudo-inverse with fixed damping).</summary>
    DampedLeastSquares,
}

/// <summary>IK options.</summary>
public sealed record IkOptions
{
    public IkMethod Method { get; init; } = IkMethod.LevenbergMarquardt;

    public int MaxIterations { get; init; } = 200;

    /// <summary>Random restarts after the seed fails.</summary>
    public int Restarts { get; init; } = 30;

    /// <summary>Position tolerance (m).</summary>
    public double PositionTolerance { get; init; } = 1e-6;

    /// <summary>Orientation tolerance (rad).</summary>
    public double OrientationTolerance { get; init; } = 1e-6;

    /// <summary>Ignore orientation (3-DOF position targets).</summary>
    public bool PositionOnly { get; init; }

    /// <summary>Initial damping.</summary>
    public double Damping { get; init; } = 1e-3;

    /// <summary>Seed of the restart sampler (deterministic by default).</summary>
    public int RandomSeed { get; init; } = 42;

    /// <summary>Optional extra acceptance test (e.g. collision-free) for a solution.</summary>
    public Func<double[], bool>? Validator { get; init; }
}

/// <summary>Outcome of an IK query.</summary>
public sealed record IkResult(bool Success, double[] Solution, double PositionError, double OrientationError, int Iterations);

/// <summary>Numerical inverse kinematics on a <see cref="KinematicChain"/>, respecting joint limits.</summary>
public sealed class IkSolver(KinematicChain chain, IkOptions? options = null)
{
    public KinematicChain Chain { get; } = chain;

    public IkOptions Options { get; } = options ?? new IkOptions();

    /// <summary>Solves for <paramref name="target"/> (tip pose in the base frame), starting from <paramref name="seed"/>.</summary>
    public IkResult Solve(Pose3 target, ReadOnlySpan<double> seed)
    {
        var random = new Random(Options.RandomSeed);
        var best = (IkResult?)null;
        var totalIterations = 0;
        var start = seed.ToArray();
        for (var attempt = 0; attempt <= Options.Restarts; attempt++)
        {
            if (attempt > 0)
            {
                start = Chain.RandomConfiguration(random);
            }

            var r = SolveOnce(target, start);
            totalIterations += r.Iterations;
            if (r.Success && (Options.Validator?.Invoke(r.Solution) ?? true))
            {
                return r with { Iterations = totalIterations };
            }

            if (best is null || r.PositionError + r.OrientationError < best.PositionError + best.OrientationError)
            {
                best = r with { Success = false };
            }
        }

        return best! with { Iterations = totalIterations };
    }

    public IkResult Solve(Pose3 target) => Solve(target, new double[Chain.Dof]);

    private IkResult SolveOnce(Pose3 target, double[] q0)
    {
        var n = Chain.Dof;
        var rows = Options.PositionOnly ? 3 : 6;
        var q = (double[])q0.Clone();
        Chain.ClampToLimits(q);
        var jac = new double[6 * n];
        var err = new double[6];
        var trial = new double[n];
        var lambda = Options.Damping;
        var pose = Chain.Jacobian(q, jac);
        var cost = Error(target, pose, err, rows);
        var iterations = 0;

        for (; iterations < Options.MaxIterations; iterations++)
        {
            if (Converged(target, pose))
            {
                break;
            }

            var dq = Options.Method == IkMethod.LevenbergMarquardt ? StepLm(jac, err, n, rows, lambda) : StepDls(jac, err, n, rows, lambda);
            if (dq is null)
            {
                lambda *= 10;
                continue;
            }

            for (var i = 0; i < n; i++)
            {
                trial[i] = q[i] + dq[i];
            }

            Chain.ClampToLimits(trial);
            var trialPose = Chain.ForwardKinematics(trial);
            var trialErr = new double[6];
            var trialCost = Error(target, trialPose, trialErr, rows);
            if (Options.Method == IkMethod.DampedLeastSquares || trialCost < cost)
            {
                Array.Copy(trial, q, n);
                pose = Chain.Jacobian(q, jac);
                cost = Error(target, pose, err, rows);
                lambda = Math.Max(lambda * 0.5, 1e-9);
            }
            else
            {
                lambda = Math.Min(lambda * 4, 1e6);
                if (lambda >= 1e6)
                {
                    break; // Stuck in a local minimum: let the caller restart.
                }
            }
        }

        var posErr = (target.Position - pose.Position).Length;
        var oriErr = Options.PositionOnly ? 0 : target.Rotation.AngleTo(pose.Rotation);
        return new IkResult(Converged(target, pose), q, posErr, oriErr, iterations);
    }

    private bool Converged(Pose3 target, Pose3 pose) =>
        (target.Position - pose.Position).Length <= Options.PositionTolerance &&
        (Options.PositionOnly || target.Rotation.AngleTo(pose.Rotation) <= Options.OrientationTolerance);

    private static double Error(Pose3 target, Pose3 pose, double[] err, int rows)
    {
        var dp = target.Position - pose.Position;
        err[0] = dp.X;
        err[1] = dp.Y;
        err[2] = dp.Z;
        if (rows == 6)
        {
            var dr = (target.Rotation * pose.Rotation.Conjugate()).ToRotationVector();
            err[3] = dr.X;
            err[4] = dr.Y;
            err[5] = dr.Z;
        }

        var c = 0.0;
        for (var i = 0; i < rows; i++)
        {
            c += err[i] * err[i];
        }

        return c;
    }

    // (JᵀJ + λ·diag(JᵀJ) + λI) Δq = Jᵀe
    private static double[]? StepLm(double[] j, double[] e, int n, int rows, double lambda)
    {
        var a = new double[n * n];
        var b = new double[n];
        for (var r = 0; r < n; r++)
        {
            for (var c = 0; c < n; c++)
            {
                var s = 0.0;
                for (var k = 0; k < rows; k++)
                {
                    s += j[(k * n) + r] * j[(k * n) + c];
                }

                a[(r * n) + c] = s;
            }

            a[(r * n) + r] += (lambda * a[(r * n) + r]) + lambda;
            var g = 0.0;
            for (var k = 0; k < rows; k++)
            {
                g += j[(k * n) + r] * e[k];
            }

            b[r] = g;
        }

        return DenseMath.Solve(a, b, n) ? b : null;
    }

    // Δq = Jᵀ (JJᵀ + λ²I)⁻¹ e
    private static double[]? StepDls(double[] j, double[] e, int n, int rows, double lambda)
    {
        var a = new double[rows * rows];
        var b = new double[rows];
        for (var r = 0; r < rows; r++)
        {
            for (var c = 0; c < rows; c++)
            {
                var s = 0.0;
                for (var k = 0; k < n; k++)
                {
                    s += j[(r * n) + k] * j[(c * n) + k];
                }

                a[(r * rows) + c] = s + (r == c ? Math.Max(lambda, 1e-4) * Math.Max(lambda, 1e-4) : 0);
            }

            b[r] = e[r];
        }

        if (!DenseMath.Solve(a, b, rows))
        {
            return null;
        }

        var dq = new double[n];
        for (var k = 0; k < n; k++)
        {
            var s = 0.0;
            for (var r = 0; r < rows; r++)
            {
                s += j[(r * n) + k] * b[r];
            }

            dq[k] = s;
        }

        return dq;
    }

    /// <summary>
    /// Closed-form IK of a planar 2-link arm with link lengths <paramref name="l1"/>, <paramref name="l2"/>.
    /// Returns (q1, q2) for the requested elbow branch, or null when unreachable.
    /// </summary>
    public static (double Q1, double Q2)? TwoLinkPlanar(double x, double y, double l1, double l2, bool elbowUp = true)
    {
        var d2 = (x * x) + (y * y);
        var c2 = (d2 - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2);
        if (c2 < -1 - 1e-12 || c2 > 1 + 1e-12)
        {
            return null;
        }

        c2 = Math.Clamp(c2, -1, 1);
        var s2 = Math.Sqrt(1 - (c2 * c2)) * (elbowUp ? 1 : -1);
        var q2 = Math.Atan2(s2, c2);
        var q1 = Math.Atan2(y, x) - Math.Atan2(l2 * s2, l1 + (l2 * c2));
        return (q1, q2);
    }
}
