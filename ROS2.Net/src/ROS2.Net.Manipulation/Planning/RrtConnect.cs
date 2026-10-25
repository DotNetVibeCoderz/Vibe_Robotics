namespace ROS2.Net.Manipulation;

/// <summary>Options for <see cref="RrtConnectPlanner"/>.</summary>
public sealed record RrtOptions
{
    public int MaxIterations { get; init; } = 10_000;

    /// <summary>Maximum extension per step (rad / m in joint space).</summary>
    public double StepSize { get; init; } = 0.15;

    /// <summary>Resolution of edge collision checks.</summary>
    public double EdgeResolution { get; init; } = 0.02;

    public TimeSpan Timeout { get; init; } = TimeSpan.FromSeconds(5);

    /// <summary>Shortcut attempts after a path is found (0 disables smoothing).</summary>
    public int ShortcutIterations { get; init; } = 200;

    public int RandomSeed { get; init; } = 7;
}

/// <summary>Result of a joint-space plan.</summary>
public sealed record PlanResult(bool Success, IReadOnlyList<double[]> Waypoints, int Iterations, TimeSpan PlanningTime, string Message = "");

/// <summary>
/// Bidirectional RRT-Connect (Kuffner &amp; LaValle 2000) in the joint space of a <see cref="KinematicChain"/>,
/// with edge collision checking against a <see cref="PlanningScene"/> and random shortcut smoothing.
/// </summary>
public sealed class RrtConnectPlanner(KinematicChain chain, PlanningScene? scene = null, RrtOptions? options = null)
{
    private sealed class Tree
    {
        public List<double[]> Nodes { get; } = [];
        public List<int> Parents { get; } = [];

        public int Add(double[] q, int parent)
        {
            Nodes.Add(q);
            Parents.Add(parent);
            return Nodes.Count - 1;
        }

        public int Nearest(double[] q)
        {
            var best = 0;
            var bestD = double.PositiveInfinity;
            for (var i = 0; i < Nodes.Count; i++)
            {
                var d = DistanceSquared(Nodes[i], q);
                if (d < bestD)
                {
                    bestD = d;
                    best = i;
                }
            }

            return best;
        }

        public List<double[]> PathTo(int index)
        {
            var path = new List<double[]>();
            for (var i = index; i >= 0; i = Parents[i])
            {
                path.Add(Nodes[i]);
            }

            path.Reverse();
            return path;
        }
    }

    private enum Extend
    {
        Trapped,
        Advanced,
        Reached,
    }

    public KinematicChain Chain { get; } = chain;

    public PlanningScene? Scene { get; } = scene;

    public RrtOptions Options { get; } = options ?? new RrtOptions();

    public bool IsValid(double[] q) => Chain.WithinLimits(q) && (Scene?.IsStateValid(Chain, q) ?? true);

    /// <summary>True when the straight joint-space segment is collision-free at <see cref="RrtOptions.EdgeResolution"/>.</summary>
    public bool IsEdgeValid(double[] a, double[] b)
    {
        var dist = Math.Sqrt(DistanceSquared(a, b));
        var steps = Math.Max(1, (int)Math.Ceiling(dist / Options.EdgeResolution));
        var q = new double[a.Length];
        for (var s = 1; s <= steps; s++)
        {
            var t = (double)s / steps;
            for (var i = 0; i < q.Length; i++)
            {
                q[i] = a[i] + ((b[i] - a[i]) * t);
            }

            if (!IsValid(q))
            {
                return false;
            }
        }

        return true;
    }

    public PlanResult Plan(double[] start, double[] goal)
    {
        var watch = System.Diagnostics.Stopwatch.StartNew();
        if (!IsValid(start))
        {
            return new PlanResult(false, [], 0, watch.Elapsed, "start state is invalid (in collision or outside limits)");
        }

        if (!IsValid(goal))
        {
            return new PlanResult(false, [], 0, watch.Elapsed, "goal state is invalid (in collision or outside limits)");
        }

        if (IsEdgeValid(start, goal))
        {
            return new PlanResult(true, [start, goal], 0, watch.Elapsed);
        }

        var random = new Random(Options.RandomSeed);
        var a = new Tree();
        var b = new Tree();
        a.Add((double[])start.Clone(), -1);
        b.Add((double[])goal.Clone(), -1);
        var aIsStart = true;

        for (var iter = 1; iter <= Options.MaxIterations; iter++)
        {
            if (watch.Elapsed > Options.Timeout)
            {
                return new PlanResult(false, [], iter, watch.Elapsed, "timed out");
            }

            var sample = Chain.RandomConfiguration(random);
            if (ExtendTree(a, sample, out var newIndex) != Extend.Trapped)
            {
                var target = a.Nodes[newIndex];
                if (ConnectTree(b, target, out var otherIndex) == Extend.Reached)
                {
                    var pa = a.PathTo(newIndex);
                    var pb = b.PathTo(otherIndex);
                    pb.Reverse();
                    // root(a) -> ... -> root(b); flip when tree a grew from the goal.
                    var path = pa.Concat(pb.Skip(1)).ToList();
                    if (!aIsStart)
                    {
                        path.Reverse();
                    }

                    path = Shortcut(path, random);
                    return new PlanResult(true, path, iter, watch.Elapsed);
                }
            }

            (a, b) = (b, a);
            aIsStart = !aIsStart;
        }

        return new PlanResult(false, [], Options.MaxIterations, watch.Elapsed, "no path found within the iteration limit");
    }

    private Extend ExtendTree(Tree tree, double[] target, out int index)
    {
        var near = tree.Nearest(target);
        var from = tree.Nodes[near];
        var dist = Math.Sqrt(DistanceSquared(from, target));
        double[] next;
        Extend status;
        if (dist <= Options.StepSize)
        {
            next = (double[])target.Clone();
            status = Extend.Reached;
        }
        else
        {
            next = new double[from.Length];
            for (var i = 0; i < next.Length; i++)
            {
                next[i] = from[i] + ((target[i] - from[i]) * Options.StepSize / dist);
            }

            status = Extend.Advanced;
        }

        if (!IsEdgeValid(from, next))
        {
            index = -1;
            return Extend.Trapped;
        }

        index = tree.Add(next, near);
        return status;
    }

    private Extend ConnectTree(Tree tree, double[] target, out int index)
    {
        Extend s;
        index = -1;
        do
        {
            s = ExtendTree(tree, target, out var i);
            if (i >= 0)
            {
                index = i;
            }
        }
        while (s == Extend.Advanced);

        return s;
    }

    private List<double[]> Shortcut(List<double[]> path, Random random)
    {
        for (var k = 0; k < Options.ShortcutIterations && path.Count > 2; k++)
        {
            var i = random.Next(path.Count - 2);
            var j = random.Next(i + 2, path.Count);
            if (IsEdgeValid(path[i], path[j]))
            {
                path.RemoveRange(i + 1, j - i - 1);
            }
        }

        return path;
    }

    internal static double DistanceSquared(double[] a, double[] b)
    {
        var s = 0.0;
        for (var i = 0; i < a.Length; i++)
        {
            var d = a[i] - b[i];
            s += d * d;
        }

        return s;
    }
}
