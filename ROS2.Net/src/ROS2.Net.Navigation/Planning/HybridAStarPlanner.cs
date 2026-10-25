using ROS2.Net.Navigation.Costmaps;

namespace ROS2.Net.Navigation.Planning;

/// <summary>Options for <see cref="HybridAStarPlanner"/>.</summary>
public sealed record HybridAStarOptions
{
    /// <summary>Minimum turning radius of the vehicle in meters.</summary>
    public double MinTurningRadius { get; init; } = 0.5;

    /// <summary>Number of heading bins (72 → 5°), as Smac's <c>angle_quantization_bins</c>.</summary>
    public int AngleBins { get; init; } = 72;

    /// <summary>Allow reversing (Reeds-Shepp-like motion) with <see cref="ReversePenalty"/>.</summary>
    public bool AllowReverse { get; init; }

    public double ReversePenalty { get; init; } = 2.0;

    /// <summary>Penalty for changing steering direction.</summary>
    public double ChangePenalty { get; init; } = 0.05;

    /// <summary>Penalty for turning (vs straight).</summary>
    public double NonStraightPenalty { get; init; } = 1.05;

    public double CostWeight { get; init; } = 2.0;

    /// <summary>Try an analytic Dubins shot to the goal every this many expansions.</summary>
    public int AnalyticExpansionInterval { get; init; } = 20;

    public double GoalTolerance { get; init; } = 0.15;

    public double GoalHeadingTolerance { get; init; } = 0.26;

    public int MaxIterations { get; init; } = 300_000;

    public bool AllowUnknown { get; init; } = true;
}

/// <summary>
/// A Smac-style Hybrid-A* planner for car-like (and diff-drive) robots: searches a continuous
/// (x, y, θ) state space with arc motion primitives at the minimum turning radius, guided by an
/// obstacle-aware 2-D distance heuristic, and finishes with an analytic Dubins expansion.
/// </summary>
public sealed class HybridAStarPlanner(Footprint footprint, HybridAStarOptions? options = null) : IGlobalPlanner
{
    private sealed class SearchNode
    {
        public double X;
        public double Y;
        public double Theta;
        public double G;
        public SearchNode? Parent;
        public int Steer;
        public int Direction;
        public bool Closed;
    }

    private float[] _heuristic = [];

    public HybridAStarOptions Options { get; } = options ?? new HybridAStarOptions();

    public Footprint Footprint { get; } = footprint;

    public string Name => "SmacHybrid";

    public int LastExpansions { get; private set; }

    public PlanResult CreatePlan(Costmap2D costmap, Pose2D start, Pose2D goal, CancellationToken cancellationToken = default)
    {
        lock (costmap.SyncRoot)
        {
            return Plan(costmap, start, goal, cancellationToken);
        }
    }

    private PlanResult Plan(Costmap2D costmap, Pose2D start, Pose2D goal, CancellationToken ct)
    {
        if (!costmap.WorldToCell(start.X, start.Y, out _, out _) || !costmap.WorldToCell(goal.X, goal.Y, out var gx, out var gy))
        {
            return PlanResult.Fail("start or goal is outside the costmap");
        }

        if (Footprint.InCollision(costmap, goal))
        {
            return PlanResult.Fail("goal is in an obstacle");
        }

        ComputeHeuristic(costmap, gx, gy);
        var bins = Options.AngleBins;
        var res = costmap.Resolution;
        var step = Math.Max(res * Math.Sqrt(2), (2 * Math.PI * Options.MinTurningRadius) / bins * 1.05);
        var dTheta = step / Options.MinTurningRadius;
        var visited = new Dictionary<long, SearchNode>();
        var open = new PriorityQueue<SearchNode, double>();
        var root = new SearchNode { X = start.X, Y = start.Y, Theta = start.Theta, Direction = 1 };
        visited[Key(costmap, root, bins)] = root;
        open.Enqueue(root, H(costmap, root.X, root.Y, goal));
        var expansions = 0;

        while (open.TryDequeue(out var node, out _))
        {
            if (node.Closed)
            {
                continue;
            }

            node.Closed = true;
            if (++expansions > Options.MaxIterations)
            {
                break;
            }

            if ((expansions & 0x3FF) == 0 && ct.IsCancellationRequested)
            {
                return PlanResult.Fail("canceled");
            }

            var dist = Math.Sqrt(((node.X - goal.X) * (node.X - goal.X)) + ((node.Y - goal.Y) * (node.Y - goal.Y)));
            if (dist <= Options.GoalTolerance && Math.Abs(Angles.Difference(node.Theta, goal.Theta)) <= Options.GoalHeadingTolerance)
            {
                LastExpansions = expansions;
                return new PlanResult(true, Reconstruct(node, null));
            }

            if (expansions % Options.AnalyticExpansionInterval == 0 || dist < 4 * Options.MinTurningRadius)
            {
                var shot = Dubins.Sample(new Pose2D(node.X, node.Y, node.Theta), goal, Options.MinTurningRadius, res);
                if (shot is not null && shot.TrueForAll(p => IsFree(costmap, p)))
                {
                    LastExpansions = expansions;
                    return new PlanResult(true, Reconstruct(node, shot));
                }
            }

            for (var dir = 1; dir >= (Options.AllowReverse ? -1 : 1); dir -= 2)
            {
                for (var steer = -1; steer <= 1; steer++)
                {
                    var theta = node.Theta + (dir * steer * dTheta);
                    double nx, ny;
                    if (steer == 0)
                    {
                        nx = node.X + (dir * step * Math.Cos(node.Theta));
                        ny = node.Y + (dir * step * Math.Sin(node.Theta));
                    }
                    else
                    {
                        // Exact arc: chord of length 2R·sin(Δθ/2) along the mid heading.
                        var chord = 2 * Options.MinTurningRadius * Math.Sin(dTheta / 2);
                        var mid = node.Theta + (dir * steer * dTheta / 2);
                        nx = node.X + (dir * chord * Math.Cos(mid));
                        ny = node.Y + (dir * chord * Math.Sin(mid));
                    }

                    theta = Angles.Normalize(theta);
                    if (!costmap.WorldToCell(nx, ny, out var cx, out var cy))
                    {
                        continue;
                    }

                    var c = costmap.GetCost(cx, cy);
                    if (!IsFree(costmap, new Pose2D(nx, ny, theta)))
                    {
                        continue;
                    }

                    var travel = step * (steer == 0 ? 1.0 : Options.NonStraightPenalty) * (dir < 0 ? Options.ReversePenalty : 1.0);
                    travel *= 1.0 + (Options.CostWeight * (c == Costs.NoInformation ? 0.5 : c / (double)Costs.MaxNonObstacle));
                    if (steer != node.Steer || dir != node.Direction)
                    {
                        travel += Options.ChangePenalty * step;
                    }

                    var g = node.G + travel;
                    var child = new SearchNode { X = nx, Y = ny, Theta = theta, G = g, Parent = node, Steer = steer, Direction = dir };
                    var key = Key(costmap, child, bins);
                    if (visited.TryGetValue(key, out var existing))
                    {
                        if (existing.Closed || existing.G <= g)
                        {
                            continue;
                        }
                    }

                    visited[key] = child;
                    open.Enqueue(child, g + H(costmap, nx, ny, goal));
                }
            }
        }

        LastExpansions = expansions;
        return PlanResult.Fail("no path found");
    }

    private bool IsFree(Costmap2D costmap, Pose2D p)
    {
        if (!costmap.WorldToCell(p.X, p.Y, out var cx, out var cy))
        {
            return false;
        }

        return costmap.GetCost(cx, cy) == Costs.NoInformation ? Options.AllowUnknown : !Footprint.InCollision(costmap, p);
    }

    private static long Key(Costmap2D costmap, SearchNode n, int bins)
    {
        costmap.WorldToCell(n.X, n.Y, out var cx, out var cy);
        var tb = (int)Math.Floor((Angles.Normalize(n.Theta) + Math.PI) / (2 * Math.PI) * bins) % bins;
        return ((((long)cy * costmap.Width) + cx) * bins) + tb;
    }

    private double H(Costmap2D costmap, double x, double y, Pose2D goal)
    {
        var euclid = Math.Sqrt(((x - goal.X) * (x - goal.X)) + ((y - goal.Y) * (y - goal.Y)));
        if (!costmap.WorldToCell(x, y, out var cx, out var cy))
        {
            return euclid;
        }

        var obstacle = _heuristic[(cy * costmap.Width) + cx];
        return obstacle == float.MaxValue ? euclid * 10 : Math.Max(euclid, obstacle);
    }

    /// <summary>Obstacle-aware 2-D distance-to-goal for every cell (Dijkstra from the goal).</summary>
    private void ComputeHeuristic(Costmap2D costmap, int gx, int gy)
    {
        var w = costmap.Width;
        var n = w * costmap.Height;
        if (_heuristic.Length != n)
        {
            _heuristic = new float[n];
        }

        _heuristic.AsSpan().Fill(float.MaxValue);
        var queue = new PriorityQueue<int, float>();
        var gi = (gy * w) + gx;
        _heuristic[gi] = 0;
        queue.Enqueue(gi, 0);
        var res = (float)costmap.Resolution;
        var diag = res * MathF.Sqrt(2);
        while (queue.TryDequeue(out var i, out var d))
        {
            if (d > _heuristic[i])
            {
                continue;
            }

            var x = i % w;
            var y = i / w;
            for (var dy = -1; dy <= 1; dy++)
            {
                for (var dx = -1; dx <= 1; dx++)
                {
                    if (dx == 0 && dy == 0)
                    {
                        continue;
                    }

                    var nx = x + dx;
                    var ny = y + dy;
                    if (!costmap.InBounds(nx, ny))
                    {
                        continue;
                    }

                    var ni = (ny * w) + nx;
                    var c = costmap.Data[ni];
                    if (c >= Costs.Inscribed && c != Costs.NoInformation)
                    {
                        continue;
                    }

                    var nd = d + (dx != 0 && dy != 0 ? diag : res);
                    if (nd < _heuristic[ni])
                    {
                        _heuristic[ni] = nd;
                        queue.Enqueue(ni, nd);
                    }
                }
            }
        }
    }

    private static List<Pose2D> Reconstruct(SearchNode end, List<Pose2D>? tail)
    {
        var poses = new List<Pose2D>();
        for (var n = end; n is not null; n = n.Parent)
        {
            poses.Add(new Pose2D(n.X, n.Y, n.Theta));
        }

        poses.Reverse();
        if (tail is not null)
        {
            poses.AddRange(tail.Skip(1));
        }

        return poses;
    }
}

/// <summary>Shortest forward-only paths for a vehicle with a bounded turning radius (Dubins, 1957).</summary>
public static class Dubins
{
    /// <summary>Samples the shortest Dubins path from <paramref name="from"/> to <paramref name="to"/> every <paramref name="spacing"/> meters.</summary>
    public static List<Pose2D>? Sample(Pose2D from, Pose2D to, double radius, double spacing)
    {
        var dx = to.X - from.X;
        var dy = to.Y - from.Y;
        var d = Math.Sqrt((dx * dx) + (dy * dy)) / radius;
        var th = Math.Atan2(dy, dx);
        var a = Mod2Pi(from.Theta - th);
        var b = Mod2Pi(to.Theta - th);

        (double T, double P, double Q, string Word)? best = null;
        foreach (var word in new[] { "LSL", "RSR", "LSR", "RSL", "RLR", "LRL" })
        {
            if (Solve(word, a, b, d) is { } s && (best is null || s.T + s.P + s.Q < best.Value.T + best.Value.P + best.Value.Q))
            {
                best = (s.T, s.P, s.Q, word);
            }
        }

        if (best is null)
        {
            return null;
        }

        var (t, p, q, w) = best.Value;
        var segments = new[] { t, p, q };
        var poses = new List<Pose2D> { from };
        var cur = from;
        var stepNorm = spacing / radius;
        for (var s = 0; s < 3; s++)
        {
            var len = segments[s];
            var kind = w[s];
            var done = 0.0;
            while (done < len - 1e-9)
            {
                var ds = Math.Min(stepNorm, len - done);
                cur = Advance(cur, kind, ds, radius);
                poses.Add(cur);
                done += ds;
            }
        }

        poses[^1] = to;
        return poses;
    }

    private static Pose2D Advance(Pose2D p, char kind, double ds, double r)
    {
        switch (kind)
        {
            case 'S':
                return new Pose2D(p.X + (ds * r * Math.Cos(p.Theta)), p.Y + (ds * r * Math.Sin(p.Theta)), p.Theta);
            case 'L':
            {
                var th = p.Theta + ds;
                return new Pose2D(p.X + (r * (Math.Sin(th) - Math.Sin(p.Theta))), p.Y - (r * (Math.Cos(th) - Math.Cos(p.Theta))), Angles.Normalize(th));
            }

            default:
            {
                var th = p.Theta - ds;
                return new Pose2D(p.X - (r * (Math.Sin(th) - Math.Sin(p.Theta))), p.Y + (r * (Math.Cos(th) - Math.Cos(p.Theta))), Angles.Normalize(th));
            }
        }
    }

    private static (double T, double P, double Q)? Solve(string word, double a, double b, double d)
    {
        double sa = Math.Sin(a), sb = Math.Sin(b), ca = Math.Cos(a), cb = Math.Cos(b), cab = Math.Cos(a - b);
        switch (word)
        {
            case "LSL":
            {
                var p2 = 2 + (d * d) - (2 * cab) + (2 * d * (sa - sb));
                if (p2 < 0)
                {
                    return null;
                }

                var tmp = Math.Atan2(cb - ca, d + sa - sb);
                return (Mod2Pi(-a + tmp), Math.Sqrt(p2), Mod2Pi(b - tmp));
            }

            case "RSR":
            {
                var p2 = 2 + (d * d) - (2 * cab) + (2 * d * (sb - sa));
                if (p2 < 0)
                {
                    return null;
                }

                var tmp = Math.Atan2(ca - cb, d - sa + sb);
                return (Mod2Pi(a - tmp), Math.Sqrt(p2), Mod2Pi(-b + tmp));
            }

            case "LSR":
            {
                var p2 = -2 + (d * d) + (2 * cab) + (2 * d * (sa + sb));
                if (p2 < 0)
                {
                    return null;
                }

                var p = Math.Sqrt(p2);
                var tmp = Math.Atan2(-ca - cb, d + sa + sb) - Math.Atan2(-2, p);
                return (Mod2Pi(-a + tmp), p, Mod2Pi(-Mod2Pi(b) + tmp));
            }

            case "RSL":
            {
                var p2 = (d * d) - 2 + (2 * cab) - (2 * d * (sa + sb));
                if (p2 < 0)
                {
                    return null;
                }

                var p = Math.Sqrt(p2);
                var tmp = Math.Atan2(ca + cb, d - sa - sb) - Math.Atan2(2, p);
                return (Mod2Pi(a - tmp), p, Mod2Pi(b - tmp));
            }

            case "RLR":
            {
                var tmp = (6 - (d * d) + (2 * cab) + (2 * d * (sa - sb))) / 8;
                if (Math.Abs(tmp) > 1)
                {
                    return null;
                }

                var p = Mod2Pi((2 * Math.PI) - Math.Acos(tmp));
                var t = Mod2Pi(a - Math.Atan2(ca - cb, d - sa + sb) + (p / 2));
                return (t, p, Mod2Pi(a - b - t + p));
            }

            default: // LRL
            {
                var tmp = (6 - (d * d) + (2 * cab) + (2 * d * (sb - sa))) / 8;
                if (Math.Abs(tmp) > 1)
                {
                    return null;
                }

                var p = Mod2Pi((2 * Math.PI) - Math.Acos(tmp));
                var t = Mod2Pi(-a - Math.Atan2(ca - cb, d + sa - sb) + (p / 2));
                return (t, p, Mod2Pi(Mod2Pi(b) - a - t + p));
            }
        }
    }

    private static double Mod2Pi(double x)
    {
        var r = x % (2 * Math.PI);
        return r < 0 ? r + (2 * Math.PI) : r;
    }
}

/// <summary>
/// Gradient-descent path smoother (like Nav2's simple smoother): pulls points toward their
/// neighbors while keeping them near the original path and out of obstacles. Endpoints are fixed.
/// </summary>
public sealed class PathSmoother
{
    public double DataWeight { get; set; } = 0.2;

    public double SmoothWeight { get; set; } = 0.3;

    public int MaxIterations { get; set; } = 500;

    public double Tolerance { get; set; } = 1e-6;

    public List<Pose2D> Smooth(IReadOnlyList<Pose2D> path, Costmap2D? costmap = null)
    {
        var n = path.Count;
        if (n < 3)
        {
            return path.ToList();
        }

        var ox = new double[n];
        var oy = new double[n];
        for (var i = 0; i < n; i++)
        {
            ox[i] = path[i].X;
            oy[i] = path[i].Y;
        }

        var x = (double[])ox.Clone();
        var y = (double[])oy.Clone();
        for (var iter = 0; iter < MaxIterations; iter++)
        {
            var change = 0.0;
            for (var i = 1; i < n - 1; i++)
            {
                var nx = x[i] + (DataWeight * (ox[i] - x[i])) + (SmoothWeight * (x[i - 1] + x[i + 1] - (2 * x[i])));
                var ny = y[i] + (DataWeight * (oy[i] - y[i])) + (SmoothWeight * (y[i - 1] + y[i + 1] - (2 * y[i])));
                if (costmap is not null)
                {
                    var c = costmap.GetCostAt(nx, ny);
                    if (c >= Costs.Inscribed && c != Costs.NoInformation)
                    {
                        continue;
                    }
                }

                change += Math.Abs(nx - x[i]) + Math.Abs(ny - y[i]);
                x[i] = nx;
                y[i] = ny;
            }

            if (change < Tolerance)
            {
                break;
            }
        }

        var pts = new List<(double X, double Y)>(n);
        for (var i = 0; i < n; i++)
        {
            pts.Add((x[i], y[i]));
        }

        return PathUtils.OrientAlongPath(pts, path[^1].Theta);
    }
}
