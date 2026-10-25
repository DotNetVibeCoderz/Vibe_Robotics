using ROS2.Net.Navigation.Costmaps;
using ROS2.Net.Navigation.Maps;

namespace ROS2.Net.Navigation.Planning;

/// <summary>
/// Shared grid search (8-connected) used by A*, Dijkstra and Theta*. Search buffers are reused
/// between plans, so repeated planning on the same costmap does not allocate per cell.
/// </summary>
public abstract class GridPlannerBase(GridPlannerOptions? options = null) : IGlobalPlanner
{
    private static readonly int[] s_dx = [1, -1, 0, 0, 1, 1, -1, -1];
    private static readonly int[] s_dy = [0, 0, 1, -1, 1, -1, 1, -1];
    private static readonly double[] s_step = [1, 1, 1, 1, Math.Sqrt(2), Math.Sqrt(2), Math.Sqrt(2), Math.Sqrt(2)];

    private float[] _g = [];
    private int[] _parent = [];
    private int[] _stamp = [];
    private bool[] _closed = [];
    private int _generation;
    private readonly PriorityQueue<int, double> _open = new();

    public GridPlannerOptions Options { get; } = options ?? new GridPlannerOptions();

    public abstract string Name { get; }

    /// <summary>Heuristic weight: 1 for A*, 0 for Dijkstra.</summary>
    protected abstract double HeuristicWeight { get; }

    /// <summary>Whether to apply Theta* any-angle parent shortcuts.</summary>
    protected virtual bool AnyAngle => false;

    /// <summary>Number of cells expanded by the last plan (diagnostics).</summary>
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
        if (!costmap.WorldToCell(start.X, start.Y, out var sx, out var sy))
        {
            return PlanResult.Fail("start is outside the costmap");
        }

        if (!costmap.WorldToCell(goal.X, goal.Y, out var gx, out var gy))
        {
            return PlanResult.Fail("goal is outside the costmap");
        }

        var w = costmap.Width;
        var n = w * costmap.Height;
        if (!Traversable(costmap.Data[(gy * w) + gx]) && !FindNearestFree(costmap, ref gx, ref gy))
        {
            return PlanResult.Fail("goal is in an obstacle");
        }

        Prepare(n);
        var gen = _generation;
        var startIndex = (sy * w) + sx;
        var goalIndex = (gy * w) + gx;
        _open.Clear();
        Touch(startIndex, gen);
        _g[startIndex] = 0;
        _parent[startIndex] = startIndex;
        _open.Enqueue(startIndex, Heuristic(sx, sy, gx, gy));
        var data = costmap.Data;
        var expansions = 0;
        var found = false;

        while (_open.TryDequeue(out var current, out _))
        {
            if (_closed[current])
            {
                continue;
            }

            _closed[current] = true;
            if (current == goalIndex)
            {
                found = true;
                break;
            }

            if (++expansions > Options.MaxIterations)
            {
                break;
            }

            if ((expansions & 0xFFF) == 0 && ct.IsCancellationRequested)
            {
                return PlanResult.Fail("canceled");
            }

            var cx = current % w;
            var cy = current / w;
            for (var k = 0; k < 8; k++)
            {
                var nx = cx + s_dx[k];
                var ny = cy + s_dy[k];
                if (!costmap.InBounds(nx, ny))
                {
                    continue;
                }

                var ni = (ny * w) + nx;
                var cost = data[ni];
                if (!Traversable(cost))
                {
                    continue;
                }

                // Do not cut corners diagonally past obstacles.
                if (k >= 4 && (!Traversable(data[(cy * w) + nx]) || !Traversable(data[(ny * w) + cx])))
                {
                    continue;
                }

                if (_stamp[ni] != gen)
                {
                    Touch(ni, gen);
                }
                else if (_closed[ni])
                {
                    continue;
                }

                var parent = current;
                double tentative;
                var p = _parent[current];
                if (AnyAngle && p != current && LineOfSight(costmap, p % w, p / w, nx, ny))
                {
                    parent = p;
                    var dx = (p % w) - nx;
                    var dy = (p / w) - ny;
                    tentative = _g[p] + (Math.Sqrt((dx * dx) + (dy * dy)) * StepCost(cost));
                }
                else
                {
                    tentative = _g[current] + (s_step[k] * StepCost(cost));
                }

                if (tentative < _g[ni])
                {
                    _g[ni] = (float)tentative;
                    _parent[ni] = parent;
                    _open.Enqueue(ni, tentative + Heuristic(nx, ny, gx, gy));
                }
            }
        }

        LastExpansions = expansions;
        if (!found)
        {
            return PlanResult.Fail("no path found");
        }

        var cells = new List<(double X, double Y)>();
        for (var i = goalIndex; ; i = _parent[i])
        {
            cells.Add(costmap.CellToWorld(i % w, i / w));
            if (_parent[i] == i)
            {
                break;
            }
        }

        cells.Reverse();
        cells[0] = (start.X, start.Y);
        if (goalIndex == (gy * w) + gx && costmap.WorldToCell(goal.X, goal.Y, out var ogx, out var ogy) && ogx == gx && ogy == gy)
        {
            cells[^1] = (goal.X, goal.Y);
        }

        var poses = PathUtils.OrientAlongPath(cells, goal.Theta);
        return new PlanResult(true, AnyAngle ? PathUtils.Densify(poses, costmap.Resolution * 2) : poses);
    }

    private double StepCost(byte cost) => 1.0 + (Options.CostWeight * (cost == Costs.NoInformation ? 0.5 : cost / (double)Costs.MaxNonObstacle));

    private bool Traversable(byte cost) =>
        cost == Costs.NoInformation ? Options.AllowUnknown : cost < Options.ObstacleCost;

    private double Heuristic(int x, int y, int gx, int gy)
    {
        if (HeuristicWeight == 0)
        {
            return 0;
        }

        // Octile distance: admissible for 8-connected grids with unit/√2 steps.
        var dx = Math.Abs(x - gx);
        var dy = Math.Abs(y - gy);
        var h = AnyAngle ? Math.Sqrt((dx * dx) + (dy * dy)) : Math.Max(dx, dy) + ((Math.Sqrt(2) - 1) * Math.Min(dx, dy));
        return HeuristicWeight * h;
    }

    private bool LineOfSight(Costmap2D costmap, int x0, int y0, int x1, int y1)
    {
        var ok = true;
        OccupancyGridMap.RayTrace(x0, y0, x1, y1, (x, y) =>
        {
            // Line-of-sight shortcuts must stay out of inflation as well, so the path keeps its clearance.
            var c = costmap.GetCost(x, y);
            ok = c == Costs.NoInformation ? Options.AllowUnknown : c < Costs.MaxNonObstacle / 4;
            return ok;
        });
        return ok;
    }

    private bool FindNearestFree(Costmap2D costmap, ref int gx, ref int gy)
    {
        var r = (int)Math.Ceiling(Options.GoalTolerance / costmap.Resolution);
        var best = double.MaxValue;
        int bx = -1, by = -1;
        for (var dy = -r; dy <= r; dy++)
        {
            for (var dx = -r; dx <= r; dx++)
            {
                var x = gx + dx;
                var y = gy + dy;
                var d = (dx * dx) + (dy * dy);
                if (d < best && d <= r * r && costmap.InBounds(x, y) && Traversable(costmap.GetCost(x, y)))
                {
                    best = d;
                    bx = x;
                    by = y;
                }
            }
        }

        if (bx < 0)
        {
            return false;
        }

        gx = bx;
        gy = by;
        return true;
    }

    private void Prepare(int n)
    {
        if (_g.Length != n)
        {
            _g = new float[n];
            _parent = new int[n];
            _stamp = new int[n];
            _closed = new bool[n];
            _generation = 0;
        }

        _generation++;
    }

    private void Touch(int i, int gen)
    {
        _stamp[i] = gen;
        _g[i] = float.MaxValue;
        _closed[i] = false;
    }
}

/// <summary>A* on the 8-connected grid with the octile heuristic (Nav2 NavFn with <c>use_astar: true</c>).</summary>
public sealed class AStarPlanner(GridPlannerOptions? options = null) : GridPlannerBase(options)
{
    public override string Name => "AStar";

    protected override double HeuristicWeight => 1.0;
}

/// <summary>Dijkstra's algorithm (Nav2 NavFn default): optimal, explores uniformly.</summary>
public sealed class DijkstraPlanner(GridPlannerOptions? options = null) : GridPlannerBase(options)
{
    public override string Name => "Dijkstra";

    protected override double HeuristicWeight => 0.0;
}

/// <summary>Theta*: A* whose parents may be any visible ancestor, giving shorter any-angle paths.</summary>
public sealed class ThetaStarPlanner(GridPlannerOptions? options = null) : GridPlannerBase(options)
{
    public override string Name => "ThetaStar";

    protected override double HeuristicWeight => 1.0;

    protected override bool AnyAngle => true;
}
