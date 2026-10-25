using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Navigation.Costmaps;

namespace ROS2.Net.Navigation.Planning;

/// <summary>Outcome of a planning request.</summary>
public sealed record PlanResult(bool Success, IReadOnlyList<Pose2D> Poses, string? Error = null)
{
    public static PlanResult Fail(string error) => new(false, [], error);

    /// <summary>Path length in meters.</summary>
    public double Length => PathUtils.Length(Poses);
}

/// <summary>A global planner (Nav2 planner plugin equivalent).</summary>
public interface IGlobalPlanner
{
    string Name { get; }

    PlanResult CreatePlan(Costmap2D costmap, Pose2D start, Pose2D goal, CancellationToken cancellationToken = default);
}

/// <summary>Settings shared by the grid planners.</summary>
public sealed record GridPlannerOptions
{
    /// <summary>Allow planning through unknown (255) cells.</summary>
    public bool AllowUnknown { get; init; } = true;

    /// <summary>How strongly inflation cost increases the traversal cost (0 = shortest path).</summary>
    public double CostWeight { get; init; } = 3.0;

    /// <summary>Cells with cost at or above this are not traversable.</summary>
    public byte ObstacleCost { get; init; } = Costs.Inscribed;

    /// <summary>When the goal cell is blocked, accept the nearest free cell within this many meters.</summary>
    public double GoalTolerance { get; init; } = 0.5;

    /// <summary>Upper bound on expanded cells, to keep worst cases bounded.</summary>
    public int MaxIterations { get; init; } = 5_000_000;
}

/// <summary>Path helpers.</summary>
public static class PathUtils
{
    public static double Length(IReadOnlyList<Pose2D> poses)
    {
        var len = 0.0;
        for (var i = 1; i < poses.Count; i++)
        {
            len += poses[i - 1].DistanceTo(poses[i]);
        }

        return len;
    }

    /// <summary>Sets each pose's heading to point at the next one; the last keeps <paramref name="finalTheta"/>.</summary>
    public static List<Pose2D> OrientAlongPath(IReadOnlyList<(double X, double Y)> points, double finalTheta)
    {
        var result = new List<Pose2D>(points.Count);
        for (var i = 0; i < points.Count; i++)
        {
            double theta;
            if (i + 1 < points.Count)
            {
                theta = Math.Atan2(points[i + 1].Y - points[i].Y, points[i + 1].X - points[i].X);
            }
            else
            {
                theta = finalTheta;
            }

            result.Add(new Pose2D(points[i].X, points[i].Y, theta));
        }

        return result;
    }

    /// <summary>Resamples a path so consecutive poses are at most <paramref name="spacing"/> apart.</summary>
    public static List<Pose2D> Densify(IReadOnlyList<Pose2D> poses, double spacing)
    {
        var result = new List<Pose2D>();
        if (poses.Count == 0)
        {
            return result;
        }

        result.Add(poses[0]);
        for (var i = 1; i < poses.Count; i++)
        {
            var a = poses[i - 1];
            var b = poses[i];
            var d = a.DistanceTo(b);
            var steps = (int)Math.Ceiling(d / spacing);
            for (var s = 1; s <= steps; s++)
            {
                var t = (double)s / steps;
                result.Add(s == steps ? b : new Pose2D(a.X + ((b.X - a.X) * t), a.Y + ((b.Y - a.Y) * t), b.Theta));
            }
        }

        return result;
    }

    public static Msgs.NavMsgs.Path ToMsg(IReadOnlyList<Pose2D> poses, string frameId, RosTime stamp)
    {
        var path = new Msgs.NavMsgs.Path { Poses = new PoseStamped[poses.Count] };
        stamp.CopyTo(path.Header.Stamp);
        path.Header.FrameId = frameId;
        for (var i = 0; i < poses.Count; i++)
        {
            path.Poses[i] = poses[i].ToPoseStamped(frameId, stamp);
        }

        return path;
    }

    public static List<Pose2D> FromMsg(Msgs.NavMsgs.Path path)
    {
        var list = new List<Pose2D>(path.Poses.Length);
        foreach (var p in path.Poses)
        {
            list.Add(Pose2D.FromMsg(p.Pose));
        }

        return list;
    }

    /// <summary>True when every pose of the path is collision-free.</summary>
    public static bool IsCollisionFree(Costmap2D costmap, Footprint footprint, IReadOnlyList<Pose2D> poses)
    {
        foreach (var p in poses)
        {
            if (footprint.InCollision(costmap, p))
            {
                return false;
            }
        }

        return true;
    }
}
