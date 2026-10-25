namespace ROS2.Net.Native;

/// <summary>ICP outcome. <see cref="Transform"/> is a row-major 4×4 matrix mapping source onto target.</summary>
public sealed record IcpResult(double[] Transform, double Rmse, int Iterations, bool Converged, int Correspondences);

/// <summary>A plane <c>a·x + b·y + c·z + d = 0</c> with a unit normal, plus its inlier count.</summary>
public readonly record struct PlaneFit(double A, double B, double C, double D, int InlierCount);

/// <summary>Geometry of a 2D grid map: cell (0,0) has its lower-left corner at the origin.</summary>
public readonly record struct OccupancyGridSpec(int Width, int Height, double Resolution, double OriginX, double OriginY);

/// <summary>Log-odds increments and clamping bounds for occupancy updates (defaults ≈ p 0.4 free / 0.7 occupied).</summary>
public readonly record struct LogOddsModel(float Free = -0.4f, float Occupied = 0.85f, float Min = -2f, float Max = 3.5f)
{
    public LogOddsModel()
        : this(-0.4f, 0.85f, -2f, 3.5f)
    {
    }

    /// <summary>Converts a log-odds value to an occupancy probability.</summary>
    public static float ToProbability(float logOdds) => 1f - (1f / (1f + MathF.Exp(logOdds)));
}
