using ROS2.Net.Msgs.SensorMsgs;

namespace ROS2.Net.Navigation.Slam;

/// <summary>Converts laser scans to 2-D points.</summary>
public static class ScanPoints
{
    /// <summary>Valid returns of <paramref name="scan"/> as points in the robot frame, keeping every <paramref name="stride"/>-th beam.</summary>
    public static (double X, double Y)[] FromScan(LaserScan scan, Pose2D laserOffset = default, double maxRange = double.MaxValue, int stride = 1)
    {
        var list = new List<(double, double)>(scan.Ranges.Length / Math.Max(1, stride));
        var range = Math.Min(maxRange, scan.RangeMax);
        for (var i = 0; i < scan.Ranges.Length; i += Math.Max(1, stride))
        {
            var r = scan.Ranges[i];
            if (!float.IsFinite(r) || r < scan.RangeMin || r >= range)
            {
                continue;
            }

            var a = scan.AngleMin + (i * scan.AngleIncrement);
            list.Add(laserOffset.TransformPoint(r * Math.Cos(a), r * Math.Sin(a)));
        }

        return list.ToArray();
    }
}

/// <summary>
/// A likelihood grid for correlative matching: each hit is splatted with a Gaussian, so the value
/// at a cell is the probability that a point there lies on a surface.
/// </summary>
public sealed class MatchGrid
{
    private MatchGrid(double originX, double originY, double resolution, int width, int height, float[] values)
    {
        OriginX = originX;
        OriginY = originY;
        Resolution = resolution;
        Width = width;
        Height = height;
        Values = values;
    }

    public double OriginX { get; }

    public double OriginY { get; }

    public double Resolution { get; }

    public int Width { get; }

    public int Height { get; }

    public float[] Values { get; }

    /// <summary>Builds a grid from world-frame points.</summary>
    public static MatchGrid Build(IReadOnlyList<(double X, double Y)> points, double resolution, double sigma, double margin = 2.0)
    {
        if (points.Count == 0)
        {
            return new MatchGrid(0, 0, resolution, 1, 1, new float[1]);
        }

        double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
        foreach (var (x, y) in points)
        {
            minX = Math.Min(minX, x);
            minY = Math.Min(minY, y);
            maxX = Math.Max(maxX, x);
            maxY = Math.Max(maxY, y);
        }

        minX -= margin;
        minY -= margin;
        var w = (int)Math.Ceiling((maxX + margin - minX) / resolution) + 1;
        var h = (int)Math.Ceiling((maxY + margin - minY) / resolution) + 1;
        var values = new float[w * h];
        var k = Math.Max(1, (int)Math.Ceiling(2.5 * sigma / resolution));
        var kernel = new float[((2 * k) + 1) * ((2 * k) + 1)];
        for (var dy = -k; dy <= k; dy++)
        {
            for (var dx = -k; dx <= k; dx++)
            {
                var d2 = ((dx * dx) + (dy * dy)) * resolution * resolution;
                kernel[((dy + k) * ((2 * k) + 1)) + dx + k] = (float)Math.Exp(-d2 / (2 * sigma * sigma));
            }
        }

        var kw = (2 * k) + 1;
        foreach (var (x, y) in points)
        {
            var cx = (int)((x - minX) / resolution);
            var cy = (int)((y - minY) / resolution);
            for (var dy = -k; dy <= k; dy++)
            {
                var yy = cy + dy;
                if ((uint)yy >= (uint)h)
                {
                    continue;
                }

                for (var dx = -k; dx <= k; dx++)
                {
                    var xx = cx + dx;
                    if ((uint)xx >= (uint)w)
                    {
                        continue;
                    }

                    var v = kernel[((dy + k) * kw) + dx + k];
                    ref var cell = ref values[(yy * w) + xx];
                    if (v > cell)
                    {
                        cell = v;
                    }
                }
            }
        }

        return new MatchGrid(minX, minY, resolution, w, h, values);
    }

    /// <summary>A coarser grid where each cell holds the maximum of the <paramref name="factor"/>² fine cells (branch-and-bound style upper bound).</summary>
    public MatchGrid Downsample(int factor)
    {
        var w = (Width + factor - 1) / factor;
        var h = (Height + factor - 1) / factor;
        var values = new float[w * h];
        for (var y = 0; y < Height; y++)
        {
            var row = (y / factor) * w;
            for (var x = 0; x < Width; x++)
            {
                ref var cell = ref values[row + (x / factor)];
                var v = Values[(y * Width) + x];
                if (v > cell)
                {
                    cell = v;
                }
            }
        }

        // Dilate by one coarse cell so sub-cell offsets cannot make the bound miss the true peak.
        var dilated = new float[w * h];
        for (var y = 0; y < h; y++)
        {
            for (var x = 0; x < w; x++)
            {
                var m = 0f;
                for (var dy = -1; dy <= 1; dy++)
                {
                    for (var dx = -1; dx <= 1; dx++)
                    {
                        var xx = x + dx;
                        var yy = y + dy;
                        if ((uint)xx < (uint)w && (uint)yy < (uint)h)
                        {
                            m = Math.Max(m, values[(yy * w) + xx]);
                        }
                    }
                }

                dilated[(y * w) + x] = m;
            }
        }

        return new MatchGrid(OriginX, OriginY, Resolution * factor, w, h, dilated);
    }

    /// <summary>Bilinearly interpolated value between cell centers, for sub-cell refinement.</summary>
    public double ValueAtBilinear(double x, double y)
    {
        var fx = ((x - OriginX) / Resolution) - 0.5;
        var fy = ((y - OriginY) / Resolution) - 0.5;
        var x0 = (int)Math.Floor(fx);
        var y0 = (int)Math.Floor(fy);
        var ax = fx - x0;
        var ay = fy - y0;
        return ((1 - ax) * (1 - ay) * Cell(x0, y0)) + (ax * (1 - ay) * Cell(x0 + 1, y0)) +
               ((1 - ax) * ay * Cell(x0, y0 + 1)) + (ax * ay * Cell(x0 + 1, y0 + 1));
    }

    private float Cell(int cx, int cy) => (uint)cx < (uint)Width && (uint)cy < (uint)Height ? Values[(cy * Width) + cx] : 0f;

    public float ValueAt(double x, double y)
    {
        var cx = (int)Math.Floor((x - OriginX) / Resolution);
        var cy = (int)Math.Floor((y - OriginY) / Resolution);
        return (uint)cx < (uint)Width && (uint)cy < (uint)Height ? Values[(cy * Width) + cx] : 0f;
    }
}

/// <summary>Result of a scan match: the best pose and its score (mean point likelihood, 0..1).</summary>
public readonly record struct MatchResult(Pose2D Pose, double Score);

/// <summary>
/// Multi-resolution correlative scan matcher (Olson, 2009): exhaustively searches a window of
/// translations and rotations on a coarse upper-bound grid, then refines on the fine grid.
/// </summary>
public sealed class CorrelativeScanMatcher
{
    public int CoarseFactor { get; set; } = 4;

    /// <summary>Matches <paramref name="points"/> (robot frame) against <paramref name="grid"/> around <paramref name="guess"/>.</summary>
    public MatchResult Match(MatchGrid grid, IReadOnlyList<(double X, double Y)> points, Pose2D guess, double linearWindow, double angularWindow, double angularStep)
    {
        if (points.Count == 0)
        {
            return new MatchResult(guess, 0);
        }

        var coarse = grid.Downsample(CoarseFactor);
        var best = Search(coarse, points, guess, linearWindow, angularWindow, angularStep, coarse.Resolution);

        // Fine pass around the coarse optimum.
        var fine = Search(grid, points, best.Pose, coarse.Resolution, angularStep * 5, angularStep, grid.Resolution);
        var refined = Search(grid, points, fine.Pose, grid.Resolution, angularStep, angularStep / 4, grid.Resolution / 5, interpolate: true);
        return refined.Score >= fine.Score ? refined : fine;
    }

    /// <summary>Mean likelihood of the points placed at <paramref name="pose"/>.</summary>
    public static double Score(MatchGrid grid, IReadOnlyList<(double X, double Y)> points, Pose2D pose)
    {
        var c = Math.Cos(pose.Theta);
        var s = Math.Sin(pose.Theta);
        var sum = 0.0;
        foreach (var (px, py) in points)
        {
            sum += grid.ValueAt(pose.X + (c * px) - (s * py), pose.Y + (s * px) + (c * py));
        }

        return sum / points.Count;
    }

    private static MatchResult Search(MatchGrid grid, IReadOnlyList<(double X, double Y)> points, Pose2D center, double linearWindow, double angularWindow, double angularStep, double linearStep, bool interpolate = false)
    {
        var n = points.Count;
        var rotatedX = new double[n];
        var rotatedY = new double[n];
        var best = new MatchResult(center, -1);
        var steps = Math.Max(0, (int)Math.Round(linearWindow / linearStep));
        var aSteps = Math.Max(0, (int)Math.Round(angularWindow / angularStep));
        var inv = 1.0 / grid.Resolution;
        var w = grid.Width;
        var h = grid.Height;
        var values = grid.Values;
        for (var ai = -aSteps; ai <= aSteps; ai++)
        {
            var theta = center.Theta + (ai * angularStep);
            var c = Math.Cos(theta);
            var s = Math.Sin(theta);
            for (var i = 0; i < n; i++)
            {
                rotatedX[i] = (c * points[i].X) - (s * points[i].Y);
                rotatedY[i] = (s * points[i].X) + (c * points[i].Y);
            }

            for (var yi = -steps; yi <= steps; yi++)
            {
                var ty = center.Y + (yi * linearStep);
                for (var xi = -steps; xi <= steps; xi++)
                {
                    var tx = center.X + (xi * linearStep);
                    var sum = 0.0;
                    for (var i = 0; i < n && interpolate; i++)
                    {
                        sum += grid.ValueAtBilinear(tx + rotatedX[i], ty + rotatedY[i]);
                    }

                    for (var i = 0; i < n && !interpolate; i++)
                    {
                        var cx = (int)Math.Floor((tx + rotatedX[i] - grid.OriginX) * inv);
                        var cy = (int)Math.Floor((ty + rotatedY[i] - grid.OriginY) * inv);
                        if ((uint)cx < (uint)w && (uint)cy < (uint)h)
                        {
                            sum += values[(cy * w) + cx];
                        }
                    }

                    // Prefer the solution closest to the guess on ties (penalize distance very slightly).
                    var score = (sum / n) - (1e-4 * ((xi * xi) + (yi * yi) + (ai * ai)) / Math.Max(1, (steps * steps) + (aSteps * aSteps)));
                    if (score > best.Score)
                    {
                        best = new MatchResult(new Pose2D(tx, ty, Angles.Normalize(theta)), score);
                    }
                }
            }
        }

        return best;
    }
}
