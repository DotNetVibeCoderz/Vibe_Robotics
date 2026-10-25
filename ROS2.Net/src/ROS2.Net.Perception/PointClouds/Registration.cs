using System.Numerics;
using ROS2.Net.Native;

namespace ROS2.Net.Perception;

/// <summary>ICP error metric.</summary>
public enum IcpMethod
{
    /// <summary>Minimizes point-to-point distances (closed form, native when available).</summary>
    PointToPoint,
    /// <summary>Minimizes distances along target normals; converges faster on surfaces (managed).</summary>
    PointToPlane,
}

/// <summary>ICP settings.</summary>
public sealed record IcpOptions
{
    public IcpMethod Method { get; init; } = IcpMethod.PointToPoint;

    public int MaxIterations { get; init; } = 50;

    /// <summary>Pairs farther apart than this are ignored.</summary>
    public float MaxCorrespondenceDistance { get; init; } = 1f;

    /// <summary>Stop when the incremental rotation (rad) and translation (m) fall below this.</summary>
    public double Tolerance { get; init; } = 1e-6;

    /// <summary>Neighbours used to estimate target normals for point-to-plane.</summary>
    public int NormalNeighbors { get; init; } = 12;

    /// <summary>Initial guess (row-major 4×4), identity when null.</summary>
    public double[]? InitialTransform { get; init; }
}

/// <summary>Point cloud registration.</summary>
public static class Registration
{
    /// <summary>Aligns <paramref name="source"/> onto <paramref name="target"/>; the result maps source points into the target frame.</summary>
    public static IcpResult Icp(PointCloud source, PointCloud target, IcpOptions? options = null)
    {
        options ??= new IcpOptions();
        var src = source.Xyz.ToArray();
        if (options.InitialTransform is { } init)
        {
            src = PointCloud.FromXyz(src).Transform(init).Xyz.ToArray();
        }

        var result = options.Method == IcpMethod.PointToPoint
            ? PointKernels.Icp(src, target.Xyz, options.MaxIterations, options.MaxCorrespondenceDistance, options.Tolerance)
            : PointToPlane(src, target, options);

        return options.InitialTransform is { } t0 ? result with { Transform = Multiply(result.Transform, t0) } : result;
    }

    private static IcpResult PointToPlane(float[] src, PointCloud target, IcpOptions o)
    {
        var dst = target.Xyz.ToArray();
        var normals = target.HasNormals ? target.Normals.ToArray() : Normals.Estimate(target.Clone(), o.NormalNeighbors);
        using var tree = new KdTree(dst);
        var t = Identity();
        var n = src.Length / 3;
        var max2 = o.MaxCorrespondenceDistance * o.MaxCorrespondenceDistance;
        var result = new IcpResult(t, double.PositiveInfinity, 0, false, 0);
        for (var it = 0; it < o.MaxIterations; it++)
        {
            var ata = new double[36];
            var atb = new double[6];
            double err = 0;
            var pairs = 0;
            for (var i = 0; i < n; i++)
            {
                double x = src[3 * i], y = src[(3 * i) + 1], z = src[(3 * i) + 2];
                if (!double.IsFinite(x + y + z))
                {
                    continue;
                }

                var px = (t[0] * x) + (t[1] * y) + (t[2] * z) + t[3];
                var py = (t[4] * x) + (t[5] * y) + (t[6] * z) + t[7];
                var pz = (t[8] * x) + (t[9] * y) + (t[10] * z) + t[11];
                var nn = tree.Nearest((float)px, (float)py, (float)pz, 1);
                if (nn.Length == 0 || nn[0].Distance2 > max2)
                {
                    continue;
                }

                var j = nn[0].Index;
                double nx = normals[3 * j], ny = normals[(3 * j) + 1], nz = normals[(3 * j) + 2];
                if (!double.IsFinite(nx + ny + nz))
                {
                    continue;
                }

                // Linearized residual: n·(R p + t − q) with small angles.
                Span<double> a = [(py * nz) - (pz * ny), (pz * nx) - (px * nz), (px * ny) - (py * nx), nx, ny, nz];
                var b = (nx * (dst[3 * j] - px)) + (ny * (dst[(3 * j) + 1] - py)) + (nz * (dst[(3 * j) + 2] - pz));
                for (var r = 0; r < 6; r++)
                {
                    atb[r] += a[r] * b;
                    for (var c = 0; c < 6; c++)
                    {
                        ata[(r * 6) + c] += a[r] * a[c];
                    }
                }

                err += b * b;
                pairs++;
            }

            result = result with { Iterations = it + 1, Correspondences = pairs };
            if (pairs < 6 || !Solve6(ata, atb, out var xv))
            {
                break;
            }

            result = result with { Rmse = Math.Sqrt(err / pairs) };
            var delta = FromRpyTranslation(xv[0], xv[1], xv[2], xv[3], xv[4], xv[5]);
            t = Multiply(delta, t);
            result = result with { Transform = t };
            var angle = Math.Sqrt((xv[0] * xv[0]) + (xv[1] * xv[1]) + (xv[2] * xv[2]));
            var shift = Math.Sqrt((xv[3] * xv[3]) + (xv[4] * xv[4]) + (xv[5] * xv[5]));
            if (angle < o.Tolerance && shift < o.Tolerance)
            {
                result = result with { Converged = true };
                break;
            }
        }

        return result;
    }

    private static bool Solve6(double[] a, double[] b, out double[] x)
    {
        // Gaussian elimination with partial pivoting on the 6×6 normal equations.
        var m = (double[])a.Clone();
        x = (double[])b.Clone();
        for (var col = 0; col < 6; col++)
        {
            var piv = col;
            for (var r = col + 1; r < 6; r++)
            {
                if (Math.Abs(m[(r * 6) + col]) > Math.Abs(m[(piv * 6) + col]))
                {
                    piv = r;
                }
            }

            if (Math.Abs(m[(piv * 6) + col]) < 1e-12)
            {
                return false;
            }

            if (piv != col)
            {
                for (var c = 0; c < 6; c++)
                {
                    (m[(col * 6) + c], m[(piv * 6) + c]) = (m[(piv * 6) + c], m[(col * 6) + c]);
                }

                (x[col], x[piv]) = (x[piv], x[col]);
            }

            for (var r = 0; r < 6; r++)
            {
                if (r == col)
                {
                    continue;
                }

                var f = m[(r * 6) + col] / m[(col * 6) + col];
                for (var c = col; c < 6; c++)
                {
                    m[(r * 6) + c] -= f * m[(col * 6) + c];
                }

                x[r] -= f * x[col];
            }
        }

        for (var r = 0; r < 6; r++)
        {
            x[r] /= m[(r * 6) + r];
        }

        return true;
    }

    public static double[] Identity() => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];

    /// <summary>Row-major 4×4 product a·b.</summary>
    public static double[] Multiply(double[] a, double[] b)
    {
        var r = new double[16];
        for (var i = 0; i < 4; i++)
        {
            for (var j = 0; j < 4; j++)
            {
                double s = 0;
                for (var k = 0; k < 4; k++)
                {
                    s += a[(i * 4) + k] * b[(k * 4) + j];
                }

                r[(i * 4) + j] = s;
            }
        }

        return r;
    }

    /// <summary>Row-major 4×4 transform from roll/pitch/yaw (rad, R = Rz·Ry·Rx) and a translation.</summary>
    public static double[] FromRpyTranslation(double roll, double pitch, double yaw, double tx, double ty, double tz)
    {
        double cr = Math.Cos(roll), sr = Math.Sin(roll), cp = Math.Cos(pitch), sp = Math.Sin(pitch), cy = Math.Cos(yaw), sy = Math.Sin(yaw);
        return
        [
            cy * cp, (cy * sp * sr) - (sy * cr), (cy * sp * cr) + (sy * sr), tx,
            sy * cp, (sy * sp * sr) + (cy * cr), (sy * sp * cr) - (cy * sr), ty,
            -sp, cp * sr, cp * cr, tz,
            0, 0, 0, 1,
        ];
    }

    /// <summary>Converts a row-major 4×4 to System.Numerics (row-vector convention).</summary>
    public static Matrix4x4 ToMatrix4x4(double[] m) => new(
        (float)m[0], (float)m[4], (float)m[8], 0,
        (float)m[1], (float)m[5], (float)m[9], 0,
        (float)m[2], (float)m[6], (float)m[10], 0,
        (float)m[3], (float)m[7], (float)m[11], 1);
}
