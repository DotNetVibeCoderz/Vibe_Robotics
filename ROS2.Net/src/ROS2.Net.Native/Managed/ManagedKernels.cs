namespace ROS2.Net.Native.Managed;

/// <summary>
/// Managed implementations of every native kernel, written to follow native/ros2net_native/src/cloud.rs
/// step for step so results match (exactly, except where transcendental functions are involved).
/// </summary>
public static class ManagedKernels
{
    private static bool Finite(ReadOnlySpan<float> xyz, int i) =>
        float.IsFinite(xyz[3 * i]) && float.IsFinite(xyz[(3 * i) + 1]) && float.IsFinite(xyz[(3 * i) + 2]);

    public static float[] VoxelDownsample(ReadOnlySpan<float> xyz, float leaf)
    {
        var n = xyz.Length / 3;
        var inv = 1.0 / leaf;
        var map = new Dictionary<(long, long, long), (double X, double Y, double Z, uint C)>();
        for (var i = 0; i < n; i++)
        {
            if (!Finite(xyz, i))
            {
                continue;
            }

            float x = xyz[3 * i], y = xyz[(3 * i) + 1], z = xyz[(3 * i) + 2];
            var key = ((long)Math.Floor(x * inv), (long)Math.Floor(y * inv), (long)Math.Floor(z * inv));
            map.TryGetValue(key, out var e);
            map[key] = (e.X + x, e.Y + y, e.Z + z, e.C + 1);
        }

        var keys = map.Keys.ToArray();
        Array.Sort(keys);
        var output = new float[keys.Length * 3];
        for (var k = 0; k < keys.Length; k++)
        {
            var (sx, sy, sz, c) = map[keys[k]];
            output[3 * k] = (float)(sx / c);
            output[(3 * k) + 1] = (float)(sy / c);
            output[(3 * k) + 2] = (float)(sz / c);
        }

        return output;
    }

    public static void StatisticalOutlier(ReadOnlySpan<float> xyz, int k, double stdMul, Span<byte> keep)
    {
        var n = xyz.Length / 3;
        var tree = new ManagedKdTree(xyz);
        var mean = new double[n];
        mean.AsSpan().Fill(double.NaN);
        var buf = new List<(float, int)>(k + 1);
        var valid = 0;
        double sum = 0, sq = 0;
        for (var i = 0; i < n; i++)
        {
            if (!Finite(xyz, i))
            {
                continue;
            }

            tree.Knn(xyz.Slice(3 * i, 3), k + 1, buf);
            double acc = 0;
            var cnt = 0;
            var skipped = false;
            foreach (var (d, j) in buf)
            {
                if (!skipped && j == i)
                {
                    skipped = true;
                    continue;
                }

                if (cnt == k)
                {
                    break;
                }

                acc += Math.Sqrt(d);
                cnt++;
            }

            if (cnt == 0)
            {
                continue;
            }

            var m = acc / cnt;
            mean[i] = m;
            sum += m;
            sq += m * m;
            valid++;
        }

        if (valid == 0)
        {
            keep[..n].Clear();
            return;
        }

        var mu = sum / valid;
        var variance = valid > 1 ? Math.Max(0.0, (sq - (sum * mu)) / (valid - 1.0)) : 0.0;
        var threshold = mu + (stdMul * Math.Sqrt(variance));
        for (var i = 0; i < n; i++)
        {
            keep[i] = double.IsFinite(mean[i]) && mean[i] <= threshold ? (byte)1 : (byte)0;
        }
    }

    public static void RadiusOutlier(ReadOnlySpan<float> xyz, float radius, int minNeighbors, Span<byte> keep)
    {
        var n = xyz.Length / 3;
        var tree = new ManagedKdTree(xyz);
        var buf = new List<(float, int)>();
        for (var i = 0; i < n; i++)
        {
            if (!Finite(xyz, i))
            {
                keep[i] = 0;
                continue;
            }

            tree.Radius(xyz.Slice(3 * i, 3), radius, buf);
            keep[i] = Math.Max(0, buf.Count - 1) >= minNeighbors ? (byte)1 : (byte)0;
        }
    }

    public static int EuclideanCluster(ReadOnlySpan<float> xyz, float tolerance, int minSize, int maxSize, Span<int> labels)
    {
        var n = xyz.Length / 3;
        var tree = new ManagedKdTree(xyz);
        var raw = new int[n];
        raw.AsSpan().Fill(-1);
        var sizes = new List<int>();
        var queue = new Queue<int>();
        var buf = new List<(float, int)>();
        for (var seed = 0; seed < n; seed++)
        {
            if (raw[seed] >= 0 || !Finite(xyz, seed))
            {
                continue;
            }

            var id = sizes.Count;
            raw[seed] = id;
            var size = 1;
            queue.Enqueue(seed);
            while (queue.TryDequeue(out var i))
            {
                tree.Radius(xyz.Slice(3 * i, 3), tolerance, buf);
                foreach (var (_, j) in buf)
                {
                    if (raw[j] < 0)
                    {
                        raw[j] = id;
                        size++;
                        queue.Enqueue(j);
                    }
                }
            }

            sizes.Add(size);
        }

        var remap = new int[sizes.Count];
        var next = 0;
        for (var c = 0; c < sizes.Count; c++)
        {
            remap[c] = sizes[c] >= minSize && sizes[c] <= maxSize ? next++ : -1;
        }

        for (var i = 0; i < n; i++)
        {
            labels[i] = raw[i] >= 0 ? remap[raw[i]] : -1;
        }

        return next;
    }

    /// <summary>xorshift64*, identical to the native generator.</summary>
    public struct Rng(ulong seed)
    {
        private ulong _state = seed == 0 ? 0x9E3779B97F4A7C15UL : seed;

        public ulong Next()
        {
            var x = _state;
            x ^= x >> 12;
            x ^= x << 25;
            x ^= x >> 27;
            _state = x;
            return unchecked(x * 0x2545F4914F6CDD1DUL);
        }
    }

    public static (int Count, double A, double B, double C, double D) RansacPlane(ReadOnlySpan<float> xyz, double threshold, int iterations, ulong seed, Span<byte> inliers)
    {
        var n = xyz.Length / 3;
        inliers[..n].Clear();
        if (n < 3)
        {
            return default;
        }

        var rng = new Rng(seed);
        (int Count, double A, double B, double C, double D) best = default;
        for (var it = 0; it < iterations; it++)
        {
            var a = (int)(rng.Next() % (ulong)n);
            var b = (int)(rng.Next() % (ulong)n);
            var c = (int)(rng.Next() % (ulong)n);
            if (a == b || b == c || a == c || !Finite(xyz, a) || !Finite(xyz, b) || !Finite(xyz, c))
            {
                continue;
            }

            double ux = xyz[3 * b] - xyz[3 * a], uy = xyz[(3 * b) + 1] - xyz[(3 * a) + 1], uz = xyz[(3 * b) + 2] - xyz[(3 * a) + 2];
            double vx = xyz[3 * c] - xyz[3 * a], vy = xyz[(3 * c) + 1] - xyz[(3 * a) + 1], vz = xyz[(3 * c) + 2] - xyz[(3 * a) + 2];

            var nx = (uy * vz) - (uz * vy);
            var ny = (uz * vx) - (ux * vz);
            var nz = (ux * vy) - (uy * vx);
            var len = Math.Sqrt((nx * nx) + (ny * ny) + (nz * nz));
            if (len < 1e-12)
            {
                continue;
            }

            nx /= len;
            ny /= len;
            nz /= len;
            var d = -((nx * xyz[3 * a]) + (ny * xyz[(3 * a) + 1]) + (nz * xyz[(3 * a) + 2]));
            var count = 0;
            for (var i = 0; i < n; i++)
            {
                if (Math.Abs((nx * xyz[3 * i]) + (ny * xyz[(3 * i) + 1]) + (nz * xyz[(3 * i) + 2]) + d) <= threshold)
                {
                    count++;
                }
            }

            if (count > best.Count)
            {
                best = (count, nx, ny, nz, d);
            }
        }

        if (best.Count > 0)
        {
            for (var i = 0; i < n; i++)
            {
                if (Math.Abs((best.A * xyz[3 * i]) + (best.B * xyz[(3 * i) + 1]) + (best.C * xyz[(3 * i) + 2]) + best.D) <= threshold)
                {
                    inliers[i] = 1;
                }
            }
        }

        return best;
    }

    /// <summary>Cyclic Jacobi eigen-decomposition of a symmetric n×n row-major matrix (destroys <paramref name="a"/>).</summary>
    public static (double[] Values, double[] Vectors) JacobiEigen(double[] a, int n)
    {
        var v = new double[n * n];
        for (var i = 0; i < n; i++)
        {
            v[(i * n) + i] = 1;
        }

        for (var sweep = 0; sweep < 64; sweep++)
        {
            double off = 0;
            for (var p = 0; p < n; p++)
            {
                for (var q = p + 1; q < n; q++)
                {
                    off += a[(p * n) + q] * a[(p * n) + q];
                }
            }

            if (off < 1e-30)
            {
                break;
            }

            for (var p = 0; p < n; p++)
            {
                for (var q = p + 1; q < n; q++)
                {
                    var apq = a[(p * n) + q];
                    if (Math.Abs(apq) < 1e-300)
                    {
                        continue;
                    }

                    var theta = (a[(q * n) + q] - a[(p * n) + p]) / (2 * apq);
                    var t = theta == 0 ? 1.0 : Math.Sign(theta) / (Math.Abs(theta) + Math.Sqrt((theta * theta) + 1));
                    var c = 1 / Math.Sqrt((t * t) + 1);
                    var s = t * c;
                    for (var k = 0; k < n; k++)
                    {
                        double akp = a[(k * n) + p], akq = a[(k * n) + q];
                        a[(k * n) + p] = (c * akp) - (s * akq);
                        a[(k * n) + q] = (s * akp) + (c * akq);
                    }

                    for (var k = 0; k < n; k++)
                    {
                        double apk = a[(p * n) + k], aqk = a[(q * n) + k];
                        a[(p * n) + k] = (c * apk) - (s * aqk);
                        a[(q * n) + k] = (s * apk) + (c * aqk);
                    }

                    for (var k = 0; k < n; k++)
                    {
                        double vkp = v[(k * n) + p], vkq = v[(k * n) + q];
                        v[(k * n) + p] = (c * vkp) - (s * vkq);
                        v[(k * n) + q] = (s * vkp) + (c * vkq);
                    }
                }
            }
        }

        var values = new double[n];
        for (var i = 0; i < n; i++)
        {
            values[i] = a[(i * n) + i];
        }

        return (values, v);
    }

    /// <summary>Least-squares rigid transform aligning paired points (Horn's quaternion method). Rotation is row-major 3×3.</summary>
    public static (double[] R, double[] T) BestFit(IReadOnlyList<(double X, double Y, double Z)> src, IReadOnlyList<(double X, double Y, double Z)> dst)
    {
        var m = (double)src.Count;
        double csx = 0, csy = 0, csz = 0, cdx = 0, cdy = 0, cdz = 0;
        for (var i = 0; i < src.Count; i++)
        {
            csx += src[i].X;
            csy += src[i].Y;
            csz += src[i].Z;
            cdx += dst[i].X;
            cdy += dst[i].Y;
            cdz += dst[i].Z;
        }

        csx /= m;
        csy /= m;
        csz /= m;
        cdx /= m;
        cdy /= m;
        cdz /= m;
        var s = new double[9];
        for (var i = 0; i < src.Count; i++)
        {
            Span<double> a = [src[i].X - csx, src[i].Y - csy, src[i].Z - csz];
            Span<double> b = [dst[i].X - cdx, dst[i].Y - cdy, dst[i].Z - cdz];
            for (var r = 0; r < 3; r++)
            {
                for (var c = 0; c < 3; c++)
                {
                    s[(r * 3) + c] += a[r] * b[c];
                }
            }
        }

        double sxx = s[0], sxy = s[1], sxz = s[2], syx = s[3], syy = s[4], syz = s[5], szx = s[6], szy = s[7], szz = s[8];
        double[] nm =
        [
            sxx + syy + szz, syz - szy, szx - sxz, sxy - syx,
            syz - szy, sxx - syy - szz, sxy + syx, szx + sxz,
            szx - sxz, sxy + syx, -sxx + syy - szz, syz + szy,
            sxy - syx, szx + sxz, syz + szy, -sxx - syy + szz,
        ];
        var (vals, vecs) = JacobiEigen(nm, 4);
        var best = 0;
        for (var i = 1; i < 4; i++)
        {
            if (vals[i] > vals[best])
            {
                best = i;
            }
        }

        double w = vecs[best], x = vecs[4 + best], y = vecs[8 + best], z = vecs[12 + best];
        double[] rot =
        [
            (w * w) + (x * x) - (y * y) - (z * z), 2 * ((x * y) - (w * z)), 2 * ((x * z) + (w * y)),
            2 * ((x * y) + (w * z)), (w * w) - (x * x) + (y * y) - (z * z), 2 * ((y * z) - (w * x)),
            2 * ((x * z) - (w * y)), 2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z),
        ];
        double[] t =
        [
            cdx - ((rot[0] * csx) + (rot[1] * csy) + (rot[2] * csz)),
            cdy - ((rot[3] * csx) + (rot[4] * csy) + (rot[5] * csz)),
            cdz - ((rot[6] * csx) + (rot[7] * csy) + (rot[8] * csz)),
        ];
        return (rot, t);
    }

    public static IcpResult Icp(ReadOnlySpan<float> src, ReadOnlySpan<float> dst, int maxIterations, float maxCorrespondence, double tolerance)
    {
        var tree = new ManagedKdTree(dst);
        var pts = tree.Points;
        var ns = src.Length / 3;
        double[] t = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
        var result = new IcpResult((double[])t.Clone(), double.PositiveInfinity, 0, false, 0);
        var max2 = maxCorrespondence * maxCorrespondence;
        var buf = new List<(float, int)>(1);
        var ps = new List<(double, double, double)>(ns);
        var pd = new List<(double, double, double)>(ns);
        Span<float> qf = stackalloc float[3];
        for (var it = 0; it < maxIterations; it++)
        {
            ps.Clear();
            pd.Clear();
            double err = 0;
            for (var i = 0; i < ns; i++)
            {
                if (!Finite(src, i))
                {
                    continue;
                }

                double x = src[3 * i], y = src[(3 * i) + 1], z = src[(3 * i) + 2];
                var qx = (t[0] * x) + (t[1] * y) + (t[2] * z) + t[3];
                var qy = (t[4] * x) + (t[5] * y) + (t[6] * z) + t[7];
                var qz = (t[8] * x) + (t[9] * y) + (t[10] * z) + t[11];
                qf[0] = (float)qx;
                qf[1] = (float)qy;
                qf[2] = (float)qz;
                tree.Knn(qf, 1, buf);
                if (buf.Count > 0 && buf[0].Item1 <= max2)
                {
                    var j = buf[0].Item2;
                    ps.Add((qx, qy, qz));
                    pd.Add((pts[3 * j], pts[(3 * j) + 1], pts[(3 * j) + 2]));
                    err += buf[0].Item1;
                }
            }

            result = result with { Iterations = it + 1, Correspondences = ps.Count };
            if (ps.Count < 3)
            {
                break;
            }

            result = result with { Rmse = Math.Sqrt(err / ps.Count) };
            var (r, tr) = BestFit(ps, pd);
            var next = new double[16];
            for (var row = 0; row < 3; row++)
            {
                for (var col = 0; col < 4; col++)
                {
                    next[(row * 4) + col] = (r[row * 3] * t[col]) + (r[(row * 3) + 1] * t[4 + col]) + (r[(row * 3) + 2] * t[8 + col])
                        + (col == 3 ? tr[row] : 0.0);
                }
            }

            next[15] = 1;
            t = next;
            result = result with { Transform = (double[])t.Clone() };
            var angle = Math.Acos(Math.Clamp((r[0] + r[4] + r[8] - 1) * 0.5, -1, 1));
            var shift = Math.Sqrt((tr[0] * tr[0]) + (tr[1] * tr[1]) + (tr[2] * tr[2]));
            if (angle < tolerance && shift < tolerance)
            {
                result = result with { Converged = true };
                break;
            }
        }

        return result;
    }

    public static int ProjectScan(ReadOnlySpan<float> ranges, float angleMin, float angleIncrement, float rangeMin, float rangeMax, Span<float> output)
    {
        var n = 0;
        for (var i = 0; i < ranges.Length; i++)
        {
            var r = ranges[i];
            if (!float.IsFinite(r) || r < rangeMin || r > rangeMax)
            {
                continue;
            }

            var a = angleMin + (i * (double)angleIncrement);
            output[3 * n] = (float)(r * Math.Cos(a));
            output[(3 * n) + 1] = (float)(r * Math.Sin(a));
            output[(3 * n) + 2] = 0;
            n++;
        }

        return n;
    }

    public static void OccupancyUpdate(Span<float> grid, in OccupancyGridSpec spec, double sensorX, double sensorY, ReadOnlySpan<float> pointsXy, in LogOddsModel model)
    {
        long width = spec.Width, height = spec.Height;
        var res = spec.Resolution;
        var ox = spec.OriginX;
        var oy = spec.OriginY;
        float free = model.Free, occupied = model.Occupied, min = model.Min, max = model.Max;
        var sx = (long)Math.Floor((sensorX - ox) / res);
        var sy = (long)Math.Floor((sensorY - oy) / res);
        for (var i = 0; i < pointsXy.Length / 2; i++)
        {
            double px = pointsXy[2 * i], py = pointsXy[(2 * i) + 1];
            if (!double.IsFinite(px) || !double.IsFinite(py))
            {
                continue;
            }

            var ex = (long)Math.Floor((px - ox) / res);
            var ey = (long)Math.Floor((py - oy) / res);
            long x = sx, y = sy;
            var dx = Math.Abs(ex - sx);
            var dy = -Math.Abs(ey - sy);
            var stx = sx < ex ? 1 : -1;
            var sty = sy < ey ? 1 : -1;
            var err = dx + dy;
            while (!(x == ex && y == ey))
            {
                Add(grid, x, y, free);
                var e2 = 2 * err;
                if (e2 >= dy)
                {
                    err += dy;
                    x += stx;
                }

                if (e2 <= dx)
                {
                    err += dx;
                    y += sty;
                }
            }

            Add(grid, ex, ey, occupied);
        }

        void Add(Span<float> g, long cx, long cy, float v)
        {
            if (cx >= 0 && cy >= 0 && cx < width && cy < height)
            {
                var k = (int)((cy * width) + cx);
                g[k] = Math.Clamp(g[k] + v, min, max);
            }
        }
    }
}
