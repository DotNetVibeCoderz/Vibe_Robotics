namespace ROS2.Net.Native.Managed;

/// <summary>
/// Managed twin of the Rust k-d tree. Results are ordered by (squared distance, index), so they are
/// identical to the native implementation; non-finite points are never returned.
/// </summary>
public sealed class ManagedKdTree
{
    private readonly float[] _xyz;
    private readonly int[] _idx;
    private readonly byte[] _axis;

    public ManagedKdTree(ReadOnlySpan<float> xyz)
    {
        _xyz = xyz[..(xyz.Length / 3 * 3)].ToArray();
        var n = _xyz.Length / 3;
        var idx = new List<int>(n);
        for (var i = 0; i < n; i++)
        {
            if (float.IsFinite(_xyz[3 * i]) && float.IsFinite(_xyz[(3 * i) + 1]) && float.IsFinite(_xyz[(3 * i) + 2]))
            {
                idx.Add(i);
            }
        }

        _idx = idx.ToArray();
        _axis = new byte[_idx.Length];
        Build(0, _idx.Length);
    }

    public int Count => _xyz.Length / 3;

    internal ReadOnlySpan<float> Points => _xyz;

    internal static float Dist2(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
    {
        var dx = a[0] - b[0];
        var dy = a[1] - b[1];
        var dz = a[2] - b[2];
        return (dx * dx) + (dy * dy) + (dz * dz);
    }

    private ReadOnlySpan<float> P(int i) => _xyz.AsSpan(3 * i, 3);

    private static bool Better(float d, int i, float od, int oi) => d < od || (d == od && i < oi);

    private void Build(int lo, int hi)
    {
        if (hi <= lo + 1)
        {
            return;
        }

        Span<float> mn = [float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity];
        Span<float> mx = [float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity];
        for (var k = lo; k < hi; k++)
        {
            var p = P(_idx[k]);
            for (var a = 0; a < 3; a++)
            {
                mn[a] = Math.Min(mn[a], p[a]);
                mx[a] = Math.Max(mx[a], p[a]);
            }
        }

        var ax = 0;
        for (var a = 1; a < 3; a++)
        {
            if (mx[a] - mn[a] > mx[ax] - mn[ax])
            {
                ax = a;
            }
        }

        var xyz = _xyz;
        Array.Sort(_idx, lo, hi - lo, Comparer<int>.Create((a, b) =>
        {
            var c = xyz[(3 * a) + ax].CompareTo(xyz[(3 * b) + ax]);
            return c != 0 ? c : a.CompareTo(b);
        }));
        var mid = (lo + hi) / 2;
        _axis[mid] = (byte)ax;
        Build(lo, mid);
        Build(mid + 1, hi);
    }

    /// <summary>k nearest neighbours of <paramref name="query"/>, sorted by (distance², index).</summary>
    public List<(float Dist2, int Index)> Knn(ReadOnlySpan<float> query, int k)
    {
        var result = new List<(float, int)>(k);
        Knn(query, k, result);
        return result;
    }

    public void Knn(ReadOnlySpan<float> query, int k, List<(float Dist2, int Index)> result)
    {
        result.Clear();
        if (k <= 0)
        {
            return;
        }

        KnnRec(0, _idx.Length, query, k, result);
        result.Sort(static (a, b) => a.Dist2 != b.Dist2 ? a.Dist2.CompareTo(b.Dist2) : a.Index.CompareTo(b.Index));
    }

    private static int Worst(List<(float D, int I)> r)
    {
        var w = 0;
        for (var j = 1; j < r.Count; j++)
        {
            if (Better(r[w].D, r[w].I, r[j].D, r[j].I))
            {
                w = j;
            }
        }

        return w;
    }

    private void KnnRec(int lo, int hi, ReadOnlySpan<float> q, int k, List<(float D, int I)> r)
    {
        if (hi <= lo)
        {
            return;
        }

        var mid = (lo + hi) / 2;
        var i = _idx[mid];
        var p = P(i);
        var d = Dist2(p, q);
        if (r.Count < k)
        {
            r.Add((d, i));
        }
        else
        {
            var w = Worst(r);
            if (Better(d, i, r[w].D, r[w].I))
            {
                r[w] = (d, i);
            }
        }

        if (hi == lo + 1)
        {
            return;
        }

        var ax = _axis[mid];
        var diff = q[ax] - p[ax];
        if (diff <= 0)
        {
            KnnRec(lo, mid, q, k, r);
            if (r.Count < k || diff * diff <= r[Worst(r)].D)
            {
                KnnRec(mid + 1, hi, q, k, r);
            }
        }
        else
        {
            KnnRec(mid + 1, hi, q, k, r);
            if (r.Count < k || diff * diff <= r[Worst(r)].D)
            {
                KnnRec(lo, mid, q, k, r);
            }
        }
    }

    /// <summary>All points with distance² ≤ radius², sorted by (distance², index).</summary>
    public List<(float Dist2, int Index)> Radius(ReadOnlySpan<float> query, float radius)
    {
        var result = new List<(float, int)>();
        Radius(query, radius, result);
        return result;
    }

    public void Radius(ReadOnlySpan<float> query, float radius, List<(float Dist2, int Index)> result)
    {
        result.Clear();
        RadiusRec(0, _idx.Length, query, radius * radius, result);
        result.Sort(static (a, b) => a.Dist2 != b.Dist2 ? a.Dist2.CompareTo(b.Dist2) : a.Index.CompareTo(b.Index));
    }

    private void RadiusRec(int lo, int hi, ReadOnlySpan<float> q, float r2, List<(float, int)> r)
    {
        if (hi <= lo)
        {
            return;
        }

        var mid = (lo + hi) / 2;
        var i = _idx[mid];
        var p = P(i);
        var d = Dist2(p, q);
        if (d <= r2)
        {
            r.Add((d, i));
        }

        if (hi == lo + 1)
        {
            return;
        }

        var ax = _axis[mid];
        var diff = q[ax] - p[ax];
        var (a0, a1, b0, b1) = diff <= 0 ? (lo, mid, mid + 1, hi) : (mid + 1, hi, lo, mid);
        RadiusRec(a0, a1, q, r2, r);
        if (diff * diff <= r2)
        {
            RadiusRec(b0, b1, q, r2, r);
        }
    }
}
