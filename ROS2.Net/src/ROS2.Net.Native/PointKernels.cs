using ROS2.Net.Native.Managed;

namespace ROS2.Net.Native;

/// <summary>
/// Point cloud, laser and grid kernels. Each call goes to the Rust library when
/// <see cref="NativeLibraryInfo.UseNative"/> is true and to <see cref="ManagedKernels"/> otherwise;
/// both produce the same results. Point buffers are interleaved xyz floats (length 3·N).
/// </summary>
public static unsafe class PointKernels
{
    private static int Count(ReadOnlySpan<float> xyz) => xyz.Length / 3;

    /// <summary>Voxel-grid downsampling: one centroid per occupied voxel, ordered by voxel index (x, y, z).</summary>
    public static float[] VoxelDownsample(ReadOnlySpan<float> xyz, float leafSize)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(leafSize);
        if (!NativeLibraryInfo.UseNative)
        {
            return ManagedKernels.VoxelDownsample(xyz, leafSize);
        }

        var n = Count(xyz);
        var output = new float[xyz.Length];
        fixed (float* p = xyz)
        fixed (float* o = output)
        {
            NativeMethods.Check(NativeMethods.VoxelDownsample(p, (nuint)n, leafSize, o, (nuint)n, out var count), "voxel_downsample");
            return output.AsSpan(0, (int)count * 3).ToArray();
        }
    }

    /// <summary>Statistical outlier removal mask (1 = keep).</summary>
    public static void StatisticalOutlierMask(ReadOnlySpan<float> xyz, int k, double stdMultiplier, Span<byte> keep)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(k);
        CheckLength(keep.Length, Count(xyz));
        if (!NativeLibraryInfo.UseNative)
        {
            ManagedKernels.StatisticalOutlier(xyz, k, stdMultiplier, keep);
            return;
        }

        fixed (float* p = xyz)
        fixed (byte* m = keep)
        {
            NativeMethods.Check(NativeMethods.StatisticalOutlier(p, (nuint)Count(xyz), (nuint)k, stdMultiplier, m), "statistical_outlier");
        }
    }

    /// <summary>Radius outlier removal mask (1 = at least <paramref name="minNeighbors"/> others within <paramref name="radius"/>).</summary>
    public static void RadiusOutlierMask(ReadOnlySpan<float> xyz, float radius, int minNeighbors, Span<byte> keep)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(radius);
        CheckLength(keep.Length, Count(xyz));
        if (!NativeLibraryInfo.UseNative)
        {
            ManagedKernels.RadiusOutlier(xyz, radius, minNeighbors, keep);
            return;
        }

        fixed (float* p = xyz)
        fixed (byte* m = keep)
        {
            NativeMethods.Check(NativeMethods.RadiusOutlier(p, (nuint)Count(xyz), radius, (nuint)Math.Max(0, minNeighbors), m), "radius_outlier");
        }
    }

    /// <summary>Euclidean clustering. Writes labels (-1 = rejected) and returns the number of clusters.</summary>
    public static int EuclideanCluster(ReadOnlySpan<float> xyz, float tolerance, int minSize, int maxSize, Span<int> labels)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(tolerance);
        CheckLength(labels.Length, Count(xyz));
        if (!NativeLibraryInfo.UseNative)
        {
            return ManagedKernels.EuclideanCluster(xyz, tolerance, minSize, maxSize, labels);
        }

        fixed (float* p = xyz)
        fixed (int* l = labels)
        {
            NativeMethods.Check(NativeMethods.EuclideanCluster(p, (nuint)Count(xyz), tolerance, (nuint)Math.Max(0, minSize), (nuint)Math.Max(0, maxSize), l, out var clusters), "euclidean_cluster");
            return (int)clusters;
        }
    }

    /// <summary>RANSAC plane fit with a reproducible seed; fills the inlier mask.</summary>
    public static PlaneFit RansacPlane(ReadOnlySpan<float> xyz, double distanceThreshold, int iterations, ulong seed, Span<byte> inliers)
    {
        CheckLength(inliers.Length, Count(xyz));
        if (!NativeLibraryInfo.UseNative)
        {
            var (count, a, b, c, d) = ManagedKernels.RansacPlane(xyz, distanceThreshold, iterations, seed, inliers);
            return new PlaneFit(a, b, c, d, count);
        }

        var coeffs = stackalloc double[4];
        fixed (float* p = xyz)
        fixed (byte* m = inliers)
        {
            NativeMethods.Check(NativeMethods.RansacPlane(p, (nuint)Count(xyz), distanceThreshold, (nuint)Math.Max(0, iterations), seed, coeffs, m, out var count), "ransac_plane");
            return new PlaneFit(coeffs[0], coeffs[1], coeffs[2], coeffs[3], (int)count);
        }
    }

    /// <summary>Point-to-point ICP aligning <paramref name="source"/> onto <paramref name="target"/>.</summary>
    public static IcpResult Icp(ReadOnlySpan<float> source, ReadOnlySpan<float> target, int maxIterations = 50, float maxCorrespondenceDistance = 1f, double tolerance = 1e-6)
    {
        if (target.Length < 3)
        {
            throw new ArgumentException("Target cloud is empty.", nameof(target));
        }

        if (!NativeLibraryInfo.UseNative)
        {
            return ManagedKernels.Icp(source, target, maxIterations, maxCorrespondenceDistance, tolerance);
        }

        var transform = new double[16];
        fixed (float* s = source)
        fixed (float* t = target)
        fixed (double* m = transform)
        {
            NativeMethods.Check(NativeMethods.Icp(s, (nuint)Count(source), t, (nuint)Count(target), (uint)Math.Max(0, maxIterations), maxCorrespondenceDistance, tolerance, m, out var rmse, out var iterations, out var converged), "icp");
            return new IcpResult(transform, rmse, (int)iterations, converged != 0, -1);
        }
    }

    /// <summary>Projects scan ranges into xyz (z = 0). <paramref name="output"/> needs 3·ranges.Length floats. Returns the point count.</summary>
    public static int ProjectScan(ReadOnlySpan<float> ranges, float angleMin, float angleIncrement, float rangeMin, float rangeMax, Span<float> output)
    {
        CheckLength(output.Length, ranges.Length * 3);
        if (!NativeLibraryInfo.UseNative)
        {
            return ManagedKernels.ProjectScan(ranges, angleMin, angleIncrement, rangeMin, rangeMax, output);
        }

        fixed (float* r = ranges)
        fixed (float* o = output)
        {
            NativeMethods.Check(NativeMethods.ScanProject(r, (nuint)ranges.Length, angleMin, angleIncrement, rangeMin, rangeMax, o, out var n), "scan_project");
            return (int)n;
        }
    }

    /// <summary>Log-odds occupancy update: ray-traces from the sensor to each xy endpoint.</summary>
    public static void UpdateOccupancy(Span<float> logOddsGrid, OccupancyGridSpec spec, double sensorX, double sensorY, ReadOnlySpan<float> pointsXy, LogOddsModel model)
    {
        CheckLength(logOddsGrid.Length, spec.Width * spec.Height);
        if (!NativeLibraryInfo.UseNative)
        {
            ManagedKernels.OccupancyUpdate(logOddsGrid, spec, sensorX, sensorY, pointsXy, model);
            return;
        }

        fixed (float* g = logOddsGrid)
        fixed (float* p = pointsXy)
        {
            NativeMethods.Check(NativeMethods.OccupancyUpdate(g, spec.Width, spec.Height, spec.Resolution, spec.OriginX, spec.OriginY, sensorX, sensorY, p, (nuint)(pointsXy.Length / 2), model.Free, model.Occupied, model.Min, model.Max), "occupancy_update");
        }
    }

    private static void CheckLength(int actual, int required)
    {
        if (actual < required)
        {
            throw new ArgumentException($"Output buffer holds {actual} elements but {required} are required.");
        }
    }
}

/// <summary>A k-d tree over xyz points, backed by native code when available.</summary>
public sealed unsafe class KdTree : IDisposable
{
    private readonly ManagedKdTree? _managed;
    private nint _native;

    public KdTree(ReadOnlySpan<float> xyz)
    {
        Count = xyz.Length / 3;
        if (NativeLibraryInfo.UseNative)
        {
            fixed (float* p = xyz)
            {
                _native = NativeMethods.KdTreeBuild(p, (nuint)Count);
            }

            if (_native == 0)
            {
                throw new NativeKernelException("kdtree_build", NativeMethods.StatusNull);
            }
        }
        else
        {
            _managed = new ManagedKdTree(xyz);
        }
    }

    public int Count { get; }

    public bool IsNative => _native != 0;

    /// <summary>The k nearest neighbours of a point, sorted by (squared distance, index).</summary>
    public (int Index, float Distance2)[] Nearest(float x, float y, float z, int k)
    {
        if (_managed is not null)
        {
            ReadOnlySpan<float> q = [x, y, z];
            return _managed.Knn(q, k).Select(r => (r.Index, r.Dist2)).ToArray();
        }

        ObjectDisposedException.ThrowIf(_native == 0, this);
        var idx = new uint[k];
        var d2 = new float[k];
        var q2 = stackalloc float[3] { x, y, z };
        fixed (uint* pi = idx)
        fixed (float* pd = d2)
        {
            NativeMethods.Check(NativeMethods.KdTreeKnn(_native, q2, (nuint)k, pi, pd, out var found), "kdtree_knn");
            var result = new (int, float)[(int)found];
            for (var i = 0; i < result.Length; i++)
            {
                result[i] = ((int)idx[i], d2[i]);
            }

            return result;
        }
    }

    /// <summary>All points within <paramref name="radius"/>, sorted by (squared distance, index).</summary>
    public (int Index, float Distance2)[] WithinRadius(float x, float y, float z, float radius)
    {
        if (_managed is not null)
        {
            ReadOnlySpan<float> q = [x, y, z];
            return _managed.Radius(q, radius).Select(r => (r.Index, r.Dist2)).ToArray();
        }

        ObjectDisposedException.ThrowIf(_native == 0, this);
        var q2 = stackalloc float[3] { x, y, z };
        var cap = 64;
        while (true)
        {
            var idx = new uint[cap];
            var d2 = new float[cap];
            fixed (uint* pi = idx)
            fixed (float* pd = d2)
            {
                var status = NativeMethods.KdTreeRadius(_native, q2, radius, pi, pd, (nuint)cap, out var n);
                if (status == NativeMethods.StatusBufferTooSmall)
                {
                    cap = (int)n;
                    continue;
                }

                NativeMethods.Check(status, "kdtree_radius");
                var result = new (int, float)[(int)n];
                for (var i = 0; i < result.Length; i++)
                {
                    result[i] = ((int)idx[i], d2[i]);
                }

                return result;
            }
        }
    }

    public void Dispose()
    {
        var h = Interlocked.Exchange(ref _native, 0);
        if (h != 0)
        {
            NativeMethods.KdTreeFree(h);
        }

        GC.SuppressFinalize(this);
    }

    ~KdTree()
    {
        var h = Interlocked.Exchange(ref _native, 0);
        if (h != 0)
        {
            NativeMethods.KdTreeFree(h);
        }
    }
}
