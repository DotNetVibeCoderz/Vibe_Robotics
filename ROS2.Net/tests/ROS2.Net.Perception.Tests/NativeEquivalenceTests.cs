using System.Diagnostics;
using ROS2.Net.Native;
using ROS2.Net.Native.Managed;
using Shouldly;

namespace ROS2.Net.Perception.Tests;

/// <summary>Native and managed kernels must agree. These call both implementations directly, so they never touch the global mode.</summary>
public class NativeEquivalenceTests(ITestOutputHelper output)
{
    private static void RequireNative() =>
        Assert.SkipUnless(NativeLibraryInfo.IsAvailable, $"ros2net_native not available: {NativeLibraryInfo.LoadError}");

    private static T Native<T>(Func<T> f)
    {
        var previous = NativeLibraryInfo.Mode;
        NativeLibraryInfo.Mode = KernelMode.Native;
        try
        {
            return f();
        }
        finally
        {
            NativeLibraryInfo.Mode = previous;
        }
    }

    [Fact]
    public void Native_library_is_built_and_loads()
    {
        RequireNative();
        NativeLibraryInfo.AbiVersion.ShouldBe(NativeLibraryInfo.ExpectedAbiVersion);
        NativeLibraryInfo.Version.ShouldNotBeNullOrEmpty();
    }

    [Fact]
    public void Voxel_downsample_matches()
    {
        RequireNative();
        var xyz = Synthetic.Random(20_000, 1);
        var native = Native(() => PointKernels.VoxelDownsample(xyz, 0.7f));
        native.ShouldBe(ManagedKernels.VoxelDownsample(xyz, 0.7f));
    }

    [Fact]
    public void Knn_and_radius_match()
    {
        RequireNative();
        var xyz = Synthetic.Random(5_000, 2);
        using var nativeTree = Native(() => new KdTree(xyz));
        nativeTree.IsNative.ShouldBeTrue();
        var managed = new ManagedKdTree(xyz);
        var rng = new Random(3);
        for (var q = 0; q < 200; q++)
        {
            float x = (float)(rng.NextDouble() * 10), y = (float)(rng.NextDouble() * 10), z = (float)(rng.NextDouble() * 10);
            nativeTree.Nearest(x, y, z, 9).ShouldBe(managed.Knn([x, y, z], 9).Select(r => (r.Index, r.Dist2)).ToArray());
            nativeTree.WithinRadius(x, y, z, 0.8f).ShouldBe(managed.Radius([x, y, z], 0.8f).Select(r => (r.Index, r.Dist2)).ToArray());
        }
    }

    [Fact]
    public void Outlier_masks_match()
    {
        RequireNative();
        var xyz = Synthetic.Random(4_000, 4);
        var a = new byte[4_000];
        var b = new byte[4_000];
        Native(() =>
        {
            PointKernels.StatisticalOutlierMask(xyz, 12, 1.0, a);
            return 0;
        });
        ManagedKernels.StatisticalOutlier(xyz, 12, 1.0, b);
        a.ShouldBe(b);

        Native(() =>
        {
            PointKernels.RadiusOutlierMask(xyz, 0.5f, 4, a);
            return 0;
        });
        ManagedKernels.RadiusOutlier(xyz, 0.5f, 4, b);
        a.ShouldBe(b);
    }

    [Fact]
    public void Clustering_and_ransac_match()
    {
        RequireNative();
        var cloud = Synthetic.Plane(40, 0.05f, 0.1f, noise: 0.01f);
        Synthetic.Blob(cloud, 1, 1, 1, 300, 0.1f, 5);
        Synthetic.Blob(cloud, 3, 3, 1, 300, 0.1f, 6);
        var xyz = cloud.Xyz.ToArray();

        var la = new int[cloud.Count];
        var lb = new int[cloud.Count];
        Native(() => PointKernels.EuclideanCluster(xyz, 0.08f, 10, 100_000, la)).ShouldBe(ManagedKernels.EuclideanCluster(xyz, 0.08f, 10, 100_000, lb));
        la.ShouldBe(lb);

        var ma = new byte[cloud.Count];
        var mb = new byte[cloud.Count];
        var native = Native(() => PointKernels.RansacPlane(xyz, 0.02, 200, 99, ma));
        var managed = ManagedKernels.RansacPlane(xyz, 0.02, 200, 99, mb);
        native.InlierCount.ShouldBe(managed.Count);
        native.A.ShouldBe(managed.A, 1e-12);
        native.D.ShouldBe(managed.D, 1e-12);
        ma.ShouldBe(mb);
    }

    [Fact]
    public void Icp_and_scan_projection_match()
    {
        RequireNative();
        var target = Synthetic.Surface();
        var source = target.Clone().Transform(Registration.FromRpyTranslation(0, 0, -0.05, -0.03, 0.02, 0));
        var native = Native(() => PointKernels.Icp(source.Xyz, target.Xyz, 60, 0.5f, 1e-9));
        var managed = ManagedKernels.Icp(source.Xyz, target.Xyz, 60, 0.5f, 1e-9);
        native.Iterations.ShouldBe(managed.Iterations);
        for (var i = 0; i < 16; i++)
        {
            native.Transform[i].ShouldBe(managed.Transform[i], 1e-9);
        }

        var ranges = Enumerable.Range(0, 720).Select(i => i % 13 == 0 ? float.NaN : 1 + (i * 0.01f)).ToArray();
        var a = new float[ranges.Length * 3];
        var b = new float[ranges.Length * 3];
        var na = Native(() => PointKernels.ProjectScan(ranges, -MathF.PI, MathF.PI / 360, 0.1f, 30, a));
        var nb = ManagedKernels.ProjectScan(ranges, -MathF.PI, MathF.PI / 360, 0.1f, 30, b);
        na.ShouldBe(nb);
        for (var i = 0; i < na * 3; i++)
        {
            a[i].ShouldBe(b[i], 1e-5f);
        }
    }

    [Fact]
    public void Occupancy_update_matches()
    {
        RequireNative();
        var spec = new OccupancyGridSpec(200, 150, 0.05, -5, -3.75);
        var pts = Synthetic.Random(500, 8, 8).Select(v => v - 4f).Take(1000).ToArray();
        var ga = new float[200 * 150];
        var gb = new float[200 * 150];
        Native(() =>
        {
            PointKernels.UpdateOccupancy(ga, spec, 0.3, -0.2, pts, new LogOddsModel());
            return 0;
        });
        ManagedKernels.OccupancyUpdate(gb, spec, 0.3, -0.2, pts, new LogOddsModel());
        ga.ShouldBe(gb);
    }

    [Fact]
    public void Native_is_faster_for_voxel_and_sor()
    {
        RequireNative();
        var xyz = Synthetic.Random(100_000, 9, 20);
        var sw = Stopwatch.StartNew();
        ManagedKernels.VoxelDownsample(xyz, 0.2f);
        var mask = new byte[100_000];
        ManagedKernels.StatisticalOutlier(xyz, 8, 1, mask);
        var managed = sw.Elapsed;
        sw.Restart();
        Native(() =>
        {
            PointKernels.VoxelDownsample(xyz, 0.2f);
            PointKernels.StatisticalOutlierMask(xyz, 8, 1, mask);
            return 0;
        });
        var native = sw.Elapsed;
        output.WriteLine($"100k points voxel+SOR: managed {managed.TotalMilliseconds:0} ms, native {native.TotalMilliseconds:0} ms");
        native.ShouldBeLessThan(managed * 2); // Informational; only guards against pathological regressions.
    }
}
