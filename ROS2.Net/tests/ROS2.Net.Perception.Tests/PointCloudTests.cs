using System.Buffers.Binary;
using System.Numerics;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Native;
using Shouldly;

namespace ROS2.Net.Perception.Tests;

public class PointCloudTests
{
    [Fact]
    public void Round_trips_xyz_intensity_rgb()
    {
        var cloud = new PointCloud().WithIntensity().WithRgb();
        cloud.Add(1, 2, 3, 0.5f, 0x00FF8000);
        cloud.Add(-1, float.NaN, 0, 1f, 0x000000FF);
        var msg = cloud.ToMessage("lidar", new RosTime(5_000_000_000));

        msg.IsDense.ShouldBeFalse();
        msg.PointStep.ShouldBe(20u);
        msg.Header.FrameId.ShouldBe("lidar");

        var back = PointCloud.FromMessage(msg);
        back.Count.ShouldBe(2);
        back[0].ShouldBe(new Vector3(1, 2, 3));
        back.Intensity[0].ShouldBe(0.5f);
        back.Rgb[0].ShouldBe(0x00FF8000u);
        float.IsNaN(back[1].Y).ShouldBeTrue();
        PointCloud.FromMessage(msg, skipNonFinite: true).Count.ShouldBe(1);
    }

    [Fact]
    public void Reads_pcl_padded_layout()
    {
        // PCL PointXYZI: x y z _ (16 bytes), intensity at 16, point_step 32.
        var w = new PointCloud2Writer().AddField("x", PointField.FLOAT32).AddField("y", PointField.FLOAT32).AddField("z", PointField.FLOAT32).Pad(4).AddField("intensity", PointField.FLOAT32).Pad(12);
        w.PointStep.ShouldBe(32);
        var msg = w.Create(3);
        var fx = w.Field("x");
        var fi = w.Field("intensity");
        fi.Offset.ShouldBe(16);
        for (var i = 0; i < 3; i++)
        {
            var p = msg.Data.AsSpan(i * 32, 32);
            fx.Write(p, i * 1.5);
            fi.Write(p, 100 + i);
        }

        var cloud = PointCloud.FromMessage(msg);
        cloud[2].X.ShouldBe(3f);
        cloud.Intensity[1].ShouldBe(101f);
    }

    [Fact]
    public void Reads_big_endian_doubles_and_organized_rows_with_padding()
    {
        var w = new PointCloud2Writer { BigEndian = true }.AddField("x", PointField.FLOAT64).AddField("y", PointField.FLOAT64).AddField("z", PointField.FLOAT64);
        var msg = w.Create(2, 2);
        // Add 8 bytes of padding per row.
        var rowStep = (int)msg.RowStep + 8;
        var data = new byte[rowStep * 2];
        for (var r = 0; r < 2; r++)
        {
            for (var c = 0; c < 2; c++)
            {
                var p = data.AsSpan((r * rowStep) + (c * 24), 24);
                BinaryPrimitives.WriteDoubleBigEndian(p, r * 10 + c);
                BinaryPrimitives.WriteDoubleBigEndian(p[8..], -1);
                BinaryPrimitives.WriteDoubleBigEndian(p[16..], 7);
            }
        }

        msg.Data = data;
        msg.RowStep = (uint)rowStep;
        var reader = new PointCloud2Reader(msg);
        reader.Count.ShouldBe(4);
        var xyz = new float[12];
        reader.ReadXyz(xyz).ShouldBe(4);
        xyz[9].ShouldBe(11f);
        xyz[10].ShouldBe(-1f);
        xyz[11].ShouldBe(7f);
    }

    [Fact]
    public void Integer_fields_are_supported()
    {
        var w = new PointCloud2Writer().AddField("x", PointField.INT16).AddField("y", PointField.UINT8).AddField("z", PointField.INT32);
        var msg = w.Create(1);
        w.Field("x").Write(msg.Data, -5);
        w.Field("y").Write(msg.Data, 200);
        w.Field("z").Write(msg.Data, 70000);
        var p = PointCloud.FromMessage(msg)[0];
        p.ShouldBe(new Vector3(-5, 200, 70000));
    }

    [Fact]
    public void Inconsistent_message_is_rejected()
    {
        var msg = new PointCloud2Writer().AddField("x", PointField.FLOAT32).AddField("y", PointField.FLOAT32).AddField("z", PointField.FLOAT32).Create(10);
        msg.Data = new byte[5];
        Should.Throw<ArgumentException>(() => new PointCloud2Reader(msg));
    }

    [Fact]
    public void Filters_pass_through_crop_and_voxel()
    {
        var cloud = Synthetic.Plane(10, 0.1f, 0);
        cloud.Add(0, 0, 5);
        Filters.PassThrough(cloud, Axis.Z, -1, 1).Count.ShouldBe(100);
        Filters.PassThrough(cloud, Axis.Z, -1, 1, negative: true).Count.ShouldBe(1);
        Filters.CropBox(cloud, new Vector3(-0.05f), new Vector3(0.45f, 0.45f, 0.5f)).Count.ShouldBe(25);

        var voxel = Filters.VoxelGrid(cloud, 0.5f);
        voxel.Count.ShouldBe(5); // 2x2 voxels for the plane + the elevated point
        Filters.VoxelGrid(cloud.Clone().WithIntensity(), 0.5f).Count.ShouldBe(5);
    }

    [Fact]
    public void Outlier_removal_drops_isolated_points()
    {
        var cloud = Synthetic.Plane(20, 0.05f, 0);
        cloud.Add(30, 30, 30);
        cloud.Add(-30, 5, 1);
        var sor = Filters.StatisticalOutlierRemoval(cloud, 8, 1.0);
        sor.Count.ShouldBeLessThan(cloud.Count);
        Enumerable.Range(0, sor.Count).Select(i => sor[i]).ShouldNotContain(p => p.X > 10 || p.X < -10);

        var ror = Filters.RadiusOutlierRemoval(cloud, 0.12f, 3);
        ror.Count.ShouldBe(400);
    }

    [Fact]
    public void Ransac_finds_the_dominant_plane()
    {
        var cloud = Synthetic.Plane(30, 0.1f, 0.25f, noise: 0.004f);
        Synthetic.Blob(cloud, 1.5f, 1.5f, 1.5f, 200, 0.2f, 3);
        var seg = Segmentation.SegmentPlane(cloud, 0.01, 300, 7);
        seg.Inliers.Length.ShouldBeGreaterThanOrEqualTo(880);
        Math.Abs(seg.Plane.C).ShouldBeGreaterThan(0.99);
        (Math.Abs(seg.Plane.D / seg.Plane.C) - 0.25).ShouldBeLessThan(0.01);
        (seg.Inliers.Length + seg.Outliers.Length).ShouldBe(cloud.Count);
    }

    [Fact]
    public void Euclidean_clusters_separate_objects()
    {
        var cloud = new PointCloud();
        Synthetic.Blob(cloud, 0, 0, 0, 150, 0.05f, 1);
        Synthetic.Blob(cloud, 3, 0, 0, 120, 0.05f, 2);
        Synthetic.Blob(cloud, 0, 4, 0, 5, 0.01f, 3);
        var clusters = Segmentation.EuclideanClusters(cloud, 0.2f, minSize: 20);
        clusters.Count.ShouldBe(2);
        clusters[0].Length.ShouldBe(150);
        clusters[1].Length.ShouldBe(120);

        var box = BoundingBoxes.ComputeAabb(cloud, clusters[1]);
        box.Center.X.ShouldBe(3f, 0.1f);
    }

    [Fact]
    public void Normals_of_a_plane_point_up_towards_the_viewpoint()
    {
        var cloud = Synthetic.Plane(10, 0.1f, 0);
        var curvature = new float[cloud.Count];
        var normals = Normals.Estimate(cloud, 8, new Vector3(0.5f, 0.5f, 3), curvature);
        for (var i = 0; i < cloud.Count; i++)
        {
            normals[(3 * i) + 2].ShouldBe(1f, 1e-4f);
            curvature[i].ShouldBe(0f, 1e-5f);
        }

        cloud.HasNormals.ShouldBeTrue();
    }

    [Fact]
    public void Oriented_box_follows_the_principal_axes()
    {
        var cloud = new PointCloud();
        for (var i = 0; i <= 40; i++)
        {
            for (var j = 0; j <= 4; j++)
            {
                // A 4 x 0.4 rectangle rotated by 45°.
                float u = (i * 0.1f) - 2, v = (j * 0.1f) - 0.2f;
                cloud.Add((u - v) * MathF.Sqrt(0.5f), (u + v) * MathF.Sqrt(0.5f), 0);
            }
        }

        var obb = BoundingBoxes.ComputeObb(cloud);
        obb.HalfExtents.X.ShouldBe(2f, 1e-3f);
        obb.HalfExtents.Y.ShouldBe(0.2f, 1e-3f);
        Math.Abs(obb.Axes[0].X).ShouldBe(MathF.Sqrt(0.5f), 1e-3f);
    }

    [Theory]
    [InlineData(IcpMethod.PointToPoint)]
    [InlineData(IcpMethod.PointToPlane)]
    public void Icp_recovers_a_rigid_motion(IcpMethod method)
    {
        var target = Synthetic.Surface();
        var motion = Registration.FromRpyTranslation(0.02, -0.01, 0.08, 0.05, -0.04, 0.02);
        var inverse = Invert(motion);
        var source = target.Clone().Transform(inverse);

        var result = Registration.Icp(source, target, new IcpOptions { Method = method, MaxIterations = 100, MaxCorrespondenceDistance = 0.5f, Tolerance = 1e-9 });

        result.Rmse.ShouldBeLessThan(2e-3);
        for (var i = 0; i < 12; i++)
        {
            result.Transform[i].ShouldBe(motion[i], 5e-3);
        }
    }

    private static double[] Invert(double[] m)
    {
        // Rigid inverse: [Rᵀ | -Rᵀt].
        var r = new double[16];
        for (var i = 0; i < 3; i++)
        {
            for (var j = 0; j < 3; j++)
            {
                r[(i * 4) + j] = m[(j * 4) + i];
            }
        }

        for (var i = 0; i < 3; i++)
        {
            r[(i * 4) + 3] = -((r[i * 4] * m[3]) + (r[(i * 4) + 1] * m[7]) + (r[(i * 4) + 2] * m[11]));
        }

        r[15] = 1;
        return r;
    }

    [Fact]
    public void Laser_scan_projection_keeps_valid_beams_and_intensities()
    {
        var scan = new LaserScan
        {
            AngleMin = 0,
            AngleIncrement = MathF.PI / 2,
            RangeMin = 0.1f,
            RangeMax = 5,
            Ranges = [1, float.PositiveInfinity, 2, 0.05f],
            Intensities = [10, 20, 30, 40],
        };
        scan.Header.FrameId = "laser";
        var cloud = LaserScanProjector.Project(scan);
        cloud.Count.ShouldBe(2);
        cloud[0].X.ShouldBe(1f, 1e-6f);
        cloud[1].X.ShouldBe(-2f, 1e-5f);
        cloud.Intensity.ToArray().ShouldBe(new[] { 10f, 30f });
        cloud.FrameId.ShouldBe("laser");
        LaserScanProjector.ToPointCloud2(scan).Width.ShouldBe(2u);
    }

    [Fact]
    public void Occupancy_update_marks_free_and_occupied_cells()
    {
        var spec = new OccupancyGridSpec(20, 20, 0.1, -1, -1);
        var grid = new float[400];
        PointKernels.UpdateOccupancy(grid, spec, 0, 0, [0.55f, 0.05f], new LogOddsModel());
        grid[(10 * 20) + 10].ShouldBe(-0.4f); // sensor cell
        grid[(10 * 20) + 15].ShouldBe(0.85f); // endpoint
        grid[(10 * 20) + 16].ShouldBe(0f);
        LogOddsModel.ToProbability(0).ShouldBe(0.5f);
    }
}
