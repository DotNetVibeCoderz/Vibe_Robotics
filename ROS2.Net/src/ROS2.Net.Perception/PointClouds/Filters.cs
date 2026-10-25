using System.Numerics;
using ROS2.Net.Native;

namespace ROS2.Net.Perception;

/// <summary>Coordinate axis selector.</summary>
public enum Axis
{
    X = 0,
    Y = 1,
    Z = 2,
}

/// <summary>Point cloud filters (PCL equivalents). Heavy kernels run natively when available.</summary>
public static class Filters
{
    /// <summary>Keeps points whose <paramref name="axis"/> coordinate is in [min, max] (or outside, with <paramref name="negative"/>).</summary>
    public static PointCloud PassThrough(PointCloud cloud, Axis axis, float min, float max, bool negative = false)
    {
        var xyz = cloud.Xyz;
        var mask = new byte[cloud.Count];
        var a = (int)axis;
        for (var i = 0; i < cloud.Count; i++)
        {
            var v = xyz[(3 * i) + a];
            var inside = v >= min && v <= max;
            mask[i] = (byte)(inside != negative && float.IsFinite(v) ? 1 : 0);
        }

        return cloud.Select(mask);
    }

    /// <summary>Keeps points inside the axis-aligned box (or outside, with <paramref name="negative"/>).</summary>
    public static PointCloud CropBox(PointCloud cloud, Vector3 min, Vector3 max, bool negative = false)
    {
        var xyz = cloud.Xyz;
        var mask = new byte[cloud.Count];
        for (var i = 0; i < cloud.Count; i++)
        {
            float x = xyz[3 * i], y = xyz[(3 * i) + 1], z = xyz[(3 * i) + 2];
            var inside = x >= min.X && x <= max.X && y >= min.Y && y <= max.Y && z >= min.Z && z <= max.Z;
            mask[i] = (byte)(inside != negative ? 1 : 0);
        }

        return cloud.Select(mask);
    }

    /// <summary>Drops points with NaN/Inf coordinates.</summary>
    public static PointCloud RemoveNonFinite(PointCloud cloud)
    {
        var xyz = cloud.Xyz;
        var mask = new byte[cloud.Count];
        for (var i = 0; i < cloud.Count; i++)
        {
            mask[i] = (byte)(float.IsFinite(xyz[3 * i]) && float.IsFinite(xyz[(3 * i) + 1]) && float.IsFinite(xyz[(3 * i) + 2]) ? 1 : 0);
        }

        return cloud.Select(mask);
    }

    /// <summary>
    /// Voxel-grid downsampling: one centroid per voxel. Positions-only clouds use the native kernel;
    /// clouds with intensity/RGB average those channels too (managed).
    /// </summary>
    public static PointCloud VoxelGrid(PointCloud cloud, float leafSize)
    {
        if (!cloud.HasIntensity && !cloud.HasRgb)
        {
            var result = PointCloud.FromXyz(PointKernels.VoxelDownsample(cloud.Xyz, leafSize));
            result.FrameId = cloud.FrameId;
            result.Stamp = cloud.Stamp;
            return result;
        }

        var inv = 1.0 / leafSize;
        var xyz = cloud.Xyz;
        var intensity = cloud.Intensity;
        var rgb = cloud.Rgb;
        var acc = new SortedDictionary<(long, long, long), double[]>();
        for (var i = 0; i < cloud.Count; i++)
        {
            float x = xyz[3 * i], y = xyz[(3 * i) + 1], z = xyz[(3 * i) + 2];
            if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z))
            {
                continue;
            }

            var key = ((long)Math.Floor(x * inv), (long)Math.Floor(y * inv), (long)Math.Floor(z * inv));
            if (!acc.TryGetValue(key, out var s))
            {
                acc[key] = s = new double[8];
            }

            s[0] += x;
            s[1] += y;
            s[2] += z;
            s[3] += intensity.IsEmpty ? 0 : intensity[i];
            if (!rgb.IsEmpty)
            {
                s[4] += (rgb[i] >> 16) & 0xFF;
                s[5] += (rgb[i] >> 8) & 0xFF;
                s[6] += rgb[i] & 0xFF;
            }

            s[7] += 1;
        }

        var outCloud = new PointCloud(acc.Count) { FrameId = cloud.FrameId, Stamp = cloud.Stamp };
        if (cloud.HasIntensity)
        {
            outCloud.WithIntensity();
        }

        if (cloud.HasRgb)
        {
            outCloud.WithRgb();
        }

        foreach (var s in acc.Values)
        {
            var c = s[7];
            var color = ((uint)Math.Round(s[4] / c) << 16) | ((uint)Math.Round(s[5] / c) << 8) | (uint)Math.Round(s[6] / c);
            outCloud.Add((float)(s[0] / c), (float)(s[1] / c), (float)(s[2] / c), (float)(s[3] / c), color);
        }

        return outCloud;
    }

    /// <summary>Statistical outlier removal (PCL <c>StatisticalOutlierRemoval</c>).</summary>
    public static PointCloud StatisticalOutlierRemoval(PointCloud cloud, int meanK = 16, double stdDevMultiplier = 1.0)
    {
        var mask = new byte[cloud.Count];
        PointKernels.StatisticalOutlierMask(cloud.Xyz, meanK, stdDevMultiplier, mask);
        return cloud.Select(mask);
    }

    /// <summary>Radius outlier removal (PCL <c>RadiusOutlierRemoval</c>).</summary>
    public static PointCloud RadiusOutlierRemoval(PointCloud cloud, float radius, int minNeighbors)
    {
        var mask = new byte[cloud.Count];
        PointKernels.RadiusOutlierMask(cloud.Xyz, radius, minNeighbors, mask);
        return cloud.Select(mask);
    }
}
