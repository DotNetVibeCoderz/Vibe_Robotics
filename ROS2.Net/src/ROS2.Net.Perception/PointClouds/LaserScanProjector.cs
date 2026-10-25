using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Native;

namespace ROS2.Net.Perception;

/// <summary>
/// Projects <c>sensor_msgs/LaserScan</c> into Cartesian points in the scan frame, like
/// <c>laser_geometry::LaserProjection::projectLaser</c> (no per-beam time correction).
/// </summary>
public static class LaserScanProjector
{
    /// <summary>Projects valid ranges (finite and within [range_min, range_max]); intensities are kept when present.</summary>
    public static PointCloud Project(LaserScan scan, bool includeIntensity = true)
    {
        var ranges = scan.Ranges;
        var xyz = new float[ranges.Length * 3];
        var n = PointKernels.ProjectScan(ranges, scan.AngleMin, scan.AngleIncrement, scan.RangeMin, scan.RangeMax, xyz);
        var cloud = PointCloud.FromXyz(xyz, n);
        cloud.FrameId = scan.Header.FrameId;
        cloud.Stamp = RosTime.FromMsg(scan.Header.Stamp);
        if (includeIntensity && scan.Intensities.Length == ranges.Length && ranges.Length > 0)
        {
            cloud.WithIntensity();
            var dst = cloud.Intensity;
            var k = 0;
            for (var i = 0; i < ranges.Length; i++)
            {
                var r = ranges[i];
                if (float.IsFinite(r) && r >= scan.RangeMin && r <= scan.RangeMax)
                {
                    dst[k++] = scan.Intensities[i];
                }
            }
        }

        return cloud;
    }

    /// <summary>Projects into a PointCloud2 (x, y, z [, intensity]).</summary>
    public static PointCloud2 ToPointCloud2(LaserScan scan, bool includeIntensity = true) => Project(scan, includeIntensity).ToMessage();

    /// <summary>Projects into interleaved xy pairs, the input of occupancy updates.</summary>
    public static float[] ProjectXy(LaserScan scan)
    {
        var cloud = Project(scan, includeIntensity: false);
        var xy = new float[cloud.Count * 2];
        var xyz = cloud.Xyz;
        for (var i = 0; i < cloud.Count; i++)
        {
            xy[2 * i] = xyz[3 * i];
            xy[(2 * i) + 1] = xyz[(3 * i) + 1];
        }

        return xy;
    }
}
