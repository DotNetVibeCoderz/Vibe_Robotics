using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Navigation.Maps;

namespace ROS2.Net.Navigation.Tests;

/// <summary>Synthetic maps and a ray-cast lidar for tests.</summary>
internal static class TestWorlds
{
    /// <summary>An empty room with walls, of the given size in meters.</summary>
    public static OccupancyGridMap Room(double width, double height, double resolution = 0.05)
    {
        var map = new OccupancyGridMap((int)Math.Round(width / resolution), (int)Math.Round(height / resolution), resolution, default, OccupancyGridMap.Free);
        map.FillRect(0, 0, width, resolution, OccupancyGridMap.Occupied);
        map.FillRect(0, height - resolution, width, height, OccupancyGridMap.Occupied);
        map.FillRect(0, 0, resolution, height, OccupancyGridMap.Occupied);
        map.FillRect(width - resolution, 0, width, height, OccupancyGridMap.Occupied);
        return map;
    }

    /// <summary>A 12×12 m square loop corridor around a 6×6 m block, with asymmetric features.</summary>
    public static OccupancyGridMap LoopCorridor()
    {
        var map = Room(12, 12);
        map.FillRect(3, 3, 9, 9, OccupancyGridMap.Occupied);

        // Asymmetric clutter so the corridors are distinguishable.
        map.FillRect(5, 0.05, 5.4, 0.5, OccupancyGridMap.Occupied);
        map.FillRect(11.4, 6, 11.95, 6.3, OccupancyGridMap.Occupied);
        map.FillRect(2.5, 9.5, 2.95, 9.9, OccupancyGridMap.Occupied);
        map.FillRect(7, 11.5, 7.6, 11.95, OccupancyGridMap.Occupied);
        map.FillRect(0.05, 4, 0.6, 4.2, OccupancyGridMap.Occupied);
        map.FillRect(9, 2.6, 9.3, 3, OccupancyGridMap.Occupied);
        map.FillRect(0.05, 8, 0.3, 9, OccupancyGridMap.Occupied);
        return map;
    }

    public static LaserScan Scan(OccupancyGridMap map, Pose2D pose, int beams = 360, double maxRange = 10, double noise = 0.0, GaussianRandom? rng = null)
    {
        var scan = new LaserScan
        {
            AngleMin = (float)-Math.PI,
            AngleMax = (float)(Math.PI - (2 * Math.PI / beams)),
            AngleIncrement = (float)(2 * Math.PI / beams),
            RangeMin = 0.05f,
            RangeMax = (float)maxRange,
            Ranges = new float[beams],
        };
        for (var i = 0; i < beams; i++)
        {
            var a = pose.Theta + scan.AngleMin + (i * scan.AngleIncrement);
            var r = map.Raycast(pose.X, pose.Y, a, maxRange);
            if (r >= maxRange)
            {
                scan.Ranges[i] = float.PositiveInfinity;
            }
            else
            {
                scan.Ranges[i] = (float)(r + (rng?.Next(noise) ?? 0));
            }
        }

        return scan;
    }
}
