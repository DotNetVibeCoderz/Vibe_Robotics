using ROS2.Net.Navigation.Maps;

namespace ROS2.Net.Navigation.Localization;

/// <summary>
/// Distance from every cell to the nearest occupied cell (capped), computed with a nearest-obstacle
/// brushfire. The basis of AMCL's likelihood-field sensor model and of scan matching.
/// </summary>
public sealed class LikelihoodField
{
    private readonly float[] _distance;

    public LikelihoodField(OccupancyGridMap map, double maxDistance = 2.0)
    {
        Map = map;
        MaxDistance = maxDistance;
        _distance = Compute(map, maxDistance);
    }

    public OccupancyGridMap Map { get; }

    public double MaxDistance { get; }

    /// <summary>Distance in meters from the world point to the nearest obstacle (<see cref="MaxDistance"/> off-map).</summary>
    public double DistanceAt(double wx, double wy) =>
        Map.WorldToCell(wx, wy, out var cx, out var cy) ? _distance[(cy * Map.Width) + cx] : MaxDistance;

    /// <summary>Raw distance grid (row-major, meters).</summary>
    public ReadOnlySpan<float> Distances => _distance;

    private static float[] Compute(OccupancyGridMap map, double maxDistance)
    {
        var w = map.Width;
        var n = w * map.Height;
        var dist = new float[n];
        var nearest = new int[n];
        dist.AsSpan().Fill((float)maxDistance);
        nearest.AsSpan().Fill(-1);
        var queue = new Queue<int>();
        for (var i = 0; i < n; i++)
        {
            if (map.Data[i] >= map.OccupiedThreshold)
            {
                dist[i] = 0;
                nearest[i] = i;
                queue.Enqueue(i);
            }
        }

        var res = map.Resolution;
        var maxCells2 = (maxDistance / res) * (maxDistance / res);
        while (queue.Count > 0)
        {
            var i = queue.Dequeue();
            var x = i % w;
            var y = i / w;
            var o = nearest[i];
            var ox = o % w;
            var oy = o / w;
            for (var dy = -1; dy <= 1; dy++)
            {
                for (var dx = -1; dx <= 1; dx++)
                {
                    var nx = x + dx;
                    var ny = y + dy;
                    if ((dx == 0 && dy == 0) || !map.InBounds(nx, ny))
                    {
                        continue;
                    }

                    var ni = (ny * w) + nx;
                    double ddx = nx - ox, ddy = ny - oy;
                    var d2 = (ddx * ddx) + (ddy * ddy);
                    if (d2 > maxCells2)
                    {
                        continue;
                    }

                    var d = (float)(Math.Sqrt(d2) * res);
                    if (d < dist[ni])
                    {
                        dist[ni] = d;
                        nearest[ni] = o;
                        queue.Enqueue(ni);
                    }
                }
            }
        }

        return dist;
    }
}
