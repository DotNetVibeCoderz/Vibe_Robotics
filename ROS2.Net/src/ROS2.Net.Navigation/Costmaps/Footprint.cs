using ROS2.Net.Navigation.Maps;

namespace ROS2.Net.Navigation.Costmaps;

/// <summary>A robot footprint: a circle (<see cref="Radius"/>) or a polygon in the robot frame.</summary>
public sealed class Footprint
{
    private Footprint(double radius, (double X, double Y)[] polygon)
    {
        Radius = radius;
        Polygon = polygon;
    }

    /// <summary>Circle radius, or the circumscribed radius of the polygon.</summary>
    public double Radius { get; }

    /// <summary>Polygon vertices in the robot frame (empty for circular robots).</summary>
    public IReadOnlyList<(double X, double Y)> Polygon { get; }

    public bool IsCircular => Polygon.Count == 0;

    /// <summary>Inscribed radius (distance from center to the closest polygon edge).</summary>
    public double InscribedRadius { get; private init; }

    public static Footprint Circle(double radius) => new(radius, []) { InscribedRadius = radius };

    public static Footprint FromPolygon(params (double X, double Y)[] vertices)
    {
        if (vertices.Length < 3)
        {
            throw new ArgumentException("A polygon footprint needs at least 3 vertices.", nameof(vertices));
        }

        var circumscribed = vertices.Max(v => Math.Sqrt((v.X * v.X) + (v.Y * v.Y)));
        var inscribed = double.MaxValue;
        for (var i = 0; i < vertices.Length; i++)
        {
            var a = vertices[i];
            var b = vertices[(i + 1) % vertices.Length];
            inscribed = Math.Min(inscribed, DistanceToSegment(0, 0, a.X, a.Y, b.X, b.Y));
        }

        return new Footprint(circumscribed, vertices) { InscribedRadius = inscribed };
    }

    /// <summary>A rectangle centered on the robot origin.</summary>
    public static Footprint Rectangle(double length, double width) =>
        FromPolygon((length / 2, width / 2), (length / 2, -width / 2), (-length / 2, -width / 2), (-length / 2, width / 2));

    /// <summary>
    /// Highest cost under the footprint at <paramref name="pose"/>. For circles this is the center
    /// cell's cost (the inflation layer already encodes the radius); polygons rasterize their outline.
    /// Returns <see cref="Costs.Lethal"/> when any part is off the map.
    /// </summary>
    public byte Cost(Costmap2D costmap, Pose2D pose)
    {
        if (IsCircular)
        {
            return costmap.GetCostAt(pose.X, pose.Y);
        }

        byte max = 0;
        for (var i = 0; i < Polygon.Count; i++)
        {
            var (ax, ay) = pose.TransformPoint(Polygon[i].X, Polygon[i].Y);
            var (bx, by) = pose.TransformPoint(Polygon[(i + 1) % Polygon.Count].X, Polygon[(i + 1) % Polygon.Count].Y);
            if (!costmap.WorldToCell(ax, ay, out var x0, out var y0) || !costmap.WorldToCell(bx, by, out var x1, out var y1))
            {
                return Costs.Lethal;
            }

            OccupancyGridMap.RayTrace(x0, y0, x1, y1, (x, y) =>
            {
                var c = costmap.GetCost(x, y);
                if (c != Costs.NoInformation && c > max)
                {
                    max = c;
                }

                return max != Costs.Lethal;
            });
            if (max == Costs.Lethal)
            {
                return max;
            }
        }

        return max;
    }

    /// <summary>True when the footprint at <paramref name="pose"/> overlaps an obstacle.</summary>
    public bool InCollision(Costmap2D costmap, Pose2D pose)
    {
        var c = Cost(costmap, pose);
        return IsCircular ? c >= Costs.Inscribed && c != Costs.NoInformation : c == Costs.Lethal;
    }

    private static double DistanceToSegment(double px, double py, double ax, double ay, double bx, double by)
    {
        var dx = bx - ax;
        var dy = by - ay;
        var len2 = (dx * dx) + (dy * dy);
        var t = len2 == 0 ? 0 : Math.Clamp((((px - ax) * dx) + ((py - ay) * dy)) / len2, 0, 1);
        var cx = ax + (t * dx) - px;
        var cy = ay + (t * dy) - py;
        return Math.Sqrt((cx * cx) + (cy * cy));
    }
}
