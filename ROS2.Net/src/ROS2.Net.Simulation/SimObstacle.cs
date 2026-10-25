namespace ROS2.Net.Simulation;

/// <summary>Primitive shape of an obstacle.</summary>
public enum ShapeKind
{
    /// <summary>Oriented rectangle (shelves, crates, desks).</summary>
    Box,
    /// <summary>Vertical cylinder (pillars, trees, barrels).</summary>
    Cylinder,
    /// <summary>Arbitrary simple polygon.</summary>
    Polygon,
    /// <summary>A thick line segment; stored internally as an oriented box.</summary>
    Wall,
}

/// <summary>
/// A static or dynamic obstacle. Geometry is planar (for physics, LiDAR and maps) with a height used
/// by renderers. Dynamic obstacles can be pushed by robots.
/// </summary>
public sealed class SimObstacle
{
    private Vec2[] _localVertices = [];
    private Vec2[] _vertices = [];
    private Vec2 _initialCenter;
    private double _initialRotation;

    private SimObstacle(string name, ShapeKind kind)
    {
        Name = name;
        Kind = kind;
    }

    public string Name { get; }

    public ShapeKind Kind { get; }

    /// <summary>Center (box, cylinder, wall) or centroid (polygon) in world coordinates.</summary>
    public Vec2 Center { get; private set; }

    /// <summary>Rotation in radians (boxes, walls and polygons).</summary>
    public double Rotation { get; private set; }

    /// <summary>Half extents of a box or wall (along its local X and Y).</summary>
    public Vec2 HalfExtents { get; private set; }

    /// <summary>Radius of a cylinder.</summary>
    public double Radius { get; private set; }

    /// <summary>Height for rendering (meters).</summary>
    public double Height { get; set; } = 1.0;

    public SimColor Color { get; set; } = SimColor.Wall;

    /// <summary>Pushable by robots.</summary>
    public bool IsDynamic { get; init; }

    /// <summary>Mass of a dynamic obstacle (kg); heavier objects move less when pushed.</summary>
    public double Mass { get; init; } = 5;

    /// <summary>Free-form tag (e.g. <c>shelf</c>, <c>wall</c>, <c>crop</c>).</summary>
    public string Tag { get; init; } = string.Empty;

    /// <summary>World-space outline (empty for cylinders).</summary>
    public ReadOnlySpan<Vec2> Vertices => _vertices;

    /// <summary>World-space bounding box.</summary>
    public Bounds2D Bounds { get; private set; }

    /// <summary>Incremented whenever the obstacle moves.</summary>
    public int Version { get; private set; }

    public bool IsCircle => Kind == ShapeKind.Cylinder;

    public static SimObstacle Box(string name, double x, double y, double width, double depth, double rotation = 0, double height = 1, SimColor? color = null, bool dynamic = false, double mass = 5, string tag = "")
    {
        var o = new SimObstacle(name, ShapeKind.Box) { IsDynamic = dynamic, Mass = mass, Tag = tag };
        o.HalfExtents = new Vec2(width / 2, depth / 2);
        o._localVertices =
        [
            new(-width / 2, -depth / 2), new(width / 2, -depth / 2), new(width / 2, depth / 2), new(-width / 2, depth / 2),
        ];
        o.Height = height;
        o.Color = color ?? (dynamic ? SimColor.Crate : SimColor.Shelf);
        o.Place(new Vec2(x, y), rotation);
        o.MarkInitial();
        return o;
    }

    public static SimObstacle Cylinder(string name, double x, double y, double radius, double height = 1, SimColor? color = null, bool dynamic = false, string tag = "")
    {
        var o = new SimObstacle(name, ShapeKind.Cylinder) { IsDynamic = dynamic, Tag = tag, Radius = radius, Height = height };
        o.Color = color ?? SimColor.Gray;
        o.Place(new Vec2(x, y), 0);
        o.MarkInitial();
        return o;
    }

    public static SimObstacle Wall(string name, Vec2 a, Vec2 b, double thickness = 0.1, double height = 1.2, SimColor? color = null)
    {
        var d = b - a;
        var length = Math.Max(1e-6, d.Length);
        var o = new SimObstacle(name, ShapeKind.Wall) { Tag = "wall" };
        o.HalfExtents = new Vec2(length / 2, thickness / 2);
        o._localVertices =
        [
            new(-length / 2, -thickness / 2), new(length / 2, -thickness / 2), new(length / 2, thickness / 2), new(-length / 2, thickness / 2),
        ];
        o.Height = height;
        o.Color = color ?? SimColor.Wall;
        o.Place((a + b) * 0.5, Math.Atan2(d.Y, d.X));
        o.MarkInitial();
        return o;
    }

    /// <summary>A polygon from world-space points (counter-clockwise or clockwise).</summary>
    public static SimObstacle Polygon(string name, IReadOnlyList<Vec2> points, double height = 1, SimColor? color = null, string tag = "")
    {
        if (points.Count < 3)
        {
            throw new ArgumentException("A polygon needs at least three points.", nameof(points));
        }

        var cx = points.Average(p => p.X);
        var cy = points.Average(p => p.Y);
        var c = new Vec2(cx, cy);
        var o = new SimObstacle(name, ShapeKind.Polygon) { Tag = tag };
        o._localVertices = points.Select(p => p - c).ToArray();
        o.Height = height;
        o.Color = color ?? SimColor.Gray;
        o.Place(c, 0);
        o.MarkInitial();
        return o;
    }

    /// <summary>Moves the obstacle (used for dynamic objects and editors).</summary>
    public void Place(Vec2 center, double rotation)
    {
        Center = center;
        Rotation = rotation;
        if (Kind == ShapeKind.Cylinder)
        {
            Bounds = new Bounds2D(center.X - Radius, center.Y - Radius, center.X + Radius, center.Y + Radius);
        }
        else
        {
            if (_vertices.Length != _localVertices.Length)
            {
                _vertices = new Vec2[_localVertices.Length];
            }

            var (s, c) = Math.SinCos(rotation);
            double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
            for (var i = 0; i < _localVertices.Length; i++)
            {
                var l = _localVertices[i];
                var w = new Vec2(center.X + (c * l.X) - (s * l.Y), center.Y + (s * l.X) + (c * l.Y));
                _vertices[i] = w;
                minX = Math.Min(minX, w.X);
                minY = Math.Min(minY, w.Y);
                maxX = Math.Max(maxX, w.X);
                maxY = Math.Max(maxY, w.Y);
            }

            Bounds = new Bounds2D(minX, minY, maxX, maxY);
        }

        Version++;
    }

    internal void MarkInitial()
    {
        _initialCenter = Center;
        _initialRotation = Rotation;
    }

    internal void ResetToInitial() => Place(_initialCenter, _initialRotation);

    /// <summary>True when the point is inside the shape (optionally inflated by <paramref name="inflate"/>).</summary>
    public bool Contains(Vec2 p, double inflate = 0)
    {
        if (p.X < Bounds.MinX - inflate || p.X > Bounds.MaxX + inflate || p.Y < Bounds.MinY - inflate || p.Y > Bounds.MaxY + inflate)
        {
            return false;
        }

        if (Kind == ShapeKind.Cylinder)
        {
            var r = Radius + inflate;
            return (p - Center).LengthSquared <= r * r;
        }

        if (PointInPolygon(_vertices, p))
        {
            return true;
        }

        return inflate > 0 && DistanceToBoundary(p, out _) <= inflate;
    }

    /// <summary>Distance along a ray to the shape, or <see cref="double.PositiveInfinity"/> if missed.</summary>
    public double RayIntersect(Vec2 origin, Vec2 dir, double maxRange)
    {
        if (Kind == ShapeKind.Cylinder)
        {
            return RayCircle(origin, dir, Center, Radius, maxRange);
        }

        var best = double.PositiveInfinity;
        var v = _vertices;
        for (int i = 0, j = v.Length - 1; i < v.Length; j = i++)
        {
            var t = RaySegment(origin, dir, v[j], v[i]);
            if (t < best && t <= maxRange)
            {
                best = t;
            }
        }

        return best;
    }

    /// <summary>
    /// For a circle of radius <paramref name="radius"/> at <paramref name="center"/>, returns the
    /// penetration depth and the direction to push the circle out (zero when not overlapping).
    /// </summary>
    public double Penetration(Vec2 center, double radius, out Vec2 pushDirection)
    {
        pushDirection = Vec2.Zero;
        if (center.X < Bounds.MinX - radius || center.X > Bounds.MaxX + radius || center.Y < Bounds.MinY - radius || center.Y > Bounds.MaxY + radius)
        {
            return 0;
        }

        if (Kind == ShapeKind.Cylinder)
        {
            var d = center - Center;
            var dist = d.Length;
            var depth = Radius + radius - dist;
            if (depth <= 0)
            {
                return 0;
            }

            pushDirection = dist > 1e-9 ? d * (1 / dist) : new Vec2(1, 0);
            return depth;
        }

        var boundaryDistance = DistanceToBoundary(center, out var closest);
        var inside = PointInPolygon(_vertices, center);
        if (!inside && boundaryDistance >= radius)
        {
            return 0;
        }

        var away = center - closest;
        var len = away.Length;
        var dir = len > 1e-9 ? away * (1 / len) : (center - Center).Normalized();
        if (inside)
        {
            pushDirection = -dir;
            return boundaryDistance + radius;
        }

        pushDirection = dir;
        return radius - boundaryDistance;
    }

    /// <summary>Distance from a point to the outline, with the closest outline point.</summary>
    public double DistanceToBoundary(Vec2 p, out Vec2 closest)
    {
        if (Kind == ShapeKind.Cylinder)
        {
            var d = p - Center;
            var len = d.Length;
            closest = len > 1e-9 ? Center + (d * (Radius / len)) : Center + new Vec2(Radius, 0);
            return Math.Abs(len - Radius);
        }

        var best = double.MaxValue;
        closest = Center;
        var v = _vertices;
        for (int i = 0, j = v.Length - 1; i < v.Length; j = i++)
        {
            var c = ClosestOnSegment(p, v[j], v[i]);
            var dist = (p - c).LengthSquared;
            if (dist < best)
            {
                best = dist;
                closest = c;
            }
        }

        return Math.Sqrt(best);
    }

    internal static bool PointInPolygon(ReadOnlySpan<Vec2> v, Vec2 p)
    {
        var inside = false;
        for (int i = 0, j = v.Length - 1; i < v.Length; j = i++)
        {
            if (((v[i].Y > p.Y) != (v[j].Y > p.Y)) &&
                (p.X < ((v[j].X - v[i].X) * (p.Y - v[i].Y) / (v[j].Y - v[i].Y)) + v[i].X))
            {
                inside = !inside;
            }
        }

        return inside;
    }

    internal static Vec2 ClosestOnSegment(Vec2 p, Vec2 a, Vec2 b)
    {
        var ab = b - a;
        var l2 = ab.LengthSquared;
        if (l2 < 1e-18)
        {
            return a;
        }

        var t = Math.Clamp(Vec2.Dot(p - a, ab) / l2, 0, 1);
        return a + (ab * t);
    }

    /// <summary>Ray/segment intersection distance, or +∞.</summary>
    internal static double RaySegment(Vec2 o, Vec2 d, Vec2 a, Vec2 b)
    {
        var e = b - a;
        var denom = Vec2.Cross(d, e);
        if (Math.Abs(denom) < 1e-12)
        {
            return double.PositiveInfinity;
        }

        var ao = a - o;
        var t = Vec2.Cross(ao, e) / denom;
        var u = Vec2.Cross(ao, d) / denom;
        return t >= 0 && u >= 0 && u <= 1 ? t : double.PositiveInfinity;
    }

    /// <summary>Ray/circle intersection distance (entry point), or +∞.</summary>
    internal static double RayCircle(Vec2 o, Vec2 d, Vec2 c, double r, double maxRange)
    {
        var oc = o - c;
        var b = Vec2.Dot(oc, d);
        var cc = oc.LengthSquared - (r * r);
        var disc = (b * b) - cc;
        if (disc < 0)
        {
            return double.PositiveInfinity;
        }

        var sq = Math.Sqrt(disc);
        var t = -b - sq;
        if (t < 0)
        {
            t = -b + sq; // origin inside the circle
        }

        return t >= 0 && t <= maxRange ? t : double.PositiveInfinity;
    }

    public override string ToString() => $"{Kind} '{Name}' at {Center}";
}
