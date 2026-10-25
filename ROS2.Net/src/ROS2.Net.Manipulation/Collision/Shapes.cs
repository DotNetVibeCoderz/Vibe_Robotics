namespace ROS2.Net.Manipulation;

/// <summary>
/// A collision primitive in world coordinates. Cylinders are approximated by capsules (conservative at
/// the caps), meshes are not supported (approximate them with primitives in the URDF collision tags).
/// </summary>
public abstract record CollisionShape
{
    /// <summary>True when the shapes overlap or are closer than <paramref name="margin"/>.</summary>
    public bool Intersects(CollisionShape other, double margin = 0) => Distance(this, other) <= margin;

    /// <summary>Approximate separation distance (negative or zero when overlapping).</summary>
    public static double Distance(CollisionShape a, CollisionShape b) => (a, b) switch
    {
        (SphereShape s1, SphereShape s2) => (s1.Center - s2.Center).Length - s1.Radius - s2.Radius,
        (SphereShape s, CapsuleShape c) => Geometry3.PointSegmentDistance(s.Center, c.A, c.B) - s.Radius - c.Radius,
        (CapsuleShape c, SphereShape s) => Distance(s, c),
        (CapsuleShape c1, CapsuleShape c2) => Geometry3.SegmentSegmentDistance(c1.A, c1.B, c2.A, c2.B) - c1.Radius - c2.Radius,
        (SphereShape s, BoxShape bx) => bx.DistanceToPoint(s.Center) - s.Radius,
        (BoxShape bx, SphereShape s) => Distance(s, bx),
        (CapsuleShape c, BoxShape bx) => bx.DistanceToSegment(c.A, c.B) - c.Radius,
        (BoxShape bx, CapsuleShape c) => Distance(c, bx),
        (BoxShape b1, BoxShape b2) => BoxShape.Overlaps(b1, b2) ? 0 : b1.SeparationLowerBound(b2),
        _ => double.PositiveInfinity,
    };

    /// <summary>The shape moved by <paramref name="pose"/>.</summary>
    public abstract CollisionShape Transformed(Pose3 pose);
}

public sealed record SphereShape(Vec3 Center, double Radius) : CollisionShape
{
    public override CollisionShape Transformed(Pose3 pose) => this with { Center = pose.Apply(Center) };
}

/// <summary>A segment A–B swept by a sphere.</summary>
public sealed record CapsuleShape(Vec3 A, Vec3 B, double Radius) : CollisionShape
{
    public override CollisionShape Transformed(Pose3 pose) => this with { A = pose.Apply(A), B = pose.Apply(B) };
}

/// <summary>An oriented box with half extents along its local axes.</summary>
public sealed record BoxShape(Pose3 Pose, Vec3 HalfExtents) : CollisionShape
{
    public override CollisionShape Transformed(Pose3 pose) => this with { Pose = pose * Pose };

    public double DistanceToPoint(Vec3 p)
    {
        var local = Pose.Inverse().Apply(p);
        var dx = Math.Max(Math.Abs(local.X) - HalfExtents.X, 0);
        var dy = Math.Max(Math.Abs(local.Y) - HalfExtents.Y, 0);
        var dz = Math.Max(Math.Abs(local.Z) - HalfExtents.Z, 0);
        var outside = Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
        if (outside > 0)
        {
            return outside;
        }

        // Inside: negative depth to the nearest face.
        return -Math.Min(HalfExtents.X - Math.Abs(local.X), Math.Min(HalfExtents.Y - Math.Abs(local.Y), HalfExtents.Z - Math.Abs(local.Z)));
    }

    /// <summary>Distance from a segment, by golden-section search along it (the point-box distance is convex in t).</summary>
    public double DistanceToSegment(Vec3 a, Vec3 b)
    {
        double lo = 0, hi = 1;
        const double phi = 0.6180339887498949;
        var x1 = hi - (phi * (hi - lo));
        var x2 = lo + (phi * (hi - lo));
        var f1 = DistanceToPoint(Vec3.Lerp(a, b, x1));
        var f2 = DistanceToPoint(Vec3.Lerp(a, b, x2));
        for (var i = 0; i < 40; i++)
        {
            if (f1 < f2)
            {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - (phi * (hi - lo));
                f1 = DistanceToPoint(Vec3.Lerp(a, b, x1));
            }
            else
            {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + (phi * (hi - lo));
                f2 = DistanceToPoint(Vec3.Lerp(a, b, x2));
            }
        }

        return Math.Min(Math.Min(f1, f2), Math.Min(DistanceToPoint(a), DistanceToPoint(b)));
    }

    private Vec3[] Axes() => [Pose.Rotation.Rotate(Vec3.UnitX), Pose.Rotation.Rotate(Vec3.UnitY), Pose.Rotation.Rotate(Vec3.UnitZ)];

    private double Project(Vec3 axis, Vec3[] axes) =>
        (HalfExtents.X * Math.Abs(axes[0].Dot(axis))) + (HalfExtents.Y * Math.Abs(axes[1].Dot(axis))) + (HalfExtents.Z * Math.Abs(axes[2].Dot(axis)));

    /// <summary>Separating-axis test over the 15 candidate axes.</summary>
    public static bool Overlaps(BoxShape a, BoxShape b) => a.MaxSeparation(b) <= 0;

    /// <summary>Largest separation along any SAT axis: a lower bound of the true distance (0 or less when overlapping).</summary>
    public double SeparationLowerBound(BoxShape other) => Math.Max(0, MaxSeparation(other));

    private double MaxSeparation(BoxShape other)
    {
        var aa = Axes();
        var ba = other.Axes();
        var d = other.Pose.Position - Pose.Position;
        var best = double.NegativeInfinity;
        void Test(Vec3 axis)
        {
            var len = axis.Length;
            if (len < 1e-9)
            {
                return;
            }

            axis /= len;
            var sep = Math.Abs(d.Dot(axis)) - Project(axis, aa) - other.Project(axis, ba);
            if (sep > best)
            {
                best = sep;
            }
        }

        foreach (var x in aa)
        {
            Test(x);
        }

        foreach (var y in ba)
        {
            Test(y);
        }

        foreach (var x in aa)
        {
            foreach (var y in ba)
            {
                Test(x.Cross(y));
            }
        }

        return best;
    }
}

/// <summary>Distance helpers.</summary>
public static class Geometry3
{
    public static double PointSegmentDistance(Vec3 p, Vec3 a, Vec3 b)
    {
        var ab = b - a;
        var len2 = ab.LengthSquared;
        var t = len2 < 1e-18 ? 0 : Math.Clamp((p - a).Dot(ab) / len2, 0, 1);
        return (p - (a + (ab * t))).Length;
    }

    /// <summary>Closest distance between segments p1–q1 and p2–q2 (Ericson, Real-Time Collision Detection 5.1.9).</summary>
    public static double SegmentSegmentDistance(Vec3 p1, Vec3 q1, Vec3 p2, Vec3 q2)
    {
        var d1 = q1 - p1;
        var d2 = q2 - p2;
        var r = p1 - p2;
        var a = d1.Dot(d1);
        var e = d2.Dot(d2);
        var f = d2.Dot(r);
        double s, t;
        if (a <= 1e-18 && e <= 1e-18)
        {
            return r.Length;
        }

        if (a <= 1e-18)
        {
            s = 0;
            t = Math.Clamp(f / e, 0, 1);
        }
        else
        {
            var c = d1.Dot(r);
            if (e <= 1e-18)
            {
                t = 0;
                s = Math.Clamp(-c / a, 0, 1);
            }
            else
            {
                var b = d1.Dot(d2);
                var denom = (a * e) - (b * b);
                s = denom > 1e-18 ? Math.Clamp(((b * f) - (c * e)) / denom, 0, 1) : 0;
                t = ((b * s) + f) / e;
                if (t < 0)
                {
                    t = 0;
                    s = Math.Clamp(-c / a, 0, 1);
                }
                else if (t > 1)
                {
                    t = 1;
                    s = Math.Clamp((b - c) / a, 0, 1);
                }
            }
        }

        return ((p1 + (d1 * s)) - (p2 + (d2 * t))).Length;
    }

    /// <summary>Converts URDF geometry at an origin into a collision primitive in the link frame, or null for meshes.</summary>
    public static CollisionShape? FromGeometry(GeometryElement element) => element.Geometry switch
    {
        SphereGeometry s => new SphereShape(element.Origin.Position, s.Radius),
        BoxGeometry b => new BoxShape(element.Origin, b.Size / 2),
        CylinderGeometry c => new CapsuleShape(
            element.Origin.Apply(new Vec3(0, 0, -c.Length / 2)),
            element.Origin.Apply(new Vec3(0, 0, c.Length / 2)),
            c.Radius),
        _ => null,
    };
}
