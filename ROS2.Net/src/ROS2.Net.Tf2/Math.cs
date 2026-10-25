using System.Runtime.CompilerServices;
using ROS2.Net.Msgs.GeometryMsgs;

namespace ROS2.Net.Tf2;

/// <summary>An immutable 3D vector of doubles.</summary>
public readonly record struct Vector3d(double X, double Y, double Z)
{
    public static Vector3d Zero => default;

    public static Vector3d UnitX => new(1, 0, 0);

    public static Vector3d UnitY => new(0, 1, 0);

    public static Vector3d UnitZ => new(0, 0, 1);

    public double Length => Math.Sqrt(LengthSquared);

    public double LengthSquared => (X * X) + (Y * Y) + (Z * Z);

    public Vector3d Normalized()
    {
        var l = Length;
        return l > 0 ? this / l : this;
    }

    public static double Dot(Vector3d a, Vector3d b) => (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z);

    public static Vector3d Cross(Vector3d a, Vector3d b) =>
        new((a.Y * b.Z) - (a.Z * b.Y), (a.Z * b.X) - (a.X * b.Z), (a.X * b.Y) - (a.Y * b.X));

    public static Vector3d Lerp(Vector3d a, Vector3d b, double t) => a + ((b - a) * t);

    public static double Distance(Vector3d a, Vector3d b) => (a - b).Length;

    public static Vector3d operator +(Vector3d a, Vector3d b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    public static Vector3d operator -(Vector3d a, Vector3d b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    public static Vector3d operator -(Vector3d a) => new(-a.X, -a.Y, -a.Z);

    public static Vector3d operator *(Vector3d a, double s) => new(a.X * s, a.Y * s, a.Z * s);

    public static Vector3d operator *(double s, Vector3d a) => a * s;

    public static Vector3d operator /(Vector3d a, double s) => new(a.X / s, a.Y / s, a.Z / s);

    public Vector3 ToVector3Msg() => new() { X = X, Y = Y, Z = Z };

    public Point ToPointMsg() => new() { X = X, Y = Y, Z = Z };

    public static Vector3d FromMsg(Vector3 v) => new(v.X, v.Y, v.Z);

    public static Vector3d FromMsg(Point p) => new(p.X, p.Y, p.Z);

    public override string ToString() => $"[{X:0.######}, {Y:0.######}, {Z:0.######}]";
}

/// <summary>An immutable unit quaternion (x, y, z, w) representing a rotation.</summary>
public readonly record struct Quaterniond(double X, double Y, double Z, double W)
{
    public static Quaterniond Identity => new(0, 0, 0, 1);

    public double Norm => Math.Sqrt((X * X) + (Y * Y) + (Z * Z) + (W * W));

    public Quaterniond Normalized()
    {
        var n = Norm;
        return n > 0 ? new Quaterniond(X / n, Y / n, Z / n, W / n) : Identity;
    }

    /// <summary>Inverse of a unit quaternion (its conjugate).</summary>
    public Quaterniond Inverse() => new(-X, -Y, -Z, W);

    public static Quaterniond operator *(Quaterniond a, Quaterniond b) => new(
        (a.W * b.X) + (a.X * b.W) + (a.Y * b.Z) - (a.Z * b.Y),
        (a.W * b.Y) - (a.X * b.Z) + (a.Y * b.W) + (a.Z * b.X),
        (a.W * b.Z) + (a.X * b.Y) - (a.Y * b.X) + (a.Z * b.W),
        (a.W * b.W) - (a.X * b.X) - (a.Y * b.Y) - (a.Z * b.Z));

    /// <summary>Rotates a vector.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vector3d Rotate(Vector3d v)
    {
        // v' = v + 2w(q×v) + 2q×(q×v), cheaper than two quaternion products.
        var q = new Vector3d(X, Y, Z);
        var t = 2 * Vector3d.Cross(q, v);
        return v + (W * t) + Vector3d.Cross(q, t);
    }

    public static Vector3d operator *(Quaterniond q, Vector3d v) => q.Rotate(v);

    public static double Dot(Quaterniond a, Quaterniond b) => (a.X * b.X) + (a.Y * b.Y) + (a.Z * b.Z) + (a.W * b.W);

    /// <summary>Spherical linear interpolation along the shortest arc.</summary>
    public static Quaterniond Slerp(Quaterniond a, Quaterniond b, double t)
    {
        var dot = Dot(a, b);
        if (dot < 0)
        {
            b = new Quaterniond(-b.X, -b.Y, -b.Z, -b.W);
            dot = -dot;
        }

        if (dot > 0.9995)
        {
            return new Quaterniond(
                a.X + (t * (b.X - a.X)),
                a.Y + (t * (b.Y - a.Y)),
                a.Z + (t * (b.Z - a.Z)),
                a.W + (t * (b.W - a.W))).Normalized();
        }

        var theta0 = Math.Acos(dot);
        var theta = theta0 * t;
        var sinTheta0 = Math.Sin(theta0);
        var s0 = Math.Cos(theta) - (dot * Math.Sin(theta) / sinTheta0);
        var s1 = Math.Sin(theta) / sinTheta0;
        return new Quaterniond(
            (s0 * a.X) + (s1 * b.X),
            (s0 * a.Y) + (s1 * b.Y),
            (s0 * a.Z) + (s1 * b.Z),
            (s0 * a.W) + (s1 * b.W));
    }

    /// <summary>From roll/pitch/yaw in radians (fixed axes X, then Y, then Z — the tf2 convention).</summary>
    public static Quaterniond FromRpy(double roll, double pitch, double yaw)
    {
        double cr = Math.Cos(roll * 0.5), sr = Math.Sin(roll * 0.5);
        double cp = Math.Cos(pitch * 0.5), sp = Math.Sin(pitch * 0.5);
        double cy = Math.Cos(yaw * 0.5), sy = Math.Sin(yaw * 0.5);
        return new Quaterniond(
            (sr * cp * cy) - (cr * sp * sy),
            (cr * sp * cy) + (sr * cp * sy),
            (cr * cp * sy) - (sr * sp * cy),
            (cr * cp * cy) + (sr * sp * sy));
    }

    public static Quaterniond FromYaw(double yaw) => FromRpy(0, 0, yaw);

    public (double Roll, double Pitch, double Yaw) ToRpy()
    {
        var sinr = 2 * ((W * X) + (Y * Z));
        var cosr = 1 - (2 * ((X * X) + (Y * Y)));
        var sinp = 2 * ((W * Y) - (Z * X));
        var pitch = Math.Abs(sinp) >= 1 ? Math.CopySign(Math.PI / 2, sinp) : Math.Asin(sinp);
        var siny = 2 * ((W * Z) + (X * Y));
        var cosy = 1 - (2 * ((Y * Y) + (Z * Z)));
        return (Math.Atan2(sinr, cosr), pitch, Math.Atan2(siny, cosy));
    }

    public double Yaw => ToRpy().Yaw;

    /// <summary>From a rotation of <paramref name="angle"/> radians about <paramref name="axis"/>.</summary>
    public static Quaterniond FromAxisAngle(Vector3d axis, double angle)
    {
        var n = axis.Normalized();
        var s = Math.Sin(angle * 0.5);
        return new Quaterniond(n.X * s, n.Y * s, n.Z * s, Math.Cos(angle * 0.5));
    }

    public (Vector3d Axis, double Angle) ToAxisAngle()
    {
        var q = W < 0 ? new Quaterniond(-X, -Y, -Z, -W) : this;
        var angle = 2 * Math.Acos(Math.Clamp(q.W, -1, 1));
        var s = Math.Sqrt(1 - (q.W * q.W));
        return s < 1e-12 ? (Vector3d.UnitX, 0) : (new Vector3d(q.X / s, q.Y / s, q.Z / s), angle);
    }

    /// <summary>Angle in radians between two rotations.</summary>
    public static double AngleBetween(Quaterniond a, Quaterniond b) =>
        2 * Math.Acos(Math.Clamp(Math.Abs(Dot(a.Normalized(), b.Normalized())), 0, 1));

    public Quaternion ToMsg() => new() { X = X, Y = Y, Z = Z, W = W };

    public static Quaterniond FromMsg(Quaternion q) => new(q.X, q.Y, q.Z, q.W);

    public override string ToString() => $"[{X:0.######}, {Y:0.######}, {Z:0.######}, {W:0.######}]";
}

/// <summary>A rigid transform: rotation followed by translation (p' = R·p + t).</summary>
public readonly record struct Transform3d(Vector3d Translation, Quaterniond Rotation)
{
    public static Transform3d Identity => new(Vector3d.Zero, Quaterniond.Identity);

    /// <summary>Composition: (a * b) applies b first, then a.</summary>
    public static Transform3d operator *(Transform3d a, Transform3d b) =>
        new(a.Translation + a.Rotation.Rotate(b.Translation), (a.Rotation * b.Rotation).Normalized());

    public Transform3d Inverse()
    {
        var inv = Rotation.Inverse();
        return new Transform3d(-inv.Rotate(Translation), inv);
    }

    public Vector3d TransformPoint(Vector3d p) => Rotation.Rotate(p) + Translation;

    public Vector3d TransformVector(Vector3d v) => Rotation.Rotate(v);

    public static Vector3d operator *(Transform3d t, Vector3d p) => t.TransformPoint(p);

    /// <summary>Interpolates translation linearly and rotation by slerp.</summary>
    public static Transform3d Interpolate(Transform3d a, Transform3d b, double t) =>
        new(Vector3d.Lerp(a.Translation, b.Translation, t), Quaterniond.Slerp(a.Rotation, b.Rotation, t));

    public Transform ToMsg() => new() { Translation = Translation.ToVector3Msg(), Rotation = Rotation.ToMsg() };

    public Pose ToPoseMsg() => new() { Position = Translation.ToPointMsg(), Orientation = Rotation.ToMsg() };

    public static Transform3d FromMsg(Transform t) => new(Vector3d.FromMsg(t.Translation), Quaterniond.FromMsg(t.Rotation));

    public static Transform3d FromMsg(Pose p) => new(Vector3d.FromMsg(p.Position), Quaterniond.FromMsg(p.Orientation));

    /// <summary>Planar transform helper.</summary>
    public static Transform3d From2D(double x, double y, double yaw) => new(new Vector3d(x, y, 0), Quaterniond.FromYaw(yaw));

    public override string ToString() => $"t={Translation} q={Rotation}";
}
