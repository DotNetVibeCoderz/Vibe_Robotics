using GeometryMsgs = ROS2.Net.Msgs.GeometryMsgs;

namespace ROS2.Net.Manipulation;

/// <summary>A 3D vector of doubles.</summary>
public readonly record struct Vec3(double X, double Y, double Z)
{
    public static Vec3 Zero => default;

    public static Vec3 UnitX => new(1, 0, 0);

    public static Vec3 UnitY => new(0, 1, 0);

    public static Vec3 UnitZ => new(0, 0, 1);

    public double Length => Math.Sqrt(Dot(this));

    public double LengthSquared => Dot(this);

    public Vec3 Normalized()
    {
        var l = Length;
        return l < 1e-15 ? Zero : this / l;
    }

    public double Dot(Vec3 o) => (X * o.X) + (Y * o.Y) + (Z * o.Z);

    public Vec3 Cross(Vec3 o) => new((Y * o.Z) - (Z * o.Y), (Z * o.X) - (X * o.Z), (X * o.Y) - (Y * o.X));

    public static Vec3 operator +(Vec3 a, Vec3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

    public static Vec3 operator -(Vec3 a, Vec3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

    public static Vec3 operator -(Vec3 a) => new(-a.X, -a.Y, -a.Z);

    public static Vec3 operator *(Vec3 a, double s) => new(a.X * s, a.Y * s, a.Z * s);

    public static Vec3 operator *(double s, Vec3 a) => a * s;

    public static Vec3 operator /(Vec3 a, double s) => new(a.X / s, a.Y / s, a.Z / s);

    public static Vec3 Lerp(Vec3 a, Vec3 b, double t) => a + ((b - a) * t);

    public GeometryMsgs.Point ToPoint() => new() { X = X, Y = Y, Z = Z };

    public GeometryMsgs.Vector3 ToVector3() => new() { X = X, Y = Y, Z = Z };

    public static Vec3 From(GeometryMsgs.Point p) => new(p.X, p.Y, p.Z);

    public static Vec3 From(GeometryMsgs.Vector3 v) => new(v.X, v.Y, v.Z);

    public override string ToString() => $"({X:0.####}, {Y:0.####}, {Z:0.####})";
}

/// <summary>A unit quaternion (x, y, z, w) of doubles.</summary>
public readonly record struct Quat(double X, double Y, double Z, double W)
{
    public static Quat Identity => new(0, 0, 0, 1);

    public static Quat FromAxisAngle(Vec3 axis, double angle)
    {
        var a = axis.Normalized();
        var s = Math.Sin(angle * 0.5);
        return new Quat(a.X * s, a.Y * s, a.Z * s, Math.Cos(angle * 0.5));
    }

    /// <summary>From URDF roll/pitch/yaw (fixed axes X, then Y, then Z).</summary>
    public static Quat FromRpy(double roll, double pitch, double yaw)
    {
        double cr = Math.Cos(roll * 0.5), sr = Math.Sin(roll * 0.5);
        double cp = Math.Cos(pitch * 0.5), sp = Math.Sin(pitch * 0.5);
        double cy = Math.Cos(yaw * 0.5), sy = Math.Sin(yaw * 0.5);
        return new Quat(
            (sr * cp * cy) - (cr * sp * sy),
            (cr * sp * cy) + (sr * cp * sy),
            (cr * cp * sy) - (sr * sp * cy),
            (cr * cp * cy) + (sr * sp * sy));
    }

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

    public Quat Normalized()
    {
        var n = Math.Sqrt((X * X) + (Y * Y) + (Z * Z) + (W * W));
        return n < 1e-15 ? Identity : new Quat(X / n, Y / n, Z / n, W / n);
    }

    public Quat Conjugate() => new(-X, -Y, -Z, W);

    public static Quat operator *(Quat a, Quat b) => new(
        (a.W * b.X) + (a.X * b.W) + (a.Y * b.Z) - (a.Z * b.Y),
        (a.W * b.Y) - (a.X * b.Z) + (a.Y * b.W) + (a.Z * b.X),
        (a.W * b.Z) + (a.X * b.Y) - (a.Y * b.X) + (a.Z * b.W),
        (a.W * b.W) - (a.X * b.X) - (a.Y * b.Y) - (a.Z * b.Z));

    public Vec3 Rotate(Vec3 v)
    {
        var u = new Vec3(X, Y, Z);
        var t = 2 * u.Cross(v);
        return v + (W * t) + u.Cross(t);
    }

    public double Dot(Quat o) => (X * o.X) + (Y * o.Y) + (Z * o.Z) + (W * o.W);

    /// <summary>Rotation vector (axis × angle) of this rotation, angle in [0, π].</summary>
    public Vec3 ToRotationVector()
    {
        var q = W < 0 ? new Quat(-X, -Y, -Z, -W) : this;
        var s = Math.Sqrt((q.X * q.X) + (q.Y * q.Y) + (q.Z * q.Z));
        if (s < 1e-12)
        {
            return new Vec3(2 * q.X, 2 * q.Y, 2 * q.Z);
        }

        var angle = 2 * Math.Atan2(s, q.W);
        return new Vec3(q.X, q.Y, q.Z) * (angle / s);
    }

    /// <summary>Angular distance (rad) between two rotations.</summary>
    public double AngleTo(Quat o) => 2 * Math.Acos(Math.Min(1, Math.Abs(Dot(o))));

    public static Quat Slerp(Quat a, Quat b, double t)
    {
        var d = a.Dot(b);
        if (d < 0)
        {
            b = new Quat(-b.X, -b.Y, -b.Z, -b.W);
            d = -d;
        }

        if (d > 0.9995)
        {
            return new Quat(a.X + ((b.X - a.X) * t), a.Y + ((b.Y - a.Y) * t), a.Z + ((b.Z - a.Z) * t), a.W + ((b.W - a.W) * t)).Normalized();
        }

        var theta = Math.Acos(d);
        var s = Math.Sin(theta);
        var wa = Math.Sin((1 - t) * theta) / s;
        var wb = Math.Sin(t * theta) / s;
        return new Quat((wa * a.X) + (wb * b.X), (wa * a.Y) + (wb * b.Y), (wa * a.Z) + (wb * b.Z), (wa * a.W) + (wb * b.W));
    }

    public GeometryMsgs.Quaternion ToMsg() => new() { X = X, Y = Y, Z = Z, W = W };

    public static Quat From(GeometryMsgs.Quaternion q) => new(q.X, q.Y, q.Z, q.W);
}

/// <summary>A rigid transform (rotation then translation).</summary>
public readonly record struct Pose3(Vec3 Position, Quat Rotation)
{
    public static Pose3 Identity => new(Vec3.Zero, Quat.Identity);

    public static Pose3 FromXyzRpy(double x, double y, double z, double roll = 0, double pitch = 0, double yaw = 0) =>
        new(new Vec3(x, y, z), Quat.FromRpy(roll, pitch, yaw));

    /// <summary>This ∘ other: apply <paramref name="b"/> in this frame.</summary>
    public static Pose3 operator *(Pose3 a, Pose3 b) => new(a.Position + a.Rotation.Rotate(b.Position), (a.Rotation * b.Rotation).Normalized());

    public Pose3 Inverse()
    {
        var inv = Rotation.Conjugate();
        return new Pose3(inv.Rotate(-Position), inv);
    }

    public Vec3 Apply(Vec3 p) => Position + Rotation.Rotate(p);

    public static Pose3 Interpolate(Pose3 a, Pose3 b, double t) =>
        new(Vec3.Lerp(a.Position, b.Position, t), Quat.Slerp(a.Rotation, b.Rotation, t));

    /// <summary>Row-major 4×4 homogeneous matrix.</summary>
    public double[] ToMatrix4()
    {
        var (x, y, z, w) = (Rotation.X, Rotation.Y, Rotation.Z, Rotation.W);
        return
        [
            1 - (2 * ((y * y) + (z * z))), 2 * ((x * y) - (z * w)), 2 * ((x * z) + (y * w)), Position.X,
            2 * ((x * y) + (z * w)), 1 - (2 * ((x * x) + (z * z))), 2 * ((y * z) - (x * w)), Position.Y,
            2 * ((x * z) - (y * w)), 2 * ((y * z) + (x * w)), 1 - (2 * ((x * x) + (y * y))), Position.Z,
            0, 0, 0, 1,
        ];
    }

    public GeometryMsgs.Pose ToMsg() => new() { Position = Position.ToPoint(), Orientation = Rotation.ToMsg() };

    public GeometryMsgs.Transform ToTransformMsg() => new() { Translation = Position.ToVector3(), Rotation = Rotation.ToMsg() };

    public static Pose3 From(GeometryMsgs.Pose p) => new(Vec3.From(p.Position), Quat.From(p.Orientation));

    public override string ToString() => $"{Position} {Rotation.ToRpy()}";
}

/// <summary>Small dense linear algebra for kinematics (row-major).</summary>
internal static class DenseMath
{
    /// <summary>Solves A x = b in place (A n×n, row-major) by Gaussian elimination with partial pivoting. Returns false if singular.</summary>
    public static bool Solve(double[] a, double[] b, int n)
    {
        for (var col = 0; col < n; col++)
        {
            var pivot = col;
            var best = Math.Abs(a[(col * n) + col]);
            for (var r = col + 1; r < n; r++)
            {
                var v = Math.Abs(a[(r * n) + col]);
                if (v > best)
                {
                    best = v;
                    pivot = r;
                }
            }

            if (best < 1e-14)
            {
                return false;
            }

            if (pivot != col)
            {
                for (var c = 0; c < n; c++)
                {
                    (a[(col * n) + c], a[(pivot * n) + c]) = (a[(pivot * n) + c], a[(col * n) + c]);
                }

                (b[col], b[pivot]) = (b[pivot], b[col]);
            }

            for (var r = col + 1; r < n; r++)
            {
                var f = a[(r * n) + col] / a[(col * n) + col];
                if (f == 0)
                {
                    continue;
                }

                for (var c = col; c < n; c++)
                {
                    a[(r * n) + c] -= f * a[(col * n) + c];
                }

                b[r] -= f * b[col];
            }
        }

        for (var r = n - 1; r >= 0; r--)
        {
            var s = b[r];
            for (var c = r + 1; c < n; c++)
            {
                s -= a[(r * n) + c] * b[c];
            }

            b[r] = s / a[(r * n) + r];
        }

        return true;
    }
}
