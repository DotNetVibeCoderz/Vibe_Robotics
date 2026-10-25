using System.Runtime.CompilerServices;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.GeometryMsgs;

namespace ROS2.Net.Navigation;

/// <summary>A planar pose (SE(2)): position in meters and heading in radians.</summary>
public readonly record struct Pose2D(double X, double Y, double Theta)
{
    public static Pose2D Identity => default;

    /// <summary>Composes two poses: <c>this ⊕ other</c> (apply <paramref name="other"/> in this pose's frame).</summary>
    public Pose2D Compose(Pose2D other)
    {
        var c = Math.Cos(Theta);
        var s = Math.Sin(Theta);
        return new Pose2D(X + (c * other.X) - (s * other.Y), Y + (s * other.X) + (c * other.Y), Angles.Normalize(Theta + other.Theta));
    }

    /// <summary>The inverse transform.</summary>
    public Pose2D Inverse()
    {
        var c = Math.Cos(Theta);
        var s = Math.Sin(Theta);
        return new Pose2D(-((c * X) + (s * Y)), (s * X) - (c * Y), Angles.Normalize(-Theta));
    }

    /// <summary>The pose of <paramref name="other"/> expressed in this pose's frame: <c>this⁻¹ ⊕ other</c>.</summary>
    public Pose2D Between(Pose2D other) => Inverse().Compose(other);

    /// <summary>Transforms a point from this frame to the parent frame.</summary>
    public (double X, double Y) TransformPoint(double x, double y)
    {
        var c = Math.Cos(Theta);
        var s = Math.Sin(Theta);
        return (X + (c * x) - (s * y), Y + (s * x) + (c * y));
    }

    public double DistanceTo(Pose2D other) => Math.Sqrt(DistanceSquaredTo(other));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public double DistanceSquaredTo(Pose2D other)
    {
        var dx = other.X - X;
        var dy = other.Y - Y;
        return (dx * dx) + (dy * dy);
    }

    public static Pose2D FromMsg(Pose pose) => new(pose.Position.X, pose.Position.Y, pose.Orientation.Yaw());

    public Pose ToMsg() => new()
    {
        Position = new Point { X = X, Y = Y },
        Orientation = MessageExtensions.QuaternionFromYaw(Theta),
    };

    public PoseStamped ToPoseStamped(string frameId, RosTime stamp)
    {
        var ps = new PoseStamped { Pose = ToMsg() };
        stamp.CopyTo(ps.Header.Stamp);
        ps.Header.FrameId = frameId;
        return ps;
    }

    public override string ToString() => $"({X:0.###}, {Y:0.###}, {Theta:0.###} rad)";
}

/// <summary>Angle helpers.</summary>
public static class Angles
{
    /// <summary>Wraps an angle to (-π, π].</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Normalize(double a)
    {
        if (a > Math.PI || a <= -Math.PI)
        {
            a = Math.IEEERemainder(a, 2 * Math.PI);
            if (a <= -Math.PI)
            {
                a += 2 * Math.PI;
            }
        }

        return a;
    }

    /// <summary>Shortest signed difference <c>a - b</c>.</summary>
    public static double Difference(double a, double b) => Normalize(a - b);
}

/// <summary>Gaussian random numbers from a seeded generator (deterministic tests, Box-Muller).</summary>
public sealed class GaussianRandom(int? seed = null)
{
    private readonly Random _random = seed is { } s ? new Random(s) : new Random();
    private double? _spare;

    public double NextDouble() => _random.NextDouble();

    public double Next(double stdDev = 1.0)
    {
        if (stdDev <= 0)
        {
            return 0;
        }

        if (_spare is { } spare)
        {
            _spare = null;
            return spare * stdDev;
        }

        double u, v, s;
        do
        {
            u = (_random.NextDouble() * 2) - 1;
            v = (_random.NextDouble() * 2) - 1;
            s = (u * u) + (v * v);
        }
        while (s >= 1 || s == 0);

        var m = Math.Sqrt(-2.0 * Math.Log(s) / s);
        _spare = v * m;
        return u * m * stdDev;
    }
}
