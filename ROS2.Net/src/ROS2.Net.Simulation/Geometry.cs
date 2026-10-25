using System.Runtime.CompilerServices;

namespace ROS2.Net.Simulation;

/// <summary>A 2D vector in world coordinates (meters).</summary>
public readonly record struct Vec2(double X, double Y)
{
    public static Vec2 Zero => default;

    public double Length => Math.Sqrt((X * X) + (Y * Y));

    public double LengthSquared => (X * X) + (Y * Y);

    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);

    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);

    public static Vec2 operator *(Vec2 a, double s) => new(a.X * s, a.Y * s);

    public static Vec2 operator -(Vec2 a) => new(-a.X, -a.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Dot(Vec2 a, Vec2 b) => (a.X * b.X) + (a.Y * b.Y);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Cross(Vec2 a, Vec2 b) => (a.X * b.Y) - (a.Y * b.X);

    public Vec2 Rotate(double angle)
    {
        var (s, c) = Math.SinCos(angle);
        return new Vec2((c * X) - (s * Y), (s * X) + (c * Y));
    }

    public Vec2 Normalized()
    {
        var l = Length;
        return l > 1e-12 ? new Vec2(X / l, Y / l) : Zero;
    }

    public override string ToString() => $"({X:0.###}, {Y:0.###})";
}

/// <summary>A planar pose: position and heading (radians, counter-clockwise from +X).</summary>
public readonly record struct Pose2D(double X, double Y, double Theta)
{
    public Vec2 Position => new(X, Y);

    /// <summary>Transforms a point from this pose's frame into the parent frame.</summary>
    public Vec2 TransformPoint(Vec2 local) => Position + local.Rotate(Theta);

    /// <summary>Transforms a point from the parent frame into this pose's frame.</summary>
    public Vec2 InverseTransformPoint(Vec2 world) => (world - Position).Rotate(-Theta);

    /// <summary>Composes <c>this ∘ local</c>.</summary>
    public Pose2D Compose(Pose2D local)
    {
        var p = TransformPoint(local.Position);
        return new Pose2D(p.X, p.Y, Angles.Normalize(Theta + local.Theta));
    }

    public override string ToString() => $"({X:0.###}, {Y:0.###}, {Theta:0.###} rad)";
}

/// <summary>Angle helpers.</summary>
public static class Angles
{
    /// <summary>Wraps an angle into (-π, π].</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Normalize(double a)
    {
        a %= 2 * Math.PI;
        if (a > Math.PI)
        {
            a -= 2 * Math.PI;
        }
        else if (a <= -Math.PI)
        {
            a += 2 * Math.PI;
        }

        return a;
    }

    public static double ToRadians(double degrees) => degrees * Math.PI / 180.0;

    public static double ToDegrees(double radians) => radians * 180.0 / Math.PI;
}

/// <summary>An axis-aligned rectangle.</summary>
public readonly record struct Bounds2D(double MinX, double MinY, double MaxX, double MaxY)
{
    public double Width => MaxX - MinX;

    public double Height => MaxY - MinY;

    public bool Contains(Vec2 p) => p.X >= MinX && p.X <= MaxX && p.Y >= MinY && p.Y <= MaxY;

    public Bounds2D Union(Bounds2D o) => new(Math.Min(MinX, o.MinX), Math.Min(MinY, o.MinY), Math.Max(MaxX, o.MaxX), Math.Max(MaxY, o.MaxY));

    public Bounds2D Inflate(double m) => new(MinX - m, MinY - m, MaxX + m, MaxY + m);
}

/// <summary>An RGB color for rendering and the simulated camera.</summary>
public readonly record struct SimColor(byte R, byte G, byte B)
{
    public static SimColor Gray => new(128, 128, 128);

    public static SimColor Wall => new(90, 98, 112);

    public static SimColor Shelf => new(196, 140, 72);

    public static SimColor Floor => new(232, 234, 238);

    public static SimColor Crop => new(76, 160, 70);

    public static SimColor Robot => new(40, 120, 220);

    public static SimColor Crate => new(170, 110, 60);

    public static SimColor Parse(string hex)
    {
        var h = hex.TrimStart('#');
        return h.Length == 6
            ? new SimColor(Convert.ToByte(h[..2], 16), Convert.ToByte(h[2..4], 16), Convert.ToByte(h[4..6], 16))
            : Gray;
    }

    public override string ToString() => $"#{R:X2}{G:X2}{B:X2}";
}

/// <summary>
/// Deterministic, allocation-free random source (xoshiro256**) with Gaussian samples. Every noise
/// source in a world derives from the world seed, so runs are reproducible.
/// </summary>
public sealed class SimRandom
{
    private ulong _s0, _s1, _s2, _s3;
    private double _spare;
    private bool _hasSpare;

    public SimRandom(ulong seed) => Reseed(seed);

    public void Reseed(ulong seed)
    {
        // SplitMix64 to spread the seed.
        ulong Next(ref ulong x)
        {
            var z = x += 0x9E3779B97F4A7C15UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        var s = seed;
        _s0 = Next(ref s);
        _s1 = Next(ref s);
        _s2 = Next(ref s);
        _s3 = Next(ref s);
        _hasSpare = false;
    }

    public ulong NextUInt64()
    {
        var result = RotL(_s1 * 5, 7) * 9;
        var t = _s1 << 17;
        _s2 ^= _s0;
        _s3 ^= _s1;
        _s1 ^= _s2;
        _s0 ^= _s3;
        _s2 ^= t;
        _s3 = RotL(_s3, 45);
        return result;
    }

    /// <summary>Uniform in [0, 1).</summary>
    public double NextDouble() => (NextUInt64() >> 11) * (1.0 / (1UL << 53));

    public double Uniform(double min, double max) => min + ((max - min) * NextDouble());

    /// <summary>Standard normal sample (Marsaglia polar method).</summary>
    public double NextGaussian()
    {
        if (_hasSpare)
        {
            _hasSpare = false;
            return _spare;
        }

        double u, v, s;
        do
        {
            u = (2 * NextDouble()) - 1;
            v = (2 * NextDouble()) - 1;
            s = (u * u) + (v * v);
        }
        while (s >= 1 || s == 0);

        var m = Math.Sqrt(-2 * Math.Log(s) / s);
        _spare = v * m;
        _hasSpare = true;
        return u * m;
    }

    /// <summary>Normal sample with the given standard deviation (0 returns 0 without consuming randomness).</summary>
    public double Gaussian(double stdDev) => stdDev <= 0 ? 0 : NextGaussian() * stdDev;

    private static ulong RotL(ulong x, int k) => (x << k) | (x >> (64 - k));
}
