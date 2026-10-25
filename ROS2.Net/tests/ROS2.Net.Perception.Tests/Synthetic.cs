using ROS2.Net.Perception;

namespace ROS2.Net.Perception.Tests;

/// <summary>Deterministic synthetic clouds.</summary>
internal static class Synthetic
{
    public static float[] Random(int n, int seed, float scale = 10f)
    {
        var rng = new Random(seed);
        var xyz = new float[n * 3];
        for (var i = 0; i < xyz.Length; i++)
        {
            xyz[i] = (float)(rng.NextDouble() * scale);
        }

        return xyz;
    }

    /// <summary>A z = height grid plane with optional noise.</summary>
    public static PointCloud Plane(int side, float spacing, float height, int seed = 1, float noise = 0)
    {
        var rng = new Random(seed);
        var cloud = new PointCloud(side * side);
        for (var i = 0; i < side; i++)
        {
            for (var j = 0; j < side; j++)
            {
                cloud.Add(i * spacing, j * spacing, height + (float)((rng.NextDouble() - 0.5) * noise));
            }
        }

        return cloud;
    }

    /// <summary>Gaussian blob of points around a center.</summary>
    public static void Blob(PointCloud cloud, float cx, float cy, float cz, int n, float sigma, int seed)
    {
        var rng = new Random(seed);
        for (var i = 0; i < n; i++)
        {
            cloud.Add(cx + (float)(Gauss(rng) * sigma), cy + (float)(Gauss(rng) * sigma), cz + (float)(Gauss(rng) * sigma));
        }
    }

    private static double Gauss(Random r) => Math.Sqrt(-2 * Math.Log(1 - r.NextDouble())) * Math.Cos(2 * Math.PI * r.NextDouble());

    /// <summary>A wavy surface, good for ICP.</summary>
    public static PointCloud Surface(int side = 20, float spacing = 0.1f)
    {
        var cloud = new PointCloud(side * side);
        for (var i = 0; i < side; i++)
        {
            for (var j = 0; j < side; j++)
            {
                float x = i * spacing, y = j * spacing;
                cloud.Add(x, y, (MathF.Sin(x * 3) * 0.2f) + (MathF.Cos(y * 2) * 0.15f));
            }
        }

        return cloud;
    }
}
