using System.Numerics;
using ROS2.Net.Native;
using ROS2.Net.Native.Managed;

namespace ROS2.Net.Perception;

/// <summary>Result of a PCA over a point set.</summary>
/// <param name="Mean">Centroid.</param>
/// <param name="EigenValues">Ascending eigenvalues of the covariance.</param>
/// <param name="EigenVectors">Unit eigenvectors matching <paramref name="EigenValues"/>.</param>
public sealed record PcaResult(Vector3 Mean, double[] EigenValues, Vector3[] EigenVectors);

/// <summary>Surface normal estimation by local PCA.</summary>
public static class Normals
{
    /// <summary>Principal component analysis of the points at <paramref name="indices"/> (all when null).</summary>
    public static PcaResult Pca(ReadOnlySpan<float> xyz, IReadOnlyList<int>? indices = null)
    {
        var count = indices?.Count ?? xyz.Length / 3;
        double mx = 0, my = 0, mz = 0;
        for (var k = 0; k < count; k++)
        {
            var i = indices?[k] ?? k;
            mx += xyz[3 * i];
            my += xyz[(3 * i) + 1];
            mz += xyz[(3 * i) + 2];
        }

        if (count == 0)
        {
            return new PcaResult(Vector3.Zero, [0, 0, 0], [Vector3.UnitX, Vector3.UnitY, Vector3.UnitZ]);
        }

        mx /= count;
        my /= count;
        mz /= count;
        var c = new double[9];
        for (var k = 0; k < count; k++)
        {
            var i = indices?[k] ?? k;
            double dx = xyz[3 * i] - mx, dy = xyz[(3 * i) + 1] - my, dz = xyz[(3 * i) + 2] - mz;
            c[0] += dx * dx;
            c[1] += dx * dy;
            c[2] += dx * dz;
            c[4] += dy * dy;
            c[5] += dy * dz;
            c[8] += dz * dz;
        }

        c[3] = c[1];
        c[6] = c[2];
        c[7] = c[5];
        for (var j = 0; j < 9; j++)
        {
            c[j] /= count;
        }

        var (values, vectors) = ManagedKernels.JacobiEigen(c, 3);
        var order = new[] { 0, 1, 2 }.OrderBy(j => values[j]).ToArray();
        return new PcaResult(
            new Vector3((float)mx, (float)my, (float)mz),
            order.Select(j => values[j]).ToArray(),
            order.Select(j => Vector3.Normalize(new Vector3((float)vectors[j], (float)vectors[3 + j], (float)vectors[6 + j]))).ToArray());
    }

    /// <summary>
    /// Estimates a unit normal per point from its <paramref name="k"/> nearest neighbours, oriented
    /// towards <paramref name="viewpoint"/> (the sensor origin by default). Also returns surface
    /// curvature λ₀/(λ₀+λ₁+λ₂). The normals are stored on the cloud.
    /// </summary>
    public static float[] Estimate(PointCloud cloud, int k = 16, Vector3? viewpoint = null, float[]? curvature = null)
    {
        var xyz = cloud.Xyz.ToArray();
        using var tree = new KdTree(xyz);
        var normals = new float[cloud.Count * 3];
        var vp = viewpoint ?? Vector3.Zero;
        for (var i = 0; i < cloud.Count; i++)
        {
            var p = new Vector3(xyz[3 * i], xyz[(3 * i) + 1], xyz[(3 * i) + 2]);
            if (!float.IsFinite(p.X) || !float.IsFinite(p.Y) || !float.IsFinite(p.Z))
            {
                normals[3 * i] = normals[(3 * i) + 1] = normals[(3 * i) + 2] = float.NaN;
                continue;
            }

            var nn = tree.Nearest(p.X, p.Y, p.Z, k);
            var pca = Pca(xyz, nn.Select(n => n.Index).ToArray());
            var n = pca.EigenVectors[0];
            if (Vector3.Dot(n, vp - p) < 0)
            {
                n = -n;
            }

            normals[3 * i] = n.X;
            normals[(3 * i) + 1] = n.Y;
            normals[(3 * i) + 2] = n.Z;
            if (curvature is not null)
            {
                var sum = pca.EigenValues.Sum();
                curvature[i] = sum > 0 ? (float)(pca.EigenValues[0] / sum) : 0;
            }
        }

        cloud.SetNormals(normals);
        return normals;
    }
}

/// <summary>A plane found by <see cref="Segmentation.SegmentPlane"/>.</summary>
public sealed record PlaneSegmentation(PlaneFit Plane, int[] Inliers, int[] Outliers)
{
    public Vector3 Normal => new((float)Plane.A, (float)Plane.B, (float)Plane.C);

    /// <summary>Signed distance of a point to the plane.</summary>
    public double Distance(Vector3 p) => (Plane.A * p.X) + (Plane.B * p.Y) + (Plane.C * p.Z) + Plane.D;
}

/// <summary>Model fitting and clustering.</summary>
public static class Segmentation
{
    /// <summary>RANSAC plane segmentation (e.g. ground removal). Reproducible for a given <paramref name="seed"/>.</summary>
    public static PlaneSegmentation SegmentPlane(PointCloud cloud, double distanceThreshold = 0.02, int iterations = 500, ulong seed = 1)
    {
        var mask = new byte[cloud.Count];
        var fit = PointKernels.RansacPlane(cloud.Xyz, distanceThreshold, iterations, seed, mask);
        var inliers = new List<int>(fit.InlierCount);
        var outliers = new List<int>(cloud.Count - fit.InlierCount);
        for (var i = 0; i < mask.Length; i++)
        {
            (mask[i] != 0 ? inliers : outliers).Add(i);
        }

        return new PlaneSegmentation(fit, inliers.ToArray(), outliers.ToArray());
    }

    /// <summary>Euclidean cluster extraction. Clusters are ordered by their lowest point index.</summary>
    public static IReadOnlyList<int[]> EuclideanClusters(PointCloud cloud, float tolerance, int minSize = 10, int maxSize = int.MaxValue)
    {
        var labels = new int[cloud.Count];
        var count = PointKernels.EuclideanCluster(cloud.Xyz, tolerance, minSize, maxSize, labels);
        var clusters = new List<int>[count];
        for (var c = 0; c < count; c++)
        {
            clusters[c] = [];
        }

        for (var i = 0; i < labels.Length; i++)
        {
            if (labels[i] >= 0)
            {
                clusters[labels[i]].Add(i);
            }
        }

        return clusters.Select(c => c.ToArray()).ToList();
    }
}

/// <summary>An axis-aligned bounding box.</summary>
public readonly record struct Aabb(Vector3 Min, Vector3 Max)
{
    public Vector3 Center => (Min + Max) * 0.5f;

    public Vector3 Size => Max - Min;
}

/// <summary>An oriented bounding box: <see cref="Axes"/> are unit columns of the box frame.</summary>
public sealed record Obb(Vector3 Center, Vector3[] Axes, Vector3 HalfExtents)
{
    /// <summary>Box orientation as a quaternion.</summary>
    public Quaternion Orientation
    {
        get
        {
            var m = new Matrix4x4(
                Axes[0].X, Axes[0].Y, Axes[0].Z, 0,
                Axes[1].X, Axes[1].Y, Axes[1].Z, 0,
                Axes[2].X, Axes[2].Y, Axes[2].Z, 0,
                0, 0, 0, 1);
            return Quaternion.Normalize(Quaternion.CreateFromRotationMatrix(m));
        }
    }
}

/// <summary>Bounding boxes of point sets.</summary>
public static class BoundingBoxes
{
    public static Aabb ComputeAabb(ReadOnlySpan<float> xyz, IReadOnlyList<int>? indices = null)
    {
        var min = new Vector3(float.PositiveInfinity);
        var max = new Vector3(float.NegativeInfinity);
        var count = indices?.Count ?? xyz.Length / 3;
        for (var k = 0; k < count; k++)
        {
            var i = indices?[k] ?? k;
            var p = new Vector3(xyz[3 * i], xyz[(3 * i) + 1], xyz[(3 * i) + 2]);
            if (float.IsFinite(p.X) && float.IsFinite(p.Y) && float.IsFinite(p.Z))
            {
                min = Vector3.Min(min, p);
                max = Vector3.Max(max, p);
            }
        }

        return new Aabb(min, max);
    }

    public static Aabb ComputeAabb(PointCloud cloud, IReadOnlyList<int>? indices = null) => ComputeAabb(cloud.Xyz, indices);

    /// <summary>PCA-aligned oriented box.</summary>
    public static Obb ComputeObb(PointCloud cloud, IReadOnlyList<int>? indices = null)
    {
        var xyz = cloud.Xyz;
        var pca = Normals.Pca(xyz, indices);
        // Largest variance first, right-handed.
        var a0 = pca.EigenVectors[2];
        var a1 = pca.EigenVectors[1];
        var a2 = Vector3.Normalize(Vector3.Cross(a0, a1));
        var min = new Vector3(float.PositiveInfinity);
        var max = new Vector3(float.NegativeInfinity);
        var count = indices?.Count ?? cloud.Count;
        for (var k = 0; k < count; k++)
        {
            var i = indices?[k] ?? k;
            var d = new Vector3(xyz[3 * i], xyz[(3 * i) + 1], xyz[(3 * i) + 2]) - pca.Mean;
            var local = new Vector3(Vector3.Dot(d, a0), Vector3.Dot(d, a1), Vector3.Dot(d, a2));
            min = Vector3.Min(min, local);
            max = Vector3.Max(max, local);
        }

        var mid = (min + max) * 0.5f;
        var center = pca.Mean + (a0 * mid.X) + (a1 * mid.Y) + (a2 * mid.Z);
        return new Obb(center, [a0, a1, a2], (max - min) * 0.5f);
    }
}
