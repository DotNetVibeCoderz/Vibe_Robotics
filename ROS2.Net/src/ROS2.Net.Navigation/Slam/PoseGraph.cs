namespace ROS2.Net.Navigation.Slam;

/// <summary>A relative-pose constraint between two graph nodes.</summary>
/// <param name="From">Index of the first node.</param>
/// <param name="To">Index of the second node.</param>
/// <param name="Measurement">Pose of <paramref name="To"/> in the frame of <paramref name="From"/>.</param>
/// <param name="InfoXy">Information (1/σ²) of the translation.</param>
/// <param name="InfoTheta">Information (1/σ²) of the rotation.</param>
/// <param name="IsLoopClosure">True for loop-closure constraints.</param>
public sealed record PoseGraphEdge(int From, int To, Pose2D Measurement, double InfoXy, double InfoTheta, bool IsLoopClosure);

/// <summary>
/// A 2-D pose graph optimized with Levenberg-Marquardt (Gauss-Newton with damping), following
/// Grisetti et al., "A Tutorial on Graph-Based SLAM". The first node is held fixed.
/// </summary>
public sealed class PoseGraph
{
    private readonly List<Pose2D> _nodes = [];
    private readonly List<PoseGraphEdge> _edges = [];

    public IReadOnlyList<Pose2D> Nodes => _nodes;

    public IReadOnlyList<PoseGraphEdge> Edges => _edges;

    public int AddNode(Pose2D pose)
    {
        _nodes.Add(pose);
        return _nodes.Count - 1;
    }

    public void SetNode(int index, Pose2D pose) => _nodes[index] = pose;

    public void AddEdge(PoseGraphEdge edge) => _edges.Add(edge);

    /// <summary>Total weighted squared error χ².</summary>
    public double Chi2()
    {
        var chi = 0.0;
        foreach (var e in _edges)
        {
            var (ex, ey, et) = Error(e);
            chi += (e.InfoXy * ((ex * ex) + (ey * ey))) + (e.InfoTheta * et * et);
        }

        return chi;
    }

    /// <summary>Optimizes node poses; returns the final χ².</summary>
    public double Optimize(int maxIterations = 20, double tolerance = 1e-6)
    {
        var n = _nodes.Count;
        if (n < 2 || _edges.Count == 0)
        {
            return Chi2();
        }

        var dim = 3 * n;
        var h = new double[dim * dim];
        var b = new double[dim];
        var lambda = 1e-4;
        var chi = Chi2();
        for (var iter = 0; iter < maxIterations; iter++)
        {
            Array.Clear(h);
            Array.Clear(b);
            foreach (var e in _edges)
            {
                Linearize(e, h, b, dim);
            }

            // Anchor the first node.
            for (var k = 0; k < 3; k++)
            {
                h[(k * dim) + k] += 1e9;
            }

            var saved = _nodes.ToArray();
            var accepted = false;
            for (var attempt = 0; attempt < 8; attempt++)
            {
                var damped = (double[])h.Clone();
                for (var k = 0; k < dim; k++)
                {
                    damped[(k * dim) + k] += lambda * (1 + damped[(k * dim) + k]);
                }

                var rhs = new double[dim];
                for (var k = 0; k < dim; k++)
                {
                    rhs[k] = -b[k];
                }

                if (!CholeskySolve(damped, rhs, dim))
                {
                    lambda *= 10;
                    continue;
                }

                for (var i = 0; i < n; i++)
                {
                    var p = saved[i];
                    _nodes[i] = new Pose2D(p.X + rhs[3 * i], p.Y + rhs[(3 * i) + 1], Angles.Normalize(p.Theta + rhs[(3 * i) + 2]));
                }

                var newChi = Chi2();
                if (newChi < chi)
                {
                    accepted = true;
                    var improvement = chi - newChi;
                    chi = newChi;
                    lambda = Math.Max(1e-9, lambda / 10);
                    if (improvement < tolerance * Math.Max(1, chi))
                    {
                        return chi;
                    }

                    break;
                }

                for (var i = 0; i < n; i++)
                {
                    _nodes[i] = saved[i];
                }

                lambda *= 10;
            }

            if (!accepted)
            {
                break;
            }
        }

        return chi;
    }

    private (double X, double Y, double Theta) Error(PoseGraphEdge e)
    {
        var predicted = _nodes[e.From].Between(_nodes[e.To]);
        var err = e.Measurement.Between(predicted);
        return (err.X, err.Y, err.Theta);
    }

    private void Linearize(PoseGraphEdge e, double[] h, double[] b, int dim)
    {
        var xi = _nodes[e.From];
        var xj = _nodes[e.To];
        var z = e.Measurement;
        double ci = Math.Cos(xi.Theta), si = Math.Sin(xi.Theta);
        double cz = Math.Cos(z.Theta), sz = Math.Sin(z.Theta);
        var dx = xj.X - xi.X;
        var dy = xj.Y - xi.Y;

        // Rᵢᵀ·(tⱼ − tᵢ)
        var lx = (ci * dx) + (si * dy);
        var ly = (-si * dx) + (ci * dy);

        // Error in the measurement frame: e = Rzᵀ(Rᵢᵀ(tⱼ−tᵢ) − tz), θⱼ − θᵢ − θz.
        var ex = (cz * (lx - z.X)) + (sz * (ly - z.Y));
        var ey = (-sz * (lx - z.X)) + (cz * (ly - z.Y));
        var et = Angles.Normalize(xj.Theta - xi.Theta - z.Theta);

        // dRᵢᵀ/dθᵢ · (tⱼ − tᵢ)
        var dlx = (-si * dx) + (ci * dy);
        var dly = (-ci * dx) - (si * dy);

        // RzᵀRᵢᵀ rows.
        double m00 = (cz * ci) - (sz * si), m01 = (cz * si) + (sz * ci);
        double m10 = (-sz * ci) - (cz * si), m11 = (-sz * si) + (cz * ci);

        Span<double> a = [
            -m00, -m01, (cz * dlx) + (sz * dly),
            -m10, -m11, (-sz * dlx) + (cz * dly),
            0, 0, -1,
        ];
        Span<double> bj = [
            m00, m01, 0,
            m10, m11, 0,
            0, 0, 1,
        ];
        Span<double> info = [e.InfoXy, e.InfoXy, e.InfoTheta];
        Span<double> err = [ex, ey, et];

        var i0 = 3 * e.From;
        var j0 = 3 * e.To;
        for (var r = 0; r < 3; r++)
        {
            for (var c = 0; c < 3; c++)
            {
                double haa = 0, hab = 0, hbb = 0;
                for (var k = 0; k < 3; k++)
                {
                    haa += a[(k * 3) + r] * info[k] * a[(k * 3) + c];
                    hab += a[(k * 3) + r] * info[k] * bj[(k * 3) + c];
                    hbb += bj[(k * 3) + r] * info[k] * bj[(k * 3) + c];
                }

                h[((i0 + r) * dim) + i0 + c] += haa;
                h[((i0 + r) * dim) + j0 + c] += hab;
                h[((j0 + c) * dim) + i0 + r] += hab;
                h[((j0 + r) * dim) + j0 + c] += hbb;
            }

            double ba = 0, bb = 0;
            for (var k = 0; k < 3; k++)
            {
                ba += a[(k * 3) + r] * info[k] * err[k];
                bb += bj[(k * 3) + r] * info[k] * err[k];
            }

            b[i0 + r] += ba;
            b[j0 + r] += bb;
        }
    }

    /// <summary>Solves A·x = b in place (b becomes x) for symmetric positive-definite A.</summary>
    internal static bool CholeskySolve(double[] a, double[] b, int n)
    {
        for (var j = 0; j < n; j++)
        {
            var sum = a[(j * n) + j];
            for (var k = 0; k < j; k++)
            {
                sum -= a[(j * n) + k] * a[(j * n) + k];
            }

            if (sum <= 0)
            {
                return false;
            }

            var d = Math.Sqrt(sum);
            a[(j * n) + j] = d;
            for (var i = j + 1; i < n; i++)
            {
                var s = a[(i * n) + j];
                for (var k = 0; k < j; k++)
                {
                    s -= a[(i * n) + k] * a[(j * n) + k];
                }

                a[(i * n) + j] = s / d;
            }
        }

        for (var i = 0; i < n; i++)
        {
            var s = b[i];
            for (var k = 0; k < i; k++)
            {
                s -= a[(i * n) + k] * b[k];
            }

            b[i] = s / a[(i * n) + i];
        }

        for (var i = n - 1; i >= 0; i--)
        {
            var s = b[i];
            for (var k = i + 1; k < n; k++)
            {
                s -= a[(k * n) + i] * b[k];
            }

            b[i] = s / a[(i * n) + i];
        }

        return true;
    }
}
