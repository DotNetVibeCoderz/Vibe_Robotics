namespace ROS2.Net.Navigation.Localization;

/// <summary>
/// A planar extended Kalman filter in the style of <c>robot_localization</c>'s ekf_node: state
/// (x, y, θ, v, ω) with a constant-velocity unicycle model, fusing any subset of pose, heading,
/// forward velocity and yaw rate measurements (wheel odometry, IMU gyro, compass, GPS…).
/// </summary>
public sealed class Ekf2D
{
    public const int X = 0;
    public const int Y = 1;
    public const int Theta = 2;
    public const int V = 3;
    public const int W = 4;
    private const int N = 5;

    private readonly double[] _s = new double[N];
    private readonly double[] _p = new double[N * N];
    private readonly double[] _q;
    private double _lastTime = double.NaN;

    /// <param name="processNoise">Per-state process noise spectral densities (x, y, θ, v, ω), per second.</param>
    public Ekf2D(double[]? processNoise = null)
    {
        _q = processNoise ?? [0.05, 0.05, 0.06, 0.5, 0.5];
        for (var i = 0; i < N; i++)
        {
            _p[(i * N) + i] = 1e-3;
        }
    }

    public Pose2D Pose => new(_s[X], _s[Y], _s[Theta]);

    public double LinearVelocity => _s[V];

    public double AngularVelocity => _s[W];

    /// <summary>State covariance, row-major 5x5.</summary>
    public ReadOnlySpan<double> Covariance => _p;

    /// <summary>Time of the last prediction, in seconds.</summary>
    public double LastTime => _lastTime;

    public void Reset(Pose2D pose, double sigmaXy = 0.01, double sigmaTheta = 0.01)
    {
        Array.Clear(_s);
        Array.Clear(_p);
        _s[X] = pose.X;
        _s[Y] = pose.Y;
        _s[Theta] = pose.Theta;
        _p[(X * N) + X] = sigmaXy * sigmaXy;
        _p[(Y * N) + Y] = sigmaXy * sigmaXy;
        _p[(Theta * N) + Theta] = sigmaTheta * sigmaTheta;
        _p[(V * N) + V] = 1;
        _p[(W * N) + W] = 1;
        _lastTime = double.NaN;
    }

    /// <summary>Propagates the state to <paramref name="time"/> (seconds).</summary>
    public void PredictTo(double time)
    {
        if (double.IsNaN(_lastTime))
        {
            _lastTime = time;
            return;
        }

        var dt = time - _lastTime;
        if (dt <= 0)
        {
            return;
        }

        _lastTime = time;
        var th = _s[Theta];
        var v = _s[V];
        var c = Math.Cos(th);
        var s = Math.Sin(th);
        _s[X] += v * c * dt;
        _s[Y] += v * s * dt;
        _s[Theta] = Angles.Normalize(th + (_s[W] * dt));

        // F = I + ∂f/∂s · dt
        Span<double> f = stackalloc double[N * N];
        for (var i = 0; i < N; i++)
        {
            f[(i * N) + i] = 1;
        }

        f[(X * N) + Theta] = -v * s * dt;
        f[(X * N) + V] = c * dt;
        f[(Y * N) + Theta] = v * c * dt;
        f[(Y * N) + V] = s * dt;
        f[(Theta * N) + W] = dt;

        // P = F P Fᵀ + Q dt
        Span<double> fp = stackalloc double[N * N];
        Mul(f, _p, fp);
        for (var i = 0; i < N; i++)
        {
            for (var j = 0; j < N; j++)
            {
                var sum = 0.0;
                for (var k = 0; k < N; k++)
                {
                    sum += fp[(i * N) + k] * f[(j * N) + k];
                }

                _p[(i * N) + j] = sum + (i == j ? _q[i] * dt : 0);
            }
        }
    }

    /// <summary>Fuses a measurement of selected state components with diagonal noise variances.</summary>
    public void Update(ReadOnlySpan<int> indices, ReadOnlySpan<double> values, ReadOnlySpan<double> variances)
    {
        var m = indices.Length;

        // Innovation y and S = H P Hᵀ + R, where H selects state rows.
        Span<double> y = stackalloc double[m];
        Span<double> sMat = stackalloc double[m * m];
        for (var a = 0; a < m; a++)
        {
            var ia = indices[a];
            y[a] = ia == Theta ? Angles.Difference(values[a], _s[Theta]) : values[a] - _s[ia];
            for (var b = 0; b < m; b++)
            {
                sMat[(a * m) + b] = _p[(ia * N) + indices[b]] + (a == b ? variances[a] : 0);
            }
        }

        Span<double> sInv = stackalloc double[m * m];
        if (!Invert(sMat, sInv, m))
        {
            return;
        }

        // K = P Hᵀ S⁻¹  (N x m)
        Span<double> k = stackalloc double[N * m];
        for (var i = 0; i < N; i++)
        {
            for (var b = 0; b < m; b++)
            {
                var sum = 0.0;
                for (var a = 0; a < m; a++)
                {
                    sum += _p[(i * N) + indices[a]] * sInv[(a * m) + b];
                }

                k[(i * m) + b] = sum;
            }
        }

        for (var i = 0; i < N; i++)
        {
            var dx = 0.0;
            for (var a = 0; a < m; a++)
            {
                dx += k[(i * m) + a] * y[a];
            }

            _s[i] += dx;
        }

        _s[Theta] = Angles.Normalize(_s[Theta]);

        // P = (I - K H) P
        Span<double> newP = stackalloc double[N * N];
        for (var i = 0; i < N; i++)
        {
            for (var j = 0; j < N; j++)
            {
                var kh = 0.0;
                for (var a = 0; a < m; a++)
                {
                    kh += k[(i * m) + a] * _p[(indices[a] * N) + j];
                }

                newP[(i * N) + j] = _p[(i * N) + j] - kh;
            }
        }

        // Symmetrize to keep numerical drift out.
        for (var i = 0; i < N; i++)
        {
            for (var j = 0; j < N; j++)
            {
                _p[(i * N) + j] = 0.5 * (newP[(i * N) + j] + newP[(j * N) + i]);
            }
        }
    }

    /// <summary>Fuses wheel odometry twist (forward speed, yaw rate).</summary>
    public void UpdateTwist(double time, double linear, double angular, double linearVariance = 0.01, double angularVariance = 0.02)
    {
        PredictTo(time);
        Update([V, W], [linear, angular], [linearVariance, angularVariance]);
    }

    /// <summary>Fuses a gyro yaw rate (IMU <c>angular_velocity.z</c>).</summary>
    public void UpdateYawRate(double time, double yawRate, double variance = 0.001)
    {
        PredictTo(time);
        Update([W], [yawRate], [variance]);
    }

    /// <summary>Fuses an absolute heading (IMU orientation, compass).</summary>
    public void UpdateYaw(double time, double yaw, double variance = 0.01)
    {
        PredictTo(time);
        Update([Theta], [yaw], [variance]);
    }

    /// <summary>Fuses an absolute position (GPS, AMCL, motion capture).</summary>
    public void UpdatePosition(double time, double x, double y, double variance = 0.05)
    {
        PredictTo(time);
        Update([X, Y], [x, y], [variance, variance]);
    }

    private static void Mul(ReadOnlySpan<double> a, ReadOnlySpan<double> b, Span<double> result)
    {
        for (var i = 0; i < N; i++)
        {
            for (var j = 0; j < N; j++)
            {
                var sum = 0.0;
                for (var k = 0; k < N; k++)
                {
                    sum += a[(i * N) + k] * b[(k * N) + j];
                }

                result[(i * N) + j] = sum;
            }
        }
    }

    /// <summary>Gauss-Jordan inversion of a small dense matrix.</summary>
    internal static bool Invert(ReadOnlySpan<double> source, Span<double> inverse, int n)
    {
        Span<double> a = stackalloc double[n * n];
        source.CopyTo(a);
        inverse.Clear();
        for (var i = 0; i < n; i++)
        {
            inverse[(i * n) + i] = 1;
        }

        for (var col = 0; col < n; col++)
        {
            var pivot = col;
            for (var r = col + 1; r < n; r++)
            {
                if (Math.Abs(a[(r * n) + col]) > Math.Abs(a[(pivot * n) + col]))
                {
                    pivot = r;
                }
            }

            if (Math.Abs(a[(pivot * n) + col]) < 1e-12)
            {
                return false;
            }

            if (pivot != col)
            {
                for (var c = 0; c < n; c++)
                {
                    (a[(col * n) + c], a[(pivot * n) + c]) = (a[(pivot * n) + c], a[(col * n) + c]);
                    (inverse[(col * n) + c], inverse[(pivot * n) + c]) = (inverse[(pivot * n) + c], inverse[(col * n) + c]);
                }
            }

            var d = a[(col * n) + col];
            for (var c = 0; c < n; c++)
            {
                a[(col * n) + c] /= d;
                inverse[(col * n) + c] /= d;
            }

            for (var r = 0; r < n; r++)
            {
                if (r == col)
                {
                    continue;
                }

                var f = a[(r * n) + col];
                if (f == 0)
                {
                    continue;
                }

                for (var c = 0; c < n; c++)
                {
                    a[(r * n) + c] -= f * a[(col * n) + c];
                    inverse[(r * n) + c] -= f * inverse[(col * n) + c];
                }
            }
        }

        return true;
    }
}
