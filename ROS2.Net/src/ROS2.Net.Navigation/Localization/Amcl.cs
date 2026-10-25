using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Navigation.Maps;

namespace ROS2.Net.Navigation.Localization;

/// <summary>Laser sensor models.</summary>
public enum LaserModel
{
    /// <summary>Tempered product of per-beam likelihood-field probabilities (Thrun et al.); converges fast.</summary>
    LikelihoodFieldProduct,
    /// <summary>nav2_amcl's <c>likelihood_field</c> combination: <c>1 + Σ pz³</c>; conservative and slow to converge.</summary>
    LikelihoodField,
}

/// <summary>AMCL parameters (names follow <c>nav2_amcl</c>).</summary>
public sealed record AmclOptions
{
    public int MinParticles { get; init; } = 500;

    public int MaxParticles { get; init; } = 2000;

    /// <summary>Rotation noise from rotation.</summary>
    public double Alpha1 { get; init; } = 0.2;

    /// <summary>Rotation noise from translation.</summary>
    public double Alpha2 { get; init; } = 0.2;

    /// <summary>Translation noise from translation.</summary>
    public double Alpha3 { get; init; } = 0.2;

    /// <summary>Translation noise from rotation.</summary>
    public double Alpha4 { get; init; } = 0.2;

    public double ZHit { get; init; } = 0.5;

    public double ZRand { get; init; } = 0.5;

    public double SigmaHit { get; init; } = 0.2;

    public double LaserMaxRange { get; init; } = 12.0;

    public double LaserLikelihoodMaxDist { get; init; } = 2.0;

    public int MaxBeams { get; init; } = 60;

    /// <summary>How beam likelihoods are combined.</summary>
    public LaserModel LaserModel { get; init; } = LaserModel.LikelihoodFieldProduct;

    /// <summary>
    /// Exponent applied to the beam-likelihood product (<see cref="LaserModel.LikelihoodFieldProduct"/>), which
    /// tempers the over-confidence caused by treating correlated beams as independent.
    /// </summary>
    public double LikelihoodTemper { get; init; } = 0.33;

    /// <summary>Translation needed before a filter update.</summary>
    public double UpdateMinD { get; init; } = 0.25;

    /// <summary>Rotation needed before a filter update.</summary>
    public double UpdateMinA { get; init; } = 0.2;

    public int ResampleInterval { get; init; } = 1;

    /// <summary>KLD-sampling error bound and quantile.</summary>
    public double PfErr { get; init; } = 0.05;

    public double PfZ { get; init; } = 0.99;

    /// <summary>Recovery: exponential decay rates of the slow/fast average likelihood (0 disables).</summary>
    public double RecoveryAlphaSlow { get; init; }

    public double RecoveryAlphaFast { get; init; }

    public int? Seed { get; init; }
}

/// <summary>
/// Adaptive Monte Carlo Localization: a particle filter with the differential-drive odometry motion
/// model, the likelihood-field laser model, KLD-adaptive resampling and optional random-particle recovery.
/// </summary>
public sealed class Amcl
{
    private readonly AmclOptions _options;
    private readonly GaussianRandom _random;
    private LikelihoodField _field;
    private double[] _x = [];
    private double[] _y = [];
    private double[] _t = [];
    private double[] _w = [];
    private int _count;
    private Pose2D? _lastOdom;
    private int _updates;
    private double[] _logp = [];
    private double _wSlow;
    private double _wFast;

    public Amcl(OccupancyGridMap map, AmclOptions? options = null)
    {
        _options = options ?? new AmclOptions();
        _random = new GaussianRandom(_options.Seed);
        _field = new LikelihoodField(map, _options.LaserLikelihoodMaxDist);
    }

    public OccupancyGridMap Map => _field.Map;

    public int ParticleCount => _count;

    /// <summary>Weighted-mean estimate of the robot pose in the map frame.</summary>
    public Pose2D Estimate { get; private set; }

    /// <summary>Row-major 3x3 covariance (x, y, θ) of the estimate.</summary>
    public double[] Covariance { get; } = new double[9];

    public bool IsInitialized => _count > 0;

    /// <summary>Raised after each filter update.</summary>
    public event Action<Amcl>? Updated;

    /// <summary>Replaces the map (e.g. new <c>/map</c>).</summary>
    public void SetMap(OccupancyGridMap map) => _field = new LikelihoodField(map, _options.LaserLikelihoodMaxDist);

    /// <summary>Reads a particle (for <c>/particle_cloud</c>).</summary>
    public Pose2D GetParticle(int i) => new(_x[i], _y[i], _t[i]);

    /// <summary>Initializes particles around <paramref name="pose"/> with standard deviations.</summary>
    public void Initialize(Pose2D pose, double sigmaXy = 0.5, double sigmaTheta = 0.26)
    {
        Allocate(_options.MaxParticles);
        _count = _options.MinParticles > 0 ? Math.Max(_options.MinParticles, 1) : _options.MaxParticles;
        for (var i = 0; i < _count; i++)
        {
            _x[i] = pose.X + _random.Next(sigmaXy);
            _y[i] = pose.Y + _random.Next(sigmaXy);
            _t[i] = Angles.Normalize(pose.Theta + _random.Next(sigmaTheta));
            _w[i] = 1.0 / _count;
        }

        Estimate = pose;
        ComputeEstimate();
    }

    /// <summary>Spreads particles uniformly over free space (global localization).</summary>
    public void InitializeGlobal()
    {
        Allocate(_options.MaxParticles);
        _count = _options.MaxParticles;
        var free = new List<int>();
        var map = Map;
        for (var i = 0; i < map.Data.Length; i++)
        {
            if (map.Data[i] >= 0 && map.Data[i] <= map.FreeThreshold)
            {
                free.Add(i);
            }
        }

        if (free.Count == 0)
        {
            throw new InvalidOperationException("The map has no free cells.");
        }

        for (var i = 0; i < _count; i++)
        {
            var cell = free[(int)(_random.NextDouble() * free.Count)];
            var (wx, wy) = map.CellToWorld(cell % map.Width, cell / map.Width);
            _x[i] = wx;
            _y[i] = wy;
            _t[i] = (_random.NextDouble() * 2 * Math.PI) - Math.PI;
            _w[i] = 1.0 / _count;
        }

        ComputeEstimate();
    }

    /// <summary>
    /// Feeds an odometry pose and a scan (taken at the same time). Returns true when the filter
    /// updated (the robot moved enough since the last update).
    /// </summary>
    /// <param name="odom">Robot pose in the odometry frame.</param>
    /// <param name="scan">Laser scan.</param>
    /// <param name="laserOffset">Laser pose in the robot frame.</param>
    /// <param name="force">Update even if the robot has not moved.</param>
    public bool Update(Pose2D odom, LaserScan scan, Pose2D laserOffset = default, bool force = false)
    {
        if (!IsInitialized)
        {
            return false;
        }

        if (_lastOdom is not { } last)
        {
            _lastOdom = odom;
            if (!force)
            {
                return false;
            }

            last = odom;
        }

        // Like nav2_amcl, odometry is accumulated and applied once per filter update, so motion noise
        // does not depend on how often odometry arrives.
        var delta = last.Between(odom);
        var moved = Math.Sqrt((delta.X * delta.X) + (delta.Y * delta.Y)) >= _options.UpdateMinD ||
                    Math.Abs(delta.Theta) >= _options.UpdateMinA;
        if (!moved && !force)
        {
            return false;
        }

        MotionUpdate(last, odom);
        _lastOdom = odom;
        SensorUpdate(scan, laserOffset);
        if (++_updates % Math.Max(1, _options.ResampleInterval) == 0)
        {
            Resample();
        }

        ComputeEstimate();
        Updated?.Invoke(this);
        return true;
    }

    /// <summary>Applies the odometry motion model between two odometry poses (Thrun et al., Table 5.6).</summary>
    public void MotionUpdate(Pose2D from, Pose2D to)
    {
        var dx = to.X - from.X;
        var dy = to.Y - from.Y;
        var trans = Math.Sqrt((dx * dx) + (dy * dy));
        var rot1 = trans < 0.01 ? 0.0 : Angles.Difference(Math.Atan2(dy, dx), from.Theta);
        var rot2 = Angles.Difference(Angles.Difference(to.Theta, from.Theta), rot1);
        if (trans == 0 && rot2 == 0)
        {
            return;
        }

        // Treat backwards motion as forward with flipped rotations, as nav2_amcl does.
        var r1n = Math.Min(Math.Abs(rot1), Math.Abs(Angles.Normalize(rot1 - Math.PI)));
        var r2n = Math.Min(Math.Abs(rot2), Math.Abs(Angles.Normalize(rot2 - Math.PI)));
        var o = _options;
        var sr1 = Math.Sqrt((o.Alpha1 * r1n * r1n) + (o.Alpha2 * trans * trans));
        var st = Math.Sqrt((o.Alpha3 * trans * trans) + (o.Alpha4 * ((r1n * r1n) + (r2n * r2n))));
        var sr2 = Math.Sqrt((o.Alpha1 * r2n * r2n) + (o.Alpha2 * trans * trans));
        for (var i = 0; i < _count; i++)
        {
            var hr1 = rot1 - _random.Next(sr1);
            var ht = trans - _random.Next(st);
            var hr2 = rot2 - _random.Next(sr2);
            var th = _t[i] + hr1;
            _x[i] += ht * Math.Cos(th);
            _y[i] += ht * Math.Sin(th);
            _t[i] = Angles.Normalize(th + hr2);
        }
    }

    /// <summary>Weights particles with the likelihood-field model.</summary>
    public void SensorUpdate(LaserScan scan, Pose2D laserOffset = default)
    {
        var o = _options;
        var n = scan.Ranges.Length;
        if (n == 0)
        {
            return;
        }

        var step = Math.Max(1, n / Math.Max(1, o.MaxBeams));
        var rangeMax = Math.Min(o.LaserMaxRange, scan.RangeMax);
        var zRandMult = o.ZRand / rangeMax;
        var denom = 2 * o.SigmaHit * o.SigmaHit;
        var product = o.LaserModel == LaserModel.LikelihoodFieldProduct;
        if (_logp.Length < _count)
        {
            _logp = new double[_w.Length];
        }

        var maxLog = double.MinValue;
        for (var i = 0; i < _count; i++)
        {
            var robot = new Pose2D(_x[i], _y[i], _t[i]);
            var laser = robot.Compose(laserOffset);
            double p = product ? 0.0 : 1.0;
            for (var b = 0; b < n; b += step)
            {
                var r = scan.Ranges[b];
                if (!float.IsFinite(r) || r >= rangeMax || r < scan.RangeMin)
                {
                    continue;
                }

                var a = laser.Theta + scan.AngleMin + (b * scan.AngleIncrement);
                var hx = laser.X + (r * Math.Cos(a));
                var hy = laser.Y + (r * Math.Sin(a));
                var z = _field.DistanceAt(hx, hy);
                var pz = (o.ZHit * Math.Exp(-(z * z) / denom)) + zRandMult;

                // Product model: Σ log pz (tempered below). nav2's empirical model: 1 + Σ pz³.
                p += product ? Math.Log(pz) : pz * pz * pz;
            }

            _logp[i] = product ? p * o.LikelihoodTemper : Math.Log(p);
            maxLog = Math.Max(maxLog, _logp[i]);
        }

        var total = 0.0;
        for (var i = 0; i < _count; i++)
        {
            _w[i] *= Math.Exp(_logp[i] - maxLog);
            total += _w[i];
        }

        if (total <= 0 || !double.IsFinite(total))
        {
            for (var i = 0; i < _count; i++)
            {
                _w[i] = 1.0 / _count;
            }

            return;
        }

        // Average absolute likelihood (weights were normalized before this update), for random-particle recovery.
        var avg = Math.Exp(Math.Clamp(maxLog, -700, 700)) * total;
        if (o.RecoveryAlphaSlow > 0)
        {
            _wSlow = _wSlow == 0 ? avg : _wSlow + (o.RecoveryAlphaSlow * (avg - _wSlow));
            _wFast = _wFast == 0 ? avg : _wFast + (o.RecoveryAlphaFast * (avg - _wFast));
        }

        for (var i = 0; i < _count; i++)
        {
            _w[i] /= total;
        }
    }

    /// <summary>Resampling with a KLD-adaptive particle count (Fox, 2003).</summary>
    public void Resample()
    {
        var o = _options;
        var nx = new double[o.MaxParticles];
        var ny = new double[o.MaxParticles];
        var nt = new double[o.MaxParticles];
        var bins = new HashSet<(int, int, int)>();
        var injectProbability = _wSlow > 0 ? Math.Max(0, 1 - (_wFast / _wSlow)) : 0;

        // Cumulative weights for low-variance sampling.
        var cumulative = new double[_count];
        var acc = 0.0;
        for (var i = 0; i < _count; i++)
        {
            acc += _w[i];
            cumulative[i] = acc;
        }

        var target = o.MaxParticles;
        var m = 0;
        while (m < target && m < o.MaxParticles)
        {
            if (injectProbability > 0 && _random.NextDouble() < injectProbability)
            {
                var c = RandomFreePose();
                (nx[m], ny[m], nt[m]) = (c.X, c.Y, c.Theta);
            }
            else
            {
                // KLD sampling stops at a data-dependent count, so draw independently (binary search on the CDF)
                // rather than with a fixed systematic stride, which would bias toward the first particles.
                var u = _random.NextDouble() * acc;
                var idx = Array.BinarySearch(cumulative, 0, _count, u);
                idx = idx < 0 ? Math.Min(~idx, _count - 1) : idx;
                (nx[m], ny[m], nt[m]) = (_x[idx], _y[idx], _t[idx]);
            }

            // KLD bin occupancy decides how many particles are enough.
            if (bins.Add(((int)Math.Floor(nx[m] / 0.5), (int)Math.Floor(ny[m] / 0.5), (int)Math.Floor(nt[m] / (10 * Math.PI / 180)))))
            {
                target = Math.Clamp(KldLimit(bins.Count), o.MinParticles, o.MaxParticles);
            }

            m++;
        }

        _count = m;
        Array.Copy(nx, _x, m);
        Array.Copy(ny, _y, m);
        Array.Copy(nt, _t, m);
        for (var i = 0; i < m; i++)
        {
            _w[i] = 1.0 / m;
        }

        if (injectProbability > 0)
        {
            _wSlow = 0;
            _wFast = 0;
        }
    }

    private int KldLimit(int k)
    {
        if (k <= 1)
        {
            return _options.MinParticles;
        }

        var a = 1.0;
        var b = 2.0 / (9.0 * (k - 1));
        var c = Math.Sqrt(2.0 / (9.0 * (k - 1))) * _options.PfZ;
        var x = a - b + c;
        return (int)Math.Ceiling((k - 1) / (2 * _options.PfErr) * x * x * x);
    }

    private Pose2D RandomFreePose()
    {
        var map = Map;
        for (var tries = 0; tries < 1000; tries++)
        {
            var cx = (int)(_random.NextDouble() * map.Width);
            var cy = (int)(_random.NextDouble() * map.Height);
            if (map.IsFree(cx, cy))
            {
                var (wx, wy) = map.CellToWorld(cx, cy);
                return new Pose2D(wx, wy, (_random.NextDouble() * 2 * Math.PI) - Math.PI);
            }
        }

        return Estimate;
    }

    private void ComputeEstimate()
    {
        double sx = 0, sy = 0, sc = 0, ss = 0, sw = 0;
        for (var i = 0; i < _count; i++)
        {
            var w = _w[i];
            sx += w * _x[i];
            sy += w * _y[i];
            sc += w * Math.Cos(_t[i]);
            ss += w * Math.Sin(_t[i]);
            sw += w;
        }

        if (sw <= 0)
        {
            return;
        }

        var mx = sx / sw;
        var my = sy / sw;
        var mt = Math.Atan2(ss, sc);
        Estimate = new Pose2D(mx, my, mt);
        Array.Clear(Covariance);
        for (var i = 0; i < _count; i++)
        {
            var w = _w[i] / sw;
            var dx = _x[i] - mx;
            var dy = _y[i] - my;
            var dt = Angles.Difference(_t[i], mt);
            Covariance[0] += w * dx * dx;
            Covariance[1] += w * dx * dy;
            Covariance[4] += w * dy * dy;
            Covariance[8] += w * dt * dt;
        }

        Covariance[3] = Covariance[1];
    }

    private void Allocate(int n)
    {
        if (_x.Length < n)
        {
            _x = new double[n];
            _y = new double[n];
            _t = new double[n];
            _w = new double[n];
        }
    }
}
