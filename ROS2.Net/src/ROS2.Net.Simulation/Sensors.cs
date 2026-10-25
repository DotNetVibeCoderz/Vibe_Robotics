namespace ROS2.Net.Simulation;

/// <summary>
/// Base of simulated sensors. A sensor updates at its own rate during world steps; consumers detect
/// fresh data through <see cref="Sequence"/>. Updates reuse preallocated buffers.
/// </summary>
public abstract class SimSensor
{
    private double _accumulator;

    protected SimSensor(string name, string frameSuffix, double rateHz)
    {
        Name = name;
        FrameSuffix = frameSuffix;
        RateHz = rateHz;
    }

    public string Name { get; }

    /// <summary>TF child frame (without the robot prefix), e.g. <c>laser_link</c>.</summary>
    public string FrameSuffix { get; }

    /// <summary>Mounting pose relative to <c>base_link</c>.</summary>
    public Pose2D Mount { get; set; }

    /// <summary>Mounting height above the ground (m).</summary>
    public double MountHeight { get; set; } = 0.1;

    public double RateHz { get; set; }

    public bool Enabled { get; set; } = true;

    /// <summary>Incremented on each update.</summary>
    public long Sequence { get; private set; }

    /// <summary>World time of the latest update (s).</summary>
    public double LastUpdateTime { get; private set; }

    internal void Tick(SimWorld world, SimRobot robot, double dt)
    {
        if (!Enabled || RateHz <= 0)
        {
            return;
        }

        _accumulator += dt;
        var period = 1.0 / RateHz;
        if (_accumulator + 1e-9 >= period)
        {
            _accumulator -= period;
            if (_accumulator > period)
            {
                _accumulator = 0;
            }

            Update(world, robot, period);
            LastUpdateTime = world.Time;
            Sequence++;
        }
    }

    /// <summary>Produces a new measurement.</summary>
    protected abstract void Update(SimWorld world, SimRobot robot, double period);

    internal virtual void Reset()
    {
        _accumulator = 0;
        Sequence = 0;
        LastUpdateTime = 0;
    }

    /// <summary>World pose of the sensor.</summary>
    public Pose2D WorldPose(SimRobot robot) => robot.Pose.Compose(Mount);
}

/// <summary>A planar LiDAR (sensor_msgs/LaserScan).</summary>
public sealed class LidarSensor : SimSensor
{
    public LidarSensor(string name = "lidar", int beams = 360, double angleMin = -Math.PI, double angleMax = Math.PI, double rangeMin = 0.12, double rangeMax = 3.5, double rateHz = 10, double noiseStdDev = 0.01)
        : base(name, "laser_link", rateHz)
    {
        if (beams < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(beams));
        }

        Beams = beams;
        AngleMin = angleMin;
        AngleMax = angleMax;
        RangeMin = rangeMin;
        RangeMax = rangeMax;
        NoiseStdDev = noiseStdDev;
        Ranges = new float[beams];
        HitPoints = new Vec2[beams];
        Ranges.AsSpan().Fill(float.PositiveInfinity);
    }

    public int Beams { get; }

    public double AngleMin { get; }

    public double AngleMax { get; }

    /// <summary>Angle between beams. A full 360° scan does not repeat the first beam.</summary>
    public double AngleIncrement => Beams == 1 ? 0 : IsFullCircle ? (AngleMax - AngleMin) / Beams : (AngleMax - AngleMin) / (Beams - 1);

    public bool IsFullCircle => AngleMax - AngleMin >= (2 * Math.PI) - 1e-6;

    public double RangeMin { get; }

    public double RangeMax { get; }

    public double NoiseStdDev { get; set; }

    /// <summary>Probability that a beam returns no echo (reported as +∞).</summary>
    public double DropoutProbability { get; set; }

    /// <summary>Latest ranges (m); +∞ for no return.</summary>
    public float[] Ranges { get; }

    /// <summary>World-space end point of each beam (hit or max range), for rendering.</summary>
    public Vec2[] HitPoints { get; }

    /// <summary>World pose of the sensor at the latest update.</summary>
    public Pose2D LastOrigin { get; private set; }

    protected override void Update(SimWorld world, SimRobot robot, double period)
    {
        var origin = WorldPose(robot);
        LastOrigin = origin;
        var inc = AngleIncrement;
        var rng = world.Random;
        for (var i = 0; i < Beams; i++)
        {
            var angle = origin.Theta + AngleMin + (i * inc);
            var d = world.RayCast(origin.Position, angle, RangeMax, robot);
            var (s, c) = Math.SinCos(angle);
            HitPoints[i] = new Vec2(origin.X + (c * Math.Min(d, RangeMax)), origin.Y + (s * Math.Min(d, RangeMax)));
            if (d >= RangeMax || (DropoutProbability > 0 && rng.NextDouble() < DropoutProbability))
            {
                Ranges[i] = float.PositiveInfinity;
                continue;
            }

            d += rng.Gaussian(NoiseStdDev);
            Ranges[i] = d < RangeMin ? float.NegativeInfinity : (float)d;
        }
    }

    internal override void Reset()
    {
        base.Reset();
        Ranges.AsSpan().Fill(float.PositiveInfinity);
    }
}

/// <summary>An inertial measurement unit (sensor_msgs/Imu).</summary>
public sealed class ImuSensor(string name = "imu", double rateHz = 100) : SimSensor(name, "imu_link", rateHz)
{
    private double _gyroBias;

    public double GyroNoiseStdDev { get; set; } = 0.002;

    public double AccelNoiseStdDev { get; set; } = 0.02;

    /// <summary>Random-walk rate of the gyro bias (rad/s per √s).</summary>
    public double GyroBiasWalk { get; set; } = 0.0002;

    public double OrientationNoiseStdDev { get; set; } = 0.001;

    /// <summary>Measured yaw (rad).</summary>
    public double Yaw { get; private set; }

    /// <summary>Measured angular velocity about Z (rad/s).</summary>
    public double AngularVelocityZ { get; private set; }

    /// <summary>Measured linear acceleration in the sensor frame (m/s²), gravity included on Z.</summary>
    public (double X, double Y, double Z) LinearAcceleration { get; private set; }

    public double GyroBias => _gyroBias;

    protected override void Update(SimWorld world, SimRobot robot, double period)
    {
        var rng = world.Random;
        _gyroBias += rng.Gaussian(GyroBiasWalk * Math.Sqrt(period));
        Yaw = Angles.Normalize(robot.Pose.Theta + Mount.Theta + rng.Gaussian(OrientationNoiseStdDev));
        AngularVelocityZ = robot.AngularVelocity + _gyroBias + rng.Gaussian(GyroNoiseStdDev);

        // Body acceleration plus centripetal term v·ω, rotated into the mount frame.
        var a = robot.LinearAcceleration + new Vec2(-robot.LateralVelocity * robot.AngularVelocity, robot.LinearVelocity * robot.AngularVelocity);
        var m = a.Rotate(-Mount.Theta);
        LinearAcceleration = (m.X + rng.Gaussian(AccelNoiseStdDev), m.Y + rng.Gaussian(AccelNoiseStdDev), 9.80665 + rng.Gaussian(AccelNoiseStdDev));
    }

    internal override void Reset()
    {
        base.Reset();
        _gyroBias = 0;
    }
}

/// <summary>A GNSS receiver (sensor_msgs/NavSatFix) around a geographic origin.</summary>
public sealed class GpsSensor(string name = "gps", double rateHz = 5) : SimSensor(name, "gps_link", rateHz)
{
    private const double EarthRadius = 6_378_137.0;

    /// <summary>Horizontal noise (m, 1σ).</summary>
    public double NoiseStdDev { get; set; } = 0.5;

    public double Latitude { get; private set; }

    public double Longitude { get; private set; }

    public double Altitude { get; private set; }

    /// <summary>Converts a world position (x east, y north) to latitude/longitude using a local tangent plane.</summary>
    public static (double Latitude, double Longitude) ToGeodetic(GeoOrigin origin, Vec2 position)
    {
        var lat = origin.Latitude + Angles.ToDegrees(position.Y / EarthRadius);
        var lon = origin.Longitude + Angles.ToDegrees(position.X / (EarthRadius * Math.Cos(Angles.ToRadians(origin.Latitude))));
        return (lat, lon);
    }

    protected override void Update(SimWorld world, SimRobot robot, double period)
    {
        var rng = world.Random;
        var p = WorldPose(robot).Position + new Vec2(rng.Gaussian(NoiseStdDev), rng.Gaussian(NoiseStdDev));
        (Latitude, Longitude) = ToGeodetic(world.GeoOrigin, p);
        Altitude = world.GeoOrigin.Altitude + MountHeight + rng.Gaussian(NoiseStdDev * 1.5);
    }
}

/// <summary>Geographic origin of the world frame.</summary>
public readonly record struct GeoOrigin(double Latitude, double Longitude, double Altitude)
{
    /// <summary>Bandung, Indonesia — a friendly default.</summary>
    public static GeoOrigin Default => new(-6.914744, 107.609810, 768);
}

/// <summary>An ultrasonic/IR range finder (sensor_msgs/Range).</summary>
public sealed class RangeSensor : SimSensor
{
    public RangeSensor(string name, Pose2D mount, double fieldOfView = 0.5, double minRange = 0.02, double maxRange = 4.0, double rateHz = 20)
        : base(name, name + "_link", rateHz)
    {
        Mount = mount;
        FieldOfView = fieldOfView;
        MinRange = minRange;
        MaxRange = maxRange;
    }

    public double FieldOfView { get; }

    public double MinRange { get; }

    public double MaxRange { get; }

    public double NoiseStdDev { get; set; } = 0.01;

    /// <summary>Latest distance (m); <see cref="MaxRange"/> + ∞ semantics as in REP-117: +∞ when nothing is detected.</summary>
    public double Range { get; private set; } = double.PositiveInfinity;

    /// <summary>The cone is sampled with this many rays.</summary>
    public int Rays { get; set; } = 7;

    protected override void Update(SimWorld world, SimRobot robot, double period)
    {
        var origin = WorldPose(robot);
        var best = double.PositiveInfinity;
        for (var i = 0; i < Rays; i++)
        {
            var a = origin.Theta - (FieldOfView / 2) + (Rays == 1 ? FieldOfView / 2 : FieldOfView * i / (Rays - 1));
            best = Math.Min(best, world.RayCast(origin.Position, a, MaxRange, robot));
        }

        Range = best >= MaxRange ? double.PositiveInfinity : Math.Max(MinRange, best + world.Random.Gaussian(NoiseStdDev));
    }
}

/// <summary>A battery discharging with motion (sensor_msgs/BatteryState).</summary>
public sealed class BatterySensor(string name = "battery", double rateHz = 1) : SimSensor(name, "battery_link", rateHz)
{
    public double CapacityAh { get; set; } = 1.8;

    public double NominalVoltage { get; set; } = 11.1;

    public double FullVoltage { get; set; } = 12.6;

    public double EmptyVoltage { get; set; } = 9.9;

    public double IdleCurrent { get; set; } = 0.35;

    /// <summary>Extra current per m/s of linear speed and per rad/s of rotation (A).</summary>
    public double MotionCurrentPerSpeed { get; set; } = 1.2;

    /// <summary>Scales how fast simulated time drains the battery (1 = realistic).</summary>
    public double DrainFactor { get; set; } = 1;

    /// <summary>Charging current while <see cref="IsCharging"/> (A).</summary>
    public double ChargeCurrent { get; set; } = 1.5;

    public bool IsCharging { get; set; }

    public double ChargeAh { get; private set; } = 1.8;

    public double Current { get; private set; }

    public double Voltage { get; private set; } = 12.6;

    /// <summary>State of charge in [0, 1].</summary>
    public double Percentage => CapacityAh <= 0 ? 0 : Math.Clamp(ChargeAh / CapacityAh, 0, 1);

    protected override void Update(SimWorld world, SimRobot robot, double period)
    {
        var load = IdleCurrent + (MotionCurrentPerSpeed * (Math.Abs(robot.LinearVelocity) + Math.Abs(robot.LateralVelocity) + (0.1 * Math.Abs(robot.AngularVelocity))));
        Current = IsCharging ? ChargeCurrent : -load;
        ChargeAh = Math.Clamp(ChargeAh + (Current * DrainFactor * period / 3600.0), 0, CapacityAh);
        Voltage = EmptyVoltage + ((FullVoltage - EmptyVoltage) * Percentage) + (0.05 * Current);
    }

    internal override void Reset()
    {
        base.Reset();
        ChargeAh = CapacityAh;
        Voltage = FullVoltage;
        Current = 0;
    }
}

/// <summary>
/// A simple top-down camera: renders an RGB image (row-major, rgb8) of the floor area in front of
/// the robot, with the robot's heading pointing up in the image.
/// </summary>
public sealed class CameraSensor : SimSensor
{
    public CameraSensor(string name = "camera", int width = 96, int height = 96, double metersPerPixel = 0.04, double rateHz = 5)
        : base(name, "camera_link", rateHz)
    {
        Width = width;
        Height = height;
        MetersPerPixel = metersPerPixel;
        Pixels = new byte[width * height * 3];
    }

    public int Width { get; }

    public int Height { get; }

    public double MetersPerPixel { get; }

    /// <summary>Latest image, rgb8, <c>Width*3</c> bytes per row.</summary>
    public byte[] Pixels { get; }

    protected override void Update(SimWorld world, SimRobot robot, double period)
    {
        var pose = WorldPose(robot);
        var (s, c) = Math.SinCos(pose.Theta);
        var i = 0;
        for (var row = 0; row < Height; row++)
        {
            // Row 0 is farthest ahead; the robot sits at the bottom center.
            var forward = (Height - row - 0.5) * MetersPerPixel;
            for (var col = 0; col < Width; col++)
            {
                var left = ((Width / 2.0) - col - 0.5) * MetersPerPixel;
                var p = new Vec2(pose.X + (c * forward) - (s * left), pose.Y + (s * forward) + (c * left));
                var color = world.SampleColor(p, robot);
                Pixels[i++] = color.R;
                Pixels[i++] = color.G;
                Pixels[i++] = color.B;
            }
        }
    }
}
