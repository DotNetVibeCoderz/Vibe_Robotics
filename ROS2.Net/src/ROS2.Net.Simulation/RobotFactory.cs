namespace ROS2.Net.Simulation;

/// <summary>Which sensors <see cref="RobotFactory"/> mounts on a new robot.</summary>
public sealed class RobotSensorOptions
{
    public bool Lidar { get; set; } = true;

    public int LidarBeams { get; set; } = 360;

    public double LidarRange { get; set; } = 3.5;

    public bool Imu { get; set; } = true;

    public bool Gps { get; set; } = true;

    public bool Battery { get; set; } = true;

    /// <summary>Front and rear ultrasonic sensors.</summary>
    public bool Sonar { get; set; } = true;

    /// <summary>Top-down RGB camera (off by default: it is the most expensive sensor).</summary>
    public bool Camera { get; set; }

    /// <summary>Arm placeholder: <c>none</c>, <c>2dof</c> or <c>6dof</c>.</summary>
    public string Arm { get; set; } = "none";
}

/// <summary>Creates robots with a standard sensor suite and TurtleBot3-like mounting.</summary>
public static class RobotFactory
{
    public static SimRobot Create(RobotKind kind, string name, Pose2D pose, RobotParameters? parameters = null, RobotSensorOptions? sensors = null)
    {
        SimRobot robot = kind switch
        {
            RobotKind.Ackermann => new AckermannRobot(name, pose, parameters),
            RobotKind.Omni => new OmniRobot(name, pose, parameters),
            _ => new DiffDriveRobot(name, pose, parameters),
        };
        Equip(robot, sensors ?? new RobotSensorOptions());
        return robot;
    }

    /// <summary>Mounts the standard sensors on an existing robot.</summary>
    public static void Equip(SimRobot robot, RobotSensorOptions o)
    {
        var p = robot.Parameters;
        if (o.Lidar)
        {
            robot.AddSensor(new LidarSensor(beams: o.LidarBeams, rangeMax: o.LidarRange)
            {
                Mount = new Pose2D(-0.032 * (p.Length / 0.14), 0, 0),
                MountHeight = p.Height * 0.9,
            });
        }

        if (o.Imu)
        {
            robot.AddSensor(new ImuSensor { Mount = new Pose2D(0, 0, 0), MountHeight = 0.07 });
        }

        if (o.Gps)
        {
            robot.AddSensor(new GpsSensor { MountHeight = p.Height });
        }

        if (o.Battery)
        {
            robot.AddSensor(new BatterySensor());
        }

        if (o.Sonar)
        {
            robot.AddSensor(new RangeSensor("sonar_front", new Pose2D(p.Length / 2, 0, 0)));
            robot.AddSensor(new RangeSensor("sonar_rear", new Pose2D(-p.Length / 2, 0, Math.PI)));
        }

        if (o.Camera)
        {
            robot.AddSensor(new CameraSensor { Mount = new Pose2D(p.Length / 2, 0, 0), MountHeight = p.Height });
        }

        robot.Arm = o.Arm.ToLowerInvariant() switch
        {
            "2dof" => SimArm.TwoDof(),
            "6dof" => SimArm.SixDof(),
            _ => null,
        };
    }

    /// <summary>Parses <c>diffdrive</c>, <c>ackermann</c>/<c>car</c>, <c>omni</c>/<c>mecanum</c>.</summary>
    public static RobotKind ParseKind(string? text) => text?.ToLowerInvariant() switch
    {
        "ackermann" or "car" => RobotKind.Ackermann,
        "omni" or "mecanum" or "holonomic" => RobotKind.Omni,
        _ => RobotKind.DiffDrive,
    };

    /// <summary>Parameter presets: <c>burger</c>, <c>waffle</c>, <c>car</c>, <c>omni</c>.</summary>
    public static RobotParameters? Preset(string? name) => name?.ToLowerInvariant() switch
    {
        "burger" or "turtlebot3_burger" => RobotParameters.TurtleBot3Burger(),
        "waffle" or "turtlebot3_waffle" => RobotParameters.TurtleBot3Waffle(),
        "car" => RobotParameters.Car(),
        "omni" => RobotParameters.OmniBase(),
        _ => null,
    };
}
