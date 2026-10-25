using ROS2.Net.Simulation;
using Shouldly;

namespace ROS2.Net.Simulation.Tests;

public class WorldTests
{
    private static RobotParameters NoNoise() => new() { WheelSlip = 0, EncoderNoise = 0 };

    private static SimWorld Box(double halfSize = 2, ulong seed = 1)
    {
        var w = new SimWorld(new SimWorldOptions { Bounds = new Bounds2D(-halfSize, -halfSize, halfSize, halfSize), Seed = seed });
        w.AddObstacle(SimObstacle.Wall("e", new Vec2(halfSize, -halfSize), new Vec2(halfSize, halfSize)));
        w.AddObstacle(SimObstacle.Wall("w", new Vec2(-halfSize, -halfSize), new Vec2(-halfSize, halfSize)));
        w.AddObstacle(SimObstacle.Wall("n", new Vec2(-halfSize, halfSize), new Vec2(halfSize, halfSize)));
        w.AddObstacle(SimObstacle.Wall("s", new Vec2(-halfSize, -halfSize), new Vec2(halfSize, -halfSize)));
        return w;
    }

    private static void Drive(SimWorld w, SimRobot r, double v, double omega, double seconds)
    {
        var steps = (int)Math.Round(seconds / w.StepSize);
        for (var i = 0; i < steps; i++)
        {
            r.SetCommand(v, 0, omega, w.Time);
            w.Step();
        }
    }

    [Fact]
    public void Same_seed_gives_identical_runs_and_different_seeds_differ()
    {
        Pose2D Run(ulong seed)
        {
            var w = Box(seed: seed);
            var r = w.AddRobot(RobotFactory.Create(RobotKind.DiffDrive, "r", new Pose2D(0, 0, 0)));
            Drive(w, r, 0.2, 0.5, 3);
            return r.Pose;
        }

        Run(5).ShouldBe(Run(5));
        Run(5).ShouldNotBe(Run(6));
    }

    [Fact]
    public void Diff_drive_follows_unicycle_kinematics()
    {
        var w = new SimWorld();
        var r = w.AddRobot(new DiffDriveRobot("r", new Pose2D(0, 0, 0), NoNoise()));
        Drive(w, r, 0.2, 0, 2);

        // 2 s at 0.2 m/s minus the acceleration ramp (0.2/2.5 s at half speed).
        r.Pose.X.ShouldBe(0.4 - (0.2 * 0.2 / 2.5 / 2), 0.005);
        r.Pose.Y.ShouldBe(0, 1e-9);
        r.OdomPose.X.ShouldBe(r.Pose.X, 1e-9);

        var w2 = new SimWorld();
        var r2 = w2.AddRobot(new DiffDriveRobot("r", new Pose2D(0, 0, 0), NoNoise()));
        Drive(w2, r2, 0, 1.0, Math.PI / 2 + 0.05);
        Math.Abs(Angles.Normalize(r2.Pose.Theta - (Math.PI / 2))).ShouldBeLessThan(0.02);
        r2.Pose.Position.Length.ShouldBeLessThan(1e-9);
    }

    [Fact]
    public void Commands_time_out()
    {
        var w = new SimWorld();
        var r = w.AddRobot(new DiffDriveRobot("r", new Pose2D(0, 0, 0), NoNoise()));
        r.SetCommand(0.2, 0, 0, w.Time);
        w.StepFor(2);
        var x = r.Pose.X;
        x.ShouldBeLessThan(0.15); // only ~0.5 s of motion
        w.StepFor(1);
        r.Pose.X.ShouldBe(x, 1e-6);
    }

    [Fact]
    public void Ackermann_and_omni_move_as_expected()
    {
        var w = new SimWorld();
        var car = w.AddRobot(new AckermannRobot("car", new Pose2D(0, 0, 0), RobotParameters.Car()));
        car.Parameters.WheelSlip = 0;
        var omni = w.AddRobot(new OmniRobot("omni", new Pose2D(5, 0, 0)));
        omni.Parameters.WheelSlip = 0;
        for (var i = 0; i < 200; i++)
        {
            car.SetCommand(1.0, 0, 0.5, w.Time);
            omni.SetCommand(0, 0.5, 0, w.Time);
            w.Step();
        }

        car.Pose.Theta.ShouldBeGreaterThan(0.3);
        car.SteeringAngle.ShouldBeGreaterThan(0);
        omni.Pose.Y.ShouldBeGreaterThan(0.8);
        omni.Pose.X.ShouldBe(5, 1e-6);
    }

    [Fact]
    public void Lidar_measures_known_walls()
    {
        var w = Box(2);
        w.AddObstacle(SimObstacle.Cylinder("pillar", 0, 1.0, 0.2));
        var r = w.AddRobot(new DiffDriveRobot("r", new Pose2D(0, 0, 0), NoNoise()));
        var lidar = r.AddSensor(new LidarSensor(beams: 360, rangeMax: 10, noiseStdDev: 0) { Mount = new Pose2D(0, 0, 0) });
        w.StepFor(0.1);

        lidar.Sequence.ShouldBeGreaterThan(0);
        lidar.Ranges[180].ShouldBe(1.95f, 1e-4f);   // ahead: east wall inner face
        lidar.Ranges[0].ShouldBe(1.95f, 1e-4f);     // behind
        lidar.Ranges[270].ShouldBe(0.8f, 1e-4f);    // left: pillar surface at y = 0.8
        lidar.Ranges[90].ShouldBe(1.95f, 1e-4f);    // right
        lidar.HitPoints[180].X.ShouldBe(1.95, 1e-4);

        // Diagonal hits the corner region at ~1.95·√2.
        lidar.Ranges[225].ShouldBe((float)(1.95 * Math.Sqrt(2)), 0.01f);
    }

    [Fact]
    public void Lidar_sees_other_robots_but_not_itself()
    {
        var w = new SimWorld();
        var a = w.AddRobot(new DiffDriveRobot("a", new Pose2D(0, 0, 0), NoNoise()));
        w.AddRobot(new DiffDriveRobot("b", new Pose2D(1, 0, 0), NoNoise()));
        var lidar = a.AddSensor(new LidarSensor(beams: 4, rangeMax: 5, noiseStdDev: 0));
        w.StepFor(0.1);
        lidar.Ranges[2].ShouldBe((float)(1 - 0.105), 1e-4f);
        float.IsPositiveInfinity(lidar.Ranges[0]).ShouldBeTrue();
    }

    [Fact]
    public void Robots_stop_at_walls_and_report_bumper()
    {
        var w = Box(1);
        var r = w.AddRobot(new DiffDriveRobot("r", new Pose2D(0, 0, 0), NoNoise()));
        Drive(w, r, 0.22, 0, 8);
        r.Bumper.ShouldBeTrue();
        r.CollisionCount.ShouldBeGreaterThan(0);
        r.Pose.X.ShouldBeLessThanOrEqualTo(1 - 0.05 - 0.105 + 1e-4);
        r.Pose.X.ShouldBeGreaterThan(0.8);
    }

    [Fact]
    public void Light_crates_are_pushed()
    {
        var w = new SimWorld();
        var crate = w.AddObstacle(SimObstacle.Box("crate", 0.5, 0, 0.3, 0.3, dynamic: true, mass: 1));
        var r = w.AddRobot(new DiffDriveRobot("r", new Pose2D(0, 0, 0), NoNoise()));
        Drive(w, r, 0.2, 0, 3);
        crate.Center.X.ShouldBeGreaterThan(0.7);
        w.Reset();
        crate.Center.X.ShouldBe(0.5, 1e-9);
        r.Pose.ShouldBe(new Pose2D(0, 0, 0));
    }

    [Fact]
    public void Imu_noise_matches_configuration()
    {
        var w = new SimWorld(new SimWorldOptions { Seed = 3 });
        var r = w.AddRobot(new DiffDriveRobot("r", new Pose2D(0, 0, 0.5)));
        var imu = r.AddSensor(new ImuSensor(rateHz: 100) { GyroBiasWalk = 0 });
        var samples = new List<double>();
        var az = new List<double>();
        for (var i = 0; i < 2000; i++)
        {
            w.Step();
            samples.Add(imu.AngularVelocityZ);
            az.Add(imu.LinearAcceleration.Z);
        }

        samples.Average().ShouldBe(0, 0.0005);
        Std(samples).ShouldBe(imu.GyroNoiseStdDev, 0.0003);
        az.Average().ShouldBe(9.80665, 0.005);
        imu.Yaw.ShouldBe(0.5, 0.01);
    }

    [Fact]
    public void Wheel_slip_makes_odometry_drift_from_truth()
    {
        var w = new SimWorld(new SimWorldOptions { Seed = 9 });
        var r = w.AddRobot(new DiffDriveRobot("r", new Pose2D(0, 0, 0), new RobotParameters { WheelSlip = 0.05 }));
        Drive(w, r, 0.2, 0.3, 20);
        var error = (r.Pose.Position - r.OdomPose.Position).Length;
        error.ShouldBeGreaterThan(1e-4);
        error.ShouldBeLessThan(0.5);
    }

    [Fact]
    public void Gps_battery_sonar_and_camera_report()
    {
        var w = Scenarios.Create(ScenarioKind.Warehouse, robots: 1, sensors: new RobotSensorOptions { Camera = true });
        var r = w.Robots[0];
        Drive(w, r, 0.2, 0, 1.5);

        r.Gps!.Latitude.ShouldBe(GeoOrigin.Default.Latitude, 0.001);
        r.Battery!.Percentage.ShouldBeLessThan(1);
        r.Battery.Percentage.ShouldBeGreaterThan(0.99);
        r.RangeSensors.Count().ShouldBe(2);
        r.Camera!.Sequence.ShouldBeGreaterThan(0);
        r.Camera.Pixels.Distinct().Count().ShouldBeGreaterThan(2);
    }

    [Fact]
    public void Occupancy_grid_marks_walls_and_free_space()
    {
        var w = Scenarios.Create(ScenarioKind.CorridorLoop);
        var grid = w.ToOccupancyGrid(0.1);
        grid.Info.Width.ShouldBe(140u);
        grid.Info.Height.ShouldBe(100u);
        sbyte Cell(double x, double y) => grid.Data[((int)(y / 0.1) * (int)grid.Info.Width) + (int)(x / 0.1)];
        Cell(1, 1).ShouldBe((sbyte)0);      // corridor
        Cell(7, 5).ShouldBe((sbyte)100);    // core block
        Cell(0.02, 5).ShouldBe((sbyte)100); // outer wall
        grid.Header.FrameId.ShouldBe("map");
    }

    [Theory]
    [InlineData(ScenarioKind.Empty)]
    [InlineData(ScenarioKind.Warehouse)]
    [InlineData(ScenarioKind.Office)]
    [InlineData(ScenarioKind.Maze)]
    [InlineData(ScenarioKind.CorridorLoop)]
    [InlineData(ScenarioKind.AgricultureRows)]
    public void Scenarios_spawn_robots_in_free_space(ScenarioKind kind)
    {
        var w = Scenarios.Create(kind, robots: 2);
        w.Robots.Count.ShouldBe(2);
        foreach (var r in w.Robots)
        {
            w.IsOccupied(r.Pose.Position, r.Parameters.Radius).ShouldBeFalse($"{r.Name} spawned inside an obstacle in {kind}");
        }

        w.StepFor(0.5);
        Scenarios.Parse(kind.ToString().ToLowerInvariant()).ShouldBe(kind);
    }

    [Fact]
    public void World_files_round_trip_through_json_and_yaml()
    {
        var def = Scenarios.Warehouse();
        def.Robots.Add(new RobotDefinition { Name = "bot", Type = "omni", X = 2, Y = 2, Yaw = 90 });
        var fromJson = WorldDefinition.FromJson(def.ToJson());
        var fromYaml = WorldDefinition.FromYaml(def.ToYaml());
        foreach (var d in new[] { fromJson, fromYaml })
        {
            d.Obstacles.Count.ShouldBe(def.Obstacles.Count);
            var world = d.Build();
            world.Robots.Single().Kind.ShouldBe(RobotKind.Omni);
            world.Robots.Single().Pose.Theta.ShouldBe(Math.PI / 2, 1e-9);
            world.Waypoints.ShouldContainKey("loading_dock");
        }

        var yaml = """
            name: lab
            bounds: [0, 0, 6, 4]
            obstacles:
              - { type: wall, x1: 0, y1: 0, x2: 6, y2: 0 }
              - { type: cylinder, x: 3, y: 2, radius: 0.3 }
            robots:
              - { name: r1, type: diffdrive, x: 1, y: 1 }
            """;
        var lab = WorldDefinition.FromYaml(yaml).Build();
        lab.StaticObstacles.Count.ShouldBe(2);
        lab.IsOccupied(new Vec2(3, 2)).ShouldBeTrue();
    }

    [Fact]
    public void Snapshot_reuses_static_geometry_and_copies_dynamic_state()
    {
        var w = Scenarios.Create(ScenarioKind.Warehouse, robots: 1);
        w.StepFor(0.2);
        var a = w.Snapshot();
        var b = w.Snapshot();
        a.StaticObstacles.ShouldBeSameAs(b.StaticObstacles);
        a.Robots[0].LaserHits.Count.ShouldBe(360);
        a.Robots[0].Trail.Count.ShouldBeGreaterThan(0);
        a.DynamicObstacles.Count.ShouldBe(6);
        a.Obstacles.Count().ShouldBe(a.StaticObstacles.Count + 6);
    }

    [Fact]
    public void Arm_joints_track_targets()
    {
        var w = new SimWorld();
        var r = w.AddRobot(RobotFactory.Create(RobotKind.DiffDrive, "arm", new Pose2D(0, 0, 0), sensors: new RobotSensorOptions { Arm = "6dof" }));
        r.Arm!.SetTargets([0.5, -0.5]);
        w.StepFor(1);
        r.Arm.Positions[0].ShouldBe(0.5, 1e-6);
        r.Arm.Positions[1].ShouldBe(-0.5, 1e-6);
    }

    [Fact]
    public void Physics_step_does_not_allocate()
    {
        var w = Scenarios.Create(ScenarioKind.Warehouse, robots: 3, sensors: new RobotSensorOptions { Gps = false });
        w.StepFor(1); // warm up (index build, JIT)
        var robots = w.Robots;
        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var i = 0; i < 200; i++)
        {
            for (var k = 0; k < robots.Count; k++)
            {
                robots[k].SetCommand(0.1, 0, 0.2, w.Time);
            }

            w.Step();
        }

        (GC.GetAllocatedBytesForCurrentThread() - before).ShouldBeLessThan(1024);
    }

    private static double Std(List<double> xs)
    {
        var m = xs.Average();
        return Math.Sqrt(xs.Sum(x => (x - m) * (x - m)) / xs.Count);
    }
}
