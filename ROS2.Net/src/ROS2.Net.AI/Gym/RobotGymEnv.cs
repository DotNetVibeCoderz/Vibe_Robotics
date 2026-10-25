using RLNet.Environments;
using RLNet.Spaces;
using ROS2.Net.Simulation;

namespace ROS2.Net.AI.Gym;

/// <summary>Configuration of <see cref="RobotGymEnv"/>.</summary>
public sealed class RobotGymOptions
{
    public ScenarioKind Scenario { get; set; } = ScenarioKind.Empty;

    /// <summary>Supply a custom world instead of a scenario (it must not already contain a robot named <c>agent</c>).</summary>
    public Func<SimWorld>? WorldFactory { get; set; }

    public RobotKind RobotKind { get; set; } = RobotKind.DiffDrive;

    /// <summary>LiDAR sectors in the observation; 0 removes the LiDAR (faster, goal-only tasks).</summary>
    public int Sectors { get; set; } = 8;

    public int LidarBeams { get; set; } = 72;

    public double LidarRange { get; set; } = 3.5;

    /// <summary>Number of discrete actions from <see cref="NavigationObservation.DefaultAction"/> (1..5).</summary>
    public int ActionCount { get; set; } = 5;

    /// <summary>Simulated seconds per <c>Step</c>.</summary>
    public double ControlPeriod { get; set; } = 0.2;

    public int MaxEpisodeSteps { get; set; } = 200;

    public double GoalTolerance { get; set; } = 0.3;

    public double MinGoalDistance { get; set; } = 1.0;

    public double MaxGoalDistance { get; set; } = 3.0;

    /// <summary>Fixed start pose; otherwise the scenario's first spawn point (heading randomized when <see cref="RandomizeStartHeading"/>).</summary>
    public Pose2D? Start { get; set; }

    public bool RandomizeStartHeading { get; set; } = true;

    /// <summary>Fixed goal; otherwise sampled in free space between min and max distance.</summary>
    public Vec2? Goal { get; set; }

    public double ForwardSpeed { get; set; } = 0.22;

    public double TurnSpeed { get; set; } = 1.5;

    public float GoalReward { get; set; } = 10f;

    public float CollisionPenalty { get; set; } = -10f;

    /// <summary>Reward per meter of progress toward the goal.</summary>
    public float ProgressScale { get; set; } = 10f;

    public float StepPenalty { get; set; } = 0.01f;

    public ulong Seed { get; set; } = 7;
}

/// <summary>
/// A Gym-style point-goal navigation task over <see cref="SimWorld"/>, implementing RLNet's
/// <see cref="IDiscreteEnvironment"/> so any RLNet discrete agent (DQN, Q-table, PPO, A2C) can train
/// on it. Observation: <see cref="NavigationObservation"/>. Reward: progress toward the goal, a goal
/// bonus, a collision penalty and a small per-step cost.
/// </summary>
public sealed class RobotGymEnv : IDiscreteEnvironment
{
    private readonly RobotGymOptions _options;
    private readonly float[] _observation;
    private readonly SimRandom _rng;
    private double _previousDistance;

    public RobotGymEnv(RobotGymOptions? options = null)
    {
        _options = options ?? new RobotGymOptions();
        if (_options.ActionCount is < 1 or > 5)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "ActionCount must be 1..5.");
        }

        _rng = new SimRandom(_options.Seed);
        World = _options.WorldFactory?.Invoke() ?? Scenarios.Definition(_options.Scenario, _options.Seed).Build();
        var start = _options.Start ?? (World.SpawnPoints.Count > 0 ? World.SpawnPoints[0].Pose : new Pose2D(0, 0, 0));
        Robot = World.AddRobot(RobotFactory.Create(_options.RobotKind, "agent", start, null, new RobotSensorOptions
        {
            Lidar = _options.Sectors > 0,
            LidarBeams = _options.LidarBeams,
            LidarRange = _options.LidarRange,
            Gps = false,
            Sonar = false,
            Battery = false,
            Imu = false,
        }));
        Robot.Namespace = "/";

        _observation = new float[NavigationObservation.Size(_options.Sectors)];
        var low = new float[_observation.Length];
        var high = new float[_observation.Length];
        for (var i = 0; i < low.Length; i++)
        {
            low[i] = i >= _options.Sectors + 1 ? -1f : 0f;
            high[i] = 1f;
        }

        var labels = Enumerable.Range(0, _options.Sectors).Select(i => $"sector_{i}").Concat(["distance", "cos_bearing", "sin_bearing"]).ToArray();
        ObservationSpace = new BoxSpace(low, high, labels);
        ActionSpace = new DiscreteSpace(_options.ActionCount, NavigationObservation.DefaultActionLabels.Take(_options.ActionCount).ToArray());
    }

    public SimWorld World { get; }

    public SimRobot Robot { get; }

    public Vec2 Goal { get; private set; }

    public string Name => "ROS2.Net/PointGoal-" + _options.Scenario;

    public Space ObservationSpace { get; }

    public DiscreteSpace ActionSpace { get; }

    Space IEnvironment.ActionSpace => ActionSpace;

    public ReadOnlySpan<float> Observation => _observation;

    public int ElapsedSteps { get; private set; }

    public int MaxEpisodeSteps => _options.MaxEpisodeSteps;

    public RobotGymOptions Options => _options;

    /// <summary>True when the last step reached the goal.</summary>
    public bool ReachedGoal { get; private set; }

    /// <summary>True when the last step ended in a collision.</summary>
    public bool Collided { get; private set; }

    public void Reset(int? seed = null)
    {
        if (seed is { } s)
        {
            _rng.Reseed((ulong)s);
            World.Seed = (ulong)s;
        }

        World.Reset();
        var start = _options.Start ?? (World.SpawnPoints.Count > 0 ? World.SpawnPoints[0].Pose : new Pose2D(0, 0, 0));
        if (_options.RandomizeStartHeading)
        {
            start = start with { Theta = _rng.Uniform(-Math.PI, Math.PI) };
        }

        lock (World.SyncRoot)
        {
            Robot.Teleport(start);
        }

        Goal = _options.Goal ?? SampleGoal(start.Position);
        ElapsedSteps = 0;
        ReachedGoal = Collided = false;
        if (_options.Sectors > 0)
        {
            // Refresh the scan at the start pose.
            World.Step();
        }

        _previousDistance = (Goal - Robot.Pose.Position).Length;
        Observe();
    }

    public StepResult Step(int action)
    {
        var (v, w) = NavigationObservation.DefaultAction(action, _options.ForwardSpeed, _options.TurnSpeed);
        var steps = Math.Max(1, (int)Math.Round(_options.ControlPeriod / World.StepSize));
        var collided = false;
        for (var i = 0; i < steps; i++)
        {
            Robot.SetCommand(v, 0, w, World.Time);
            World.Step();
            collided |= Robot.Bumper;
        }

        ElapsedSteps++;
        var distance = (Goal - Robot.Pose.Position).Length;
        var reward = (float)((_previousDistance - distance) * _options.ProgressScale) - _options.StepPenalty;
        _previousDistance = distance;
        Observe();

        ReachedGoal = distance <= _options.GoalTolerance;
        Collided = collided;
        if (ReachedGoal)
        {
            reward += _options.GoalReward;
        }

        if (collided)
        {
            reward += _options.CollisionPenalty;
        }

        var terminated = ReachedGoal || collided;
        var truncated = !terminated && ElapsedSteps >= _options.MaxEpisodeSteps;
        return new StepResult(reward, terminated, truncated);
    }

    private void Observe()
    {
        var lidar = Robot.Lidar;
        NavigationObservation.Fill(
            _observation,
            lidar is null ? [] : lidar.Ranges,
            lidar?.AngleMin ?? 0,
            lidar?.AngleIncrement ?? 0,
            lidar?.RangeMax ?? 1,
            _options.Sectors,
            Robot.Pose,
            Goal,
            _options.MaxGoalDistance * 1.5);
    }

    private Vec2 SampleGoal(Vec2 start)
    {
        for (var attempt = 0; attempt < 200; attempt++)
        {
            var angle = _rng.Uniform(-Math.PI, Math.PI);
            var dist = _rng.Uniform(_options.MinGoalDistance, _options.MaxGoalDistance);
            var g = start + new Vec2(Math.Cos(angle) * dist, Math.Sin(angle) * dist);
            if (World.Bounds.Contains(g) && !World.IsOccupied(g, 0.35))
            {
                return g;
            }
        }

        return start + new Vec2(_options.MinGoalDistance, 0);
    }
}
