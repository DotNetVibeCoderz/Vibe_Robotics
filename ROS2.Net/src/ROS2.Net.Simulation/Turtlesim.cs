using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.StdSrvs;
using ROS2.Net.Msgs.Turtlesim;
using TurtlePose = ROS2.Net.Msgs.Turtlesim.Pose;

namespace ROS2.Net.Simulation;

/// <summary>A line drawn by a turtle's pen.</summary>
public readonly record struct PenSegment(Vec2 From, Vec2 To, SimColor Color, int Width);

/// <summary>One turtle of a <see cref="TurtleWorld"/>.</summary>
public sealed class Turtle
{
    internal Turtle(string name, Pose2D pose)
    {
        Name = name;
        Pose = pose;
    }

    public string Name { get; }

    public Pose2D Pose { get; internal set; }

    public double LinearVelocity { get; internal set; }

    public double AngularVelocity { get; internal set; }

    public SimColor PenColor { get; internal set; } = new(179, 184, 255);

    public int PenWidth { get; internal set; } = 3;

    public bool PenOff { get; internal set; }

    internal double CommandTime { get; set; } = double.NegativeInfinity;
}

/// <summary>
/// The classic turtlesim world: an 11.09 m square, turtles that follow a command for one second,
/// pens that draw on the background. Used by <see cref="TurtlesimNode"/>.
/// </summary>
public sealed class TurtleWorld
{
    public const double Size = 11.088889;
    public static readonly SimColor DefaultBackground = new(69, 86, 255);

    private readonly object _gate = new();
    private readonly Dictionary<string, Turtle> _turtles = new(StringComparer.Ordinal);
    private readonly List<PenSegment> _segments = [];

    public SimColor Background { get; set; } = DefaultBackground;

    public double Time { get; private set; }

    /// <summary>Update period (s); turtlesim uses 16 ms.</summary>
    public double StepSize { get; set; } = 0.016;

    public object SyncRoot => _gate;

    public IReadOnlyList<Turtle> Turtles
    {
        get
        {
            lock (_gate)
            {
                return _turtles.Values.ToList();
            }
        }
    }

    public Turtle? Get(string name)
    {
        lock (_gate)
        {
            return _turtles.GetValueOrDefault(name);
        }
    }

    public Turtle Spawn(string name, double x, double y, double theta)
    {
        lock (_gate)
        {
            if (_turtles.ContainsKey(name))
            {
                throw new InvalidOperationException($"A turtle named '{name}' already exists.");
            }

            var t = new Turtle(name, new Pose2D(x, y, theta));
            _turtles[name] = t;
            return t;
        }
    }

    public bool Kill(string name)
    {
        lock (_gate)
        {
            return _turtles.Remove(name);
        }
    }

    public string NextName()
    {
        lock (_gate)
        {
            for (var i = 1; ; i++)
            {
                if (!_turtles.ContainsKey($"turtle{i}"))
                {
                    return $"turtle{i}";
                }
            }
        }
    }

    public void Command(string name, double linear, double angular)
    {
        lock (_gate)
        {
            if (_turtles.TryGetValue(name, out var t))
            {
                t.LinearVelocity = linear;
                t.AngularVelocity = angular;
                t.CommandTime = Time;
            }
        }
    }

    public void Teleport(string name, Pose2D pose)
    {
        lock (_gate)
        {
            if (_turtles.TryGetValue(name, out var t))
            {
                DrawTo(t, pose);
            }
        }
    }

    public void SetPen(string name, SimColor color, int width, bool off)
    {
        lock (_gate)
        {
            if (_turtles.TryGetValue(name, out var t))
            {
                t.PenColor = color;
                t.PenWidth = width;
                t.PenOff = off;
            }
        }
    }

    public void Clear()
    {
        lock (_gate)
        {
            _segments.Clear();
        }
    }

    /// <summary>Removes all turtles and drawings and respawns <c>turtle1</c> at the center.</summary>
    public void Reset()
    {
        lock (_gate)
        {
            _turtles.Clear();
            _segments.Clear();
            Time = 0;
            _turtles["turtle1"] = new Turtle("turtle1", new Pose2D(Size / 2, Size / 2, 0));
        }
    }

    public void Step()
    {
        lock (_gate)
        {
            Time += StepSize;
            foreach (var t in _turtles.Values)
            {
                if (Time - t.CommandTime > 1.0)
                {
                    t.LinearVelocity = 0;
                    t.AngularVelocity = 0;
                }

                var theta = Angles.Normalize(t.Pose.Theta + (t.AngularVelocity * StepSize));
                var x = t.Pose.X + (Math.Cos(t.Pose.Theta) * t.LinearVelocity * StepSize);
                var y = t.Pose.Y + (Math.Sin(t.Pose.Theta) * t.LinearVelocity * StepSize);
                DrawTo(t, new Pose2D(Math.Clamp(x, 0, Size), Math.Clamp(y, 0, Size), theta));
            }
        }
    }

    private void DrawTo(Turtle t, Pose2D pose)
    {
        if (!t.PenOff && (pose.Position - t.Pose.Position).LengthSquared > 1e-12)
        {
            _segments.Add(new PenSegment(t.Pose.Position, pose.Position, t.PenColor, t.PenWidth));
        }

        t.Pose = pose;
    }

    /// <summary>Background color or the topmost pen color under a point (turtlesim's color sensor).</summary>
    public SimColor ColorAt(Vec2 p)
    {
        lock (_gate)
        {
            for (var i = _segments.Count - 1; i >= 0; i--)
            {
                var s = _segments[i];
                var c = SimObstacle.ClosestOnSegment(p, s.From, s.To);
                if ((c - p).Length <= s.Width * 0.5 * (Size / 500.0))
                {
                    return s.Color;
                }
            }

            return Background;
        }
    }

    /// <summary>Render-facing copy of the state.</summary>
    public TurtleSnapshot Snapshot()
    {
        lock (_gate)
        {
            return new TurtleSnapshot(Time, Background,
                _turtles.Values.Select(t => new TurtleState(t.Name, t.Pose, t.LinearVelocity, t.AngularVelocity, t.PenColor, t.PenOff)).ToArray(),
                _segments.ToArray());
        }
    }
}

/// <summary>Immutable state of a turtle.</summary>
public sealed record TurtleState(string Name, Pose2D Pose, double LinearVelocity, double AngularVelocity, SimColor PenColor, bool PenOff);

/// <summary>Immutable state of a <see cref="TurtleWorld"/>.</summary>
public sealed record TurtleSnapshot(double Time, SimColor Background, IReadOnlyList<TurtleState> Turtles, IReadOnlyList<PenSegment> Segments);

/// <summary>
/// A drop-in for the ROS 2 <c>turtlesim_node</c>: <c>/turtleN/cmd_vel</c>, <c>/turtleN/pose</c>,
/// <c>/turtleN/color_sensor</c>, <c>/spawn</c>, <c>/kill</c>, <c>/clear</c>, <c>/reset</c>,
/// <c>/turtleN/set_pen</c>, <c>/turtleN/teleport_absolute</c>, <c>/turtleN/teleport_relative</c>
/// and the <c>/turtleN/rotate_absolute</c> action — so the standard ROS tutorials run unchanged.
/// </summary>
public sealed class TurtlesimNode : Node
{
    private readonly Dictionary<string, TurtleEndpoints> _endpoints = new(StringComparer.Ordinal);
    private readonly object _gate = new();
    private readonly CancellationTokenSource _cts = new();
    private Task? _loop;

    public TurtlesimNode(Ros2Context context, TurtleWorld? world = null, string name = "turtlesim", NodeOptions? options = null)
        : base(context, name, options)
    {
        World = world ?? new TurtleWorld();
        if (World.Turtles.Count == 0)
        {
            World.Reset();
        }

        foreach (var t in World.Turtles)
        {
            AddEndpoints(t.Name);
        }

        CreateService<SpawnRequest, SpawnResponse>("/spawn", req =>
        {
            var n = string.IsNullOrWhiteSpace(req.Name) ? World.NextName() : req.Name;
            if (World.Get(n) is not null)
            {
                return new SpawnResponse { Name = string.Empty };
            }

            World.Spawn(n, req.X, req.Y, req.Theta);
            AddEndpoints(n);
            return new SpawnResponse { Name = n };
        });
        CreateService<KillRequest, KillResponse>("/kill", req =>
        {
            if (World.Kill(req.Name))
            {
                RemoveEndpoints(req.Name);
            }

            return new KillResponse();
        });
        CreateService<EmptyRequest, EmptyResponse>("/clear", _ =>
        {
            World.Clear();
            return new EmptyResponse();
        });
        CreateService<EmptyRequest, EmptyResponse>("/reset", _ =>
        {
            lock (_gate)
            {
                foreach (var n in _endpoints.Keys.ToList())
                {
                    RemoveEndpoints(n);
                }
            }

            World.Reset();
            AddEndpoints("turtle1");
            return new EmptyResponse();
        });
    }

    public TurtleWorld World { get; }

    /// <summary>Starts the 62.5 Hz update loop (also started by <see cref="RunAsync"/>).</summary>
    public void Start() => _loop ??= Task.Run(() => LoopAsync(_cts.Token));

    /// <summary>Runs the update loop and spins the node until cancelled.</summary>
    public async Task RunAsync(CancellationToken cancellationToken = default)
    {
        Start();
        await SpinAsync(cancellationToken).ConfigureAwait(false);
    }

    /// <summary>Advances the world one tick and publishes poses (for tests and manual stepping).</summary>
    public void Tick()
    {
        TurtleEndpoints[] eps;
        lock (_gate)
        {
            eps = [.. _endpoints.Values];
        }

        foreach (var e in eps)
        {
            while (e.Cmd.TryTake(out var cmd))
            {
                World.Command(e.Name, cmd.Linear.X, cmd.Angular.Z);
            }
        }

        World.Step();
        foreach (var e in eps)
        {
            if (World.Get(e.Name) is not { } t)
            {
                continue;
            }

            e.Pose.Publish(new TurtlePose
            {
                X = (float)t.Pose.X,
                Y = (float)t.Pose.Y,
                Theta = (float)t.Pose.Theta,
                LinearVelocity = (float)t.LinearVelocity,
                AngularVelocity = (float)t.AngularVelocity,
            });
            var c = World.ColorAt(t.Pose.Position);
            e.Color.Publish(new Color { R = c.R, G = c.G, B = c.B });
        }
    }

    private async Task LoopAsync(CancellationToken ct)
    {
        var next = DateTime.UtcNow;
        while (!ct.IsCancellationRequested && !Context.IsShutdown)
        {
            Tick();
            next += TimeSpan.FromSeconds(World.StepSize);
            var wait = next - DateTime.UtcNow;
            if (wait > TimeSpan.Zero)
            {
                try
                {
                    await Task.Delay(wait, ct).ConfigureAwait(false);
                }
                catch (OperationCanceledException)
                {
                    return;
                }
            }
            else
            {
                next = DateTime.UtcNow;
            }
        }
    }

    private void AddEndpoints(string turtle)
    {
        var p = "/" + turtle;
        var e = new TurtleEndpoints(
            turtle,
            CreateSubscription<Twist>(p + "/cmd_vel", QosProfile.Default),
            CreatePublisher<TurtlePose>(p + "/pose"),
            CreatePublisher<Color>(p + "/color_sensor"));

        e.Owned.Add(CreateService<SetPenRequest, SetPenResponse>(p + "/set_pen", req =>
        {
            World.SetPen(turtle, new SimColor(req.R, req.G, req.B), req.Width, req.Off != 0);
            return new SetPenResponse();
        }));
        e.Owned.Add(CreateService<TeleportAbsoluteRequest, TeleportAbsoluteResponse>(p + "/teleport_absolute", req =>
        {
            World.Teleport(turtle, new Pose2D(req.X, req.Y, req.Theta));
            return new TeleportAbsoluteResponse();
        }));
        e.Owned.Add(CreateService<TeleportRelativeRequest, TeleportRelativeResponse>(p + "/teleport_relative", req =>
        {
            if (World.Get(turtle) is { } t)
            {
                var theta = Angles.Normalize(t.Pose.Theta + req.Angular);
                World.Teleport(turtle, new Pose2D(t.Pose.X + (Math.Cos(theta) * req.Linear), t.Pose.Y + (Math.Sin(theta) * req.Linear), theta));
            }

            return new TeleportRelativeResponse();
        }));
        e.Owned.Add(CreateActionServer<RotateAbsoluteGoal, RotateAbsoluteResult, RotateAbsoluteFeedback>(p + "/rotate_absolute", async (goal, ct) =>
        {
            var start = World.Get(turtle)?.Pose.Theta ?? 0;
            while (true)
            {
                ct.ThrowIfCancellationRequested();
                if (World.Get(turtle) is not { } t)
                {
                    goal.Abort();
                    return new RotateAbsoluteResult();
                }

                var remaining = Angles.Normalize(goal.Goal.Theta - t.Pose.Theta);
                goal.PublishFeedback(new RotateAbsoluteFeedback { Remaining = (float)remaining });
                if (Math.Abs(remaining) < 0.02)
                {
                    World.Command(turtle, 0, 0);
                    return new RotateAbsoluteResult { Delta = (float)Angles.Normalize(t.Pose.Theta - start) };
                }

                World.Command(turtle, 0, remaining > 0 ? 1.0 : -1.0);
                await Task.Delay(16, ct).ConfigureAwait(false);
            }
        }));

        lock (_gate)
        {
            _endpoints[turtle] = e;
        }
    }

    private void RemoveEndpoints(string turtle)
    {
        TurtleEndpoints? e;
        lock (_gate)
        {
            _endpoints.Remove(turtle, out e);
        }

        e?.Dispose();
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _cts.Cancel();
        }

        base.Dispose(disposing);
    }

    private sealed class TurtleEndpoints(string name, Subscription<Twist> cmd, Publisher<TurtlePose> pose, Publisher<Color> color) : IDisposable
    {
        public string Name { get; } = name;

        public Subscription<Twist> Cmd { get; } = cmd;

        public Publisher<TurtlePose> Pose { get; } = pose;

        public Publisher<Color> Color { get; } = color;

        public List<IDisposable> Owned { get; } = [];

        public void Dispose()
        {
            Cmd.Dispose();
            Pose.Dispose();
            Color.Dispose();
            foreach (var o in Owned)
            {
                o.Dispose();
            }
        }
    }
}
