using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using StdString = ROS2.Net.Msgs.StdMsgs.String;

namespace ROS2.Net.AI.Llm;

/// <summary>Hard limits applied to everything the language model asks for.</summary>
public sealed class SafetyLimiter
{
    public double MaxLinearSpeed { get; set; } = 0.3;

    public double MaxAngularSpeed { get; set; } = 1.0;

    /// <summary>Longest single translation (m).</summary>
    public double MaxDistance { get; set; } = 5.0;

    /// <summary>Largest single rotation (degrees).</summary>
    public double MaxRotationDegrees { get; set; } = 360;

    /// <summary>Stop forward motion when an obstacle is closer than this in the front ±30° (m). 0 disables.</summary>
    public double StopDistance { get; set; } = 0.3;

    public double ClampLinear(double v) => Math.Clamp(double.IsFinite(v) ? v : 0, -MaxLinearSpeed, MaxLinearSpeed);

    public double ClampAngular(double w) => Math.Clamp(double.IsFinite(w) ? w : 0, -MaxAngularSpeed, MaxAngularSpeed);

    public double ClampDistance(double d) => Math.Clamp(double.IsFinite(d) ? d : 0, -MaxDistance, MaxDistance);

    public double ClampRotation(double deg) => Math.Clamp(double.IsFinite(deg) ? deg : 0, -MaxRotationDegrees, MaxRotationDegrees);
}

/// <summary>Options of <see cref="NaturalLanguageCommanderNode"/>.</summary>
public sealed class CommanderOptions
{
    public string CommandTopic { get; set; } = "/nl_command";

    public string FeedbackTopic { get; set; } = "/nl_feedback";

    public string CmdVelTopic { get; set; } = "/cmd_vel";

    /// <summary>Waypoint goals are published here (geometry_msgs/PoseStamped) for a navigation stack.</summary>
    public string GoalTopic { get; set; } = "/goal_pose";

    /// <summary>LiDAR used by the obstacle stop; null disables it.</summary>
    public string? ScanTopic { get; set; } = "/scan";

    public string GoalFrame { get; set; } = "map";

    /// <summary>Speed used when the model does not specify one (m/s).</summary>
    public double DefaultSpeed { get; set; } = 0.2;

    public double DefaultAngularSpeed { get; set; } = 0.6;

    public double PublishRateHz { get; set; } = 10;

    public SafetyLimiter Safety { get; set; } = new();

    /// <summary>Named places the model may send the robot to.</summary>
    public Dictionary<string, (double X, double Y, double Yaw)> Waypoints { get; set; } = new(StringComparer.OrdinalIgnoreCase);

    public string SystemPrompt { get; set; } =
        "You control a mobile robot. Translate the user's request (English or Bahasa Indonesia) into tool calls, in order. " +
        "Positive angles turn left (counter-clockwise). Use stop when asked to stop. Never invent waypoints.";
}

/// <summary>
/// Turns natural-language commands on <c>/nl_command</c> (std_msgs/String) into robot motion. A chat
/// model chooses tools — <c>move_forward</c>, <c>rotate</c>, <c>go_to_waypoint</c>, <c>stop</c> — and the
/// node executes them as timed <c>cmd_vel</c> motions or navigation goals, always through the
/// <see cref="SafetyLimiter"/>. Works without spinning (it reads its topics directly).
/// </summary>
public sealed class NaturalLanguageCommanderNode : Node
{
    private readonly IChatModel _model;
    private readonly CommanderOptions _options;
    private readonly Publisher<Twist> _cmd;
    private readonly Publisher<StdString> _feedback;
    private readonly Publisher<PoseStamped> _goal;
    private readonly CancellationTokenSource _cts = new();
    private readonly SemaphoreSlim _busy = new(1, 1);
    private CancellationTokenSource? _motion;
    private volatile float _frontClearance = float.PositiveInfinity;

    public NaturalLanguageCommanderNode(Ros2Context context, IChatModel model, CommanderOptions? options = null, string name = "nl_commander", NodeOptions? nodeOptions = null)
        : base(context, name, nodeOptions)
    {
        _model = model;
        _options = options ?? new CommanderOptions();
        _cmd = CreatePublisher<Twist>(_options.CmdVelTopic);
        _feedback = CreatePublisher<StdString>(_options.FeedbackTopic);
        _goal = CreatePublisher<PoseStamped>(_options.GoalTopic);
        var commands = CreateSubscription<StdString>(_options.CommandTopic, QosProfile.Default);
        _ = Task.Run(() => CommandLoopAsync(commands, _cts.Token));
        if (_options.ScanTopic is { } scanTopic)
        {
            var scans = CreateSubscription<LaserScan>(scanTopic, QosProfile.SensorData.KeepLast(1));
            _ = Task.Run(() => ScanLoopAsync(scans, _cts.Token));
        }
    }

    /// <summary>The tools offered to the model.</summary>
    public static IReadOnlyList<ToolDefinition> Tools { get; } =
    [
        new("move_forward", "Drive straight. Negative distance drives backwards.",
            """{"type":"object","properties":{"distance_m":{"type":"number","description":"meters, negative = backwards"},"speed_mps":{"type":"number"}},"required":["distance_m"]}"""),
        new("rotate", "Rotate in place. Positive = left/counter-clockwise.",
            """{"type":"object","properties":{"angle_deg":{"type":"number"},"angular_speed":{"type":"number","description":"rad/s"}},"required":["angle_deg"]}"""),
        new("go_to_waypoint", "Navigate to a named waypoint.",
            """{"type":"object","properties":{"name":{"type":"string"}},"required":["name"]}"""),
        new("stop", "Stop immediately.", """{"type":"object","properties":{}}"""),
    ];

    /// <summary>Raised with every feedback line (also published on the feedback topic).</summary>
    public event Action<string>? Feedback;

    /// <summary>Interprets and executes one command; returns the feedback lines.</summary>
    public async Task<IReadOnlyList<string>> HandleCommandAsync(string text, CancellationToken cancellationToken = default)
    {
        var lines = new List<string>();
        void Say(string line)
        {
            lines.Add(line);
            Report(line);
        }

        // "stop" must never wait behind a running motion.
        _motion?.Cancel();
        await _busy.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            var waypointList = _options.Waypoints.Count == 0 ? "none" : string.Join(", ", _options.Waypoints.Keys);
            var messages = new[]
            {
                ChatMessage.System(_options.SystemPrompt + " Known waypoints: " + waypointList + "."),
                ChatMessage.User(text),
            };

            ChatResult result;
            try
            {
                result = await _model.CompleteAsync(messages, Tools, cancellationToken).ConfigureAwait(false);
            }
            catch (Exception ex) when (ex is not OperationCanceledException)
            {
                Logger.LogError(ex, "Chat model failed");
                Say("error: language model unavailable (" + ex.Message + ")");
                return lines;
            }

            if (result.ToolCalls.Count == 0)
            {
                Say(result.Content ?? "no action");
                return lines;
            }

            _motion = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, _cts.Token);
            foreach (var call in result.ToolCalls)
            {
                if (_motion.IsCancellationRequested)
                {
                    break;
                }

                Say(await ExecuteAsync(call, _motion.Token).ConfigureAwait(false));
            }
        }
        finally
        {
            _busy.Release();
        }

        return lines;
    }

    private async Task<string> ExecuteAsync(ToolCall call, CancellationToken ct)
    {
        var s = _options.Safety;
        switch (call.Name)
        {
            case "stop":
                _cmd.Publish(new Twist());
                return "stopped";

            case "move_forward":
            {
                var distance = s.ClampDistance(call.GetDouble("distance_m"));
                var speed = Math.Abs(s.ClampLinear(call.GetDouble("speed_mps", _options.DefaultSpeed)));
                if (speed <= 0 || distance == 0)
                {
                    return "nothing to do";
                }

                var v = Math.Sign(distance) * speed;
                var completed = await DriveAsync(v, 0, Math.Abs(distance) / speed, forward: v > 0, ct).ConfigureAwait(false);
                return completed ? $"moved {distance:0.##} m" : "stopped early: obstacle ahead or interrupted";
            }

            case "rotate":
            {
                var deg = s.ClampRotation(call.GetDouble("angle_deg"));
                var w = Math.Abs(s.ClampAngular(call.GetDouble("angular_speed", _options.DefaultAngularSpeed)));
                if (w <= 0 || deg == 0)
                {
                    return "nothing to do";
                }

                var rad = deg * Math.PI / 180;
                var completed = await DriveAsync(0, Math.Sign(rad) * w, Math.Abs(rad) / w, forward: false, ct).ConfigureAwait(false);
                return completed ? $"rotated {deg:0.#}°" : "rotation interrupted";
            }

            case "go_to_waypoint":
            {
                var name = call.GetString("name") ?? string.Empty;
                if (!_options.Waypoints.TryGetValue(name, out var wp))
                {
                    return $"unknown waypoint '{name}'";
                }

                var goal = new PoseStamped();
                goal.Header.FrameId = _options.GoalFrame;
                Clock.Now().CopyTo(goal.Header.Stamp);
                goal.Pose.Position.X = wp.X;
                goal.Pose.Position.Y = wp.Y;
                goal.Pose.Orientation = MessageExtensions.QuaternionFromYaw(wp.Yaw);
                _goal.Publish(goal);
                return $"navigating to {name}";
            }

            default:
                return $"unsupported tool '{call.Name}'";
        }
    }

    /// <summary>Publishes a constant twist for <paramref name="seconds"/>, then zero. Returns false when stopped early.</summary>
    private async Task<bool> DriveAsync(double v, double w, double seconds, bool forward, CancellationToken ct)
    {
        var period = TimeSpan.FromSeconds(1.0 / _options.PublishRateHz);
        var end = DateTime.UtcNow + TimeSpan.FromSeconds(seconds);
        var twist = MessageExtensions.Twist2D(v, w);
        try
        {
            while (DateTime.UtcNow < end)
            {
                if (forward && _options.Safety.StopDistance > 0 && _frontClearance < _options.Safety.StopDistance)
                {
                    return false;
                }

                _cmd.Publish(twist);
                var remaining = end - DateTime.UtcNow;
                await Task.Delay(remaining < period ? (remaining > TimeSpan.Zero ? remaining : TimeSpan.Zero) : period, ct).ConfigureAwait(false);
            }

            return true;
        }
        catch (OperationCanceledException)
        {
            return false;
        }
        finally
        {
            _cmd.Publish(new Twist());
        }
    }

    private void Report(string line)
    {
        Logger.LogInformation("{Feedback}", line);
        _feedback.Publish(new StdString { Data = line });
        Feedback?.Invoke(line);
    }

    private async Task CommandLoopAsync(Subscription<StdString> sub, CancellationToken ct)
    {
        try
        {
            await foreach (var msg in sub.ReadAllAsync(ct).ConfigureAwait(false))
            {
                // Run each command without blocking the reader, so a later "stop" can interrupt it.
                _ = HandleCommandAsync(msg.Data, ct);
            }
        }
        catch (OperationCanceledException)
        {
        }
    }

    private async Task ScanLoopAsync(Subscription<LaserScan> sub, CancellationToken ct)
    {
        try
        {
            await foreach (var scan in sub.ReadAllAsync(ct).ConfigureAwait(false))
            {
                var best = float.PositiveInfinity;
                for (var i = 0; i < scan.Ranges.Length; i++)
                {
                    var a = Simulation.Angles.Normalize(scan.AngleMin + (i * scan.AngleIncrement));
                    var r = scan.Ranges[i];
                    if (Math.Abs(a) <= Math.PI / 6 && float.IsFinite(r) && r >= scan.RangeMin && r < best)
                    {
                        best = r;
                    }
                }

                _frontClearance = best;
            }
        }
        catch (OperationCanceledException)
        {
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _cts.Cancel();
        }

        base.Dispose(disposing);
    }
}
