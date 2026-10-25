namespace ROS2.Net;

/// <summary>How many samples are kept for late or slow readers.</summary>
public enum HistoryPolicy
{
    SystemDefault,
    /// <summary>Keep up to <see cref="QosProfile.Depth"/> samples.</summary>
    KeepLast,
    /// <summary>Keep every sample, bounded only by resource limits.</summary>
    KeepAll,
}

/// <summary>Delivery guarantee.</summary>
public enum ReliabilityPolicy
{
    SystemDefault,
    /// <summary>Retransmit until delivered (or history overflows).</summary>
    Reliable,
    /// <summary>Send once; drops are acceptable (sensor streams).</summary>
    BestEffort,
}

/// <summary>Whether samples outlive their publication for late-joining subscriptions.</summary>
public enum DurabilityPolicy
{
    SystemDefault,
    /// <summary>The publisher keeps its last <see cref="QosProfile.Depth"/> samples for late joiners ("latched").</summary>
    TransientLocal,
    /// <summary>Only subscriptions that exist at publication time receive the sample.</summary>
    Volatile,
}

/// <summary>How a publisher asserts that it is alive.</summary>
public enum LivelinessPolicy
{
    SystemDefault,
    /// <summary>The middleware asserts liveliness automatically.</summary>
    Automatic,
    /// <summary>The application asserts liveliness by publishing or calling <c>AssertLiveliness</c>.</summary>
    ManualByTopic,
}

/// <summary>
/// ROS 2 Quality of Service profile. <see langword="null"/> durations mean "infinite / unspecified",
/// as in <c>rmw_qos_profile_t</c>.
/// </summary>
public readonly record struct QosProfile
{
    public QosProfile(int depth)
        : this()
    {
        History = HistoryPolicy.KeepLast;
        Depth = depth;
        Reliability = ReliabilityPolicy.Reliable;
        Durability = DurabilityPolicy.Volatile;
        Liveliness = LivelinessPolicy.SystemDefault;
    }

    public HistoryPolicy History { get; init; }

    public int Depth { get; init; }

    public ReliabilityPolicy Reliability { get; init; }

    public DurabilityPolicy Durability { get; init; }

    /// <summary>Maximum expected period between samples.</summary>
    public TimeSpan? Deadline { get; init; }

    /// <summary>Age after which an undelivered sample is discarded.</summary>
    public TimeSpan? Lifespan { get; init; }

    public LivelinessPolicy Liveliness { get; init; }

    public TimeSpan? LivelinessLeaseDuration { get; init; }

    /// <summary>Use the topic name verbatim on the wire, without the <c>rt/</c> prefix.</summary>
    public bool AvoidRosNamespaceConventions { get; init; }

    /// <summary><c>rmw_qos_profile_default</c>: keep last 10, reliable, volatile.</summary>
    public static QosProfile Default { get; } = new(10);

    /// <summary><c>rmw_qos_profile_sensor_data</c>: keep last 5, best effort, volatile.</summary>
    public static QosProfile SensorData { get; } = new(5) { Reliability = ReliabilityPolicy.BestEffort };

    /// <summary><c>rmw_qos_profile_services_default</c>.</summary>
    public static QosProfile ServicesDefault { get; } = new(10);

    /// <summary><c>rmw_qos_profile_parameters</c>: keep last 1000.</summary>
    public static QosProfile Parameters { get; } = new(1000);

    /// <summary><c>rmw_qos_profile_parameter_events</c>.</summary>
    public static QosProfile ParameterEvents { get; } = new(1000);

    /// <summary>Latched topics such as <c>/tf_static</c>, <c>/map</c> or <c>/robot_description</c>: keep last 1, reliable, transient local.</summary>
    public static QosProfile Latched { get; } = new(1) { Durability = DurabilityPolicy.TransientLocal };

    /// <summary>The QoS rclcpp uses for <c>/rosout</c>.</summary>
    public static QosProfile Rosout { get; } = new(1000)
    {
        Durability = DurabilityPolicy.TransientLocal,
        Lifespan = TimeSpan.FromSeconds(10),
    };

    /// <summary>The QoS of action status topics.</summary>
    public static QosProfile ActionStatus { get; } = new(1) { Durability = DurabilityPolicy.TransientLocal };

    /// <summary><c>rmw_qos_profile_system_default</c>: every policy left to the middleware.</summary>
    public static QosProfile SystemDefault { get; } = new QosProfile(0) with
    {
        History = HistoryPolicy.SystemDefault,
        Reliability = ReliabilityPolicy.SystemDefault,
        Durability = DurabilityPolicy.SystemDefault,
    };

    public QosProfile KeepLast(int depth) => this with { History = HistoryPolicy.KeepLast, Depth = depth };

    public QosProfile KeepAll() => this with { History = HistoryPolicy.KeepAll };

    public QosProfile AsReliable() => this with { Reliability = ReliabilityPolicy.Reliable };

    public QosProfile AsBestEffort() => this with { Reliability = ReliabilityPolicy.BestEffort };

    public QosProfile AsTransientLocal() => this with { Durability = DurabilityPolicy.TransientLocal };

    public QosProfile AsVolatile() => this with { Durability = DurabilityPolicy.Volatile };

    public QosProfile WithDeadline(TimeSpan? deadline) => this with { Deadline = deadline };

    public QosProfile WithLifespan(TimeSpan? lifespan) => this with { Lifespan = lifespan };

    public QosProfile WithLiveliness(LivelinessPolicy policy, TimeSpan? lease = null) =>
        this with { Liveliness = policy, LivelinessLeaseDuration = lease };

    /// <summary>Resolves <c>SystemDefault</c> policies to what the built-in middlewares use.</summary>
    public QosProfile Resolve() => this with
    {
        History = History == HistoryPolicy.SystemDefault ? HistoryPolicy.KeepLast : History,
        Depth = History == HistoryPolicy.KeepAll ? int.MaxValue : Depth <= 0 ? 10 : Depth,
        Reliability = Reliability == ReliabilityPolicy.SystemDefault ? ReliabilityPolicy.Reliable : Reliability,
        Durability = Durability == DurabilityPolicy.SystemDefault ? DurabilityPolicy.Volatile : Durability,
        Liveliness = Liveliness == LivelinessPolicy.SystemDefault ? LivelinessPolicy.Automatic : Liveliness,
    };

    /// <summary>Effective queue capacity: <see cref="Depth"/> for keep-last, effectively unbounded for keep-all.</summary>
    public int Capacity => History == HistoryPolicy.KeepAll ? int.MaxValue : Math.Max(1, Depth <= 0 ? 10 : Depth);

    public override string ToString() =>
        $"{History}({Depth}), {Reliability}, {Durability}" +
        (Deadline is { } d ? $", deadline {d.TotalMilliseconds}ms" : string.Empty) +
        (Lifespan is { } l ? $", lifespan {l.TotalMilliseconds}ms" : string.Empty);
}

/// <summary>Outcome of matching an offered (publisher) QoS against a requested (subscription) QoS.</summary>
public enum QosCompatibility
{
    Compatible,
    /// <summary>Probably works, but depends on the remote middleware's defaults.</summary>
    Warning,
    /// <summary>The endpoints will never match.</summary>
    Error,
}

/// <summary>QoS request/offer matching rules, as enforced by DDS and reported by <c>rmw_qos_profile_check_compatible</c>.</summary>
public static class QosMatching
{
    /// <summary>Checks whether a publisher with <paramref name="offered"/> QoS can serve a subscription requesting <paramref name="requested"/>.</summary>
    public static QosCompatibility Check(QosProfile offered, QosProfile requested, out string reason)
    {
        var o = offered.Resolve();
        var r = requested.Resolve();
        var warning = offered.Reliability == ReliabilityPolicy.SystemDefault || requested.Reliability == ReliabilityPolicy.SystemDefault ||
                      offered.Durability == DurabilityPolicy.SystemDefault || requested.Durability == DurabilityPolicy.SystemDefault;

        if (o.Reliability == ReliabilityPolicy.BestEffort && r.Reliability == ReliabilityPolicy.Reliable)
        {
            reason = "ERROR: Best effort publisher and reliable subscription;";
            return QosCompatibility.Error;
        }

        if (o.Durability == DurabilityPolicy.Volatile && r.Durability == DurabilityPolicy.TransientLocal)
        {
            reason = "ERROR: Volatile publisher and transient local subscription;";
            return QosCompatibility.Error;
        }

        if (r.Deadline is { } rd && (o.Deadline is null || o.Deadline > rd))
        {
            reason = "ERROR: Subscription deadline is less than publisher deadline;";
            return QosCompatibility.Error;
        }

        if (o.Liveliness == LivelinessPolicy.Automatic && r.Liveliness == LivelinessPolicy.ManualByTopic)
        {
            reason = "ERROR: Publisher's liveliness is automatic and subscription's is manual by topic;";
            return QosCompatibility.Error;
        }

        if (r.LivelinessLeaseDuration is { } rl && (o.LivelinessLeaseDuration is null || o.LivelinessLeaseDuration > rl))
        {
            reason = "ERROR: Subscription liveliness lease duration is less than publisher;";
            return QosCompatibility.Error;
        }

        reason = warning ? "WARNING: a policy is SystemDefault, compatibility depends on the middleware;" : string.Empty;
        return warning ? QosCompatibility.Warning : QosCompatibility.Compatible;
    }

    /// <summary>True when the endpoints match (warnings count as matching).</summary>
    public static bool IsCompatible(QosProfile offered, QosProfile requested) =>
        Check(offered, requested, out _) != QosCompatibility.Error;
}
