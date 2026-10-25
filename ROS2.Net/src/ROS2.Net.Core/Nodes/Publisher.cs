using ROS2.Net.Rmw;

namespace ROS2.Net;

/// <summary>Options for <see cref="Node.CreatePublisher{T}"/>.</summary>
public sealed class PublisherOptions
{
    /// <summary>Override the context's intra-process setting for this publisher.</summary>
    public bool? UseIntraProcessComms { get; init; }

    /// <summary>Called for QoS events (deadline missed, incompatible QoS, matched).</summary>
    public Action<QosEvent>? EventCallback { get; init; }
}

/// <summary>Publishes messages of type <typeparamref name="T"/> on a topic.</summary>
public class Publisher<T> : IDisposable
    where T : class, IRosMessage<T>, new()
{
    private readonly IRmwPublisher<T> _rmw;
    private readonly IntraProcessManager? _intra;
    private readonly Timer? _deadlineTimer;
    private readonly Action<QosEvent>? _eventCallback;
    private long _sequence;
    private long _lastPublishTicks;
    private int _deadlineMissedTotal;
    private int _disposed;

    internal Publisher(Node node, string topic, QosProfile qos, PublisherOptions? options)
    {
        Node = node;
        TopicName = topic;
        Qos = qos;
        _eventCallback = options?.EventCallback;

        var useIntra = options?.UseIntraProcessComms ?? node.Context.UseIntraProcessComms;
        _intra = useIntra ? node.Context.IntraProcess : null;

        var spec = new RmwTopicSpec(topic, MessageTypeInfo.Of<T>(), qos, RmwEndpointOptions.Default);
        _rmw = node.RmwNode.CreatePublisher<T>(spec);
        _rmw.QosEventRaised += RaiseEvent;

        if (qos.Deadline is { } deadline)
        {
            _lastPublishTicks = Environment.TickCount64;
            _deadlineTimer = new Timer(_ => CheckDeadline(deadline), null, deadline, deadline);
        }
    }

    public Node Node { get; }

    /// <summary>Fully qualified topic name.</summary>
    public string TopicName { get; }

    public QosProfile Qos { get; }

    public Gid Gid => _rmw.Gid;

    /// <summary>Matched subscriptions, remote and intra-process.</summary>
    public int SubscriptionCount =>
        _rmw.MatchedCount + (_intra?.Count(TopicName, T.TypeName, Qos) ?? 0);

    /// <summary>Raised for QoS events on this publisher.</summary>
    public event Action<QosEvent>? QosEventRaised;

    /// <summary>Publishes a message. With intra-process comms, local subscribers receive this exact instance: do not mutate it afterwards.</summary>
    public virtual void Publish(T message)
    {
        ArgumentNullException.ThrowIfNull(message);
        ObjectDisposedException.ThrowIf(Volatile.Read(ref _disposed) != 0, this);
        var seq = Interlocked.Increment(ref _sequence);
        _intra?.Publish(TopicName, T.TypeName, Qos, message, _rmw.Gid, seq);
        _rmw.Publish(message);
        Volatile.Write(ref _lastPublishTicks, Environment.TickCount64);
    }

    /// <summary>Asserts liveliness for <see cref="LivelinessPolicy.ManualByTopic"/> publishers without publishing.</summary>
    public void AssertLiveliness() => _rmw.AssertLiveliness();

    /// <summary>Waits until at least <paramref name="count"/> subscriptions have matched.</summary>
    public async Task<bool> WaitForSubscriptionsAsync(int count = 1, TimeSpan? timeout = null, CancellationToken cancellationToken = default)
    {
        var deadline = timeout is { } t ? DateTime.UtcNow + t : DateTime.MaxValue;
        while (SubscriptionCount < count)
        {
            if (DateTime.UtcNow >= deadline)
            {
                return false;
            }

            await Task.Delay(20, cancellationToken).ConfigureAwait(false);
        }

        return true;
    }

    private void CheckDeadline(TimeSpan deadline)
    {
        if (Environment.TickCount64 - Volatile.Read(ref _lastPublishTicks) > deadline.TotalMilliseconds)
        {
            var total = Interlocked.Increment(ref _deadlineMissedTotal);
            RaiseEvent(new QosEvent(QosEventKind.OfferedDeadlineMissed, total, 1));
        }
    }

    private void RaiseEvent(QosEvent e)
    {
        _eventCallback?.Invoke(e);
        QosEventRaised?.Invoke(e);
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
        {
            _deadlineTimer?.Dispose();
            _rmw.QosEventRaised -= RaiseEvent;
            _rmw.Dispose();
            Node.Untrack(this);
        }

        GC.SuppressFinalize(this);
    }
}

/// <summary>Publishes pre-serialized CDR (bag playback, bridges, relays).</summary>
public sealed class SerializedPublisher : IDisposable
{
    private readonly IRmwSerializedPublisher _rmw;
    private readonly Node _node;

    internal SerializedPublisher(Node node, string topic, MessageTypeInfo type, QosProfile qos)
    {
        _node = node;
        TopicName = topic;
        Type = type;
        _rmw = node.RmwNode.CreateSerializedPublisher(new RmwTopicSpec(topic, type, qos, RmwEndpointOptions.Default));
    }

    public string TopicName { get; }

    public MessageTypeInfo Type { get; }

    public int SubscriptionCount => _rmw.MatchedCount;

    /// <summary>Publishes CDR bytes, encapsulation header included.</summary>
    public void Publish(ReadOnlySpan<byte> cdr) => _rmw.Publish(cdr);

    public void Dispose()
    {
        _rmw.Dispose();
        _node.Untrack(this);
    }
}
