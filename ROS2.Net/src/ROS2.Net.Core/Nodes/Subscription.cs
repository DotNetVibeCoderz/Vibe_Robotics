using System.Runtime.CompilerServices;
using System.Threading.Channels;
using ROS2.Net.Rmw;

namespace ROS2.Net;

/// <summary>Options for <see cref="Node.CreateSubscription{T}(string, Action{T}, QosProfile?, SubscriptionOptions?, CallbackGroup?)"/>.</summary>
public sealed class SubscriptionOptions
{
    /// <summary>Override the context's intra-process setting for this subscription.</summary>
    public bool? UseIntraProcessComms { get; init; }

    /// <summary>Ignore messages published from this context (e.g. a node listening to its own topic).</summary>
    public bool IgnoreLocalPublications { get; init; }

    /// <summary>
    /// Client-side content filter, evaluated on the receiving thread before queueing. Rejected
    /// messages never occupy queue slots or wake the executor.
    /// </summary>
    public Func<IRosMessage, bool>? ContentFilter { get; init; }

    /// <summary>Called for QoS events (deadline missed, lost messages, liveliness, matched).</summary>
    public Action<QosEvent>? EventCallback { get; init; }
}

/// <summary>
/// Receives messages of type <typeparamref name="T"/>. Created with a callback, messages are queued
/// (bounded by QoS depth, oldest dropped first) and handed to the callback by the node's executor.
/// Created without one, messages flow into <see cref="ReadAllAsync"/> and no executor is needed.
/// </summary>
public sealed class Subscription<T> : Waitable
    where T : class, IRosMessage<T>, new()
{
    private readonly Func<T, MessageInfo, ValueTask>? _callback;
    private readonly Queue<(T Message, MessageInfo Info)> _queue;
    private readonly Channel<T>? _channel;
    private readonly int _capacity;
    private readonly IRmwSubscription _rmw;
    private readonly IntraProcessManager.LocalSubscription? _local;
    private readonly IntraProcessManager? _intra;
    private readonly Func<IRosMessage, bool>? _filter;
    private readonly Action<QosEvent>? _eventCallback;
    private readonly Timer? _deadlineTimer;
    private readonly TimeSpan? _lifespan;
    private long _received;
    private long _lastReceivedCount;
    private int _lostTotal;
    private int _deadlineMissedTotal;

    internal Subscription(Node node, string topic, QosProfile qos, Func<T, MessageInfo, ValueTask>? callback, SubscriptionOptions? options, CallbackGroup? group)
        : base(node, group)
    {
        TopicName = topic;
        Qos = qos;
        _callback = callback;
        _capacity = qos.Capacity;
        _queue = new Queue<(T, MessageInfo)>(Math.Min(_capacity, 64));
        _filter = options?.ContentFilter;
        _eventCallback = options?.EventCallback;
        _lifespan = qos.Lifespan;

        if (callback is null)
        {
            _channel = Channel.CreateBounded<T>(new BoundedChannelOptions(Math.Min(_capacity, 100_000))
            {
                FullMode = BoundedChannelFullMode.DropOldest,
                SingleWriter = false,
            }, _ => CountLost());
        }

        var useIntra = (options?.UseIntraProcessComms ?? node.Context.UseIntraProcessComms) && node.Context.IntraProcess is not null;
        var ignoreLocal = options?.IgnoreLocalPublications == true || useIntra;
        if (useIntra)
        {
            _intra = node.Context.IntraProcess!;
            if (!options?.IgnoreLocalPublications ?? true)
            {
                _local = _intra.Add(topic, T.TypeName, qos, (m, info) => OnMessage((T)m, info));
            }
        }

        var spec = new RmwTopicSpec(topic, MessageTypeInfo.Of<T>(), qos, new RmwEndpointOptions { IgnoreLocalPublications = ignoreLocal });
        _rmw = node.RmwNode.CreateSubscription<T>(spec, OnMessage);
        _rmw.QosEventRaised += RaiseEvent;

        if (qos.Deadline is { } deadline)
        {
            _deadlineTimer = new Timer(_ => CheckDeadline(), null, deadline, deadline);
        }
    }

    /// <summary>Fully qualified topic name.</summary>
    public string TopicName { get; }

    public QosProfile Qos { get; }

    public Gid Gid => _rmw.Gid;

    /// <summary>Matched publishers on the middleware (intra-process publishers are not counted).</summary>
    public int PublisherCount => _rmw.MatchedCount;

    /// <summary>Total messages received since creation.</summary>
    public long ReceivedCount => Interlocked.Read(ref _received);

    /// <summary>Raised for QoS events on this subscription.</summary>
    public event Action<QosEvent>? QosEventRaised;

    protected internal override bool HasWork
    {
        get
        {
            lock (_queue)
            {
                return _queue.Count > 0;
            }
        }
    }

    /// <summary>
    /// Streams received messages. Only available on subscriptions created without a callback.
    /// Messages are buffered up to the QoS depth; the oldest are dropped when the reader falls behind.
    /// </summary>
    public async IAsyncEnumerable<T> ReadAllAsync([EnumeratorCancellation] CancellationToken cancellationToken = default)
    {
        if (_channel is null)
        {
            throw new InvalidOperationException("ReadAllAsync is only available on subscriptions created without a callback.");
        }

        using var linked = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, Node.Context.ShutdownToken);
        while (true)
        {
            T item;
            try
            {
                if (!await _channel.Reader.WaitToReadAsync(linked.Token).ConfigureAwait(false))
                {
                    yield break;
                }

                if (!_channel.Reader.TryRead(out item!))
                {
                    continue;
                }
            }
            catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
            {
                yield break; // Context shut down.
            }

            yield return item;
        }
    }

    /// <summary>Waits for the next message. Only available on subscriptions created without a callback.</summary>
    public async ValueTask<T> TakeAsync(CancellationToken cancellationToken = default)
    {
        if (_channel is null)
        {
            throw new InvalidOperationException("TakeAsync is only available on subscriptions created without a callback.");
        }

        return await _channel.Reader.ReadAsync(cancellationToken).ConfigureAwait(false);
    }

    /// <summary>Takes a buffered message without waiting. Only available on subscriptions created without a callback.</summary>
    public bool TryTake([System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out T? message)
    {
        if (_channel is null)
        {
            throw new InvalidOperationException("TryTake is only available on subscriptions created without a callback.");
        }

        return _channel.Reader.TryRead(out message);
    }

    private void OnMessage(T message, MessageInfo info)
    {
        if (IsDisposed || (_filter is not null && !_filter(message)))
        {
            return;
        }

        Interlocked.Increment(ref _received);
        if (_channel is not null)
        {
            _channel.Writer.TryWrite(message);
            return;
        }

        var lost = false;
        lock (_queue)
        {
            if (_queue.Count >= _capacity)
            {
                _queue.Dequeue();
                lost = true;
            }

            _queue.Enqueue((message, info));
        }

        if (lost)
        {
            CountLost();
        }

        Signal();
    }

    protected internal override ValueTask ExecuteNextAsync()
    {
        (T Message, MessageInfo Info) item;
        bool more;
        lock (_queue)
        {
            if (!_queue.TryDequeue(out item))
            {
                return ValueTask.CompletedTask;
            }

            more = _queue.Count > 0;
        }

        if (more)
        {
            Signal();
        }

        if (_lifespan is { } lifespan && !item.Info.FromIntraProcess &&
            RosTime.Now() - item.Info.SourceTimestamp > RosDuration.FromTimeSpan(lifespan))
        {
            return ValueTask.CompletedTask; // Expired while queued.
        }

        return _callback!(item.Message, item.Info);
    }

    private void CountLost()
    {
        var total = Interlocked.Increment(ref _lostTotal);
        RaiseEvent(new QosEvent(QosEventKind.MessageLost, total, 1));
    }

    private void CheckDeadline()
    {
        var received = Interlocked.Read(ref _received);
        if (received == Interlocked.Exchange(ref _lastReceivedCount, received))
        {
            var total = Interlocked.Increment(ref _deadlineMissedTotal);
            RaiseEvent(new QosEvent(QosEventKind.RequestedDeadlineMissed, total, 1));
        }
    }

    private void RaiseEvent(QosEvent e)
    {
        _eventCallback?.Invoke(e);
        QosEventRaised?.Invoke(e);
    }

    protected override void DisposeCore()
    {
        _deadlineTimer?.Dispose();
        if (_local is not null)
        {
            _intra!.Remove(_local);
        }

        _rmw.QosEventRaised -= RaiseEvent;
        _rmw.Dispose();
        _channel?.Writer.TryComplete();
    }
}

/// <summary>Receives raw CDR for a topic (bag recording, introspection). The handler runs on the middleware thread.</summary>
public sealed class SerializedSubscription : IDisposable
{
    private readonly IRmwSubscription _rmw;
    private readonly Node _node;

    internal SerializedSubscription(Node node, string topic, MessageTypeInfo type, QosProfile qos, RmwSerializedHandler handler)
    {
        _node = node;
        TopicName = topic;
        Type = type;
        _rmw = node.RmwNode.CreateSerializedSubscription(new RmwTopicSpec(topic, type, qos, RmwEndpointOptions.Default), handler);
    }

    public string TopicName { get; }

    public MessageTypeInfo Type { get; }

    public int PublisherCount => _rmw.MatchedCount;

    public void Dispose()
    {
        _rmw.Dispose();
        _node.Untrack(this);
    }
}
