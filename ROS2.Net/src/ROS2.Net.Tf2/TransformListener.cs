using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.Tf2Msgs;

namespace ROS2.Net.Tf2;

/// <summary>
/// Fills a <see cref="TfBuffer"/> from <c>/tf</c> and <c>/tf_static</c>, like <c>tf2_ros::TransformListener</c>.
/// </summary>
/// <remarks>
/// The subscriptions are callback-less and drained by a dedicated background task, so the buffer is
/// kept current whether or not the node is spinning, and TF traffic never competes with the node's
/// own callbacks on its executor.
/// </remarks>
public sealed class TransformListener : IDisposable
{
    /// <summary>QoS of <c>/tf</c> subscriptions (keep last 100, reliable, volatile).</summary>
    public static QosProfile DynamicQos { get; } = QosProfile.Default.KeepLast(100);

    /// <summary>QoS of <c>/tf_static</c> subscriptions (keep last 100, reliable, transient local).</summary>
    public static QosProfile StaticQos { get; } = QosProfile.Default.KeepLast(100).AsTransientLocal();

    private readonly Subscription<TFMessage> _dynamic;
    private readonly Subscription<TFMessage> _static;
    private readonly CancellationTokenSource _cts = new();
    private readonly Task _pump;

    /// <param name="buffer">Buffer to fill.</param>
    /// <param name="node">Node that owns the subscriptions.</param>
    /// <param name="tfTopic">Dynamic topic, default <c>/tf</c>.</param>
    /// <param name="tfStaticTopic">Static topic, default <c>/tf_static</c>.</param>
    public TransformListener(TfBuffer buffer, Node node, string tfTopic = "/tf", string tfStaticTopic = "/tf_static")
    {
        Buffer = buffer;
        Node = node;
        _dynamic = node.CreateSubscription<TFMessage>(tfTopic, DynamicQos);
        _static = node.CreateSubscription<TFMessage>(tfStaticTopic, StaticQos);
        var token = CancellationTokenSource.CreateLinkedTokenSource(_cts.Token, node.Context.ShutdownToken).Token;
        _pump = Task.WhenAll(PumpAsync(_dynamic, isStatic: false, token), PumpAsync(_static, isStatic: true, token));
    }

    public TfBuffer Buffer { get; }

    public Node Node { get; }

    /// <summary>Transforms received so far.</summary>
    public long ReceivedCount => Interlocked.Read(ref _received);

    private long _received;

    private async Task PumpAsync(Subscription<TFMessage> sub, bool isStatic, CancellationToken token)
    {
        try
        {
            await foreach (var msg in sub.ReadAllAsync(token).ConfigureAwait(false))
            {
                foreach (var t in msg.Transforms)
                {
                    try
                    {
                        Buffer.SetTransform(t, "default_authority", isStatic);
                        Interlocked.Increment(ref _received);
                    }
                    catch (TransformException)
                    {
                        // Malformed transform from a remote publisher: skip it, as tf2_ros logs and continues.
                    }
                }
            }
        }
        catch (OperationCanceledException)
        {
        }
        catch (ObjectDisposedException)
        {
        }
    }

    public void Dispose()
    {
        _cts.Cancel();
        _dynamic.Dispose();
        _static.Dispose();
        try
        {
            _pump.Wait(TimeSpan.FromSeconds(1));
        }
        catch (AggregateException)
        {
        }

        _cts.Dispose();
    }
}

/// <summary>Publishes transforms on <c>/tf</c>, like <c>tf2_ros::TransformBroadcaster</c>.</summary>
public sealed class TransformBroadcaster : IDisposable
{
    private readonly Publisher<TFMessage> _publisher;

    public TransformBroadcaster(Node node, string topic = "/tf", QosProfile? qos = null)
    {
        _publisher = node.CreatePublisher<TFMessage>(topic, qos ?? TransformListener.DynamicQos);
    }

    public void SendTransform(TransformStamped transform) => _publisher.Publish(new TFMessage { Transforms = [transform] });

    public void SendTransforms(IEnumerable<TransformStamped> transforms) => _publisher.Publish(new TFMessage { Transforms = transforms.ToArray() });

    /// <summary>Publishes <paramref name="parent"/> → <paramref name="child"/> stamped <paramref name="stamp"/>.</summary>
    public void SendTransform(string parent, string child, Transform3d transform, RosTime stamp) =>
        SendTransform(StaticTransformBroadcaster.Stamped(parent, child, transform, stamp));

    public void Dispose() => _publisher.Dispose();
}

/// <summary>
/// Publishes latched transforms on <c>/tf_static</c>, like <c>tf2_ros::StaticTransformBroadcaster</c>:
/// every call republishes the accumulated set, so late joiners receive all static frames.
/// </summary>
public sealed class StaticTransformBroadcaster : IDisposable
{
    private readonly Publisher<TFMessage> _publisher;
    private readonly Dictionary<string, TransformStamped> _frames = new(StringComparer.Ordinal);

    public StaticTransformBroadcaster(Node node, string topic = "/tf_static")
    {
        // Depth 1 + transient local: the single latched sample always holds the full set.
        _publisher = node.CreatePublisher<TFMessage>(topic, QosProfile.Latched);
    }

    public IReadOnlyCollection<TransformStamped> Transforms
    {
        get
        {
            lock (_frames)
            {
                return _frames.Values.ToList();
            }
        }
    }

    public void SendTransform(TransformStamped transform) => SendTransforms([transform]);

    public void SendTransforms(IEnumerable<TransformStamped> transforms)
    {
        TFMessage msg;
        lock (_frames)
        {
            foreach (var t in transforms)
            {
                _frames[t.ChildFrameId] = t;
            }

            msg = new TFMessage { Transforms = _frames.Values.ToArray() };
        }

        _publisher.Publish(msg);
    }

    public void SendTransform(string parent, string child, Transform3d transform) =>
        SendTransform(Stamped(parent, child, transform, RosTime.Now()));

    internal static TransformStamped Stamped(string parent, string child, Transform3d transform, RosTime stamp)
    {
        var msg = new TransformStamped { ChildFrameId = child, Transform = transform.ToMsg() };
        msg.Header.FrameId = parent;
        stamp.CopyTo(msg.Header.Stamp);
        return msg;
    }

    public void Dispose() => _publisher.Dispose();
}
