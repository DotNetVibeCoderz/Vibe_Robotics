using System.Buffers;
using System.Text.RegularExpressions;
using System.Threading.Channels;
using Microsoft.Extensions.Logging;
using ROS2.Net.Rmw;

namespace ROS2.Net.Bag;

/// <summary>Options for <see cref="BagRecorder"/>.</summary>
public sealed class BagRecorderOptions
{
    /// <summary>Topics to record (absolute or relative to the node). Ignored when <see cref="All"/> is set.</summary>
    public List<string> Topics { get; } = [];

    /// <summary>Record every discovered topic (filtered by <see cref="Include"/>/<see cref="Exclude"/>).</summary>
    public bool All { get; set; }

    /// <summary>Regex a topic must match to be recorded (like <c>ros2 bag record -e</c>).</summary>
    public string? Include { get; set; }

    /// <summary>Regex of topics never to record (like <c>ros2 bag record -x</c>).</summary>
    public string? Exclude { get; set; }

    /// <summary>Also record hidden topics (a name token starting with <c>_</c>).</summary>
    public bool IncludeHidden { get; set; }

    /// <summary>Types for topics that may not have a publisher yet, so recording starts immediately.</summary>
    public Dictionary<string, string> TopicTypes { get; } = new(StringComparer.Ordinal);

    /// <summary>How often the graph is polled for new topics (the graph-changed event also wakes discovery).</summary>
    public TimeSpan DiscoveryInterval { get; set; } = TimeSpan.FromMilliseconds(100);

    /// <summary>Use this QoS for every subscription instead of adapting to the publishers.</summary>
    public QosProfile? QosOverride { get; set; }

    public BagWriterOptions Writer { get; set; } = new();
}

/// <summary>
/// Records topics into a rosbag2 MCAP bag, like <c>ros2 bag record</c>. Messages are received as raw
/// CDR (no deserialization), copied into pooled buffers and written by a background task, so the
/// middleware's receive threads never wait on disk I/O.
/// </summary>
public sealed class BagRecorder : IAsyncDisposable
{
    private readonly Node _node;
    private readonly BagRecorderOptions _options;
    private readonly BagWriter _writer;
    private readonly Regex? _include;
    private readonly Regex? _exclude;
    private readonly Dictionary<string, SerializedSubscription> _subscriptions = new(StringComparer.Ordinal);
    private readonly HashSet<string> _explicitTopics;
    private readonly Channel<Pending> _queue = Channel.CreateUnbounded<Pending>(new UnboundedChannelOptions { SingleReader = true });
    private readonly CancellationTokenSource _cts = new();
    private Task? _discovery;
    private Task? _drain;
    private int _stopped;

    private readonly record struct Pending(string Topic, byte[] Buffer, int Length, RosTime Received, RosTime Published);

    public BagRecorder(Node node, string bagDirectory, BagRecorderOptions? options = null)
    {
        _node = node;
        _options = options ?? new BagRecorderOptions();
        _writer = new BagWriter(bagDirectory, _options.Writer);
        _include = _options.Include is { Length: > 0 } inc ? new Regex(inc, RegexOptions.Compiled) : null;
        _exclude = _options.Exclude is { Length: > 0 } exc ? new Regex(exc, RegexOptions.Compiled) : null;
        _explicitTopics = _options.Topics.Select(t => node.ResolveTopicName(t)).ToHashSet(StringComparer.Ordinal);
    }

    public string BagDirectory => _writer.Directory;

    public ulong MessageCount => _writer.MessageCount;

    public IReadOnlyList<string> RecordedTopics
    {
        get
        {
            lock (_subscriptions)
            {
                return _subscriptions.Keys.Order(StringComparer.Ordinal).ToList();
            }
        }
    }

    /// <summary>Raised when a topic starts being recorded.</summary>
    public event Action<string, string>? TopicAdded;

    /// <summary>Starts discovery and recording.</summary>
    public void Start()
    {
        foreach (var (topic, type) in _options.TopicTypes)
        {
            TrySubscribe(_node.ResolveTopicName(topic), type, offered: []);
        }

        Discover();
        _drain = Task.Run(DrainAsync);
        _discovery = Task.Run(DiscoveryLoopAsync);
    }

    /// <summary>Waits until <paramref name="count"/> topics are being recorded.</summary>
    public async Task<bool> WaitForTopicsAsync(int count, TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        var deadline = Environment.TickCount64 + (long)timeout.TotalMilliseconds;
        while (RecordedTopics.Count < count)
        {
            if (Environment.TickCount64 > deadline)
            {
                return false;
            }

            await Task.Delay(10, cancellationToken).ConfigureAwait(false);
        }

        return true;
    }

    private async Task DiscoveryLoopAsync()
    {
        while (!_cts.IsCancellationRequested)
        {
            try
            {
                await _node.WaitForGraphChangeAsync(_options.DiscoveryInterval, _cts.Token).ConfigureAwait(false);
                Discover();
            }
            catch (OperationCanceledException)
            {
                return;
            }
            catch (Exception ex) when (ex is not OutOfMemoryException)
            {
                _node.Logger.LogWarning(ex, "Bag topic discovery failed");
            }
        }
    }

    private void Discover()
    {
        foreach (var topic in _node.GetTopicNamesAndTypes(includeHidden: _options.IncludeHidden))
        {
            if (!Wanted(topic.Name) || topic.Types.Count == 0)
            {
                continue;
            }

            lock (_subscriptions)
            {
                if (_subscriptions.ContainsKey(topic.Name))
                {
                    continue;
                }
            }

            var publishers = _node.GetPublishersInfo(topic.Name);
            if (publishers.Count == 0)
            {
                continue; // Only subscribers so far: wait until someone publishes, like rosbag2.
            }

            TrySubscribe(topic.Name, publishers[0].TypeName, publishers.Select(p => p.Qos).ToList());
        }
    }

    private bool Wanted(string topic)
    {
        if (!_options.All && !_explicitTopics.Contains(topic) && _include is null)
        {
            return false;
        }

        if (_include is not null && !_include.IsMatch(topic) && !_explicitTopics.Contains(topic))
        {
            return false;
        }

        return _exclude is null || !_exclude.IsMatch(topic);
    }

    private void TrySubscribe(string topic, string type, IReadOnlyList<QosProfile> offered)
    {
        lock (_subscriptions)
        {
            if (_subscriptions.ContainsKey(topic))
            {
                return;
            }

            var info = RosTypeRegistry.TryGetMessage(type, out var known)
                ? known
                : MessageTypeInfo.Create(type, typeof(IRosMessage), () => throw new NotSupportedException($"Type '{type}' is not loaded."));
            var qos = _options.QosOverride ?? QosYaml.Adapt(offered);
            _writer.AddTopic(topic, info.Name, offered.Count > 0 ? offered : [qos]);
            _subscriptions[topic] = _node.CreateSerializedSubscription(topic, info, (cdr, mi) => OnMessage(topic, cdr, mi), qos);
        }

        _node.Logger.LogInformation("Recording {Topic} [{Type}]", topic, type);
        TopicAdded?.Invoke(topic, type);
    }

    private void OnMessage(string topic, ReadOnlySpan<byte> cdr, MessageInfo info)
    {
        if (_cts.IsCancellationRequested)
        {
            return;
        }

        var buffer = ArrayPool<byte>.Shared.Rent(cdr.Length);
        cdr.CopyTo(buffer);
        var received = RosTime.Now();
        var published = info.SourceTimestamp.IsZero ? received : info.SourceTimestamp;
        if (!_queue.Writer.TryWrite(new Pending(topic, buffer, cdr.Length, received, published)))
        {
            ArrayPool<byte>.Shared.Return(buffer);
        }
    }

    private async Task DrainAsync()
    {
        await foreach (var p in _queue.Reader.ReadAllAsync().ConfigureAwait(false))
        {
            try
            {
                _writer.Write(p.Topic, p.Buffer.AsSpan(0, p.Length), p.Received, p.Published);
            }
            catch (Exception ex) when (ex is not OutOfMemoryException)
            {
                _node.Logger.LogError(ex, "Failed to write a message of {Topic} to the bag", p.Topic);
            }
            finally
            {
                ArrayPool<byte>.Shared.Return(p.Buffer);
            }
        }
    }

    /// <summary>Stops recording, flushes queued messages and finalizes the bag (metadata.yaml).</summary>
    public async Task StopAsync()
    {
        if (Interlocked.Exchange(ref _stopped, 1) != 0)
        {
            return;
        }

        _cts.Cancel();
        lock (_subscriptions)
        {
            foreach (var s in _subscriptions.Values)
            {
                s.Dispose();
            }
        }

        _queue.Writer.TryComplete();
        if (_discovery is not null)
        {
            await _discovery.ConfigureAwait(false);
        }

        if (_drain is not null)
        {
            await _drain.ConfigureAwait(false);
        }

        _writer.Close();
    }

    public async ValueTask DisposeAsync()
    {
        await StopAsync().ConfigureAwait(false);
        _cts.Dispose();
    }
}
