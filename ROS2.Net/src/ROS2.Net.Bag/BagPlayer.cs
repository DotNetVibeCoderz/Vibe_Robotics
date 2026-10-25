using System.Diagnostics;
using ROS2.Net.Msgs.RosgraphMsgs;

namespace ROS2.Net.Bag;

/// <summary>Options for <see cref="BagPlayer"/>.</summary>
public sealed class BagPlayerOptions
{
    /// <summary>Playback speed factor (2 = twice as fast).</summary>
    public double Rate { get; set; } = 1.0;

    /// <summary>Restart from the beginning when the end is reached.</summary>
    public bool Loop { get; set; }

    /// <summary>Skip this much of the bag's beginning.</summary>
    public TimeSpan StartOffset { get; set; }

    /// <summary>Only play these topics (all when empty).</summary>
    public List<string> Topics { get; } = [];

    /// <summary>Topic remapping applied when publishing (<c>from → to</c>).</summary>
    public Dictionary<string, string> Remappings { get; } = new(StringComparer.Ordinal);

    /// <summary>Publish <c>/clock</c> at this rate in Hz (0 = do not publish).</summary>
    public double ClockRate { get; set; }

    /// <summary>Start paused (use <see cref="BagPlayer.Resume"/> or <see cref="BagPlayer.StepAsync"/>).</summary>
    public bool StartPaused { get; set; }

    /// <summary>Delay before the first message, letting subscribers discover the publishers.</summary>
    public TimeSpan Delay { get; set; }

    /// <summary>Per-topic QoS overrides; otherwise the recorded offered QoS is used.</summary>
    public Dictionary<string, QosProfile> QosOverrides { get; } = new(StringComparer.Ordinal);
}

/// <summary>
/// Replays a bag, like <c>ros2 bag play</c>: original inter-message timing scaled by
/// <see cref="Rate"/>, with pause/resume/step, looping, remapping, recorded QoS (latched topics stay
/// latched) and an optional <c>/clock</c> so <c>use_sim_time</c> nodes follow bag time.
/// </summary>
public sealed class BagPlayer : IDisposable
{
    private readonly Node _node;
    private readonly BagReader _reader;
    private readonly BagPlayerOptions _options;
    private readonly Dictionary<string, SerializedPublisher> _publishers = new(StringComparer.Ordinal);
    private readonly Publisher<Clock>? _clock;
    private readonly object _gate = new();
    private readonly SemaphoreSlim _step = new(0);
    private TaskCompletionSource _resume = new(TaskCreationOptions.RunContinuationsAsynchronously);
    private TaskCompletionSource<bool>? _stepDone;
    private long _anchorWall;
    private long _anchorBag;
    private double _rate;
    private bool _paused;
    private long _played;
    private bool _disposed;

    public BagPlayer(Node node, string bagPath, BagPlayerOptions? options = null)
        : this(node, BagReader.Open(bagPath), options)
    {
    }

    public BagPlayer(Node node, BagReader reader, BagPlayerOptions? options = null)
    {
        _node = node;
        _reader = reader;
        _options = options ?? new BagPlayerOptions();
        _rate = _options.Rate > 0 ? _options.Rate : 1.0;
        _paused = _options.StartPaused;
        if (!_paused)
        {
            _resume.TrySetResult();
        }

        foreach (var topic in reader.Topics)
        {
            if (_options.Topics.Count > 0 && !_options.Topics.Contains(topic.Name))
            {
                continue;
            }

            var info = RosTypeRegistry.TryGetMessage(topic.Type, out var known)
                ? known
                : MessageTypeInfo.Create(topic.Type, typeof(IRosMessage), () => throw new NotSupportedException($"Type '{topic.Type}' is not loaded."));
            var qos = _options.QosOverrides.TryGetValue(topic.Name, out var o) ? o : topic.PlaybackQos;
            var name = _options.Remappings.GetValueOrDefault(topic.Name, topic.Name);
            _publishers[topic.Name] = node.CreateSerializedPublisher(name, info, qos);
        }

        if (_options.ClockRate > 0)
        {
            _clock = node.CreatePublisher<Clock>("/clock", QosProfile.Default.KeepLast(1));
        }
    }

    public BagReader Reader => _reader;

    /// <summary>Messages published so far.</summary>
    public long PlayedCount => Interlocked.Read(ref _played);

    public bool IsPaused
    {
        get
        {
            lock (_gate)
            {
                return _paused;
            }
        }
    }

    /// <summary>Playback speed; changing it takes effect immediately.</summary>
    public double Rate
    {
        get
        {
            lock (_gate)
            {
                return _rate;
            }
        }

        set
        {
            if (value <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(value), "Rate must be positive.");
            }

            lock (_gate)
            {
                Reanchor(CurrentBagTimeLocked());
                _rate = value;
            }
        }
    }

    /// <summary>The bag time being played.</summary>
    public RosTime PlaybackTime
    {
        get
        {
            lock (_gate)
            {
                return new RosTime(CurrentBagTimeLocked());
            }
        }
    }

    /// <summary>Raised after each published message.</summary>
    public event Action<BagMessage>? MessagePlayed;

    public void Pause()
    {
        lock (_gate)
        {
            if (_paused)
            {
                return;
            }

            Reanchor(CurrentBagTimeLocked());
            _paused = true;
            _resume = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        }
    }

    public void Resume()
    {
        lock (_gate)
        {
            if (!_paused)
            {
                return;
            }

            _paused = false;
            _anchorWall = Stopwatch.GetTimestamp();
            _resume.TrySetResult();
        }
    }

    public void TogglePause()
    {
        if (IsPaused)
        {
            Resume();
        }
        else
        {
            Pause();
        }
    }

    /// <summary>While paused, publishes the next message immediately. Returns false at the end of the bag or when not paused.</summary>
    public Task<bool> StepAsync()
    {
        lock (_gate)
        {
            if (!_paused)
            {
                return Task.FromResult(false);
            }

            _stepDone = new TaskCompletionSource<bool>(TaskCreationOptions.RunContinuationsAsynchronously);
            _step.Release();
            return _stepDone.Task;
        }
    }

    /// <summary>Plays the bag to the end (or forever with <see cref="BagPlayerOptions.Loop"/>).</summary>
    public async Task PlayAsync(CancellationToken cancellationToken = default)
    {
        using var linked = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, _node.Context.ShutdownToken);
        var token = linked.Token;
        if (_options.Delay > TimeSpan.Zero)
        {
            await Task.Delay(_options.Delay, token).ConfigureAwait(false);
        }

        var clockTask = _clock is null ? Task.CompletedTask : Task.Run(() => ClockLoopAsync(token), token);
        try
        {
            var start = _reader.Metadata.StartingTime + RosDuration.FromTimeSpan(_options.StartOffset);
            var topics = _options.Topics.Count > 0 ? _options.Topics : null;
            do
            {
                lock (_gate)
                {
                    Reanchor(start.Nanoseconds);
                }

                foreach (var msg in _reader.ReadMessages(start: start, topics: topics))
                {
                    token.ThrowIfCancellationRequested();
                    var stepped = await WaitForAsync(msg.LogTime.Nanoseconds, token).ConfigureAwait(false);
                    if (_publishers.TryGetValue(msg.Topic, out var pub))
                    {
                        pub.Publish(msg.Data.Span);
                        Interlocked.Increment(ref _played);
                        MessagePlayed?.Invoke(msg);
                    }

                    if (stepped)
                    {
                        _stepDone?.TrySetResult(true);
                    }
                }
            }
            while (_options.Loop && !token.IsCancellationRequested);
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            // Context shut down.
        }
        finally
        {
            _stepDone?.TrySetResult(false);
            linked.Cancel();
            try
            {
                await clockTask.ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
            }
        }
    }

    /// <summary>Waits until <paramref name="bagTime"/> is due. Returns true when released by a step.</summary>
    private async Task<bool> WaitForAsync(long bagTime, CancellationToken token)
    {
        while (true)
        {
            Task resume;
            long dueWall;
            lock (_gate)
            {
                resume = _resume.Task;
                dueWall = _anchorWall + (long)((bagTime - _anchorBag) / _rate / 1e9 * Stopwatch.Frequency);
                if (_paused)
                {
                    dueWall = long.MaxValue;
                }
            }

            if (dueWall == long.MaxValue)
            {
                var stepTask = _step.WaitAsync(token);
                var done = await Task.WhenAny(resume, stepTask).ConfigureAwait(false);
                if (done == stepTask)
                {
                    await stepTask.ConfigureAwait(false);
                    lock (_gate)
                    {
                        _anchorBag = bagTime;
                    }

                    return true;
                }

                continue;
            }

            var remaining = dueWall - Stopwatch.GetTimestamp();
            if (remaining <= 0)
            {
                return false;
            }

            var ms = remaining * 1000.0 / Stopwatch.Frequency;
            if (ms > 3)
            {
                await Task.WhenAny(Task.Delay(TimeSpan.FromMilliseconds(Math.Min(ms - 2, 200)), token), resume.IsCompleted ? Task.Delay(Timeout.Infinite, token) : resume).ConfigureAwait(false);
                token.ThrowIfCancellationRequested();
            }
            else
            {
                Thread.Yield();
            }
        }
    }

    private async Task ClockLoopAsync(CancellationToken token)
    {
        var period = TimeSpan.FromSeconds(1.0 / _options.ClockRate);
        using var timer = new PeriodicTimer(period);
        try
        {
            do
            {
                var msg = new Clock();
                PlaybackTime.CopyTo(msg.ClockValue);
                _clock!.Publish(msg);
            }
            while (await timer.WaitForNextTickAsync(token).ConfigureAwait(false));
        }
        catch (OperationCanceledException)
        {
        }
    }

    private long CurrentBagTimeLocked()
    {
        if (_paused)
        {
            return _anchorBag;
        }

        var elapsed = Stopwatch.GetTimestamp() - _anchorWall;
        return _anchorBag + (long)(elapsed / (double)Stopwatch.Frequency * 1e9 * _rate);
    }

    private void Reanchor(long bagTime)
    {
        _anchorBag = bagTime;
        _anchorWall = Stopwatch.GetTimestamp();
    }

    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }

        _disposed = true;
        foreach (var p in _publishers.Values)
        {
            p.Dispose();
        }

        _clock?.Dispose();
        _reader.Dispose();
        _step.Dispose();
    }
}
