using System.Diagnostics;

namespace ROS2.Net;

/// <summary>Information passed to a timer callback.</summary>
/// <param name="ExpectedCallTime">When the callback was due.</param>
/// <param name="ActualCallTime">When the executor actually ran it.</param>
public readonly record struct TimerInfo(RosTime ExpectedCallTime, RosTime ActualCallTime);

/// <summary>
/// A periodic timer whose callback runs on the node's executor, like <c>rclcpp::TimerBase</c>.
/// Ticks never accumulate: if the executor falls behind, missed periods coalesce into one call.
/// Timers on a ROS clock follow simulation time when <c>use_sim_time</c> is enabled.
/// </summary>
public sealed class RosTimer : Waitable
{
    private readonly Func<TimerInfo, ValueTask> _callback;
    private readonly RosClock _clock;
    private readonly CancellationTokenSource _cts = new();
    private long _periodTicks;
    private long _nextDue;
    private int _pending;
    private RosTime _expected;
    private volatile bool _canceled;

    internal RosTimer(Node node, TimeSpan period, RosClock clock, Func<TimerInfo, ValueTask> callback, CallbackGroup? group, bool autostart)
        : base(node, group)
    {
        if (period <= TimeSpan.Zero)
        {
            throw new ArgumentOutOfRangeException(nameof(period), "Timer period must be positive.");
        }

        _callback = callback;
        _clock = clock;
        Period = period;
        _canceled = !autostart;
        if (autostart)
        {
            Start();
        }
    }

    /// <summary>Time between calls. Changing it takes effect from the next tick.</summary>
    public TimeSpan Period
    {
        get => TimeSpan.FromTicks(Interlocked.Read(ref _periodTicks) * TimeSpan.TicksPerSecond / Stopwatch.Frequency);
        set => Interlocked.Exchange(ref _periodTicks, (long)(value.TotalSeconds * Stopwatch.Frequency));
    }

    public bool IsCanceled => _canceled;

    /// <summary>The clock the timer runs on.</summary>
    public RosClock Clock => _clock;

    /// <summary>Time remaining until the next call.</summary>
    public TimeSpan TimeUntilNextCall =>
        _canceled ? Timeout.InfiniteTimeSpan : Stopwatch.GetElapsedTime(Stopwatch.GetTimestamp(), Interlocked.Read(ref _nextDue));

    protected internal override bool HasWork => Volatile.Read(ref _pending) != 0;

    /// <summary>Stops the timer. <see cref="Reset"/> restarts it.</summary>
    public void Cancel() => _canceled = true;

    /// <summary>Restarts the period from now (and un-cancels a cancelled timer).</summary>
    public void Reset()
    {
        var wasCanceled = _canceled;
        _canceled = false;
        if (wasCanceled)
        {
            Start();
        }
        else
        {
            Interlocked.Exchange(ref _nextDue, Stopwatch.GetTimestamp() + Interlocked.Read(ref _periodTicks));
        }
    }

    private void Start()
    {
        if (_clock.Type == ClockType.RosTime && _clock.IsSimTimeActive)
        {
            _ = RunOnSimClockAsync(_cts.Token);
            return;
        }

        var due = Stopwatch.GetTimestamp() + Interlocked.Read(ref _periodTicks);
        Interlocked.Exchange(ref _nextDue, due);
        TimerScheduler.Instance.Schedule(this, due);
    }

    /// <summary>Called by the scheduler thread when the timer is due.</summary>
    internal void OnDue()
    {
        if (_canceled || IsDisposed)
        {
            return;
        }

        var due = Interlocked.Read(ref _nextDue);
        var now = Stopwatch.GetTimestamp();
        if (now < due)
        {
            // Reset() pushed the deadline out after this tick was queued.
            TimerScheduler.Instance.Schedule(this, due);
            return;
        }

        _expected = _clock.Now();
        Tick();

        var period = Interlocked.Read(ref _periodTicks);
        var next = due + period;
        if (next <= now)
        {
            next = now + period; // Fell behind by more than a period: skip rather than burst.
        }

        Interlocked.Exchange(ref _nextDue, next);
        TimerScheduler.Instance.Schedule(this, next);
    }

    private async Task RunOnSimClockAsync(CancellationToken token)
    {
        var next = _clock.Now() + RosDuration.FromTimeSpan(Period);
        while (!_canceled && !token.IsCancellationRequested)
        {
            try
            {
                await _clock.WaitUntilAsync(next, token).ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                return;
            }

            if (_canceled)
            {
                return;
            }

            _expected = next;
            Tick();
            next += RosDuration.FromTimeSpan(Period);
            var now = _clock.Now();
            if (next <= now)
            {
                next = now + RosDuration.FromTimeSpan(Period);
            }
        }
    }

    private void Tick()
    {
        Volatile.Write(ref _pending, 1);
        Signal();
    }

    protected internal override ValueTask ExecuteNextAsync()
    {
        if (Interlocked.Exchange(ref _pending, 0) == 0 || _canceled)
        {
            return ValueTask.CompletedTask;
        }

        return _callback(new TimerInfo(_expected, _clock.Now()));
    }

    protected override void DisposeCore()
    {
        _canceled = true;
        _cts.Cancel();
        _cts.Dispose();
    }
}
