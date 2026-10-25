using System.Diagnostics;
using ROS2.Net.Msgs.BuiltinInterfaces;

namespace ROS2.Net;

/// <summary>Which clock a time point was read from.</summary>
public enum ClockType
{
    /// <summary>ROS time: system time, or simulation time from <c>/clock</c> when <c>use_sim_time</c> is set.</summary>
    RosTime = 1,
    /// <summary>Wall-clock time (UTC since the Unix epoch).</summary>
    SystemTime = 2,
    /// <summary>Monotonic time that never jumps.</summary>
    SteadyTime = 3,
}

/// <summary>A span of time with nanosecond resolution (<see cref="TimeSpan"/> only has 100 ns ticks).</summary>
public readonly record struct RosDuration(long Nanoseconds) : IComparable<RosDuration>
{
    public static RosDuration Zero => default;

    public double TotalSeconds => Nanoseconds / 1e9;

    public static RosDuration FromSeconds(double seconds) => new((long)Math.Round(seconds * 1e9));

    public static RosDuration FromMilliseconds(double ms) => new((long)Math.Round(ms * 1e6));

    public static RosDuration FromTimeSpan(TimeSpan span) => new(span.Ticks * 100);

    public TimeSpan ToTimeSpan() => TimeSpan.FromTicks(Nanoseconds / 100);

    public Duration ToMsg() => new() { Sec = (int)Math.Floor(Nanoseconds / 1e9), Nanosec = (uint)(((Nanoseconds % 1_000_000_000) + 1_000_000_000) % 1_000_000_000) };

    public static RosDuration FromMsg(Duration msg) => new((msg.Sec * 1_000_000_000L) + msg.Nanosec);

    public static RosDuration operator +(RosDuration a, RosDuration b) => new(a.Nanoseconds + b.Nanoseconds);

    public static RosDuration operator -(RosDuration a, RosDuration b) => new(a.Nanoseconds - b.Nanoseconds);

    public static RosDuration operator *(RosDuration a, double f) => new((long)(a.Nanoseconds * f));

    public static bool operator <(RosDuration a, RosDuration b) => a.Nanoseconds < b.Nanoseconds;

    public static bool operator >(RosDuration a, RosDuration b) => a.Nanoseconds > b.Nanoseconds;

    public static bool operator <=(RosDuration a, RosDuration b) => a.Nanoseconds <= b.Nanoseconds;

    public static bool operator >=(RosDuration a, RosDuration b) => a.Nanoseconds >= b.Nanoseconds;

    public static implicit operator RosDuration(TimeSpan span) => FromTimeSpan(span);

    public int CompareTo(RosDuration other) => Nanoseconds.CompareTo(other.Nanoseconds);

    public override string ToString() => $"{TotalSeconds:0.000000000}s";
}

/// <summary>A point in time on a ROS clock, in nanoseconds since that clock's epoch.</summary>
public readonly record struct RosTime(long Nanoseconds, ClockType Clock = ClockType.RosTime) : IComparable<RosTime>
{
    public static RosTime Zero => default;

    public double Seconds => Nanoseconds / 1e9;

    public bool IsZero => Nanoseconds == 0;

    /// <summary>Current wall-clock time.</summary>
    public static RosTime Now() => FromDateTime(DateTime.UtcNow);

    public static RosTime FromSeconds(double seconds, ClockType clock = ClockType.RosTime) => new((long)Math.Round(seconds * 1e9), clock);

    public static RosTime FromDateTime(DateTime utc, ClockType clock = ClockType.SystemTime) =>
        new((utc.ToUniversalTime() - DateTime.UnixEpoch).Ticks * 100, clock);

    public DateTime ToDateTime() => DateTime.UnixEpoch.AddTicks(Nanoseconds / 100);

    public Time ToMsg() => new() { Sec = (int)(Nanoseconds / 1_000_000_000), Nanosec = (uint)(Nanoseconds % 1_000_000_000) };

    public void CopyTo(Time msg)
    {
        msg.Sec = (int)(Nanoseconds / 1_000_000_000);
        msg.Nanosec = (uint)(Nanoseconds % 1_000_000_000);
    }

    public static RosTime FromMsg(Time msg, ClockType clock = ClockType.RosTime) => new((msg.Sec * 1_000_000_000L) + msg.Nanosec, clock);

    public static RosTime operator +(RosTime t, RosDuration d) => t with { Nanoseconds = t.Nanoseconds + d.Nanoseconds };

    public static RosTime operator -(RosTime t, RosDuration d) => t with { Nanoseconds = t.Nanoseconds - d.Nanoseconds };

    public static RosDuration operator -(RosTime a, RosTime b) => new(a.Nanoseconds - b.Nanoseconds);

    public static bool operator <(RosTime a, RosTime b) => a.Nanoseconds < b.Nanoseconds;

    public static bool operator >(RosTime a, RosTime b) => a.Nanoseconds > b.Nanoseconds;

    public static bool operator <=(RosTime a, RosTime b) => a.Nanoseconds <= b.Nanoseconds;

    public static bool operator >=(RosTime a, RosTime b) => a.Nanoseconds >= b.Nanoseconds;

    public int CompareTo(RosTime other) => Nanoseconds.CompareTo(other.Nanoseconds);

    public override string ToString() => $"{Nanoseconds / 1_000_000_000}.{Nanoseconds % 1_000_000_000:D9}";
}

/// <summary>
/// A ROS clock. <see cref="ClockType.RosTime"/> clocks follow <c>/clock</c> once
/// <see cref="EnableSimTime"/> is called (the node does this when <c>use_sim_time</c> is true).
/// </summary>
public sealed class RosClock
{
    private static readonly long s_steadyBase = Stopwatch.GetTimestamp();
    private static readonly long s_steadyEpochNs = RosTime.Now().Nanoseconds;

    private readonly object _gate = new();
    private long _simTimeNs;
    private bool _simActive;
    private TaskCompletionSource _simTick = new(TaskCreationOptions.RunContinuationsAsynchronously);

    public RosClock(ClockType type = ClockType.RosTime) => Type = type;

    public ClockType Type { get; }

    /// <summary>True while time comes from <c>/clock</c> instead of the system clock.</summary>
    public bool IsSimTimeActive => _simActive;

    /// <summary>Raised when simulation time jumps backwards (bag loop, simulator reset).</summary>
    public event Action<RosTime, RosTime>? TimeJumped;

    public RosTime Now()
    {
        if (Type == ClockType.SteadyTime)
        {
            var elapsed = Stopwatch.GetElapsedTime(s_steadyBase);
            return new RosTime(s_steadyEpochNs + (elapsed.Ticks * 100), ClockType.SteadyTime);
        }

        if (Type == ClockType.RosTime && _simActive)
        {
            return new RosTime(Volatile.Read(ref _simTimeNs), ClockType.RosTime);
        }

        return RosTime.Now() with { Clock = Type };
    }

    /// <summary>Switches a ROS-time clock to simulation time, starting at zero until the first <c>/clock</c> sample.</summary>
    public void EnableSimTime()
    {
        if (Type != ClockType.RosTime)
        {
            throw new InvalidOperationException("Only ROS-time clocks can follow simulation time.");
        }

        _simActive = true;
    }

    public void DisableSimTime() => _simActive = false;

    /// <summary>Sets simulation time (called for each <c>/clock</c> sample).</summary>
    public void SetSimTime(RosTime time)
    {
        TaskCompletionSource tick;
        long previous;
        lock (_gate)
        {
            previous = _simTimeNs;
            Volatile.Write(ref _simTimeNs, time.Nanoseconds);
            tick = _simTick;
            _simTick = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        }

        if (time.Nanoseconds < previous)
        {
            TimeJumped?.Invoke(new RosTime(previous), time);
        }

        tick.TrySetResult();
    }

    /// <summary>Waits until the clock reaches <paramref name="target"/>. Follows simulation time when active.</summary>
    public async ValueTask WaitUntilAsync(RosTime target, CancellationToken cancellationToken)
    {
        while (true)
        {
            cancellationToken.ThrowIfCancellationRequested();
            var now = Now();
            if (now >= target)
            {
                return;
            }

            if (Type == ClockType.RosTime && _simActive)
            {
                Task tick;
                lock (_gate)
                {
                    tick = _simTick.Task;
                }

                // Re-check after capturing the tick, so a sample that landed in between is not missed.
                if (Now() >= target)
                {
                    return;
                }

                // Poll as a fallback so a disabled sim clock or a stalled /clock cannot hang the waiter forever.
                await Task.WhenAny(tick, Task.Delay(250, cancellationToken)).ConfigureAwait(false);
                continue;
            }

            var remaining = (target - now).ToTimeSpan();
            if (remaining > TimeSpan.Zero)
            {
                await Task.Delay(remaining, cancellationToken).ConfigureAwait(false);
            }
        }
    }
}

/// <summary>Loop pacing at a fixed frequency, like <c>rclcpp::Rate</c>. Sleeps account for the time spent in the loop body.</summary>
public sealed class Rate
{
    private readonly RosClock _clock;
    private RosTime _next;

    public Rate(double hz, RosClock? clock = null)
    {
        if (hz <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(hz), "Rate must be positive.");
        }

        Period = RosDuration.FromSeconds(1.0 / hz);
        _clock = clock ?? new RosClock(ClockType.SteadyTime);
        _next = _clock.Now() + Period;
    }

    public RosDuration Period { get; }

    /// <summary>Sleeps until the next period boundary. Returns false if the loop overran its period.</summary>
    public async ValueTask<bool> SleepAsync(CancellationToken cancellationToken = default)
    {
        var now = _clock.Now();
        var onTime = now < _next;
        if (onTime)
        {
            await _clock.WaitUntilAsync(_next, cancellationToken).ConfigureAwait(false);
            _next += Period;
        }
        else
        {
            // Overran: re-anchor instead of bursting to catch up.
            _next = now + Period;
        }

        return onTime;
    }

    public void Reset() => _next = _clock.Now() + Period;
}
