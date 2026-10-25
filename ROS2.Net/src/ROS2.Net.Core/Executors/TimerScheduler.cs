using System.Diagnostics;
using System.Runtime.InteropServices;

namespace ROS2.Net;

/// <summary>
/// One dedicated thread that fires every wall/steady timer in the process. It sleeps until shortly
/// before the earliest deadline and spins the last stretch, giving sub-millisecond accuracy without
/// a thread per timer. On Windows it raises the system timer resolution to 1 ms while timers exist.
/// </summary>
internal sealed partial class TimerScheduler
{
    public static TimerScheduler Instance { get; } = new();

    private static readonly long s_spinThreshold = Stopwatch.Frequency / 1000; // ~1 ms

    private readonly PriorityQueue<RosTimer, long> _queue = new();
    private readonly AutoResetEvent _wake = new(false);
    private readonly object _gate = new();
    private Thread? _thread;
    private bool _highResolution;

    public void Schedule(RosTimer timer, long dueTimestamp)
    {
        lock (_gate)
        {
            _queue.Enqueue(timer, dueTimestamp);
            EnsureThread();
        }

        _wake.Set();
    }

    private void EnsureThread()
    {
        if (_thread is not null)
        {
            return;
        }

        if (OperatingSystem.IsWindows() && !_highResolution)
        {
            try
            {
                _ = TimeBeginPeriod(1);
                _highResolution = true;
            }
            catch (DllNotFoundException)
            {
            }
            catch (EntryPointNotFoundException)
            {
            }
        }

        _thread = new Thread(Run)
        {
            IsBackground = true,
            Name = "ROS2.Net timers",
            Priority = ThreadPriority.AboveNormal,
        };
        _thread.Start();
    }

    private void Run()
    {
        var spinner = default(SpinWait);
        while (true)
        {
            RosTimer? due = null;
            long wait;
            lock (_gate)
            {
                while (_queue.TryPeek(out var t, out var _) && t.IsCanceled)
                {
                    _queue.Dequeue();
                }

                if (!_queue.TryPeek(out var timer, out var dueAt))
                {
                    wait = -1;
                }
                else
                {
                    var now = Stopwatch.GetTimestamp();
                    wait = dueAt - now;
                    if (wait <= 0)
                    {
                        _queue.Dequeue();
                        due = timer;
                    }
                }
            }

            if (due is not null)
            {
                spinner.Reset();
                due.OnDue();
                continue;
            }

            if (wait < 0)
            {
                _wake.WaitOne();
            }
            else if (wait > s_spinThreshold * 2)
            {
                var ms = (int)Math.Max(1, ((wait - s_spinThreshold) * 1000 / Stopwatch.Frequency) - 1);
                _wake.WaitOne(ms);
            }
            else
            {
                spinner.SpinOnce(sleep1Threshold: -1);
            }
        }
    }

    [LibraryImport("winmm.dll", EntryPoint = "timeBeginPeriod")]
    private static partial uint TimeBeginPeriod(uint period);
}
