using System.Threading.Channels;
using Microsoft.Extensions.Logging;

namespace ROS2.Net;

/// <summary>
/// Runs the callbacks of the nodes added to it. Equivalent to <c>rclcpp::Executor</c>: data arriving
/// from the middleware is queued per entity and executed here, honoring callback groups.
/// </summary>
public abstract class Executor : IDisposable
{
    private readonly Channel<Waitable> _ready = Channel.CreateUnbounded<Waitable>(new UnboundedChannelOptions { SingleReader = false, SingleWriter = false });
    private readonly List<Node> _nodes = [];
    private readonly object _gate = new();
    private int _spinning;

    /// <summary>Raised when a callback throws. The executor logs the exception and keeps spinning.</summary>
    public event Action<Waitable, Exception>? CallbackFaulted;

    public IReadOnlyList<Node> Nodes
    {
        get
        {
            lock (_gate)
            {
                return _nodes.ToList();
            }
        }
    }

    public bool IsSpinning => Volatile.Read(ref _spinning) != 0;

    public void AddNode(Node node)
    {
        lock (_gate)
        {
            if (_nodes.Contains(node))
            {
                return;
            }

            _nodes.Add(node);
        }

        node.AttachExecutor(this);
    }

    public void RemoveNode(Node node)
    {
        lock (_gate)
        {
            _nodes.Remove(node);
        }

        node.DetachExecutor(this);
    }

    internal void Enqueue(Waitable waitable) => _ready.Writer.TryWrite(waitable);

    /// <summary>Spins until <paramref name="cancellationToken"/> is cancelled or the context shuts down.</summary>
    public async Task SpinAsync(CancellationToken cancellationToken = default)
    {
        if (Interlocked.Exchange(ref _spinning, 1) != 0)
        {
            throw new InvalidOperationException("The executor is already spinning.");
        }

        try
        {
            using var linked = LinkShutdown(cancellationToken);
            await SpinCoreAsync(linked.Token).ConfigureAwait(false);
        }
        catch (OperationCanceledException)
        {
            // Normal termination.
        }
        finally
        {
            Volatile.Write(ref _spinning, 0);
        }
    }

    /// <summary>Blocking variant of <see cref="SpinAsync"/>.</summary>
    public void Spin(CancellationToken cancellationToken = default) => SpinAsync(cancellationToken).GetAwaiter().GetResult();

    /// <summary>Executes at most one ready callback, waiting up to <paramref name="timeout"/> for one. Returns true if a callback ran.</summary>
    public async ValueTask<bool> SpinOnceAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        if (_ready.Reader.TryRead(out var w))
        {
            await ExecuteAsync(w).ConfigureAwait(false);
            return true;
        }

        using var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        cts.CancelAfter(timeout);
        try
        {
            if (await _ready.Reader.WaitToReadAsync(cts.Token).ConfigureAwait(false) && _ready.Reader.TryRead(out w))
            {
                await ExecuteAsync(w).ConfigureAwait(false);
                return true;
            }
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
        }

        return false;
    }

    /// <summary>Executes everything that is ready right now without waiting.</summary>
    public async ValueTask<int> SpinSomeAsync()
    {
        var count = 0;
        while (_ready.Reader.TryRead(out var w))
        {
            await ExecuteAsync(w).ConfigureAwait(false);
            count++;
        }

        return count;
    }

    /// <summary>Spins until <paramref name="task"/> completes or <paramref name="timeout"/> elapses. Returns whether it completed.</summary>
    public async Task<bool> SpinUntilCompleteAsync(Task task, TimeSpan? timeout = null, CancellationToken cancellationToken = default)
    {
        using var cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        if (timeout is { } t)
        {
            cts.CancelAfter(t);
        }

        var spin = SpinAsync(cts.Token);
        try
        {
            await task.WaitAsync(cts.Token).ConfigureAwait(false);
            return true;
        }
        catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
        {
            return false;
        }
        finally
        {
            cts.Cancel();
            await spin.ConfigureAwait(false);
        }
    }

    protected abstract Task SpinCoreAsync(CancellationToken cancellationToken);

    /// <summary>Worker loop shared by the built-in executors.</summary>
    protected async Task RunWorkerAsync(CancellationToken cancellationToken)
    {
        var reader = _ready.Reader;
        while (await reader.WaitToReadAsync(cancellationToken).ConfigureAwait(false))
        {
            while (reader.TryRead(out var w))
            {
                await ExecuteAsync(w).ConfigureAwait(false);
                if (cancellationToken.IsCancellationRequested)
                {
                    return;
                }
            }
        }
    }

    /// <summary>Synchronous worker loop for dedicated threads (see <see cref="RealtimeExecutor"/>).</summary>
    protected void RunWorkerBlocking(CancellationToken cancellationToken, bool busyWait)
    {
        var reader = _ready.Reader;
        var spinner = default(SpinWait);
        while (!cancellationToken.IsCancellationRequested)
        {
            if (reader.TryRead(out var w))
            {
                spinner.Reset();
                var vt = ExecuteAsync(w);
                if (!vt.IsCompleted)
                {
                    vt.AsTask().GetAwaiter().GetResult();
                }

                continue;
            }

            if (busyWait)
            {
                spinner.SpinOnce(sleep1Threshold: -1);
            }
            else
            {
                try
                {
                    reader.WaitToReadAsync(cancellationToken).AsTask().GetAwaiter().GetResult();
                }
                catch (OperationCanceledException)
                {
                    return;
                }
            }
        }
    }

    internal async ValueTask ExecuteAsync(Waitable w)
    {
        w.ClearQueued();
        if (w.IsDisposed)
        {
            return;
        }

        var gate = w.Group.Gate;
        if (gate is not null)
        {
            await gate.WaitAsync().ConfigureAwait(false);
        }

        try
        {
            await w.ExecuteNextAsync().ConfigureAwait(false);
        }
        catch (Exception ex) when (ex is not OutOfMemoryException)
        {
            w.Node.Logger.LogError(ex, "Callback of {Entity} threw", w.GetType().Name);
            CallbackFaulted?.Invoke(w, ex);
        }
        finally
        {
            gate?.Release();
        }
    }

    private CancellationTokenSource LinkShutdown(CancellationToken token)
    {
        var contexts = Nodes.Select(n => n.Context.ShutdownToken).Distinct().ToList();
        contexts.Add(token);
        return CancellationTokenSource.CreateLinkedTokenSource(contexts.ToArray());
    }

    public virtual void Dispose()
    {
        foreach (var node in Nodes)
        {
            RemoveNode(node);
        }

        _ready.Writer.TryComplete();
        GC.SuppressFinalize(this);
    }
}

/// <summary>Runs every callback one at a time, in arrival order.</summary>
public sealed class SingleThreadedExecutor : Executor
{
    protected override Task SpinCoreAsync(CancellationToken cancellationToken) => RunWorkerAsync(cancellationToken);
}

/// <summary>
/// Runs callbacks on several workers. Callbacks in a <see cref="CallbackGroupType.MutuallyExclusive"/>
/// group still never overlap; reentrant groups run in parallel.
/// </summary>
public sealed class MultiThreadedExecutor(int threadCount = 0) : Executor
{
    public int ThreadCount { get; } = threadCount > 0 ? threadCount : Math.Max(2, Environment.ProcessorCount);

    protected override Task SpinCoreAsync(CancellationToken cancellationToken) =>
        Task.WhenAll(Enumerable.Range(0, ThreadCount).Select(_ => Task.Run(() => RunWorkerAsync(cancellationToken), cancellationToken)));
}

/// <summary>
/// Executes callbacks on one dedicated, high-priority OS thread instead of the thread pool, for
/// control loops that need bounded latency. With <c>busyWait</c> the thread spins instead of
/// blocking, trading a CPU core for sub-millisecond wake-up. Keep callbacks allocation-free
/// (so the GC has nothing to collect) for best determinism.
/// </summary>
public sealed class RealtimeExecutor(bool busyWait = false, ThreadPriority priority = ThreadPriority.Highest) : Executor
{
    public bool BusyWait { get; } = busyWait;

    public ThreadPriority Priority { get; } = priority;

    protected override Task SpinCoreAsync(CancellationToken cancellationToken)
    {
        var done = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        var thread = new Thread(() =>
        {
            try
            {
                RunWorkerBlocking(cancellationToken, BusyWait);
                done.TrySetResult();
            }
            catch (Exception ex)
            {
                done.TrySetException(ex);
            }
        })
        {
            IsBackground = true,
            Priority = Priority,
            Name = "ROS2.Net realtime executor",
        };
        thread.Start();
        return done.Task;
    }
}
