namespace ROS2.Net;

/// <summary>How callbacks in a group may run relative to each other.</summary>
public enum CallbackGroupType
{
    /// <summary>At most one callback of the group runs at a time (the default).</summary>
    MutuallyExclusive,
    /// <summary>Callbacks of the group may run concurrently on a multi-threaded executor.</summary>
    Reentrant,
}

/// <summary>A set of callbacks with a shared concurrency rule, like <c>rclcpp::CallbackGroup</c>.</summary>
public sealed class CallbackGroup
{
    internal CallbackGroup(CallbackGroupType type)
    {
        Type = type;
        Gate = type == CallbackGroupType.MutuallyExclusive ? new SemaphoreSlim(1, 1) : null;
    }

    public CallbackGroupType Type { get; }

    internal SemaphoreSlim? Gate { get; }
}

/// <summary>
/// Base of everything an executor runs: subscriptions, timers, service servers, guard conditions.
/// A waitable is in an executor's ready queue at most once; executing it runs one unit of work and
/// re-queues it if more is pending, so queues stay bounded by QoS depth and reentrant groups can
/// still process one entity's backlog in parallel.
/// </summary>
public abstract class Waitable : IDisposable
{
    private int _queued;
    private int _disposed;

    protected Waitable(Node node, CallbackGroup? group)
    {
        Node = node;
        Group = group ?? node.DefaultCallbackGroup;
    }

    public Node Node { get; }

    public CallbackGroup Group { get; }

    public bool IsDisposed => Volatile.Read(ref _disposed) != 0;

    /// <summary>True when there is at least one unit of work to execute.</summary>
    protected internal abstract bool HasWork { get; }

    /// <summary>Takes and runs one unit of work. Implementations call <see cref="Signal"/> after taking if work remains.</summary>
    protected internal abstract ValueTask ExecuteNextAsync();

    /// <summary>Tells the node's executor this waitable has work.</summary>
    protected internal void Signal()
    {
        if (IsDisposed)
        {
            return;
        }

        if (Interlocked.Exchange(ref _queued, 1) == 0)
        {
            Node.Schedule(this);
        }
    }

    /// <summary>Called by an executor when it dequeues this waitable.</summary>
    internal void ClearQueued() => Volatile.Write(ref _queued, 0);

    /// <summary>Re-signals after an executor attaches, if work arrived while none was attached.</summary>
    internal void Rearm()
    {
        Volatile.Write(ref _queued, 0);
        if (HasWork)
        {
            Signal();
        }
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
        {
            Node.Unregister(this);
            DisposeCore();
        }

        GC.SuppressFinalize(this);
    }

    protected virtual void DisposeCore()
    {
    }
}

/// <summary>A user-triggerable wake-up, like <c>rclcpp::GuardCondition</c>. The callback runs on the executor.</summary>
public sealed class GuardCondition : Waitable
{
    private readonly Func<ValueTask> _callback;
    private int _triggered;

    internal GuardCondition(Node node, Func<ValueTask> callback, CallbackGroup? group)
        : base(node, group)
    {
        _callback = callback;
    }

    protected internal override bool HasWork => Volatile.Read(ref _triggered) != 0;

    /// <summary>Wakes the executor; triggers before execution coalesce into one callback.</summary>
    public void Trigger()
    {
        Volatile.Write(ref _triggered, 1);
        Signal();
    }

    protected internal override ValueTask ExecuteNextAsync() =>
        Interlocked.Exchange(ref _triggered, 0) != 0 ? _callback() : ValueTask.CompletedTask;
}
