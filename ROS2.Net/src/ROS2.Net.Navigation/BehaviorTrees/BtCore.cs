using System.Globalization;

namespace ROS2.Net.Navigation.BehaviorTrees;

/// <summary>Result of ticking a behavior tree node.</summary>
public enum NodeStatus
{
    Idle,
    Running,
    Success,
    Failure,
}

/// <summary>Key/value storage shared by the nodes of a tree.</summary>
public sealed class Blackboard
{
    private readonly Dictionary<string, object?> _values = new(StringComparer.Ordinal);
    private readonly object _gate = new();

    public void Set<T>(string key, T value)
    {
        lock (_gate)
        {
            _values[key] = value;
        }
    }

    public bool TryGet<T>(string key, out T value)
    {
        lock (_gate)
        {
            if (_values.TryGetValue(key, out var v) && v is T t)
            {
                value = t;
                return true;
            }
        }

        value = default!;
        return false;
    }

    public T Get<T>(string key) =>
        TryGet<T>(key, out var v) ? v : throw new KeyNotFoundException($"Blackboard has no '{key}' of type {typeof(T).Name}.");

    public bool Contains(string key)
    {
        lock (_gate)
        {
            return _values.ContainsKey(key);
        }
    }

    public object? this[string key]
    {
        get
        {
            lock (_gate)
            {
                return _values.GetValueOrDefault(key);
            }
        }
    }
}

/// <summary>Construction data for a node: its instance name, its port values and the tree's blackboard.</summary>
public sealed record BtNodeConfig(string Name, IReadOnlyDictionary<string, string> Ports, Blackboard Blackboard);

/// <summary>Base of every behavior tree node.</summary>
public abstract class BtNode
{
    protected BtNode(BtNodeConfig config)
    {
        Config = config;
    }

    public BtNodeConfig Config { get; }

    public string Name => Config.Name;

    public Blackboard Blackboard => Config.Blackboard;

    public NodeStatus Status { get; private set; } = NodeStatus.Idle;

    /// <summary>Ticks the node once.</summary>
    public NodeStatus Tick()
    {
        var s = OnTick();
        Status = s;
        return s;
    }

    /// <summary>Interrupts a running node and resets it to idle.</summary>
    public void Halt()
    {
        if (Status == NodeStatus.Running)
        {
            OnHalt();
        }

        Status = NodeStatus.Idle;
    }

    protected abstract NodeStatus OnTick();

    protected virtual void OnHalt()
    {
    }

    /// <summary>Resets the status to idle after a terminal result (used by control nodes).</summary>
    internal void ResetStatus() => Status = NodeStatus.Idle;

    /// <summary>Reads an input port: <c>{key}</c> reads the blackboard, anything else is parsed as a literal.</summary>
    public T GetInput<T>(string port, T fallback = default!)
    {
        if (!Config.Ports.TryGetValue(port, out var raw))
        {
            return fallback;
        }

        if (raw.Length > 2 && raw[0] == '{' && raw[^1] == '}')
        {
            return Blackboard.TryGet<T>(raw[1..^1], out var v) ? v : fallback;
        }

        return ParseLiteral(raw, fallback);
    }

    /// <summary>Writes an output port; the port value must be a <c>{key}</c> reference.</summary>
    public void SetOutput<T>(string port, T value)
    {
        if (Config.Ports.TryGetValue(port, out var raw) && raw.Length > 2 && raw[0] == '{' && raw[^1] == '}')
        {
            Blackboard.Set(raw[1..^1], value);
        }
        else
        {
            Blackboard.Set(port, value);
        }
    }

    private static T ParseLiteral<T>(string raw, T fallback)
    {
        try
        {
            object? value = typeof(T) switch
            {
                var t when t == typeof(string) => raw,
                var t when t == typeof(double) => double.Parse(raw, CultureInfo.InvariantCulture),
                var t when t == typeof(int) => int.Parse(raw, CultureInfo.InvariantCulture),
                var t when t == typeof(bool) => raw is "true" or "True" or "1",
                _ => Convert.ChangeType(raw, typeof(T), CultureInfo.InvariantCulture),
            };
            return (T)value!;
        }
        catch (FormatException)
        {
            return fallback;
        }
        catch (InvalidCastException)
        {
            return fallback;
        }
    }
}

/// <summary>A node with children.</summary>
public abstract class ControlNode(BtNodeConfig config) : BtNode(config)
{
    public List<BtNode> Children { get; } = [];

    protected void HaltChildren(int from = 0)
    {
        for (var i = from; i < Children.Count; i++)
        {
            Children[i].Halt();
        }
    }

    protected override void OnHalt() => HaltChildren();
}

/// <summary>A node with exactly one child.</summary>
public abstract class DecoratorNode(BtNodeConfig config) : BtNode(config)
{
    public BtNode Child { get; set; } = null!;

    protected override void OnHalt() => Child.Halt();
}

/// <summary>A synchronous action implemented by a delegate.</summary>
public sealed class ActionNode(BtNodeConfig config, Func<BtNode, NodeStatus> tick) : BtNode(config)
{
    protected override NodeStatus OnTick() => tick(this);
}

/// <summary>A condition implemented by a delegate (never returns Running).</summary>
public sealed class ConditionNode(BtNodeConfig config, Func<BtNode, bool> condition) : BtNode(config)
{
    protected override NodeStatus OnTick() => condition(this) ? NodeStatus.Success : NodeStatus.Failure;
}

/// <summary>
/// An action that runs asynchronously: the first tick starts <see cref="StartAsync"/>, later ticks
/// report Running until the task completes; halting cancels it (BehaviorTree.CPP's StatefulActionNode).
/// </summary>
public abstract class AsyncActionNode(BtNodeConfig config) : BtNode(config)
{
    private Task<NodeStatus>? _task;
    private CancellationTokenSource? _cts;

    protected abstract Task<NodeStatus> StartAsync(CancellationToken cancellationToken);

    protected override NodeStatus OnTick()
    {
        if (_task is null)
        {
            _cts = new CancellationTokenSource();
            _task = StartAsync(_cts.Token);
        }

        if (!_task.IsCompleted)
        {
            return NodeStatus.Running;
        }

        var result = _task.IsCompletedSuccessfully ? _task.Result : NodeStatus.Failure;
        _task = null;
        _cts?.Dispose();
        _cts = null;
        return result;
    }

    protected override void OnHalt()
    {
        _cts?.Cancel();
        _task = null;
    }
}

/// <summary>An async action backed by a delegate.</summary>
public sealed class DelegateAsyncActionNode(BtNodeConfig config, Func<BtNode, CancellationToken, Task<NodeStatus>> run) : AsyncActionNode(config)
{
    protected override Task<NodeStatus> StartAsync(CancellationToken cancellationToken) => run(this, cancellationToken);
}

/// <summary>A loaded tree plus its blackboard.</summary>
public sealed class BehaviorTree(BtNode root, Blackboard blackboard)
{
    public BtNode Root { get; } = root;

    public Blackboard Blackboard { get; } = blackboard;

    public NodeStatus TickOnce() => Root.Tick();

    /// <summary>Ticks every <paramref name="period"/> until the root returns Success or Failure.</summary>
    public async Task<NodeStatus> TickWhileRunningAsync(TimeSpan period, CancellationToken cancellationToken = default, Action? onTick = null)
    {
        while (true)
        {
            if (cancellationToken.IsCancellationRequested)
            {
                Root.Halt();
                cancellationToken.ThrowIfCancellationRequested();
            }

            var status = Root.Tick();
            onTick?.Invoke();
            if (status is NodeStatus.Success or NodeStatus.Failure)
            {
                return status;
            }

            try
            {
                await Task.Delay(period, cancellationToken).ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                Root.Halt();
                throw;
            }
        }
    }

    public void Halt() => Root.Halt();

    /// <summary>Every node in depth-first order.</summary>
    public IEnumerable<BtNode> Nodes()
    {
        var stack = new Stack<BtNode>();
        stack.Push(Root);
        while (stack.Count > 0)
        {
            var n = stack.Pop();
            yield return n;
            switch (n)
            {
                case ControlNode c:
                    for (var i = c.Children.Count - 1; i >= 0; i--)
                    {
                        stack.Push(c.Children[i]);
                    }

                    break;
                case DecoratorNode d:
                    stack.Push(d.Child);
                    break;
            }
        }
    }
}
