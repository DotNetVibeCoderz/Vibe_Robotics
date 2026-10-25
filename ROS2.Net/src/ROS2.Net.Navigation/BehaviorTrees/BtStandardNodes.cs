using System.Diagnostics;

namespace ROS2.Net.Navigation.BehaviorTrees;

/// <summary>Ticks children in order; remembers the running child across ticks.</summary>
public sealed class SequenceNode(BtNodeConfig config) : ControlNode(config)
{
    private int _current;

    protected override NodeStatus OnTick()
    {
        while (_current < Children.Count)
        {
            var s = Children[_current].Tick();
            if (s == NodeStatus.Running)
            {
                return s;
            }

            if (s == NodeStatus.Failure)
            {
                HaltChildren();
                _current = 0;
                return s;
            }

            _current++;
        }

        HaltChildren();
        _current = 0;
        return NodeStatus.Success;
    }

    protected override void OnHalt()
    {
        _current = 0;
        base.OnHalt();
    }
}

/// <summary>Re-evaluates every child from the first on each tick; a later running child is halted when an earlier one fails.</summary>
public sealed class ReactiveSequenceNode(BtNodeConfig config) : ControlNode(config)
{
    protected override NodeStatus OnTick()
    {
        for (var i = 0; i < Children.Count; i++)
        {
            var s = Children[i].Tick();
            if (s == NodeStatus.Running)
            {
                HaltChildren(i + 1);
                return s;
            }

            if (s == NodeStatus.Failure)
            {
                HaltChildren();
                return s;
            }
        }

        HaltChildren();
        return NodeStatus.Success;
    }
}

/// <summary>Tries children in order until one succeeds; remembers the running child.</summary>
public sealed class FallbackNode(BtNodeConfig config) : ControlNode(config)
{
    private int _current;

    protected override NodeStatus OnTick()
    {
        while (_current < Children.Count)
        {
            var s = Children[_current].Tick();
            if (s == NodeStatus.Running)
            {
                return s;
            }

            if (s == NodeStatus.Success)
            {
                HaltChildren();
                _current = 0;
                return s;
            }

            _current++;
        }

        HaltChildren();
        _current = 0;
        return NodeStatus.Failure;
    }

    protected override void OnHalt()
    {
        _current = 0;
        base.OnHalt();
    }
}

/// <summary>Reactive fallback: re-evaluates from the first child each tick.</summary>
public sealed class ReactiveFallbackNode(BtNodeConfig config) : ControlNode(config)
{
    protected override NodeStatus OnTick()
    {
        for (var i = 0; i < Children.Count; i++)
        {
            var s = Children[i].Tick();
            if (s == NodeStatus.Running)
            {
                HaltChildren(i + 1);
                return s;
            }

            if (s == NodeStatus.Success)
            {
                HaltChildren();
                return s;
            }
        }

        HaltChildren();
        return NodeStatus.Failure;
    }
}

/// <summary>Ticks all children each tick; succeeds when <c>success_count</c> succeed, fails when <c>failure_count</c> fail.</summary>
public sealed class ParallelNode(BtNodeConfig config) : ControlNode(config)
{
    private readonly HashSet<int> _done = [];
    private int _successes;
    private int _failures;

    protected override NodeStatus OnTick()
    {
        var successThreshold = GetInput("success_count", -1);
        var failureThreshold = GetInput("failure_count", 1);
        if (successThreshold < 0)
        {
            successThreshold = Children.Count;
        }

        for (var i = 0; i < Children.Count; i++)
        {
            if (_done.Contains(i))
            {
                continue;
            }

            var s = Children[i].Tick();
            if (s == NodeStatus.Success)
            {
                _done.Add(i);
                _successes++;
            }
            else if (s == NodeStatus.Failure)
            {
                _done.Add(i);
                _failures++;
            }

            if (_successes >= successThreshold)
            {
                Reset();
                return NodeStatus.Success;
            }

            if (_failures >= failureThreshold)
            {
                Reset();
                return NodeStatus.Failure;
            }
        }

        return NodeStatus.Running;
    }

    private void Reset()
    {
        HaltChildren();
        _done.Clear();
        _successes = 0;
        _failures = 0;
    }

    protected override void OnHalt() => Reset();
}

/// <summary>
/// Nav2's PipelineSequence: re-ticks earlier children while later ones run, so e.g. replanning
/// keeps refreshing the path the controller is following.
/// </summary>
public sealed class PipelineSequenceNode(BtNodeConfig config) : ControlNode(config)
{
    private int _lastTicked;

    protected override NodeStatus OnTick()
    {
        for (var i = 0; i < Children.Count; i++)
        {
            var s = Children[i].Tick();
            switch (s)
            {
                case NodeStatus.Failure:
                    HaltChildren();
                    _lastTicked = 0;
                    return s;
                case NodeStatus.Running:
                    if (i >= _lastTicked)
                    {
                        _lastTicked = i;
                        return s;
                    }

                    continue;
            }
        }

        HaltChildren();
        _lastTicked = 0;
        return NodeStatus.Success;
    }

    protected override void OnHalt()
    {
        _lastTicked = 0;
        base.OnHalt();
    }
}

/// <summary>Nav2's RoundRobin: each activation starts with the next child, cycling through recoveries.</summary>
public sealed class RoundRobinNode(BtNodeConfig config) : ControlNode(config)
{
    private int _current;
    private int _failures;

    protected override NodeStatus OnTick()
    {
        while (_failures < Children.Count)
        {
            var s = Children[_current].Tick();
            if (s == NodeStatus.Running)
            {
                return s;
            }

            _current = (_current + 1) % Children.Count;
            if (s == NodeStatus.Success)
            {
                _failures = 0;
                return s;
            }

            _failures++;
        }

        _failures = 0;
        return NodeStatus.Failure;
    }

    protected override void OnHalt()
    {
        _failures = 0;
        base.OnHalt();
    }
}

/// <summary>
/// Nav2's RecoveryNode: ticks the first child; when it fails, ticks the second (recovery) and retries
/// the first, up to <c>number_of_retries</c> times.
/// </summary>
public sealed class RecoveryNode(BtNodeConfig config) : ControlNode(config)
{
    private int _retries;
    private bool _recovering;

    protected override NodeStatus OnTick()
    {
        var max = GetInput("number_of_retries", 1);
        while (true)
        {
            if (!_recovering)
            {
                var s = Children[0].Tick();
                if (s == NodeStatus.Running)
                {
                    return s;
                }

                if (s == NodeStatus.Success)
                {
                    Reset();
                    return s;
                }

                if (_retries >= max || Children.Count < 2)
                {
                    Reset();
                    return NodeStatus.Failure;
                }

                Children[0].Halt();
                _recovering = true;
            }

            var r = Children[1].Tick();
            if (r == NodeStatus.Running)
            {
                return r;
            }

            Children[1].Halt();
            if (r == NodeStatus.Failure)
            {
                Reset();
                return NodeStatus.Failure;
            }

            _retries++;
            _recovering = false;
        }
    }

    private void Reset()
    {
        _retries = 0;
        _recovering = false;
        HaltChildren();
    }

    protected override void OnHalt() => Reset();
}

/// <summary>Swaps Success and Failure.</summary>
public sealed class InverterNode(BtNodeConfig config) : DecoratorNode(config)
{
    protected override NodeStatus OnTick() => Child.Tick() switch
    {
        NodeStatus.Success => NodeStatus.Failure,
        NodeStatus.Failure => NodeStatus.Success,
        var s => s,
    };
}

public sealed class ForceSuccessNode(BtNodeConfig config) : DecoratorNode(config)
{
    protected override NodeStatus OnTick() => Child.Tick() == NodeStatus.Running ? NodeStatus.Running : NodeStatus.Success;
}

public sealed class ForceFailureNode(BtNodeConfig config) : DecoratorNode(config)
{
    protected override NodeStatus OnTick() => Child.Tick() == NodeStatus.Running ? NodeStatus.Running : NodeStatus.Failure;
}

/// <summary>Retries a failing child up to <c>num_attempts</c> times (BT.CPP <c>RetryUntilSuccessful</c>).</summary>
public sealed class RetryNode(BtNodeConfig config) : DecoratorNode(config)
{
    private int _attempts;

    protected override NodeStatus OnTick()
    {
        var max = GetInput("num_attempts", 1);
        while (true)
        {
            var s = Child.Tick();
            if (s != NodeStatus.Failure)
            {
                if (s == NodeStatus.Success)
                {
                    _attempts = 0;
                }

                return s;
            }

            Child.Halt();
            if (++_attempts >= max && max >= 0)
            {
                _attempts = 0;
                return NodeStatus.Failure;
            }
        }
    }

    protected override void OnHalt()
    {
        _attempts = 0;
        base.OnHalt();
    }
}

/// <summary>Repeats the child while it succeeds; returns Running until it fails.</summary>
public sealed class KeepRunningUntilFailureNode(BtNodeConfig config) : DecoratorNode(config)
{
    protected override NodeStatus OnTick()
    {
        var s = Child.Tick();
        if (s == NodeStatus.Failure)
        {
            return s;
        }

        if (s == NodeStatus.Success)
        {
            Child.Halt();
        }

        return NodeStatus.Running;
    }
}

/// <summary>Nav2's RateController: ticks the child at most at <c>hz</c> (always on the first tick and while it runs).</summary>
public sealed class RateControllerNode(BtNodeConfig config) : DecoratorNode(config)
{
    private long _lastStart;
    private bool _first = true;

    protected override NodeStatus OnTick()
    {
        var hz = GetInput("hz", 10.0);
        if (Status == NodeStatus.Idle)
        {
            _first = true;
            _lastStart = Stopwatch.GetTimestamp();
        }

        var elapsed = Stopwatch.GetElapsedTime(_lastStart).TotalSeconds;
        if (_first || Child.Status == NodeStatus.Running || elapsed >= 1.0 / hz)
        {
            _first = false;
            var s = Child.Tick();
            if (s == NodeStatus.Success)
            {
                _lastStart = Stopwatch.GetTimestamp();
                Child.Halt();
                return NodeStatus.Success;
            }

            if (s == NodeStatus.Failure)
            {
                return s;
            }
        }

        return NodeStatus.Running;
    }

    protected override void OnHalt()
    {
        _first = true;
        base.OnHalt();
    }
}

/// <summary>Ticks the child up to <c>num_cycles</c> times while it succeeds.</summary>
public sealed class RepeatNode(BtNodeConfig config) : DecoratorNode(config)
{
    private int _count;

    protected override NodeStatus OnTick()
    {
        var cycles = GetInput("num_cycles", 1);
        var s = Child.Tick();
        if (s == NodeStatus.Success)
        {
            Child.Halt();
            if (++_count >= cycles)
            {
                _count = 0;
                return NodeStatus.Success;
            }

            return NodeStatus.Running;
        }

        if (s == NodeStatus.Failure)
        {
            _count = 0;
        }

        return s;
    }
}

/// <summary>A wrapper for a subtree; shares the parent blackboard.</summary>
public sealed class SubTreeNode(BtNodeConfig config) : DecoratorNode(config)
{
    protected override NodeStatus OnTick() => Child.Tick();
}
