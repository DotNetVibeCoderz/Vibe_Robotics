using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs.LifecycleMsgs;

namespace ROS2.Net;

/// <summary>Lifecycle states (<c>lifecycle_msgs/msg/State</c>).</summary>
public enum LifecycleState : byte
{
    Unknown = 0,
    Unconfigured = 1,
    Inactive = 2,
    Active = 3,
    Finalized = 4,
    Configuring = 10,
    CleaningUp = 11,
    ShuttingDown = 12,
    Activating = 13,
    Deactivating = 14,
    ErrorProcessing = 15,
}

/// <summary>Lifecycle transitions that can be requested (<c>lifecycle_msgs/msg/Transition</c>).</summary>
public enum LifecycleTransition : byte
{
    Configure = 1,
    Cleanup = 2,
    Activate = 3,
    Deactivate = 4,
    UnconfiguredShutdown = 5,
    InactiveShutdown = 6,
    ActiveShutdown = 7,
}

/// <summary>Result of a lifecycle callback.</summary>
public enum CallbackReturn
{
    Success,
    Failure,
    Error,
}

/// <summary>
/// A managed node with the ROS 2 lifecycle state machine
/// (Unconfigured → Inactive → Active → Finalized), controllable through
/// <c>~/change_state</c> and friends (<c>ros2 lifecycle set /node configure</c>).
/// Override the <c>On…</c> methods, or assign the matching handler properties.
/// </summary>
public class LifecycleNode : Node
{
    private readonly SemaphoreSlim _transitionGate = new(1, 1);
    private readonly Publisher<TransitionEvent> _transitionEvents;
    private readonly List<ILifecycleManaged> _managed = [];
    private LifecycleState _state = LifecycleState.Unconfigured;

    public LifecycleNode(Ros2Context context, string name, NodeOptions? options = null)
        : base(context, name, options)
    {
        _transitionEvents = CreatePublisher<TransitionEvent>("~/transition_event", QosProfile.Default);

        CreateService<ChangeStateRequest, ChangeStateResponse>("~/change_state", async (req, ct) =>
        {
            var before = CurrentState;
            var after = await TriggerTransitionAsync((LifecycleTransition)req.Transition.Id, ct).ConfigureAwait(false);
            return new ChangeStateResponse { Success = after != before && after != LifecycleState.ErrorProcessing && IsExpectedGoal((LifecycleTransition)req.Transition.Id, after) };
        });

        CreateService<GetStateRequest, GetStateResponse>("~/get_state", _ => new GetStateResponse { CurrentState = ToMsg(CurrentState) });

        CreateService<GetAvailableStatesRequest, GetAvailableStatesResponse>("~/get_available_states", _ => new GetAvailableStatesResponse
        {
            AvailableStates = Enum.GetValues<LifecycleState>().Where(s => s != LifecycleState.Unknown).Select(ToMsg).ToArray(),
        });

        CreateService<GetAvailableTransitionsRequest, GetAvailableTransitionsResponse>("~/get_available_transitions", _ => new GetAvailableTransitionsResponse
        {
            AvailableTransitions = AvailableTransitions(CurrentState).Select(t => new TransitionDescription
            {
                Transition = new Transition { Id = (byte)t, Label = Label(t) },
                StartState = ToMsg(CurrentState),
                GoalState = ToMsg(GoalOf(t)),
            }).ToArray(),
        });

        CreateService<GetAvailableTransitionsRequest, GetAvailableTransitionsResponse>("~/get_transition_graph", _ => new GetAvailableTransitionsResponse
        {
            AvailableTransitions = Enum.GetValues<LifecycleState>()
                .SelectMany(s => AvailableTransitions(s).Select(t => new TransitionDescription
                {
                    Transition = new Transition { Id = (byte)t, Label = Label(t) },
                    StartState = ToMsg(s),
                    GoalState = ToMsg(GoalOf(t)),
                }))
                .ToArray(),
        });
    }

    public LifecycleState CurrentState => _state;

    /// <summary>Raised after every state change (including intermediate transition states).</summary>
    public event Action<LifecycleState, LifecycleState>? StateChanged;

    public Func<LifecycleState, ValueTask<CallbackReturn>>? ConfigureHandler { get; set; }

    public Func<LifecycleState, ValueTask<CallbackReturn>>? ActivateHandler { get; set; }

    public Func<LifecycleState, ValueTask<CallbackReturn>>? DeactivateHandler { get; set; }

    public Func<LifecycleState, ValueTask<CallbackReturn>>? CleanupHandler { get; set; }

    public Func<LifecycleState, ValueTask<CallbackReturn>>? ShutdownHandler { get; set; }

    public Func<LifecycleState, ValueTask<CallbackReturn>>? ErrorHandler { get; set; }

    protected virtual ValueTask<CallbackReturn> OnConfigure(LifecycleState previous) => Invoke(ConfigureHandler, previous);

    protected virtual ValueTask<CallbackReturn> OnActivate(LifecycleState previous) => Invoke(ActivateHandler, previous);

    protected virtual ValueTask<CallbackReturn> OnDeactivate(LifecycleState previous) => Invoke(DeactivateHandler, previous);

    protected virtual ValueTask<CallbackReturn> OnCleanup(LifecycleState previous) => Invoke(CleanupHandler, previous);

    protected virtual ValueTask<CallbackReturn> OnShutdown(LifecycleState previous) => Invoke(ShutdownHandler, previous);

    /// <summary>Called when another callback returned <see cref="CallbackReturn.Error"/> or threw. Success goes to Unconfigured, otherwise Finalized.</summary>
    protected virtual ValueTask<CallbackReturn> OnError(LifecycleState previous) => Invoke(ErrorHandler, previous);

    private static ValueTask<CallbackReturn> Invoke(Func<LifecycleState, ValueTask<CallbackReturn>>? handler, LifecycleState previous) =>
        handler?.Invoke(previous) ?? ValueTask.FromResult(CallbackReturn.Success);

    /// <summary>A publisher that only publishes while the node is <see cref="LifecycleState.Active"/>.</summary>
    public LifecyclePublisher<T> CreateLifecyclePublisher<T>(string topic, QosProfile? qos = null)
        where T : class, IRosMessage<T>, new()
    {
        var pub = new LifecyclePublisher<T>(this, ResolveTopicName(topic), qos ?? QosProfile.Default);
        Track(pub);
        lock (_managed)
        {
            _managed.Add(pub);
        }

        if (_state == LifecycleState.Active)
        {
            pub.OnActivate();
        }

        return pub;
    }

    public ValueTask<LifecycleState> ConfigureAsync(CancellationToken ct = default) => TriggerTransitionAsync(LifecycleTransition.Configure, ct);

    public ValueTask<LifecycleState> ActivateAsync(CancellationToken ct = default) => TriggerTransitionAsync(LifecycleTransition.Activate, ct);

    public ValueTask<LifecycleState> DeactivateAsync(CancellationToken ct = default) => TriggerTransitionAsync(LifecycleTransition.Deactivate, ct);

    public ValueTask<LifecycleState> CleanupAsync(CancellationToken ct = default) => TriggerTransitionAsync(LifecycleTransition.Cleanup, ct);

    /// <summary>Shuts down from whichever primary state the node is in.</summary>
    public ValueTask<LifecycleState> ShutdownAsync(CancellationToken ct = default) => TriggerTransitionAsync(_state switch
    {
        LifecycleState.Active => LifecycleTransition.ActiveShutdown,
        LifecycleState.Inactive => LifecycleTransition.InactiveShutdown,
        _ => LifecycleTransition.UnconfiguredShutdown,
    }, ct);

    /// <summary>Runs a transition and returns the resulting primary state. Invalid transitions leave the state unchanged.</summary>
    public async ValueTask<LifecycleState> TriggerTransitionAsync(LifecycleTransition transition, CancellationToken cancellationToken = default)
    {
        await _transitionGate.WaitAsync(cancellationToken).ConfigureAwait(false);
        try
        {
            var start = _state;
            if (!AvailableTransitions(start).Contains(transition))
            {
                Logger.LogWarning("Transition {Transition} is not available in state {State}", transition, start);
                return start;
            }

            var (intermediate, callback, success) = transition switch
            {
                LifecycleTransition.Configure => (LifecycleState.Configuring, (Func<LifecycleState, ValueTask<CallbackReturn>>)OnConfigure, LifecycleState.Inactive),
                LifecycleTransition.Cleanup => (LifecycleState.CleaningUp, OnCleanup, LifecycleState.Unconfigured),
                LifecycleTransition.Activate => (LifecycleState.Activating, OnActivate, LifecycleState.Active),
                LifecycleTransition.Deactivate => (LifecycleState.Deactivating, OnDeactivate, LifecycleState.Inactive),
                _ => (LifecycleState.ShuttingDown, (Func<LifecycleState, ValueTask<CallbackReturn>>)OnShutdown, LifecycleState.Finalized),
            };

            SetState(intermediate, transition, start);
            CallbackReturn ret;
            try
            {
                ret = await callback(start).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Lifecycle callback for {Transition} threw", transition);
                ret = CallbackReturn.Error;
            }

            if (ret == CallbackReturn.Success)
            {
                ApplyManaged(success);
                SetState(success, transition, intermediate);
                return success;
            }

            if (ret == CallbackReturn.Failure)
            {
                // Failure returns to the start state (shutdown failures still finalize per the design).
                var back = intermediate == LifecycleState.ShuttingDown ? LifecycleState.Finalized : start;
                SetState(back, transition, intermediate);
                return back;
            }

            SetState(LifecycleState.ErrorProcessing, transition, intermediate);
            CallbackReturn errorRet;
            try
            {
                errorRet = await OnError(start).ConfigureAwait(false);
            }
            catch (Exception ex)
            {
                Logger.LogError(ex, "Lifecycle error handler threw");
                errorRet = CallbackReturn.Error;
            }

            var final = errorRet == CallbackReturn.Success ? LifecycleState.Unconfigured : LifecycleState.Finalized;
            ApplyManaged(final);
            SetState(final, transition, LifecycleState.ErrorProcessing);
            return final;
        }
        finally
        {
            _transitionGate.Release();
        }
    }

    private void ApplyManaged(LifecycleState state)
    {
        List<ILifecycleManaged> managed;
        lock (_managed)
        {
            managed = _managed.ToList();
        }

        foreach (var m in managed)
        {
            if (state == LifecycleState.Active)
            {
                m.OnActivate();
            }
            else
            {
                m.OnDeactivate();
            }
        }
    }

    private void SetState(LifecycleState state, LifecycleTransition transition, LifecycleState previous)
    {
        _state = state;
        _transitionEvents.Publish(new TransitionEvent
        {
            Timestamp = (ulong)Clock.Now().Nanoseconds,
            Transition = new Transition { Id = (byte)transition, Label = Label(transition) },
            StartState = ToMsg(previous),
            GoalState = ToMsg(state),
        });
        StateChanged?.Invoke(previous, state);
    }

    private static bool IsExpectedGoal(LifecycleTransition t, LifecycleState s) => GoalOf(t) == s;

    public static IReadOnlyList<LifecycleTransition> AvailableTransitions(LifecycleState state) => state switch
    {
        LifecycleState.Unconfigured => [LifecycleTransition.Configure, LifecycleTransition.UnconfiguredShutdown],
        LifecycleState.Inactive => [LifecycleTransition.Cleanup, LifecycleTransition.Activate, LifecycleTransition.InactiveShutdown],
        LifecycleState.Active => [LifecycleTransition.Deactivate, LifecycleTransition.ActiveShutdown],
        _ => [],
    };

    private static LifecycleState GoalOf(LifecycleTransition t) => t switch
    {
        LifecycleTransition.Configure => LifecycleState.Inactive,
        LifecycleTransition.Cleanup => LifecycleState.Unconfigured,
        LifecycleTransition.Activate => LifecycleState.Active,
        LifecycleTransition.Deactivate => LifecycleState.Inactive,
        _ => LifecycleState.Finalized,
    };

    public static string Label(LifecycleState s) => s switch
    {
        LifecycleState.Configuring => "configuring",
        LifecycleState.CleaningUp => "cleaningup",
        LifecycleState.ShuttingDown => "shuttingdown",
        LifecycleState.Activating => "activating",
        LifecycleState.Deactivating => "deactivating",
        LifecycleState.ErrorProcessing => "errorprocessing",
        _ => s.ToString().ToLowerInvariant(),
    };

    public static string Label(LifecycleTransition t) => t switch
    {
        LifecycleTransition.UnconfiguredShutdown or LifecycleTransition.InactiveShutdown or LifecycleTransition.ActiveShutdown => "shutdown",
        _ => t.ToString().ToLowerInvariant(),
    };

    private static State ToMsg(LifecycleState s) => new() { Id = (byte)s, Label = Label(s) };
}

/// <summary>An entity whose behavior follows its lifecycle node's activation.</summary>
public interface ILifecycleManaged
{
    void OnActivate();

    void OnDeactivate();
}

/// <summary>A publisher that drops messages unless its node is active, like <c>rclcpp_lifecycle::LifecyclePublisher</c>.</summary>
public sealed class LifecyclePublisher<T> : Publisher<T>, ILifecycleManaged
    where T : class, IRosMessage<T>, new()
{
    private volatile bool _active;
    private bool _warned;

    internal LifecyclePublisher(Node node, string topic, QosProfile qos)
        : base(node, topic, qos, null)
    {
    }

    public bool IsActivated => _active;

    public void OnActivate() => _active = true;

    public void OnDeactivate() => _active = false;

    public override void Publish(T message)
    {
        if (!_active)
        {
            if (!_warned)
            {
                _warned = true;
                Node.Logger.LogWarning("Publishing on '{Topic}' while the lifecycle publisher is not activated; the message is dropped.", TopicName);
            }

            return;
        }

        base.Publish(message);
    }
}
