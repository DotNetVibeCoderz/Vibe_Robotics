using System.Collections.Concurrent;
using ROS2.Net.Msgs.ActionMsgs;
using ROS2.Net.Rmw;

namespace ROS2.Net;

/// <summary>Raised when an action server rejects a goal.</summary>
public sealed class GoalRejectedException(string action) : InvalidOperationException($"Action '{action}' rejected the goal.");

/// <summary>Final outcome of a goal.</summary>
public sealed record ActionResult<TResult>(GoalStatus Status, TResult Result)
{
    public bool Succeeded => Status == GoalStatus.Succeeded;
}

/// <summary>Client-side handle of a sent goal.</summary>
public sealed class ClientGoalHandle<TGoal, TResult, TFeedback>
    where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
    where TResult : class, IRosMessage<TResult>, new()
    where TFeedback : class, IRosMessage<TFeedback>, new()
{
    private readonly ActionClient<TGoal, TResult, TFeedback> _client;
    private readonly Lazy<Task<ActionResult<TResult>>> _result;

    internal ClientGoalHandle(ActionClient<TGoal, TResult, TFeedback> client, Guid id, TGoal goal, RosTime stamp, Action<TFeedback>? feedback)
    {
        _client = client;
        GoalId = id;
        Goal = goal;
        AcceptedAt = stamp;
        FeedbackCallback = feedback;
        _result = new Lazy<Task<ActionResult<TResult>>>(() => client.FetchResultAsync(this));
    }

    public Guid GoalId { get; }

    public TGoal Goal { get; }

    public RosTime AcceptedAt { get; }

    /// <summary>Latest status reported by the server's status topic.</summary>
    public GoalStatus Status { get; private set; } = GoalStatus.Accepted;

    /// <summary>Raised when the status changes.</summary>
    public event Action<GoalStatus>? StatusChanged;

    /// <summary>Raised for each feedback message.</summary>
    public event Action<TFeedback>? FeedbackReceived;

    internal Action<TFeedback>? FeedbackCallback { get; }

    /// <summary>Awaits the result. Safe to call repeatedly; the server is asked once.</summary>
    public Task<ActionResult<TResult>> GetResultAsync(CancellationToken cancellationToken = default) =>
        _result.Value.WaitAsync(cancellationToken);

    /// <summary>Requests cancellation. Returns true if the server accepted the request.</summary>
    public Task<bool> CancelAsync(CancellationToken cancellationToken = default) => _client.CancelAsync(this, cancellationToken);

    internal void OnFeedback(TFeedback feedback)
    {
        FeedbackCallback?.Invoke(feedback);
        FeedbackReceived?.Invoke(feedback);
    }

    internal void OnStatus(GoalStatus status)
    {
        if (status != Status)
        {
            Status = status;
            StatusChanged?.Invoke(status);
        }
    }
}

/// <summary>
/// Sends goals to a ROS 2 action server. Responses, feedback and status are delivered directly, so
/// awaiting a goal does not require the node to be spinning.
/// </summary>
public sealed class ActionClient<TGoal, TResult, TFeedback> : IDisposable
    where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
    where TResult : class, IRosMessage<TResult>, new()
    where TFeedback : class, IRosMessage<TFeedback>, new()
{
    private readonly Node _node;
    private readonly ServiceClient<SendGoalRequest<TGoal>, SendGoalResponse> _sendGoal;
    private readonly ServiceClient<CancelGoalRequest, CancelGoalResponse> _cancelGoal;
    private readonly ServiceClient<GetResultRequest, GetResultResponse<TResult>> _getResult;
    private readonly IRmwSubscription _feedback;
    private readonly IRmwSubscription _status;
    private readonly ConcurrentDictionary<Guid, ClientGoalHandle<TGoal, TResult, TFeedback>> _handles = new();
    private int _disposed;

    internal ActionClient(Node node, string name)
    {
        _node = node;
        ActionName = node.ResolveTopicName(name);
        var type = TGoal.ActionTypeName;
        _sendGoal = node.CreateClientCore<SendGoalRequest<TGoal>, SendGoalResponse>(ActionName + "/_action/send_goal", ActionTypes.SendGoal<TGoal>(type));
        _cancelGoal = node.CreateClientCore<CancelGoalRequest, CancelGoalResponse>(ActionName + "/_action/cancel_goal", ActionTypes.CancelGoal());
        _getResult = node.CreateClientCore<GetResultRequest, GetResultResponse<TResult>>(ActionName + "/_action/get_result", ActionTypes.GetResult<TResult>(type));
        _feedback = node.RmwNode.CreateSubscription<FeedbackMessage<TFeedback>>(
            new RmwTopicSpec(ActionName + "/_action/feedback", ActionTypes.Feedback<TFeedback>(type), QosProfile.Default, RmwEndpointOptions.Default),
            OnFeedback);
        _status = node.RmwNode.CreateSubscription<GoalStatusArray>(
            new RmwTopicSpec(ActionName + "/_action/status", MessageTypeInfo.Of<GoalStatusArray>(), QosProfile.ActionStatus, RmwEndpointOptions.Default),
            OnStatus);
    }

    /// <summary>Fully qualified action name.</summary>
    public string ActionName { get; }

    public bool IsServerReady => _sendGoal.IsServiceReady && _cancelGoal.IsServiceReady && _getResult.IsServiceReady;

    /// <summary>Per-call timeout of the action's services.</summary>
    public TimeSpan? ServiceTimeout { get; set; } = TimeSpan.FromSeconds(10);

    /// <summary>Waits until the server's three services have been discovered.</summary>
    public async Task<bool> WaitForServerAsync(TimeSpan? timeout = null, CancellationToken cancellationToken = default)
    {
        var start = Environment.TickCount64;
        while (!IsServerReady)
        {
            if (timeout is { } t && Environment.TickCount64 - start > t.TotalMilliseconds)
            {
                return false;
            }

            await Task.Delay(20, cancellationToken).ConfigureAwait(false);
        }

        return true;
    }

    /// <summary>Sends a goal. Throws <see cref="GoalRejectedException"/> when the server rejects it.</summary>
    public async Task<ClientGoalHandle<TGoal, TResult, TFeedback>> SendGoalAsync(TGoal goal, Action<TFeedback>? feedback = null, CancellationToken cancellationToken = default)
    {
        var id = Guid.NewGuid();

        // Register before sending: feedback can arrive before the send_goal response.
        var provisional = new ClientGoalHandle<TGoal, TResult, TFeedback>(this, id, goal, RosTime.Zero, feedback);
        _handles[id] = provisional;
        SendGoalResponse response;
        try
        {
            response = await _sendGoal.CallAsync(new SendGoalRequest<TGoal> { GoalId = ActionTypes.ToUuid(id), Goal = goal }, ServiceTimeout, cancellationToken).ConfigureAwait(false);
        }
        catch
        {
            _handles.TryRemove(id, out _);
            throw;
        }

        if (!response.Accepted)
        {
            _handles.TryRemove(id, out _);
            throw new GoalRejectedException(ActionName);
        }

        var handle = new ClientGoalHandle<TGoal, TResult, TFeedback>(this, id, goal, RosTime.FromMsg(response.Stamp), feedback);
        _handles[id] = handle;
        return handle;
    }

    /// <summary>Sends a goal and awaits its result.</summary>
    public async Task<ActionResult<TResult>> SendGoalAndWaitAsync(TGoal goal, Action<TFeedback>? feedback = null, CancellationToken cancellationToken = default)
    {
        var handle = await SendGoalAsync(goal, feedback, cancellationToken).ConfigureAwait(false);
        using var registration = cancellationToken.Register(() => _ = handle.CancelAsync());
        return await handle.GetResultAsync(CancellationToken.None).ConfigureAwait(false);
    }

    /// <summary>Cancels every goal on the server.</summary>
    public async Task<int> CancelAllGoalsAsync(CancellationToken cancellationToken = default)
    {
        var res = await _cancelGoal.CallAsync(new CancelGoalRequest(), ServiceTimeout, cancellationToken).ConfigureAwait(false);
        return res.GoalsCanceling.Length;
    }

    internal async Task<bool> CancelAsync(ClientGoalHandle<TGoal, TResult, TFeedback> handle, CancellationToken cancellationToken)
    {
        var req = new CancelGoalRequest { GoalInfo = new GoalInfo { GoalId = ActionTypes.ToUuid(handle.GoalId) } };
        var res = await _cancelGoal.CallAsync(req, ServiceTimeout, cancellationToken).ConfigureAwait(false);
        return res.ReturnCode == CancelGoalResponse.ERROR_NONE;
    }

    internal async Task<ActionResult<TResult>> FetchResultAsync(ClientGoalHandle<TGoal, TResult, TFeedback> handle)
    {
        // get_result blocks server-side until the goal finishes, so no timeout here.
        var res = await _getResult.CallAsync(new GetResultRequest { GoalId = ActionTypes.ToUuid(handle.GoalId) }, null, _node.Context.ShutdownToken).ConfigureAwait(false);
        var status = (GoalStatus)res.Status;
        handle.OnStatus(status);
        _handles.TryRemove(handle.GoalId, out _);
        return new ActionResult<TResult>(status, res.Result);
    }

    private void OnFeedback(FeedbackMessage<TFeedback> message, MessageInfo _)
    {
        if (_handles.TryGetValue(ActionTypes.FromUuid(message.GoalId), out var handle))
        {
            handle.OnFeedback(message.Feedback);
        }
    }

    private void OnStatus(GoalStatusArray message, MessageInfo _)
    {
        foreach (var s in message.StatusList)
        {
            if (_handles.TryGetValue(ActionTypes.FromUuid(s.GoalInfo.GoalId), out var handle))
            {
                handle.OnStatus((GoalStatus)s.Status);
            }
        }
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0)
        {
            return;
        }

        _sendGoal.Dispose();
        _cancelGoal.Dispose();
        _getResult.Dispose();
        _feedback.Dispose();
        _status.Dispose();
        _node.Untrack(this);
    }
}
