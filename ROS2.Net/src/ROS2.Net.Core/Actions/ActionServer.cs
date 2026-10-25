using System.Collections.Concurrent;
using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs.ActionMsgs;
using ROS2.Net.Rmw;

namespace ROS2.Net;

/// <summary>State of an action goal (<c>action_msgs/msg/GoalStatus</c>).</summary>
public enum GoalStatus : sbyte
{
    Unknown = 0,
    Accepted = 1,
    Executing = 2,
    Canceling = 3,
    Succeeded = 4,
    Canceled = 5,
    Aborted = 6,
}

/// <summary>Decision on an incoming goal.</summary>
public enum GoalResponse
{
    Accept,
    Reject,
}

/// <summary>Decision on a cancel request.</summary>
public enum CancelResponse
{
    Accept,
    Reject,
}

/// <summary>Options for <see cref="ActionServer{TGoal, TResult, TFeedback}"/>.</summary>
public sealed class ActionServerOptions<TGoal, TResult, TFeedback>
    where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
    where TResult : class, IRosMessage<TResult>, new()
    where TFeedback : class, IRosMessage<TFeedback>, new()
{
    /// <summary>Accepts or rejects new goals. Defaults to accepting everything.</summary>
    public Func<Guid, TGoal, GoalResponse>? GoalCallback { get; init; }

    /// <summary>Accepts or rejects cancel requests. Defaults to accepting.</summary>
    public Func<ServerGoalHandle<TGoal, TResult, TFeedback>, CancelResponse>? CancelCallback { get; init; }

    /// <summary>How long results stay available to <c>get_result</c> after a goal finishes (ROS default: 15 minutes).</summary>
    public TimeSpan ResultTimeout { get; init; } = TimeSpan.FromMinutes(15);

    /// <summary>Callback group for the action's services.</summary>
    public CallbackGroup? CallbackGroup { get; init; }
}

/// <summary>A goal being executed by an <see cref="ActionServer{TGoal, TResult, TFeedback}"/>.</summary>
public sealed class ServerGoalHandle<TGoal, TResult, TFeedback>
    where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
    where TResult : class, IRosMessage<TResult>, new()
    where TFeedback : class, IRosMessage<TFeedback>, new()
{
    private readonly ActionServer<TGoal, TResult, TFeedback> _server;
    private readonly CancellationTokenSource _cancel = new();
    private readonly TaskCompletionSource<(GoalStatus Status, TResult Result)> _completion = new(TaskCreationOptions.RunContinuationsAsynchronously);
    private GoalStatus? _requestedFinal;

    internal ServerGoalHandle(ActionServer<TGoal, TResult, TFeedback> server, Guid id, TGoal goal, RosTime acceptedAt)
    {
        _server = server;
        GoalId = id;
        Goal = goal;
        AcceptedAt = acceptedAt;
    }

    public Guid GoalId { get; }

    public TGoal Goal { get; }

    public RosTime AcceptedAt { get; }

    public GoalStatus Status { get; internal set; } = GoalStatus.Accepted;

    public bool IsActive => Status is GoalStatus.Accepted or GoalStatus.Executing or GoalStatus.Canceling;

    public bool IsCancelRequested => _cancel.IsCancellationRequested;

    /// <summary>Cancelled when a client's cancel request is accepted.</summary>
    public CancellationToken CancellationToken => _cancel.Token;

    internal Task<(GoalStatus Status, TResult Result)> Completion => _completion.Task;

    internal RosTime? FinishedAt { get; private set; }

    /// <summary>Publishes feedback for this goal.</summary>
    public void PublishFeedback(TFeedback feedback) => _server.PublishFeedback(GoalId, feedback);

    /// <summary>Marks the goal aborted; the result returned by the execute callback is still delivered.</summary>
    public void Abort() => _requestedFinal = GoalStatus.Aborted;

    /// <summary>Marks the goal canceled (after honoring <see cref="IsCancelRequested"/>).</summary>
    public void Canceled() => _requestedFinal = GoalStatus.Canceled;

    internal void RequestCancel()
    {
        Status = GoalStatus.Canceling;
        _cancel.Cancel();
    }

    internal void Complete(GoalStatus fallback, TResult result, RosTime now)
    {
        var status = _requestedFinal ?? fallback;
        Status = status;
        FinishedAt = now;
        _completion.TrySetResult((status, result));
        _cancel.Dispose();
    }
}

/// <summary>
/// Serves a ROS 2 action over the standard <c>_action/send_goal</c>, <c>_action/cancel_goal</c>,
/// <c>_action/get_result</c> services and <c>_action/feedback</c> / <c>_action/status</c> topics, so
/// it interoperates with <c>ros2 action send_goal</c> and rclcpp/rclpy clients.
/// </summary>
public sealed class ActionServer<TGoal, TResult, TFeedback> : IDisposable
    where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
    where TResult : class, IRosMessage<TResult>, new()
    where TFeedback : class, IRosMessage<TFeedback>, new()
{
    private readonly Node _node;
    private readonly Func<ServerGoalHandle<TGoal, TResult, TFeedback>, CancellationToken, Task<TResult>> _execute;
    private readonly ActionServerOptions<TGoal, TResult, TFeedback> _options;
    private readonly ConcurrentDictionary<Guid, ServerGoalHandle<TGoal, TResult, TFeedback>> _goals = new();
    private readonly ServiceServer<SendGoalRequest<TGoal>, SendGoalResponse> _sendGoal;
    private readonly ServiceServer<CancelGoalRequest, CancelGoalResponse> _cancelGoal;
    private readonly ServiceServer<GetResultRequest, GetResultResponse<TResult>> _getResult;
    private readonly IRmwPublisher<FeedbackMessage<TFeedback>> _feedback;
    private readonly Publisher<GoalStatusArray> _status;
    private readonly Timer _expiry;
    private int _disposed;

    internal ActionServer(Node node, string name, Func<ServerGoalHandle<TGoal, TResult, TFeedback>, CancellationToken, Task<TResult>> execute, ActionServerOptions<TGoal, TResult, TFeedback> options)
    {
        _node = node;
        _execute = execute;
        _options = options;
        ActionName = node.ResolveTopicName(name);
        var type = TGoal.ActionTypeName;

        _sendGoal = node.CreateServiceCore<SendGoalRequest<TGoal>, SendGoalResponse>(ActionName + "/_action/send_goal", ActionTypes.SendGoal<TGoal>(type), HandleSendGoal, QosProfile.ServicesDefault, options.CallbackGroup);
        _cancelGoal = node.CreateServiceCore<CancelGoalRequest, CancelGoalResponse>(ActionName + "/_action/cancel_goal", ActionTypes.CancelGoal(), HandleCancel, QosProfile.ServicesDefault, options.CallbackGroup);
        _getResult = node.CreateServiceCore<GetResultRequest, GetResultResponse<TResult>>(ActionName + "/_action/get_result", ActionTypes.GetResult<TResult>(type), HandleGetResult, QosProfile.ServicesDefault, options.CallbackGroup);
        _feedback = node.RmwNode.CreatePublisher<FeedbackMessage<TFeedback>>(new RmwTopicSpec(ActionName + "/_action/feedback", ActionTypes.Feedback<TFeedback>(type), QosProfile.Default, RmwEndpointOptions.Default));
        _status = node.CreatePublisher<GoalStatusArray>(ActionName + "/_action/status", QosProfile.ActionStatus);
        _expiry = new Timer(_ => ExpireResults(), null, TimeSpan.FromSeconds(1), TimeSpan.FromSeconds(1));
    }

    /// <summary>Fully qualified action name.</summary>
    public string ActionName { get; }

    /// <summary>Goals that are accepted, executing or canceling.</summary>
    public IReadOnlyList<ServerGoalHandle<TGoal, TResult, TFeedback>> ActiveGoals =>
        _goals.Values.Where(g => g.IsActive).ToList();

    private ValueTask<SendGoalResponse> HandleSendGoal(SendGoalRequest<TGoal> request, RequestId requestId, CancellationToken cancellationToken)
    {
        var id = ActionTypes.FromUuid(request.GoalId);
        var decision = _goals.ContainsKey(id) ? GoalResponse.Reject : (_options.GoalCallback?.Invoke(id, request.Goal) ?? GoalResponse.Accept);
        var now = _node.Clock.Now();
        if (decision == GoalResponse.Reject)
        {
            return ValueTask.FromResult(new SendGoalResponse { Accepted = false, Stamp = now.ToMsg() });
        }

        var handle = new ServerGoalHandle<TGoal, TResult, TFeedback>(this, id, request.Goal, now);
        _goals[id] = handle;
        PublishStatus();

        // Execute off the executor so long-running goals never block other callbacks.
        _ = Task.Run(() => RunGoalAsync(handle));
        return ValueTask.FromResult(new SendGoalResponse { Accepted = true, Stamp = now.ToMsg() });
    }

    private async Task RunGoalAsync(ServerGoalHandle<TGoal, TResult, TFeedback> handle)
    {
        if (!handle.IsCancelRequested)
        {
            handle.Status = GoalStatus.Executing;
        }

        PublishStatus();
        TResult result;
        GoalStatus status;
        using var linked = CancellationTokenSource.CreateLinkedTokenSource(handle.CancellationToken, _node.Context.ShutdownToken);
        try
        {
            result = await _execute(handle, linked.Token).ConfigureAwait(false) ?? new TResult();
            status = handle.IsCancelRequested ? GoalStatus.Canceled : GoalStatus.Succeeded;
        }
        catch (OperationCanceledException) when (handle.IsCancelRequested || _node.Context.IsShutdown)
        {
            result = new TResult();
            status = handle.IsCancelRequested ? GoalStatus.Canceled : GoalStatus.Aborted;
        }
        catch (Exception ex)
        {
            _node.Logger.LogError(ex, "Action '{Action}' goal {Goal} aborted", ActionName, handle.GoalId);
            result = new TResult();
            status = GoalStatus.Aborted;
        }

        handle.Complete(status, result, _node.Clock.Now());
        PublishStatus();
    }

    private ValueTask<CancelGoalResponse> HandleCancel(CancelGoalRequest request, RequestId requestId, CancellationToken cancellationToken)
    {
        var id = ActionTypes.FromUuid(request.GoalInfo.GoalId);
        var stamp = RosTime.FromMsg(request.GoalInfo.Stamp);
        var response = new CancelGoalResponse();

        IEnumerable<ServerGoalHandle<TGoal, TResult, TFeedback>> candidates;
        if (id == Guid.Empty)
        {
            candidates = stamp.IsZero ? _goals.Values : _goals.Values.Where(g => g.AcceptedAt <= stamp);
        }
        else if (_goals.TryGetValue(id, out var single))
        {
            candidates = stamp.IsZero ? [single] : _goals.Values.Where(g => g.GoalId == id || g.AcceptedAt <= stamp);
        }
        else
        {
            response.ReturnCode = CancelGoalResponse.ERROR_UNKNOWN_GOAL_ID;
            return ValueTask.FromResult(response);
        }

        var list = candidates.ToList();
        var canceling = new List<GoalInfo>();
        var anyActive = false;
        foreach (var g in list)
        {
            if (!g.IsActive)
            {
                continue;
            }

            anyActive = true;
            if ((_options.CancelCallback?.Invoke(g) ?? CancelResponse.Accept) == CancelResponse.Accept)
            {
                g.RequestCancel();
                canceling.Add(new GoalInfo { GoalId = ActionTypes.ToUuid(g.GoalId), Stamp = g.AcceptedAt.ToMsg() });
            }
        }

        response.GoalsCanceling = canceling.ToArray();
        response.ReturnCode = canceling.Count > 0 ? CancelGoalResponse.ERROR_NONE
            : !anyActive && list.Count > 0 ? CancelGoalResponse.ERROR_GOAL_TERMINATED
            : CancelGoalResponse.ERROR_REJECTED;
        if (canceling.Count > 0)
        {
            PublishStatus();
        }

        return ValueTask.FromResult(response);
    }

    private ValueTask<GetResultResponse<TResult>> HandleGetResult(GetResultRequest request, RequestId requestId, CancellationToken cancellationToken)
    {
        var id = ActionTypes.FromUuid(request.GoalId);
        if (!_goals.TryGetValue(id, out var handle))
        {
            return ValueTask.FromResult(new GetResultResponse<TResult> { Status = (sbyte)GoalStatus.Unknown });
        }

        if (handle.Completion.IsCompleted)
        {
            var (status, result) = handle.Completion.Result;
            return ValueTask.FromResult(new GetResultResponse<TResult> { Status = (sbyte)status, Result = result });
        }

        // Defer the response until the goal finishes, without holding the executor.
        _ = handle.Completion.ContinueWith(t =>
        {
            var (status, result) = t.Result;
            _getResult.SendResponse(requestId, new GetResultResponse<TResult> { Status = (sbyte)status, Result = result });
        }, TaskScheduler.Default);
        return ValueTask.FromResult<GetResultResponse<TResult>>(null!);
    }

    internal void PublishFeedback(Guid id, TFeedback feedback)
    {
        if (Volatile.Read(ref _disposed) == 0)
        {
            _feedback.Publish(new FeedbackMessage<TFeedback> { GoalId = ActionTypes.ToUuid(id), Feedback = feedback });
        }
    }

    private void PublishStatus()
    {
        if (Volatile.Read(ref _disposed) != 0)
        {
            return;
        }

        _status.Publish(new GoalStatusArray
        {
            StatusList = _goals.Values
                .OrderBy(g => g.AcceptedAt)
                .Select(g => new Msgs.ActionMsgs.GoalStatus
                {
                    GoalInfo = new GoalInfo { GoalId = ActionTypes.ToUuid(g.GoalId), Stamp = g.AcceptedAt.ToMsg() },
                    Status = (sbyte)g.Status,
                })
                .ToArray(),
        });
    }

    private void ExpireResults()
    {
        var now = _node.Clock.Now();
        var expired = false;
        foreach (var (id, g) in _goals)
        {
            if (g.FinishedAt is { } done && now - done > RosDuration.FromTimeSpan(_options.ResultTimeout))
            {
                expired |= _goals.TryRemove(id, out _);
            }
        }

        if (expired)
        {
            PublishStatus();
        }
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0)
        {
            return;
        }

        _expiry.Dispose();
        _sendGoal.Dispose();
        _cancelGoal.Dispose();
        _getResult.Dispose();
        _feedback.Dispose();
        _status.Dispose();
        _node.Untrack(this);
    }
}
