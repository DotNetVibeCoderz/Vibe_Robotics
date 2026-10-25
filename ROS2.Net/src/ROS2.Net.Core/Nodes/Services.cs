using System.Collections.Concurrent;
using ROS2.Net.Rmw;

namespace ROS2.Net;

/// <summary>Raised when a service call does not get a response in time.</summary>
public sealed class ServiceTimeoutException(string service, TimeSpan timeout)
    : TimeoutException($"Service '{service}' did not respond within {timeout.TotalSeconds:0.###} s.");

/// <summary>Raised when a service has no server.</summary>
public sealed class ServiceNotAvailableException(string service)
    : InvalidOperationException($"Service '{service}' is not available.");

/// <summary>
/// Serves requests on a service. Requests are queued and executed on the node's executor; the
/// handler may be asynchronous.
/// </summary>
public sealed class ServiceServer<TRequest, TResponse> : Waitable
    where TRequest : class, IRosMessage, new()
    where TResponse : class, IRosMessage, new()
{
    private readonly Func<TRequest, RequestId, CancellationToken, ValueTask<TResponse>> _handler;
    private readonly ConcurrentQueue<(RequestId Id, TRequest Request)> _queue = new();
    private readonly IRmwServiceServer<TResponse> _rmw;

    internal ServiceServer(Node node, string serviceName, ServiceTypeInfo type, QosProfile qos, Func<TRequest, RequestId, CancellationToken, ValueTask<TResponse>> handler, CallbackGroup? group)
        : base(node, group)
    {
        ServiceName = serviceName;
        Type = type;
        _handler = handler;
        _rmw = node.RmwNode.CreateServiceServer<TRequest, TResponse>(new RmwServiceSpec(serviceName, type, qos), OnRequest);
    }

    /// <summary>Fully qualified service name.</summary>
    public string ServiceName { get; }

    public ServiceTypeInfo Type { get; }

    protected internal override bool HasWork => !_queue.IsEmpty;

    private void OnRequest(RequestId id, TRequest request)
    {
        _queue.Enqueue((id, request));
        Signal();
    }

    protected internal override async ValueTask ExecuteNextAsync()
    {
        if (!_queue.TryDequeue(out var item))
        {
            return;
        }

        if (!_queue.IsEmpty)
        {
            Signal();
        }

        var response = await _handler(item.Request, item.Id, Node.Context.ShutdownToken).ConfigureAwait(false);

        // A null response defers the reply: the handler keeps the RequestId and calls SendResponse later.
        if (response is not null)
        {
            _rmw.SendResponse(item.Id, response);
        }
    }

    /// <summary>Sends a response for a request whose handler deferred its reply by returning null.</summary>
    public void SendResponse(RequestId requestId, TResponse response) => _rmw.SendResponse(requestId, response);

    protected override void DisposeCore() => _rmw.Dispose();
}

/// <summary>
/// Calls a service. Unlike rclcpp, responses complete the returned task directly: awaiting a call
/// does not require the node to be spinning.
/// </summary>
public sealed class ServiceClient<TRequest, TResponse> : IDisposable
    where TRequest : class, IRosMessage, new()
    where TResponse : class, IRosMessage, new()
{
    private readonly Node _node;
    private readonly IRmwServiceClient<TRequest> _rmw;
    private readonly ConcurrentDictionary<long, TaskCompletionSource<TResponse>> _pending = new();
    private readonly ConcurrentDictionary<long, TResponse> _early = new();
    private readonly object _sendGate = new();
    private int _disposed;

    internal ServiceClient(Node node, string serviceName, ServiceTypeInfo type, QosProfile qos)
    {
        _node = node;
        ServiceName = serviceName;
        Type = type;
        _rmw = node.RmwNode.CreateServiceClient<TRequest, TResponse>(new RmwServiceSpec(serviceName, type, qos), OnResponse);
    }

    /// <summary>Fully qualified service name.</summary>
    public string ServiceName { get; }

    public ServiceTypeInfo Type { get; }

    /// <summary>True when a server has been discovered.</summary>
    public bool IsServiceReady => _rmw.IsServiceAvailable;

    /// <summary>Requests still waiting for a response.</summary>
    public int PendingRequestCount => _pending.Count;

    /// <summary>Waits until a server is available. Returns false on timeout.</summary>
    public async Task<bool> WaitForServiceAsync(TimeSpan? timeout = null, CancellationToken cancellationToken = default)
    {
        var start = Environment.TickCount64;
        var delay = 5;
        while (!_rmw.IsServiceAvailable)
        {
            if (timeout is { } t && Environment.TickCount64 - start > t.TotalMilliseconds)
            {
                return false;
            }

            await Task.Delay(delay, cancellationToken).ConfigureAwait(false);
            delay = Math.Min(delay * 2, 100);
        }

        return true;
    }

    /// <summary>Sends a request and awaits the response.</summary>
    /// <param name="request">The request.</param>
    /// <param name="timeout">Give up after this long (throws <see cref="ServiceTimeoutException"/>).</param>
    /// <param name="cancellationToken">Cancels the wait; the request itself cannot be recalled.</param>
    public async Task<TResponse> CallAsync(TRequest request, TimeSpan? timeout = null, CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(Volatile.Read(ref _disposed) != 0, this);
        var tcs = new TaskCompletionSource<TResponse>(TaskCreationOptions.RunContinuationsAsynchronously);
        long seq;
        lock (_sendGate)
        {
            seq = _rmw.SendRequest(request);
            if (_early.TryRemove(seq, out var early))
            {
                return early;
            }

            _pending[seq] = tcs;
        }

        try
        {
            using var linked = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, _node.Context.ShutdownToken);
            if (timeout is { } t)
            {
                try
                {
                    return await tcs.Task.WaitAsync(t, linked.Token).ConfigureAwait(false);
                }
                catch (TimeoutException)
                {
                    throw new ServiceTimeoutException(ServiceName, t);
                }
            }

            return await tcs.Task.WaitAsync(linked.Token).ConfigureAwait(false);
        }
        finally
        {
            _pending.TryRemove(seq, out _);
        }
    }

    private void OnResponse(long sequence, TResponse response)
    {
        lock (_sendGate)
        {
            if (_pending.TryRemove(sequence, out var tcs))
            {
                tcs.TrySetResult(response);
            }
            else
            {
                // Response raced ahead of registration (synchronous middlewares), or a duplicate from a second server.
                _early.TryAdd(sequence, response);
                if (_early.Count > 1024)
                {
                    _early.Clear();
                }
            }
        }
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0)
        {
            return;
        }

        _rmw.Dispose();
        foreach (var p in _pending.Values)
        {
            p.TrySetCanceled();
        }

        _pending.Clear();
        _node.Untrack(this);
    }
}
