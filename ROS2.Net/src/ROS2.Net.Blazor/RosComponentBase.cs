using Microsoft.AspNetCore.Components;
using ROS2.Net.Rmw;

namespace ROS2.Net.Blazor;

/// <summary>
/// Base class for components bound to ROS 2: owns a private node, subscribes without needing an
/// executor, and coalesces re-renders to at most <see cref="RefreshHz"/> per second so a 100 Hz topic
/// never floods the circuit.
/// </summary>
public abstract class RosComponentBase : ComponentBase, IAsyncDisposable
{
    private static long s_counter;
    private readonly CancellationTokenSource _cts = new();
    private Node? _node;
    private Timer? _renderTimer;
    private int _dirty;
    private int _disposed;

    [Inject]
    protected Ros2Context Context { get; set; } = default!;

    /// <summary>Maximum re-render rate.</summary>
    [Parameter]
    public double RefreshHz { get; set; } = 30;

    /// <summary>Cancelled when the component is disposed.</summary>
    protected CancellationToken Disposing => _cts.Token;

    /// <summary>The component's own node, created on first use.</summary>
    protected Node Node => _node ??= Context.CreateNode($"web_{GetType().Name.ToLowerInvariant().Replace('`', '_')}_{Interlocked.Increment(ref s_counter)}",
        options: new NodeOptions { StartParameterServices = false, EnableRosout = false });

    /// <summary>Subscribes to a topic; <paramref name="onMessage"/> runs on a background task, then a throttled render is requested.</summary>
    protected Subscription<T> Subscribe<T>(string topic, Action<T> onMessage, QosProfile? qos = null)
        where T : class, IRosMessage<T>, new()
    {
        var sub = Node.CreateSubscription<T>(topic, qos ?? QosProfile.Default);
        _ = Task.Run(async () =>
        {
            try
            {
                await foreach (var m in sub.ReadAllAsync(_cts.Token).ConfigureAwait(false))
                {
                    onMessage(m);
                    RequestRender();
                }
            }
            catch (OperationCanceledException)
            {
            }
        });
        return sub;
    }

    /// <summary>Raw CDR subscription for runtime-typed topics (monitors, plots). The handler runs on the middleware thread.</summary>
    protected SerializedSubscription SubscribeSerialized(string topic, MessageTypeInfo type, RmwSerializedHandler handler, QosProfile? qos = null) =>
        Node.CreateSerializedSubscription(topic, type, handler, qos ?? QosProfile.Default);

    /// <summary>Asks for a re-render, coalesced to <see cref="RefreshHz"/>.</summary>
    protected void RequestRender()
    {
        if (Volatile.Read(ref _disposed) != 0 || Interlocked.Exchange(ref _dirty, 1) != 0)
        {
            return;
        }

        var period = TimeSpan.FromSeconds(1.0 / Math.Clamp(RefreshHz, 1, 120));
        _renderTimer ??= new Timer(_ => Flush(), null, Timeout.Infinite, Timeout.Infinite);
        _renderTimer.Change(period, Timeout.InfiniteTimeSpan);
    }

    private void Flush()
    {
        if (Volatile.Read(ref _disposed) != 0)
        {
            return;
        }

        Interlocked.Exchange(ref _dirty, 0);
        _ = InvokeAsync(StateHasChanged);
    }

    /// <summary>Runs <paramref name="work"/> every <paramref name="period"/> until disposal, then re-renders.</summary>
    protected void Every(TimeSpan period, Func<CancellationToken, Task> work)
    {
        _ = Task.Run(async () =>
        {
            using var timer = new PeriodicTimer(period);
            try
            {
                do
                {
                    try
                    {
                        await work(_cts.Token).ConfigureAwait(false);
                    }
                    catch (Exception ex) when (ex is not OperationCanceledException)
                    {
                        OnBackgroundError(ex);
                    }

                    RequestRender();
                }
                while (await timer.WaitForNextTickAsync(_cts.Token).ConfigureAwait(false));
            }
            catch (OperationCanceledException)
            {
            }
        });
    }

    /// <summary>Called when a background job throws. Default: keep going.</summary>
    protected virtual void OnBackgroundError(Exception exception)
    {
    }

    protected virtual ValueTask DisposeCoreAsync() => ValueTask.CompletedTask;

    public async ValueTask DisposeAsync()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0)
        {
            return;
        }

        _cts.Cancel();
        await DisposeCoreAsync().ConfigureAwait(false);
        if (_renderTimer is not null)
        {
            await _renderTimer.DisposeAsync().ConfigureAwait(false);
        }

        _node?.Dispose();
        _cts.Dispose();
        GC.SuppressFinalize(this);
    }
}
