using Avalonia.Threading;

namespace ROS2.Net.Avalonia;

/// <summary>
/// Bridges ROS topics to the UI thread. Subscriptions are callback-less (no executor needed); a
/// background loop reads them and hands values to the UI thread. <see cref="SubscribeLatest{T}"/>
/// coalesces: however fast the topic, the UI receives at most one pending update at a time.
/// </summary>
public static class RosUi
{
    /// <summary>Delivers the newest message to <paramref name="onUi"/> on the UI thread, dropping intermediate ones.</summary>
    public static IDisposable SubscribeLatest<T>(Node node, string topic, Action<T> onUi, QosProfile? qos = null)
        where T : class, IRosMessage<T>, new()
    {
        var sub = node.CreateSubscription<T>(topic, qos ?? QosProfile.Default);
        var cts = new CancellationTokenSource();
        T? latest = null;
        var pending = 0;
        _ = Task.Run(async () =>
        {
            try
            {
                await foreach (var msg in sub.ReadAllAsync(cts.Token).ConfigureAwait(false))
                {
                    Volatile.Write(ref latest, msg);
                    if (Interlocked.Exchange(ref pending, 1) == 0)
                    {
                        Dispatcher.UIThread.Post(() =>
                        {
                            Interlocked.Exchange(ref pending, 0);
                            if (!cts.IsCancellationRequested && Volatile.Read(ref latest) is { } m)
                            {
                                onUi(m);
                            }
                        }, DispatcherPriority.Background);
                    }
                }
            }
            catch (OperationCanceledException)
            {
            }
            catch (ObjectDisposedException)
            {
            }
        });
        return new Handle(() =>
        {
            cts.Cancel();
            sub.Dispose();
        });
    }

    /// <summary>Delivers every message, batched, to <paramref name="onUi"/> on the UI thread (logs, events).</summary>
    public static IDisposable SubscribeAll<T>(Node node, string topic, Action<IReadOnlyList<T>> onUi, QosProfile? qos = null)
        where T : class, IRosMessage<T>, new()
    {
        var sub = node.CreateSubscription<T>(topic, qos ?? QosProfile.Default.KeepLast(1000));
        var cts = new CancellationTokenSource();
        var batch = new List<T>();
        var pending = 0;
        _ = Task.Run(async () =>
        {
            try
            {
                await foreach (var msg in sub.ReadAllAsync(cts.Token).ConfigureAwait(false))
                {
                    lock (batch)
                    {
                        batch.Add(msg);
                    }

                    if (Interlocked.Exchange(ref pending, 1) == 0)
                    {
                        Dispatcher.UIThread.Post(() =>
                        {
                            Interlocked.Exchange(ref pending, 0);
                            T[] items;
                            lock (batch)
                            {
                                items = batch.ToArray();
                                batch.Clear();
                            }

                            if (!cts.IsCancellationRequested && items.Length > 0)
                            {
                                onUi(items);
                            }
                        }, DispatcherPriority.Background);
                    }
                }
            }
            catch (OperationCanceledException)
            {
            }
            catch (ObjectDisposedException)
            {
            }
        });
        return new Handle(() =>
        {
            cts.Cancel();
            sub.Dispose();
        });
    }

    /// <summary>Runs <paramref name="action"/> on the UI thread (immediately when already there).</summary>
    public static void OnUi(Action action)
    {
        if (Dispatcher.UIThread.CheckAccess())
        {
            action();
        }
        else
        {
            Dispatcher.UIThread.Post(action);
        }
    }

    private sealed class Handle(Action dispose) : IDisposable
    {
        private Action? _dispose = dispose;

        public void Dispose() => Interlocked.Exchange(ref _dispose, null)?.Invoke();
    }
}
