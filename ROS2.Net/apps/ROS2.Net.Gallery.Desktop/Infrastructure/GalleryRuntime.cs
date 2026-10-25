using ROS2.Net.Rmw.InProcess;

namespace Ros2NetGallery;

/// <summary>
/// The ROS 2 context every gallery page shares: the in-process middleware on a private domain, spun by
/// a multi-threaded executor in the background. Pages create nodes on it and dispose them when the
/// user navigates away.
/// </summary>
public sealed class GalleryRuntime : IDisposable
{
    private readonly CancellationTokenSource _cts = new();
    private readonly Task _spin;

    public GalleryRuntime(int domainId = 87)
    {
        Context = Ros2Context.Create(new Ros2ContextOptions
        {
            DomainId = domainId,
            Rmw = new InProcessRmwFactory(),
        });
        _spin = Context.SpinAsync(_cts.Token, new MultiThreadedExecutor(3));
    }

    public Ros2Context Context { get; }

    public void Dispose()
    {
        _cts.Cancel();
        try
        {
            _spin.Wait(TimeSpan.FromSeconds(2));
        }
        catch (AggregateException)
        {
        }

        Context.Dispose();
    }
}
