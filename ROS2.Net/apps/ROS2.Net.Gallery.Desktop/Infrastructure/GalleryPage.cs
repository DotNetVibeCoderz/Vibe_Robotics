using Avalonia.Controls;

namespace Ros2NetGallery;

/// <summary>Sidebar group of a page; the order is the order of the ROS stack, bottom-up.</summary>
public enum PageGroup
{
    Overview,
    Communication,
    Robotics,
    Intelligence,
    Tools,
}

/// <summary>
/// A gallery use case: a live demo, the code that drives it, and the ROS interfaces it touches
/// (shown as chips in the header — the page's "wire signature").
/// </summary>
public abstract class GalleryPage : IDisposable
{
    private readonly List<IDisposable> _owned = [];

    /// <summary>Stable id, used for screenshots (<c>gallery-&lt;id&gt;.png</c>).</summary>
    public abstract string Id { get; }

    public abstract PageGroup Group { get; }

    public abstract T2 Title { get; }

    public abstract T2 Summary { get; }

    /// <summary>ROS interfaces and topics this demo uses.</summary>
    public virtual IReadOnlyList<string> Interfaces => [];

    /// <summary>The snippet shown in the code panel.</summary>
    public virtual string Code => string.Empty;

    /// <summary>Builds the live demo.</summary>
    public abstract Control Build(GalleryRuntime runtime);

    /// <summary>Registers something to dispose when the page is left.</summary>
    protected T Own<T>(T disposable)
        where T : IDisposable
    {
        _owned.Add(disposable);
        return disposable;
    }

    protected void OwnAction(Action dispose) => _owned.Add(new ActionDisposable(dispose));

    /// <summary>Releases the page's nodes, timers and subscriptions.</summary>
    public virtual void Dispose()
    {
        for (var i = _owned.Count - 1; i >= 0; i--)
        {
            try
            {
                _owned[i].Dispose();
            }
            catch (ObjectDisposedException)
            {
            }
        }

        _owned.Clear();
        GC.SuppressFinalize(this);
    }

    private sealed class ActionDisposable(Action a) : IDisposable
    {
        public void Dispose() => a();
    }
}
