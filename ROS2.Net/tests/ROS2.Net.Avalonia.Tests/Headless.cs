using Avalonia;
using Avalonia.Controls;
using Avalonia.Headless;
using Avalonia.Themes.Fluent;
using Avalonia.Threading;

namespace ROS2.Net.Avalonia.Tests;

/// <summary>Application used by the headless tests: Fluent theme, Inter, Skia rendering (real pixels).</summary>
public sealed class TestApp : Application
{
    public override void Initialize()
    {
        Styles.Add(new FluentTheme());
        RequestedThemeVariant = global::Avalonia.Styling.ThemeVariant.Dark;
    }
}

public static class TestEntry
{
    public static AppBuilder BuildAvaloniaApp() =>
        AppBuilder.Configure<TestApp>()
            .UseSkia()
            .UseHeadless(new AvaloniaHeadlessPlatformOptions { UseHeadlessDrawing = false })
            .WithInterFont();
}

/// <summary>One headless session for the whole test assembly (Avalonia allows one platform per process).</summary>
public static class Headless
{
    private static readonly Lazy<HeadlessUnitTestSession> s_session = new(() => HeadlessUnitTestSession.StartNew(typeof(TestEntry)));

    public static T Run<T>(Func<T> action) => s_session.Value.Dispatch(action, CancellationToken.None).GetAwaiter().GetResult();

    public static void Run(Action action) => s_session.Value.Dispatch(action, CancellationToken.None).GetAwaiter().GetResult();

    /// <summary>Pumps the UI thread (timers, posted work, rendering) for roughly <paramref name="ms"/> milliseconds.</summary>
    public static void Pump(int ms)
    {
        // A nested dispatcher frame runs the real loop: posted work, DispatcherTimers and rendering.
        var frame = new DispatcherFrame();
        var ticker = new DispatcherTimer(TimeSpan.FromMilliseconds(16), DispatcherPriority.Render, (_, _) => AvaloniaHeadlessPlatform.ForceRenderTimerTick());
        ticker.Start();
        DispatcherTimer.RunOnce(() => frame.Continue = false, TimeSpan.FromMilliseconds(ms));
        Dispatcher.UIThread.PushFrame(frame);
        ticker.Stop();
        Dispatcher.UIThread.RunJobs();
    }

    /// <summary>Renders a window and writes a PNG.</summary>
    public static void Capture(Window window, string path)
    {
        AvaloniaHeadlessPlatform.ForceRenderTimerTick();
        var frame = window.CaptureRenderedFrame() ?? throw new InvalidOperationException("No frame was rendered.");
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        #pragma warning disable CS0618
        frame.Save(path);
#pragma warning restore CS0618
    }

    /// <summary>The repository root (the folder holding Directory.Build.props).</summary>
    public static string RepoRoot
    {
        get
        {
            var dir = new DirectoryInfo(AppContext.BaseDirectory);
            while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Directory.Build.props")))
            {
                dir = dir.Parent;
            }

            return dir?.FullName ?? throw new DirectoryNotFoundException("Repository root not found.");
        }
    }
}
