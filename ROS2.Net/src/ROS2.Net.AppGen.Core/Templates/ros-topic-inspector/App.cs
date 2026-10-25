using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Styling;
using Avalonia.Themes.Fluent;
using ROS2.Net;
using ROS2.Net.Rmw.Rtps;

namespace {{Namespace}};

/// <summary>Application: joins the ROS 2 network over RTPS (set ROS_DOMAIN_ID to pick the domain).</summary>
public sealed class App : Application
{
    private Ros2Context? _context;
    private readonly CancellationTokenSource _spin = new();

    public override void Initialize()
    {
        Styles.Add(new FluentTheme());
        RequestedThemeVariant = ThemeVariant.Dark;
    }

    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            // Inspectors default to the network (RTPS) unless ROS2NET_RMW chooses otherwise.
            var options = AppTarget.ContextOptions(desktop.Args);
            if (string.IsNullOrEmpty(Environment.GetEnvironmentVariable("ROS2NET_RMW")))
            {
                options.UseRtps();
            }

            _context = Ros2Context.Create(options);
            desktop.MainWindow = new MainWindow(_context);
            _ = _context.SpinAsync(_spin.Token);
            desktop.ShutdownRequested += (_, _) => _spin.Cancel();
            desktop.Exit += (_, _) => _context.Dispose();
        }

        base.OnFrameworkInitializationCompleted();
    }
}
