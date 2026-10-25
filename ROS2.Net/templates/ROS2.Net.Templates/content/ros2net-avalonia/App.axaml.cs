using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Markup.Xaml;
using ROS2.Net;
using ROS2.Net.Rmw.Rtps;

namespace Ros2NetAvalonia;

public partial class App : Application
{
    private Ros2Context? _context;
    private CancellationTokenSource? _spin;

    public override void Initialize() => AvaloniaXamlLoader.Load(this);

    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            // One ROS context for the app; ROS2NET_RMW=inprocess|rosbridge overrides the RTPS default.
            var options = new Ros2ContextOptions { Arguments = desktop.Args };
            if (string.IsNullOrEmpty(Environment.GetEnvironmentVariable("ROS2NET_RMW")))
            {
                options.UseRtps();
            }

            _context = Ros2Context.Create(options);
            var robot = new RobotLink(_context);
            _spin = new CancellationTokenSource();
            _ = _context.SpinAsync(_spin.Token);

            desktop.MainWindow = new MainWindow(robot);
            desktop.ShutdownRequested += (_, _) =>
            {
                _spin.Cancel();
                _context.Dispose();
            };
        }

        base.OnFrameworkInitializationCompleted();
    }
}
