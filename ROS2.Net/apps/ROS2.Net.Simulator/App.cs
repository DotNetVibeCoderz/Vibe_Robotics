using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Styling;
using Avalonia.Themes.Fluent;

namespace Ros2NetSimulator;

/// <summary>The ROS2.Net Robot Simulator application.</summary>
public sealed class App : Application
{
    public override void Initialize()
    {
        Styles.Add(new FluentTheme());
        RequestedThemeVariant = ThemeVariant.Dark;
    }

    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            var args = desktop.Args ?? [];
            var rmw = args.SkipWhile(a => a != "--rmw").Skip(1).FirstOrDefault() ?? Environment.GetEnvironmentVariable("ROS2NET_RMW") ?? "inprocess";
            var scenario = args.SkipWhile(a => a != "--scenario").Skip(1).FirstOrDefault() ?? "Warehouse";
            desktop.MainWindow = new SimulatorWindow(rmw, scenario);
        }

        base.OnFrameworkInitializationCompleted();
    }
}

internal static class Program
{
    /// <summary>
    /// <c>ROS2.Net.Simulator [--rmw inprocess|rtps|rosbridge] [--scenario Warehouse|Office|Maze|CorridorLoop|AgricultureRows|Empty]</c>.
    /// With <c>--rmw rtps</c> the simulated robots are visible to ROS 2 nodes on the network.
    /// </summary>
    [STAThread]
    public static void Main(string[] args) =>
        BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);

    public static AppBuilder BuildAvaloniaApp() =>
        AppBuilder.Configure<App>()
            .UsePlatformDetect()
            .WithInterFont()
            .LogToTrace();
}
