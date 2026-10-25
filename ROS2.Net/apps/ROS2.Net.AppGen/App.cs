using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Markup.Xaml.Styling;
using Avalonia.Styling;
using Avalonia.Themes.Fluent;
using ROS2.Net.AppGen;
using Ros2AppGen.Ui;

namespace Ros2AppGen;

/// <summary>The application. Styles are set up in code; there is no XAML of our own.</summary>
public sealed class App : Application
{
    /// <summary>Host override for tests; the real app creates one from App.config.</summary>
    public static Func<AppGenHost>? HostFactory { get; set; }

    public override void Initialize() => AddStyles(this);

    /// <summary>Fluent theme plus AvaloniaEdit's control templates.</summary>
    public static void AddStyles(Application app)
    {
        app.Styles.Add(new FluentTheme());
        app.Styles.Add(new StyleInclude(new Uri("avares://ROS2AppGen/"))
        {
            Source = new Uri("avares://AvaloniaEdit/Themes/Fluent/AvaloniaEdit.xaml"),
        });
        app.RequestedThemeVariant = Look.IsDark ? ThemeVariant.Dark : ThemeVariant.Light;
    }

    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            var host = (HostFactory ?? (() => HostBootstrap.Create()))();
            var window = new MainWindow(host);
            desktop.MainWindow = window;
            var args = desktop.Args ?? [];
            if (args.Length > 0 && (Directory.Exists(args[0]) || File.Exists(args[0])))
            {
                window.Opened += (_, _) => window.OpenProject(args[0]);
            }
        }

        base.OnFrameworkInitializationCompleted();
    }
}

internal static class Program
{
    [STAThread]
    public static void Main(string[] args) =>
        BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);

    public static AppBuilder BuildAvaloniaApp() =>
        AppBuilder.Configure<App>()
            .UsePlatformDetect()
            .WithInterFont()
            .LogToTrace();
}
