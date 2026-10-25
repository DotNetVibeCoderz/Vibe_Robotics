// Ros2NetAvalonia — a desktop ROS 2 app built with ROS2.Net and Avalonia
// (made by Gravicode Studios, led by Kang Fadhil).
using Avalonia;

namespace Ros2NetAvalonia;

internal static class Program
{
    [STAThread]
    public static void Main(string[] args) => BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);

    public static AppBuilder BuildAvaloniaApp() => AppBuilder.Configure<App>()
        .UsePlatformDetect()
        .WithInterFont()
        .LogToTrace();
}
