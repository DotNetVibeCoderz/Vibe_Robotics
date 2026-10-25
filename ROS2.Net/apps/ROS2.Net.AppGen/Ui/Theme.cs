using Avalonia.Media;

namespace Ros2AppGen.Ui;

/// <summary>Color and type tokens. The palette is the ROS2.Net mark: violet "signal" to cyan "telemetry" over a deep navy hull.</summary>
public sealed record Palette(
    Color Background,
    Color Surface,
    Color Raised,
    Color Sunken,
    Color Seam,
    Color Ink,
    Color Haze,
    Color Faint,
    Color Signal,
    Color Telemetry,
    Color Ok,
    Color Warn,
    Color Fault,
    string EditorTheme)
{
    public static Palette Dark { get; } = new(
        Background: Color.Parse("#0F1020"),
        Surface: Color.Parse("#14162B"),
        Raised: Color.Parse("#1C1F3B"),
        Sunken: Color.Parse("#0B0C19"),
        Seam: Color.Parse("#262A4D"),
        Ink: Color.Parse("#E6E8F6"),
        Haze: Color.Parse("#8E93C2"),
        Faint: Color.Parse("#5A5F8C"),
        Signal: Color.Parse("#7C4DFF"),
        Telemetry: Color.Parse("#00D1FF"),
        Ok: Color.Parse("#3DDC97"),
        Warn: Color.Parse("#FFB547"),
        Fault: Color.Parse("#FF5C7A"),
        EditorTheme: "dark");

    public static Palette Light { get; } = new(
        Background: Color.Parse("#F4F5FB"),
        Surface: Color.Parse("#FFFFFF"),
        Raised: Color.Parse("#ECEEF8"),
        Sunken: Color.Parse("#F9FAFE"),
        Seam: Color.Parse("#D8DBEC"),
        Ink: Color.Parse("#171A38"),
        Haze: Color.Parse("#5B5F8E"),
        Faint: Color.Parse("#9296BC"),
        Signal: Color.Parse("#6A3DF0"),
        Telemetry: Color.Parse("#0093C4"),
        Ok: Color.Parse("#16A06A"),
        Warn: Color.Parse("#C27C0E"),
        Fault: Color.Parse("#D63A5A"),
        EditorTheme: "light");
}

/// <summary>The active theme. Views read it when they are (re)built.</summary>
public static class Look
{
    public static bool IsDark { get; set; } = true;

    public static Palette P => IsDark ? Palette.Dark : Palette.Light;

    public static FontFamily Sans { get; } = new("fonts:Inter#Inter, $Default");

    public static FontFamily Mono { get; } = new("Cascadia Code, JetBrains Mono, Consolas, Menlo, monospace");

    public static IBrush Brush(Color c, double opacity = 1) => new SolidColorBrush(c, opacity);

    /// <summary>The signature gradient, violet to cyan.</summary>
    public static IBrush SignalGradient(double angle = 0)
    {
        var horizontal = angle == 0;
        return new LinearGradientBrush
        {
            StartPoint = new Avalonia.RelativePoint(0, 0, Avalonia.RelativeUnit.Relative),
            EndPoint = horizontal ? new Avalonia.RelativePoint(1, 0, Avalonia.RelativeUnit.Relative) : new Avalonia.RelativePoint(0, 1, Avalonia.RelativeUnit.Relative),
            GradientStops = { new GradientStop(P.Signal, 0), new GradientStop(P.Telemetry, 1) },
        };
    }
}
