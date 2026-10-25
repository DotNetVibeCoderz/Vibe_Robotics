using Avalonia;
using Avalonia.Media;
using Avalonia.Media.Immutable;
using Avalonia.Styling;

namespace ROS2.Net.Avalonia;

/// <summary>
/// The ROS2.Net design tokens. Colours come from the package icon: a violet→cyan signal on deep
/// navy. Every control in this library draws with the current palette, so switching
/// <see cref="RosTheme.IsDark"/> restyles them all.
/// </summary>
public sealed class RosPalette
{
    private RosPalette(bool dark)
    {
        IsDark = dark;
        if (dark)
        {
            Background = Color.Parse("#0F1020");
            Surface = Color.Parse("#161936");
            SurfaceRaised = Color.Parse("#1D2145");
            Seam = Color.Parse("#2A2F5C");
            Ink = Color.Parse("#E8EAF6");
            Haze = Color.Parse("#8B90B8");
            Grid = Color.Parse("#20244A");
        }
        else
        {
            Background = Color.Parse("#F3F4FB");
            Surface = Color.Parse("#FFFFFF");
            SurfaceRaised = Color.Parse("#F8F8FE");
            Seam = Color.Parse("#DCDFF1");
            Ink = Color.Parse("#15172E");
            Haze = Color.Parse("#5D6290");
            Grid = Color.Parse("#E7E9F6");
        }

        Signal = Color.Parse("#7C4DFF");
        Telemetry = Color.Parse(dark ? "#00D1FF" : "#0096C7");
        Alert = Color.Parse("#FFB547");
        Fault = Color.Parse("#FF5C7A");
        Ok = Color.Parse("#2FD08A");
    }

    public static RosPalette Dark { get; } = new(true);

    public static RosPalette Light { get; } = new(false);

    public bool IsDark { get; }

    /// <summary>Window background.</summary>
    public Color Background { get; }

    /// <summary>Cards and panels.</summary>
    public Color Surface { get; }

    public Color SurfaceRaised { get; }

    /// <summary>Hairlines and borders.</summary>
    public Color Seam { get; }

    /// <summary>Primary text.</summary>
    public Color Ink { get; }

    /// <summary>Secondary text and axes.</summary>
    public Color Haze { get; }

    /// <summary>Plot and map grid lines.</summary>
    public Color Grid { get; }

    /// <summary>Primary accent (violet): selection, commands, the robot.</summary>
    public Color Signal { get; }

    /// <summary>Secondary accent (cyan): live sensor data.</summary>
    public Color Telemetry { get; }

    public Color Alert { get; }

    public Color Fault { get; }

    public Color Ok { get; }

    public IBrush Brush(Color c) => new ImmutableSolidColorBrush(c);

    public IBrush Brush(Color c, double opacity) => new ImmutableSolidColorBrush(c, opacity);

    public IPen Pen(Color c, double thickness = 1, double opacity = 1) =>
        new ImmutablePen(new ImmutableSolidColorBrush(c, opacity), thickness);

    /// <summary>The violet→cyan signal gradient used for accents.</summary>
    public IBrush SignalGradient(RelativePoint? start = null, RelativePoint? end = null) => new LinearGradientBrush
    {
        StartPoint = start ?? new RelativePoint(0, 0, RelativeUnit.Relative),
        EndPoint = end ?? new RelativePoint(1, 0, RelativeUnit.Relative),
        GradientStops = { new GradientStop(Signal, 0), new GradientStop(Telemetry, 1) },
    };

    /// <summary>Colour for a value in [0, 1] along the signal gradient.</summary>
    public Color Ramp(double t)
    {
        t = Math.Clamp(t, 0, 1);
        return Color.FromRgb(
            (byte)(Signal.R + ((Telemetry.R - Signal.R) * t)),
            (byte)(Signal.G + ((Telemetry.G - Signal.G) * t)),
            (byte)(Signal.B + ((Telemetry.B - Signal.B) * t)));
    }
}

/// <summary>Global theme state and typography.</summary>
public static class RosTheme
{
    private static RosPalette s_current = RosPalette.Dark;

    /// <summary>The UI typeface (Inter when the app calls <c>WithInterFont()</c>).</summary>
    public static Typeface Sans { get; } = new(new FontFamily("Inter, Segoe UI, Arial"));

    public static Typeface SansBold { get; } = new(new FontFamily("Inter, Segoe UI, Arial"), FontStyle.Normal, FontWeight.Bold);

    public static Typeface SansSemiBold { get; } = new(new FontFamily("Inter, Segoe UI, Arial"), FontStyle.Normal, FontWeight.SemiBold);

    /// <summary>Monospace for topic names, values and code.</summary>
    public static Typeface Mono { get; } = new(new FontFamily("Cascadia Mono, Cascadia Code, Consolas, Menlo, monospace"));

    public static FontFamily SansFamily { get; } = new("Inter, Segoe UI, Arial");

    public static FontFamily MonoFamily { get; } = new("Cascadia Mono, Cascadia Code, Consolas, Menlo, monospace");

    public static RosPalette Current => s_current;

    public static bool IsDark
    {
        get => s_current.IsDark;
        set
        {
            if (value == s_current.IsDark)
            {
                return;
            }

            s_current = value ? RosPalette.Dark : RosPalette.Light;
            if (Application.Current is { } app)
            {
                app.RequestedThemeVariant = value ? ThemeVariant.Dark : ThemeVariant.Light;
            }

            Changed?.Invoke();
        }
    }

    /// <summary>Raised after the palette changes. Controls invalidate themselves.</summary>
    public static event Action? Changed;
}
