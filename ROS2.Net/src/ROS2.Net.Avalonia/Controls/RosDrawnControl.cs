using System.Globalization;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>Horizontal anchor for <see cref="RosDrawnControl.DrawLabel"/>.</summary>
public enum TextAnchor
{
    Left,
    Center,
    Right,
}

/// <summary>
/// Base of the custom-drawn controls: themed background, redraw on palette changes, and text
/// helpers. Drawing everything in <c>Render</c> keeps the controls fast at sensor rates and free
/// of template dependencies.
/// </summary>
public abstract class RosDrawnControl : Control
{
    public static readonly StyledProperty<string?> TitleProperty =
        AvaloniaProperty.Register<RosDrawnControl, string?>(nameof(Title));

    public static readonly StyledProperty<bool> ShowFrameProperty =
        AvaloniaProperty.Register<RosDrawnControl, bool>(nameof(ShowFrame), true);

    static RosDrawnControl()
    {
        AffectsRender<RosDrawnControl>(TitleProperty, ShowFrameProperty);
    }

    protected RosDrawnControl()
    {
        ClipToBounds = true;
        MinHeight = 60;
        MinWidth = 60;
    }

    /// <summary>Caption drawn in the top-left corner.</summary>
    public string? Title
    {
        get => GetValue(TitleProperty);
        set => SetValue(TitleProperty, value);
    }

    /// <summary>Draw the rounded surface and hairline border.</summary>
    public bool ShowFrame
    {
        get => GetValue(ShowFrameProperty);
        set => SetValue(ShowFrameProperty, value);
    }

    protected static RosPalette P => RosTheme.Current;

    protected override void OnAttachedToVisualTree(VisualTreeAttachmentEventArgs e)
    {
        base.OnAttachedToVisualTree(e);
        RosTheme.Changed += InvalidateVisual;
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        RosTheme.Changed -= InvalidateVisual;
        base.OnDetachedFromVisualTree(e);
    }

    public sealed override void Render(DrawingContext context)
    {
        var bounds = new Rect(Bounds.Size);
        if (ShowFrame)
        {
            context.DrawRectangle(P.Brush(P.Surface), P.Pen(P.Seam), bounds.Deflate(0.5), 10, 10);
        }

        using (context.PushClip(new RoundedRect(bounds.Deflate(1), 10)))
        {
            RenderContent(context, bounds);
        }

        if (!string.IsNullOrEmpty(Title))
        {
            DrawLabel(context, Title!.ToUpperInvariant(), new Point(14, 10), 10.5, P.Haze, RosTheme.SansSemiBold);
        }
    }

    /// <summary>Draws the control's content inside <paramref name="bounds"/> (clipped to the frame).</summary>
    protected abstract void RenderContent(DrawingContext context, Rect bounds);

    /// <summary>Draws a single line of text; returns its size.</summary>
    protected static Size DrawLabel(DrawingContext context, string text, Point origin, double size, Color color, Typeface? typeface = null, TextAnchor anchor = TextAnchor.Left, bool verticalCenter = false)
    {
        var ft = new FormattedText(text, CultureInfo.InvariantCulture, FlowDirection.LeftToRight, typeface ?? RosTheme.Sans, size, new SolidColorBrush(color));
        var x = anchor switch
        {
            TextAnchor.Center => origin.X - (ft.Width / 2),
            TextAnchor.Right => origin.X - ft.Width,
            _ => origin.X,
        };
        var y = verticalCenter ? origin.Y - (ft.Height / 2) : origin.Y;
        context.DrawText(ft, new Point(x, y));
        return new Size(ft.Width, ft.Height);
    }

    /// <summary>Draws a rounded "pill" with text (status chips, values).</summary>
    protected static void DrawPill(DrawingContext context, string text, Point topRight, Color accent)
    {
        var ft = new FormattedText(text, CultureInfo.InvariantCulture, FlowDirection.LeftToRight, RosTheme.Mono, 11, new SolidColorBrush(P.Ink));
        var rect = new Rect(topRight.X - ft.Width - 16, topRight.Y, ft.Width + 16, ft.Height + 6);
        context.DrawRectangle(P.Brush(accent, 0.16), P.Pen(accent, 1, 0.55), rect, rect.Height / 2, rect.Height / 2);
        context.DrawText(ft, new Point(rect.X + 8, rect.Y + 3));
    }

    /// <summary>Message shown when there is no data yet.</summary>
    protected static void DrawEmpty(DrawingContext context, Rect bounds, string text) =>
        DrawLabel(context, text, bounds.Center, 12, P.Haze, RosTheme.Sans, TextAnchor.Center, verticalCenter: true);
}
