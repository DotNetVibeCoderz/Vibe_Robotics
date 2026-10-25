using Avalonia;
using Avalonia.Input;
using Avalonia.Media;
using Log = ROS2.Net.Msgs.RclInterfaces.Log;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// A <c>/rosout</c> console (like <c>rqt_console</c>): newest entries at the bottom, coloured by
/// level, with a minimum-level filter. Scroll with the wheel; new entries re-follow the tail.
/// </summary>
public sealed class LogConsole : RosDrawnControl
{
    private readonly List<(DateTime Time, byte Level, string Name, string Text)> _entries = [];
    private IDisposable? _sub;
    private int _scrollBack;

    public LogConsole() => Title = "Log  /rosout";

    public int Capacity { get; set; } = 500;

    /// <summary>Minimum level shown (rcl_interfaces/Log constants: 10 debug … 50 fatal).</summary>
    public byte MinimumLevel { get; set; } = Log.DEBUG;

    public int Count => _entries.Count;

    public void Attach(Node node, string topic = "/rosout")
    {
        _sub?.Dispose();
        _sub = RosUi.SubscribeAll<Log>(node, topic, batch =>
        {
            foreach (var m in batch)
            {
                Append(RosTime.FromMsg(m.Stamp).ToDateTime().ToLocalTime(), m.Level, m.Name, m.Msg);
            }
        }, QosProfile.Rosout);
    }

    /// <summary>Adds a line (call on the UI thread).</summary>
    public void Append(DateTime time, byte level, string name, string text)
    {
        _entries.Add((time, level, name, text));
        if (_entries.Count > Capacity)
        {
            _entries.RemoveRange(0, _entries.Count - Capacity);
        }

        _scrollBack = 0;
        InvalidateVisual();
    }

    public void Clear()
    {
        _entries.Clear();
        InvalidateVisual();
    }

    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        base.OnPointerWheelChanged(e);
        _scrollBack = Math.Clamp(_scrollBack + (int)(e.Delta.Y * 3), 0, Math.Max(0, _entries.Count - 1));
        InvalidateVisual();
        e.Handled = true;
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        var visible = _entries.Where(e => e.Level >= MinimumLevel).ToList();
        if (visible.Count == 0)
        {
            DrawEmpty(context, bounds, "No log entries yet");
            return;
        }

        const double line = 18;
        var rows = (int)((bounds.Height - 34) / line);
        var end = Math.Max(0, visible.Count - _scrollBack);
        var start = Math.Max(0, end - rows);
        var y = 32.0;
        for (var i = start; i < end; i++, y += line)
        {
            var (time, level, name, text) = visible[i];
            var (tag, color) = level switch
            {
                >= Log.FATAL => ("FATAL", P.Fault),
                >= Log.ERROR => ("ERROR", P.Fault),
                >= Log.WARN => ("WARN ", P.Alert),
                >= Log.INFO => ("INFO ", P.Telemetry),
                _ => ("DEBUG", P.Haze),
            };
            DrawLabel(context, time.ToString("HH:mm:ss.fff"), new Point(14, y), 10.5, P.Haze, RosTheme.Mono);
            DrawLabel(context, tag, new Point(104, y), 10.5, color, RosTheme.Mono);
            var n = DrawLabel(context, $"[{name}]", new Point(150, y), 10.5, P.Signal, RosTheme.Mono);
            DrawLabel(context, text.ReplaceLineEndings(" ⏎ "), new Point(156 + n.Width, y), 10.5, P.Ink, RosTheme.Mono);
        }

        if (_scrollBack > 0)
        {
            DrawPill(context, $"↑ {_scrollBack} newer", new Point(bounds.Right - 10, 6), P.Alert);
        }
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        _sub?.Dispose();
        _sub = null;
        base.OnDetachedFromVisualTree(e);
    }
}
