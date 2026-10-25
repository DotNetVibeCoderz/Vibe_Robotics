using Avalonia;
using Avalonia.Media;
using JointState = ROS2.Net.Msgs.SensorMsgs.JointState;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// One bar per joint of a <c>sensor_msgs/JointState</c>: position within its limits (or ±π),
/// with velocity shown as a tick.
/// </summary>
public sealed class JointStatePanel : RosDrawnControl
{
    private JointState? _state;
    private IDisposable? _sub;

    public JointStatePanel() => Title = "Joint states";

    /// <summary>Joint limits by name; joints without limits use ±π.</summary>
    public Dictionary<string, (double Min, double Max)> Limits { get; } = new(StringComparer.Ordinal);

    public JointState? State
    {
        get => _state;
        set
        {
            _state = value;
            InvalidateVisual();
        }
    }

    public void Attach(Node node, string topic = "/joint_states")
    {
        _sub?.Dispose();
        _sub = RosUi.SubscribeLatest<JointState>(node, topic, s => State = s);
    }

    protected override Size MeasureOverride(Size availableSize) =>
        new(Math.Min(availableSize.Width, 360), 40 + ((_state?.Name.Length ?? 3) * 30));

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        var s = _state;
        if (s is null || s.Name.Length == 0)
        {
            DrawEmpty(context, bounds, "Waiting for /joint_states…");
            return;
        }

        var y = 36.0;
        var labelW = 120.0;
        var barX = labelW + 20;
        var barW = Math.Max(40, bounds.Width - barX - 80);
        for (var i = 0; i < s.Name.Length; i++, y += 30)
        {
            var name = s.Name[i];
            var pos = i < s.Position.Length ? s.Position[i] : 0;
            var (min, max) = Limits.TryGetValue(name, out var l) ? l : (-Math.PI, Math.PI);
            var t = Math.Clamp((pos - min) / Math.Max(1e-9, max - min), 0, 1);

            DrawLabel(context, name.Length > 18 ? name[..18] + "…" : name, new Point(14, y), 11, P.Ink, RosTheme.Mono);
            var bar = new Rect(barX, y + 4, barW, 8);
            context.DrawRectangle(P.Brush(P.Grid), null, bar, 4, 4);
            context.DrawRectangle(P.SignalGradient(), null, new Rect(bar.X, bar.Y, Math.Max(8, bar.Width * t), bar.Height), 4, 4);
            if (i < s.Velocity.Length && Math.Abs(s.Velocity[i]) > 1e-3)
            {
                var vx = bar.X + (bar.Width * t);
                context.DrawLine(P.Pen(P.Alert, 2), new Point(vx, bar.Y - 4), new Point(vx + Math.Clamp(s.Velocity[i] * 10, -20, 20), bar.Y - 4));
            }

            DrawLabel(context, $"{pos,7:0.000}", new Point(bounds.Right - 12, y), 11, P.Haze, RosTheme.Mono, TextAnchor.Right);
        }
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        _sub?.Dispose();
        _sub = null;
        base.OnDetachedFromVisualTree(e);
    }
}
