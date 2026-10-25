using Avalonia;
using Avalonia.Media;
using BatteryState = ROS2.Net.Msgs.SensorMsgs.BatteryState;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>An arc gauge for <c>sensor_msgs/BatteryState</c>: percentage, voltage and charging state.</summary>
public sealed class BatteryGauge : RosDrawnControl
{
    private BatteryState? _state;
    private IDisposable? _sub;

    public BatteryGauge()
    {
        Title = "Battery";
        MinWidth = 140;
        MinHeight = 120;
    }

    /// <summary>Charge in [0, 1].</summary>
    public double Percentage => _state is { } s ? Math.Clamp(s.Percentage, 0, 1) : double.NaN;

    public BatteryState? State
    {
        get => _state;
        set
        {
            _state = value;
            InvalidateVisual();
        }
    }

    public void Attach(Node node, string topic)
    {
        _sub?.Dispose();
        _sub = RosUi.SubscribeLatest<BatteryState>(node, topic, s => State = s);
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        var c = new Point(bounds.Width / 2, (bounds.Height / 2) + 14);
        var r = Math.Max(20, (Math.Min(bounds.Width, bounds.Height - 24) / 2) - 14);
        const double start = Math.PI * 0.8;
        const double sweep = Math.PI * 1.4;
        DrawArc(context, c, r, start, sweep, P.Pen(P.Grid, 10));

        var pct = Percentage;
        if (double.IsNaN(pct))
        {
            DrawEmpty(context, bounds, "No data");
            return;
        }

        var color = pct < 0.15 ? P.Fault : pct < 0.35 ? P.Alert : P.Ok;
        DrawArc(context, c, r, start, sweep * pct, new Pen(new SolidColorBrush(color), 10, lineCap: PenLineCap.Round));
        DrawLabel(context, $"{pct * 100:0}%", c, 22, P.Ink, RosTheme.SansBold, TextAnchor.Center, verticalCenter: true);
        var charging = _state!.PowerSupplyStatus == BatteryState.POWER_SUPPLY_STATUS_CHARGING;
        DrawLabel(context, $"{_state.Voltage:0.0} V{(charging ? "  ⚡ charging" : string.Empty)}", new Point(c.X, c.Y + 16), 10.5, P.Haze, RosTheme.Mono, TextAnchor.Center);
    }

    private static void DrawArc(DrawingContext context, Point c, double r, double start, double sweep, IPen pen)
    {
        if (sweep <= 0.001)
        {
            return;
        }

        var g = new StreamGeometry();
        using (var ctx = g.Open())
        {
            var p0 = new Point(c.X + (r * Math.Cos(start)), c.Y + (r * Math.Sin(start)));
            var p1 = new Point(c.X + (r * Math.Cos(start + sweep)), c.Y + (r * Math.Sin(start + sweep)));
            ctx.BeginFigure(p0, false);
            ctx.ArcTo(p1, new Size(r, r), 0, sweep > Math.PI, SweepDirection.Clockwise);
            ctx.EndFigure(false);
        }

        context.DrawGeometry(null, pen, g);
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        _sub?.Dispose();
        _sub = null;
        base.OnDetachedFromVisualTree(e);
    }
}
