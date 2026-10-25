using Avalonia;
using Avalonia.Media;
using ROS2.Net.Msgs.SensorMsgs;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// Polar view of a <c>sensor_msgs/LaserScan</c>, robot at the centre facing up. Points are coloured
/// by range; range rings are labeled in metres.
/// </summary>
public sealed class LaserScanView : RosDrawnControl
{
    private LaserScan? _scan;
    private IDisposable? _subscription;

    public LaserScanView() => Title = "LiDAR";

    /// <summary>Displayed range [m]; 0 uses the scan's <c>range_max</c>.</summary>
    public double DisplayRange { get; set; }

    public LaserScan? Scan
    {
        get => _scan;
        set
        {
            _scan = value;
            InvalidateVisual();
        }
    }

    /// <summary>Subscribes to a scan topic (sensor-data QoS).</summary>
    public void Attach(Node node, string topic)
    {
        Detach();
        _subscription = RosUi.SubscribeLatest<LaserScan>(node, topic, s => Scan = s, QosProfile.SensorData);
    }

    public void Detach()
    {
        _subscription?.Dispose();
        _subscription = null;
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        var c = new Point(bounds.Width / 2, (bounds.Height / 2) + 8);
        var radius = (Math.Min(bounds.Width, bounds.Height) / 2) - 24;
        var scan = _scan;
        var range = DisplayRange > 0 ? DisplayRange : scan is { RangeMax: > 0 } ? Math.Min(scan.RangeMax, 30) : 4;
        var scale = radius / range;

        // Rings every metre (or a sensible step).
        var step = range <= 5 ? 1 : range <= 12 ? 2 : 5;
        for (var m = step; m <= range + 1e-6; m += step)
        {
            context.DrawEllipse(null, P.Pen(P.Grid, 1), c, m * scale, m * scale);
            DrawLabel(context, $"{m} m", new Point(c.X + 4, c.Y - (m * scale) - 13), 9, P.Haze, RosTheme.Mono);
        }

        context.DrawLine(P.Pen(P.Grid), new Point(c.X - radius, c.Y), new Point(c.X + radius, c.Y));
        context.DrawLine(P.Pen(P.Grid), new Point(c.X, c.Y - radius), new Point(c.X, c.Y + radius));

        if (scan is null || scan.Ranges.Length == 0)
        {
            DrawEmpty(context, bounds, "Waiting for scan…");
            DrawRobot(context, c);
            return;
        }

        var a = scan.AngleMin;
        var brushes = new IBrush[8];
        for (var i = 0; i < brushes.Length; i++)
        {
            brushes[i] = P.Brush(P.Ramp(i / (double)(brushes.Length - 1)));
        }

        var hits = 0;
        for (var i = 0; i < scan.Ranges.Length; i++, a += scan.AngleIncrement)
        {
            var r = scan.Ranges[i];
            if (!float.IsFinite(r) || r < scan.RangeMin || r > scan.RangeMax || r > range)
            {
                continue;
            }

            // Robot frame: x forward (up on screen), y left.
            var px = c.X - (Math.Sin(a) * r * scale);
            var py = c.Y - (Math.Cos(a) * r * scale);
            var t = r / range;
            context.DrawEllipse(brushes[(int)(t * (brushes.Length - 1))], null, new Point(px, py), 1.8, 1.8);
            hits++;
        }

        DrawRobot(context, c);
        DrawPill(context, $"{hits}/{scan.Ranges.Length} pts", new Point(bounds.Right - 10, 8), P.Telemetry);
    }

    private static void DrawRobot(DrawingContext context, Point c)
    {
        var g = new StreamGeometry();
        using (var ctx = g.Open())
        {
            ctx.BeginFigure(new Point(c.X, c.Y - 9), true);
            ctx.LineTo(new Point(c.X + 6, c.Y + 6));
            ctx.LineTo(new Point(c.X - 6, c.Y + 6));
            ctx.EndFigure(true);
        }

        context.DrawGeometry(P.Brush(P.Signal), P.Pen(P.Ink, 1, 0.4), g);
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        Detach();
        base.OnDetachedFromVisualTree(e);
    }
}
