using Avalonia;
using Avalonia.Media;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// A real-time line plot (like <c>rqt_plot</c>): several named series over a sliding time window,
/// auto-scaled. Add samples from any thread with <see cref="AddSample"/>; the plot redraws at most
/// once per frame.
/// </summary>
public sealed class PlotView : RosDrawnControl
{
    private readonly object _gate = new();
    private readonly Dictionary<string, Series> _series = new(StringComparer.Ordinal);
    private int _invalidatePending;

    private sealed class Series(Color color)
    {
        public Color Color { get; } = color;
        public Queue<(double T, double V)> Points { get; } = new();
    }

    public PlotView() => Title = "Plot";

    /// <summary>Visible time window in seconds.</summary>
    public double WindowSeconds { get; set; } = 10;

    /// <summary>Fixed Y range; null auto-scales.</summary>
    public (double Min, double Max)? YRange { get; set; }

    public string Unit { get; set; } = string.Empty;

    /// <summary>Unit of the X axis (time in seconds by default; e.g. "ep" for training episodes).</summary>
    public string XUnit { get; set; } = "s";

    /// <summary>Adds a sample; <paramref name="t"/> in seconds (any monotonic origin).</summary>
    public void AddSample(string series, double t, double value)
    {
        lock (_gate)
        {
            if (!_series.TryGetValue(series, out var s))
            {
                var palette = new[] { P.Telemetry, P.Signal, P.Alert, P.Ok, P.Fault, P.Haze };
                s = _series[series] = new Series(palette[_series.Count % palette.Length]);
            }

            s.Points.Enqueue((t, value));
            while (s.Points.Count > 0 && s.Points.Peek().T < t - (WindowSeconds * 1.5))
            {
                s.Points.Dequeue();
            }
        }

        if (Interlocked.Exchange(ref _invalidatePending, 1) == 0)
        {
            RosUi.OnUi(() =>
            {
                Interlocked.Exchange(ref _invalidatePending, 0);
                InvalidateVisual();
            });
        }
    }

    public void Clear()
    {
        lock (_gate)
        {
            _series.Clear();
        }

        InvalidateVisual();
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        var plot = new Rect(bounds.X + 48, bounds.Y + 34, Math.Max(10, bounds.Width - 62), Math.Max(10, bounds.Height - 58));
        List<(string Name, Color Color, (double T, double V)[] Pts)> snapshot;
        lock (_gate)
        {
            snapshot = _series.Select(kv => (kv.Key, kv.Value.Color, kv.Value.Points.ToArray())).ToList();
        }

        if (snapshot.Count == 0 || snapshot.All(s => s.Pts.Length == 0))
        {
            DrawEmpty(context, bounds, "Waiting for samples…");
            return;
        }

        var tMax = snapshot.Where(s => s.Pts.Length > 0).Max(s => s.Pts[^1].T);
        var tMin = tMax - WindowSeconds;
        double vMin, vMax;
        if (YRange is { } yr)
        {
            (vMin, vMax) = yr;
        }
        else
        {
            vMin = double.MaxValue;
            vMax = double.MinValue;
            foreach (var s in snapshot)
            {
                foreach (var (t, v) in s.Pts)
                {
                    if (t >= tMin && double.IsFinite(v))
                    {
                        vMin = Math.Min(vMin, v);
                        vMax = Math.Max(vMax, v);
                    }
                }
            }

            if (vMin > vMax)
            {
                vMin = -1;
                vMax = 1;
            }

            var pad = Math.Max(1e-6, (vMax - vMin) * 0.12);
            vMin -= pad;
            vMax += pad;
        }

        // Grid: 4 horizontal lines with labels.
        for (var i = 0; i <= 4; i++)
        {
            var y = plot.Bottom - (plot.Height * i / 4);
            context.DrawLine(P.Pen(P.Grid), new Point(plot.Left, y), new Point(plot.Right, y));
            var v = vMin + ((vMax - vMin) * i / 4);
            DrawLabel(context, Fmt(v), new Point(plot.Left - 6, y), 9.5, P.Haze, RosTheme.Mono, TextAnchor.Right, verticalCenter: true);
        }

        DrawLabel(context, XUnit == "s" ? $"−{WindowSeconds:0}s" : $"{tMin:0} {XUnit}", new Point(plot.Left, plot.Bottom + 4), 9.5, P.Haze, RosTheme.Mono);
        DrawLabel(context, XUnit == "s" ? "now" : $"{tMax:0} {XUnit}", new Point(plot.Right, plot.Bottom + 4), 9.5, P.Haze, RosTheme.Mono, TextAnchor.Right);

        using (context.PushClip(plot))
        {
            foreach (var (_, color, pts) in snapshot)
            {
                if (pts.Length < 2)
                {
                    continue;
                }

                var geo = new StreamGeometry();
                using (var ctx = geo.Open())
                {
                    var started = false;
                    foreach (var (t, v) in pts)
                    {
                        if (t < tMin - 1 || !double.IsFinite(v))
                        {
                            continue;
                        }

                        var p = new Point(plot.Left + ((t - tMin) / WindowSeconds * plot.Width), plot.Bottom - ((v - vMin) / (vMax - vMin) * plot.Height));
                        if (!started)
                        {
                            ctx.BeginFigure(p, false);
                            started = true;
                        }
                        else
                        {
                            ctx.LineTo(p);
                        }
                    }

                    if (started)
                    {
                        ctx.EndFigure(false);
                    }
                }

                context.DrawGeometry(null, new Pen(new SolidColorBrush(color), 2, lineJoin: PenLineJoin.Round), geo);
            }
        }

        // Legend with the latest values.
        var x = bounds.Right - 10;
        foreach (var (name, color, pts) in snapshot.AsEnumerable().Reverse())
        {
            var label = $"{name} {(pts.Length > 0 ? Fmt(pts[^1].V) : "–")}{Unit}";
            var size = DrawLabelRight(context, label, new Point(x, 10), color);
            x -= size + 16;
        }
    }

    private static double DrawLabelRight(DrawingContext context, string text, Point topRight, Color color)
    {
        var s = DrawLabel(context, text, new Point(topRight.X, topRight.Y), 10.5, P.Ink, RosTheme.Mono, TextAnchor.Right);
        context.DrawEllipse(new SolidColorBrush(color), null, new Point(topRight.X - s.Width - 8, topRight.Y + (s.Height / 2)), 4, 4);
        return s.Width + 12;
    }

    private static string Fmt(double v) => Math.Abs(v) >= 1000 ? v.ToString("0") : Math.Abs(v) >= 10 ? v.ToString("0.0") : v.ToString("0.00");
}
