using Avalonia;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.Tf2;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// The TF frame tree (like <c>view_frames</c> / <c>rqt_tf_tree</c>) of a <see cref="TfBuffer"/>, laid out
/// top-down, with each child's translation from its parent. Refreshes once per second.
/// </summary>
public sealed class TfTreeView : RosDrawnControl
{
    private TfBuffer? _buffer;
    private DispatcherTimer? _timer;
    private List<(string Frame, string? Parent, int Depth, int Row, string Detail)> _layout = [];

    public TfTreeView() => Title = "TF tree";

    public TfBuffer? Buffer
    {
        get => _buffer;
        set
        {
            _buffer = value;
            _timer?.Stop();
            if (value is not null)
            {
                _timer = new DispatcherTimer(TimeSpan.FromSeconds(1), DispatcherPriority.Background, (_, _) => Refresh());
                _timer.Start();
            }

            Refresh();
        }
    }

    public int FrameCount => _layout.Count;

    public void Refresh()
    {
        var b = _buffer;
        var layout = new List<(string, string?, int, int, string)>();
        if (b is not null)
        {
            var frames = b.FrameNames;
            var parents = frames.ToDictionary(f => f, f => b.GetParent(f), StringComparer.Ordinal);
            var children = frames.GroupBy(f => parents[f] ?? string.Empty).ToDictionary(g => g.Key, g => g.Order(StringComparer.Ordinal).ToList());
            var roots = frames.Where(f => parents[f] is null || !parents.ContainsKey(parents[f]!)).Order(StringComparer.Ordinal).ToList();
            var row = 0;
            var seen = new HashSet<string>(StringComparer.Ordinal);
            void Visit(string f, string? parent, int depth)
            {
                if (!seen.Add(f))
                {
                    return;
                }

                var detail = string.Empty;
                if (parent is not null && b.TryLookupTransform(parent, f, RosTime.Zero, out var t, out _))
                {
                    detail = $"({t.Translation.X:0.00}, {t.Translation.Y:0.00}, {t.Translation.Z:0.00})";
                }

                layout.Add((f, parent, depth, row++, detail));
                if (children.TryGetValue(f, out var kids))
                {
                    foreach (var k in kids)
                    {
                        Visit(k, f, depth + 1);
                    }
                }
            }

            foreach (var r in roots)
            {
                // A root whose parent is unknown still shows that parent as context.
                Visit(r, parents[r], 0);
            }
        }

        _layout = layout;
        InvalidateMeasure();
        InvalidateVisual();
    }

    protected override Size MeasureOverride(Size availableSize) => new(Math.Min(availableSize.Width, 520), 50 + (Math.Max(3, _layout.Count) * 34));

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        if (_layout.Count == 0)
        {
            DrawEmpty(context, bounds, _buffer is null ? "No TF buffer" : "Waiting for /tf…");
            return;
        }

        const double indent = 28;
        const double rowH = 34;
        var positions = new Dictionary<string, Point>(StringComparer.Ordinal);
        foreach (var (frame, parent, depth, row, detail) in _layout)
        {
            var p = new Point(18 + (depth * indent), 40 + (row * rowH));
            positions[frame] = p;
            if (parent is not null && positions.TryGetValue(parent, out var pp))
            {
                var elbow = new Point(pp.X + 8, p.Y + 11);
                context.DrawLine(P.Pen(P.Seam, 1.4), new Point(pp.X + 8, pp.Y + 22), elbow);
                context.DrawLine(P.Pen(P.Seam, 1.4), elbow, new Point(p.X, p.Y + 11));
            }

            var isRoot = depth == 0;
            var label = DrawPillAt(context, frame, p, isRoot ? P.Signal : P.Telemetry);
            if (detail.Length > 0)
            {
                DrawLabel(context, detail, new Point(p.X + label + 10, p.Y + 3), 10, P.Haze, RosTheme.Mono);
            }
        }
    }

    private static double DrawPillAt(DrawingContext context, string text, Point topLeft, Color accent)
    {
        var size = new FormattedText(text, System.Globalization.CultureInfo.InvariantCulture, FlowDirection.LeftToRight, RosTheme.Mono, 11.5, null);
        var rect = new Rect(topLeft.X, topLeft.Y, size.Width + 18, 22);
        context.DrawRectangle(new SolidColorBrush(accent, 0.15), new Pen(new SolidColorBrush(accent, 0.7), 1), rect, 11, 11);
        DrawLabel(context, text, new Point(rect.X + 9, rect.Center.Y), 11.5, P.Ink, RosTheme.Mono, TextAnchor.Left, verticalCenter: true);
        return rect.Width;
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        _timer?.Stop();
        base.OnDetachedFromVisualTree(e);
    }
}
