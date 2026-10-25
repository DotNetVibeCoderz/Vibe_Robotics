using Avalonia;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.Rmw;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// The computation graph (like <c>rqt_graph</c>): nodes as boxes, topics as pills in the middle,
/// arrows from publishers to topics and from topics to subscribers. Refreshes from the middleware's
/// graph every two seconds.
/// </summary>
public sealed class NodeGraphView : RosDrawnControl
{
    private Node? _node;
    private DispatcherTimer? _timer;
    private IReadOnlyList<EndpointInfo> _endpoints = [];
    private IReadOnlyList<NodeInfo> _nodes = [];

    public NodeGraphView() => Title = "Node graph";

    /// <summary>Include <c>/rosout</c>, <c>/parameter_events</c>, hidden topics and this view's own node.</summary>
    public bool ShowInfrastructure { get; set; }

    public int NodeCount => _nodes.Count;

    public void Attach(Node node)
    {
        Detach();
        _node = node;
        _timer = new DispatcherTimer(TimeSpan.FromSeconds(2), DispatcherPriority.Background, (_, _) => Refresh());
        _timer.Start();
        Refresh();
    }

    public void Detach()
    {
        _timer?.Stop();
        _timer = null;
        _node = null;
    }

    public void Refresh()
    {
        if (_node is not { IsDisposed: false } node)
        {
            return;
        }

        var graph = node.Context.Rmw.Graph;
        _endpoints = graph.GetEndpoints()
            .Where(e => e.Kind is EndpointKind.Publisher or EndpointKind.Subscription)
            .Where(e => ShowInfrastructure || (!RosNames.IsHidden(e.Name) && e.Name is not "/rosout" and not "/parameter_events"))
            .Where(e => ShowInfrastructure || !(e.NodeName == node.Name && e.NodeNamespace == node.Namespace))
            .ToList();
        var active = _endpoints.Select(e => RosNames.FullyQualifiedNodeName(e.NodeNamespace, e.NodeName)).ToHashSet(StringComparer.Ordinal);
        _nodes = graph.GetNodes().Where(n => active.Contains(n.FullyQualifiedName)).DistinctBy(n => n.FullyQualifiedName).OrderBy(n => n.FullyQualifiedName, StringComparer.Ordinal).ToList();
        InvalidateMeasure();
        InvalidateVisual();
    }

    protected override Size MeasureOverride(Size availableSize)
    {
        var topics = _endpoints.Select(e => e.Name).Distinct().Count();
        return new Size(Math.Min(availableSize.Width, 900), 60 + (Math.Max(Math.Max(topics, _nodes.Count), 3) * 44));
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        if (_nodes.Count == 0)
        {
            DrawEmpty(context, bounds, _node is null ? "Not attached" : "No nodes with topics yet");
            return;
        }

        var topics = _endpoints.Select(e => e.Name).Distinct().Order(StringComparer.Ordinal).ToList();

        // Nodes that mostly publish go left, the rest right.
        var left = new List<NodeInfo>();
        var right = new List<NodeInfo>();
        foreach (var n in _nodes)
        {
            var pubs = _endpoints.Count(e => Owns(e, n) && e.Kind == EndpointKind.Publisher);
            var subs = _endpoints.Count(e => Owns(e, n) && e.Kind == EndpointKind.Subscription);
            (pubs >= subs ? left : right).Add(n);
        }

        const double rowH = 44;
        var top = 44.0;
        var colL = 16.0;
        var colW = Math.Min(200, bounds.Width * 0.27);
        var colR = bounds.Width - colW - 16;
        var topicW = Math.Min(240, bounds.Width * 0.30);
        var topicX = (bounds.Width - topicW) / 2;

        var nodeRects = new Dictionary<string, Rect>(StringComparer.Ordinal);
        Place(left, colL);
        Place(right, colR);
        void Place(List<NodeInfo> list, double x)
        {
            var offset = top + (((Math.Max(topics.Count, 1) - list.Count) * rowH) / 2);
            for (var i = 0; i < list.Count; i++)
            {
                nodeRects[list[i].FullyQualifiedName] = new Rect(x, Math.Max(top, offset) + (i * rowH), colW, 30);
            }
        }

        var topicRects = new Dictionary<string, Rect>(StringComparer.Ordinal);
        var tOffset = top + (((Math.Max(left.Count, right.Count) - topics.Count) * rowH) / 2);
        for (var i = 0; i < topics.Count; i++)
        {
            topicRects[topics[i]] = new Rect(topicX, Math.Max(top, tOffset) + (i * rowH), topicW, 26);
        }

        // Edges first.
        foreach (var e in _endpoints)
        {
            var key = RosNames.FullyQualifiedNodeName(e.NodeNamespace, e.NodeName);
            if (!nodeRects.TryGetValue(key, out var nr) || !topicRects.TryGetValue(e.Name, out var tr))
            {
                continue;
            }

            var nodeOnLeft = nr.X < topicX;
            var nodePort = new Point(nodeOnLeft ? nr.Right : nr.Left, nr.Center.Y);
            var topicPort = new Point(nodeOnLeft ? tr.Left : tr.Right, tr.Center.Y);
            var publish = e.Kind == EndpointKind.Publisher;
            var (from, to) = publish ? (nodePort, topicPort) : (topicPort, nodePort);
            DrawEdge(context, from, to, publish ? P.Signal : P.Telemetry);
        }

        foreach (var n in _nodes)
        {
            var r = nodeRects[n.FullyQualifiedName];
            context.DrawRectangle(P.Brush(P.SurfaceRaised), P.Pen(P.Seam, 1.2), r, 7, 7);
            context.DrawRectangle(P.SignalGradient(), null, new Rect(r.X, r.Y + 6, 3, r.Height - 12), 1.5, 1.5);
            DrawLabel(context, Ellipsize(n.FullyQualifiedName, 24), new Point(r.X + 12, r.Center.Y), 11.5, P.Ink, RosTheme.SansSemiBold, TextAnchor.Left, verticalCenter: true);
        }

        foreach (var (name, r) in topicRects)
        {
            context.DrawRectangle(P.Brush(P.Telemetry, 0.12), P.Pen(P.Telemetry, 1, 0.6), r, r.Height / 2, r.Height / 2);
            DrawLabel(context, Ellipsize(name, 30), r.Center, 11, P.Ink, RosTheme.Mono, TextAnchor.Center, verticalCenter: true);
        }

        DrawPill(context, $"{_nodes.Count} nodes · {topics.Count} topics", new Point(bounds.Right - 10, 8), P.Signal);
    }

    private static bool Owns(EndpointInfo e, NodeInfo n) => e.NodeName == n.Name && e.NodeNamespace == n.Namespace;

    private static string Ellipsize(string s, int max) => s.Length <= max ? s : "…" + s[^(max - 1)..];

    private static void DrawEdge(DrawingContext context, Point from, Point to, Color color)
    {
        var dx = Math.Max(30, Math.Abs(to.X - from.X) * 0.5) * Math.Sign(to.X - from.X == 0 ? 1 : to.X - from.X);
        var g = new StreamGeometry();
        using (var ctx = g.Open())
        {
            ctx.BeginFigure(from, false);
            ctx.CubicBezierTo(new Point(from.X + dx, from.Y), new Point(to.X - dx, to.Y), to);
            ctx.EndFigure(false);
        }

        context.DrawGeometry(null, new Pen(new SolidColorBrush(color, 0.8), 1.6), g);
        var dir = Math.Sign(to.X - from.X);
        var a = new StreamGeometry();
        using (var ctx = a.Open())
        {
            ctx.BeginFigure(to, true);
            ctx.LineTo(new Point(to.X - (8 * dir), to.Y - 4));
            ctx.LineTo(new Point(to.X - (8 * dir), to.Y + 4));
            ctx.EndFigure(true);
        }

        context.DrawGeometry(new SolidColorBrush(color), null, a);
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        Detach();
        base.OnDetachedFromVisualTree(e);
    }
}
