using Avalonia;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.Rmw;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>One row of a <see cref="TopicMonitor"/>.</summary>
public sealed record TopicStats(string Name, string Type, int Publishers, int Subscribers, double Hz, double BytesPerSecond);

/// <summary>
/// Live topic table (like <c>rqt_topic</c> / <c>ros2 topic hz</c>): every topic in the graph with its
/// type, endpoint counts, message rate and bandwidth. Rates are measured with raw (serialized)
/// subscriptions, so nothing is deserialized.
/// </summary>
public sealed class TopicMonitor : RosDrawnControl
{
    private readonly Dictionary<string, Meter> _meters = new(StringComparer.Ordinal);
    private IReadOnlyList<TopicStats> _rows = [];
    private DispatcherTimer? _timer;
    private Node? _node;
    private DateTime _lastSample = DateTime.UtcNow;

    private sealed class Meter
    {
        public SerializedSubscription? Subscription;
        public long Messages;
        public long Bytes;
        public double Hz;
        public double Bps;
    }

    public TopicMonitor() => Title = "Topics";

    /// <summary>Include <c>/rosout</c>, <c>/parameter_events</c> and hidden topics.</summary>
    public bool ShowInfrastructure { get; set; }

    /// <summary>Measure rates (opens a raw subscription per topic).</summary>
    public bool MeasureRates { get; set; } = true;

    public IReadOnlyList<TopicStats> Rows => _rows;

    public void Attach(Node node)
    {
        Detach();
        _node = node;
        _timer = new DispatcherTimer(TimeSpan.FromSeconds(1), DispatcherPriority.Background, (_, _) => Refresh());
        _timer.Start();
        Refresh();
    }

    public void Detach()
    {
        _timer?.Stop();
        _timer = null;
        foreach (var m in _meters.Values)
        {
            m.Subscription?.Dispose();
        }

        _meters.Clear();
        _node = null;
    }

    /// <summary>Re-reads the graph and recomputes rates (called every second when attached).</summary>
    public void Refresh()
    {
        var node = _node;
        if (node is null || node.IsDisposed)
        {
            return;
        }

        var now = DateTime.UtcNow;
        var dt = Math.Max(0.2, (now - _lastSample).TotalSeconds);
        _lastSample = now;

        var topics = node.GetTopicNamesAndTypes(ShowInfrastructure)
            .Where(t => ShowInfrastructure || (t.Name != "/rosout" && t.Name != "/parameter_events"))
            .ToList();
        var endpoints = node.Context.Rmw.Graph.GetEndpoints();
        var rows = new List<TopicStats>(topics.Count);
        foreach (var t in topics)
        {
            var type = t.Types.Count > 0 ? t.Types[0] : "?";
            if (!_meters.TryGetValue(t.Name, out var m))
            {
                m = _meters[t.Name] = new Meter();
                if (MeasureRates && RosTypeRegistry.TryGetMessage(type, out var info))
                {
                    var meter = m;
                    try
                    {
                        m.Subscription = node.CreateSerializedSubscription(t.Name, info, (cdr, _) =>
                        {
                            Interlocked.Increment(ref meter.Messages);
                            Interlocked.Add(ref meter.Bytes, cdr.Length);
                        }, QosProfile.SensorData.KeepLast(1));
                    }
                    catch (Exception)
                    {
                        // Some middlewares refuse raw subscriptions for unknown types: show the topic without rates.
                    }
                }
            }

            var msgs = Interlocked.Exchange(ref m.Messages, 0);
            var bytes = Interlocked.Exchange(ref m.Bytes, 0);
            m.Hz = (m.Hz * 0.4) + (msgs / dt * 0.6);
            m.Bps = (m.Bps * 0.4) + (bytes / dt * 0.6);

            // Exclude our own meter subscriptions from the counts.
            var pubs = endpoints.Count(e => e.Name == t.Name && e.Kind == EndpointKind.Publisher);
            var subs = endpoints.Count(e => e.Name == t.Name && e.Kind == EndpointKind.Subscription && !(e.NodeName == node.Name && e.NodeNamespace == node.Namespace));
            rows.Add(new TopicStats(t.Name, type, pubs, subs, m.Subscription is null ? double.NaN : m.Hz, m.Subscription is null ? double.NaN : m.Bps));
        }

        _rows = rows;
        InvalidateVisual();
    }

    protected override Size MeasureOverride(Size availableSize) => new(Math.Min(availableSize.Width, 720), 50 + (Math.Max(3, _rows.Count) * 24));

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        var rows = _rows;
        var cols = new[] { 14.0, bounds.Width * 0.34, bounds.Width * 0.64, bounds.Width * 0.74, bounds.Width * 0.86 };
        var y = 32.0;
        DrawLabel(context, "TOPIC", new Point(cols[0], y), 9.5, P.Haze, RosTheme.SansSemiBold);
        DrawLabel(context, "TYPE", new Point(cols[1], y), 9.5, P.Haze, RosTheme.SansSemiBold);
        DrawLabel(context, "PUB/SUB", new Point(cols[2], y), 9.5, P.Haze, RosTheme.SansSemiBold);
        DrawLabel(context, "RATE", new Point(cols[4] - 10, y), 9.5, P.Haze, RosTheme.SansSemiBold, TextAnchor.Right);
        DrawLabel(context, "BW", new Point(bounds.Right - 12, y), 9.5, P.Haze, RosTheme.SansSemiBold, TextAnchor.Right);
        y += 20;
        context.DrawLine(P.Pen(P.Seam), new Point(10, y - 4), new Point(bounds.Right - 10, y - 4));

        if (rows.Count == 0)
        {
            DrawLabel(context, _node is null ? "Not attached" : "No topics yet", new Point(14, y + 4), 11, P.Haze);
            return;
        }

        foreach (var r in rows)
        {
            var active = r.Hz > 0.05;
            context.DrawEllipse(P.Brush(active ? P.Ok : P.Seam), null, new Point(cols[0] + 4, y + 8), 3.5, 3.5);
            DrawLabel(context, r.Name, new Point(cols[0] + 14, y), 11, P.Ink, RosTheme.Mono);
            DrawLabel(context, Shorten(r.Type), new Point(cols[1], y), 11, P.Haze, RosTheme.Mono);
            DrawLabel(context, $"{r.Publishers}/{r.Subscribers}", new Point(cols[2], y), 11, P.Ink, RosTheme.Mono);
            DrawLabel(context, double.IsNaN(r.Hz) ? "–" : $"{r.Hz:0.0} Hz", new Point(cols[4] - 10, y), 11, active ? P.Telemetry : P.Haze, RosTheme.Mono, TextAnchor.Right);
            DrawLabel(context, double.IsNaN(r.BytesPerSecond) ? "–" : FormatBytes(r.BytesPerSecond), new Point(bounds.Right - 12, y), 11, P.Haze, RosTheme.Mono, TextAnchor.Right);
            y += 24;
        }
    }

    private static string Shorten(string type) => type.Replace("/msg/", "/", StringComparison.Ordinal);

    internal static string FormatBytes(double bps) =>
        bps >= 1 << 20 ? $"{bps / (1 << 20):0.0} MB/s" : bps >= 1024 ? $"{bps / 1024:0.0} KB/s" : $"{bps:0} B/s";

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        Detach();
        base.OnDetachedFromVisualTree(e);
    }
}
