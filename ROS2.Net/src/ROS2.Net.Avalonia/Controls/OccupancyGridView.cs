using Avalonia;
using Avalonia.Input;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using OccupancyGrid = ROS2.Net.Msgs.NavMsgs.OccupancyGrid;
using PathMsg = ROS2.Net.Msgs.NavMsgs.Path;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>A pose drawn on a map view.</summary>
public readonly record struct MapPose(double X, double Y, double Yaw, Color? Color = null, string? Label = null);

/// <summary>
/// Shows a <c>nav_msgs/OccupancyGrid</c> (map or costmap) with pan (drag) and zoom (wheel), plus
/// overlays: robot poses, a planned path, a goal and world-space points (e.g. a projected scan).
/// Clicking without dragging raises <see cref="WorldClicked"/> with map coordinates — use it to
/// send navigation goals.
/// </summary>
public sealed class OccupancyGridView : RosDrawnControl
{
    private OccupancyGrid? _grid;
    private WriteableBitmap? _bitmap;
    private double _scale = 40; // pixels per metre
    private Vector _offset;     // screen position of world (0,0)
    private bool _fitted;
    private Point? _press;
    private Point _last;
    private bool _panned;
    private IDisposable? _mapSub;
    private IDisposable? _pathSub;

    public OccupancyGridView()
    {
        Title = "Map";
        Cursor = new Cursor(StandardCursorType.Cross);
    }

    /// <summary>Colour cells as a costmap (0–100 gradient) instead of free/occupied/unknown.</summary>
    public bool CostmapColors { get; set; }

    public IReadOnlyList<MapPose> Robots { get; set; } = [];

    public IReadOnlyList<(double X, double Y)> PathPoints { get; set; } = [];

    public MapPose? Goal { get; set; }

    public IReadOnlyList<(double X, double Y)> Points { get; set; } = [];

    /// <summary>Raised on a click (not a pan) with map coordinates.</summary>
    public event Action<double, double>? WorldClicked;

    public OccupancyGrid? Grid
    {
        get => _grid;
        set
        {
            var sizeChanged = _grid is null || value is null || _grid.Info.Width != value.Info.Width || _grid.Info.Height != value.Info.Height;
            _grid = value;
            RebuildBitmap();
            if (sizeChanged)
            {
                _fitted = false;
            }

            InvalidateVisual();
        }
    }

    /// <summary>Subscribes to a latched map topic and, optionally, a <c>nav_msgs/Path</c> topic.</summary>
    public void Attach(Node node, string mapTopic = "/map", string? pathTopic = null)
    {
        Detach();
        _mapSub = RosUi.SubscribeLatest<OccupancyGrid>(node, mapTopic, g => Grid = g, QosProfile.Latched);
        if (pathTopic is not null)
        {
            _pathSub = RosUi.SubscribeLatest<PathMsg>(node, pathTopic, p =>
            {
                PathPoints = p.Poses.Select(s => (s.Pose.Position.X, s.Pose.Position.Y)).ToArray();
                InvalidateVisual();
            });
        }
    }

    public void Detach()
    {
        _mapSub?.Dispose();
        _pathSub?.Dispose();
        _mapSub = _pathSub = null;
    }

    /// <summary>Fits the whole map into view.</summary>
    public void FitToView()
    {
        _fitted = false;
        InvalidateVisual();
    }

    public Point WorldToScreen(double x, double y) => new(_offset.X + (x * _scale), _offset.Y - (y * _scale));

    public (double X, double Y) ScreenToWorld(Point p) => ((p.X - _offset.X) / _scale, (_offset.Y - p.Y) / _scale);

    private void RebuildBitmap()
    {
        var g = _grid;
        if (g is null || g.Info.Width == 0 || g.Info.Height == 0)
        {
            _bitmap = null;
            return;
        }

        int w = (int)g.Info.Width, h = (int)g.Info.Height;
        if (_bitmap is null || _bitmap.PixelSize.Width != w || _bitmap.PixelSize.Height != h)
        {
            _bitmap = new WriteableBitmap(new PixelSize(w, h), new Vector(96, 96), PixelFormat.Bgra8888, AlphaFormat.Premul);
        }

        var free = P.Surface;
        var occ = P.Ink;
        var unknown = P.SurfaceRaised;
        using var fb = _bitmap.Lock();
        for (var y = 0; y < h; y++)
        {
            // Grid row 0 is the bottom of the map: flip so north is up.
            Span<uint> row;
            unsafe
            {
                row = new Span<uint>((byte*)fb.Address + ((h - 1 - y) * fb.RowBytes), w);
            }

            for (var x = 0; x < w; x++)
            {
                var i = (y * w) + x;
                var v = i < g.Data.Length ? g.Data[i] : (sbyte)-1;
                Color c;
                if (v < 0)
                {
                    c = unknown;
                }
                else if (CostmapColors)
                {
                    c = v >= 99 ? P.Fault : v == 0 ? free : Blend(free, P.Ramp(v / 100.0), 0.25 + (v / 140.0));
                }
                else
                {
                    c = v >= 65 ? occ : v <= 25 ? free : Blend(free, occ, v / 100.0);
                }

                row[x] = (uint)(0xFF << 24 | c.R << 16 | c.G << 8 | c.B);
            }
        }
    }

    private static Color Blend(Color a, Color b, double t)
    {
        t = Math.Clamp(t, 0, 1);
        return Color.FromRgb((byte)(a.R + ((b.R - a.R) * t)), (byte)(a.G + ((b.G - a.G) * t)), (byte)(a.B + ((b.B - a.B) * t)));
    }

    private void Fit(Rect bounds)
    {
        var g = _grid;
        double minX = -5, minY = -5, maxX = 5, maxY = 5;
        if (g is not null && g.Info.Width > 0)
        {
            minX = g.Info.Origin.Position.X;
            minY = g.Info.Origin.Position.Y;
            maxX = minX + (g.Info.Width * g.Info.Resolution);
            maxY = minY + (g.Info.Height * g.Info.Resolution);
        }

        var pad = 30.0;
        _scale = Math.Min((bounds.Width - (2 * pad)) / Math.Max(0.1, maxX - minX), (bounds.Height - (2 * pad) - 16) / Math.Max(0.1, maxY - minY));
        _scale = Math.Max(1, _scale);
        var cx = (minX + maxX) / 2;
        var cy = (minY + maxY) / 2;
        _offset = new Vector((bounds.Width / 2) - (cx * _scale), (bounds.Height / 2) + 8 + (cy * _scale));
        _fitted = true;
    }

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        _press = _last = e.GetPosition(this);
        _panned = false;
        e.Pointer.Capture(this);
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        if (_press is null)
        {
            return;
        }

        var p = e.GetPosition(this);
        if (!_panned && Math.Abs(p.X - _press.Value.X) + Math.Abs(p.Y - _press.Value.Y) < 4)
        {
            return;
        }

        _panned = true;
        _offset += new Vector(p.X - _last.X, p.Y - _last.Y);
        _last = p;
        InvalidateVisual();
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        if (_press is { } start && !_panned)
        {
            var (x, y) = ScreenToWorld(start);
            WorldClicked?.Invoke(x, y);
        }

        _press = null;
        e.Pointer.Capture(null);
    }

    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        base.OnPointerWheelChanged(e);
        var p = e.GetPosition(this);
        var (wx, wy) = ScreenToWorld(p);
        _scale = Math.Clamp(_scale * Math.Pow(1.15, e.Delta.Y), 2, 2000);
        _offset = new Vector(p.X - (wx * _scale), p.Y + (wy * _scale));
        InvalidateVisual();
        e.Handled = true;
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        context.DrawRectangle(P.Brush(P.Background, P.IsDark ? 0.6 : 0.3), null, bounds);
        if (!_fitted && bounds.Width > 0)
        {
            Fit(bounds);
        }

        // Metre grid.
        var gridStep = _scale > 60 ? 0.5 : _scale > 15 ? 1 : 5;
        var (x0, y1) = ScreenToWorld(bounds.TopLeft);
        var (x1, y0) = ScreenToWorld(bounds.BottomRight);
        var pen = P.Pen(P.Grid, 1);
        for (var gx = Math.Floor(x0 / gridStep) * gridStep; gx <= x1; gx += gridStep)
        {
            var s = WorldToScreen(gx, 0);
            context.DrawLine(pen, new Point(s.X, bounds.Top), new Point(s.X, bounds.Bottom));
        }

        for (var gy = Math.Floor(y0 / gridStep) * gridStep; gy <= y1; gy += gridStep)
        {
            var s = WorldToScreen(0, gy);
            context.DrawLine(pen, new Point(bounds.Left, s.Y), new Point(bounds.Right, s.Y));
        }

        var g = _grid;
        if (_bitmap is not null && g is not null)
        {
            var ox = g.Info.Origin.Position.X;
            var oy = g.Info.Origin.Position.Y;
            var topLeft = WorldToScreen(ox, oy + (g.Info.Height * g.Info.Resolution));
            var dest = new Rect(topLeft, new Size(g.Info.Width * g.Info.Resolution * _scale, g.Info.Height * g.Info.Resolution * _scale));
            using (context.PushRenderOptions(new RenderOptions { BitmapInterpolationMode = BitmapInterpolationMode.None }))
            {
                context.DrawImage(_bitmap, new Rect(_bitmap.Size), dest);
            }
        }
        else if (Robots.Count == 0)
        {
            DrawEmpty(context, bounds, "Waiting for map…");
        }

        // Points (e.g. scan hits).
        if (Points.Count > 0)
        {
            var brush = P.Brush(P.Telemetry);
            foreach (var (px, py) in Points)
            {
                context.DrawEllipse(brush, null, WorldToScreen(px, py), 1.8, 1.8);
            }
        }

        // Path.
        if (PathPoints.Count > 1)
        {
            var geo = new StreamGeometry();
            using (var ctx = geo.Open())
            {
                ctx.BeginFigure(WorldToScreen(PathPoints[0].X, PathPoints[0].Y), false);
                for (var i = 1; i < PathPoints.Count; i++)
                {
                    ctx.LineTo(WorldToScreen(PathPoints[i].X, PathPoints[i].Y));
                }

                ctx.EndFigure(false);
            }

            context.DrawGeometry(null, new Pen(P.Brush(P.Signal), 2.5, lineCap: PenLineCap.Round, lineJoin: PenLineJoin.Round), geo);
        }

        if (Goal is { } goal)
        {
            var s = WorldToScreen(goal.X, goal.Y);
            context.DrawEllipse(P.Brush(P.Alert, 0.25), P.Pen(P.Alert, 2), s, 9, 9);
            context.DrawEllipse(P.Brush(P.Alert), null, s, 3, 3);
        }

        foreach (var r in Robots)
        {
            DrawRobotGlyph(context, WorldToScreen(r.X, r.Y), r.Yaw, Math.Max(7, 0.18 * _scale), r.Color ?? P.Signal);
            if (r.Label is { } label)
            {
                DrawLabel(context, label, WorldToScreen(r.X, r.Y) + new Vector(12, -18), 10, P.Ink, RosTheme.Mono);
            }
        }

        DrawPill(context, $"1 m = {_scale:0} px", new Point(bounds.Right - 10, 8), P.Haze);
    }

    /// <summary>Draws a robot: a disc with a heading wedge.</summary>
    internal static void DrawRobotGlyph(DrawingContext context, Point c, double yaw, double radius, Color color)
    {
        context.DrawEllipse(new SolidColorBrush(color, 0.25), new Pen(new SolidColorBrush(color), 2), c, radius, radius);
        var tip = new Point(c.X + (Math.Cos(yaw) * radius * 1.35), c.Y - (Math.Sin(yaw) * radius * 1.35));
        var left = new Point(c.X + (Math.Cos(yaw + 2.4) * radius * 0.6), c.Y - (Math.Sin(yaw + 2.4) * radius * 0.6));
        var right = new Point(c.X + (Math.Cos(yaw - 2.4) * radius * 0.6), c.Y - (Math.Sin(yaw - 2.4) * radius * 0.6));
        var g = new StreamGeometry();
        using (var ctx = g.Open())
        {
            ctx.BeginFigure(tip, true);
            ctx.LineTo(left);
            ctx.LineTo(right);
            ctx.EndFigure(true);
        }

        context.DrawGeometry(new SolidColorBrush(color), null, g);
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        Detach();
        base.OnDetachedFromVisualTree(e);
    }
}
