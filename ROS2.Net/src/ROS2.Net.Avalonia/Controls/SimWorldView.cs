using Avalonia;
using Avalonia.Input;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.Simulation;
using SimPose = ROS2.Net.Simulation.Pose2D;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// Top-down view of a <see cref="SimWorld"/> (ROS2.Net.Simulation): obstacles, robots with heading,
/// LiDAR rays, trails, arm placeholders and an optional goal/path overlay. Pan with drag, zoom with
/// the wheel; clicks raise <see cref="WorldClicked"/>. <see cref="Attach"/> polls snapshots at
/// <see cref="FramesPerSecond"/>.
/// </summary>
public sealed class SimWorldView : RosDrawnControl
{
    private WorldSnapshot? _snapshot;
    private DispatcherTimer? _timer;
    private SimWorld? _world;
    private double _scale = 30;
    private Vector _offset;
    private bool _fitted;
    private Point? _press;
    private Point _last;
    private bool _panned;

    public SimWorldView()
    {
        Title = "World";
        Cursor = new Cursor(StandardCursorType.Cross);
    }

    public bool ShowLidar { get; set; } = true;

    public bool ShowTrails { get; set; } = true;

    public bool ShowGrid { get; set; } = true;

    /// <summary>Name of the highlighted robot.</summary>
    public string? SelectedRobot { get; set; }

    public (double X, double Y)? Goal { get; set; }

    public IReadOnlyList<(double X, double Y)> PathPoints { get; set; } = [];

    public double FramesPerSecond { get; set; } = 30;

    public WorldSnapshot? Snapshot
    {
        get => _snapshot;
        set
        {
            if (_snapshot?.Name != value?.Name || _snapshot?.Bounds != value?.Bounds)
            {
                _fitted = false;
            }

            _snapshot = value;
            InvalidateVisual();
        }
    }

    /// <summary>Raised on a click (not a pan) with world coordinates.</summary>
    public event Action<double, double>? WorldClicked;

    /// <summary>Raised after each refresh with the snapshot that was drawn.</summary>
    public event Action<WorldSnapshot>? SnapshotUpdated;

    public void Attach(SimWorld world)
    {
        Detach();
        _world = world;
        _timer = new DispatcherTimer(TimeSpan.FromSeconds(1 / FramesPerSecond), DispatcherPriority.Render, (_, _) =>
        {
            if (_world is { } w)
            {
                var s = w.Snapshot();
                Snapshot = s;
                SnapshotUpdated?.Invoke(s);
            }
        });
        _timer.Start();
        Snapshot = world.Snapshot();
    }

    public void Detach()
    {
        _timer?.Stop();
        _timer = null;
        _world = null;
    }

    public void FitToView()
    {
        _fitted = false;
        InvalidateVisual();
    }

    public Point WorldToScreen(double x, double y) => new(_offset.X + (x * _scale), _offset.Y - (y * _scale));

    public (double X, double Y) ScreenToWorld(Point p) => ((p.X - _offset.X) / _scale, (_offset.Y - p.Y) / _scale);

    private Point W(Vec2 v) => WorldToScreen(v.X, v.Y);

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
        if (_press is not { } start)
        {
            return;
        }

        var p = e.GetPosition(this);
        if (!_panned && Math.Abs(p.X - start.X) + Math.Abs(p.Y - start.Y) < 4)
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
        _scale = Math.Clamp(_scale * Math.Pow(1.15, e.Delta.Y), 3, 800);
        _offset = new Vector(p.X - (wx * _scale), p.Y + (wy * _scale));
        InvalidateVisual();
        e.Handled = true;
    }

    private void Fit(Rect bounds, Bounds2D world)
    {
        const double pad = 24;
        _scale = Math.Max(2, Math.Min((bounds.Width - (2 * pad)) / Math.Max(0.5, world.Width), (bounds.Height - (2 * pad) - 20) / Math.Max(0.5, world.Height)));
        var cx = (world.MinX + world.MaxX) / 2;
        var cy = (world.MinY + world.MaxY) / 2;
        _offset = new Vector((bounds.Width / 2) - (cx * _scale), (bounds.Height / 2) + 10 + (cy * _scale));
        _fitted = true;
    }

    private static Color Tone(SimColor c, double mix)
    {
        var bg = P.Surface;
        return Color.FromRgb((byte)(c.R + ((bg.R - c.R) * mix)), (byte)(c.G + ((bg.G - c.G) * mix)), (byte)(c.B + ((bg.B - c.B) * mix)));
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        var s = _snapshot;
        if (s is null)
        {
            DrawEmpty(context, bounds, "No world");
            return;
        }

        if (!_fitted)
        {
            Fit(bounds, s.Bounds);
        }

        // Floor.
        var floor = new Rect(WorldToScreen(s.Bounds.MinX, s.Bounds.MaxY), WorldToScreen(s.Bounds.MaxX, s.Bounds.MinY));
        context.DrawRectangle(P.Brush(P.IsDark ? Color.Parse("#12142B") : Color.Parse("#FAFAFD")), P.Pen(P.Seam), floor);
        if (ShowGrid)
        {
            var pen = P.Pen(P.Grid, 1);
            for (var x = Math.Ceiling(s.Bounds.MinX); x <= s.Bounds.MaxX; x++)
            {
                context.DrawLine(pen, WorldToScreen(x, s.Bounds.MinY), WorldToScreen(x, s.Bounds.MaxY));
            }

            for (var y = Math.Ceiling(s.Bounds.MinY); y <= s.Bounds.MaxY; y++)
            {
                context.DrawLine(pen, WorldToScreen(s.Bounds.MinX, y), WorldToScreen(s.Bounds.MaxX, y));
            }
        }

        // Obstacles.
        var mix = P.IsDark ? 0.35 : 0.1;
        foreach (var o in s.Obstacles)
        {
            var fill = new SolidColorBrush(Tone(o.Color, mix));
            var stroke = new Pen(new SolidColorBrush(Tone(o.Color, mix * 0.4)), 1);
            if (o.Kind == ShapeKind.Cylinder || o.Vertices.Count == 0)
            {
                context.DrawEllipse(fill, stroke, W(o.Center), Math.Max(1, o.Radius * _scale), Math.Max(1, o.Radius * _scale));
                continue;
            }

            var g = new StreamGeometry();
            using (var ctx = g.Open())
            {
                ctx.BeginFigure(W(o.Vertices[0]), true);
                for (var i = 1; i < o.Vertices.Count; i++)
                {
                    ctx.LineTo(W(o.Vertices[i]));
                }

                ctx.EndFigure(true);
            }

            context.DrawGeometry(fill, stroke, g);
        }

        // Planned path & goal.
        if (PathPoints.Count > 1)
        {
            var pg = new StreamGeometry();
            using (var ctx = pg.Open())
            {
                ctx.BeginFigure(WorldToScreen(PathPoints[0].X, PathPoints[0].Y), false);
                foreach (var (x, y) in PathPoints.Skip(1))
                {
                    ctx.LineTo(WorldToScreen(x, y));
                }

                ctx.EndFigure(false);
            }

            context.DrawGeometry(null, new Pen(P.Brush(P.Signal), 2.5, dashStyle: new DashStyle([2, 2], 0), lineCap: PenLineCap.Round), pg);
        }

        if (Goal is { } goal)
        {
            var gp = WorldToScreen(goal.X, goal.Y);
            context.DrawEllipse(P.Brush(P.Alert, 0.2), P.Pen(P.Alert, 2), gp, 10, 10);
            context.DrawEllipse(P.Brush(P.Alert), null, gp, 3, 3);
        }

        foreach (var r in s.Robots)
        {
            DrawRobot(context, r);
        }

        DrawPill(context, $"t = {s.Time:0.0} s{(s.Paused ? "  ⏸ paused" : string.Empty)}", new Point(bounds.Right - 10, 8), s.Paused ? P.Alert : P.Ok);
    }

    private void DrawRobot(DrawingContext context, RobotSnapshot r)
    {
        var selected = SelectedRobot == r.Name;
        var accent = selected || SelectedRobot is null ? P.Signal : P.Haze;

        if (ShowTrails && r.Trail.Count > 1)
        {
            var tg = new StreamGeometry();
            using (var ctx = tg.Open())
            {
                ctx.BeginFigure(W(r.Trail[0]), false);
                for (var i = 1; i < r.Trail.Count; i++)
                {
                    ctx.LineTo(W(r.Trail[i]));
                }

                ctx.EndFigure(false);
            }

            context.DrawGeometry(null, new Pen(new SolidColorBrush(accent, 0.35), 1.5), tg);
        }

        if (ShowLidar && r.LaserOrigin is { } lo && r.LaserHits.Count > 0)
        {
            var origin = WorldToScreen(lo.X, lo.Y);
            var rayPen = new Pen(new SolidColorBrush(P.Telemetry, 0.10), 1);
            var hit = new SolidColorBrush(P.Telemetry, 0.9);
            for (var i = 0; i < r.LaserHits.Count; i++)
            {
                var h = W(r.LaserHits[i]);
                if ((i & 3) == 0)
                {
                    context.DrawLine(rayPen, origin, h);
                }

                context.DrawRectangle(hit, null, new Rect(h.X - 1.2, h.Y - 1.2, 2.4, 2.4));
            }
        }

        var c = W(r.Pose.Position);
        var body = new SolidColorBrush(accent, 0.28);
        var outline = new Pen(new SolidColorBrush(r.Bumper ? P.Fault : accent), selected ? 2.5 : 1.8);
        if (r.Kind == RobotKind.DiffDrive && Math.Abs(r.Length - r.Width) < 0.05)
        {
            context.DrawEllipse(body, outline, c, r.Radius * _scale, r.Radius * _scale);
        }
        else
        {
            var hl = r.Length / 2;
            var hw = r.Width / 2;
            var corners = new[] { new Vec2(hl, hw), new Vec2(hl, -hw), new Vec2(-hl, -hw), new Vec2(-hl, hw) };
            var g = new StreamGeometry();
            using (var ctx = g.Open())
            {
                ctx.BeginFigure(W(r.Pose.TransformPoint(corners[0])), true);
                for (var i = 1; i < 4; i++)
                {
                    ctx.LineTo(W(r.Pose.TransformPoint(corners[i])));
                }

                ctx.EndFigure(true);
            }

            context.DrawGeometry(body, outline, g);
        }

        // Heading.
        var tip = W(r.Pose.TransformPoint(new Vec2(Math.Max(r.Radius, r.Length / 2) * 1.25, 0)));
        context.DrawLine(new Pen(new SolidColorBrush(accent), 2.5, lineCap: PenLineCap.Round), c, tip);

        // Arm placeholder: a planar chain from the robot centre.
        if (r.ArmLinkLengths.Count > 0 && r.JointPositions.Count > 0)
        {
            var angle = r.Pose.Theta;
            var p = r.Pose.Position;
            var armPen = new Pen(P.Brush(P.Alert), 3, lineCap: PenLineCap.Round);
            for (var i = 0; i < Math.Min(r.ArmLinkLengths.Count, r.JointPositions.Count); i++)
            {
                angle += r.JointPositions[i];
                var next = p + new Vec2(Math.Cos(angle), Math.Sin(angle)) * r.ArmLinkLengths[i] * 0.5;
                context.DrawLine(armPen, W(p), W(next));
                p = next;
            }
        }

        DrawLabel(context, r.Name, new Point(c.X + (r.Radius * _scale) + 6, c.Y - 18), 10.5, P.Ink, RosTheme.Mono);
    }

    /// <summary>The pose of a robot in the current snapshot.</summary>
    public SimPose? PoseOf(string robot) => _snapshot?.Robots.FirstOrDefault(r => r.Name == robot)?.Pose;

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        Detach();
        base.OnDetachedFromVisualTree(e);
    }
}
