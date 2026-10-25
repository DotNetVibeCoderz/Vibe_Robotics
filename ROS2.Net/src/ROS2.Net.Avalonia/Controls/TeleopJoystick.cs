using Avalonia;
using Avalonia.Input;
using Avalonia.Media;
using Avalonia.Threading;
using Twist = ROS2.Net.Msgs.GeometryMsgs.Twist;
using Vector3 = ROS2.Net.Msgs.GeometryMsgs.Vector3;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// A virtual joystick that produces <c>geometry_msgs/Twist</c>: up/down is linear X, left/right is
/// angular Z. Drag the knob, or use the arrow keys / WASD when focused. Releasing returns to zero.
/// <see cref="Attach"/> publishes the twist at <see cref="PublishRateHz"/> while it is non-zero
/// (plus one final zero), so a robot with a command timeout stops when the user lets go.
/// </summary>
public sealed class TeleopJoystick : RosDrawnControl
{
    public static readonly StyledProperty<double> MaxLinearProperty =
        AvaloniaProperty.Register<TeleopJoystick, double>(nameof(MaxLinear), 0.5);

    public static readonly StyledProperty<double> MaxAngularProperty =
        AvaloniaProperty.Register<TeleopJoystick, double>(nameof(MaxAngular), 1.5);

    private Vector _knob; // normalized [-1, 1]
    private bool _dragging;
    private readonly HashSet<Key> _keys = [];
    private DispatcherTimer? _timer;
    private Publisher<Twist>? _publisher;
    private bool _sentZero = true;

    public TeleopJoystick()
    {
        Focusable = true;
        Cursor = new Cursor(StandardCursorType.Hand);
        MinWidth = 160;
        MinHeight = 160;
    }

    /// <summary>Linear speed at full deflection [m/s].</summary>
    public double MaxLinear
    {
        get => GetValue(MaxLinearProperty);
        set => SetValue(MaxLinearProperty, value);
    }

    /// <summary>Angular speed at full deflection [rad/s].</summary>
    public double MaxAngular
    {
        get => GetValue(MaxAngularProperty);
        set => SetValue(MaxAngularProperty, value);
    }

    public double PublishRateHz { get; set; } = 10;

    /// <summary>Current command.</summary>
    public (double Linear, double Angular) Command => (Clean(-_knob.Y * MaxLinear), Clean(-_knob.X * MaxAngular));

    private static double Clean(double v) => Math.Abs(v) < 1e-9 ? 0 : v; // no negative zero

    /// <summary>Raised whenever the command changes.</summary>
    public event Action<double, double>? CommandChanged;

    /// <summary>Publishes the command on <paramref name="topic"/> (e.g. <c>/robot1/cmd_vel</c>).</summary>
    public void Attach(Node node, string topic)
    {
        Detach();
        _publisher = node.CreatePublisher<Twist>(topic);
        _timer = new DispatcherTimer(TimeSpan.FromSeconds(1 / PublishRateHz), DispatcherPriority.Background, (_, _) => Publish());
        _timer.Start();
    }

    public void Detach()
    {
        _timer?.Stop();
        _timer = null;
        _publisher?.Dispose();
        _publisher = null;
    }

    /// <summary>Sets the knob position programmatically (normalized, -1..1; +Y is backward like screen space).</summary>
    public void SetKnob(double x, double y)
    {
        _knob = Clamp(new Vector(x, y));
        Changed();
    }

    private void Publish()
    {
        if (_publisher is null)
        {
            return;
        }

        var (lin, ang) = Command;
        var zero = Math.Abs(lin) < 1e-6 && Math.Abs(ang) < 1e-6;
        if (zero && _sentZero)
        {
            return;
        }

        _publisher.Publish(new Twist { Linear = new Vector3 { X = lin }, Angular = new Vector3 { Z = ang } });
        _sentZero = zero;
    }

    private void Changed()
    {
        InvalidateVisual();
        var (l, a) = Command;
        CommandChanged?.Invoke(l, a);
    }

    private static Vector Clamp(Vector v)
    {
        var len = Math.Sqrt((v.X * v.X) + (v.Y * v.Y));
        return len > 1 ? v / len : v;
    }

    private (Point Center, double Radius) Geometry()
    {
        var r = (Math.Min(Bounds.Width, Bounds.Height) / 2) - 26;
        return (new Point(Bounds.Width / 2, (Bounds.Height / 2) - 2), Math.Max(20, r - 6));
    }

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        _dragging = true;
        e.Pointer.Capture(this);
        Focus();
        Move(e.GetPosition(this));
        e.Handled = true;
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        if (_dragging)
        {
            Move(e.GetPosition(this));
        }
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        _dragging = false;
        e.Pointer.Capture(null);
        _knob = default;
        Changed();
    }

    protected override void OnKeyDown(KeyEventArgs e)
    {
        base.OnKeyDown(e);
        if (_keys.Add(e.Key))
        {
            UpdateFromKeys();
        }

        e.Handled = IsDriveKey(e.Key);
    }

    protected override void OnKeyUp(KeyEventArgs e)
    {
        base.OnKeyUp(e);
        _keys.Remove(e.Key);
        UpdateFromKeys();
    }

    private static bool IsDriveKey(Key k) => k is Key.Up or Key.Down or Key.Left or Key.Right or Key.W or Key.A or Key.S or Key.D or Key.Space;

    private void UpdateFromKeys()
    {
        if (_keys.Contains(Key.Space))
        {
            _knob = default;
            Changed();
            return;
        }

        var y = (_keys.Contains(Key.Up) || _keys.Contains(Key.W) ? -1 : 0) + (_keys.Contains(Key.Down) || _keys.Contains(Key.S) ? 1 : 0);
        var x = (_keys.Contains(Key.Left) || _keys.Contains(Key.A) ? -1 : 0) + (_keys.Contains(Key.Right) || _keys.Contains(Key.D) ? 1 : 0);
        _knob = Clamp(new Vector(x * 0.8, y));
        Changed();
    }

    private void Move(Point p)
    {
        var (c, r) = Geometry();
        _knob = Clamp(new Vector((p.X - c.X) / r, (p.Y - c.Y) / r));
        Changed();
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        var (c, r) = Geometry();

        // Rings and cross hairs.
        context.DrawEllipse(P.Brush(P.SurfaceRaised), P.Pen(P.Seam, 1.2), c, r, r);
        context.DrawEllipse(null, P.Pen(P.Seam, 1, 0.6), c, r * 0.5, r * 0.5);
        context.DrawLine(P.Pen(P.Seam, 1, 0.7), new Point(c.X - r, c.Y), new Point(c.X + r, c.Y));
        context.DrawLine(P.Pen(P.Seam, 1, 0.7), new Point(c.X, c.Y - r), new Point(c.X, c.Y + r));
        DrawLabel(context, "FWD", new Point(c.X, c.Y - r - 15), 9, P.Haze, RosTheme.SansSemiBold, TextAnchor.Center);

        // Command vector.
        var k = new Point(c.X + (_knob.X * r), c.Y + (_knob.Y * r));
        if (_knob.Length > 0.01)
        {
            context.DrawLine(new Pen(P.SignalGradient(), 3, lineCap: PenLineCap.Round), c, k);
        }

        var knobR = Math.Max(14, r * 0.22);
        context.DrawEllipse(P.SignalGradient(new RelativePoint(0, 0, RelativeUnit.Relative), new RelativePoint(1, 1, RelativeUnit.Relative)), P.Pen(P.Ink, 1.5, 0.35), k, knobR, knobR);

        var (lin, ang) = Command;
        DrawPill(context, $"v {lin,5:+0.00;-0.00} m/s  ω {ang,5:+0.00;-0.00} rad/s", new Point(bounds.Right - 10, bounds.Bottom - 30), P.Signal);
        if (IsFocused)
        {
            context.DrawRectangle(null, P.Pen(P.Signal, 1.5, 0.6), bounds.Deflate(2), 9, 9);
        }
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        Detach();
        base.OnDetachedFromVisualTree(e);
    }
}
