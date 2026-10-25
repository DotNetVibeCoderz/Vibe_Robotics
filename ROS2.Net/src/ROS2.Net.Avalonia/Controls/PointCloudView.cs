using Avalonia;
using Avalonia.Input;
using Avalonia.Media;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>Which plane a <see cref="PointCloudView"/> projects onto.</summary>
public enum CloudProjection
{
    /// <summary>Looking down: X up the screen, Y to the left.</summary>
    Top,
    /// <summary>From the side: X to the right, Z up.</summary>
    Side,
    /// <summary>From the front: Y to the left, Z up.</summary>
    Front,
    /// <summary>A fixed isometric view (rotate with drag).</summary>
    Orbit,
}

/// <summary>
/// Fast 2D projection of a point cloud (interleaved xyz floats), coloured by height. Feed it
/// <see cref="SetPoints"/> or several layers (e.g. raw vs filtered) via <see cref="Layers"/>.
/// </summary>
public sealed class PointCloudView : RosDrawnControl
{
    private readonly List<(float[] Xyz, Color? Color, double Size)> _layers = [];
    private double _yaw = 0.7;
    private double _pitch = 0.5;
    private Point? _drag;

    public PointCloudView() => Title = "Point cloud";

    public CloudProjection Projection { get; set; } = CloudProjection.Orbit;

    /// <summary>Half-extent of the view in metres (0 = auto from data).</summary>
    public double Extent { get; set; }

    public IReadOnlyList<(float[] Xyz, Color? Color, double Size)> Layers => _layers;

    /// <summary>Replaces all layers with one cloud.</summary>
    public void SetPoints(float[] xyz)
    {
        _layers.Clear();
        _layers.Add((xyz, null, 1.6));
        InvalidateVisual();
    }

    public void ClearLayers()
    {
        _layers.Clear();
        InvalidateVisual();
    }

    /// <summary>Adds a layer; <paramref name="color"/> null colours by height.</summary>
    public void AddLayer(float[] xyz, Color? color = null, double size = 1.6)
    {
        _layers.Add((xyz, color, size));
        InvalidateVisual();
    }

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        base.OnPointerPressed(e);
        _drag = e.GetPosition(this);
        e.Pointer.Capture(this);
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        base.OnPointerMoved(e);
        if (_drag is { } d && Projection == CloudProjection.Orbit)
        {
            var p = e.GetPosition(this);
            _yaw += (p.X - d.X) * 0.01;
            _pitch = Math.Clamp(_pitch + ((p.Y - d.Y) * 0.01), -1.4, 1.4);
            _drag = p;
            InvalidateVisual();
        }
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        _drag = null;
        e.Pointer.Capture(null);
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        if (_layers.Count == 0 || _layers.All(l => l.Xyz.Length == 0))
        {
            DrawEmpty(context, bounds, "No points");
            return;
        }

        // Bounds for scaling and the colour ramp.
        float minZ = float.MaxValue, maxZ = float.MinValue;
        double extent = Extent;
        double cx = 0, cy = 0, cz = 0;
        long n = 0;
        foreach (var (xyz, _, _) in _layers)
        {
            for (var i = 0; i + 2 < xyz.Length; i += 3)
            {
                var z = xyz[i + 2];
                if (!float.IsFinite(z))
                {
                    continue;
                }

                minZ = Math.Min(minZ, z);
                maxZ = Math.Max(maxZ, z);
                cx += xyz[i];
                cy += xyz[i + 1];
                cz += z;
                n++;
            }
        }

        if (n == 0)
        {
            DrawEmpty(context, bounds, "No finite points");
            return;
        }

        cx /= n;
        cy /= n;
        cz /= n;
        if (extent <= 0)
        {
            foreach (var (xyz, _, _) in _layers)
            {
                for (var i = 0; i + 2 < xyz.Length; i += 3)
                {
                    if (float.IsFinite(xyz[i]))
                    {
                        extent = Math.Max(extent, Math.Max(Math.Abs(xyz[i] - cx), Math.Max(Math.Abs(xyz[i + 1] - cy), Math.Abs(xyz[i + 2] - cz))));
                    }
                }
            }

            extent = Math.Max(0.1, extent * 1.1);
        }

        var scale = (Math.Min(bounds.Width, bounds.Height) / 2 - 16) / extent;
        var center = new Point(bounds.Width / 2, (bounds.Height / 2) + 8);
        var cyaw = Math.Cos(_yaw);
        var syaw = Math.Sin(_yaw);
        var cp = Math.Cos(_pitch);
        var sp = Math.Sin(_pitch);

        const int bins = 12;
        var ramp = new IBrush[bins];
        for (var i = 0; i < bins; i++)
        {
            ramp[i] = P.Brush(P.Ramp(i / (double)(bins - 1)));
        }

        var zRange = Math.Max(1e-3, maxZ - minZ);
        foreach (var (xyz, color, size) in _layers)
        {
            var fixedBrush = color is { } c ? P.Brush(c) : null;
            for (var i = 0; i + 2 < xyz.Length; i += 3)
            {
                double x = xyz[i] - cx, y = xyz[i + 1] - cy, z = xyz[i + 2] - cz;
                if (!double.IsFinite(x) || !double.IsFinite(y) || !double.IsFinite(z))
                {
                    continue;
                }

                double sx, sy;
                switch (Projection)
                {
                    case CloudProjection.Top:
                        sx = -y;
                        sy = -x;
                        break;
                    case CloudProjection.Side:
                        sx = x;
                        sy = -z;
                        break;
                    case CloudProjection.Front:
                        sx = -y;
                        sy = -z;
                        break;
                    default:
                        var rx = (x * cyaw) - (y * syaw);
                        var ry = (x * syaw) + (y * cyaw);
                        sx = rx;
                        sy = -((z * cp) - (ry * sp));
                        break;
                }

                var brush = fixedBrush ?? ramp[(int)((xyz[i + 2] - minZ) / zRange * (bins - 1))];
                context.DrawRectangle(brush, null, new Rect(center.X + (sx * scale) - (size / 2), center.Y + (sy * scale) - (size / 2), size, size));
            }
        }

        DrawPill(context, $"{n:N0} pts · z {minZ:0.00}…{maxZ:0.00} m", new Point(bounds.Right - 10, 8), P.Telemetry);
    }
}
