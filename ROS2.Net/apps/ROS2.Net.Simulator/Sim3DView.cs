using System.Numerics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using ROS2.Net.Avalonia;
using ROS2.Net.Simulation;
using ThreeNet;
using ThreeNet.Avalonia;

namespace Ros2NetSimulator;

/// <summary>
/// A GPU 3D view of a <see cref="SimWorld"/> rendered with Three.Net: obstacles extruded to their
/// heights, robots as cylinders that follow the simulation. ROS x/y map to Three.Net x/−z, z is up (Y).
/// Drag to orbit, wheel to zoom. Falls back to a message when no GPU renderer can be created.
/// </summary>
public sealed class Sim3DView : UserControl
{
    private readonly SimWorld _world;
    private readonly Scene _scene = new();
    private readonly Dictionary<string, ThreeNet.Node> _robots = new(StringComparer.Ordinal);
    private readonly Dictionary<string, ThreeNet.Node> _dynamic = new(StringComparer.Ordinal);
    private readonly ThreeNet.Node _camera;
    private readonly Material _robotMaterial;
    private readonly ThreeNetView _view;
    private double _yaw = -0.8;
    private double _pitch = 0.75;
    private double _distance;
    private Vector3 _target;
    private Point? _drag;

    public Sim3DView(SimWorld world)
    {
        _world = world;
        var b = world.Bounds;
        _target = new Vector3((float)((b.MinX + b.MaxX) / 2), 0, (float)-((b.MinY + b.MaxY) / 2));
        _distance = Math.Max(b.Width, b.Height) * 1.1;

        _scene.AddLight(Light.Hemisphere(new Vector3(0.85f, 0.9f, 1f), new Vector3(0.12f, 0.12f, 0.2f), 0.9f), null!, "sky");
        var sun = _scene.AddLight(Light.Directional(new Vector3(1f, 0.97f, 0.92f), 2.6f), null!, "sun");
        sun.EulerAngles = new Vector3(-0.9f, 0.6f, 0f);

        var floor = _scene.AddMesh(_scene.CreatePlaneGeometry((float)b.Width, (float)b.Height, 1, 1), _scene.CreateMaterial(new Vector4(0.10f, 0.11f, 0.20f, 1), 0f, 0.95f), null!, "floor");
        floor.SetTransform(_target, Quaternion.CreateFromAxisAngle(Vector3.UnitX, -MathF.PI / 2), Vector3.One);

        foreach (var o in world.StaticObstacles)
        {
            AddObstacle(o.Kind, o.Center, o.Rotation, o.HalfExtents * 2, o.Radius, o.Height, o.Color, o.Name, isDynamic: false);
        }

        foreach (var o in world.DynamicObstacles)
        {
            _dynamic[o.Name] = AddObstacle(o.Kind, o.Center, o.Rotation, o.HalfExtents * 2, o.Radius, o.Height, o.Color, o.Name, isDynamic: true);
        }

        var accent = RosTheme.Current.Signal;
        _robotMaterial = _scene.CreateMaterial(new Vector4(accent.R / 255f, accent.G / 255f, accent.B / 255f, 1), 0.2f, 0.4f);
        _camera = _scene.AddCamera(Camera.Perspective(0.9f, 0.05f, 500f), Vector3.Zero, null!, true);
        UpdateCamera();

        _view = new ThreeNetView { Scene = _scene, MaxFramesPerSecond = 60 };
        _view.Frame += (_, _) => Sync();
        var error = new TextBlock
        {
            Text = "3D view unavailable on this machine (no GPU renderer). The 2D view shows the same world.",
            IsVisible = false,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            Foreground = new SolidColorBrush(RosTheme.Current.Haze),
        };
        _view.RenderFailed += (_, _) => global::Avalonia.Threading.Dispatcher.UIThread.Post(() => error.IsVisible = true);
        Content = new Grid { Children = { _view, error } };
    }

    private ThreeNet.Node AddObstacle(ShapeKind kind, Vec2 center, double rotation, Vec2 size, double radius, double height, SimColor color, string name, bool isDynamic)
    {
        var h = (float)Math.Max(0.05, height);
        var geometry = kind == ShapeKind.Cylinder
            ? _scene.CreateCylinderGeometry((float)radius, (float)radius, h, 24, 1, false)
            : _scene.CreateBoxGeometry((float)Math.Max(0.02, size.X), h, (float)Math.Max(0.02, size.Y), 1);
        var material = _scene.CreateMaterial(new Vector4(color.R / 255f, color.G / 255f, color.B / 255f, 1), isDynamic ? 0.1f : 0f, 0.7f);
        var node = _scene.AddMesh(geometry, material, null!, name);
        node.SetTransform(new Vector3((float)center.X, h / 2, (float)-center.Y), Quaternion.CreateFromAxisAngle(Vector3.UnitY, (float)rotation), Vector3.One);
        return node;
    }

    /// <summary>Moves robots and pushable obstacles to their current poses (called every frame).</summary>
    private void Sync()
    {
        foreach (var r in _world.Robots)
        {
            if (!_robots.TryGetValue(r.Name, out var node))
            {
                var p = r.Parameters;
                var body = _scene.CreateCylinderGeometry((float)p.Radius, (float)p.Radius, (float)p.Height, 32, 1, false);
                node = _scene.AddMesh(body, _robotMaterial, null!, r.Name);
                var nose = _scene.AddMesh(_scene.CreateBoxGeometry((float)p.Radius, 0.03f, 0.04f, 1), _scene.CreateMaterial(new Vector4(0, 0.82f, 1f, 1), 0, 0.3f), node, r.Name + "_nose");
                nose.Position = new Vector3((float)(p.Radius * 0.6), (float)(p.Height / 2) + 0.02f, 0);
                _robots[r.Name] = node;
            }

            var pose = r.Pose;
            node.SetTransform(new Vector3((float)pose.X, (float)(r.Parameters.Height / 2), (float)-pose.Y), Quaternion.CreateFromAxisAngle(Vector3.UnitY, (float)pose.Theta), Vector3.One);
        }

        foreach (var o in _world.DynamicObstacles)
        {
            if (_dynamic.TryGetValue(o.Name, out var node))
            {
                node.Position = new Vector3((float)o.Center.X, node.Position.Y, (float)-o.Center.Y);
            }
        }
    }

    private void UpdateCamera()
    {
        var offset = new Vector3(
            (float)(Math.Cos(_pitch) * Math.Cos(_yaw) * _distance),
            (float)(Math.Sin(_pitch) * _distance),
            (float)(Math.Cos(_pitch) * Math.Sin(_yaw) * _distance));
        _camera.Position = _target + offset;
        _camera.LookAt(_target, Vector3.UnitY);
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
        if (_drag is { } d)
        {
            var p = e.GetPosition(this);
            _yaw += (p.X - d.X) * 0.008;
            _pitch = Math.Clamp(_pitch + ((p.Y - d.Y) * 0.008), 0.1, 1.5);
            _drag = p;
            UpdateCamera();
        }
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        base.OnPointerReleased(e);
        _drag = null;
        e.Pointer.Capture(null);
    }

    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        base.OnPointerWheelChanged(e);
        _distance = Math.Clamp(_distance * Math.Pow(0.9, e.Delta.Y), 1, 200);
        UpdateCamera();
        e.Handled = true;
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        _view.IsRendering = false;
        base.OnDetachedFromVisualTree(e);
    }
}
