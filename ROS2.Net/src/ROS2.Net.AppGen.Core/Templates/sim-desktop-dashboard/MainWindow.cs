using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using ROS2.Net;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Simulation;

namespace {{Namespace}};

/// <summary>Simulator dashboard: world view, teleop, LiDAR and battery for robot1.</summary>
public sealed class MainWindow : Window
{
    public MainWindow(Ros2Context context)
    {
        Title = "{{ProjectName}} — robot simulator";
        Width = 1280;
        Height = 780;
        var p = RosTheme.Current;
        Background = new SolidColorBrush(p.Background);

        // Simulator: a 2-robot warehouse (robot1 is driven). Physical robot: its own topics, no world view.
        var sim = AppTarget.StartSimulator(context, ScenarioKind.Warehouse, robots: 2);
        var ns = AppTarget.RobotNamespaces()[0];
        var node = context.CreateNode("dashboard");

        Control view;
        var status = new TextBlock { Text = AppTarget.Describe(), Foreground = new SolidColorBrush(p.Haze), FontFamily = RosTheme.MonoFamily, TextWrapping = TextWrapping.Wrap };
        if (AppTarget.World is { } world)
        {
            var worldView = new SimWorldView { ShowLidar = true, ShowTrails = true, SelectedRobot = "robot1" };
            worldView.Attach(world);
            view = worldView;
            world.Stepped += w =>
            {
                if (w.StepCount % 20 == 0)
                {
                    var r = w.Robots[0];
                    RosUi.OnUi(() => status.Text = $"t={w.Time:0.0}s  x={r.Pose.X:0.00}  y={r.Pose.Y:0.00}  v={r.LinearVelocity:0.00} m/s");
                }
            };
        }
        else
        {
            var map = new OccupancyGridView();
            map.Attach(node, "/map");
            view = map;
        }

        // SAFETY: joystick limits come from AppTarget (conservative defaults for real robots).
        var joystick = new TeleopJoystick { Height = 220, MaxLinear = AppTarget.MaxLinearSpeed, MaxAngular = AppTarget.MaxAngularSpeed };
        joystick.Attach(node, ns + "/cmd_vel");
        var scan = new LaserScanView { Height = 240 };
        scan.Attach(node, ns + "/scan");
        var battery = new BatteryGauge { Height = 140 };
        battery.Attach(node, ns + "/battery_state");

        var side = new StackPanel { Width = 300, Spacing = 12, Margin = new Thickness(16) };
        side.Children.Add(Heading("{{ProjectName}}", p));
        side.Children.Add(new TextBlock { Text = "Drag the joystick (or WASD) to drive the robot.", Foreground = new SolidColorBrush(p.Haze), TextWrapping = TextWrapping.Wrap });
        side.Children.Add(Card(joystick, p));
        side.Children.Add(Card(scan, p));
        side.Children.Add(Card(battery, p));
        side.Children.Add(status);

        var root = new DockPanel();
        DockPanel.SetDock(side, Dock.Right);
        root.Children.Add(side);
        root.Children.Add(new Border { Margin = new Thickness(16, 16, 0, 16), CornerRadius = new CornerRadius(12), ClipToBounds = true, Child = view });
        Content = root;
        Closed += (_, _) => sim?.Dispose();
    }

    private static TextBlock Heading(string text, RosPalette p) => new() { Text = text, FontSize = 22, FontWeight = FontWeight.Bold, Foreground = new SolidColorBrush(p.Ink) };

    private static Border Card(Control child, RosPalette p) => new()
    {
        Background = new SolidColorBrush(p.Surface),
        BorderBrush = new SolidColorBrush(p.Seam),
        BorderThickness = new Thickness(1),
        CornerRadius = new CornerRadius(12),
        Padding = new Thickness(10),
        Child = child,
    };
}
