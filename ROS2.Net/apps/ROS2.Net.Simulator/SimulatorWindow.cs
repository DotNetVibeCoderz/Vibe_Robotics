using System.Diagnostics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using Avalonia.Threading;
using ROS2.Net;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Rmw.InProcess;
using ROS2.Net.Rmw.Rosbridge;
using ROS2.Net.Rmw.Rtps;
using ROS2.Net.Simulation;
using Imu = ROS2.Net.Msgs.SensorMsgs.Imu;
using SimPose = ROS2.Net.Simulation.Pose2D;

namespace Ros2NetSimulator;

/// <summary>
/// The Robot Simulator: pick a scenario, drive robots, watch their sensors, and expose everything as
/// ROS 2 topics on the chosen middleware so external nodes (Nav2, RViz, your own) can use it.
/// </summary>
public sealed class SimulatorWindow : Window
{
    private readonly SimWorldView _view2D = new() { Title = "World · top-down" };
    private readonly ContentControl _view3DHost = new();
    private readonly TeleopJoystick _joystick = new() { Title = "cmd_vel", MaxLinear = 0.22, MaxAngular = 2.0 };
    private readonly LaserScanView _scan = new();
    private readonly BatteryGauge _battery = new();
    private readonly ImageView _camera = new();
    private readonly PlotView _imu = new() { Title = "IMU", WindowSeconds = 8 };
    private readonly TopicMonitor _topics = new();
    private readonly ComboBox _scenarioPicker = new() { ItemsSource = Enum.GetNames<ScenarioKind>(), MinWidth = 150 };
    private readonly ComboBox _robotPicker = new() { MinWidth = 120 };
    private readonly ComboBox _rmwPicker = new() { ItemsSource = new[] { "inprocess", "rtps", "rosbridge" }, MinWidth = 120 };
    private readonly TextBox _url = new() { Text = "ws://localhost:9090", MinWidth = 170, IsVisible = false };
    private readonly TextBlock _status = new() { FontSize = 11.5, VerticalAlignment = VerticalAlignment.Center };
    private readonly Button _playPause = new();
    private readonly List<IDisposable> _subscriptions = [];
    private readonly Stopwatch _clock = Stopwatch.StartNew();
    private Ros2Context? _context;
    private SimulationNode? _sim;
    private Node? _ui;
    private SimWorld? _world;
    private string _rmw;
    private string _robot = "robot1";
    private bool _starting;

    public SimulatorWindow(string rmw = "inprocess", string scenario = "Warehouse")
    {
        _rmw = rmw;
        Title = "ROS2.Net Robot Simulator";
        Width = 1440;
        Height = 900;
        MinWidth = 1100;
        MinHeight = 700;
        Background = new SolidColorBrush(RosTheme.Current.Background);
        try
        {
            Icon = new WindowIcon(AssetLoader.Open(new Uri("avares://ROS2.Net.Simulator/Assets/icon-128.png")));
        }
        catch (Exception)
        {
        }

        _scenarioPicker.SelectedItem = Enum.TryParse<ScenarioKind>(scenario, true, out var k) ? k.ToString() : "Warehouse";
        _rmwPicker.SelectedItem = rmw;
        _rmwPicker.SelectionChanged += (_, _) => _url.IsVisible = (_rmwPicker.SelectedItem as string) == "rosbridge";
        _url.IsVisible = rmw == "rosbridge";
        Content = BuildLayout();
        _scenarioPicker.SelectionChanged += (_, _) => _ = RestartAsync();
        _robotPicker.SelectionChanged += (_, _) =>
        {
            if (_robotPicker.SelectedItem is string r && r != _robot)
            {
                _robot = r;
                AttachRobot();
            }
        };
        _view2D.WorldClicked += (x, y) =>
        {
            // Click a robot to select it.
            var hit = _world?.Robots.MinBy(r => Math.Pow(r.Pose.X - x, 2) + Math.Pow(r.Pose.Y - y, 2));
            if (hit is not null && Math.Pow(hit.Pose.X - x, 2) + Math.Pow(hit.Pose.Y - y, 2) < 0.25)
            {
                _robotPicker.SelectedItem = hit.Name;
            }
        };
        var statusTimer = new DispatcherTimer(TimeSpan.FromMilliseconds(250), DispatcherPriority.Background, (_, _) => UpdateStatus());
        statusTimer.Start();
        Closed += (_, _) =>
        {
            statusTimer.Stop();
            _ = StopAsync();
        };
        _ = RestartAsync();
    }

    /// <summary>The running world (for tests and tooling).</summary>
    public SimWorld? World => _world;

    /// <summary>The context the simulator publishes on.</summary>
    public Ros2Context? Context => _context;

    /// <summary>Completes when the current (re)start has finished.</summary>
    public Task Ready { get; private set; } = Task.CompletedTask;

    private Control BuildLayout()
    {
        var p = RosTheme.Current;
        Control logo;
        try
        {
            logo = new Image { Source = new Bitmap(AssetLoader.Open(new Uri("avares://ROS2.Net.Simulator/Assets/icon-128.png"))), Width = 30, Height = 30 };
        }
        catch (Exception)
        {
            logo = new Border { Width = 30, Height = 30, CornerRadius = new CornerRadius(8), Background = p.SignalGradient() };
        }

        Button B(string text, Action click, bool primary = false)
        {
            var b = new Button
            {
                Content = new TextBlock { Text = text, FontWeight = FontWeight.SemiBold, FontSize = 12.5, Foreground = primary ? Brushes.White : new SolidColorBrush(p.Ink) },
                Padding = new Thickness(12, 6),
                CornerRadius = new CornerRadius(8),
                Background = primary ? p.SignalGradient() : new SolidColorBrush(p.SurfaceRaised),
                BorderBrush = new SolidColorBrush(primary ? p.Signal : p.Seam),
                BorderThickness = new Thickness(1),
            };
            b.Click += (_, _) => click();
            return b;
        }

        _playPause.Padding = new Thickness(12, 6);
        _playPause.CornerRadius = new CornerRadius(8);
        _playPause.Click += (_, _) =>
        {
            if (_world is { } w)
            {
                w.Paused = !w.Paused;
                UpdatePlayPause();
            }
        };
        UpdatePlayPause();

        var rtf = new Slider { Minimum = 0.25, Maximum = 4, Value = 1, Width = 90, VerticalAlignment = VerticalAlignment.Center };
        var rtfText = new TextBlock { Text = "1.00×", FontFamily = RosTheme.MonoFamily, Foreground = new SolidColorBrush(p.Telemetry), VerticalAlignment = VerticalAlignment.Center };
        rtf.ValueChanged += (_, e) =>
        {
            rtfText.Text = $"{e.NewValue:0.00}×";
            if (_world is { } w)
            {
                w.RealTimeFactor = e.NewValue;
            }
        };

        StackPanel Group(params Control[] items)
        {
            var s = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 8 };
            foreach (var i in items)
            {
                i.VerticalAlignment = VerticalAlignment.Center;
                s.Children.Add(i);
            }

            return s;
        }

        TextBlock Label(string t) => new() { Text = t, FontSize = 11, Foreground = new SolidColorBrush(p.Haze), VerticalAlignment = VerticalAlignment.Center };

        var toolbar = new Border
        {
            Background = new SolidColorBrush(p.Surface),
            BorderBrush = new SolidColorBrush(p.Seam),
            BorderThickness = new Thickness(0, 0, 0, 1),
            Padding = new Thickness(16, 10),
            Child = new DockPanel
            {
                Children =
                {
                    Docked(Group(B("About", ShowAbout)), global::Avalonia.Controls.Dock.Right),
                    Group(
                        logo,
                        new StackPanel { Children = { new TextBlock { Text = "ROS2.Net Simulator", FontWeight = FontWeight.Bold, FontSize = 15, Foreground = new SolidColorBrush(p.Ink) }, new TextBlock { Text = "robots · sensors · ROS 2 topics", FontSize = 10.5, Foreground = new SolidColorBrush(p.Haze) } } },
                        new Border { Width = 18 },
                        Label("SCENARIO"), _scenarioPicker,
                        _playPause,
                        B("Step", () => _world?.Step(10)),
                        B("Reset", () => _world?.Reset()),
                        Label("SPEED"), rtf, rtfText,
                        B("+ Robot", SpawnRobot),
                        new Border { Width = 18 },
                        Label("MIDDLEWARE"), _rmwPicker, _url,
                        B("Apply", () =>
                        {
                            _rmw = _rmwPicker.SelectedItem as string ?? "inprocess";
                            _ = RestartAsync();
                        }, primary: true)),
                },
            },
        };

        var externalHint = new Border
        {
            Background = new SolidColorBrush(p.Surface),
            BorderBrush = new SolidColorBrush(p.Seam),
            BorderThickness = new Thickness(1),
            CornerRadius = new CornerRadius(10),
            Padding = new Thickness(12),
            Child = new StackPanel
            {
                Spacing = 4,
                Children =
                {
                    new TextBlock { Text = "EXTERNAL NODES", FontSize = 10.5, FontWeight = FontWeight.SemiBold, Foreground = new SolidColorBrush(p.Haze) },
                    new TextBlock { Text = "ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2}}\"", FontFamily = RosTheme.MonoFamily, FontSize = 10.5, TextWrapping = TextWrapping.Wrap, Foreground = new SolidColorBrush(p.Telemetry) },
                    new TextBlock { Text = "Use the rtps middleware (domain from ROS_DOMAIN_ID) to reach ROS 2 nodes on this network, or rosbridge to go through rosbridge_server.", FontSize = 11, TextWrapping = TextWrapping.Wrap, Foreground = new SolidColorBrush(p.Haze) },
                },
            },
        };
        externalHint.Margin = new Thickness(0, 0, 0, 12);

        var tabs = new TabControl
        {
            Items =
            {
                new TabItem { Header = "2D", Content = _view2D },
                new TabItem { Header = "3D", Content = _view3DHost },
                new TabItem { Header = "Topics", Content = new ScrollViewer { Content = new StackPanel { Children = { externalHint, _topics } } } },
            },
        };
        tabs.SelectionChanged += (_, _) =>
        {
            // Create the GPU view lazily, only when the tab is opened.
            if (tabs.SelectedIndex == 1 && _view3DHost.Content is null && _world is { } w)
            {
                try
                {
                    _view3DHost.Content = new Sim3DView(w);
                }
                catch (Exception ex)
                {
                    _view3DHost.Content = new TextBlock { Text = "3D view unavailable: " + ex.Message, Margin = new Thickness(20), Foreground = new SolidColorBrush(p.Haze) };
                }
            }
        };

        var robotRow = Group(Label("ROBOT"), _robotPicker);
        var side = new Grid
        {
            Width = 340,
            RowDefinitions = new RowDefinitions("Auto,200,140,*,150"),
            Margin = new Thickness(14, 0, 0, 0),
        };
        void Put(Control c, int row, double top = 10)
        {
            c.Margin = new Thickness(0, row == 0 ? 0 : top, 0, 0);
            Grid.SetRow(c, row);
            side.Children.Add(c);
        }

        Put(robotRow, 0);
        Put(_joystick, 1);
        Put(new Grid { ColumnDefinitions = new ColumnDefinitions("*,*"), Children = { _battery, Col1(_camera) } }, 2);
        Put(_scan, 3);
        Put(_imu, 4);

        static Control Col1(Control c)
        {
            Grid.SetColumn(c, 1);
            c.Margin = new Thickness(10, 0, 0, 0);
            return c;
        }

        var statusBar = new Border
        {
            Background = new SolidColorBrush(p.Surface),
            BorderBrush = new SolidColorBrush(p.Seam),
            BorderThickness = new Thickness(0, 1, 0, 0),
            Padding = new Thickness(16, 6),
            Child = new DockPanel
            {
                Children =
                {
                    Docked(new TextBlock { Text = "Made by Gravicode Studios, led by Kang Fadhil", FontSize = 11, Foreground = new SolidColorBrush(p.Haze), VerticalAlignment = VerticalAlignment.Center }, global::Avalonia.Controls.Dock.Right),
                    _status,
                },
            },
        };

        var main = new DockPanel { Margin = new Thickness(16, 14, 16, 14) };
        main.Children.Add(Docked(side, global::Avalonia.Controls.Dock.Right));
        main.Children.Add(tabs);
        var root = new DockPanel();
        root.Children.Add(Docked(toolbar, global::Avalonia.Controls.Dock.Top));
        root.Children.Add(Docked(statusBar, global::Avalonia.Controls.Dock.Bottom));
        root.Children.Add(main);
        return root;
    }

    private static T Docked<T>(T c, global::Avalonia.Controls.Dock d)
        where T : Control
    {
        DockPanel.SetDock(c, d);
        return c;
    }

    private void UpdatePlayPause()
    {
        var p = RosTheme.Current;
        var paused = _world?.Paused ?? false;
        _playPause.Content = new TextBlock { Text = paused ? "▶ Play" : "⏸ Pause", FontWeight = FontWeight.SemiBold, FontSize = 12.5, Foreground = new SolidColorBrush(p.Ink) };
        _playPause.Background = new SolidColorBrush(paused ? p.Alert : p.SurfaceRaised, paused ? 0.25 : 1);
    }

    private void UpdateStatus()
    {
        var p = RosTheme.Current;
        if (_world is not { } w || _context is null)
        {
            _status.Text = _starting ? "Starting…" : "Stopped";
            return;
        }

        var topics = _topics.Rows.Count;
        _status.Text = $"● {_rmw}   domain {_context.DomainId}   t = {w.Time:0.0} s   robots {w.Robots.Count}   topics {topics}   {(w.Paused ? "paused" : $"{w.RealTimeFactor:0.00}× real time")}";
        _status.Foreground = new SolidColorBrush(w.Paused ? p.Alert : p.Ok);
    }

    private async Task StopAsync()
    {
        foreach (var s in _subscriptions)
        {
            s.Dispose();
        }

        _subscriptions.Clear();
        _joystick.Detach();
        _scan.Detach();
        _topics.Detach();
        if (_sim is { } sim)
        {
            await sim.StopAsync();
            sim.Dispose();
        }

        _sim = null;
        _ui = null;
        if (_context is { } ctx)
        {
            await ctx.DisposeAsync();
        }

        _context = null;
    }

    private Task RestartAsync() => Ready = RestartCoreAsync();

    private async Task RestartCoreAsync()
    {
        if (_starting)
        {
            return;
        }

        _starting = true;
        try
        {
            await StopAsync();
            var options = new Ros2ContextOptions { Arguments = Environment.GetCommandLineArgs() };
            options = _rmw switch
            {
                "rtps" => options.UseRtps(),
                "rosbridge" => options.UseRosbridge(_url.Text ?? "ws://localhost:9090"),
                _ => new Ros2ContextOptions { Rmw = new InProcessRmwFactory(), DomainId = options.DomainId },
            };
            _context = await Ros2Context.CreateAsync(options);
            _ = _context.SpinAsync(executor: new MultiThreadedExecutor(2));

            var kind = Enum.TryParse<ScenarioKind>(_scenarioPicker.SelectedItem as string, out var k) ? k : ScenarioKind.Warehouse;
            _world = Scenarios.Create(kind, robots: kind == ScenarioKind.Warehouse ? 2 : 1, sensors: new RobotSensorOptions { Camera = true });
            foreach (var r in _world.Robots)
            {
                if (r.Battery is { } b)
                {
                    b.DrainFactor = 10;
                }
            }

            _sim = new SimulationNode(_context, _world, new SimulationNodeOptions { PublishMapToOdom = true });
            _sim.Start();
            _ui = _context.CreateNode("simulator_ui");
            _view2D.Attach(_world);
            _view3DHost.Content = null;
            _topics.Attach(_ui);
            _robot = _world.Robots.FirstOrDefault()?.Name ?? "robot1";
            _robotPicker.ItemsSource = _world.Robots.Select(r => r.Name).ToList();
            _robotPicker.SelectedItem = _robot;
            AttachRobot();
            UpdatePlayPause();
        }
        catch (Exception ex)
        {
            _status.Text = $"Could not start on '{_rmw}': {ex.Message}";
            _status.Foreground = new SolidColorBrush(RosTheme.Current.Fault);
        }
        finally
        {
            _starting = false;
        }
    }

    private void AttachRobot()
    {
        if (_ui is not { } ui)
        {
            return;
        }

        foreach (var s in _subscriptions)
        {
            s.Dispose();
        }

        _subscriptions.Clear();
        var ns = "/" + _robot;
        _view2D.SelectedRobot = _robot;
        _joystick.Title = ns + "/cmd_vel";
        _joystick.Attach(ui, ns + "/cmd_vel");
        _scan.Title = ns + "/scan";
        _scan.Attach(ui, ns + "/scan");
        _battery.Attach(ui, ns + "/battery_state");
        _camera.Title = ns + "/camera";
        _camera.Attach(ui, ns + "/camera/image_raw");
        _imu.Clear();
        _imu.Title = ns + "/imu";
        _subscriptions.Add(RosUi.SubscribeAll<Imu>(ui, ns + "/imu", batch =>
        {
            var t = _clock.Elapsed.TotalSeconds;
            var m = batch[^1];
            _imu.AddSample("gyro z", t, m.AngularVelocity.Z);
            _imu.AddSample("acc x", t, m.LinearAcceleration.X);
        }, QosProfile.SensorData));
    }

    private void SpawnRobot()
    {
        if (_world is not { } w)
        {
            return;
        }

        var rng = new Random();
        for (var attempt = 0; attempt < 200; attempt++)
        {
            var x = w.Bounds.MinX + 0.5 + (rng.NextDouble() * (w.Bounds.Width - 1));
            var y = w.Bounds.MinY + 0.5 + (rng.NextDouble() * (w.Bounds.Height - 1));
            if (w.IsOccupied(new Vec2(x, y), 0.35))
            {
                continue;
            }

            var name = Enumerable.Range(1, 100).Select(i => $"robot{i}").First(n => w.GetRobot(n) is null);
            w.AddRobot(RobotFactory.Create(RobotKind.DiffDrive, name, new SimPose(x, y, rng.NextDouble() * Math.PI * 2), sensors: new RobotSensorOptions { Camera = true }));
            _robotPicker.ItemsSource = w.Robots.Select(r => r.Name).ToList();
            _robotPicker.SelectedItem = name;
            return;
        }
    }

    private void ShowAbout()
    {
        var p = RosTheme.Current;
        var dialog = new Window
        {
            Title = "About ROS2.Net Simulator",
            Width = 460,
            Height = 300,
            Background = new SolidColorBrush(p.Surface),
            WindowStartupLocation = WindowStartupLocation.CenterOwner,
            Content = new StackPanel
            {
                Margin = new Thickness(24),
                Spacing = 10,
                Children =
                {
                    new TextBlock { Text = "ROS2.Net Robot Simulator", FontSize = 20, FontWeight = FontWeight.Bold, Foreground = new SolidColorBrush(p.Ink) },
                    new TextBlock { Text = "A deterministic 2.5D simulator that speaks ROS 2: /cmd_vel in; odom, scan, imu, gps, battery, camera, tf, clock and map out.", TextWrapping = TextWrapping.Wrap, Foreground = new SolidColorBrush(p.Haze) },
                    new TextBlock { Text = "Made by Gravicode Studios, led by Kang Fadhil", FontWeight = FontWeight.SemiBold, Foreground = new SolidColorBrush(p.Ink) },
                    new TextBlock { Text = "Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil", Foreground = new SolidColorBrush(p.Haze) },
                    new TextBlock { Text = "MIT licence · github.com/DotNetVibeCoderz", FontSize = 11, Foreground = new SolidColorBrush(p.Haze) },
                },
            },
        };
        dialog.ShowDialog(this);
    }
}
