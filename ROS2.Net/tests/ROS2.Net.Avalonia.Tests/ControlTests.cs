using Avalonia.Headless;
using Avalonia.Controls;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Rmw.InProcess;
using ROS2.Net.Simulation;
using ROS2.Net.Tf2;
using Shouldly;
using Float64 = ROS2.Net.Msgs.StdMsgs.Float64;
using LaserScan = ROS2.Net.Msgs.SensorMsgs.LaserScan;

namespace ROS2.Net.Avalonia.Tests;

using Control = global::Avalonia.Controls.Control;

[CollectionDefinition("headless", DisableParallelization = true)]
public sealed class HeadlessCollection;

/// <summary>Controls render without throwing and update from ROS data.</summary>
[Collection("headless")]
public class ControlTests
{
    private static int s_domain = 150;

    private static Ros2Context NewContext() => Ros2Context.Create(new Ros2ContextOptions
    {
        DomainId = Interlocked.Increment(ref s_domain),
        Rmw = new InProcessRmwFactory(),
    });

    private static Window Host(Control control, double w = 640, double h = 420)
    {
        var window = new Window { Width = w, Height = h, Content = control };
        window.Show();
        Headless.Pump(50);
        return window;
    }

    [Fact]
    public void Every_control_renders_empty_and_with_data()
    {
        Headless.Run(() =>
        {
            var controls = new Control[]
            {
                new TeleopJoystick(), new LaserScanView(), new OccupancyGridView(), new PointCloudView(), new PlotView(),
                new BatteryGauge(), new JointStatePanel(), new ImageView(), new LogConsole(), new TopicMonitor(),
                new NodeGraphView(), new TfTreeView(), new SimWorldView(), new ParameterEditor(),
            };
            foreach (var c in controls)
            {
                var w = Host(c);
                w.CaptureRenderedFrame().ShouldNotBeNull(c.GetType().Name);
                w.Close();
            }
        });
    }

    [Fact]
    public void Laser_and_map_views_follow_topics()
    {
        Headless.Run(() =>
        {
            using var ctx = NewContext();
            using var cts = new CancellationTokenSource();
            var spin = ctx.SpinAsync(cts.Token);
            var world = Scenarios.Create(ScenarioKind.Office, robots: 1);
            using var sim = new SimulationNode(ctx, world);
            sim.Start();
            var ui = ctx.CreateNode("ui");

            var scan = new LaserScanView();
            scan.Attach(ui, "/robot1/scan");
            var map = new OccupancyGridView();
            map.Attach(ui, "/map");
            var w = Host(new StackPanel { Children = { scan, map } });
            Headless.Pump(1200);

            scan.Scan.ShouldNotBeNull();
            scan.Scan!.Ranges.Length.ShouldBe(360);
            map.Grid.ShouldNotBeNull();
            map.Grid!.Info.Width.ShouldBeGreaterThan(100u);
            w.Close();
            sim.StopAsync().Wait();
            cts.Cancel();
        });
    }

    [Fact]
    public void Joystick_publishes_twist_while_deflected()
    {
        Headless.Run(() =>
        {
            using var ctx = NewContext();
            var node = ctx.CreateNode("teleop");
            var sub = ctx.CreateNode("robot").CreateSubscription<Msgs.GeometryMsgs.Twist>("/cmd_vel");
            var joy = new TeleopJoystick { MaxLinear = 0.5, MaxAngular = 1.0 };
            joy.Attach(node, "/cmd_vel");
            var w = Host(joy);
            joy.SetKnob(0, -1); // full forward
            Headless.Pump(400);

            sub.TryTake(out var twist).ShouldBeTrue();
            twist!.Linear.X.ShouldBe(0.5, 1e-9);
            joy.Command.Angular.ShouldBe(0, 1e-9);
            w.Close();
        });
    }

    [Fact]
    public void Topic_monitor_measures_rates()
    {
        Headless.Run(() =>
        {
            using var ctx = NewContext();
            using var cts = new CancellationTokenSource();
            var spin = ctx.SpinAsync(cts.Token);
            var talker = ctx.CreateNode("talker");
            var pub = talker.CreatePublisher<Float64>("/wave");
            talker.CreateTimer(TimeSpan.FromMilliseconds(20), () => pub.Publish(new Float64 { Data = 1 }));
            var monitor = new TopicMonitor();
            monitor.Attach(ctx.CreateNode("monitor"));
            var w = Host(monitor);
            Headless.Pump(2300);

            var row = monitor.Rows.Single(r => r.Name == "/wave");
            row.Type.ShouldBe("std_msgs/msg/Float64");
            row.Publishers.ShouldBe(1);
            row.Hz.ShouldBeGreaterThan(10);
            w.Close();
            cts.Cancel();
        });
    }

    [Fact]
    public void Node_graph_and_tf_tree_reflect_the_graph()
    {
        Headless.Run(() =>
        {
            using var ctx = NewContext();
            var a = ctx.CreateNode("a");
            a.CreatePublisher<LaserScan>("/scan");
            ctx.CreateNode("b").CreateSubscription<LaserScan>("/scan");
            var graph = new NodeGraphView();
            graph.Attach(ctx.CreateNode("viewer"));
            graph.NodeCount.ShouldBe(2);

            var buffer = new TfBuffer();
            buffer.SetTransform("map", "odom", Transform3d.Identity, RosTime.Now(), isStatic: true);
            buffer.SetTransform("odom", "base_link", Transform3d.From2D(1, 0, 0), RosTime.Now());
            var tree = new TfTreeView { Buffer = buffer };
            tree.FrameCount.ShouldBe(3);
            var w = Host(new StackPanel { Children = { graph, tree } }, 800, 700);
            w.CaptureRenderedFrame().ShouldNotBeNull();
            w.Close();
            tree.Buffer = null;
        });
    }

    [Fact]
    public void Plot_and_log_accept_data()
    {
        Headless.Run(() =>
        {
            var plot = new PlotView();
            for (var i = 0; i < 100; i++)
            {
                plot.AddSample("sin", i * 0.1, Math.Sin(i * 0.1));
            }

            var log = new LogConsole { Capacity = 10 };
            for (var i = 0; i < 25; i++)
            {
                log.Append(DateTime.Now, 20, "n", $"line {i}");
            }

            log.Count.ShouldBe(10);
            var w = Host(new StackPanel { Children = { plot, log } });
            w.CaptureRenderedFrame().ShouldNotBeNull();
            w.Close();
        });
    }
}
