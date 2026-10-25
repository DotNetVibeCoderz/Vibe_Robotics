using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Bag;
using ROS2.Net.Rmw.Rosbridge;
using ROS2.Net.Rmw.Rtps;
using ROS2.Net.Simulation;
using Imu = ROS2.Net.Msgs.SensorMsgs.Imu;
using LaserScan = ROS2.Net.Msgs.SensorMsgs.LaserScan;
using Odometry = ROS2.Net.Msgs.NavMsgs.Odometry;
using Twist = ROS2.Net.Msgs.GeometryMsgs.Twist;

namespace Ros2NetGallery.Pages;

/// <summary>rosbag2 (MCAP) record, inspect and replay.</summary>
public sealed class BagPage : GalleryPage
{
    public override string Id => "bag";

    public override PageGroup Group => PageGroup.Tools;

    public override T2 Title => new("rosbag2 recording", "Perekaman rosbag2");

    public override T2 Summary => new(
        "Record raw CDR from a moving robot into an MCAP bag with a rosbag2 metadata.yaml, inspect it like `ros2 bag info`, and replay the scan onto a remapped topic.",
        "Rekam CDR mentah dari robot yang bergerak ke bag MCAP dengan metadata.yaml rosbag2, periksa seperti `ros2 bag info`, lalu putar ulang scan ke topic yang di-remap.");

    public override IReadOnlyList<string> Interfaces => ["MCAP · ros2msg · cdr", "/robot1/scan", "/robot1/odom", "/replay/scan"];

    public override string Code => """
        await using var recorder = new BagRecorder(node, "bags/run1", new BagRecorderOptions
        {
            Topics = { "/robot1/scan", "/robot1/odom", "/tf" },
            Writer = new BagWriterOptions { Compression = McapCompression.Zstd },
        });
        recorder.Start();
        await Task.Delay(TimeSpan.FromSeconds(4));
        await recorder.StopAsync();
        Console.WriteLine(BagInfo.Read("bags/run1"));          // like `ros2 bag info`

        using var player = new BagPlayer(node, "bags/run1", new BagPlayerOptions
        {
            Loop = true, Remappings = { ["/robot1/scan"] = "/replay/scan" },
        });
        await player.PlayAsync(ct);
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var p = RosTheme.Current;
        var world = Scenarios.Create(ScenarioKind.Warehouse, robots: 1, seed: 21);
        Own(new Wanderer(world));
        SimPages.StartSim(runtime, world, d => Own(d), OwnAction);
        var node = Own(runtime.Context.CreateNode("gallery_bag"));
        var dir = Path.Combine(Path.GetTempPath(), "ros2net-gallery-bags", $"run_{DateTime.Now:yyyyMMdd_HHmmss}");
        var info = Ui.Text(Loc.T("No bag recorded yet.", "Belum ada bag yang direkam."), 11, mono: true, color: p.Haze, wrap: false);
        var live = new LaserScanView { Title = "/robot1/scan (live)" };
        live.Attach(node, "/robot1/scan");
        var replay = new LaserScanView { Title = "/replay/scan (bag)" };
        replay.Attach(node, "/replay/scan");
        var status = Ui.Chip("idle", p.Haze);
        var cts = new CancellationTokenSource();
        OwnAction(cts.Cancel);
        BagPlayer? player = null;
        OwnAction(() => player?.Dispose());
        var busy = false;

        void SetStatus(string s, Color c)
        {
            status.Child = Ui.Text(s, 11, mono: true, wrap: false);
            status.BorderBrush = new SolidColorBrush(c, 0.6);
            status.Background = new SolidColorBrush(c, 0.15);
        }

        async Task RecordAsync()
        {
            if (busy)
            {
                return;
            }

            busy = true;
            player?.Dispose();
            player = null;
            try
            {
                dir = Path.Combine(Path.GetTempPath(), "ros2net-gallery-bags", $"run_{DateTime.Now:yyyyMMdd_HHmmss_fff}");
                SetStatus("● recording", p.Fault);
                var recorder = new BagRecorder(node, dir, new BagRecorderOptions
                {
                    Topics = { "/robot1/scan", "/robot1/odom", "/robot1/imu", "/tf" },
                });
                recorder.Start();
                await Task.Delay(TimeSpan.FromSeconds(3), cts.Token);
                await recorder.StopAsync();
                info.Text = BagInfo.Read(dir).ToString();
                info.Foreground = new SolidColorBrush(p.Ink);
                SetStatus("▶ replaying", p.Ok);
                player = new BagPlayer(node, dir, new BagPlayerOptions { Loop = true, Remappings = { ["/robot1/scan"] = "/replay/scan", ["/robot1/odom"] = "/replay/odom", ["/robot1/imu"] = "/replay/imu", ["/tf"] = "/replay/tf" } });
                _ = player.PlayAsync(cts.Token);
            }
            catch (OperationCanceledException)
            {
            }
            catch (Exception ex)
            {
                SetStatus("error", p.Fault);
                info.Text = ex.Message;
            }
            finally
            {
                busy = false;
            }
        }

        var controls = Ui.Card(Ui.Row(14,
            Ui.Button(Loc.T("Record 3 s", "Rekam 3 dtk"), () => _ = RecordAsync(), primary: true),
            Ui.Button(Loc.T("Open folder path", "Salin path folder"), () => info.Text = dir + "\n\n" + info.Text),
            status), padding: 14);
        Dispatcher.UIThread.Post(() => _ = RecordAsync(), DispatcherPriority.Background);
        return Ui.Rows("Auto,*,Auto", 14, controls, Ui.Columns("*,*", 14, live, replay), Ui.Card(info, "ros2 bag info", 14));
    }
}

/// <summary>Middleware switch, node graph and topic monitor.</summary>
public sealed class ConnectivityPage : GalleryPage
{
    public override string Id => "connectivity";

    public override PageGroup Group => PageGroup.Tools;

    public override T2 Title => new("Connectivity & graph", "Konektivitas & graf");

    public override T2 Summary => new(
        "The same API runs on three middlewares: in-process (zero copy), managed DDS-RTPS (talks to ROS 2 nodes on the LAN) and rosbridge (WebSocket). The graph and topic rates below come from the selected middleware's discovery.",
        "API yang sama berjalan di tiga middleware: in-process (tanpa salinan), DDS-RTPS terkelola (berbicara dengan node ROS 2 di LAN), dan rosbridge (WebSocket). Graf dan laju topic di bawah berasal dari discovery middleware yang dipilih.");

    public override IReadOnlyList<string> Interfaces => ["ROS2NET_RMW", "rtps · rosbridge · inprocess", "/scan", "/odom", "/cmd_vel"];

    public override string Code => """
        // Talk to real ROS 2 nodes on the network (Fast DDS / Cyclone DDS wire protocol):
        await using var context = await Ros2Context.CreateAsync(new Ros2ContextOptions().UseRtps());

        // ...or through rosbridge_server, e.g. across NAT:
        await using var remote = await Ros2Context.CreateAsync(
            new Ros2ContextOptions().UseRosbridge("ws://robot.local:9090"));

        graph.Attach(context.CreateNode("graph_viewer"));
        topics.Attach(context.CreateNode("topic_monitor"));
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var p = RosTheme.Current;
        var graph = new NodeGraphView();
        var topics = new TopicMonitor();
        var status = Ui.Muted(string.Empty, 12);
        Ros2Context? external = null;
        OwnAction(() => external?.Dispose());

        void BuildDemoGraph(Ros2Context ctx)
        {
            var driver = Own(ctx.CreateNode("lidar_driver"));
            var scan = driver.CreatePublisher<LaserScan>("/scan", QosProfile.SensorData);
            driver.CreateTimer(TimeSpan.FromMilliseconds(100), () => scan.Publish(new LaserScan { Ranges = new float[360], RangeMax = 4 }));
            var imuDriver = Own(ctx.CreateNode("imu_driver"));
            var imu = imuDriver.CreatePublisher<Imu>("/imu", QosProfile.SensorData);
            imuDriver.CreateTimer(TimeSpan.FromMilliseconds(10), () => imu.Publish(new Imu()));
            var fusion = Own(ctx.CreateNode("ekf_fusion"));
            var odom = fusion.CreatePublisher<Odometry>("/odom");
            fusion.CreateSubscription<Imu>("/imu", _ => { }, QosProfile.SensorData);
            fusion.CreateTimer(TimeSpan.FromMilliseconds(33), () => odom.Publish(new Odometry()));
            var planner = Own(ctx.CreateNode("local_planner"));
            var cmd = planner.CreatePublisher<Twist>("/cmd_vel");
            planner.CreateSubscription<Odometry>("/odom", _ => { });
            planner.CreateSubscription<LaserScan>("/scan", _ => { }, QosProfile.SensorData);
            planner.CreateTimer(TimeSpan.FromMilliseconds(50), () => cmd.Publish(new Twist()));
            var bas = Own(ctx.CreateNode("base_controller"));
            bas.CreateSubscription<Twist>("/cmd_vel", _ => { });
        }

        void AttachViews(Ros2Context ctx)
        {
            graph.Attach(Own(ctx.CreateNode("gallery_graph_viewer")));
            topics.Attach(Own(ctx.CreateNode("gallery_topic_monitor")));
        }

        BuildDemoGraph(runtime.Context);
        AttachViews(runtime.Context);
        status.Text = Loc.T("Showing the gallery's in-process graph.", "Menampilkan graf in-process milik galeri.");

        var rmw = new ComboBox { ItemsSource = new[] { "inprocess", "rtps", "rosbridge" }, SelectedIndex = 0, MinWidth = 140 };
        var url = new TextBox { Text = "ws://localhost:9090", MinWidth = 220 };
        var domain = new NumericUpDown { Value = 0, Minimum = 0, Maximum = 232, FormatString = "0", MinWidth = 110 };
        async Task ConnectAsync()
        {
            var choice = rmw.SelectedItem as string ?? "inprocess";
            if (choice == "inprocess")
            {
                graph.Attach(Own(runtime.Context.CreateNode("gallery_graph_viewer2")));
                topics.Attach(Own(runtime.Context.CreateNode("gallery_topic_monitor2")));
                status.Text = Loc.T("Showing the gallery's in-process graph.", "Menampilkan graf in-process milik galeri.");
                return;
            }

            status.Text = Loc.T("Connecting…", "Menghubungkan…");
            try
            {
                external?.Dispose();
                var options = new Ros2ContextOptions { DomainId = (int)(domain.Value ?? 0) };
                options = choice == "rtps" ? options.UseRtps() : options.UseRosbridge(url.Text ?? "ws://localhost:9090");
                external = await Ros2Context.CreateAsync(options);
                _ = external.SpinAsync();
                BuildDemoGraph(external);
                AttachViews(external);
                status.Text = choice == "rtps"
                    ? Loc.T($"RTPS participant on domain {options.DomainId}. ROS 2 nodes on this network appear below.", $"Participant RTPS di domain {options.DomainId}. Node ROS 2 di jaringan ini akan muncul di bawah.")
                    : Loc.T($"Connected to {url.Text}.", $"Terhubung ke {url.Text}.");
                status.Foreground = new SolidColorBrush(p.Ok);
            }
            catch (Exception ex)
            {
                status.Text = ex.Message;
                status.Foreground = new SolidColorBrush(p.Fault);
            }
        }

        var bar = Ui.Card(Ui.Col(8, Ui.Row(12,
            Ui.Col(4, Ui.Muted("middleware", 12), rmw),
            Ui.Col(4, Ui.Muted("ROS_DOMAIN_ID", 12), domain),
            Ui.Col(4, Ui.Muted("rosbridge url", 12), url),
            Ui.Button(Loc.T("Connect", "Hubungkan"), () => _ = ConnectAsync(), primary: true)), status), padding: 14);
        return Ui.Rows("Auto,3*,2*", 14, bar, graph, topics);
    }
}
