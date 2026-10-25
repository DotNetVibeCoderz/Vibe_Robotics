using System.Diagnostics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.AI.Gym;
using ROS2.Net.AI.Llm;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Native;
using ROS2.Net.Perception;
using ROS2.Net.Simulation;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace Ros2NetGallery.Pages;

/// <summary>Point-cloud filtering and segmentation with the Rust kernels.</summary>
public sealed class PerceptionPage : GalleryPage
{
    public override string Id => "perception";

    public override PageGroup Group => PageGroup.Intelligence;

    public override T2 Title => new("Point clouds", "Point cloud");

    public override T2 Summary => new(
        "A noisy synthetic depth scene goes through a PCL-style pipeline: voxel grid, statistical outlier removal, RANSAC floor segmentation and Euclidean clustering. Heavy kernels run in the Rust library when it is present.",
        "Adegan kedalaman sintetis yang berderau melewati pipeline ala PCL: voxel grid, statistical outlier removal, segmentasi lantai RANSAC, dan Euclidean clustering. Kernel berat berjalan di pustaka Rust bila tersedia.");

    public override IReadOnlyList<string> Interfaces => ["sensor_msgs/msg/PointCloud2", "ros2net_native (Rust)", "vision_msgs"];

    public override string Code => """
        var cloud = PointCloud.FromMessage(msg, skipNonFinite: true);
        cloud = Filters.VoxelGrid(cloud, leafSize: 0.03f);
        cloud = Filters.StatisticalOutlierRemoval(cloud, meanK: 16, stdDevMultiplier: 1.0);

        var floor = Segmentation.SegmentPlane(cloud, distanceThreshold: 0.02);
        var objects = cloud.Select(floor.Outliers);
        var clusters = Segmentation.EuclideanClusters(objects, tolerance: 0.06f, minSize: 30);

        Console.WriteLine(NativeLibraryInfo.IsAvailable ? "Rust kernels" : "managed fallback");
        publisher.Publish(objects.ToMessage("base_link"));
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var p = RosTheme.Current;
        var raw = MakeScene();
        var view = new PointCloudView { Title = Loc.T("Filtered cloud · drag to orbit", "Cloud hasil filter · seret untuk memutar") };
        var leaf = 0.03f;
        var sor = true;
        var removeFloor = true;
        var (inView, inText) = Ui.Stat(Loc.T("Input points", "Titik masukan"), raw.Count.ToString("N0"));
        var (outView, outText) = Ui.Stat(Loc.T("After filters", "Setelah filter"));
        var (clView, clText) = Ui.Stat(Loc.T("Objects", "Objek"));
        var (msView, msText) = Ui.Stat(Loc.T("Pipeline time", "Waktu pipeline"));
        var version = 0;

        void Run()
        {
            var my = ++version;
            var (l, s, f) = (leaf, sor, removeFloor);
            _ = Task.Run(() =>
            {
                var sw = Stopwatch.StartNew();
                var cloud = Filters.VoxelGrid(raw, l);
                if (s)
                {
                    cloud = Filters.StatisticalOutlierRemoval(cloud, 16, 1.0);
                }

                PointCloud? floor = null;
                var objects = cloud;
                if (f)
                {
                    var seg = Segmentation.SegmentPlane(cloud, 0.02, 300);
                    floor = cloud.Select(seg.Inliers);
                    objects = cloud.Select(seg.Outliers);
                }

                var clusters = Segmentation.EuclideanClusters(objects, 0.06f, 30);
                sw.Stop();
                Dispatcher.UIThread.Post(() =>
                {
                    if (my != version)
                    {
                        return;
                    }

                    view.ClearLayers();
                    if (floor is not null)
                    {
                        view.AddLayer(floor.Xyz.ToArray(), Color.FromArgb(110, p.Haze.R, p.Haze.G, p.Haze.B), 1.4);
                    }

                    var palette = new[] { p.Telemetry, p.Signal, p.Alert, p.Ok, p.Fault, Color.Parse("#B9A6FF") };
                    for (var i = 0; i < clusters.Count; i++)
                    {
                        view.AddLayer(objects.Select(clusters[i]).Xyz.ToArray(), palette[i % palette.Length], 2.0);
                    }

                    if (clusters.Count == 0)
                    {
                        view.AddLayer(objects.Xyz.ToArray());
                    }

                    outText.Text = cloud.Count.ToString("N0");
                    clText.Text = clusters.Count.ToString();
                    msText.Text = $"{sw.Elapsed.TotalMilliseconds:0} ms";
                });
            });
        }

        var backend = Ui.Chip(NativeLibraryInfo.IsAvailable ? $"Rust kernels · {NativeLibraryInfo.Version}" : "managed fallback", NativeLibraryInfo.IsAvailable ? p.Ok : p.Alert);
        var sorToggle = new ToggleSwitch { IsChecked = true, OnContent = "SOR", OffContent = "SOR" };
        sorToggle.IsCheckedChanged += (_, _) => { sor = sorToggle.IsChecked == true; Run(); };
        var floorToggle = new ToggleSwitch { IsChecked = true, OnContent = Loc.T("Remove floor", "Buang lantai"), OffContent = Loc.T("Keep floor", "Simpan lantai") };
        floorToggle.IsCheckedChanged += (_, _) => { removeFloor = floorToggle.IsChecked == true; Run(); };
        var projection = new ComboBox { ItemsSource = Enum.GetNames<CloudProjection>(), SelectedIndex = 3 };
        projection.SelectionChanged += (_, _) => { view.Projection = (CloudProjection)Math.Max(0, projection.SelectedIndex); view.InvalidateVisual(); };

        var side = Ui.Col(16,
            Ui.Card(Ui.Col(12,
                backend,
                Ui.Slider(Loc.T("Voxel leaf", "Ukuran voxel"), 0.01, 0.10, leaf, v => { leaf = (float)v; Run(); }, "0.000", " m"),
                sorToggle, floorToggle,
                Ui.Col(4, Ui.Muted(Loc.T("Projection", "Proyeksi"), 12), projection)), padding: 14),
            Ui.Card(Ui.Col(12, inView, outView, clView, msView), padding: 14));
        Run();
        return Ui.Columns("*,280", 14, view, side);
    }

    /// <summary>Floor, three boxes, a ball and sparse outliers, with sensor noise.</summary>
    private static PointCloud MakeScene()
    {
        var r = new Random(4);
        var cloud = new PointCloud(40_000);
        float N(double s) => (float)((r.NextDouble() - 0.5) * 2 * s);
        for (var i = 0; i < 16_000; i++)
        {
            cloud.Add(N(1.6), N(1.2), N(0.006));
        }

        void Box(double cx, double cy, double w, double d, double h, int n)
        {
            for (var i = 0; i < n; i++)
            {
                var face = r.Next(5);
                double x = cx + ((r.NextDouble() - 0.5) * w), y = cy + ((r.NextDouble() - 0.5) * d), z = r.NextDouble() * h;
                switch (face)
                {
                    case 0: x = cx - (w / 2); break;
                    case 1: x = cx + (w / 2); break;
                    case 2: y = cy - (d / 2); break;
                    case 3: y = cy + (d / 2); break;
                    default: z = h; break;
                }

                cloud.Add((float)x + N(0.004), (float)y + N(0.004), (float)z + N(0.004));
            }
        }

        Box(-0.8, 0.5, 0.4, 0.4, 0.5, 4000);
        Box(0.6, -0.4, 0.6, 0.3, 0.3, 4000);
        Box(0.3, 0.7, 0.25, 0.25, 0.8, 3500);
        for (var i = 0; i < 3500; i++)
        {
            var th = r.NextDouble() * Math.PI * 2;
            var ph = Math.Acos((2 * r.NextDouble()) - 1);
            cloud.Add((float)(-0.2 + (0.18 * Math.Sin(ph) * Math.Cos(th))), (float)(-0.6 + (0.18 * Math.Sin(ph) * Math.Sin(th))), (float)(0.25 + (0.18 * Math.Cos(ph))));
        }

        for (var i = 0; i < 600; i++)
        {
            cloud.Add(N(1.6), N(1.2), (float)(r.NextDouble() * 1.2));
        }

        return cloud;
    }
}

/// <summary>Reinforcement learning on the simulator: train a Q-table agent, then watch it drive.</summary>
public sealed class RlPage : GalleryPage
{
    public override string Id => "rl";

    public override PageGroup Group => PageGroup.Intelligence;

    public override T2 Title => new("Reinforcement learning", "Reinforcement learning");

    public override T2 Summary => new(
        "RobotGymEnv turns the simulator into an RLNet environment (observation: goal distance and bearing, plus LiDAR sectors). A tabular agent learns to reach the goal in seconds; then the greedy policy drives the robot.",
        "RobotGymEnv mengubah simulator menjadi environment RLNet (observasi: jarak dan arah tujuan, ditambah sektor LiDAR). Agen tabular belajar mencapai tujuan dalam hitungan detik; lalu policy greedy mengemudikan robot.");

    public override IReadOnlyList<string> Interfaces => ["Gravicode.RLNet", "IDiscreteEnvironment", "/cmd_vel"];

    public override string Code => """
        var env = new RobotGymEnv(new RobotGymOptions
        {
            Sectors = 0, ActionCount = 3, Goal = new Vec2(1.2, 0), MaxEpisodeSteps = 40,
        });
        var agent = AgentFactory.CreateQTable(env, bins: 6);
        RlTrainer.Train(env, agent, episodes: 300, onEpisode: s => plot.Add(s.Episode, s.Return));

        // Deploy on a real (or simulated) robot over ROS:
        var runner = new PolicyRunnerNode(context, obs => agent.SelectAction(obs, deterministic: true));
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var p = RosTheme.Current;
        var options = new RobotGymOptions
        {
            Sectors = 0, ActionCount = 3, Start = new ROS2.Net.Simulation.Pose2D(0, 0, 0), RandomizeStartHeading = true, Goal = new Vec2(1.2, 0),
            MaxEpisodeSteps = 40, ControlPeriod = 0.25, Seed = 3,
        };
        var env = new RobotGymEnv(options);
        var agent = AgentFactory.CreateQTable(env, bins: 6, seed: 1);
        var plot = new PlotView { Title = Loc.T("Return per episode", "Return per episode"), WindowSeconds = 300, XUnit = "ep" };
        var view = new SimWorldView { Title = Loc.T("Greedy policy", "Policy greedy"), Goal = (options.Goal.Value.X, options.Goal.Value.Y) };
        view.Attach(env.World);
        var (epView, epText) = Ui.Stat(Loc.T("Episodes", "Episode"), "0");
        var (avgView, avgText) = Ui.Stat(Loc.T("Avg return (last 20)", "Rata-rata return (20 terakhir)"));
        var cts = new CancellationTokenSource();
        OwnAction(cts.Cancel);
        var training = false;
        var episodes = 0;
        var recent = new Queue<float>();

        void Train(int count)
        {
            if (training)
            {
                return;
            }

            training = true;
            _ = Task.Run(() =>
            {
                RlTrainer.Train(env, agent, count, s =>
                {
                    var e = Interlocked.Increment(ref episodes);
                    plot.AddSample("return", e, s.Return);
                    Dispatcher.UIThread.Post(() =>
                    {
                        recent.Enqueue(s.Return);
                        while (recent.Count > 20)
                        {
                            recent.Dequeue();
                        }

                        epText.Text = e.ToString();
                        avgText.Text = recent.Average().ToString("0.0");
                    });
                }, cancellationToken: cts.Token);
                training = false;
                Dispatcher.UIThread.Post(StartDemo);
            }, cts.Token);
        }

        DispatcherTimer? demo = null;
        void StartDemo()
        {
            demo?.Stop();
            env.Reset();
            demo = new DispatcherTimer(TimeSpan.FromMilliseconds(120), DispatcherPriority.Background, (_, _) =>
            {
                if (training)
                {
                    return;
                }

                var result = env.Step(agent.SelectAction(env.Observation, deterministic: true));
                if (result.Done)
                {
                    env.Reset();
                }
            });
            demo.Start();
        }

        OwnAction(() => demo?.Stop());
        var controls = Ui.Card(Ui.Row(18,
            Ui.Button(Loc.T("Train 300 episodes", "Latih 300 episode"), () => Train(300), primary: true),
            Ui.Button(Loc.T("Watch policy", "Lihat policy"), StartDemo),
            epView, avgView), padding: 14);
        Train(300);
        return Ui.Rows("Auto,*", 14, controls, Ui.Columns("*,*", 14, plot, view));
    }
}

/// <summary>Natural-language robot commands (offline rule-based model; swap in any OpenAI-compatible LLM).</summary>
public sealed class CommanderPage : GalleryPage
{
    public override string Id => "commander";

    public override PageGroup Group => PageGroup.Intelligence;

    public override T2 Title => new("Language commander", "Perintah bahasa alami");

    public override T2 Summary => new(
        "Type an instruction in English or Bahasa Indonesia. The commander turns it into tool calls (move, rotate, go to waypoint, stop) behind a safety limiter. This demo uses the offline rule-based model; point it at Ollama or any OpenAI-compatible server for a real LLM.",
        "Ketik perintah dalam bahasa Inggris atau Bahasa Indonesia. Commander mengubahnya menjadi pemanggilan tool (maju, berputar, ke waypoint, berhenti) di balik pembatas keselamatan. Demo ini memakai model rule-based offline; arahkan ke Ollama atau server kompatibel OpenAI untuk LLM sungguhan.");

    public override IReadOnlyList<string> Interfaces => ["std_msgs/msg/String", "/nl_command", "/nl_feedback", "/robot1/cmd_vel"];

    public override string Code => """
        var commander = new NaturalLanguageCommanderNode(context,
            new OpenAICompatibleChatModel(),          // or new RuleBasedChatModel() offline
            new CommanderOptions
            {
                CmdVelTopic = "/robot1/cmd_vel",
                ScanTopic = "/robot1/scan",          // stops before obstacles
                Waypoints = { ["dock"] = (2, 2, 0) },
            });

        say.Publish(new StdMsgs.String { Data = "maju 1 meter lalu belok kiri 90 derajat" });
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var world = Scenarios.Create(ScenarioKind.Empty, robots: 1, seed: 2);
        SimPages.StartSim(runtime, world, d => Own(d), OwnAction);
        Own(new NaturalLanguageCommanderNode(runtime.Context, new RuleBasedChatModel(), new CommanderOptions
        {
            CmdVelTopic = "/robot1/cmd_vel",
            ScanTopic = "/robot1/scan",
            Waypoints = { ["dock"] = (2, 2, 0), ["kitchen"] = (-3, 2, 1.57) },
        }));
        var ui = Own(runtime.Context.CreateNode("gallery_commander"));
        var say = ui.CreatePublisher<StdMsgs.String>("/nl_command");
        var log = new LogConsole { Title = "/nl_command  →  /nl_feedback" };
        Own(RosUi.SubscribeAll<StdMsgs.String>(ui, "/nl_feedback", batch =>
        {
            foreach (var m in batch)
            {
                log.Append(DateTime.Now, RosLog.INFO, "commander", m.Data);
            }
        }));

        var view = new SimWorldView { Title = "/robot1", SelectedRobot = "robot1" };
        view.Attach(world);
        var input = new TextBox { Text = Loc.T("forward 1 m then turn left 90", "maju 1 meter lalu belok kiri 90"), MinWidth = 360 };
        void Send(string text)
        {
            log.Append(DateTime.Now, RosLog.WARN, "you", text);
            say.Publish(new StdMsgs.String { Data = text });
        }

        input.KeyDown += (_, e) =>
        {
            if (e.Key == global::Avalonia.Input.Key.Enter)
            {
                Send(input.Text ?? string.Empty);
            }
        };
        var examples = new WrapPanel();
        foreach (var ex in new[] { "forward 1 m then turn right 90", "rotate 180", "maju 50 cm lalu belok kiri", "pergi ke dock", "stop" })
        {
            var b = Ui.Button(ex, () => Send(ex));
            b.Margin = new Thickness(0, 0, 8, 8);
            examples.Children.Add(b);
        }

        var form = Ui.Card(Ui.Col(10, Ui.Row(10, input, Ui.Button(Loc.T("Send", "Kirim"), () => Send(input.Text ?? string.Empty), primary: true)), examples), padding: 14);
        DispatcherTimer.RunOnce(() => Send(Loc.T("forward 1 m then turn left 90", "maju 1 meter lalu belok kiri 90")), TimeSpan.FromMilliseconds(600));
        return Ui.Rows("Auto,*", 14, form, Ui.Columns("3*,2*", 14, view, log));
    }
}
