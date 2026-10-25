using System.Diagnostics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Control;
using ROS2.Net.Manipulation;
using ROS2.Net.Navigation;
using ROS2.Net.Navigation.Costmaps;
using ROS2.Net.Navigation.Maps;
using ROS2.Net.Navigation.Planning;
using ROS2.Net.Simulation;
using ROS2.Net.Tf2;
using JointState = ROS2.Net.Msgs.SensorMsgs.JointState;
using NavPose = ROS2.Net.Navigation.Pose2D;
using Odometry = ROS2.Net.Msgs.NavMsgs.Odometry;
using PoseStamped = ROS2.Net.Msgs.GeometryMsgs.PoseStamped;

namespace Ros2NetGallery.Pages;

/// <summary>Helpers shared by the simulator-backed pages.</summary>
internal static class SimPages
{
    /// <summary>Creates and starts a <see cref="SimulationNode"/>, registering its teardown with the page.</summary>
    public static SimulationNode StartSim(GalleryRuntime runtime, SimWorld world, Action<IDisposable> own, Action<Action> ownAction, bool mapToOdom = false)
    {
        var sim = new SimulationNode(runtime.Context, world, new SimulationNodeOptions { PublishMapToOdom = mapToOdom });
        own(sim);
        sim.Start();
        ownAction(() => sim.StopAsync().Wait(TimeSpan.FromSeconds(1)));
        return sim;
    }
}

/// <summary>Teleoperation of a simulated TurtleBot over /cmd_vel.</summary>
public sealed class TeleopPage : GalleryPage
{
    public override string Id => "teleop";

    public override PageGroup Group => PageGroup.Robotics;

    public override T2 Title => new("Teleop a TurtleBot", "Teleop TurtleBot");

    public override T2 Summary => new(
        "Drive a simulated TurtleBot3 in an office. The joystick publishes geometry_msgs/Twist on /robot1/cmd_vel; the simulator answers with odometry, battery and LiDAR exactly like the real robot. Arrow keys or WASD work too.",
        "Kemudikan TurtleBot3 simulasi di kantor. Joystick menerbitkan geometry_msgs/Twist ke /robot1/cmd_vel; simulator membalas dengan odometri, baterai, dan LiDAR persis seperti robot aslinya. Tombol panah atau WASD juga bisa.");

    public override IReadOnlyList<string> Interfaces => ["geometry_msgs/msg/Twist", "nav_msgs/msg/Odometry", "/robot1/cmd_vel", "/robot1/odom"];

    public override string Code => """
        var world = Scenarios.Create(ScenarioKind.Office, robots: 1);
        using var sim = new SimulationNode(context, world);
        sim.Start();   // steps physics in real time, publishes /robot1/odom, /scan, /imu, /tf ...

        joystick.Attach(node, "/robot1/cmd_vel");         // Twist at 10 Hz while deflected
        battery.Attach(node, "/robot1/battery_state");

        var odom = node.CreateSubscription<Odometry>("/robot1/odom");
        await foreach (var o in odom.ReadAllAsync(ct))
            plot.AddSample("v", t, o.Twist.Twist.Linear.X);
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var world = Scenarios.Create(ScenarioKind.Office, robots: 1, seed: 3);
        if (world.Robots[0].Battery is { } b)
        {
            b.DrainFactor = 40;
        }

        SimPages.StartSim(runtime, world, d => Own(d), OwnAction);
        var ui = Own(runtime.Context.CreateNode("gallery_teleop"));

        var view = new SimWorldView { Title = "/robot1 · office", SelectedRobot = "robot1" };
        view.Attach(world);
        var joystick = new TeleopJoystick { Title = "/robot1/cmd_vel", MaxLinear = 0.22, MaxAngular = 2.0 };
        joystick.Attach(ui, "/robot1/cmd_vel");
        var battery = new BatteryGauge();
        battery.Attach(ui, "/robot1/battery_state");
        var plot = new PlotView { Title = "/robot1/odom", WindowSeconds = 12 };
        var clock = Stopwatch.StartNew();
        Own(RosUi.SubscribeLatest<Odometry>(ui, "/robot1/odom", o =>
        {
            var t = clock.Elapsed.TotalSeconds;
            plot.AddSample("v [m/s]", t, o.Twist.Twist.Linear.X);
            plot.AddSample("ω [rad/s]", t, o.Twist.Twist.Angular.Z);
        }));

        // A gentle arc so the page is alive before the user touches anything.
        joystick.SetKnob(-0.25, -0.7);
        DispatcherTimer.RunOnce(() => joystick.SetKnob(0, 0), TimeSpan.FromSeconds(3));

        var side = Ui.Rows("2*,*", 14, joystick, battery);
        side.MinWidth = 260;
        return Ui.Rows("3*,2*", 14, Ui.Columns("*,280", 14, view, side), plot);
    }
}

/// <summary>LiDAR scan and the ground-truth occupancy grid.</summary>
public sealed class LidarPage : GalleryPage
{
    public override string Id => "lidar";

    public override PageGroup Group => PageGroup.Robotics;

    public override T2 Title => new("LiDAR & maps", "LiDAR & peta");

    public override T2 Summary => new(
        "A robot roams the warehouse. Left: sensor_msgs/LaserScan in the robot frame. Right: the latched nav_msgs/OccupancyGrid on /map with the robot's ground-truth pose and the scan projected into the map.",
        "Robot berkeliling gudang. Kiri: sensor_msgs/LaserScan dalam frame robot. Kanan: nav_msgs/OccupancyGrid yang di-latch di /map, dengan pose ground-truth robot dan scan yang diproyeksikan ke peta.");

    public override IReadOnlyList<string> Interfaces => ["sensor_msgs/msg/LaserScan", "nav_msgs/msg/OccupancyGrid", "/robot1/scan", "/map"];

    public override string Code => """
        scanView.Attach(node, "/robot1/scan");           // sensor-data QoS
        mapView.Attach(node, "/map");                     // latched (transient local)

        var pose = node.CreateSubscription<PoseStamped>("/robot1/ground_truth/pose");
        await foreach (var p in pose.ReadAllAsync(ct))
            mapView.Robots = [new MapPose(p.Pose.Position.X, p.Pose.Position.Y, p.Pose.Orientation.Yaw())];
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var world = Scenarios.Create(ScenarioKind.Warehouse, robots: 1, seed: 9);
        Own(new Wanderer(world));
        SimPages.StartSim(runtime, world, d => Own(d), OwnAction);
        var ui = Own(runtime.Context.CreateNode("gallery_lidar"));

        var scan = new LaserScanView { Title = "/robot1/scan" };
        scan.Attach(ui, "/robot1/scan");
        var map = new OccupancyGridView { Title = "/map" };
        map.Attach(ui, "/map");
        Own(RosUi.SubscribeLatest<PoseStamped>(ui, "/robot1/ground_truth/pose", p =>
        {
            var yaw = ROS2.Net.Msgs.MessageExtensions.Yaw(p.Pose.Orientation);
            map.Robots = [new MapPose(p.Pose.Position.X, p.Pose.Position.Y, yaw, Label: "robot1")];
            var hits = world.Robots.Count > 0 ? world.Snapshot().Robots[0].LaserHits : [];
            map.Points = hits.Select(h => (h.X, h.Y)).ToArray();
            map.InvalidateVisual();
        }));

        return Ui.Columns("2*,3*", 14, scan, map);
    }
}

/// <summary>Global path planning on an inflated costmap; click the map to set the goal.</summary>
public sealed class NavigationPage : GalleryPage
{
    public override string Id => "navigation";

    public override PageGroup Group => PageGroup.Robotics;

    public override T2 Title => new("Path planning", "Perencanaan jalur");

    public override T2 Summary => new(
        "Nav2-style global planning on the office map: a static layer plus inflation, then A*, Theta*, Dijkstra or Hybrid-A* (car-like). Click anywhere on the map to plan to that point.",
        "Perencanaan global ala Nav2 di peta kantor: static layer ditambah inflation, lalu A*, Theta*, Dijkstra, atau Hybrid-A* (seperti mobil). Klik di mana saja pada peta untuk merencanakan jalur ke titik itu.");

    public override IReadOnlyList<string> Interfaces => ["nav_msgs/msg/OccupancyGrid", "nav_msgs/msg/Path", "nav2_msgs/action/ComputePathToPose"];

    public override string Code => """
        var map = OccupancyGridMap.FromMsg(world.ToOccupancyGrid(0.05));
        var costmap = Costmap2D.FromMap(map)
            .AddLayer(new StaticLayer(map))
            .AddLayer(new InflationLayer(inflationRadius: 0.45, costScalingFactor: 3.0, inscribedRadius: 0.2));
        costmap.Update();

        IGlobalPlanner planner = new ThetaStarPlanner();
        PlanResult plan = planner.CreatePlan(costmap, start, goal);
        if (plan.Success)
            pathPublisher.Publish(PathUtils.ToMsg(plan.Poses, "map", RosTime.Now()));
        else
            Console.WriteLine(plan.Error);   // "goal is in an obstacle", ...
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var p = RosTheme.Current;
        var world = Scenarios.Create(ScenarioKind.Office, robots: 0, seed: 3);
        var grid = world.ToOccupancyGrid(0.05);
        var map = OccupancyGridMap.FromMsg(grid);
        var costmap = Costmap2D.FromMap(map)
            .AddLayer(new StaticLayer(map))
            .AddLayer(new InflationLayer(0.45, 3.0, 0.2));
        costmap.Update();
        var costMsg = costmap.ToMsg();

        var view = new OccupancyGridView { Title = Loc.T("Costmap · click to set the goal", "Costmap · klik untuk memilih tujuan"), CostmapColors = true, Grid = costMsg };
        var start = FindFree(costmap, world.Bounds.MinX + 1.2, world.Bounds.MinY + 1.2);
        var goal = FindFree(costmap, world.Bounds.MaxX - 1.5, world.Bounds.MaxY - 1.5);
        var planners = new (string Name, IGlobalPlanner Planner)[]
        {
            ("A*", new AStarPlanner()),
            ("Theta*", new ThetaStarPlanner()),
            ("Dijkstra", new DijkstraPlanner()),
            ("Hybrid-A*", new HybridAStarPlanner(Footprint.Circle(0.2))),
        };
        var selected = 1;
        var (lenView, lenText) = Ui.Stat(Loc.T("Path length", "Panjang jalur"));
        var (timeView, timeText) = Ui.Stat(Loc.T("Planning time", "Waktu perencanaan"));
        var (ptsView, ptsText) = Ui.Stat(Loc.T("Poses", "Pose"));
        var error = Ui.Muted(string.Empty, 12);

        void Plan()
        {
            var sw = Stopwatch.StartNew();
            var result = planners[selected].Planner.CreatePlan(costmap, start, goal);
            sw.Stop();
            view.Robots = [new MapPose(start.X, start.Y, start.Theta, p.Signal, "start")];
            view.Goal = new MapPose(goal.X, goal.Y, 0);
            view.PathPoints = result.Success ? result.Poses.Select(x => (x.X, x.Y)).ToArray() : [];
            lenText.Text = result.Success ? $"{result.Length:0.00} m" : "–";
            timeText.Text = $"{sw.Elapsed.TotalMilliseconds:0.0} ms";
            ptsText.Text = result.Success ? result.Poses.Count.ToString() : "–";
            error.Text = result.Success ? string.Empty : result.Error ?? "planning failed";
            error.Foreground = new SolidColorBrush(p.Fault);
            view.InvalidateVisual();
        }

        view.WorldClicked += (x, y) =>
        {
            goal = new NavPose(x, y, 0);
            Plan();
        };

        var picker = new ComboBox { ItemsSource = planners.Select(x => x.Name).ToList(), SelectedIndex = selected, MinWidth = 140 };
        picker.SelectionChanged += (_, _) =>
        {
            selected = Math.Max(0, picker.SelectedIndex);
            Plan();
        };
        var costToggle = new ToggleSwitch { IsChecked = true, OnContent = Loc.T("Costmap", "Costmap"), OffContent = Loc.T("Map", "Peta") };
        costToggle.IsCheckedChanged += (_, _) =>
        {
            view.CostmapColors = costToggle.IsChecked == true;
            view.Grid = view.CostmapColors ? costMsg : grid;
        };

        var side = Ui.Col(18,
            Ui.Card(Ui.Col(10, Ui.Muted(Loc.T("Planner", "Planner"), 12), picker, costToggle), padding: 14),
            Ui.Card(Ui.Col(14, lenView, timeView, ptsView, error), padding: 14),
            Ui.Card(Ui.Muted(Loc.T("Hybrid-A* plans (x, y, θ) with a minimum turning radius, the way Nav2's Smac planner does for car-like robots.", "Hybrid-A* merencanakan (x, y, θ) dengan radius belok minimum, seperti planner Smac di Nav2 untuk robot mirip mobil."), 12), padding: 14));
        Plan();
        return Ui.Columns("*,260", 14, view, side);
    }

    internal static NavPose FindFree(Costmap2D costmap, double x, double y)
    {
        for (var r = 0.0; r < 6; r += 0.1)
        {
            for (var a = 0.0; a < Math.PI * 2; a += 0.35)
            {
                var px = x + (r * Math.Cos(a));
                var py = y + (r * Math.Sin(a));
                if (costmap.WorldToCell(px, py, out var mx, out var my) && costmap.GetCost(mx, my) < 50)
                {
                    return new NavPose(px, py, 0);
                }
            }
        }

        return new NavPose(x, y, 0);
    }
}

/// <summary>The TF tree of a two-robot simulation, with a live lookup.</summary>
public sealed class TfPage : GalleryPage
{
    public override string Id => "tf";

    public override PageGroup Group => PageGroup.Robotics;

    public override T2 Title => new("TF2 frames", "Frame TF2");

    public override T2 Summary => new(
        "Two robots publish odom → base_link on /tf and their sensor mounts on /tf_static. A TfBuffer fed by a TransformListener answers lookups between any two frames, interpolating in time.",
        "Dua robot menerbitkan odom → base_link di /tf dan posisi sensornya di /tf_static. TfBuffer yang diisi TransformListener menjawab lookup antara dua frame mana pun, dengan interpolasi waktu.");

    public override IReadOnlyList<string> Interfaces => ["tf2_msgs/msg/TFMessage", "/tf", "/tf_static"];

    public override string Code => """
        var buffer = new TfBuffer();                       // 10 s of history per frame
        using var listener = new TransformListener(buffer, node);

        // Where is robot2's LiDAR, seen from robot1?
        var t = buffer.LookupTransform("robot1/base_link", "robot2/laser_link", RosTime.Zero);
        Console.WriteLine(buffer.AllFramesAsString());
        Console.WriteLine(buffer.ToMermaid());             // paste into docs
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var world = Scenarios.Create(ScenarioKind.Empty, robots: 2, seed: 4);
        Own(new Wanderer(world));
        SimPages.StartSim(runtime, world, d => Own(d), OwnAction, mapToOdom: true);
        var node = Own(runtime.Context.CreateNode("gallery_tf"));
        var buffer = new TfBuffer();
        Own(new TransformListener(buffer, node));

        var tree = new TfTreeView { Buffer = buffer };
        OwnAction(() => tree.Buffer = null);
        var view = new SimWorldView { Title = Loc.T("Two robots, one map frame", "Dua robot, satu frame map"), ShowLidar = false };
        view.Attach(world);

        var target = new ComboBox { MinWidth = 190 };
        var source = new ComboBox { MinWidth = 190 };
        var result = Ui.Text("–", 13, mono: true);
        var timer = new DispatcherTimer(TimeSpan.FromMilliseconds(300), DispatcherPriority.Background, (_, _) =>
        {
            var frames = buffer.FrameNames.Order(StringComparer.Ordinal).ToList();
            if (frames.Count > 0 && target.ItemCount != frames.Count)
            {
                target.ItemsSource = frames;
                source.ItemsSource = frames;
                target.SelectedItem = frames.FirstOrDefault(f => f == "robot1/base_link") ?? frames[0];
                source.SelectedItem = frames.FirstOrDefault(f => f == "robot2/base_link") ?? frames[^1];
            }

            if (target.SelectedItem is string t && source.SelectedItem is string s)
            {
                result.Text = buffer.TryLookupTransform(t, s, RosTime.Zero, out var tf, out var err)
                    ? $"xyz  ({tf.Translation.X:+0.000;-0.000}, {tf.Translation.Y:+0.000;-0.000}, {tf.Translation.Z:+0.000;-0.000})\nyaw  {tf.Rotation.ToRpy().Yaw * 180 / Math.PI:+0.0;-0.0}°"
                    : err ?? "–";
            }
        });
        timer.Start();
        OwnAction(timer.Stop);

        var lookup = Ui.Card(Ui.Col(10,
            Ui.Row(10, Ui.Muted("target", 12), target, Ui.Muted("← source", 12), source),
            result), Loc.T("lookupTransform", "lookupTransform"), 14);
        return Ui.Columns("*,*", 14, tree, Ui.Rows("*,Auto", 14, view, lookup));
    }
}

/// <summary>ros2_control: controller manager, hardware interfaces, diff-drive controller.</summary>
public sealed class ControlPage : GalleryPage
{
    public override string Id => "control";

    public override PageGroup Group => PageGroup.Robotics;

    public override T2 Title => new("ros2_control", "ros2_control");

    public override T2 Summary => new(
        "A controller manager runs a 100 Hz read → update → write loop on a dedicated thread. The DiffDriveController claims the wheel velocity interfaces, integrates odometry and publishes /joint_states through the broadcaster.",
        "Controller manager menjalankan loop baca → update → tulis 100 Hz di thread khusus. DiffDriveController mengklaim interface kecepatan roda, mengintegrasikan odometri, dan /joint_states diterbitkan lewat broadcaster.");

    public override IReadOnlyList<string> Interfaces => ["controller_manager_msgs/srv/ListControllers", "/diff_drive_controller/cmd_vel", "/joint_states"];

    public override string Code => """
        var resources = new ResourceManager();
        resources.Add(new SimulatedDiffDriveHardware());
        var cm = new ControllerManager(context, resources);   // update_rate 100 Hz

        cm.Spawn(new JointStateBroadcaster(), "joint_state_broadcaster");
        var dd = cm.Spawn(new DiffDriveController(), "diff_drive_controller",
        [
            new Parameter("left_wheel_names",  new[] { "left_wheel_joint" }),
            new Parameter("right_wheel_names", new[] { "right_wheel_joint" }),
            new Parameter("wheel_separation", 0.34),
            new Parameter("wheel_radius", 0.05),
        ]);
        cm.Start();   // high-priority control thread
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var p = RosTheme.Current;
        var resources = new ResourceManager();
        resources.Add(new SimulatedDiffDriveHardware());
        var cm = Own(new ControllerManager(runtime.Context, resources));
        cm.Spawn(new JointStateBroadcaster(), "joint_state_broadcaster");
        var dd = cm.Spawn(new DiffDriveController(), "diff_drive_controller",
        [
            new Parameter("left_wheel_names", new[] { "left_wheel_joint" }),
            new Parameter("right_wheel_names", new[] { "right_wheel_joint" }),
            new Parameter("wheel_separation", 0.34),
            new Parameter("wheel_radius", 0.05),
        ]);
        cm.Start();
        OwnAction(cm.Stop);

        var ui = Own(runtime.Context.CreateNode("gallery_control"));
        var joystick = new TeleopJoystick { Title = "/diff_drive_controller/cmd_vel", MaxLinear = 0.6, MaxAngular = 2.0 };
        joystick.Attach(ui, "/diff_drive_controller/cmd_vel");
        var joints = new JointStatePanel { Title = "/joint_states" };
        joints.Limits["left_wheel_joint"] = (-Math.PI, Math.PI);
        joints.Limits["right_wheel_joint"] = (-Math.PI, Math.PI);
        Own(RosUi.SubscribeLatest<JointState>(ui, "/joint_states", s =>
        {
            // Wrap continuous wheel angles for display.
            var copy = s.Clone();
            copy.Position = copy.Position.Select(x => Math.IEEERemainder(x, 2 * Math.PI)).ToArray();
            joints.State = copy;
        }));

        var track = new OccupancyGridView { Title = Loc.T("Odometry (odom frame)", "Odometri (frame odom)") };
        var trail = new List<(double, double)>();
        var plot = new PlotView { Title = Loc.T("Wheel velocity [rad/s]", "Kecepatan roda [rad/s]"), WindowSeconds = 10 };
        var clock = Stopwatch.StartNew();
        var timer = new DispatcherTimer(TimeSpan.FromMilliseconds(50), DispatcherPriority.Background, (_, _) =>
        {
            var o = dd.Odometry;
            if (trail.Count == 0 || Math.Abs(trail[^1].Item1 - o.X) + Math.Abs(trail[^1].Item2 - o.Y) > 0.01)
            {
                trail.Add((o.X, o.Y));
                if (trail.Count > 2000)
                {
                    trail.RemoveAt(0);
                }
            }

            track.Robots = [new MapPose(o.X, o.Y, o.Heading, p.Signal, "base_link")];
            track.PathPoints = trail.ToArray();
            track.InvalidateVisual();
        });
        timer.Start();
        OwnAction(timer.Stop);
        Own(RosUi.SubscribeAll<JointState>(ui, "/joint_states", batch =>
        {
            var t = clock.Elapsed.TotalSeconds;
            var last = batch[^1];
            for (var i = 0; i < last.Name.Length && i < last.Velocity.Length; i++)
            {
                plot.AddSample(last.Name[i].Replace("_wheel_joint", string.Empty), t, last.Velocity[i]);
            }
        }));

        joystick.SetKnob(0.3, -0.8);
        DispatcherTimer.RunOnce(() => joystick.SetKnob(0, 0), TimeSpan.FromSeconds(4));
        var info = Ui.Card(Ui.Col(6,
            Ui.Text(Loc.T("Active controllers", "Controller aktif"), 13, FontWeight.SemiBold),
            Ui.Text("joint_state_broadcaster  ·  active", 12, mono: true, color: p.Ok),
            Ui.Text("diff_drive_controller    ·  active", 12, mono: true, color: p.Ok),
            Ui.Muted(Loc.T("Claimed: left_wheel_joint/velocity, right_wheel_joint/velocity", "Diklaim: left_wheel_joint/velocity, right_wheel_joint/velocity"), 11.5)), padding: 14);
        return Ui.Columns("*,320", 14, Ui.Rows("3*,2*", 14, track, plot), Ui.Rows("*,Auto,Auto", 14, joystick, joints, info));
    }
}

/// <summary>Inverse kinematics on a UR5 loaded from URDF.</summary>
public sealed class ArmPage : GalleryPage
{
    public override string Id => "arm";

    public override PageGroup Group => PageGroup.Robotics;

    public override T2 Title => new("Arm kinematics", "Kinematika lengan");

    public override T2 Summary => new(
        "A UR5 parsed from URDF. Move the target and Levenberg–Marquardt IK solves the six joint angles under their limits, warm-started from the previous solution. Drag the 3D view to orbit.",
        "UR5 yang di-parse dari URDF. Geser target dan IK Levenberg–Marquardt menghitung enam sudut sendi dalam batasnya, dimulai dari solusi sebelumnya. Seret tampilan 3D untuk memutar.");

    public override IReadOnlyList<string> Interfaces => ["URDF", "sensor_msgs/msg/JointState", "control_msgs/action/FollowJointTrajectory"];

    public override string Code => """
        var model = UrdfModel.LoadBuiltIn("ur5");
        var arm = new KinematicChain(model, "base_link", "tool0");
        var ik = new IkSolver(arm);

        // Tool pointing down, 40 cm in front, 30 cm up:
        var target = Pose3.FromXyzRpy(0.4, 0.1, 0.3, Math.PI, 0, 0);
        var result = ik.Solve(target, seed: current);
        if (result.Success)
            await new MoveGroup(node, arm).MoveToJointsAsync(result.Solution);
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var model = UrdfModel.LoadBuiltIn("ur5");
        var chain = new KinematicChain(model, "base_link", "tool0");
        var ik = new IkSolver(chain, new IkOptions { Restarts = 8, MaxIterations = 150 });
        double[] q = [0, -1.4, 1.6, -1.8, -1.57, 0];
        double tx = 0.4, ty = 0.15, tz = 0.35;
        var view = new ArmView(chain);
        var joints = new JointStatePanel { Title = Loc.T("Joint solution", "Solusi sendi") };
        foreach (var j in chain.Joints.Where(j => j.IsMovable))
        {
            joints.Limits[j.Name] = (double.IsFinite(j.Lower) ? j.Lower : -Math.PI, double.IsFinite(j.Upper) ? j.Upper : Math.PI);
        }

        var (errView, errText) = Ui.Stat(Loc.T("Position error", "Galat posisi"));
        var (itView, itText) = Ui.Stat(Loc.T("Iterations", "Iterasi"));
        var (msView, msText) = Ui.Stat(Loc.T("Solve time", "Waktu solusi"));

        void Solve()
        {
            var sw = Stopwatch.StartNew();
            var r = ik.Solve(Pose3.FromXyzRpy(tx, ty, tz, Math.PI, 0, 0), q);
            sw.Stop();
            if (r.Success)
            {
                q = r.Solution;
            }

            errText.Text = r.Success ? $"{r.PositionError * 1000:0.000} mm" : Loc.T("unreachable", "tak terjangkau");
            errText.Foreground = new SolidColorBrush(r.Success ? RosTheme.Current.Ink : RosTheme.Current.Fault);
            itText.Text = r.Iterations.ToString();
            msText.Text = $"{sw.Elapsed.TotalMilliseconds:0.0} ms";
            view.Joints = q;
            view.Target = new Vec3(tx, ty, tz);
            joints.State = new JointState { Name = chain.JointNames, Position = q.ToArray() };
        }

        var sliders = Ui.Card(Ui.Col(12,
            Ui.Slider("x", -0.6, 0.7, tx, v => { tx = v; Solve(); }, "0.00", " m"),
            Ui.Slider("y", -0.6, 0.6, ty, v => { ty = v; Solve(); }, "0.00", " m"),
            Ui.Slider("z", 0.05, 0.8, tz, v => { tz = v; Solve(); }, "0.00", " m"),
            Ui.Row(22, errView, itView, msView)), Loc.T("Tool target (tool pointing down)", "Target alat (menghadap ke bawah)"), 14);
        Solve();
        return Ui.Columns("*,360", 14, view, Ui.Rows("Auto,*", 14, sliders, joints));
    }

    /// <summary>Orbit view of the chain's link frames, with the target and a floor grid.</summary>
    private sealed class ArmView : RosDrawnControl
    {
        private readonly KinematicChain _chain;
        private double _yaw = -0.9;
        private double _pitch = 0.45;
        private Point? _drag;
        private double[] _joints = [];

        public ArmView(KinematicChain chain)
        {
            _chain = chain;
            Title = "ur5 · base_link → tool0";
            Cursor = new Cursor(StandardCursorType.SizeAll);
        }

        public double[] Joints
        {
            get => _joints;
            set
            {
                _joints = value;
                InvalidateVisual();
            }
        }

        public Vec3 Target { get; set; }

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
                _yaw += (p.X - d.X) * 0.01;
                _pitch = Math.Clamp(_pitch + ((p.Y - d.Y) * 0.01), -0.2, 1.4);
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

        private Point Project(Vec3 v, Rect b, double scale)
        {
            var cy = Math.Cos(_yaw);
            var sy = Math.Sin(_yaw);
            var x = (v.X * cy) - (v.Y * sy);
            var y = (v.X * sy) + (v.Y * cy);
            var sx = x;
            var syy = (v.Z * Math.Cos(_pitch)) - (y * Math.Sin(_pitch));
            return new Point(b.Center.X + (sx * scale), b.Center.Y + (b.Height * 0.22) - (syy * scale));
        }

        protected override void RenderContent(DrawingContext context, Rect bounds)
        {
            var scale = Math.Min(bounds.Width, bounds.Height) * 0.85;
            var grid = P.Pen(P.Grid, 1);
            for (var i = -5; i <= 5; i++)
            {
                context.DrawLine(grid, Project(new Vec3(i * 0.1, -0.5, 0), bounds, scale), Project(new Vec3(i * 0.1, 0.5, 0), bounds, scale));
                context.DrawLine(grid, Project(new Vec3(-0.5, i * 0.1, 0), bounds, scale), Project(new Vec3(0.5, i * 0.1, 0), bounds, scale));
            }

            // Axes at the base.
            var o = Project(new Vec3(0, 0, 0), bounds, scale);
            context.DrawLine(P.Pen(P.Fault, 2), o, Project(new Vec3(0.12, 0, 0), bounds, scale));
            context.DrawLine(P.Pen(P.Ok, 2), o, Project(new Vec3(0, 0.12, 0), bounds, scale));
            context.DrawLine(P.Pen(P.Telemetry, 2), o, Project(new Vec3(0, 0, 0.12), bounds, scale));

            if (_joints.Length == _chain.Dof)
            {
                var poses = _chain.LinkPoses(_joints);
                var points = new List<Point> { o };
                points.AddRange(poses.Select(p => Project(p.Position, bounds, scale)));

                // Shadow on the floor.
                var shadow = new Pen(new SolidColorBrush(P.Ink, 0.12), 10, lineCap: PenLineCap.Round);
                var prevS = o;
                foreach (var pose in poses)
                {
                    var s = Project(pose.Position with { Z = 0 }, bounds, scale);
                    context.DrawLine(shadow, prevS, s);
                    prevS = s;
                }

                var link = new Pen(P.SignalGradient(), 11, lineCap: PenLineCap.Round, lineJoin: PenLineJoin.Round);
                for (var i = 1; i < points.Count; i++)
                {
                    context.DrawLine(link, points[i - 1], points[i]);
                }

                foreach (var pt in points)
                {
                    context.DrawEllipse(P.Brush(P.Surface), P.Pen(P.Ink, 2, 0.8), pt, 6, 6);
                }

                var tip = poses[^1];
                context.DrawEllipse(P.Brush(P.Telemetry), null, points[^1], 5, 5);
                var dir = tip.Rotation.Rotate(new Vec3(0, 0, 0.08));
                context.DrawLine(P.Pen(P.Telemetry, 2), points[^1], Project(tip.Position + dir, bounds, scale));
            }

            var t = Project(Target, bounds, scale);
            context.DrawEllipse(P.Brush(P.Alert, 0.25), P.Pen(P.Alert, 2), t, 11, 11);
            context.DrawLine(P.Pen(P.Alert, 1, 0.5), t, Project(Target with { Z = 0 }, bounds, scale));
            DrawPill(context, $"target ({Target.X:0.00}, {Target.Y:0.00}, {Target.Z:0.00})", new Point(bounds.Right - 10, 8), P.Alert);
        }
    }
}
