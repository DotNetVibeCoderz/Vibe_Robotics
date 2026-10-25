using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Simulation;

namespace Ros2NetGallery.Pages;

/// <summary>Landing page: a live robot fleet as the hero, then what the library contains.</summary>
public sealed class HomePage : GalleryPage
{
    public override string Id => "home";

    public override PageGroup Group => PageGroup.Overview;

    public override T2 Title => new("Welcome", "Selamat datang");

    public override T2 Summary => new(
        "Every demo in this gallery runs a real ROS 2 graph inside this window — nodes, topics, services, actions — on ROS2.Net's in-process middleware. No ROS installation needed.",
        "Setiap demo di galeri ini menjalankan graf ROS 2 sungguhan di dalam jendela ini — node, topic, service, action — di atas middleware in-process ROS2.Net. Tidak perlu memasang ROS.");

    public override IReadOnlyList<string> Interfaces => ["net10.0", "rclcpp-style API", "DDS-RTPS", "rosbridge"];

    public override Control Build(GalleryRuntime runtime)
    {
        var p = RosTheme.Current;
        var world = Scenarios.Create(ScenarioKind.Warehouse, robots: 4, seed: 5);
        Own(new Wanderer(world));
        world.StepFor(25); // let the fleet spread out before the first frame
        Own(new WorldRunner(world));
        var view = new SimWorldView { Title = Loc.T("Live · warehouse fleet", "Langsung · armada gudang"), ShowTrails = true };
        view.Attach(world);
        OwnAction(view.Detach);

        var headline = Ui.Col(4,
            new TextBlock { Text = "ROS 2,", FontSize = 54, FontWeight = FontWeight.Bold, LetterSpacing = -1.5, Foreground = new SolidColorBrush(p.Ink) },
            new TextBlock { Text = Loc.T("native in .NET.", "asli di .NET."), FontSize = 54, FontWeight = FontWeight.Bold, LetterSpacing = -1.5, Foreground = p.SignalGradient() });

        var lead = new TextBlock
        {
            Text = Loc.T(
                "A pure C# implementation of the ROS 2 client stack: typed messages generated from .msg files, executors, QoS, actions, lifecycle nodes, and middlewares that talk DDS-RTPS or rosbridge. Then TF2, rosbag2, navigation, control, manipulation, perception, simulation and AI on top.",
                "Implementasi stack client ROS 2 dalam C# murni: tipe pesan yang di-generate dari file .msg, executor, QoS, action, lifecycle node, serta middleware yang berbicara DDS-RTPS atau rosbridge. Di atasnya ada TF2, rosbag2, navigasi, kontrol, manipulasi, persepsi, simulasi, dan AI."),
            FontSize = 15,
            LineHeight = 23,
            TextWrapping = TextWrapping.Wrap,
            Foreground = new SolidColorBrush(p.Haze),
            MaxWidth = 520,
            HorizontalAlignment = HorizontalAlignment.Left,
        };

        var install = Ui.Card(Ui.Text("dotnet add package ROS2.Net", 13, mono: true, color: p.Telemetry, wrap: false), padding: 12);
        install.HorizontalAlignment = HorizontalAlignment.Left;

        var left = Ui.Col(18, headline, lead, install, Ui.Row(10,
            Ui.Button(Loc.T("Drive a robot", "Kemudikan robot"), () => GalleryNavigation.Go("teleop"), primary: true),
            Ui.Button(Loc.T("Plan a path", "Rencanakan jalur"), () => GalleryNavigation.Go("navigation"))));
        left.VerticalAlignment = VerticalAlignment.Center;

        var layers = Ui.Columns("*,*,*,*", 14,
            Layer(Loc.Communication, ["Nodes · executors", "Topics · QoS", "Services · actions", "Parameters · lifecycle"]),
            Layer(Loc.Robotics, ["TF2 · URDF", "Nav2-style navigation", "ros2_control", "MoveIt-style IK"]),
            Layer(Loc.Intelligence, ["Point clouds (Rust)", "SLAM · AMCL · EKF", "RL on the simulator", "LLM commander"]),
            Layer(Loc.Tools, ["rosbag2 (MCAP)", "Robot simulator", "ros2net CLI", "dotnet new templates"]));

        var credit = Ui.Card(Ui.Row(12,
            new Border { Width = 4, Height = 34, CornerRadius = new CornerRadius(2), Background = p.SignalGradient(new RelativePoint(0, 0, RelativeUnit.Relative), new RelativePoint(0, 1, RelativeUnit.Relative)) },
            Ui.Col(2, Ui.Text(Loc.Credit, 13.5, FontWeight.SemiBold), Ui.Muted("github.com/DotNetVibeCoderz · MIT", 11.5))), padding: 14);

        var hero = Ui.Columns("5*,6*", 28, left, view);
        hero.MinHeight = 380;
        return Ui.Rows("*,Auto,Auto", 18, hero, layers, credit);
    }

    private static Control Layer(string title, string[] items)
    {
        var p = RosTheme.Current;
        var list = Ui.Col(6, items.Select(i => (Control)Ui.Row(8,
            new Border { Width = 5, Height = 5, CornerRadius = new CornerRadius(3), Background = new SolidColorBrush(p.Telemetry) },
            Ui.Text(i, 12.5, wrap: false))).ToArray());
        return Ui.Card(list, title, 14);
    }
}

/// <summary>Lets pages ask the shell to open another page.</summary>
public static class GalleryNavigation
{
    public static Action<string>? Navigate { get; set; }

    public static void Go(string id) => Navigate?.Invoke(id);
}
