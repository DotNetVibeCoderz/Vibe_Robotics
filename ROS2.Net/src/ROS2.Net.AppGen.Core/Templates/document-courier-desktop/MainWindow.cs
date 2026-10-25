using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using ROS2.Net;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.Nav2Msgs;
using ROS2.Net.Navigation.Control;
using ROS2.Net.Navigation.Nodes;
using ROS2.Net.Simulation;
using NavPose = ROS2.Net.Navigation.Pose2D;

namespace {{Namespace}};

/// <summary>Courier dispatch window: room buttons send NavigateToPose goals.</summary>
public sealed class MainWindow : Window
{
    private readonly ActionClient<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback> _nav;
    private readonly TextBlock _status;

    public MainWindow(Ros2Context context)
    {
        Title = "{{ProjectName}} — document courier";
        Width = 1280;
        Height = 760;
        var p = RosTheme.Current;
        Background = new SolidColorBrush(p.Background);

        // Simulator: the office scenario with ground-truth poses. Physical robot: odometry poses and the robot's /map.
        var sim = AppTarget.StartSimulator(context, ScenarioKind.Office, robots: 1, new SimulationNodeOptions { PublishMapToOdom = true });
        var node = context.CreateNode("courier_ui");
        IRobotPoseProvider pose = AppTarget.World is { } w
            ? new DelegatePoseProvider(() =>
            {
                var r = w.Robots[0];
                return (new NavPose(r.Pose.X, r.Pose.Y, r.Pose.Theta), new Velocity2D(r.LinearVelocity, r.AngularVelocity));
            })
            : new OdometryPoseProvider(node, AppTarget.Topic("odom"));
        _ = new PlannerServerNode(context, pose);
        _ = new ControllerServerNode(context, pose, options: new NodeOptions().WithRemap("/cmd_vel", AppTarget.Topic("cmd_vel")));
        _ = new BtNavigatorNode(context, pose);
        _nav = node.CreateActionClient<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback>("/navigate_to_pose");

        Control view;
        if (AppTarget.World is { } world)
        {
            var worldView = new SimWorldView { ShowLidar = true, ShowTrails = true };
            worldView.Attach(world);
            view = worldView;
        }
        else
        {
            var map = new OccupancyGridView();
            map.Attach(node, "/map", "/plan");
            view = map;
        }

        _status = new TextBlock { Text = AppTarget.Describe() + " — pick a destination.", Foreground = new SolidColorBrush(p.Haze), TextWrapping = TextWrapping.Wrap };

        // Destinations: simulator rooms, or edit these map coordinates for your building.
        var places = AppTarget.World?.Waypoints.Select(x => (x.Key, x.Value.X, x.Value.Y, x.Value.Theta)).ToList()
            ?? [("reception", 1.0, 1.0, 0.0), ("meeting_room", 5.0, 3.0, 1.57)];
        var buttons = new StackPanel { Spacing = 8 };
        foreach (var (name, x, y, theta) in places)
        {
            var b = new Button { Content = name.Replace('_', ' '), HorizontalAlignment = HorizontalAlignment.Stretch };
            b.Click += async (_, _) => await SendAsync(name, x, y, theta);
            buttons.Children.Add(b);
        }

        var side = new StackPanel { Width = 280, Spacing = 14, Margin = new Thickness(16) };
        side.Children.Add(new TextBlock { Text = "Document courier", FontSize = 22, FontWeight = FontWeight.Bold, Foreground = new SolidColorBrush(p.Ink) });
        side.Children.Add(buttons);
        side.Children.Add(_status);

        var root = new DockPanel();
        DockPanel.SetDock(side, Dock.Right);
        root.Children.Add(side);
        root.Children.Add(view);
        Content = root;
        Closed += (_, _) => sim?.Dispose();
    }

    private async Task SendAsync(string name, double x, double y, double yaw)
    {
        _status.Text = $"Heading to {name}…";
        var goal = new NavigateToPoseGoal();
        goal.Pose.Header.FrameId = "map";
        goal.Pose.Pose.Position.X = x;
        goal.Pose.Pose.Position.Y = y;
        goal.Pose.Pose.Orientation = MessageExtensions.QuaternionFromYaw(yaw);
        try
        {
            await _nav.WaitForServerAsync(TimeSpan.FromSeconds(5));
            var result = await _nav.SendGoalAndWaitAsync(goal, fb => RosUi.OnUi(() => _status.Text = $"{name}: {fb.DistanceRemaining:0.0} m remaining"));
            _status.Text = $"{name}: {result.Status}";
        }
        catch (Exception ex)
        {
            _status.Text = "Navigation failed: " + ex.Message;
        }
    }
}
