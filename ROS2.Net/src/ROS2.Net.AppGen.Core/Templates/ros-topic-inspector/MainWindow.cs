using Avalonia;
using Avalonia.Controls;
using Avalonia.Media;
using ROS2.Net;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;

namespace {{Namespace}};

/// <summary>Topics, graph and log of whatever ROS 2 system is on the network.</summary>
public sealed class MainWindow : Window
{
    public MainWindow(Ros2Context context)
    {
        Title = $"{{ProjectName}} — ROS 2 inspector (domain {context.DomainId}, {context.Rmw.Identifier})";
        Width = 1280;
        Height = 800;
        var p = RosTheme.Current;
        Background = new SolidColorBrush(p.Background);

        var node = context.CreateNode("inspector");
        var topics = new TopicMonitor { MeasureRates = true };
        topics.Attach(node);
        var graph = new NodeGraphView();
        graph.Attach(node);
        var log = new LogConsole();
        log.Attach(node);

        var tabs = new TabControl
        {
            Margin = new Thickness(12),
            ItemsSource = new[]
            {
                new TabItem { Header = "Topics", Content = topics },
                new TabItem { Header = "Graph", Content = graph },
                new TabItem { Header = "Log", Content = log },
            },
        };
        Content = tabs;
    }
}
