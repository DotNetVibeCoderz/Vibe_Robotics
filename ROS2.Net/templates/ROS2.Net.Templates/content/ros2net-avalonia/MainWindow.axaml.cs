using System.Collections.ObjectModel;
using Avalonia.Controls;
using Avalonia.Interactivity;
using Avalonia.Threading;

namespace Ros2NetAvalonia;

public partial class MainWindow : Window
{
    private const double Speed = 0.5;
    private const double Turn = 1.0;
    private readonly RobotLink? _robot;
    private readonly ObservableCollection<string> _chatter = [];

    /// <summary>Design-time constructor.</summary>
    public MainWindow()
    {
        InitializeComponent();
        ChatterList.ItemsSource = _chatter;
    }

    public MainWindow(RobotLink robot)
        : this()
    {
        _robot = robot;
        robot.ChatterReceived += text => Dispatcher.UIThread.Post(() =>
        {
            _chatter.Insert(0, $"{DateTime.Now:HH:mm:ss}  {text}");
            while (_chatter.Count > 200)
            {
                _chatter.RemoveAt(_chatter.Count - 1);
            }
        });
        robot.PoseReceived += p => Dispatcher.UIThread.Post(() =>
            PoseText.Text = $"x {p.X:0.00}  y {p.Y:0.00}  θ {p.Yaw * 180 / Math.PI:0}°");
    }

    private void OnForward(object? sender, RoutedEventArgs e) => _robot?.Drive(Speed, 0);

    private void OnBack(object? sender, RoutedEventArgs e) => _robot?.Drive(-Speed, 0);

    private void OnLeft(object? sender, RoutedEventArgs e) => _robot?.Drive(0, Turn);

    private void OnRight(object? sender, RoutedEventArgs e) => _robot?.Drive(0, -Turn);

    private void OnStop(object? sender, RoutedEventArgs e) => _robot?.Drive(0, 0);
}
