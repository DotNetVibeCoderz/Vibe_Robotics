using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// Edits the parameters of any node through its parameter services (like <c>rqt_reconfigure</c>).
/// Each parameter shows its type and description; press Enter (or toggle a switch) to set it.
/// Rejections from the node's validation are shown inline.
/// </summary>
public sealed class ParameterEditor : UserControl
{
    private readonly StackPanel _rows = new() { Spacing = 6 };
    private readonly TextBlock _status = new() { FontSize = 11.5, TextWrapping = TextWrapping.Wrap };
    private ParameterClient? _client;

    public ParameterEditor()
    {
        var refresh = new Button { Content = "Reload", Padding = new Thickness(12, 4) };
        refresh.Click += async (_, _) => await LoadAsync();
        var header = new DockPanel { Margin = new Thickness(0, 0, 0, 8) };
        DockPanel.SetDock(refresh, Dock.Right);
        header.Children.Add(refresh);
        header.Children.Add(new TextBlock { Text = "PARAMETERS", FontSize = 10.5, FontWeight = FontWeight.SemiBold, VerticalAlignment = VerticalAlignment.Center, Foreground = RosTheme.Current.Brush(RosTheme.Current.Haze) });
        Content = new Border
        {
            Padding = new Thickness(14),
            CornerRadius = new CornerRadius(10),
            Background = RosTheme.Current.Brush(RosTheme.Current.Surface),
            BorderBrush = RosTheme.Current.Brush(RosTheme.Current.Seam),
            BorderThickness = new Thickness(1),
            Child = new StackPanel { Children = { header, new ScrollViewer { Content = _rows, MaxHeight = 520 }, _status } },
        };
    }

    public string? RemoteNode => _client?.RemoteNode;

    public int ParameterCount => _rows.Children.Count;

    /// <summary>Connects to <paramref name="remoteNode"/> (fully qualified) using <paramref name="node"/> for the calls.</summary>
    public async Task AttachAsync(Node node, string remoteNode)
    {
        _client?.Dispose();
        _client = new ParameterClient(node, remoteNode) { Timeout = TimeSpan.FromSeconds(3) };
        await LoadAsync();
    }

    public async Task LoadAsync()
    {
        if (_client is null)
        {
            return;
        }

        try
        {
            if (!await _client.WaitForServiceAsync(TimeSpan.FromSeconds(3)))
            {
                Status($"{_client.RemoteNode} does not answer parameter requests.", RosTheme.Current.Fault);
                return;
            }

            var names = await _client.ListAsync();
            var values = await _client.GetAsync(names);
            var descriptors = await _client.DescribeAsync(names);
            _rows.Children.Clear();
            for (var i = 0; i < values.Count; i++)
            {
                _rows.Children.Add(Row(values[i], i < descriptors.Count ? descriptors[i] : new ParameterDescriptor()));
            }

            Status($"{values.Count} parameters on {_client.RemoteNode}", RosTheme.Current.Haze);
        }
        catch (Exception ex) when (ex is TimeoutException or OperationCanceledException or InvalidOperationException)
        {
            Status(ex.Message, RosTheme.Current.Fault);
        }
    }

    private Control Row(Parameter p, ParameterDescriptor d)
    {
        var palette = RosTheme.Current;
        var name = new TextBlock { Text = p.Name, FontFamily = RosTheme.MonoFamily, FontSize = 12.5, VerticalAlignment = VerticalAlignment.Center, Foreground = palette.Brush(palette.Ink) };
        var type = new TextBlock { Text = p.Value.Type.ToString().ToLowerInvariant() + (d.ReadOnly ? " · read-only" : string.Empty), FontSize = 10.5, Foreground = palette.Brush(palette.Haze) };
        Control editor;
        if (p.Value.Type == ParameterType.Bool)
        {
            var toggle = new ToggleSwitch { IsChecked = p.Value.AsBool(), IsEnabled = !d.ReadOnly, OnContent = "true", OffContent = "false" };
            toggle.IsCheckedChanged += async (_, _) => await SetAsync(p.Name, toggle.IsChecked == true, null);
            editor = toggle;
        }
        else
        {
            var box = new TextBox { Text = p.Value.ToString(), FontFamily = RosTheme.MonoFamily, IsReadOnly = d.ReadOnly, MinWidth = 160 };
            box.KeyDown += async (_, e) =>
            {
                if (e.Key == Key.Enter)
                {
                    var parsed = ParameterValue.Parse(box.Text ?? string.Empty);
                    await SetAsync(p.Name, parsed, box);
                }
            };
            editor = box;
        }

        var left = new StackPanel { Spacing = 2, Children = { name, type } };
        if (d.Description.Length > 0)
        {
            left.Children.Add(new TextBlock { Text = d.Description, FontSize = 11, TextWrapping = TextWrapping.Wrap, Foreground = palette.Brush(palette.Haze) });
        }

        var grid = new Grid { ColumnDefinitions = new ColumnDefinitions("*,Auto"), Margin = new Thickness(0, 4) };
        grid.Children.Add(left);
        Grid.SetColumn(editor, 1);
        editor.VerticalAlignment = VerticalAlignment.Center;
        grid.Children.Add(editor);
        return grid;
    }

    private async Task SetAsync(string name, object value, TextBox? box)
    {
        if (_client is null)
        {
            return;
        }

        try
        {
            var r = await _client.SetAsync(name, value);
            Status(r.Successful ? $"Set {name}." : $"Rejected {name}: {r.Reason}", r.Successful ? RosTheme.Current.Ok : RosTheme.Current.Fault);
            if (!r.Successful && box is not null)
            {
                box.Text = (await _client.GetAsync(name)).ToString();
            }
        }
        catch (Exception ex) when (ex is TimeoutException or OperationCanceledException)
        {
            Status(ex.Message, RosTheme.Current.Fault);
        }
    }

    private void Status(string text, Color color)
    {
        _status.Text = text;
        _status.Foreground = new SolidColorBrush(color);
        _status.Margin = new Thickness(0, 8, 0, 0);
    }
}
