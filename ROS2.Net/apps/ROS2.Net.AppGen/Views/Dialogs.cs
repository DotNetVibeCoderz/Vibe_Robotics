using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Ros2AppGen.Ui;

namespace Ros2AppGen.Views;

/// <summary>Base for the app's modal dialogs: themed chrome, title, body, and a button row.</summary>
public class AppDialog : Window
{
    private readonly DockPanel _root = new();
    private readonly StackPanel _buttons = new() { Orientation = Orientation.Horizontal, Spacing = 8, HorizontalAlignment = HorizontalAlignment.Right };

    public AppDialog(string title, string? subtitle = null, double width = 480, double height = 260)
    {
        var p = Look.P;
        Title = title;
        Width = width;
        Height = height;
        CanResize = true;
        WindowStartupLocation = WindowStartupLocation.CenterOwner;
        Background = Look.Brush(p.Surface);
        FontFamily = Look.Sans;

        var head = Kit.Col(3, Kit.Text(title, 17, FontWeight.Bold));
        if (subtitle is not null)
        {
            head.Children.Add(Kit.Muted(subtitle, 12.5, wrap: true));
        }

        head.Margin = new Thickness(22, 18, 22, 12);
        _root.Children.Add(Kit.Dock(new StackPanel { Children = { head, new Border { Height = 2, Background = Look.SignalGradient(), Opacity = 0.7 } } }, Avalonia.Controls.Dock.Top));
        _root.Children.Add(Kit.Dock(new Border
        {
            BorderBrush = Look.Brush(p.Seam),
            BorderThickness = new Thickness(0, 1, 0, 0),
            Background = Look.Brush(p.Background),
            Padding = new Thickness(18, 12),
            Child = _buttons,
        }, Avalonia.Controls.Dock.Bottom));
        Content = _root;
        KeyDown += (_, e) =>
        {
            if (e.Key == Key.Escape)
            {
                Close(null);
            }
        };
    }

    protected void SetBody(Control body, double padding = 20) =>
        _root.Children.Add(new Border { Padding = new Thickness(padding, 14, padding, 14), Child = body });

    protected Button AddButton(string text, Action onClick, bool primary = false)
    {
        var b = Kit.Button(text, onClick, primary);
        b.MinWidth = 88;
        b.HorizontalContentAlignment = HorizontalAlignment.Center;
        _buttons.Children.Add(b);
        return b;
    }
}

/// <summary>Asks for one line of text.</summary>
public sealed class InputDialog : AppDialog
{
    private readonly TextBox _box;

    public InputDialog(string title, string label, string initial = "", string? hint = null)
        : base(title, hint, 440, 220)
    {
        _box = Kit.Input(initial);
        _box.KeyDown += (_, e) =>
        {
            if (e.Key == Key.Enter)
            {
                Close(_box.Text);
            }
        };
        SetBody(Kit.Field(label, _box));
        AddButton("Cancel", () => Close(null));
        AddButton("OK", () => Close(_box.Text), primary: true);
        Opened += (_, _) =>
        {
            _box.Focus();
            _box.SelectAll();
        };
    }

    public static Task<string?> AskAsync(Window owner, string title, string label, string initial = "", string? hint = null) =>
        new InputDialog(title, label, initial, hint).ShowDialog<string?>(owner);
}

/// <summary>Yes/no (or three-way) confirmation.</summary>
public sealed class ConfirmDialog : AppDialog
{
    public ConfirmDialog(string title, string message, string confirm, string? alternative = null)
        : base(title, null, 460, 210)
    {
        SetBody(Kit.Muted(message, 13, wrap: true));
        AddButton("Cancel", () => Close("cancel"));
        if (alternative is not null)
        {
            AddButton(alternative, () => Close("alternative"));
        }

        AddButton(confirm, () => Close("confirm"), primary: true);
    }

    /// <summary>Returns "confirm", "alternative" or "cancel".</summary>
    public static async Task<string> AskAsync(Window owner, string title, string message, string confirm, string? alternative = null) =>
        await new ConfirmDialog(title, message, confirm, alternative).ShowDialog<string?>(owner) ?? "cancel";
}

/// <summary>About box with the credit line.</summary>
public sealed class AboutDialog : AppDialog
{
    public AboutDialog()
        : base("About ROS2.Net App Generator", null, 520, 380)
    {
        var p = Look.P;
        Control logo;
        try
        {
            logo = new Image
            {
                Source = new Avalonia.Media.Imaging.Bitmap(Avalonia.Platform.AssetLoader.Open(new Uri("avares://ROS2AppGen/Assets/icon-128.png"))),
                Width = 64,
                Height = 64,
            };
        }
        catch (Exception)
        {
            logo = new Border { Width = 64, Height = 64, CornerRadius = new CornerRadius(14), Background = Look.SignalGradient() };
        }

        var version = typeof(AboutDialog).Assembly.GetName().Version?.ToString(3) ?? "0.1.0";
        SetBody(Kit.Col(14,
            Kit.Row(16, logo, Kit.Col(2,
                Kit.Text("ROS2AppGen", 20, FontWeight.Bold),
                Kit.Muted($"ROS2.Net App Generator · v{version}", 12.5),
                Kit.Text("with Jack — The Code Bender", 12.5, color: p.Telemetry))),
            Kit.Muted("Describe a robot app and Jack writes it with the ROS2.Net libraries: UI, nodes, simulation and tests — then builds, runs and deploys it.", 12.5, wrap: true),
            Kit.HRule(),
            Kit.Text("Made by Gravicode Studios, led by Kang Fadhil", 13, FontWeight.SemiBold),
            Kit.Muted("Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil", 12.5)));
        AddButton("Close", () => Close(null), primary: true);
    }
}

/// <summary>
/// Shown once per session before the first run on a physical robot: what will be driven, how, and how fast.
/// Closes with true to run.
/// </summary>
public sealed class RobotSafetyDialog : AppDialog
{
    public RobotSafetyDialog(ROS2.Net.AppGen.RunTarget target)
        : base("Run on a physical robot?", "The app will command real hardware, not the simulator.", 560, 460)
    {
        var p = Look.P;
        var amber = Color.Parse("#FFB547");
        string Speed(double v, string unit) => v > 0 ? $"{v:0.##} {unit}" : "app default (no limit set)";
        var link = target.Rmw == "rosbridge" ? $"rosbridge {target.RosbridgeUrl}" : $"{target.Rmw.ToUpperInvariant()} · domain {target.DomainId?.ToString() ?? "from environment"}";

        var rows = new Grid { ColumnDefinitions = new ColumnDefinitions("Auto,*"), ColumnSpacing = 16, RowSpacing = 7 };
        var items = new (string Label, string Value)[]
        {
            ("Target", target.Name),
            ("Connection", link),
            ("Namespace", string.IsNullOrWhiteSpace(target.RobotNamespace) ? "(none) — /cmd_vel, /odom, /scan" : target.RobotNamespace),
            ("Max linear speed", Speed(target.MaxLinearSpeed, "m/s")),
            ("Max angular speed", Speed(target.MaxAngularSpeed, "rad/s")),
        };
        for (var i = 0; i < items.Length; i++)
        {
            rows.RowDefinitions.Add(new RowDefinition(GridLength.Auto));
            rows.Children.Add(Kit.At(Kit.Muted(items[i].Label, 12.5), i, 0));
            rows.Children.Add(Kit.At(Kit.Text(items[i].Value, 12.5, FontWeight.SemiBold, mono: i > 0), i, 1));
        }

        var warning = new Border
        {
            Background = Look.Brush(amber, 0.14),
            BorderBrush = Look.Brush(amber),
            BorderThickness = new Thickness(1),
            CornerRadius = new CornerRadius(8),
            Padding = new Thickness(12, 10),
            Child = new DockPanel
            {
                Children =
                {
                    Kit.Dock(new Border { Margin = new Thickness(0, 0, 10, 0), VerticalAlignment = VerticalAlignment.Top, Child = Kit.Icon(Icons.Shield, 22, amber, 2) }, Avalonia.Controls.Dock.Left),
                    Kit.Col(3,
                        Kit.Text("Make sure the area around the robot is clear", 13, FontWeight.Bold, wrap: true),
                        Kit.Muted("Keep the emergency stop within reach and be ready to press Stop (Shift+F5).", 12.5, wrap: true)),
                },
            },
        };

        SetBody(Kit.Col(16, rows, warning, Kit.Muted("You will not be asked again for this target until the app restarts.", 11.5, wrap: true)));
        AddButton("Cancel", () => Close(false));
        AddButton("Run on robot", () => Close(true), primary: true);
    }
}
