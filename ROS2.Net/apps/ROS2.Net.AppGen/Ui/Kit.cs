using Avalonia;
using Avalonia.Controls;
using Shape = Avalonia.Controls.Shapes.Path;
using Avalonia.Layout;
using Avalonia.Media;

namespace Ros2AppGen.Ui;

/// <summary>Vector icons (24×24 path data) used across the app.</summary>
public static class Icons
{
    public const string NewProject = "M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z M14 2v6h6 M12 18v-6 M9 15h6";
    public const string Open = "M3 7a2 2 0 0 1 2-2h4l2 2h8a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z";
    public const string Save = "M5 3h11l5 5v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2z M7 3v6h8V3 M7 21v-7h10v7";
    public const string Close = "M6 6l12 12 M18 6L6 18";
    public const string Build = "M14.7 6.3a4 4 0 0 0-5.4 5.4L3 18l3 3 6.3-6.3a4 4 0 0 0 5.4-5.4l-2.6 2.6-2.4-.6-.6-2.4z";
    public const string Run = "M7 4l13 8-13 8z";
    public const string Stop = "M6 6h12v12H6z";
    public const string Deploy = "M12 3l4 4h-3v8h-2V7H8z M4 15v4a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2v-4";
    public const string Format = "M4 6h16 M4 10h10 M4 14h16 M4 18h10";
    public const string GoTo = "M4 12h12 M12 6l6 6-6 6";
    public const string Settings = "M12 15a3 3 0 1 0 0-6 3 3 0 0 0 0 6z M19.4 15a1.7 1.7 0 0 0 .3 1.8l.1.1a2 2 0 1 1-2.8 2.8l-.1-.1a1.7 1.7 0 0 0-1.8-.3 1.7 1.7 0 0 0-1 1.5V21a2 2 0 1 1-4 0v-.1a1.7 1.7 0 0 0-1.1-1.5 1.7 1.7 0 0 0-1.8.3l-.1.1a2 2 0 1 1-2.8-2.8l.1-.1a1.7 1.7 0 0 0 .3-1.8 1.7 1.7 0 0 0-1.5-1H3a2 2 0 1 1 0-4h.1a1.7 1.7 0 0 0 1.5-1.1 1.7 1.7 0 0 0-.3-1.8l-.1-.1a2 2 0 1 1 2.8-2.8l.1.1a1.7 1.7 0 0 0 1.8.3H9a1.7 1.7 0 0 0 1-1.5V3a2 2 0 1 1 4 0v.1a1.7 1.7 0 0 0 1 1.5 1.7 1.7 0 0 0 1.8-.3l.1-.1a2 2 0 1 1 2.8 2.8l-.1.1a1.7 1.7 0 0 0-.3 1.8V9a1.7 1.7 0 0 0 1.5 1H21a2 2 0 1 1 0 4h-.1a1.7 1.7 0 0 0-1.5 1z";
    public const string Chat = "M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z";
    public const string Send = "M22 2L11 13 M22 2l-7 20-4-9-9-4z";
    public const string Attach = "M21.4 11.1l-9.2 9.2a6 6 0 0 1-8.5-8.5l9.2-9.2a4 4 0 0 1 5.7 5.7l-9.2 9.2a2 2 0 0 1-2.8-2.8l8.5-8.5";
    public const string Trash = "M3 6h18 M8 6V4h8v2 M19 6l-1 14a2 2 0 0 1-2 2H8a2 2 0 0 1-2-2L5 6";
    public const string Folder = "M3 7a2 2 0 0 1 2-2h4l2 2h8a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z";
    public const string File = "M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8z M14 2v6h6";
    public const string Copy = "M9 9h11v11H9z M5 15H4V4h11v1";
    public const string Insert = "M12 5v14 M5 12h14";
    public const string Logs = "M4 5h16 M4 10h16 M4 15h10 M4 20h7";
    public const string Explorer = "M3 4h7v7H3z M14 4h7v7h-7z M3 15h7v5H3z M14 15h7v5h-7z";
    public const string Sun = "M12 17a5 5 0 1 0 0-10 5 5 0 0 0 0 10z M12 1v2 M12 21v2 M4.2 4.2l1.4 1.4 M18.4 18.4l1.4 1.4 M1 12h2 M21 12h2 M4.2 19.8l1.4-1.4 M18.4 5.6l1.4-1.4";
    public const string Moon = "M21 12.8A9 9 0 1 1 11.2 3a7 7 0 0 0 9.8 9.8z";
    public const string Refresh = "M23 4v6h-6 M1 20v-6h6 M3.5 9a9 9 0 0 1 14.9-3.4L23 10 M1 14l4.6 4.4A9 9 0 0 0 20.5 15";
    public const string Search = "M11 19a8 8 0 1 0 0-16 8 8 0 0 0 0 16z M21 21l-4.3-4.3";
    public const string Spark = "M12 2l2.4 7.6L22 12l-7.6 2.4L12 22l-2.4-7.6L2 12l7.6-2.4z";
    public const string Bot = "M12 2v3 M5 8h14a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-8a2 2 0 0 1 2-2z M9 13h.01 M15 13h.01 M9 17h6";
    public const string Image = "M3 3h18v18H3z M8.5 10a1.5 1.5 0 1 0 0-3 1.5 1.5 0 0 0 0 3z M21 15l-5-5L5 21";
    public const string Chevron = "M9 6l6 6-6 6";
    public const string ChevronDown = "M6 9l6 6 6-6";
    public const string Wrap = "M3 6h18 M3 12h15a3 3 0 0 1 0 6h-4 M16 16l-2 2 2 2 M3 18h7";
    public const string Hash = "M4 9h16 M4 15h16 M10 3L8 21 M16 3l-2 18";
    public const string Info = "M12 22a10 10 0 1 0 0-20 10 10 0 0 0 0 20z M12 16v-4 M12 8h.01";
    public const string Check = "M20 6L9 17l-5-5";
    public const string Plus = "M12 5v14 M5 12h14";
    public const string Robot = "M12 2v3 M7 5h10a3 3 0 0 1 3 3v6a3 3 0 0 1-3 3H7a3 3 0 0 1-3-3V8a3 3 0 0 1 3-3z M9 10h.01 M15 10h.01 M8 21l2-4 M16 21l-2-4";
    public const string Sim = "M3 4h18v12H3z M8 20h8 M12 16v4 M7 12l3-3 2 2 4-4";
    public const string Shield = "M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z";
    public const string Exit = "M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4 M16 17l5-5-5-5 M21 12H9";
}

/// <summary>Small building blocks for the code-only UI.</summary>
public static class Kit
{
    private static Palette P => Look.P;

    public static TextBlock Text(string text, double size = 13, FontWeight weight = FontWeight.Normal, Color? color = null, bool mono = false, bool wrap = false) => new()
    {
        Text = text,
        FontSize = size,
        FontWeight = weight,
        FontFamily = mono ? Look.Mono : Look.Sans,
        Foreground = Look.Brush(color ?? P.Ink),
        TextWrapping = wrap ? TextWrapping.Wrap : TextWrapping.NoWrap,
        VerticalAlignment = VerticalAlignment.Center,
    };

    public static TextBlock Muted(string text, double size = 12, bool wrap = false) => Text(text, size, color: P.Haze, wrap: wrap);

    /// <summary>Uppercase micro label, used for panel and section headings.</summary>
    public static TextBlock Eyebrow(string text) => new()
    {
        Text = text.ToUpperInvariant(),
        FontSize = 10.5,
        FontWeight = FontWeight.SemiBold,
        LetterSpacing = 1.1,
        FontFamily = Look.Sans,
        Foreground = Look.Brush(P.Haze),
        VerticalAlignment = VerticalAlignment.Center,
    };

    public static Control Icon(string data, double size = 16, Color? color = null, double stroke = 1.8)
    {
        var path = new Shape
        {
            Data = Geometry.Parse(data),
            Stroke = Look.Brush(color ?? P.Haze),
            StrokeThickness = stroke,
            StrokeLineCap = PenLineCap.Round,
            StrokeJoin = PenLineJoin.Round,
            Stretch = Stretch.Uniform,
        };
        return new Viewbox { Width = size, Height = size, Child = new Canvas { Width = 24, Height = 24, Children = { path } } , VerticalAlignment = VerticalAlignment.Center };
    }

    /// <summary>A filled icon (for play/stop).</summary>
    public static Control SolidIcon(string data, double size, Color color) =>
        new Viewbox
        {
            Width = size,
            Height = size,
            VerticalAlignment = VerticalAlignment.Center,
            Child = new Canvas { Width = 24, Height = 24, Children = { new Shape { Data = Geometry.Parse(data), Fill = Look.Brush(color) } } },
        };

    public static Button Button(string text, Action onClick, bool primary = false, string? icon = null, string? tip = null)
    {
        var content = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 7 };
        if (icon is not null)
        {
            content.Children.Add(Icon(icon, 14, primary ? Colors.White : P.Ink));
        }

        content.Children.Add(new TextBlock
        {
            Text = text,
            FontSize = 12.5,
            FontWeight = FontWeight.SemiBold,
            FontFamily = Look.Sans,
            Foreground = primary ? Brushes.White : Look.Brush(P.Ink),
            VerticalAlignment = VerticalAlignment.Center,
        });
        var b = new Button
        {
            Content = content,
            Padding = new Thickness(13, 7),
            CornerRadius = new CornerRadius(7),
            Background = primary ? Look.SignalGradient() : Look.Brush(P.Raised),
            BorderBrush = Look.Brush(primary ? P.Signal : P.Seam),
            BorderThickness = new Thickness(1),
            VerticalAlignment = VerticalAlignment.Center,
        };
        if (tip is not null)
        {
            ToolTip.SetTip(b, tip);
        }

        b.Click += (_, _) => onClick();
        return b;
    }

    /// <summary>A square icon-only button with a tooltip.</summary>
    public static Button IconButton(string icon, string tip, Action onClick, Color? color = null, double size = 15, bool solid = false)
    {
        var b = new Button
        {
            Content = solid ? SolidIcon(icon, size - 2, color ?? P.Ink) : Icon(icon, size, color ?? P.Haze),
            Padding = new Thickness(7),
            CornerRadius = new CornerRadius(6),
            Background = Brushes.Transparent,
            BorderThickness = new Thickness(0),
            VerticalAlignment = VerticalAlignment.Center,
        };
        ToolTip.SetTip(b, tip);
        b.Click += (_, _) => onClick();
        return b;
    }

    public static Border Card(Control child, double padding = 14, Color? background = null) => new()
    {
        Background = Look.Brush(background ?? P.Surface),
        BorderBrush = Look.Brush(P.Seam),
        BorderThickness = new Thickness(1),
        CornerRadius = new CornerRadius(10),
        Padding = new Thickness(padding),
        Child = child,
    };

    /// <summary>A monospace pill, used for tool calls, file kinds and model ids.</summary>
    public static Border Chip(string text, Color accent, string? icon = null)
    {
        var row = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 5 };
        if (icon is not null)
        {
            row.Children.Add(Icon(icon, 11, accent, 2.2));
        }

        row.Children.Add(Text(text, 11, mono: true, color: P.Ink));
        return new Border
        {
            Background = Look.Brush(accent, 0.12),
            BorderBrush = Look.Brush(accent, 0.45),
            BorderThickness = new Thickness(1),
            CornerRadius = new CornerRadius(20),
            Padding = new Thickness(8, 2),
            HorizontalAlignment = HorizontalAlignment.Left,
            Child = row,
        };
    }

    public static StackPanel Row(double spacing, params Control[] children)
    {
        var s = new StackPanel { Orientation = Orientation.Horizontal, Spacing = spacing };
        foreach (var c in children)
        {
            c.VerticalAlignment = VerticalAlignment.Center;
            s.Children.Add(c);
        }

        return s;
    }

    public static StackPanel Col(double spacing, params Control[] children)
    {
        var s = new StackPanel { Spacing = spacing };
        s.Children.AddRange(children);
        return s;
    }

    /// <summary>A labeled form field.</summary>
    public static Control Field(string label, Control input, string? hint = null)
    {
        var col = new StackPanel { Spacing = 5 };
        col.Children.Add(Text(label, 12, FontWeight.SemiBold, P.Haze));
        if (!double.IsNaN(input.Width))
        {
            input.HorizontalAlignment = HorizontalAlignment.Left;
        }

        col.Children.Add(input);
        if (hint is not null)
        {
            col.Children.Add(Muted(hint, 11, wrap: true));
        }

        return col;
    }

    public static TextBox Input(string text = "", string? watermark = null, bool password = false, bool mono = false) => new()
    {
        Text = text,
        PlaceholderText = watermark,
        PasswordChar = password ? '•' : default,
        FontFamily = mono ? Look.Mono : Look.Sans,
        FontSize = 12.5,
        CornerRadius = new CornerRadius(6),
    };

    /// <summary>Removes a control from its parent so it can be placed again.</summary>
    public static T Detach<T>(T control)
        where T : Control
    {
        switch (control.Parent)
        {
            case Panel panel:
                panel.Children.Remove(control);
                break;
            case Decorator decorator:
                decorator.Child = null;
                break;
            case ContentControl cc:
                cc.Content = null;
                break;
        }

        return control;
    }

    public static Border HRule() => new() { Height = 1, Background = Look.Brush(P.Seam), Margin = new Thickness(0, 4) };

    public static T Dock<T>(T control, Avalonia.Controls.Dock dock)
        where T : Control
    {
        DockPanel.SetDock(control, dock);
        return control;
    }

    public static T At<T>(T control, int row, int column, int columnSpan = 1)
        where T : Control
    {
        Grid.SetRow(control, row);
        Grid.SetColumn(control, column);
        if (columnSpan > 1)
        {
            Grid.SetColumnSpan(control, columnSpan);
        }

        return control;
    }

    /// <summary>Color for a file by extension (explorer icons, tabs).</summary>
    public static Color FileColor(string path) => System.IO.Path.GetExtension(path).ToLowerInvariant() switch
    {
        ".cs" => Color.Parse("#9B7BFF"),
        ".axaml" or ".xaml" or ".razor" or ".html" => Color.Parse("#FF8A5C"),
        ".csproj" or ".props" or ".targets" or ".slnx" or ".sln" => Color.Parse("#B98CFF"),
        ".json" or ".yaml" or ".yml" or ".config" => Color.Parse("#FFC857"),
        ".md" => Color.Parse("#5CC8FF"),
        ".msg" or ".srv" or ".action" => Color.Parse("#00D1FF"),
        ".css" => Color.Parse("#4FA3FF"),
        ".js" or ".ts" => Color.Parse("#F7DF1E"),
        ".png" or ".jpg" or ".svg" or ".ico" => Color.Parse("#3DDC97"),
        _ => Look.P.Haze,
    };
}
