using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Documents;
using Avalonia.Layout;
using Avalonia.Media;
using ROS2.Net.Avalonia;

namespace Ros2NetGallery;

/// <summary>Small builders for the gallery's visual language (all colours come from <see cref="RosTheme"/>).</summary>
public static class Ui
{
    private static RosPalette P => RosTheme.Current;

    public static TextBlock Text(string text, double size = 13, FontWeight weight = FontWeight.Normal, Color? color = null, bool mono = false, bool wrap = true) => new()
    {
        Text = text,
        FontSize = size,
        FontWeight = weight,
        FontFamily = mono ? RosTheme.MonoFamily : RosTheme.SansFamily,
        Foreground = new SolidColorBrush(color ?? P.Ink),
        TextWrapping = wrap ? TextWrapping.Wrap : TextWrapping.NoWrap,
    };

    public static TextBlock Muted(string text, double size = 12.5) => Text(text, size, color: P.Haze);

    /// <summary>Small uppercase label above a group of controls.</summary>
    public static TextBlock Eyebrow(string text) => new()
    {
        Text = text.ToUpperInvariant(),
        FontSize = 10.5,
        FontWeight = FontWeight.SemiBold,
        LetterSpacing = 1.2,
        Foreground = new SolidColorBrush(P.Haze),
        Margin = new Thickness(0, 0, 0, 6),
    };

    /// <summary>A surface with a hairline border.</summary>
    public static Border Card(Control child, string? title = null, double padding = 16)
    {
        Control content = child;
        if (title is not null)
        {
            content = new DockPanel { Children = { Dock(Eyebrow(title), global::Avalonia.Controls.Dock.Top), child } };
        }

        return new Border
        {
            Background = new SolidColorBrush(P.Surface),
            BorderBrush = new SolidColorBrush(P.Seam),
            BorderThickness = new Thickness(1),
            CornerRadius = new CornerRadius(12),
            Padding = new Thickness(padding),
            Child = content,
        };
    }

    public static T Dock<T>(T control, Dock dock)
        where T : Control
    {
        DockPanel.SetDock(control, dock);
        return control;
    }

    /// <summary>A monospace chip naming a ROS interface or topic.</summary>
    public static Border Chip(string text, Color? accent = null)
    {
        var a = accent ?? P.Telemetry;
        return new Border
        {
            Background = new SolidColorBrush(a, 0.13),
            BorderBrush = new SolidColorBrush(a, 0.5),
            BorderThickness = new Thickness(1),
            CornerRadius = new CornerRadius(20),
            Padding = new Thickness(9, 3),
            VerticalAlignment = VerticalAlignment.Center,
            HorizontalAlignment = HorizontalAlignment.Left,
            Child = Text(text, 11, mono: true, wrap: false),
        };
    }

    public static Button Button(string text, Action onClick, bool primary = false)
    {
        var b = new Button
        {
            Content = new TextBlock { Text = text, FontWeight = FontWeight.SemiBold, FontSize = 12.5, Foreground = primary ? Brushes.White : new SolidColorBrush(P.Ink) },
            Padding = new Thickness(14, 7),
            CornerRadius = new CornerRadius(8),
            Background = primary ? P.SignalGradient() : new SolidColorBrush(P.SurfaceRaised),
            BorderBrush = new SolidColorBrush(primary ? P.Signal : P.Seam),
            BorderThickness = new Thickness(1),
        };
        b.Click += (_, _) => onClick();
        return b;
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

    /// <summary>Grid with star columns, e.g. <c>Columns("2*,*", a, b)</c>.</summary>
    public static Grid Columns(string definition, double gap, params Control[] children)
    {
        var g = new Grid { ColumnDefinitions = new ColumnDefinitions(definition) };
        for (var i = 0; i < children.Length; i++)
        {
            Grid.SetColumn(children[i], i);
            if (i > 0)
            {
                children[i].Margin = new Thickness(children[i].Margin.Left + gap, children[i].Margin.Top, children[i].Margin.Right, children[i].Margin.Bottom);
            }

            g.Children.Add(children[i]);
        }

        return g;
    }

    /// <summary>Grid with star rows.</summary>
    public static Grid Rows(string definition, double gap, params Control[] children)
    {
        var g = new Grid { RowDefinitions = new RowDefinitions(definition) };
        for (var i = 0; i < children.Length; i++)
        {
            Grid.SetRow(children[i], i);
            if (i > 0)
            {
                children[i].Margin = new Thickness(children[i].Margin.Left, children[i].Margin.Top + gap, children[i].Margin.Right, children[i].Margin.Bottom);
            }

            g.Children.Add(children[i]);
        }

        return g;
    }

    /// <summary>A labeled slider showing its value.</summary>
    public static Control Slider(string label, double min, double max, double value, Action<double> changed, string format = "0.00", string unit = "")
    {
        var valueText = Text(value.ToString(format) + unit, 12, mono: true, color: P.Telemetry, wrap: false);
        var slider = new Slider { Minimum = min, Maximum = max, Value = value, MinWidth = 140 };
        slider.ValueChanged += (_, e) =>
        {
            valueText.Text = e.NewValue.ToString(format) + unit;
            changed(e.NewValue);
        };
        var header = new DockPanel { Children = { Dock(valueText, global::Avalonia.Controls.Dock.Right), Muted(label, 12) } };
        return new StackPanel { Spacing = 0, Children = { header, slider } };
    }

    /// <summary>A key/value readout.</summary>
    public static (Control View, TextBlock Value) Stat(string label, string initial = "–")
    {
        var value = Text(initial, 20, FontWeight.Bold, wrap: false);
        return (new StackPanel { Spacing = 2, Children = { Muted(label, 11.5), value } }, value);
    }

    private static readonly HashSet<string> s_keywords =
    [
        "var", "new", "await", "using", "async", "return", "public", "private", "class", "sealed", "static", "void", "if", "else", "for", "foreach",
        "in", "while", "true", "false", "null", "override", "protected", "readonly", "string", "double", "int", "long", "bool", "float", "base", "this", "throw",
    ];

    /// <summary>A read-only code view with light C# highlighting.</summary>
    public static Control Code(string code)
    {
        var tb = new SelectableTextBlock
        {
            FontFamily = RosTheme.MonoFamily,
            FontSize = 12,
            LineHeight = 18,
            TextWrapping = TextWrapping.NoWrap,
            Foreground = new SolidColorBrush(P.Ink),
        };
        foreach (var (text, kind) in Tokenize(code.Replace("\r\n", "\n").Trim('\n')))
        {
            var color = kind switch
            {
                TokenKind.Keyword => P.Signal,
                TokenKind.String => P.Telemetry,
                TokenKind.Comment => P.Haze,
                TokenKind.Number => P.Alert,
                TokenKind.Type => P.IsDark ? Color.Parse("#B9A6FF") : Color.Parse("#5A3FC0"),
                _ => P.Ink,
            };
            tb.Inlines!.Add(new Run(text) { Foreground = new SolidColorBrush(color), FontStyle = kind == TokenKind.Comment ? FontStyle.Italic : FontStyle.Normal });
        }

        return new ScrollViewer
        {
            HorizontalScrollBarVisibility = global::Avalonia.Controls.Primitives.ScrollBarVisibility.Auto,
            VerticalScrollBarVisibility = global::Avalonia.Controls.Primitives.ScrollBarVisibility.Auto,
            Content = tb,
        };
    }

    private enum TokenKind
    {
        Plain,
        Keyword,
        String,
        Comment,
        Number,
        Type,
    }

    private static IEnumerable<(string, TokenKind)> Tokenize(string code)
    {
        var i = 0;
        var plain = new System.Text.StringBuilder();
        IEnumerable<(string, TokenKind)> Flush()
        {
            if (plain.Length > 0)
            {
                var s = plain.ToString();
                plain.Clear();
                yield return (s, TokenKind.Plain);
            }
        }

        while (i < code.Length)
        {
            var c = code[i];
            if (c == '/' && i + 1 < code.Length && code[i + 1] == '/')
            {
                foreach (var f in Flush())
                {
                    yield return f;
                }

                var end = code.IndexOf('\n', i);
                end = end < 0 ? code.Length : end;
                yield return (code[i..end], TokenKind.Comment);
                i = end;
            }
            else if (c == '"' || (c == '$' && i + 1 < code.Length && code[i + 1] == '"'))
            {
                foreach (var f in Flush())
                {
                    yield return f;
                }

                var start = i;
                i += c == '$' ? 2 : 1;
                while (i < code.Length && code[i] != '"')
                {
                    i += code[i] == '\\' ? 2 : 1;
                }

                i = Math.Min(code.Length, i + 1);
                yield return (code[start..i], TokenKind.String);
            }
            else if (char.IsLetter(c) || c == '_')
            {
                var start = i;
                while (i < code.Length && (char.IsLetterOrDigit(code[i]) || code[i] == '_'))
                {
                    i++;
                }

                var word = code[start..i];
                var kind = s_keywords.Contains(word) ? TokenKind.Keyword
                    : char.IsUpper(word[0]) && (start == 0 || code[start - 1] != '.') ? TokenKind.Type
                    : TokenKind.Plain;
                if (kind == TokenKind.Plain)
                {
                    plain.Append(word);
                }
                else
                {
                    foreach (var f in Flush())
                    {
                        yield return f;
                    }

                    yield return (word, kind);
                }
            }
            else if (char.IsDigit(c))
            {
                foreach (var f in Flush())
                {
                    yield return f;
                }

                var start = i;
                while (i < code.Length && (char.IsLetterOrDigit(code[i]) || code[i] == '.'))
                {
                    i++;
                }

                yield return (code[start..i], TokenKind.Number);
            }
            else
            {
                plain.Append(c);
                i++;
            }
        }

        foreach (var f in Flush())
        {
            yield return f;
        }
    }

    /// <summary>A thin violet→cyan rule: the gallery's "signal trace".</summary>
    public static Control SignalTrace(double height = 2) => new Border
    {
        Height = height,
        Background = P.SignalGradient(),
        CornerRadius = new CornerRadius(height),
        HorizontalAlignment = HorizontalAlignment.Stretch,
    };
}
