using System.Text.RegularExpressions;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Documents;
using Avalonia.Layout;
using Avalonia.Media;
using Ros2AppGen.Ui;

namespace Ros2AppGen.Views;

/// <summary>
/// Renders the subset of Markdown LLMs actually emit: fenced code blocks (with copy / insert actions),
/// headings, bullet and numbered lists, block quotes, bold, italics and inline code.
/// </summary>
public static partial class MarkdownView
{
    /// <summary>Called with (language, code) when the user presses "Insert" on a code block.</summary>
    public static Action<string, string>? InsertCode { get; set; }

    /// <summary>Called with the text to copy (the UI wires it to the clipboard).</summary>
    public static Action<string>? CopyText { get; set; }

    public static Control Render(string markdown, Color? ink = null)
    {
        var p = Look.P;
        var panel = new StackPanel { Spacing = 7 };
        var lines = markdown.Replace("\r\n", "\n").Split('\n');
        var paragraph = new List<string>();

        void FlushParagraph()
        {
            if (paragraph.Count > 0)
            {
                panel.Children.Add(Inline(string.Join(" ", paragraph), 13, FontWeight.Normal, ink ?? p.Ink));
                paragraph.Clear();
            }
        }

        for (var i = 0; i < lines.Length; i++)
        {
            var line = lines[i];
            var trimmed = line.TrimStart();
            if (trimmed.StartsWith("```", StringComparison.Ordinal))
            {
                FlushParagraph();
                var lang = trimmed[3..].Trim();
                var code = new List<string>();
                i++;
                while (i < lines.Length && !lines[i].TrimStart().StartsWith("```", StringComparison.Ordinal))
                {
                    code.Add(lines[i]);
                    i++;
                }

                panel.Children.Add(CodeBlock(lang, string.Join("\n", code)));
                continue;
            }

            if (trimmed.Length == 0)
            {
                FlushParagraph();
                continue;
            }

            var heading = HeadingRegex().Match(trimmed);
            if (heading.Success)
            {
                FlushParagraph();
                var level = heading.Groups[1].Value.Length;
                var size = level switch { 1 => 17, 2 => 15, _ => 13.5 };
                var h = Inline(heading.Groups[2].Value, size, FontWeight.Bold, ink ?? p.Ink);
                h.Margin = new Thickness(0, 4, 0, 0);
                panel.Children.Add(h);
                continue;
            }

            var bullet = BulletRegex().Match(line);
            if (bullet.Success)
            {
                FlushParagraph();
                var indent = Math.Min(3, bullet.Groups[1].Value.Length / 2);
                var marker = bullet.Groups[2].Value;
                var isNumber = char.IsDigit(marker[0]);
                var row = new DockPanel { Margin = new Thickness(indent * 14, 0, 0, 0) };
                var dot = new TextBlock
                {
                    Text = isNumber ? marker : "•",
                    Width = isNumber ? 22 : 14,
                    FontSize = 13,
                    Foreground = Look.Brush(p.Telemetry),
                    FontFamily = isNumber ? Look.Mono : Look.Sans,
                };
                DockPanel.SetDock(dot, Avalonia.Controls.Dock.Left);
                row.Children.Add(dot);
                row.Children.Add(Inline(bullet.Groups[3].Value, 13, FontWeight.Normal, ink ?? p.Ink));
                panel.Children.Add(row);
                continue;
            }

            if (trimmed.StartsWith('>'))
            {
                FlushParagraph();
                panel.Children.Add(new Border
                {
                    BorderBrush = Look.Brush(p.Signal),
                    BorderThickness = new Thickness(3, 0, 0, 0),
                    Padding = new Thickness(10, 2),
                    Child = Inline(trimmed.TrimStart('>', ' '), 13, FontWeight.Normal, p.Haze),
                });
                continue;
            }

            if (trimmed is "---" or "***")
            {
                FlushParagraph();
                panel.Children.Add(Kit.HRule());
                continue;
            }

            paragraph.Add(trimmed);
        }

        FlushParagraph();
        return panel;
    }

    /// <summary>A paragraph with **bold**, *italic* and `code` spans.</summary>
    public static SelectableTextBlock Inline(string text, double size, FontWeight weight, Color color)
    {
        var p = Look.P;
        var block = new SelectableTextBlock
        {
            FontSize = size,
            FontWeight = weight,
            FontFamily = Look.Sans,
            Foreground = Look.Brush(color),
            TextWrapping = TextWrapping.Wrap,
            LineHeight = size * 1.45,
        };
        var inlines = block.Inlines ??= [];
        foreach (Match m in SpanRegex().Matches(text))
        {
            if (m.Groups["code"].Success)
            {
                inlines.Add(new Run(m.Groups["code"].Value)
                {
                    FontFamily = Look.Mono,
                    FontSize = size - 1,
                    Foreground = Look.Brush(p.Telemetry),
                    Background = Look.Brush(p.Telemetry, 0.10),
                });
            }
            else if (m.Groups["bold"].Success)
            {
                // Bold may contain `code`: split it so the code keeps its own styling.
                foreach (var part in BoldPartsRegex().Split(m.Groups["bold"].Value))
                {
                    if (part.Length == 0)
                    {
                        continue;
                    }

                    inlines.Add(part.Length > 1 && part[0] == '`' && part[^1] == '`'
                        ? new Run(part[1..^1]) { FontFamily = Look.Mono, FontSize = size - 1, FontWeight = FontWeight.Bold, Foreground = Look.Brush(p.Telemetry), Background = Look.Brush(p.Telemetry, 0.10) }
                        : new Run(part) { FontWeight = FontWeight.Bold });
                }
            }
            else if (m.Groups["italic"].Success)
            {
                inlines.Add(new Run(m.Groups["italic"].Value) { FontStyle = FontStyle.Italic });
            }
            else
            {
                inlines.Add(new Run(m.Value));
            }
        }

        return block;
    }

    private static Control CodeBlock(string language, string code)
    {
        var p = Look.P;
        var header = new DockPanel { Margin = new Thickness(10, 5, 4, 5) };
        var actions = Kit.Row(2,
            Kit.IconButton(Icons.Copy, "Copy code", () => CopyText?.Invoke(code), size: 13),
            Kit.IconButton(Icons.Insert, "Insert into the editor at the cursor", () => InsertCode?.Invoke(language, code), size: 13));
        header.Children.Add(Kit.Dock(actions, Avalonia.Controls.Dock.Right));
        header.Children.Add(Kit.Text(language.Length == 0 ? "code" : language, 11, color: p.Haze, mono: true));

        var body = new SelectableTextBlock
        {
            Text = code,
            FontFamily = Look.Mono,
            FontSize = 12,
            Foreground = Look.Brush(p.Ink),
            TextWrapping = TextWrapping.NoWrap,
            Margin = new Thickness(12, 2, 12, 10),
        };
        return new Border
        {
            Background = Look.Brush(p.Sunken),
            BorderBrush = Look.Brush(p.Seam),
            BorderThickness = new Thickness(1),
            CornerRadius = new CornerRadius(8),
            Child = new DockPanel
            {
                Children =
                {
                    Kit.Dock(new Border { BorderBrush = Look.Brush(p.Seam), BorderThickness = new Thickness(0, 0, 0, 1), Child = header }, Avalonia.Controls.Dock.Top),
                    new ScrollViewer { HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Auto, VerticalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Disabled, Content = body },
                },
            },
        };
    }

    [GeneratedRegex(@"(`[^`]+`)")]
    private static partial Regex BoldPartsRegex();

    [GeneratedRegex(@"^(#{1,6})\s+(.*)$")]
    private static partial Regex HeadingRegex();

    [GeneratedRegex(@"^(\s*)([-*+]|\d+[.)])\s+(.*)$")]
    private static partial Regex BulletRegex();

    [GeneratedRegex(@"\*\*(?<bold>[^*]+)\*\*|`(?<code>[^`]+)`|(?<![\w*])\*(?<italic>[^*\s][^*]*)\*(?!\w)|(?<!\w)_(?<italic>[^_\s][^_]*)_(?!\w)|[^`*_]+|[`*_]")]
    private static partial Regex SpanRegex();
}
