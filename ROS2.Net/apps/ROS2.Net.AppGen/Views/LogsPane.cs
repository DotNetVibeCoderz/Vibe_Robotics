using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using AvaloniaEdit;
using Ros2AppGen.Ui;

namespace Ros2AppGen.Views;

/// <summary>The bottom panel: build/run output and Jack's tool log, each a read-only editor (handles large logs cheaply).</summary>
public sealed class LogsPane : DockPanel
{
    private readonly TextEditor _output;
    private readonly TextEditor _jack;
    private readonly StackPanel _tabBar = new() { Orientation = Orientation.Horizontal, Spacing = 2 };
    private readonly ContentControl _body = new();
    private string _active = "output";

    public LogsPane(Action hide)
    {
        var p = Look.P;
        Background = Look.Brush(p.Surface);
        _output = MakeLog();
        _jack = MakeLog();

        var header = new DockPanel { Margin = new Thickness(8, 4, 6, 0) };
        header.Children.Add(Kit.Dock(Kit.Row(0,
            Kit.IconButton(Icons.Trash, "Clear this log", Clear, size: 13),
            Kit.IconButton(Icons.Close, "Hide panel (Ctrl+J)", hide, size: 13)), Avalonia.Controls.Dock.Right));
        header.Children.Add(_tabBar);
        Children.Add(Kit.Dock(new Border { BorderBrush = Look.Brush(p.Seam), BorderThickness = new Thickness(0, 1, 0, 1), Child = header }, Avalonia.Controls.Dock.Top));
        Children.Add(_body);
        Select("output");
    }

    public string OutputText => _output.Text;

    /// <summary>Re-applies palette colors after a theme switch.</summary>
    public void ApplyTheme()
    {
        var p = Look.P;
        Background = Look.Brush(p.Surface);
        foreach (var e in new[] { _output, _jack })
        {
            e.Background = Look.Brush(p.Sunken);
            e.Foreground = Look.Brush(p.Haze);
        }

        if (Children.Count > 0 && Children[0] is Border header)
        {
            header.BorderBrush = Look.Brush(p.Seam);
        }

        Select(_active);
    }

    public string JackText => _jack.Text;

    public void Select(string tab)
    {
        _active = tab;
        _tabBar.Children.Clear();
        _tabBar.Children.Add(Tab("output", "Output"));
        _tabBar.Children.Add(Tab("jack", "Jack · tools"));
        _body.Content = tab == "jack" ? _jack : _output;
    }

    /// <summary>Appends a line to the output log (thread-safe).</summary>
    public void WriteOutput(string line) => Append(_output, line);

    /// <summary>Appends a line to Jack's tool log (thread-safe).</summary>
    public void WriteJack(string line) => Append(_jack, line);

    public void Clear()
    {
        (_active == "jack" ? _jack : _output).Text = string.Empty;
    }

    private static void Append(TextEditor editor, string line)
    {
        void Do()
        {
            var stamped = $"[{DateTime.Now:HH:mm:ss}] {line}\n";
            editor.Document.Insert(editor.Document.TextLength, stamped);

            // Cap memory: keep the last ~200k characters.
            if (editor.Document.TextLength > 250_000)
            {
                editor.Document.Remove(0, editor.Document.TextLength - 200_000);
            }

            editor.ScrollToEnd();
        }

        if (Dispatcher.UIThread.CheckAccess())
        {
            Do();
        }
        else
        {
            Dispatcher.UIThread.Post(Do);
        }
    }

    private static TextEditor MakeLog()
    {
        var p = Look.P;
        return new TextEditor
        {
            IsReadOnly = true,
            FontFamily = Look.Mono,
            FontSize = 12,
            Background = Look.Brush(p.Sunken),
            Foreground = Look.Brush(p.Haze),
            ShowLineNumbers = false,
            WordWrap = true,
            Padding = new Thickness(10, 6, 0, 0),
        };
    }

    private Control Tab(string id, string text)
    {
        var p = Look.P;
        var active = id == _active;
        var b = new Button
        {
            Background = Brushes.Transparent,
            BorderThickness = new Thickness(0, 0, 0, 2),
            BorderBrush = active ? Look.SignalGradient() : Brushes.Transparent,
            CornerRadius = new CornerRadius(0),
            Padding = new Thickness(10, 6),
            Content = Kit.Text(text.ToUpperInvariant(), 10.5, FontWeight.SemiBold, active ? p.Ink : p.Haze),
        };
        b.Click += (_, _) => Select(id);
        return b;
    }
}

/// <summary>The status bar: project, file, cursor, model, build state and Jack's activity.</summary>
public sealed class StatusBar : Border
{
    private readonly TextBlock _project = Kit.Text("No project", 11.5, color: Colors.White);
    private readonly TextBlock _file = Kit.Text(string.Empty, 11.5, color: Colors.White);
    private readonly TextBlock _cursor = Kit.Text(string.Empty, 11.5, color: Colors.White, mono: true);
    private readonly TextBlock _model = Kit.Text(string.Empty, 11.5, color: Colors.White);
    private readonly TextBlock _build = Kit.Text("Ready", 11.5, color: Colors.White);
    private readonly TextBlock _jack = Kit.Text("Jack · idle", 11.5, color: Colors.White);
    private readonly TextBlock _targetText = Kit.Text("Simulator", 11.5, FontWeight.SemiBold);
    private readonly Border _target = new() { CornerRadius = new CornerRadius(9), Padding = new Thickness(8, 1), VerticalAlignment = VerticalAlignment.Center };
    private readonly ContentControl _targetIcon = new();
    private readonly Border _buildDot = new() { Width = 7, Height = 7, CornerRadius = new CornerRadius(4), Background = Brushes.White, Opacity = 0.8 };

    public StatusBar()
    {
        Height = 26;
        Background = Look.SignalGradient();
        Padding = new Thickness(12, 0);
        var left = Kit.Row(16, Kit.Row(6, Kit.Icon(Icons.Folder, 12, Colors.White), _project), _file);
        _target.Child = Kit.Row(5, _targetIcon, _targetText);
        var right = Kit.Row(18, _cursor, _target, Kit.Row(6, _buildDot, _build), Kit.Row(6, Kit.Icon(Icons.Spark, 12, Colors.White), _model), _jack);
        var dock = new DockPanel();
        dock.Children.Add(Kit.Dock(right, Avalonia.Controls.Dock.Right));
        dock.Children.Add(left);
        Child = dock;
    }

    public string BuildText => _build.Text ?? string.Empty;

    public void SetProject(string? name) => _project.Text = name ?? "No project";

    public void SetFile(string? name, bool dirty) => _file.Text = name is null ? string.Empty : dirty ? name + " ●" : name;

    public void SetCursor(int line, int column) => _cursor.Text = line <= 0 ? string.Empty : $"Ln {line}, Col {column}";

    public void SetModel(string? name) => _model.Text = string.IsNullOrEmpty(name) ? "No model" : name;

    public void SetBuild(string text, Color? state = null)
    {
        _build.Text = text;
        _buildDot.Background = state is { } c ? Look.Brush(c) : Brushes.White;
    }

    public string TargetText => _targetText.Text ?? string.Empty;

    public bool TargetIsRobot { get; private set; }

    /// <summary>Shows the run target: a dark pill for the simulator, a high-visibility amber pill for a physical robot.</summary>
    public void SetTarget(string name, bool robot)
    {
        TargetIsRobot = robot;
        _targetText.Text = robot ? "ROBOT · " + name : name;
        var fg = robot ? Color.Parse("#1A1200") : Colors.White;
        _targetText.Foreground = Look.Brush(fg);
        _target.Background = robot ? Look.Brush(Color.Parse("#FFB547")) : Look.Brush(Colors.Black, 0.22);
        _targetIcon.Content = Kit.Icon(robot ? Icons.Robot : Icons.Sim, 12, fg, 2.2);
        ToolTip.SetTip(_target, robot ? "Run target: physical robot" : "Run target: simulator");
    }

    public void SetJack(string activity) => _jack.Text = "Jack · " + activity.ToLowerInvariant();
}
