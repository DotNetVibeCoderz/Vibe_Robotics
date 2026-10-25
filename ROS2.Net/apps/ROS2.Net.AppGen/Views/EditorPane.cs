using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using AvaloniaEdit;
using AvaloniaEdit.Document;
using AvaloniaEdit.TextMate;
using ROS2.Net.AppGen;
using Ros2AppGen.Ui;
using TextMateSharp.Grammars;

namespace Ros2AppGen.Views;

/// <summary>An open file in the editor.</summary>
public sealed class EditorDocument
{
    internal EditorDocument(string fullPath, string text)
    {
        FullPath = fullPath;
        Document = new TextDocument(text);
        SavedText = text;
    }

    public string FullPath { get; }

    public string FileName => Path.GetFileName(FullPath);

    public TextDocument Document { get; }

    public string SavedText { get; internal set; }

    public bool IsDirty { get; internal set; }

    internal Vector ScrollOffset { get; set; }

    internal int CaretOffset { get; set; }
}

/// <summary>
/// The center of the app: a tab strip over one AvaloniaEdit <see cref="TextEditor"/> whose document
/// is swapped per tab, with TextMate highlighting chosen by file extension.
/// </summary>
public sealed class EditorPane : DockPanel
{
    private readonly AppGenHost _host;
    private readonly List<EditorDocument> _docs = [];
    private readonly StackPanel _tabs = new() { Orientation = Orientation.Horizontal };
    private readonly ContentControl _body = new();
    private readonly RegistryOptions _registry;
    private readonly TextMate.Installation _textMate;
    private readonly Border _tabBar;
    private Control? _empty;
    private bool _switching;

    public EditorPane(AppGenHost host)
    {
        _host = host;
        var p = Look.P;
        Background = Look.Brush(p.Sunken);

        Editor = new TextEditor
        {
            FontFamily = Look.Mono,
            FontSize = host.Settings.EditorFontSize,
            ShowLineNumbers = host.Settings.ShowLineNumbers,
            WordWrap = host.Settings.WordWrap,
            Background = Look.Brush(p.Sunken),
            Foreground = Look.Brush(p.Ink),
            LineNumbersForeground = Look.Brush(p.Faint),
            HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Auto,
            VerticalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Auto,
            Padding = new Thickness(6, 8, 0, 0),
        };
        Editor.Options.ConvertTabsToSpaces = true;
        Editor.Options.IndentationSize = 4;
        Editor.Options.HighlightCurrentLine = true;
        Editor.Options.EnableRectangularSelection = true;
        Editor.TextArea.SelectionBrush = Look.Brush(p.Signal, 0.35);
        Editor.TextArea.TextView.CurrentLineBackground = Look.Brush(p.Raised, 0.6);
        Editor.TextArea.TextView.CurrentLineBorder = new Pen(Brushes.Transparent);
        Editor.TextArea.Caret.CaretBrush = Look.Brush(p.Telemetry);
        Editor.TextArea.Caret.PositionChanged += (_, _) => CaretMoved?.Invoke(Editor.TextArea.Caret.Line, Editor.TextArea.Caret.Column);
        Editor.TextChanged += OnTextChanged;
        Editor.TextArea.SelectionChanged += (_, _) => _host.Workspace.ActiveSelection = Editor.SelectedText;

        _registry = new RegistryOptions(Look.IsDark ? ThemeName.DarkPlus : ThemeName.LightPlus);
        _textMate = Editor.InstallTextMate(_registry);
        _textMate.AppliedTheme += (_, _) => ApplyEditorColors();
        ApplyEditorColors();

        _tabBar = new Border
        {
            Background = Look.Brush(p.Surface),
            BorderBrush = Look.Brush(p.Seam),
            BorderThickness = new Thickness(0, 0, 0, 1),
            Height = 36,
            Child = new ScrollViewer
            {
                HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Hidden,
                VerticalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Disabled,
                Content = _tabs,
            },
        };
        Children.Add(Kit.Dock(_tabBar, Avalonia.Controls.Dock.Top));
        Children.Add(_body);

        host.Workspace.FileChanged += path => Dispatcher.UIThread.Post(() => OnFileChanged(path));
        Refresh();
    }

    public TextEditor Editor { get; }

    public IReadOnlyList<EditorDocument> Documents => _docs;

    public EditorDocument? Active { get; private set; }

    /// <summary>Raised with (line, column) as the caret moves.</summary>
    public event Action<int, int>? CaretMoved;

    /// <summary>Raised when the active document or its dirty state changes.</summary>
    public event Action? ActiveChanged;

    /// <summary>Shown when no file is open (the welcome view).</summary>
    public Control? EmptyContent
    {
        get => _empty;
        set
        {
            _empty = value;
            Refresh();
        }
    }

    public bool ShowLineNumbers
    {
        get => Editor.ShowLineNumbers;
        set => Editor.ShowLineNumbers = value;
    }

    public bool WordWrap
    {
        get => Editor.WordWrap;
        set => Editor.WordWrap = value;
    }

    public double EditorFontSize
    {
        get => Editor.FontSize;
        set => Editor.FontSize = Math.Clamp(value, 8, 32);
    }

    /// <summary>Opens a file (absolute, or relative to the workspace root) and focuses it.</summary>
    public EditorDocument? Open(string path)
    {
        var full = ToFull(path);
        if (full is null || !File.Exists(full))
        {
            _host.WriteLog($"File not found: {path}");
            return null;
        }

        var doc = _docs.FirstOrDefault(d => string.Equals(d.FullPath, full, StringComparison.OrdinalIgnoreCase));
        if (doc is null)
        {
            if (new FileInfo(full).Length > 8 * 1024 * 1024)
            {
                _host.WriteLog($"{Path.GetFileName(full)} is larger than 8 MB and was not opened.");
                return null;
            }

            doc = new EditorDocument(full, File.ReadAllText(full));
            _docs.Add(doc);
        }

        Activate(doc);
        return doc;
    }

    public void Activate(EditorDocument doc)
    {
        if (Active is { } current)
        {
            current.CaretOffset = Editor.CaretOffset;
            current.ScrollOffset = new Vector(Editor.HorizontalOffset, Editor.VerticalOffset);
        }

        _switching = true;
        Active = doc;
        Editor.Document = doc.Document;
        SetGrammar(doc.FullPath);
        Editor.CaretOffset = Math.Min(doc.CaretOffset, doc.Document.TextLength);
        _switching = false;
        Dispatcher.UIThread.Post(() => Editor.ScrollToVerticalOffset(doc.ScrollOffset.Y), DispatcherPriority.Background);
        _host.Workspace.ActiveFile = Relative(doc.FullPath);
        Refresh();
        ActiveChanged?.Invoke();
        Dispatcher.UIThread.Post(() => Editor.TextArea.Focus(), DispatcherPriority.Background);
    }

    /// <summary>Saves the active file (or <paramref name="doc"/>).</summary>
    public void Save(EditorDocument? doc = null)
    {
        doc ??= Active;
        if (doc is null)
        {
            return;
        }

        var text = doc.Document.Text;
        if (Relative(doc.FullPath) is { } rel && _host.Workspace.RootPath is not null)
        {
            _host.Workspace.WriteFile(rel, text);
        }
        else
        {
            File.WriteAllText(doc.FullPath, text);
        }

        doc.SavedText = text;
        doc.IsDirty = false;
        Refresh();
        ActiveChanged?.Invoke();
    }

    public void SaveAll()
    {
        foreach (var d in _docs.Where(d => d.IsDirty).ToList())
        {
            Save(d);
        }
    }

    public bool HasUnsavedChanges => _docs.Any(d => d.IsDirty);

    public void Close(EditorDocument doc)
    {
        var index = _docs.IndexOf(doc);
        if (index < 0)
        {
            return;
        }

        _docs.RemoveAt(index);
        if (Active == doc)
        {
            Active = null;
            if (_docs.Count > 0)
            {
                Activate(_docs[Math.Min(index, _docs.Count - 1)]);
                return;
            }

            _host.Workspace.ActiveFile = null;
        }

        Refresh();
        ActiveChanged?.Invoke();
    }

    public void CloseAll()
    {
        _docs.Clear();
        Active = null;
        _host.Workspace.ActiveFile = null;
        Refresh();
        ActiveChanged?.Invoke();
    }

    /// <summary>Moves the caret to a 1-based line and scrolls it into view.</summary>
    public bool GoToLine(int line)
    {
        if (Active is null || line < 1)
        {
            return false;
        }

        line = Math.Min(line, Editor.Document.LineCount);
        var target = Editor.Document.GetLineByNumber(line);
        Editor.CaretOffset = target.Offset;
        Editor.TextArea.Caret.BringCaretToView();
        Editor.ScrollToLine(line);
        Editor.TextArea.Focus();
        return true;
    }

    /// <summary>Formats the active document with the host formatter. Returns false when the file type is unsupported.</summary>
    public bool Format()
    {
        if (Active is null || !_host.Formatter.CanFormat(Active.FileName))
        {
            return false;
        }

        var original = Editor.Document.Text;
        var formatted = _host.Formatter.Format(Active.FileName, original);
        if (formatted != original)
        {
            var caret = Editor.CaretOffset;
            Editor.Document.Replace(0, Editor.Document.TextLength, formatted);
            Editor.CaretOffset = Math.Min(caret, Editor.Document.TextLength);
        }

        return true;
    }

    /// <summary>Inserts text at the caret, replacing the selection (used by "Insert" on Jack's code blocks).</summary>
    public void InsertAtCaret(string text)
    {
        if (Active is null)
        {
            return;
        }

        if (Editor.SelectionLength > 0)
        {
            Editor.Document.Replace(Editor.SelectionStart, Editor.SelectionLength, text);
        }
        else
        {
            Editor.Document.Insert(Editor.CaretOffset, text);
        }
    }

    public void OpenFind()
    {
        Editor.SearchPanel?.Open();
    }

    /// <summary>Re-reads colors after a theme switch.</summary>
    public void ApplyTheme()
    {
        _textMate.SetTheme(_registry.LoadTheme(Look.IsDark ? ThemeName.DarkPlus : ThemeName.LightPlus));
        var p = Look.P;
        Background = Look.Brush(p.Sunken);
        _tabBar.Background = Look.Brush(p.Surface);
        _tabBar.BorderBrush = Look.Brush(p.Seam);
        ApplyEditorColors();
        Refresh();
    }

    private void ApplyEditorColors()
    {
        var p = Look.P;
        Editor.Background = Look.Brush(p.Sunken);
        Editor.Foreground = Look.Brush(p.Ink);
        Editor.LineNumbersForeground = Look.Brush(p.Faint);
        Editor.TextArea.TextView.CurrentLineBackground = Look.Brush(p.Raised, 0.55);
        Editor.TextArea.SelectionBrush = Look.Brush(p.Signal, 0.35);
        Editor.TextArea.Caret.CaretBrush = Look.Brush(p.Telemetry);
    }

    private void SetGrammar(string path)
    {
        var ext = Path.GetExtension(path).ToLowerInvariant() switch
        {
            ".csproj" or ".props" or ".targets" or ".axaml" or ".slnx" or ".config" or ".resx" or ".xaml" or ".urdf" or ".launch" => ".xml",
            ".razor" or ".cshtml" => ".html",
            ".msg" or ".srv" or ".action" => ".py",
            var e => e,
        };
        try
        {
            var language = _registry.GetLanguageByExtension(ext);
            if (language is null)
            {
                _textMate.SetGrammar(null);
                return;
            }

            _textMate.SetGrammar(_registry.GetScopeByLanguageId(language.Id));
        }
        catch (Exception)
        {
            _textMate.SetGrammar(null);
        }
    }

    private void OnTextChanged(object? sender, EventArgs e)
    {
        if (_switching || Active is null)
        {
            return;
        }

        var dirty = Active.Document.TextLength != Active.SavedText.Length || Active.Document.Text != Active.SavedText;
        if (dirty != Active.IsDirty)
        {
            Active.IsDirty = dirty;
            Refresh();
            ActiveChanged?.Invoke();
        }
    }

    private void OnFileChanged(string path)
    {
        var doc = _docs.FirstOrDefault(d => string.Equals(d.FullPath, path, StringComparison.OrdinalIgnoreCase));
        if (doc is null)
        {
            return;
        }

        if (!File.Exists(path))
        {
            Close(doc);
            return;
        }

        string text;
        try
        {
            text = File.ReadAllText(path);
        }
        catch (IOException)
        {
            return;
        }

        if (text == doc.SavedText)
        {
            return;
        }

        if (doc.IsDirty)
        {
            _host.WriteLog($"{doc.FileName} changed on disk while it has unsaved edits; keeping your version.");
            return;
        }

        // Jack (or another tool) rewrote the file: reload it in place, keeping the caret near where it was.
        var caret = doc == Active ? Editor.CaretOffset : doc.CaretOffset;
        _switching = doc == Active;
        doc.Document.Text = text;
        doc.SavedText = text;
        doc.IsDirty = false;
        _switching = false;
        if (doc == Active)
        {
            Editor.CaretOffset = Math.Min(caret, text.Length);
        }

        Refresh();
    }

    private string? ToFull(string path)
    {
        if (Path.IsPathRooted(path))
        {
            return Path.GetFullPath(path);
        }

        return _host.Workspace.RootPath is { } root ? Path.GetFullPath(Path.Combine(root, path)) : null;
    }

    private string? Relative(string full)
    {
        var root = _host.Workspace.RootPath;
        if (root is null)
        {
            return null;
        }

        var rel = Path.GetRelativePath(root, full);
        return rel.StartsWith("..", StringComparison.Ordinal) || Path.IsPathRooted(rel) ? null : rel.Replace('\\', '/');
    }

    private void Refresh()
    {
        _tabs.Children.Clear();
        foreach (var d in _docs)
        {
            _tabs.Children.Add(Tab(d));
        }

        _tabBar.IsVisible = _docs.Count > 0;
        _body.Content = Active is null ? _empty ?? new Border() : Editor;
    }

    private Control Tab(EditorDocument doc)
    {
        var p = Look.P;
        var active = doc == Active;
        var dot = new Border { Width = 7, Height = 7, CornerRadius = new CornerRadius(2), Background = Look.Brush(Kit.FileColor(doc.FullPath)) };
        var name = Kit.Text(doc.FileName, 12.5, active ? FontWeight.SemiBold : FontWeight.Normal, active ? p.Ink : p.Haze);
        var close = new Button
        {
            Padding = new Thickness(4),
            Background = Brushes.Transparent,
            BorderThickness = new Thickness(0),
            CornerRadius = new CornerRadius(4),
            Content = doc.IsDirty
                ? new Border { Width = 8, Height = 8, CornerRadius = new CornerRadius(4), Background = Look.Brush(p.Warn) }
                : Kit.Icon(Icons.Close, 10, p.Faint, 2.4),
        };
        ToolTip.SetTip(close, doc.IsDirty ? "Unsaved changes — click to close" : "Close");
        close.Click += (_, _) => Close(doc);

        var tab = new Border
        {
            Padding = new Thickness(12, 0, 6, 0),
            Background = active ? Look.Brush(p.Sunken) : Brushes.Transparent,
            BorderBrush = active ? Look.SignalGradient() : Look.Brush(p.Seam),
            BorderThickness = active ? new Thickness(0, 2, 0, 0) : new Thickness(0, 0, 1, 0),
            Cursor = new Cursor(StandardCursorType.Hand),
            Child = Kit.Row(8, dot, name, close),
        };
        ToolTip.SetTip(tab, doc.FullPath);
        tab.PointerPressed += (_, e) =>
        {
            if (e.GetCurrentPoint(tab).Properties.IsMiddleButtonPressed)
            {
                Close(doc);
            }
            else
            {
                Activate(doc);
            }
        };
        return tab;
    }
}
