using System.Diagnostics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.AppGen;
using Ros2AppGen.Ui;

namespace Ros2AppGen.Views;

/// <summary>
/// VS Code-style explorer: a lazily expanded tree of the open project with indent guides, file-type
/// colors and a context menu. It refreshes when the workspace reports file changes.
/// </summary>
public sealed class ExplorerPane : DockPanel
{
    private static readonly HashSet<string> s_hidden = new(StringComparer.OrdinalIgnoreCase) { "bin", "obj", ".git", ".vs", ".idea", "node_modules", "artifacts", "TestResults" };

    private readonly AppGenHost _host;
    private readonly StackPanel _rows = new();
    private readonly HashSet<string> _expanded = new(StringComparer.OrdinalIgnoreCase);
    private readonly TextBlock _projectName;
    private string? _selected;
    private string? _root;
    private bool _refreshQueued;

    public ExplorerPane(AppGenHost host)
    {
        _host = host;
        var p = Look.P;
        Background = Look.Brush(p.Surface);

        _projectName = Kit.Text("No project", 12.5, FontWeight.SemiBold);
        var header = new DockPanel { Margin = new Thickness(14, 10, 6, 6) };
        header.Children.Add(Kit.Dock(Kit.Row(0,
            Kit.IconButton(Icons.NewProject, "New file", () => _ = NewFileAsync(null), size: 14),
            Kit.IconButton(Icons.Folder, "New folder", () => _ = NewFolderAsync(null), size: 14),
            Kit.IconButton(Icons.Refresh, "Refresh", Refresh, size: 14),
            Kit.IconButton(Icons.Format, "Collapse all", CollapseAll, size: 14)), Avalonia.Controls.Dock.Right));
        header.Children.Add(Kit.Eyebrow("Explorer"));
        Children.Add(Kit.Dock(header, Avalonia.Controls.Dock.Top));
        Children.Add(Kit.Dock(new Border { Padding = new Thickness(14, 0, 8, 8), Child = _projectName }, Avalonia.Controls.Dock.Top));
        Children.Add(new ScrollViewer { Content = _rows, HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Disabled });

        host.Workspace.FileChanged += OnFileChanged;
        host.Workspace.ProjectChanged += OnProjectChanged;
        if (host.Workspace.RootPath is { } r)
        {
            _expanded.Add(r);
        }

        Refresh();
    }

    private void OnFileChanged(string _) => Dispatcher.UIThread.Post(QueueRefresh);

    private void OnProjectChanged() => Dispatcher.UIThread.Post(() =>
    {
        // Only reset the tree when a different project opened (a Reveal may already have expanded folders).
        var root = _host.Workspace.RootPath;
        if (!string.Equals(root, _root, StringComparison.OrdinalIgnoreCase))
        {
            _expanded.Clear();
            _selected = null;
            _root = root;
            if (root is not null)
            {
                _expanded.Add(root);
            }
        }

        Refresh();
    });

    /// <summary>Unsubscribes from the workspace (call before discarding the pane).</summary>
    public void Release()
    {
        _host.Workspace.FileChanged -= OnFileChanged;
        _host.Workspace.ProjectChanged -= OnProjectChanged;
    }

    /// <summary>Raised with the absolute path when a file is opened (double-click / Enter).</summary>
    public event Action<string>? FileOpened;

    /// <summary>Window used as owner for prompts.</summary>
    public Func<Window?>? Owner { get; set; }

    /// <summary>Number of rows currently shown (for tests).</summary>
    public int VisibleRowCount => _rows.Children.Count;

    public void Reveal(string fullPath)
    {
        var root = _host.Workspace.RootPath;
        if (root is null)
        {
            return;
        }

        if (!string.Equals(root, _root, StringComparison.OrdinalIgnoreCase))
        {
            _expanded.Clear();
            _root = root;
        }

        var dir = Path.GetDirectoryName(fullPath);
        while (dir is not null && dir.Length >= root.Length)
        {
            _expanded.Add(dir);
            dir = Path.GetDirectoryName(dir);
        }

        _selected = fullPath;
        Refresh();
    }

    public void Refresh()
    {
        _rows.Children.Clear();
        var root = _host.Workspace.RootPath;
        if (root is null || !Directory.Exists(root))
        {
            _projectName.Text = "No project open";
            _rows.Children.Add(new Border
            {
                Padding = new Thickness(14, 6, 14, 0),
                Child = Kit.Muted("Create or open a project to browse its files here.", 12, wrap: true),
            });
            return;
        }

        _projectName.Text = Path.GetFileName(root.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
        AddChildren(root, 0);
    }

    private void QueueRefresh()
    {
        if (_refreshQueued)
        {
            return;
        }

        _refreshQueued = true;
        DispatcherTimer.RunOnce(() =>
        {
            _refreshQueued = false;
            Refresh();
        }, TimeSpan.FromMilliseconds(150));
    }

    private void CollapseAll()
    {
        _expanded.Clear();
        if (_host.Workspace.RootPath is { } root)
        {
            _expanded.Add(root);
        }

        Refresh();
    }

    private void AddChildren(string folder, int depth)
    {
        IEnumerable<string> dirs, files;
        try
        {
            dirs = Directory.EnumerateDirectories(folder)
                .Where(d => !s_hidden.Contains(Path.GetFileName(d)))
                .Order(StringComparer.OrdinalIgnoreCase);
            files = Directory.EnumerateFiles(folder).Order(StringComparer.OrdinalIgnoreCase);
        }
        catch (Exception)
        {
            return;
        }

        foreach (var d in dirs)
        {
            var open = _expanded.Contains(d);
            _rows.Children.Add(Row(d, depth, isFolder: true, open));
            if (open)
            {
                AddChildren(d, depth + 1);
            }
        }

        foreach (var f in files)
        {
            _rows.Children.Add(Row(f, depth, isFolder: false, false));
        }
    }

    private Control Row(string path, int depth, bool isFolder, bool open)
    {
        var p = Look.P;
        var selected = string.Equals(_selected, path, StringComparison.OrdinalIgnoreCase);
        var name = Path.GetFileName(path);

        Control glyph;
        if (isFolder)
        {
            glyph = Kit.Icon(open ? Icons.ChevronDown : Icons.Chevron, 11, p.Faint, 2.4);
        }
        else
        {
            glyph = new Border { Width = 11 };
        }

        Control icon = isFolder
            ? Kit.Icon(Icons.Folder, 14, open ? p.Telemetry : p.Haze, 1.8)
            : new Border
            {
                Width = 14,
                Child = new TextBlock
                {
                    Text = Badge(name),
                    FontFamily = Look.Mono,
                    FontSize = 8.5,
                    FontWeight = FontWeight.Bold,
                    Foreground = Look.Brush(Kit.FileColor(path)),
                    HorizontalAlignment = HorizontalAlignment.Center,
                    VerticalAlignment = VerticalAlignment.Center,
                },
            };

        // Indent guides: one hairline per level, like VS Code.
        var guides = new StackPanel { Orientation = Orientation.Horizontal };
        for (var i = 0; i < depth; i++)
        {
            guides.Children.Add(new Border { Width = 14, Child = new Border { Width = 1, HorizontalAlignment = HorizontalAlignment.Left, Margin = new Thickness(6, 0, 0, 0), Background = Look.Brush(p.Seam) } });
        }

        var label = Kit.Text(name, 12.5, selected ? FontWeight.SemiBold : FontWeight.Normal, selected ? p.Ink : isFolder ? p.Ink : p.Haze);
        label.TextTrimming = TextTrimming.CharacterEllipsis;

        var content = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 6, Children = { guides, glyph, icon, label } };
        var row = new Border
        {
            Height = 24,
            Padding = new Thickness(10, 0, 8, 0),
            Background = selected ? Look.Brush(p.Signal, 0.16) : Brushes.Transparent,
            BorderBrush = selected ? Look.Brush(p.Signal) : Brushes.Transparent,
            BorderThickness = new Thickness(2, 0, 0, 0),
            Child = content,
            Cursor = new Cursor(StandardCursorType.Hand),
            Tag = path,
        };
        ToolTip.SetTip(row, Path.GetRelativePath(_host.Workspace.RootPath ?? path, path));
        row.PointerEntered += (_, _) =>
        {
            if (!selected)
            {
                row.Background = Look.Brush(p.Raised, 0.8);
            }
        };
        row.PointerExited += (_, _) =>
        {
            if (!selected)
            {
                row.Background = Brushes.Transparent;
            }
        };
        row.PointerPressed += (_, e) =>
        {
            if (!e.GetCurrentPoint(row).Properties.IsLeftButtonPressed)
            {
                return;
            }

            _selected = path;
            if (isFolder)
            {
                if (!_expanded.Add(path))
                {
                    _expanded.Remove(path);
                }
            }
            else
            {
                FileOpened?.Invoke(path);
            }

            Refresh();
        };
        row.ContextMenu = BuildMenu(path, isFolder);
        return row;
    }

    private static string Badge(string name) => Path.GetExtension(name).ToLowerInvariant() switch
    {
        ".cs" => "C#",
        ".csproj" or ".props" or ".targets" or ".slnx" => "PJ",
        ".axaml" or ".xaml" => "AX",
        ".razor" => "RZ",
        ".json" => "{}",
        ".md" => "MD",
        ".msg" or ".srv" or ".action" => "IF",
        ".yaml" or ".yml" => "YM",
        ".config" or ".xml" => "<>",
        ".css" => "#",
        ".js" => "JS",
        ".png" or ".jpg" or ".ico" or ".svg" => "IM",
        ".urdf" => "UR",
        _ => "··",
    };

    private ContextMenu BuildMenu(string path, bool isFolder)
    {
        var folder = isFolder ? path : Path.GetDirectoryName(path)!;
        var items = new List<object>();
        if (!isFolder)
        {
            items.Add(Item("Open", () => FileOpened?.Invoke(path)));
        }

        items.Add(Item("New file…", () => _ = NewFileAsync(folder)));
        items.Add(Item("New folder…", () => _ = NewFolderAsync(folder)));
        items.Add(new Separator());
        items.Add(Item("Rename…", () => _ = RenameAsync(path)));
        items.Add(Item("Delete", () => _ = DeleteAsync(path, isFolder)));
        items.Add(new Separator());
        items.Add(Item("Copy relative path", () => _ = CopyAsync(Rel(path))));
        items.Add(Item("Reveal in file manager", () => RevealInShell(path)));
        return new ContextMenu { ItemsSource = items };
    }

    private static MenuItem Item(string header, Action action)
    {
        var m = new MenuItem { Header = header };
        m.Click += (_, _) => action();
        return m;
    }

    private string Rel(string full) => Path.GetRelativePath(_host.Workspace.RootPath!, full).Replace('\\', '/');

    private async Task NewFileAsync(string? folder)
    {
        if (_host.Workspace.RootPath is null || Owner?.Invoke() is not { } owner)
        {
            return;
        }

        folder ??= _host.Workspace.RootPath;
        var name = await InputDialog.AskAsync(owner, "New file", "File name", "NewFile.cs", $"Created in {Rel(folder)}");
        if (string.IsNullOrWhiteSpace(name))
        {
            return;
        }

        var rel = Path.Combine(Rel(folder), name.Trim()).Replace('\\', '/').TrimStart('.', '/');
        _host.Workspace.WriteFile(rel, string.Empty);
        var full = Path.Combine(_host.Workspace.RootPath, rel);
        Reveal(full);
        FileOpened?.Invoke(full);
    }

    private async Task NewFolderAsync(string? folder)
    {
        if (_host.Workspace.RootPath is null || Owner?.Invoke() is not { } owner)
        {
            return;
        }

        folder ??= _host.Workspace.RootPath;
        var name = await InputDialog.AskAsync(owner, "New folder", "Folder name", "Folder");
        if (string.IsNullOrWhiteSpace(name))
        {
            return;
        }

        var rel = Path.Combine(Rel(folder), name.Trim()).Replace('\\', '/').TrimStart('.', '/');
        _host.Workspace.CreateFolder(rel);
        Reveal(Path.Combine(_host.Workspace.RootPath, rel, "x"));
    }

    private async Task RenameAsync(string path)
    {
        if (Owner?.Invoke() is not { } owner)
        {
            return;
        }

        var name = await InputDialog.AskAsync(owner, "Rename", "New name", Path.GetFileName(path));
        if (string.IsNullOrWhiteSpace(name) || name == Path.GetFileName(path))
        {
            return;
        }

        var target = Path.Combine(Path.GetDirectoryName(path)!, name.Trim());
        _host.Workspace.RenameFile(Rel(path), Rel(target));
        _selected = target;
        Refresh();
    }

    private async Task DeleteAsync(string path, bool isFolder)
    {
        if (Owner?.Invoke() is not { } owner)
        {
            return;
        }

        var what = isFolder ? "folder and everything in it" : "file";
        var answer = await ConfirmDialog.AskAsync(owner, $"Delete {Path.GetFileName(path)}?", $"This permanently deletes the {what}.", "Delete");
        if (answer != "confirm")
        {
            return;
        }

        if (isFolder)
        {
            Directory.Delete(path, recursive: true);
        }
        else
        {
            _host.Workspace.DeleteFile(Rel(path));
        }

        Refresh();
    }

    private async Task CopyAsync(string text)
    {
        if (TopLevel.GetTopLevel(this)?.Clipboard is { } clipboard)
        {
            await ClipboardImages.SetTextAsync(clipboard, text);
        }
    }

    private void RevealInShell(string path)
    {
        try
        {
            if (OperatingSystem.IsWindows())
            {
                Process.Start(new ProcessStartInfo("explorer.exe", $"/select,\"{path}\"") { UseShellExecute = true });
            }
            else
            {
                Process.Start(new ProcessStartInfo(Path.GetDirectoryName(path)!) { UseShellExecute = true });
            }
        }
        catch (Exception ex)
        {
            _host.WriteLog("Could not open the file manager: " + ex.Message);
        }
    }
}
