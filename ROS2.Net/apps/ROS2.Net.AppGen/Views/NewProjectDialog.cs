using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Platform.Storage;
using ROS2.Net.AppGen;
using Ros2AppGen.Ui;

namespace Ros2AppGen.Views;

/// <summary>
/// New project: a blank ROS2.Net app or one of the use-case templates, filtered by category and search.
/// Closes with the created project folder.
/// </summary>
public sealed class NewProjectDialog : AppDialog
{
    private readonly AppGenHost _host;
    private readonly TextBox _name;
    private readonly TextBox _location;
    private readonly TextBox _search;
    private readonly WrapPanel _cards = new() { Orientation = Orientation.Horizontal };
    private readonly StackPanel _categories = new() { Spacing = 2 };
    private readonly ContentControl _modeBody = new();
    private readonly StackPanel _modeTabs = new() { Orientation = Orientation.Horizontal, Spacing = 6 };
    private readonly TextBlock _error;
    private string _mode = "template";
    private string _category = "All";
    private string _appKind = "console";
    private ProjectTemplate? _selected;

    public NewProjectDialog(AppGenHost host, string mode = "template")
        : base("New project", "Start blank or from a ROS2.Net use-case template. Jack can take it from there.", 1040, 720)
    {
        _host = host;
        _mode = mode;
        var p = Look.P;
        _name = Kit.Input("MyRobotApp");
        var folder = string.IsNullOrWhiteSpace(host.Settings.ProjectsFolder)
            ? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "ROS2AppGen")
            : host.Settings.ProjectsFolder;
        _location = Kit.Input(folder, mono: true);
        _search = Kit.Input(string.Empty, "Search templates");
        _search.TextChanged += (_, _) => RenderCards();
        _selected = host.Projects.Templates.FirstOrDefault();
        _error = Kit.Text(string.Empty, 12, color: p.Fault, wrap: true);

        var browse = Kit.Button("Browse…", () => _ = BrowseAsync());
        var locationRow = new DockPanel { Children = { Kit.Dock(browse, Avalonia.Controls.Dock.Right), _location } };
        browse.Margin = new Thickness(6, 0, 0, 0);

        var top = new Grid { ColumnDefinitions = new ColumnDefinitions("260,*"), ColumnSpacing = 16 };
        top.Children.Add(Kit.At(Kit.Field("Project name", _name), 0, 0));
        top.Children.Add(Kit.At(Kit.Field("Location", locationRow), 0, 1));

        var body = new DockPanel();
        body.Children.Add(Kit.Dock(Kit.Col(14, _modeTabs, top), Avalonia.Controls.Dock.Top));
        body.Children.Add(Kit.Dock(_error, Avalonia.Controls.Dock.Bottom));
        _modeBody.Margin = new Thickness(0, 14, 0, 0);
        body.Children.Add(_modeBody);
        SetBody(body);

        AddButton("Cancel", () => Close(null));
        AddButton("Create project", Create, primary: true);
        KeyDown += (_, e) =>
        {
            if (e.Key == Key.Enter && e.KeyModifiers.HasFlag(KeyModifiers.Control))
            {
                Create();
            }
        };
        RenderMode();
    }

    /// <summary>Selects a template by id (tests / screenshots).</summary>
    public void SelectTemplate(string id)
    {
        _selected = _host.Projects.Templates.FirstOrDefault(t => t.Id == id) ?? _selected;
        RenderCards();
    }

    public void SetMode(string mode)
    {
        _mode = mode;
        RenderMode();
    }

    private void RenderMode()
    {
        _modeTabs.Children.Clear();
        _modeTabs.Children.Add(ModeTab("template", "From template", Icons.Explorer));
        _modeTabs.Children.Add(ModeTab("blank", "Blank", Icons.NewProject));
        _modeBody.Content = _mode == "blank" ? BlankBody() : TemplateBody();
    }

    private Control ModeTab(string id, string text, string icon)
    {
        var p = Look.P;
        var active = id == _mode;
        var b = new Button
        {
            Padding = new Thickness(14, 8),
            CornerRadius = new CornerRadius(8),
            Background = active ? Look.Brush(p.Signal, 0.18) : Look.Brush(p.Raised),
            BorderBrush = Look.Brush(active ? p.Signal : p.Seam),
            BorderThickness = new Thickness(1),
            Content = Kit.Row(8, Kit.Icon(icon, 14, active ? p.Telemetry : p.Haze), Kit.Text(text, 12.5, FontWeight.SemiBold, active ? p.Ink : p.Haze)),
        };
        b.Click += (_, _) =>
        {
            _mode = id;
            RenderMode();
        };
        return b;
    }

    private Control BlankBody()
    {
        var p = Look.P;
        var kinds = new (string Id, string Title, string Text)[]
        {
            ("console", "Console node", "A ROS2.Net node with a publisher, a subscription and a timer. Runs headless; best for robot-side services."),
            ("avalonia", "Desktop app", "An Avalonia window wired to a ROS2.Net context — for operator dashboards and tools."),
            ("blazor", "Web app", "A Blazor Server app hosting ROS2.Net — for browser dashboards across the network."),
        };
        var row = new WrapPanel();
        foreach (var k in kinds)
        {
            var selected = k.Id == _appKind;
            var card = new Button
            {
                Width = 300,
                Height = 130,
                Margin = new Thickness(0, 0, 12, 12),
                Padding = new Thickness(16),
                HorizontalContentAlignment = HorizontalAlignment.Left,
                VerticalContentAlignment = VerticalAlignment.Top,
                CornerRadius = new CornerRadius(10),
                Background = Look.Brush(selected ? p.Raised : p.Surface),
                BorderBrush = selected ? Look.SignalGradient() : Look.Brush(p.Seam),
                BorderThickness = new Thickness(selected ? 2 : 1),
                Content = Kit.Col(6, Kit.Text(k.Title, 14, FontWeight.Bold), Kit.Muted(k.Text, 12, wrap: true)),
            };
            card.Click += (_, _) =>
            {
                _appKind = k.Id;
                RenderMode();
            };
            row.Children.Add(card);
        }

        return Kit.Col(10, Kit.Eyebrow("App kind"), row);
    }

    private Control TemplateBody()
    {
        var p = Look.P;
        _categories.Children.Clear();
        foreach (var c in new[] { "All" }.Concat(_host.Projects.Categories))
        {
            var count = c == "All" ? _host.Projects.Templates.Count : _host.Projects.Templates.Count(t => t.Category == c);
            var active = c == _category;
            var b = new Button
            {
                HorizontalAlignment = HorizontalAlignment.Stretch,
                HorizontalContentAlignment = HorizontalAlignment.Stretch,
                Background = active ? Look.Brush(p.Signal, 0.16) : Brushes.Transparent,
                BorderThickness = new Thickness(0),
                CornerRadius = new CornerRadius(6),
                Padding = new Thickness(10, 6),
                Content = new DockPanel
                {
                    Children =
                    {
                        Kit.Dock(Kit.Text(count.ToString(), 11, color: p.Faint, mono: true), Avalonia.Controls.Dock.Right),
                        Kit.Text(c, 12.5, active ? FontWeight.SemiBold : FontWeight.Normal, active ? p.Ink : p.Haze),
                    },
                },
            };
            b.Click += (_, _) =>
            {
                _category = c;
                RenderMode();
            };
            _categories.Children.Add(b);
        }

        RenderCards();
        var grid = new Grid { ColumnDefinitions = new ColumnDefinitions("190,*"), ColumnSpacing = 16 };
        grid.Children.Add(Kit.At(new ScrollViewer { Content = Kit.Col(8, Kit.Eyebrow("Categories"), Kit.Detach(_categories)) }, 0, 0));
        var right = new DockPanel();
        right.Children.Add(Kit.Dock(new Border { Margin = new Thickness(0, 0, 0, 10), Child = Kit.Detach(_search) }, Avalonia.Controls.Dock.Top));
        right.Children.Add(new ScrollViewer { Content = Kit.Detach(_cards), HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Disabled });
        grid.Children.Add(Kit.At(right, 0, 1));
        return grid;
    }

    private void RenderCards()
    {
        var p = Look.P;
        _cards.Children.Clear();
        var query = (_search.Text ?? string.Empty).Trim();
        var templates = _host.Projects.Templates
            .Where(t => _category == "All" || t.Category == _category)
            .Where(t => query.Length == 0
                        || t.Name.Contains(query, StringComparison.OrdinalIgnoreCase)
                        || t.Description.Contains(query, StringComparison.OrdinalIgnoreCase)
                        || t.Tags.Any(tag => tag.Contains(query, StringComparison.OrdinalIgnoreCase)))
            .ToList();
        if (templates.Count == 0)
        {
            _cards.Children.Add(Kit.Muted("No template matches. Clear the search or pick another category.", 12.5));
            return;
        }

        foreach (var t in templates)
        {
            var selected = _selected?.Id == t.Id;
            var tags = new WrapPanel();
            tags.Children.Add(WithMargin(Kit.Chip(t.AppKind, t.AppKind switch { "blazor" => p.Warn, "avalonia" => p.Signal, _ => p.Telemetry })));
            foreach (var tag in t.Tags.Take(3))
            {
                tags.Children.Add(WithMargin(Kit.Chip(tag, p.Haze)));
            }

            var card = new Button
            {
                Width = 252,
                Height = 168,
                Margin = new Thickness(0, 0, 10, 10),
                Padding = new Thickness(14, 12),
                HorizontalContentAlignment = HorizontalAlignment.Stretch,
                VerticalContentAlignment = VerticalAlignment.Stretch,
                CornerRadius = new CornerRadius(10),
                Background = Look.Brush(selected ? p.Raised : p.Surface),
                BorderBrush = selected ? Look.SignalGradient() : Look.Brush(p.Seam),
                BorderThickness = new Thickness(selected ? 2 : 1),
                Content = new DockPanel
                {
                    Children =
                    {
                        Kit.Dock(Kit.Text(t.Category.ToUpperInvariant(), 9.5, FontWeight.SemiBold, p.Telemetry), Avalonia.Controls.Dock.Top),
                        Kit.Dock(new Border { Margin = new Thickness(0, 4, 0, 6), Child = Kit.Text(t.Name, 14, FontWeight.Bold, wrap: true) }, Avalonia.Controls.Dock.Top),
                        Kit.Dock(tags, Avalonia.Controls.Dock.Bottom),
                        new TextBlock { Text = t.Description, FontSize = 11.5, TextWrapping = TextWrapping.Wrap, Foreground = Look.Brush(p.Haze), TextTrimming = TextTrimming.WordEllipsis, MaxHeight = 52 },
                    },
                },
            };
            ToolTip.SetTip(card, t.Description);
            card.Click += (_, _) =>
            {
                _selected = t;
                RenderCards();
            };
            card.DoubleTapped += (_, _) =>
            {
                _selected = t;
                Create();
            };
            _cards.Children.Add(card);
        }
    }

    private static Control WithMargin(Control c)
    {
        c.Margin = new Thickness(0, 4, 4, 0);
        return c;
    }

    private async Task BrowseAsync()
    {
        var folders = await StorageProvider.OpenFolderPickerAsync(new FolderPickerOpenOptions { Title = "Choose where to create the project" });
        if (folders.FirstOrDefault()?.TryGetLocalPath() is { } path)
        {
            _location.Text = path;
        }
    }

    private void Create()
    {
        var name = (_name.Text ?? string.Empty).Trim();
        var parent = (_location.Text ?? string.Empty).Trim();
        if (name.Length == 0 || name.Any(c => Path.GetInvalidFileNameChars().Contains(c) || c == ' '))
        {
            _error.Text = "Use a project name without spaces or path characters, e.g. WarehouseMonitor.";
            return;
        }

        if (parent.Length == 0)
        {
            _error.Text = "Choose a location for the project.";
            return;
        }

        if (Directory.Exists(Path.Combine(parent, name)) && Directory.EnumerateFileSystemEntries(Path.Combine(parent, name)).Any())
        {
            _error.Text = $"{Path.Combine(parent, name)} already exists and is not empty. Pick another name.";
            return;
        }

        try
        {
            Directory.CreateDirectory(parent);
            var folder = _mode == "blank" || _selected is null
                ? _host.Projects.CreateBlank(parent, name, _appKind)
                : _host.Projects.CreateFromTemplate(_selected.Id, parent, name);
            _host.Settings.ProjectsFolder = parent;
            Close(folder);
        }
        catch (Exception ex)
        {
            _error.Text = "Could not create the project: " + ex.Message;
        }
    }
}
