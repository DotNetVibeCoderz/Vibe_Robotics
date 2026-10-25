using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Input.Platform;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Platform.Storage;
using Avalonia.Styling;
using Avalonia.Threading;
using ROS2.Net.AppGen;
using Ros2AppGen.Ui;
using Ros2AppGen.ViewModels;
using Ros2AppGen.Views;

namespace Ros2AppGen;

/// <summary>
/// ROS2.Net App Generator: explorer on the left, editor in the middle, Jack on the right, logs and the
/// status bar at the bottom. All panels are resizable and can be hidden; layout persists in App.config.
/// </summary>
public sealed class MainWindow : Window
{
    private readonly AppGenHost _host;
    private readonly EditorPane _editor;
    private readonly LogsPane _logs;
    private readonly ChatViewModel _chatVm;
    private ChatPane _chat = null!;
    private ExplorerPane _explorer = null!;
    private StatusBar _status = null!;
    private Grid _columns = null!;
    private Grid _rows = null!;
    private Button _runButton = null!;
    private string _buildState = "Ready";
    private Color? _buildColor;
    private CancellationTokenSource? _buildCts;
    private string? _lastBuildErrors;
    private bool _explorerVisible = true;
    private bool _closingConfirmed;
    private bool _themeChanged;
    private readonly HashSet<string> _robotRunsConfirmed = new(StringComparer.OrdinalIgnoreCase);

    public MainWindow(AppGenHost host)
    {
        _host = host;
        Look.IsDark = !string.Equals(host.Settings.Theme, "Light", StringComparison.OrdinalIgnoreCase);
        ApplyFluentVariant();

        Title = "ROS2.Net App Generator";
        Width = 1480;
        Height = 920;
        MinWidth = 900;
        MinHeight = 560;
        FontFamily = Look.Sans;
        WindowStartupLocation = WindowStartupLocation.CenterScreen;
        try
        {
            Icon = new WindowIcon(Avalonia.Platform.AssetLoader.Open(new Uri("avares://ROS2AppGen/Assets/icon-128.png")));
        }
        catch (Exception)
        {
        }

        if (host.Settings.RunTargets.Count == 0)
        {
            host.Settings.RunTargets = RunTargetEnvironment.Defaults();
        }

        _editor = new EditorPane(host);
        _logs = new LogsPane(() => SetLogsVisible(false));
        ConfirmRobotRun = target => new RobotSafetyDialog(target).ShowDialog<bool>(this);
        _chatVm = new ChatViewModel(host);
        _chatVm.StateChanged += () => Dispatcher.UIThread.Post(() => _status.SetJack(_chatVm.Activity));
        _chatVm.ToolLog += line => _logs.WriteJack(line);
        host.Log += line => _logs.WriteJack(line);

        _editor.CaretMoved += (l, c) => _status.SetCursor(l, c);
        _editor.ActiveChanged += () =>
        {
            UpdateStatus();
            if (_editor.Active is { } doc && _host.Workspace.RootPath is { } root
                && doc.FullPath.StartsWith(root, StringComparison.OrdinalIgnoreCase))
            {
                _explorer?.Reveal(doc.FullPath);
            }
        };
        host.Workspace.ProjectChanged += () => Dispatcher.UIThread.Post(UpdateStatus);

        MarkdownView.InsertCode = (_, code) =>
        {
            if (_editor.Active is null)
            {
                _logs.WriteOutput("Open a file first, then Insert puts Jack's code at the cursor.");
                return;
            }

            _editor.InsertAtCaret(code);
        };
        MarkdownView.CopyText = text => _ = Clipboard?.SetTextAsync(text);

        Build();
        KeyDown += OnKeyDown;
        Closing += OnClosing;
        Opened += (_, _) =>
        {
            _logs.WriteOutput($"ROS2.Net App Generator ready. Settings: {host.SettingsStore.ConfigPath}");
            if (host.Settings.Profiles.Count == 0)
            {
                _logs.WriteOutput("No model is configured yet: open Settings → Models to connect Jack to OpenAI, Claude, Gemini, Ollama or DeepSeek.");
            }
        };
    }

    public AppGenHost Host => _host;

    public EditorPane Editor => _editor;

    public LogsPane Logs => _logs;

    public ChatPane Chat => _chat;

    public ExplorerPane Explorer => _explorer;

    public StatusBar Status => _status;

    public ChatViewModel ChatViewModel => _chatVm;

    public bool IsChatVisible => _host.Settings.ChatPanelVisible;

    public bool IsLogsVisible => _host.Settings.LogsPanelVisible;

    // ------------------------------------------------------------------ layout

    private void Build()
    {
        var p = Look.P;
        Background = Look.Brush(p.Background);
        Detach(_editor);
        Detach(_logs);
        if (_chat is null || _themeChanged)
        {
            _chat?.Release();
            _explorer?.Release();
            _explorer = new ExplorerPane(_host) { Owner = () => this };
            _explorer.FileOpened += path => _editor.Open(path);
            _chat = new ChatPane(_host, _chatVm, () => SetChatVisible(false));
            _themeChanged = false;
            if (_editor.Active is { } active && _host.Workspace.RootPath is not null)
            {
                _explorer.Reveal(active.FullPath);
            }
        }
        else
        {
            Detach(_chat);
            Detach(_explorer);
            _chat.RefreshProfiles();
            _explorer.Refresh();
        }

        _status = new StatusBar();
        _editor.EmptyContent = Welcome();

        var s = _host.Settings;
        _columns = new Grid
        {
            ColumnDefinitions = new ColumnDefinitions($"{(_explorerVisible ? s.ExplorerWidth : 0)},Auto,*,Auto,{(s.ChatPanelVisible ? s.ChatPanelWidth : 0)}"),
        };
        var leftSplit = Splitter(vertical: true);
        var rightSplit = Splitter(vertical: true);
        leftSplit.IsVisible = _explorerVisible;
        rightSplit.IsVisible = s.ChatPanelVisible;
        _columns.ColumnDefinitions[0].MinWidth = _explorerVisible ? 160 : 0;
        _columns.ColumnDefinitions[4].MinWidth = s.ChatPanelVisible ? 300 : 0;
        _columns.Children.Add(Kit.At(new Border { BorderBrush = Look.Brush(p.Seam), BorderThickness = new Thickness(0, 0, 1, 0), Child = _explorer, IsVisible = _explorerVisible }, 0, 0));
        _columns.Children.Add(Kit.At(leftSplit, 0, 1));
        _columns.Children.Add(Kit.At(_editor, 0, 2));
        _columns.Children.Add(Kit.At(rightSplit, 0, 3));
        _columns.Children.Add(Kit.At(new Border { BorderBrush = Look.Brush(p.Seam), BorderThickness = new Thickness(1, 0, 0, 0), Child = _chat, IsVisible = s.ChatPanelVisible }, 0, 4));

        _rows = new Grid { RowDefinitions = new RowDefinitions($"*,Auto,{(s.LogsPanelVisible ? 200 : 0)}") };
        var bottomSplit = Splitter(vertical: false);
        bottomSplit.IsVisible = s.LogsPanelVisible;
        _rows.Children.Add(Kit.At(_columns, 0, 0));
        _rows.Children.Add(Kit.At(bottomSplit, 1, 0));
        _logs.IsVisible = s.LogsPanelVisible;
        _rows.Children.Add(Kit.At(_logs, 2, 0));

        var root = new DockPanel();
        root.Children.Add(Kit.Dock(new StackPanel { Children = { BuildMenu(), BuildToolbar() } }, Avalonia.Controls.Dock.Top));
        root.Children.Add(Kit.Dock(_status, Avalonia.Controls.Dock.Bottom));
        root.Children.Add(_rows);
        Content = root;

        _editor.ShowLineNumbers = s.ShowLineNumbers;
        _editor.WordWrap = s.WordWrap;
        _editor.EditorFontSize = s.EditorFontSize;
        UpdateStatus();
        _status.SetBuild(_buildState, _buildColor);
        _status.SetJack(_chatVm.Activity);
    }

    private static GridSplitter Splitter(bool vertical)
    {
        var p = Look.P;
        return new GridSplitter
        {
            Background = Look.Brush(p.Background),
            Width = vertical ? 4 : double.NaN,
            Height = vertical ? double.NaN : 4,
            ResizeDirection = vertical ? GridResizeDirection.Columns : GridResizeDirection.Rows,
        };
    }

    private static void Detach(Control control)
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
    }

    private Control BuildMenu()
    {
        var p = Look.P;
        MenuItem Item(string header, Action action, string? gesture = null, string? icon = null)
        {
            var m = new MenuItem { Header = header };
            if (gesture is not null)
            {
                m.InputGesture = KeyGesture.Parse(gesture);
            }

            if (icon is not null)
            {
                m.Icon = Kit.Icon(icon, 14);
            }

            m.Click += (_, _) => action();
            return m;
        }

        var recent = new MenuItem { Header = "Open recent" };
        var recentItems = _host.Settings.RecentProjects.Where(Directory.Exists).Take(10).Select(path => (object)Item(path, () => OpenProject(path))).ToList();
        recent.ItemsSource = recentItems.Count > 0 ? recentItems : [new MenuItem { Header = "No recent projects", IsEnabled = false }];

        var file = new MenuItem
        {
            Header = "_File",
            ItemsSource = new object[]
            {
                Item("New project…", () => _ = NewProjectAsync("template"), "Ctrl+Shift+N", Icons.NewProject),
                Item("New blank project…", () => _ = NewProjectAsync("blank")),
                new Separator(),
                Item("Open project…", () => _ = OpenProjectDialogAsync(), "Ctrl+O", Icons.Open),
                Item("Open file…", () => _ = OpenFileDialogAsync(), "Ctrl+Shift+O", Icons.File),
                recent,
                new Separator(),
                Item("Save", () => _editor.Save(), "Ctrl+S", Icons.Save),
                Item("Save all", _editor.SaveAll, "Ctrl+Shift+S"),
                Item("Close file", () => { if (_editor.Active is { } d) { _editor.Close(d); } }, "Ctrl+W"),
                Item("Close project", () => _ = CloseProjectAsync()),
                new Separator(),
                Item("Settings…", () => _ = OpenSettingsAsync(), "Ctrl+OemComma", Icons.Settings),
                new Separator(),
                Item("Exit", Close, "Alt+F4", Icons.Exit),
            },
        };
        var edit = new MenuItem
        {
            Header = "_Edit",
            ItemsSource = new object[]
            {
                Item("Find / replace", _editor.OpenFind, "Ctrl+F", Icons.Search),
                Item("Go to line…", () => _ = GoToLineAsync(), "Ctrl+G", Icons.GoTo),
                Item("Format code", FormatCode, "Shift+Alt+F", Icons.Format),
            },
        };
        var view = new MenuItem
        {
            Header = "_View",
            ItemsSource = new object[]
            {
                Item("Explorer", () => SetExplorerVisible(!_explorerVisible), "Ctrl+B", Icons.Explorer),
                Item("Jack chat panel", () => SetChatVisible(!_host.Settings.ChatPanelVisible), "Ctrl+L", Icons.Chat),
                Item("Logs panel", () => SetLogsVisible(!_host.Settings.LogsPanelVisible), "Ctrl+J", Icons.Logs),
                new Separator(),
                Item(_host.Settings.ShowLineNumbers ? "Hide line numbers" : "Show line numbers", () => SetLineNumbers(!_host.Settings.ShowLineNumbers), null, Icons.Hash),
                Item(_host.Settings.WordWrap ? "Disable word wrap" : "Word wrap", () => SetWordWrap(!_host.Settings.WordWrap), "Alt+Z", Icons.Wrap),
                Item("Zoom in", () => SetFontSize(_host.Settings.EditorFontSize + 1), "Ctrl+OemPlus"),
                Item("Zoom out", () => SetFontSize(_host.Settings.EditorFontSize - 1), "Ctrl+OemMinus"),
                new Separator(),
                Item(Look.IsDark ? "Light theme" : "Dark theme", () => SetDark(!Look.IsDark), null, Look.IsDark ? Icons.Sun : Icons.Moon),
            },
        };
        var build = new MenuItem
        {
            Header = "_Build",
            ItemsSource = new object[]
            {
                Item("Build", () => _ = BuildAsync(), "Ctrl+Shift+B", Icons.Build),
                Item($"Run ({ActiveTarget.Name})", () => _ = RunAsync(), "F5", Icons.Run),
                Item("Run in Simulator", () => _ = RunAsync(FirstTarget(RunTargetKind.Simulator)), null, Icons.Sim),
                Item("Run on Robot", () => _ = RunAsync(FirstTarget(RunTargetKind.PhysicalRobot)), null, Icons.Robot),
                Item("Stop", StopRun, "Shift+F5", Icons.Stop),
                Item("Manage run targets…", () => _ = OpenSettingsAsync("Run targets"), null, Icons.Settings),
                new Separator(),
                Item("Deploy…", () => _ = DeployAsync(), null, Icons.Deploy),
            },
        };
        var jack = new MenuItem
        {
            Header = "_Jack",
            ItemsSource = new object[]
            {
                Item("Ask Jack", () => { SetChatVisible(true); _chat.FocusInput(); }, "Ctrl+I", Icons.Chat),
                Item("Explain the selection", () => _ = AskAboutSelectionAsync("Explain this code from {file}:"), null, Icons.Info),
                Item("Fix the last build errors", () => _ = FixBuildAsync(), null, Icons.Build),
                new Separator(),
                Item("Clear chat thread", _chatVm.Clear, null, Icons.Trash),
            },
        };
        var help = new MenuItem
        {
            Header = "_Help",
            ItemsSource = new object[]
            {
                Item("About ROS2.Net App Generator", () => _ = new AboutDialog().ShowDialog(this), null, Icons.Info),
            },
        };

        var menu = new Menu { ItemsSource = new[] { file, edit, view, build, jack, help }, Background = Brushes.Transparent };
        var brand = Kit.Row(8,
            new Border { Width = 18, Height = 18, CornerRadius = new CornerRadius(5), Background = Look.SignalGradient(), Child = Kit.Icon(Icons.Spark, 11, Colors.White, 2.4) },
            Kit.Text("ROS2AppGen", 12.5, FontWeight.Bold));
        brand.Margin = new Thickness(12, 0, 6, 0);
        var bar = new DockPanel { Background = Look.Brush(p.Surface), Height = 32 };
        bar.Children.Add(Kit.Dock(brand, Avalonia.Controls.Dock.Left));
        bar.Children.Add(menu);
        return bar;
    }

    private Control BuildToolbar()
    {
        var p = Look.P;
        Control Sep() => new Border { Width = 1, Height = 20, Background = Look.Brush(p.Seam), Margin = new Thickness(6, 0) };

        _runButton = Kit.Button(_host.Build.IsRunning ? "Stop" : "Run", () =>
        {
            if (_host.Build.IsRunning)
            {
                StopRun();
            }
            else
            {
                _ = RunAsync();
            }
        }, primary: true, icon: _host.Build.IsRunning ? Icons.Stop : Icons.Run, tip: "Run (F5) / Stop (Shift+F5)");

        var left = Kit.Row(2,
            Kit.IconButton(Icons.NewProject, "New project (Ctrl+Shift+N)", () => _ = NewProjectAsync("template")),
            Kit.IconButton(Icons.Open, "Open project (Ctrl+O)", () => _ = OpenProjectDialogAsync()),
            Kit.IconButton(Icons.Save, "Save (Ctrl+S)", () => _editor.Save()),
            Kit.IconButton(Icons.Close, "Close project", () => _ = CloseProjectAsync()),
            Sep(),
            Kit.Button("Build", () => _ = BuildAsync(), icon: Icons.Build, tip: "Build (Ctrl+Shift+B)"),
            _runButton,
            TargetPicker(),
            Kit.Button("Deploy", () => _ = DeployAsync(), icon: Icons.Deploy, tip: "Publish with dotnet publish"),
            Sep(),
            Kit.IconButton(Icons.Format, "Format code (Shift+Alt+F)", FormatCode),
            Kit.IconButton(Icons.GoTo, "Go to line (Ctrl+G)", () => _ = GoToLineAsync()),
            Kit.IconButton(Icons.Search, "Find (Ctrl+F)", _editor.OpenFind),
            Kit.IconButton(Icons.Hash, "Show/hide line numbers", () => SetLineNumbers(!_host.Settings.ShowLineNumbers), _host.Settings.ShowLineNumbers ? p.Telemetry : null),
            Kit.IconButton(Icons.Wrap, "Word wrap (Alt+Z)", () => SetWordWrap(!_host.Settings.WordWrap), _host.Settings.WordWrap ? p.Telemetry : null));

        var right = Kit.Row(2,
            Kit.IconButton(Icons.Explorer, "Explorer (Ctrl+B)", () => SetExplorerVisible(!_explorerVisible), _explorerVisible ? p.Telemetry : null),
            Kit.IconButton(Icons.Logs, "Logs (Ctrl+J)", () => SetLogsVisible(!_host.Settings.LogsPanelVisible), _host.Settings.LogsPanelVisible ? p.Telemetry : null),
            Kit.IconButton(Icons.Chat, "Jack (Ctrl+L)", () => SetChatVisible(!_host.Settings.ChatPanelVisible), _host.Settings.ChatPanelVisible ? p.Telemetry : null),
            Sep(),
            Kit.IconButton(Look.IsDark ? Icons.Sun : Icons.Moon, Look.IsDark ? "Light theme" : "Dark theme", () => SetDark(!Look.IsDark)),
            Kit.IconButton(Icons.Settings, "Settings (Ctrl+,)", () => _ = OpenSettingsAsync()));

        var bar = new DockPanel { Margin = new Thickness(8, 5, 8, 5) };
        bar.Children.Add(Kit.Dock(right, Avalonia.Controls.Dock.Right));
        bar.Children.Add(left);
        return new Border
        {
            Background = Look.Brush(p.Surface),
            BorderBrush = Look.Brush(p.Seam),
            BorderThickness = new Thickness(0, 1, 0, 1),
            Child = bar,
        };
    }

    private Control Welcome()
    {
        var p = Look.P;
        var recent = new StackPanel { Spacing = 4 };
        foreach (var path in _host.Settings.RecentProjects.Where(Directory.Exists).Take(6))
        {
            var b = new Button
            {
                Background = Brushes.Transparent,
                BorderThickness = new Thickness(0),
                Padding = new Thickness(0, 3),
                Content = Kit.Row(8, Kit.Icon(Icons.Folder, 13, p.Telemetry), Kit.Text(Path.GetFileName(path), 13, FontWeight.SemiBold), Kit.Muted(path, 11.5)),
            };
            b.Click += (_, _) => OpenProject(path);
            recent.Children.Add(b);
        }

        if (recent.Children.Count == 0)
        {
            recent.Children.Add(Kit.Muted("Projects you open appear here.", 12.5));
        }

        var headline = new TextBlock
        {
            FontSize = 38,
            FontWeight = FontWeight.Bold,
            LetterSpacing = -0.6,
            TextWrapping = TextWrapping.Wrap,
            Foreground = Look.Brush(p.Ink),
            Inlines =
            [
                new Avalonia.Controls.Documents.Run("Describe the robot."),
                new Avalonia.Controls.Documents.LineBreak(),
                new Avalonia.Controls.Documents.Run("Jack bends the code.") { Foreground = Look.SignalGradient() },
            ],
        };

        var actions = Kit.Row(10,
            Kit.Button("New project", () => _ = NewProjectAsync("template"), primary: true, icon: Icons.NewProject),
            Kit.Button("Open project", () => _ = OpenProjectDialogAsync(), icon: Icons.Open),
            Kit.Button("Ask Jack", () => { SetChatVisible(true); _chat.FocusInput(); }, icon: Icons.Chat));

        var shortcuts = new Grid { ColumnDefinitions = new ColumnDefinitions("Auto,*"), ColumnSpacing = 18, RowSpacing = 6 };
        var keys = new (string Key, string What)[]
        {
            ("Ctrl+Shift+N", "New project from a template"),
            ("Ctrl+L", "Talk to Jack · Ctrl+Enter sends"),
            ("Ctrl+Shift+B / F5", "Build / run"),
            ("Ctrl+G · Shift+Alt+F", "Go to line · format code"),
        };
        for (var i = 0; i < keys.Length; i++)
        {
            shortcuts.RowDefinitions.Add(new RowDefinition(GridLength.Auto));
            shortcuts.Children.Add(Kit.At(Kit.Text(keys[i].Key, 12, color: p.Telemetry, mono: true), i, 0));
            shortcuts.Children.Add(Kit.At(Kit.Muted(keys[i].What, 12.5), i, 1));
        }

        var col = Kit.Col(22,
            Kit.Eyebrow("ROS2.Net App Generator"),
            headline,
            Kit.Muted("Start from a template — warehouse fleets, lab instruments, safety monitors, classroom robots — or a blank project, and let Jack write the nodes, UI and simulation with ROS2.Net.", 14, wrap: true),
            actions,
            new Grid
            {
                ColumnDefinitions = new ColumnDefinitions("*,*"),
                ColumnSpacing = 32,
                Children =
                {
                    Kit.At(Kit.Col(10, Kit.Eyebrow("Recent"), recent), 0, 0),
                    Kit.At(Kit.Col(10, Kit.Eyebrow("Shortcuts"), shortcuts), 0, 1),
                },
            });
        col.MaxWidth = 760;
        col.Margin = new Thickness(56, 48, 40, 40);
        col.HorizontalAlignment = HorizontalAlignment.Left;
        return new ScrollViewer { Content = col, Background = Look.Brush(p.Background) };
    }

    // ------------------------------------------------------------------ panels & view

    public void SetChatVisible(bool visible)
    {
        if (!visible && _columns.ColumnDefinitions[4].ActualWidth > 0)
        {
            _host.Settings.ChatPanelWidth = _columns.ColumnDefinitions[4].ActualWidth;
        }

        _host.Settings.ChatPanelVisible = visible;
        Build();
        if (visible)
        {
            _chat.FocusInput();
        }
    }

    public void SetLogsVisible(bool visible)
    {
        _host.Settings.LogsPanelVisible = visible;
        Build();
    }

    public void SetExplorerVisible(bool visible)
    {
        if (!visible && _columns.ColumnDefinitions[0].ActualWidth > 0)
        {
            _host.Settings.ExplorerWidth = _columns.ColumnDefinitions[0].ActualWidth;
        }

        _explorerVisible = visible;
        Build();
    }

    public void SetLineNumbers(bool show)
    {
        _host.Settings.ShowLineNumbers = show;
        _editor.ShowLineNumbers = show;
        Build();
    }

    public void SetWordWrap(bool wrap)
    {
        _host.Settings.WordWrap = wrap;
        _editor.WordWrap = wrap;
        Build();
    }

    public void SetFontSize(double size)
    {
        _host.Settings.EditorFontSize = Math.Clamp(size, 8, 32);
        _editor.EditorFontSize = _host.Settings.EditorFontSize;
    }

    public void SetDark(bool dark)
    {
        CaptureLayout();
        Look.IsDark = dark;
        _host.Settings.Theme = dark ? "Dark" : "Light";
        ApplyFluentVariant();
        _editor.ApplyTheme();
        _logs.ApplyTheme();
        _themeChanged = true;
        Build();
        _host.SaveSettings();
    }

    private void ApplyFluentVariant()
    {
        if (Application.Current is { } app)
        {
            app.RequestedThemeVariant = Look.IsDark ? ThemeVariant.Dark : ThemeVariant.Light;
        }

        RequestedThemeVariant = Look.IsDark ? ThemeVariant.Dark : ThemeVariant.Light;
    }

    private void CaptureLayout()
    {
        if (_columns is null)
        {
            return;
        }

        if (_explorerVisible && _columns.ColumnDefinitions[0].ActualWidth > 0)
        {
            _host.Settings.ExplorerWidth = Math.Round(_columns.ColumnDefinitions[0].ActualWidth);
        }

        if (_host.Settings.ChatPanelVisible && _columns.ColumnDefinitions[4].ActualWidth > 0)
        {
            _host.Settings.ChatPanelWidth = Math.Round(_columns.ColumnDefinitions[4].ActualWidth);
        }
    }

    private void UpdateStatus()
    {
        var root = _host.Workspace.RootPath;
        _status.SetProject(root is null ? null : Path.GetFileName(root.TrimEnd('/', '\\')));
        _status.SetFile(_editor.Active?.FileName, _editor.Active?.IsDirty == true);
        if (_editor.Active is null)
        {
            _status.SetCursor(0, 0);
        }

        _status.SetTarget(ActiveTarget.Name, ActiveTarget.Kind == RunTargetKind.PhysicalRobot);
        var profile = _host.Settings.Profiles.FirstOrDefault(x => x.Name == _host.Settings.ActiveProfile);
        _status.SetModel(profile is null ? null : $"{profile.Name}");
        Title = root is null ? "ROS2.Net App Generator" : $"{Path.GetFileName(root.TrimEnd('/', '\\'))} — ROS2.Net App Generator";
    }

    private void SetBuildState(string text, Color? color)
    {
        _buildState = text;
        _buildColor = color;
        _status.SetBuild(text, color);
    }

    // ------------------------------------------------------------------ project commands

    public async Task NewProjectAsync(string mode)
    {
        var folder = await new NewProjectDialog(_host, mode).ShowDialog<string?>(this);
        if (folder is not null)
        {
            _host.SaveSettings();
            _logs.WriteOutput($"Created project {folder}");
            OpenProject(folder);
        }
    }

    /// <summary>Opens a project folder (or the folder of a .csproj/.slnx) and shows its main file.</summary>
    public void OpenProject(string path)
    {
        if (_editor.HasUnsavedChanges)
        {
            _editor.SaveAll();
        }

        try
        {
            _editor.CloseAll();
            _host.Workspace.Open(path);
        }
        catch (Exception ex)
        {
            _logs.WriteOutput($"Could not open {path}: {ex.Message}");
            return;
        }

        var root = _host.Workspace.RootPath!;
        var recent = _host.Settings.RecentProjects;
        recent.RemoveAll(r => string.Equals(r, root, StringComparison.OrdinalIgnoreCase));
        recent.Insert(0, root);
        if (recent.Count > 12)
        {
            recent.RemoveRange(12, recent.Count - 12);
        }

        _host.SaveSettings();
        _logs.WriteOutput($"Opened {root}");

        // Show the entry point, the way IDEs land you in Program.cs.
        var files = _host.Workspace.ListFiles();
        var main = files.FirstOrDefault(f => f.EndsWith("Program.cs", StringComparison.OrdinalIgnoreCase))
                   ?? files.FirstOrDefault(f => f.EndsWith(".cs", StringComparison.OrdinalIgnoreCase))
                   ?? files.FirstOrDefault(f => f.EndsWith("README.md", StringComparison.OrdinalIgnoreCase));
        Build();
        if (main is not null)
        {
            var full = Path.Combine(root, main);
            _editor.Open(full);
            _explorer.Reveal(full);
        }
    }

    private async Task OpenProjectDialogAsync()
    {
        var folders = await StorageProvider.OpenFolderPickerAsync(new FolderPickerOpenOptions
        {
            Title = "Open project folder",
            SuggestedStartLocation = Directory.Exists(_host.Settings.ProjectsFolder) ? await StorageProvider.TryGetFolderFromPathAsync(_host.Settings.ProjectsFolder) : null,
        });
        if (folders.FirstOrDefault()?.TryGetLocalPath() is { } path)
        {
            OpenProject(path);
        }
    }

    private async Task OpenFileDialogAsync()
    {
        var files = await StorageProvider.OpenFilePickerAsync(new FilePickerOpenOptions { Title = "Open file", AllowMultiple = true });
        foreach (var f in files)
        {
            if (f.TryGetLocalPath() is not { } path)
            {
                continue;
            }

            var ext = Path.GetExtension(path).ToLowerInvariant();
            if (_host.Workspace.RootPath is null && ext is ".csproj" or ".slnx" or ".sln")
            {
                OpenProject(path);
                return;
            }

            _editor.Open(path);
        }
    }

    public async Task CloseProjectAsync()
    {
        if (_editor.HasUnsavedChanges)
        {
            var answer = await ConfirmDialog.AskAsync(this, "Save changes?", "Some files have unsaved changes.", "Save all", "Discard");
            if (answer == "cancel")
            {
                return;
            }

            if (answer == "confirm")
            {
                _editor.SaveAll();
            }
        }

        StopRun();
        _editor.CloseAll();
        _host.Workspace.Close();
        _logs.WriteOutput("Project closed.");
        Build();
    }

    private async Task GoToLineAsync()
    {
        if (_editor.Active is null)
        {
            return;
        }

        var line = await new GoToLineDialog(_editor.Editor.TextArea.Caret.Line, _editor.Editor.Document.LineCount).ShowDialog<int?>(this);
        if (line is { } l)
        {
            _editor.GoToLine(l);
        }
    }

    public void FormatCode()
    {
        if (_editor.Active is null)
        {
            return;
        }

        try
        {
            if (!_editor.Format())
            {
                _logs.WriteOutput($"No formatter for {_editor.Active.FileName}.");
            }
        }
        catch (Exception ex)
        {
            _logs.WriteOutput($"Format failed: {ex.Message}");
        }
    }

    private async Task OpenSettingsAsync(string section = "Models")
    {
        CaptureLayout();
        if (await new SettingsDialog(_host, section).ShowDialog<bool>(this))
        {
            var s = _host.Settings;
            var dark = !string.Equals(s.Theme, "Light", StringComparison.OrdinalIgnoreCase);
            if (dark != Look.IsDark)
            {
                Look.IsDark = dark;
                ApplyFluentVariant();
                _editor.ApplyTheme();
                _logs.ApplyTheme();
                _themeChanged = true;
            }

            if (s.ActiveProfile.Length > 0)
            {
                try
                {
                    _host.Assistant.UseProfile(s.ActiveProfile);
                }
                catch (Exception ex)
                {
                    _logs.WriteJack("Could not switch model: " + ex.Message);
                }
            }

            Build();
            _logs.WriteOutput("Settings saved.");
        }
    }

    // ------------------------------------------------------------------ build / run / deploy

    private string? RequireProject()
    {
        if (_host.Workspace.RootPath is { } root)
        {
            return root;
        }

        _logs.WriteOutput("Open or create a project first.");
        return null;
    }

    public async Task<ProcessResult?> BuildAsync()
    {
        if (RequireProject() is not { } root)
        {
            return null;
        }

        _editor.SaveAll();
        SetLogsVisible(true);
        _logs.Select("output");
        SetBuildState("Building…", Look.P.Warn);
        _logs.WriteOutput($"> dotnet build {Path.GetFileName(root)}");
        _buildCts = new CancellationTokenSource();
        var errors = new List<string>();
        try
        {
            var result = await _host.Build.BuildAsync(root, o =>
            {
                _logs.WriteOutput(o.Line);
                if (o.IsError || o.Line.Contains(": error ", StringComparison.Ordinal))
                {
                    lock (errors)
                    {
                        errors.Add(o.Line);
                    }
                }
            }, _buildCts.Token);
            _lastBuildErrors = errors.Count > 0 ? string.Join("\n", errors.Distinct().Take(40)) : null;
            SetBuildState(result.Success ? $"Build succeeded · {result.Duration.TotalSeconds:0.0}s" : "Build failed", result.Success ? Look.P.Ok : Look.P.Fault);
            _logs.WriteOutput(result.Success ? "Build succeeded." : "Build failed. Jack → Fix the last build errors can repair it.");
            return result;
        }
        catch (Exception ex)
        {
            SetBuildState("Build error", Look.P.Fault);
            _logs.WriteOutput("Build could not start: " + ex.Message);
            return null;
        }
    }

    /// <summary>
    /// Asked before running on a physical robot (once per target per session). Returns true to proceed.
    /// Tests replace it; the app shows <see cref="RobotSafetyDialog"/>.
    /// </summary>
    public Func<RunTarget, Task<bool>> ConfirmRobotRun { get; set; }

    /// <summary>The run target Run uses (the settings' active target, else the first, else the in-process simulator).</summary>
    public RunTarget ActiveTarget =>
        _host.Settings.RunTargets.FirstOrDefault(t => t.Name == _host.Settings.ActiveRunTarget)
        ?? _host.Settings.RunTargets.FirstOrDefault()
        ?? RunTargetEnvironment.DefaultSimulator;

    public void SetActiveTarget(string name)
    {
        _host.Settings.ActiveRunTarget = name;
        _host.SaveSettings();
        UpdateStatus();
    }

    private RunTarget? FirstTarget(RunTargetKind kind) =>
        ActiveTarget.Kind == kind ? ActiveTarget : _host.Settings.RunTargets.FirstOrDefault(t => t.Kind == kind);

    private Control TargetPicker()
    {
        var p = Look.P;
        var combo = new ComboBox
        {
            ItemsSource = _host.Settings.RunTargets,
            SelectedItem = ActiveTarget,
            MinWidth = 230,
            MaxWidth = 330,
            FontSize = 12,
            VerticalAlignment = VerticalAlignment.Center,
            ItemTemplate = new Avalonia.Controls.Templates.FuncDataTemplate<RunTarget>((t, _) => t is null
                ? new TextBlock()
                : Kit.Row(7,
                    Kit.Icon(t.Kind == RunTargetKind.PhysicalRobot ? Icons.Robot : Icons.Sim, 14, t.Kind == RunTargetKind.PhysicalRobot ? p.Warn : p.Telemetry, 2),
                    Kit.Text(t.Name, 12))),
        };
        ToolTip.SetTip(combo, "Run target: simulator or physical robot (Build → Manage run targets…)");
        combo.SelectionChanged += (_, _) =>
        {
            if (combo.SelectedItem is RunTarget t && t.Name != _host.Settings.ActiveRunTarget)
            {
                SetActiveTarget(t.Name);
                Dispatcher.UIThread.Post(Build);
            }
        };
        return combo;
    }

    /// <summary>Runs the app on <paramref name="target"/> (default: the active target).</summary>
    public async Task RunAsync(RunTarget? target = null)
    {
        if (RequireProject() is not { } root || _host.Build.IsRunning)
        {
            return;
        }

        target ??= ActiveTarget;
        var robot = target.Kind == RunTargetKind.PhysicalRobot;
        if (robot && !_robotRunsConfirmed.Contains(target.Name))
        {
            if (!await ConfirmRobotRun(target))
            {
                _logs.WriteOutput($"Run on {target.Name} cancelled.");
                return;
            }

            _robotRunsConfirmed.Add(target.Name);
        }

        _editor.SaveAll();
        SetLogsVisible(true);
        _logs.Select("output");
        SetBuildState(robot ? "Running on robot" : "Running in simulator", robot ? Look.P.Warn : Look.P.Telemetry);
        _logs.WriteOutput($"> dotnet run {Path.GetFileName(root)}");
        _logs.WriteOutput($"Run target: {target.Name} ({(robot ? "physical robot" : "simulator")})");
        foreach (var (key, value) in RunTargetEnvironment.ToEnvironment(target).OrderBy(kv => kv.Key, StringComparer.Ordinal))
        {
            _logs.WriteOutput($"  {key}={MaskSecret(key, value)}");
        }

        Build();
        try
        {
            var result = await _host.Build.RunAsync(root, target, o => _logs.WriteOutput(o.Line), CancellationToken.None);
            _logs.WriteOutput($"App exited with code {result.ExitCode} after {result.Duration.TotalSeconds:0.0}s.");
            SetBuildState(result.Success ? "Ready" : $"Exited ({result.ExitCode})", result.Success ? null : Look.P.Warn);
        }
        catch (Exception ex)
        {
            _logs.WriteOutput("Run failed: " + ex.Message);
            SetBuildState("Run failed", Look.P.Fault);
        }
        finally
        {
            Build();
        }
    }

    /// <summary>Hides values of variables that look like credentials.</summary>
    internal static string MaskSecret(string key, string value)
    {
        var k = key.ToUpperInvariant();
        return k.Contains("KEY") || k.Contains("TOKEN") || k.Contains("SECRET") || k.Contains("PASS")
            ? (value.Length == 0 ? string.Empty : "••••")
            : value;
    }

    public void StopRun()
    {
        if (_host.Build.IsRunning)
        {
            _host.Build.StopRun();
            _logs.WriteOutput("Stopping the app…");
        }

        _buildCts?.Cancel();
    }

    private async Task DeployAsync()
    {
        if (RequireProject() is not { } root)
        {
            return;
        }

        var options = await new DeployDialog(root).ShowDialog<DeployOptions?>(this);
        if (options is null)
        {
            return;
        }

        _editor.SaveAll();
        SetLogsVisible(true);
        _logs.Select("output");
        SetBuildState($"Deploying {options.Runtime}…", Look.P.Warn);
        _logs.WriteOutput($"> dotnet publish -r {options.Runtime} -o {options.OutputFolder}{(options.SelfContained ? " --self-contained" : string.Empty)}");
        try
        {
            var result = await _host.Build.DeployAsync(root, options.OutputFolder, options.Runtime, options.SelfContained, o => _logs.WriteOutput(o.Line), CancellationToken.None);
            SetBuildState(result.Success ? "Deployed" : "Deploy failed", result.Success ? Look.P.Ok : Look.P.Fault);
            _logs.WriteOutput(result.Success ? $"Published to {options.OutputFolder}" : "Deploy failed.");
        }
        catch (Exception ex)
        {
            SetBuildState("Deploy failed", Look.P.Fault);
            _logs.WriteOutput("Deploy failed: " + ex.Message);
        }
    }

    // ------------------------------------------------------------------ Jack shortcuts

    private async Task AskAboutSelectionAsync(string template)
    {
        var selection = _editor.Editor.SelectedText;
        if (string.IsNullOrWhiteSpace(selection))
        {
            _logs.WriteOutput("Select some code first.");
            return;
        }

        SetChatVisible(true);
        var file = _editor.Active?.FileName ?? "the editor";
        await _chatVm.SendAsync($"{template.Replace("{file}", file)}\n\n```\n{selection}\n```");
    }

    private async Task FixBuildAsync()
    {
        if (_lastBuildErrors is null)
        {
            _logs.WriteOutput("There are no build errors to fix. Build the project first.");
            return;
        }

        SetChatVisible(true);
        await _chatVm.SendAsync($"The build failed with these errors. Fix the code, then build again to confirm:\n\n```\n{_lastBuildErrors}\n```");
    }

    // ------------------------------------------------------------------ keys & lifetime

    private void OnKeyDown(object? sender, KeyEventArgs e)
    {
        var ctrl = e.KeyModifiers.HasFlag(KeyModifiers.Control);
        var shift = e.KeyModifiers.HasFlag(KeyModifiers.Shift);
        var alt = e.KeyModifiers.HasFlag(KeyModifiers.Alt);
        Action? action = (e.Key, ctrl, shift, alt) switch
        {
            (Key.N, true, true, false) => () => _ = NewProjectAsync("template"),
            (Key.O, true, false, false) => () => _ = OpenProjectDialogAsync(),
            (Key.O, true, true, false) => () => _ = OpenFileDialogAsync(),
            (Key.S, true, false, false) => () => _editor.Save(),
            (Key.S, true, true, false) => _editor.SaveAll,
            (Key.W, true, false, false) => () => { if (_editor.Active is { } d) { _editor.Close(d); } },
            (Key.G, true, false, false) => () => _ = GoToLineAsync(),
            (Key.F, false, true, true) => FormatCode,
            (Key.B, true, true, false) => () => _ = BuildAsync(),
            (Key.B, true, false, false) => () => SetExplorerVisible(!_explorerVisible),
            (Key.J, true, false, false) => () => SetLogsVisible(!_host.Settings.LogsPanelVisible),
            (Key.L, true, false, false) => () => SetChatVisible(!_host.Settings.ChatPanelVisible),
            (Key.I, true, false, false) => () => { SetChatVisible(true); _chat.FocusInput(); },
            (Key.OemComma, true, false, false) => () => _ = OpenSettingsAsync(),
            (Key.F5, false, false, false) => () => _ = RunAsync(),
            (Key.F5, false, true, false) => StopRun,
            (Key.Z, false, false, true) => () => SetWordWrap(!_host.Settings.WordWrap),
            (Key.OemPlus or Key.Add, true, false, false) => () => SetFontSize(_host.Settings.EditorFontSize + 1),
            (Key.OemMinus or Key.Subtract, true, false, false) => () => SetFontSize(_host.Settings.EditorFontSize - 1),
            _ => null,
        };
        if (action is not null)
        {
            e.Handled = true;
            action();
        }
    }

    private async void OnClosing(object? sender, WindowClosingEventArgs e)
    {
        if (_closingConfirmed)
        {
            return;
        }

        if (_editor.HasUnsavedChanges)
        {
            e.Cancel = true;
            var answer = await ConfirmDialog.AskAsync(this, "Save changes before exiting?", "Some files have unsaved changes.", "Save all", "Discard");
            if (answer == "cancel")
            {
                return;
            }

            if (answer == "confirm")
            {
                _editor.SaveAll();
            }

            _closingConfirmed = true;
            PersistOnExit();
            Close();
            return;
        }

        PersistOnExit();
    }

    /// <summary>Saves layout and editor preferences to App.config.</summary>
    public void PersistOnExit()
    {
        CaptureLayout();
        _host.Settings.EditorFontSize = _editor.EditorFontSize;
        _chatVm.Stop();
        StopRun();
        try
        {
            _host.SaveSettings();
        }
        catch (Exception)
        {
        }
    }
}
