using System.Globalization;
using System.Text.Json;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Primitives;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Platform.Storage;
using ROS2.Net.AppGen;
using Ros2AppGen.Ui;

namespace Ros2AppGen.Views;

/// <summary>
/// Every setting in <see cref="AppGenSettings"/>, grouped into sections. Edits apply to a copy and are
/// written to App.config on Save.
/// </summary>
public sealed class SettingsDialog : AppDialog
{
    private static readonly string[] s_sections = ["Models", "Run targets", "Jack", "Web tools", "Projects", "Editor", "Layout"];

    /// <summary>Simulation scenarios (ROS2.Net.Simulation.Scenarios); empty = the app's default.</summary>
    public static readonly string[] Scenarios = ["", "Empty", "Warehouse", "Office", "Maze", "CorridorLoop", "AgricultureRows"];

    private readonly AppGenHost _host;
    private readonly AppGenSettings _draft;
    private readonly StackPanel _nav = new() { Spacing = 2 };
    private readonly ContentControl _page = new();
    private string _section;
    private LlmProfile? _profile;
    private RunTarget? _target;
    private TextBlock? _pingResult;

    public SettingsDialog(AppGenHost host, string section = "Models")
        : base("Settings", $"Saved to {host.SettingsStore.ConfigPath}", 980, 680)
    {
        _host = host;
        _draft = Clone(host.Settings);
        _section = section;
        _profile = _draft.Profiles.FirstOrDefault(p => p.Name == _draft.ActiveProfile) ?? _draft.Profiles.FirstOrDefault();
        if (_draft.RunTargets.Count == 0)
        {
            _draft.RunTargets = RunTargetEnvironment.Defaults();
        }

        _target = _draft.RunTargets.FirstOrDefault(t => t.Name == _draft.ActiveRunTarget) ?? _draft.RunTargets.FirstOrDefault();

        var grid = new Grid { ColumnDefinitions = new ColumnDefinitions("170,*"), ColumnSpacing = 18 };
        grid.Children.Add(Kit.At(_nav, 0, 0));
        grid.Children.Add(Kit.At(new ScrollViewer { Content = _page, HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Disabled }, 0, 1));
        SetBody(grid);

        AddButton("Cancel", () => Close(false));
        AddButton("Save", Save, primary: true);
        Render();
    }

    /// <summary>The edited copy (tests).</summary>
    public AppGenSettings Draft => _draft;

    public void ShowSection(string section)
    {
        _section = section;
        Render();
    }

    private static AppGenSettings Clone(AppGenSettings s) =>
        JsonSerializer.Deserialize<AppGenSettings>(JsonSerializer.Serialize(s)) ?? new AppGenSettings();

    private void Save()
    {
        if (_draft.Profiles.Count > 0 && _draft.Profiles.All(p => p.Name != _draft.ActiveProfile))
        {
            _draft.ActiveProfile = _draft.Profiles[0].Name;
        }

        if (_draft.RunTargets.Count > 0 && _draft.RunTargets.All(t => t.Name != _draft.ActiveRunTarget))
        {
            _draft.ActiveRunTarget = _draft.RunTargets[0].Name;
        }

        _host.Settings = _draft;
        _host.SaveSettings();
        Close(true);
    }

    private void Render()
    {
        var p = Look.P;
        _nav.Children.Clear();
        foreach (var s in s_sections)
        {
            var active = s == _section;
            var b = new Button
            {
                HorizontalAlignment = HorizontalAlignment.Stretch,
                HorizontalContentAlignment = HorizontalAlignment.Left,
                Background = active ? Look.Brush(p.Signal, 0.16) : Brushes.Transparent,
                BorderThickness = new Thickness(2, 0, 0, 0),
                BorderBrush = active ? Look.Brush(p.Signal) : Brushes.Transparent,
                CornerRadius = new CornerRadius(0, 6, 6, 0),
                Padding = new Thickness(12, 8),
                Content = Kit.Text(s, 13, active ? FontWeight.SemiBold : FontWeight.Normal, active ? p.Ink : p.Haze),
            };
            b.Click += (_, _) =>
            {
                _section = s;
                Render();
            };
            _nav.Children.Add(b);
        }

        _page.Content = _section switch
        {
            "Run targets" => TargetsPage(),
            "Jack" => JackPage(),
            "Web tools" => WebPage(),
            "Projects" => ProjectsPage(),
            "Editor" => EditorPage(),
            "Layout" => LayoutPage(),
            _ => ModelsPage(),
        };
    }

    // ------------------------------------------------------------------ models

    private Control ModelsPage()
    {
        var p = Look.P;
        var list = new StackPanel { Spacing = 4 };
        foreach (var prof in _draft.Profiles)
        {
            var selected = prof == _profile;
            var isActive = prof.Name == _draft.ActiveProfile;
            var item = new Button
            {
                HorizontalAlignment = HorizontalAlignment.Stretch,
                HorizontalContentAlignment = HorizontalAlignment.Stretch,
                Background = Look.Brush(selected ? p.Raised : p.Surface),
                BorderBrush = Look.Brush(selected ? p.Signal : p.Seam),
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(8),
                Padding = new Thickness(10, 8),
                Content = Kit.Col(2,
                    Kit.Row(6, Kit.Text(prof.Name.Length == 0 ? "(unnamed)" : prof.Name, 12.5, FontWeight.SemiBold), isActive ? Kit.Chip("active", p.Ok) : new Border()),
                    Kit.Text($"{prof.Provider} · {prof.Model}", 11, color: p.Haze, mono: true)),
            };
            item.Click += (_, _) =>
            {
                _profile = prof;
                Render();
            };
            list.Children.Add(item);
        }

        var add = Kit.Button("Add model", () =>
        {
            var prof = new LlmProfile { Name = $"Model {_draft.Profiles.Count + 1}", Provider = LlmProviderKind.OpenAI, Model = "gpt-5-mini" };
            _draft.Profiles.Add(prof);
            _profile = prof;
            Render();
        }, icon: Icons.Plus);

        var left = Kit.Col(10, Kit.Eyebrow("Models"), list, add);
        var grid = new Grid { ColumnDefinitions = new ColumnDefinitions("250,*"), ColumnSpacing = 18 };
        grid.Children.Add(Kit.At(left, 0, 0));
        grid.Children.Add(Kit.At(_profile is null ? Kit.Muted("Add a model to let Jack work. Keys are stored only in App.config on this machine.", 12.5, wrap: true) : ProfileForm(_profile), 0, 1));
        return grid;
    }

    private Control ProfileForm(LlmProfile prof)
    {
        var p = Look.P;
        var name = Kit.Input(prof.Name);
        name.TextChanged += (_, _) =>
        {
            var wasActive = _draft.ActiveProfile == prof.Name;
            prof.Name = name.Text ?? string.Empty;
            if (wasActive)
            {
                _draft.ActiveProfile = prof.Name;
            }
        };

        var provider = new ComboBox { ItemsSource = Enum.GetValues<LlmProviderKind>(), SelectedItem = prof.Provider, MinWidth = 200 };
        var endpointHint = Kit.Muted(DefaultEndpoint(prof.Provider), 11, wrap: true);
        provider.SelectionChanged += (_, _) =>
        {
            if (provider.SelectedItem is LlmProviderKind k)
            {
                prof.Provider = k;
                endpointHint.Text = DefaultEndpoint(k);
            }
        };

        var model = Kit.Input(prof.Model, "e.g. claude-sonnet-5-5, gpt-5-mini, llama3.2", mono: true);
        model.TextChanged += (_, _) => prof.Model = model.Text ?? string.Empty;

        var key = Kit.Input(prof.ApiKey, "API key", password: true, mono: true);
        key.TextChanged += (_, _) => prof.ApiKey = key.Text ?? string.Empty;
        var reveal = new ToggleButton { Content = Kit.Text("Show", 11.5), Padding = new Thickness(10, 5), Margin = new Thickness(6, 0, 0, 0) };
        reveal.IsCheckedChanged += (_, _) => key.PasswordChar = reveal.IsChecked == true ? default : '•';
        var keyRow = new DockPanel { Children = { Kit.Dock(reveal, Avalonia.Controls.Dock.Right), key } };

        var endpoint = Kit.Input(prof.Endpoint, "Leave empty for the provider default", mono: true);
        endpoint.TextChanged += (_, _) => prof.Endpoint = endpoint.Text ?? string.Empty;

        var headers = new TextBox
        {
            Text = prof.ExtraHeaders,
            AcceptsReturn = true,
            TextWrapping = TextWrapping.Wrap,
            Height = 56,
            FontFamily = Look.Mono,
            FontSize = 12,
            PlaceholderText = "anthropic-workspace-id: wrkspc_…",
        };
        headers.TextChanged += (_, _) => prof.ExtraHeaders = headers.Text ?? string.Empty;

        var tempValue = Kit.Text(prof.Temperature.ToString("0.00", CultureInfo.InvariantCulture), 12, mono: true);
        var temperature = new Slider { Minimum = 0, Maximum = 2, Value = prof.Temperature, SmallChange = 0.05, Width = 260 };
        temperature.ValueChanged += (_, e) =>
        {
            prof.Temperature = Math.Round(e.NewValue, 2);
            tempValue.Text = prof.Temperature.ToString("0.00", CultureInfo.InvariantCulture);
        };

        var maxTokens = new NumericUpDown { Value = prof.MaxTokens, Minimum = 0, Maximum = 200000, Increment = 512, FormatString = "0", Width = 160 };
        maxTokens.ValueChanged += (_, _) => prof.MaxTokens = (int)(maxTokens.Value ?? 0);

        var vision = new CheckBox { Content = Kit.Text("Accepts images", 12.5), IsChecked = prof.SupportsVision };
        vision.IsCheckedChanged += (_, _) => prof.SupportsVision = vision.IsChecked == true;

        _pingResult = Kit.Text(string.Empty, 12, color: p.Haze, wrap: true);
        var actions = Kit.Row(8,
            Kit.Button("Use for Jack", () =>
            {
                _draft.ActiveProfile = prof.Name;
                Render();
            }, primary: prof.Name != _draft.ActiveProfile, icon: Icons.Check),
            Kit.Button("Test connection", () => _ = PingAsync(prof), icon: Icons.Spark),
            Kit.Button("Remove", () =>
            {
                _draft.Profiles.Remove(prof);
                _profile = _draft.Profiles.FirstOrDefault();
                Render();
            }, icon: Icons.Trash));

        var twoCol = new Grid { ColumnDefinitions = new ColumnDefinitions("*,*"), ColumnSpacing = 14, RowDefinitions = new RowDefinitions("Auto,Auto"), RowSpacing = 14 };
        twoCol.Children.Add(Kit.At(Kit.Field("Display name", name), 0, 0));
        twoCol.Children.Add(Kit.At(Kit.Field("Provider", provider), 0, 1));
        twoCol.Children.Add(Kit.At(Kit.Field("Model", model), 1, 0));
        twoCol.Children.Add(Kit.At(Kit.Field("Max output tokens", maxTokens, "0 = provider default"), 1, 1));

        return Kit.Col(16,
            twoCol,
            Kit.Field("API key", keyRow, prof.Provider == LlmProviderKind.Ollama ? "Not needed for a local Ollama server." : "Stored in App.config on this computer only."),
            Kit.Field("Endpoint", Kit.Col(4, endpoint, endpointHint)),
            Kit.Field("Extra HTTP headers", headers, "One 'Name: value' per line. Values may use env:VARIABLE."),
            Kit.Field("Temperature", Kit.Row(12, temperature, tempValue), "Lower is more deterministic. 0.2 suits code generation."),
            vision,
            actions,
            _pingResult);
    }

    private static string DefaultEndpoint(LlmProviderKind kind) => kind switch
    {
        LlmProviderKind.OpenAI => "Default: https://api.openai.com/v1",
        LlmProviderKind.AzureOpenAI => "Required: https://<resource>.openai.azure.com/ (the model field is the deployment name)",
        LlmProviderKind.Claude => "Default: https://api.anthropic.com",
        LlmProviderKind.Gemini => "Default: Google AI Studio (generativelanguage.googleapis.com)",
        LlmProviderKind.Ollama => "Default: http://localhost:11434",
        LlmProviderKind.DeepSeek => "Default: https://api.deepseek.com",
        _ => string.Empty,
    };

    private async Task PingAsync(LlmProfile prof)
    {
        if (_pingResult is null)
        {
            return;
        }

        var p = Look.P;
        _pingResult.Foreground = Look.Brush(p.Haze);
        _pingResult.Text = $"Asking {prof.Model}…";
        try
        {
            using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(60));
            var reply = await _host.Assistant.PingAsync(prof, cts.Token);
            _pingResult.Foreground = Look.Brush(p.Ok);
            _pingResult.Text = "Connected. Reply: " + (reply.Length > 160 ? reply[..160] + "…" : reply);
        }
        catch (Exception ex)
        {
            _pingResult.Foreground = Look.Brush(p.Fault);
            _pingResult.Text = "Connection failed: " + ex.Message;
        }
    }

    // ------------------------------------------------------------------ run targets

    /// <summary>Selects a run target by name (tests / screenshots).</summary>
    public void SelectTarget(string name)
    {
        _target = _draft.RunTargets.FirstOrDefault(t => t.Name == name) ?? _target;
        _section = "Run targets";
        Render();
    }

    private Control TargetsPage()
    {
        var p = Look.P;
        var list = new StackPanel { Spacing = 4 };
        foreach (var t in _draft.RunTargets)
        {
            var robot = t.Kind == RunTargetKind.PhysicalRobot;
            var selected = t == _target;
            var item = new Button
            {
                HorizontalAlignment = HorizontalAlignment.Stretch,
                HorizontalContentAlignment = HorizontalAlignment.Stretch,
                Background = Look.Brush(selected ? p.Raised : p.Surface),
                BorderBrush = Look.Brush(selected ? (robot ? p.Warn : p.Signal) : p.Seam),
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(8),
                Padding = new Thickness(10, 8),
                Content = new DockPanel
                {
                    Children =
                    {
                        Kit.Dock(new Border { Margin = new Thickness(0, 2, 9, 0), VerticalAlignment = VerticalAlignment.Top, Child = Kit.Icon(robot ? Icons.Robot : Icons.Sim, 16, robot ? p.Warn : p.Telemetry, 2) }, Avalonia.Controls.Dock.Left),
                        Kit.Dock(t.Name == _draft.ActiveRunTarget ? Kit.Chip("active", p.Ok) : new Border(), Avalonia.Controls.Dock.Right),
                        Kit.Col(2,
                            Kit.Text(t.Name.Length == 0 ? "(unnamed)" : t.Name, 12.5, FontWeight.SemiBold, wrap: true),
                            Kit.Text($"{(robot ? "robot" : "simulator")} · {t.Rmw}", 11, color: p.Haze, mono: true)),
                    },
                },
            };
            item.Click += (_, _) =>
            {
                _target = t;
                Render();
            };
            list.Children.Add(item);
        }

        var add = Kit.Row(6,
            Kit.Button("Simulator", () => AddTarget(new RunTarget { Name = $"Simulator {_draft.RunTargets.Count + 1}", Kind = RunTargetKind.Simulator, Rmw = "inprocess" }), icon: Icons.Plus),
            Kit.Button("Robot", () => AddTarget(new RunTarget { Name = $"Robot {_draft.RunTargets.Count + 1}", Kind = RunTargetKind.PhysicalRobot, Rmw = "rtps", DomainId = 0, MaxLinearSpeed = 0.3, MaxAngularSpeed = 1.0 }), icon: Icons.Plus));

        var left = Kit.Col(10, Kit.Eyebrow("Run targets"), list, add);
        var grid = new Grid { ColumnDefinitions = new ColumnDefinitions("270,*"), ColumnSpacing = 18 };
        grid.Children.Add(Kit.At(left, 0, 0));
        grid.Children.Add(Kit.At(_target is null ? Kit.Muted("Add a simulator or robot target.", 12.5) : TargetForm(_target), 0, 1));
        return grid;
    }

    private void AddTarget(RunTarget t)
    {
        _draft.RunTargets.Add(t);
        _target = t;
        Render();
    }

    private Control TargetForm(RunTarget t)
    {
        var p = Look.P;
        var name = Kit.Input(t.Name);
        name.TextChanged += (_, _) =>
        {
            var wasActive = _draft.ActiveRunTarget == t.Name;
            t.Name = name.Text ?? string.Empty;
            if (wasActive)
            {
                _draft.ActiveRunTarget = t.Name;
            }
        };

        var kind = new ComboBox { ItemsSource = new[] { "Simulator", "Physical robot" }, SelectedIndex = t.Kind == RunTargetKind.PhysicalRobot ? 1 : 0, Width = 200 };
        kind.SelectionChanged += (_, _) =>
        {
            t.Kind = kind.SelectedIndex == 1 ? RunTargetKind.PhysicalRobot : RunTargetKind.Simulator;
            if (t.Kind == RunTargetKind.PhysicalRobot && t.Rmw == "inprocess")
            {
                t.Rmw = "rtps";
            }

            Render();
        };

        var rmwOptions = t.Kind == RunTargetKind.PhysicalRobot ? new[] { "rtps", "rosbridge" } : new[] { "inprocess", "rtps", "rosbridge" };
        var rmw = new ComboBox { ItemsSource = rmwOptions, SelectedItem = rmwOptions.Contains(t.Rmw) ? t.Rmw : rmwOptions[0], Width = 200 };
        rmw.SelectionChanged += (_, _) =>
        {
            t.Rmw = rmw.SelectedItem as string ?? t.Rmw;
            Render();
        };

        var domain = Kit.Input(t.DomainId?.ToString(CultureInfo.InvariantCulture) ?? string.Empty, "from environment", mono: true);
        domain.Width = 120;
        domain.TextChanged += (_, _) => t.DomainId = int.TryParse(domain.Text, NumberStyles.Integer, CultureInfo.InvariantCulture, out var d) && d is >= 0 and <= 232 ? d : null;

        var url = Kit.Input(t.RosbridgeUrl, "ws://robot.local:9090", mono: true);
        url.TextChanged += (_, _) => t.RosbridgeUrl = url.Text ?? string.Empty;

        var ns = Kit.Input(t.RobotNamespace, "/robot1 (empty = /cmd_vel, /odom, /scan)", mono: true);
        ns.TextChanged += (_, _) => t.RobotNamespace = ns.Text ?? string.Empty;

        var nic = Kit.Input(t.NetworkInterface, "Ethernet, wlan0 or 192.168.1.20 (empty = all)", mono: true);
        nic.TextChanged += (_, _) => t.NetworkInterface = nic.Text ?? string.Empty;

        var localhost = new CheckBox { Content = Kit.Text("Localhost only (discovery stays on this machine)", 12.5), IsChecked = t.LocalhostOnly };
        localhost.IsCheckedChanged += (_, _) => t.LocalhostOnly = localhost.IsChecked == true;

        var scenario = new ComboBox
        {
            ItemsSource = Scenarios.Select(s => s.Length == 0 ? "(app default)" : s).ToArray(),
            SelectedIndex = Math.Max(0, Array.IndexOf(Scenarios, t.Scenario)),
            Width = 200,
        };
        scenario.SelectionChanged += (_, _) => t.Scenario = scenario.SelectedIndex > 0 ? Scenarios[scenario.SelectedIndex] : string.Empty;

        var maxLin = new NumericUpDown { Value = (decimal)t.MaxLinearSpeed, Minimum = 0, Maximum = 10, Increment = 0.05m, FormatString = "0.00", Width = 150 };
        maxLin.ValueChanged += (_, _) => t.MaxLinearSpeed = (double)(maxLin.Value ?? 0);
        var maxAng = new NumericUpDown { Value = (decimal)t.MaxAngularSpeed, Minimum = 0, Maximum = 20, Increment = 0.1m, FormatString = "0.00", Width = 150 };
        maxAng.ValueChanged += (_, _) => t.MaxAngularSpeed = (double)(maxAng.Value ?? 0);

        var env = new TextBox
        {
            Text = string.Join("\n", t.ExtraEnvironment.Select(kv => $"{kv.Key}={kv.Value}")),
            AcceptsReturn = true,
            Height = 64,
            FontFamily = Look.Mono,
            FontSize = 12,
            PlaceholderText = "NAME=value, one per line",
        };
        env.TextChanged += (_, _) =>
        {
            t.ExtraEnvironment = (env.Text ?? string.Empty).Split('\n')
                .Select(l => l.Trim())
                .Where(l => l.Contains('='))
                .Select(l => (Key: l[..l.IndexOf('=')].Trim(), Value: l[(l.IndexOf('=') + 1)..].Trim()))
                .Where(kv => kv.Key.Length > 0)
                .GroupBy(kv => kv.Key)
                .ToDictionary(g => g.Key, g => g.Last().Value, StringComparer.Ordinal);
        };

        var robot = t.Kind == RunTargetKind.PhysicalRobot;
        var actions = Kit.Row(8,
            Kit.Button("Use for Run", () =>
            {
                _draft.ActiveRunTarget = t.Name;
                Render();
            }, primary: t.Name != _draft.ActiveRunTarget, icon: Icons.Check),
            Kit.Button("Remove", () =>
            {
                _draft.RunTargets.Remove(t);
                _target = _draft.RunTargets.FirstOrDefault();
                Render();
            }, icon: Icons.Trash));

        var kinds = new Grid { ColumnDefinitions = new ColumnDefinitions("Auto,Auto"), ColumnSpacing = 14 };
        kinds.Children.Add(Kit.At(Kit.Field("Kind", kind), 0, 0));
        kinds.Children.Add(Kit.At(Kit.Field("Middleware", rmw), 0, 1));

        var form = Kit.Col(14, Kit.Field("Name", name), kinds);
        if (robot)
        {
            form.Children.Add(new Border
            {
                Background = Look.Brush(p.Warn, 0.12),
                BorderBrush = Look.Brush(p.Warn, 0.6),
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(8),
                Padding = new Thickness(10, 7),
                Child = new DockPanel
                {
                    Children =
                    {
                        Kit.Dock(new Border { Margin = new Thickness(0, 0, 8, 0), Child = Kit.Icon(Icons.Shield, 15, p.Warn, 2) }, Avalonia.Controls.Dock.Left),
                        Kit.Muted("Physical robot: no simulator starts. You confirm once per session before the first run.", 12, wrap: true),
                    },
                },
            });
        }

        if (t.Rmw == "rosbridge")
        {
            form.Children.Add(Kit.Field("rosbridge URL", url, "rosbridge_server on the robot (ros2 launch rosbridge_server rosbridge_websocket_launch.xml)."));
        }
        else if (t.Rmw == "rtps")
        {
            var rtps = new Grid { ColumnDefinitions = new ColumnDefinitions("Auto,*"), ColumnSpacing = 14 };
            rtps.Children.Add(Kit.At(Kit.Field("Domain id", domain, "ROS_DOMAIN_ID 0–232"), 0, 0));
            rtps.Children.Add(Kit.At(Kit.Field("Network interface", nic), 0, 1));
            form.Children.Add(rtps);
            form.Children.Add(localhost);
        }

        form.Children.Add(Kit.Field("Robot namespace", ns));
        if (!robot)
        {
            form.Children.Add(Kit.Field("Simulator scenario", scenario, "World the generated app's simulator starts with."));
        }

        var speeds = new Grid { ColumnDefinitions = new ColumnDefinitions("Auto,Auto"), ColumnSpacing = 18 };
        speeds.Children.Add(Kit.At(Kit.Field("Max linear speed (m/s)", maxLin), 0, 0));
        speeds.Children.Add(Kit.At(Kit.Field("Max angular speed (rad/s)", maxAng), 0, 1));
        form.Children.Add(speeds);
        form.Children.Add(Kit.Muted(robot ? "Safety limits passed to the app (ROS2NET_MAX_LINEAR / ROS2NET_MAX_ANGULAR). 0 leaves the app's default." : "0 leaves the app's default.", 11.5, wrap: true));
        form.Children.Add(Kit.Field("Extra environment variables", env));
        form.Children.Add(actions);
        return form;
    }

    // ------------------------------------------------------------------ other pages

    private Control JackPage()
    {
        var prompt = new TextBox
        {
            Text = _draft.SystemPrompt,
            AcceptsReturn = true,
            TextWrapping = TextWrapping.Wrap,
            Height = 300,
            FontFamily = Look.Mono,
            FontSize = 12,
            PlaceholderText = "Empty uses Jack's built-in prompt, which knows the ROS2.Net libraries and the project tools.",
        };
        prompt.TextChanged += (_, _) => _draft.SystemPrompt = prompt.Text ?? string.Empty;

        var rounds = new NumericUpDown { Value = _draft.MaxToolRounds, Minimum = 1, Maximum = 200, FormatString = "0", Width = 140 };
        rounds.ValueChanged += (_, _) => _draft.MaxToolRounds = (int)(rounds.Value ?? 24);

        var auto = new CheckBox { Content = Kit.Text("Let Jack build and run the project without asking", 12.5), IsChecked = _draft.AllowAutoBuild };
        auto.IsCheckedChanged += (_, _) => _draft.AllowAutoBuild = auto.IsChecked == true;

        return Kit.Col(16,
            Kit.Field("System prompt", prompt, "Jack — The Code Bender. Describe tone, coding rules or house conventions here."),
            Kit.Field("Tool rounds per message", rounds, "How many tool calls (read, write, build, search…) Jack may chain for one request."),
            auto);
    }

    private Control WebPage()
    {
        var key = Kit.Input(_draft.TavilyApiKey, "tvly-…", password: true, mono: true);
        key.TextChanged += (_, _) => _draft.TavilyApiKey = key.Text ?? string.Empty;
        return Kit.Col(16,
            Kit.Field("Tavily API key", key, "Used by Jack's SearchInternet tool. Without it, Jack can still scrape pages you link."),
            Kit.Muted("Other tools — page scraping, math, date and time — need no configuration.", 12.5, wrap: true));
    }

    private Control ProjectsPage()
    {
        var folder = Kit.Input(_draft.ProjectsFolder, "Documents/ROS2AppGen", mono: true);
        folder.TextChanged += (_, _) => _draft.ProjectsFolder = folder.Text ?? string.Empty;
        var browse = Kit.Button("Browse…", () => _ = BrowseAsync(folder));
        browse.Margin = new Thickness(6, 0, 0, 0);

        var source = Kit.Input(_draft.PackageSource, "Empty = nuget.org", mono: true);
        source.TextChanged += (_, _) => _draft.PackageSource = source.Text ?? string.Empty;
        var browseSource = Kit.Button("Browse…", () => _ = BrowseAsync(source));
        browseSource.Margin = new Thickness(6, 0, 0, 0);

        var version = Kit.Input(_draft.Ros2NetVersion, "0.1.0", mono: true);
        version.Width = 160;
        version.HorizontalAlignment = HorizontalAlignment.Left;
        version.TextChanged += (_, _) => _draft.Ros2NetVersion = version.Text ?? "0.1.0";

        return Kit.Col(16,
            Kit.Field("Projects folder", new DockPanel { Children = { Kit.Dock(browse, Avalonia.Controls.Dock.Right), folder } }, "Where new projects are created."),
            Kit.Field("ROS2.Net package source", new DockPanel { Children = { Kit.Dock(browseSource, Avalonia.Controls.Dock.Right), source } }, "A local feed folder (for example ROS2.Net/artifacts/packages) or empty for nuget.org."),
            Kit.Field("ROS2.Net version", version, "Package version referenced by generated projects."));
    }

    private Control EditorPage()
    {
        var lines = new CheckBox { Content = Kit.Text("Show line numbers", 12.5), IsChecked = _draft.ShowLineNumbers };
        lines.IsCheckedChanged += (_, _) => _draft.ShowLineNumbers = lines.IsChecked == true;
        var wrap = new CheckBox { Content = Kit.Text("Word wrap", 12.5), IsChecked = _draft.WordWrap };
        wrap.IsCheckedChanged += (_, _) => _draft.WordWrap = wrap.IsChecked == true;
        var size = new NumericUpDown { Value = (decimal)_draft.EditorFontSize, Minimum = 8, Maximum = 32, Increment = 1, FormatString = "0", Width = 140 };
        size.ValueChanged += (_, _) => _draft.EditorFontSize = (double)(size.Value ?? 13);
        var theme = new ComboBox { ItemsSource = new[] { "Dark", "Light" }, SelectedItem = _draft.Theme == "Light" ? "Light" : "Dark", Width = 160 };
        theme.SelectionChanged += (_, _) => _draft.Theme = theme.SelectedItem as string ?? "Dark";
        return Kit.Col(16, lines, wrap, Kit.Field("Font size", size), Kit.Field("Theme", theme, "Applies after saving."));
    }

    private Control LayoutPage()
    {
        var chat = new CheckBox { Content = Kit.Text("Show the chat panel", 12.5), IsChecked = _draft.ChatPanelVisible };
        chat.IsCheckedChanged += (_, _) => _draft.ChatPanelVisible = chat.IsChecked == true;
        var logs = new CheckBox { Content = Kit.Text("Show the logs panel", 12.5), IsChecked = _draft.LogsPanelVisible };
        logs.IsCheckedChanged += (_, _) => _draft.LogsPanelVisible = logs.IsChecked == true;
        var chatWidth = new NumericUpDown { Value = (decimal)_draft.ChatPanelWidth, Minimum = 280, Maximum = 1200, Increment = 20, FormatString = "0", Width = 140 };
        chatWidth.ValueChanged += (_, _) => _draft.ChatPanelWidth = (double)(chatWidth.Value ?? 420);
        var explorerWidth = new NumericUpDown { Value = (decimal)_draft.ExplorerWidth, Minimum = 160, Maximum = 600, Increment = 10, FormatString = "0", Width = 140 };
        explorerWidth.ValueChanged += (_, _) => _draft.ExplorerWidth = (double)(explorerWidth.Value ?? 260);
        return Kit.Col(16, chat, logs, Kit.Field("Chat panel width", chatWidth), Kit.Field("Explorer width", explorerWidth),
            Kit.Muted("Widths also update when you drag the splitters; they are saved when you exit.", 12, wrap: true));
    }

    private async Task BrowseAsync(TextBox target)
    {
        var folders = await StorageProvider.OpenFolderPickerAsync(new FolderPickerOpenOptions { Title = "Choose a folder" });
        if (folders.FirstOrDefault()?.TryGetLocalPath() is { } path)
        {
            target.Text = path;
        }
    }
}
