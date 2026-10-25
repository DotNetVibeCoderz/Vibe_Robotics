using ROS2.Net.AppGen.Ai;
using ROS2.Net.AppGen.Plugins;

namespace ROS2.Net.AppGen;

public sealed partial class AppGenHost
{
    /// <summary>
    /// Creates the host: loads settings from <paramref name="configPath"/> (default: App.config next to
    /// the executable, created with defaults on first run) and wires the workspace, templates, build
    /// tooling, formatter and Jack.
    /// </summary>
    public static AppGenHost Create(string? configPath = null)
    {
        var store = new AppConfigSettingsStore(configPath);
        var settings = store.Load();
        AppGenHost? host = null;
        void Log(string line) => host?.WriteLog(line);

        var workspace = new FileSystemWorkspace();
        var build = new BuildService { Log = Log };
        var projects = new ProjectService(() => host!.Settings);
        var tools = new ToolContext
        {
            Workspace = workspace,
            Build = build,
            Projects = projects,
            Settings = () => host!.Settings,
            Log = Log,
        };

        host = new AppGenHost
        {
            SettingsStore = store,
            Settings = settings,
            Workspace = workspace,
            Projects = projects,
            Build = build,
            Formatter = new CodeFormatter(),
            Assistant = new JackAssistant(() => host!.Settings, tools),
        };
        return host;
    }

    /// <summary>Adds a project folder to the recent list (most recent first, max 12) and saves settings.</summary>
    public void AddRecentProject(string folder)
    {
        var full = Path.GetFullPath(folder);
        Settings.RecentProjects.RemoveAll(p => string.Equals(p, full, StringComparison.OrdinalIgnoreCase));
        Settings.RecentProjects.Insert(0, full);
        if (Settings.RecentProjects.Count > 12)
        {
            Settings.RecentProjects.RemoveRange(12, Settings.RecentProjects.Count - 12);
        }

        SaveSettings();
    }

    /// <summary>The model profile Jack currently uses.</summary>
    public LlmProfile? ActiveProfile =>
        Settings.Profiles.FirstOrDefault(p => p.Name == Settings.ActiveProfile) ?? Settings.Profiles.FirstOrDefault();
}
