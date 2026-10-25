using ROS2.Net.AppGen.Plugins;

namespace ROS2.Net.AppGen.Tests;

/// <summary>Creates hosts with throw-away App.config files.</summary>
internal static class TestHost
{
    /// <summary>The repository's local package feed (<c>dotnet pack ROS2.Net.slnx -c Release -o artifacts/packages</c>).</summary>
    public static string LocalFeed { get; } = Path.GetFullPath(Path.Combine(RepoRoot(), "artifacts", "packages"));

    public static AppGenHost Create()
    {
        var config = Path.Combine(Path.GetTempPath(), "ros2appgen-tests", Guid.NewGuid().ToString("N"), "App.config");
        var host = AppGenHost.Create(config);
        host.Settings.ProjectsFolder = Path.GetDirectoryName(config)!;
        return host;
    }

    public static ToolContext Tools(AppGenHost host) => new()
    {
        Workspace = (FileSystemWorkspace)host.Workspace,
        Build = (BuildService)host.Build,
        Projects = (ProjectService)host.Projects,
        Settings = () => host.Settings,
    };

    public static string TempFolder()
    {
        var dir = Path.Combine(Path.GetTempPath(), "ros2appgen-tests", Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        return dir;
    }

    public static string RepoRoot()
    {
        var dir = AppContext.BaseDirectory;
        while (dir is not null && !File.Exists(Path.Combine(dir, "ROS2.Net.slnx")))
        {
            dir = Path.GetDirectoryName(dir);
        }

        return dir ?? throw new InvalidOperationException("ROS2.Net.slnx not found above the test output folder.");
    }
}
