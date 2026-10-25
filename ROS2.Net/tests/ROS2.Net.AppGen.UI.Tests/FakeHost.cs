using System.Runtime.CompilerServices;
using ROS2.Net.AppGen;

namespace ROS2.Net.AppGen.UI.Tests;

/// <summary>A deterministic host for UI tests and screenshots: real files on disk, scripted Jack, fake build.</summary>
public static class FakeHost
{
    public static AppGenHost Create(string? root = null, AppGenSettings? settings = null)
    {
        var store = new MemoryStore(settings ?? DefaultTestSettings());
        var ws = new FsWorkspace();
        var host = new AppGenHost
        {
            SettingsStore = store,
            Settings = store.Load(),
            Workspace = ws,
            Projects = RealTemplatesOrSamples(store),
            Build = new FakeBuild(),
            Formatter = new TrimFormatter(),
            Assistant = new ScriptedJack(ws),
        };
        if (root is not null)
        {
            ws.Open(root);
        }

        return host;
    }

    /// <summary>The backend's real template catalog once it has one; the small sample catalog otherwise.</summary>
    private static IProjectService RealTemplatesOrSamples(ISettingsStore store)
    {
        try
        {
            var real = new ProjectService(store.Load);
            if (real.Templates.Count >= 5)
            {
                return real;
            }
        }
        catch (Exception)
        {
        }

        return new SampleProjects();
    }

    public static AppGenSettings DefaultTestSettings() => new()
    {
        Profiles =
        [
            new LlmProfile { Name = "Claude Sonnet", Provider = LlmProviderKind.Claude, Model = "claude-sonnet-5-5", ApiKey = "test" },
            new LlmProfile { Name = "GPT-5 mini", Provider = LlmProviderKind.AzureOpenAI, Model = "gpt-5-mini", Endpoint = "https://example.openai.azure.com/" },
            new LlmProfile { Name = "Ollama (local)", Provider = LlmProviderKind.Ollama, Model = "llama3.2", Endpoint = "http://localhost:11434", SupportsVision = false },
        ],
        ActiveProfile = "Claude Sonnet",
        RunTargets = RunTargetEnvironment.Defaults(),
        ActiveRunTarget = "Simulator (in-process)",
        ProjectsFolder = Path.Combine(Path.GetTempPath(), "ros2appgen-tests"),
    };

    /// <summary>Creates a small ROS2.Net project on disk for the explorer and editor to show.</summary>
    public static string CreateSampleProject(string name = "WarehouseMonitor")
    {
        var root = Path.Combine(Path.GetTempPath(), "ros2appgen-ui", name + "-" + Guid.NewGuid().ToString("N")[..6]);
        Directory.CreateDirectory(Path.Combine(root, "Nodes"));
        Directory.CreateDirectory(Path.Combine(root, "Views"));
        File.WriteAllText(Path.Combine(root, name + ".csproj"), """
            <Project Sdk="Microsoft.NET.Sdk">
              <PropertyGroup>
                <OutputType>WinExe</OutputType>
                <TargetFramework>net10.0</TargetFramework>
                <Nullable>enable</Nullable>
              </PropertyGroup>
              <ItemGroup>
                <PackageReference Include="ROS2.Net" Version="0.1.0" />
                <PackageReference Include="ROS2.Net.Simulation" Version="0.1.0" />
                <PackageReference Include="ROS2.Net.Avalonia" Version="0.1.0" />
              </ItemGroup>
            </Project>
            """);
        File.WriteAllText(Path.Combine(root, "Program.cs"), """
            using ROS2.Net;
            using ROS2.Net.Simulation;
            using WarehouseMonitor.Nodes;

            // A simulated warehouse with two robots, a safety monitor and a fleet dashboard.
            await using var context = await Ros2Context.CreateAsync(new Ros2ContextOptions { Arguments = args });

            var world = Scenarios.Warehouse();
            using var sim = new SimulationNode(context, world);
            using var safety = new SafetyMonitor(context, stopDistance: 0.45);
            using var fleet = new FleetStatusNode(context, robots: ["robot1", "robot2"]);

            Console.WriteLine("Warehouse running. Ctrl+C to stop.");
            await context.SpinAsync();
            """);
        File.WriteAllText(Path.Combine(root, "Nodes", "SafetyMonitor.cs"), """
            using ROS2.Net;
            using ROS2.Net.Msgs.GeometryMsgs;
            using ROS2.Net.Msgs.SensorMsgs;

            namespace WarehouseMonitor.Nodes;

            /// <summary>Stops a robot when its LiDAR sees an obstacle closer than the stop distance.</summary>
            public sealed class SafetyMonitor : Node
            {
                private readonly Publisher<Twist> _stop;
                private readonly double _stopDistance;

                public SafetyMonitor(Ros2Context context, double stopDistance)
                    : base(context, "safety_monitor")
                {
                    _stopDistance = DeclareParameter("stop_distance", stopDistance);
                    _stop = CreatePublisher<Twist>("/robot1/cmd_vel");
                    CreateSubscription<LaserScan>("/robot1/scan", OnScan, QosProfile.SensorData);
                }

                private void OnScan(LaserScan scan)
                {
                    var nearest = scan.Ranges.Where(float.IsFinite).DefaultIfEmpty(float.MaxValue).Min();
                    if (nearest < _stopDistance)
                    {
                        _stop.Publish(new Twist());
                        Logger.LogWarning("Obstacle at {Distance:0.00} m — robot1 stopped", nearest);
                    }
                }
            }
            """);
        File.WriteAllText(Path.Combine(root, "Nodes", "FleetStatusNode.cs"), "namespace WarehouseMonitor.Nodes;\n\npublic sealed class FleetStatusNode { }\n");
        File.WriteAllText(Path.Combine(root, "Views", "Dashboard.cs"), "namespace WarehouseMonitor.Views;\n\npublic sealed class Dashboard { }\n");
        File.WriteAllText(Path.Combine(root, "appsettings.json"), "{\n  \"rmw\": \"inprocess\",\n  \"domainId\": 0\n}\n");
        File.WriteAllText(Path.Combine(root, "README.md"), "# Warehouse monitor\n\nGenerated with ROS2AppGen.\n");
        return root;
    }
}

internal sealed class MemoryStore(AppGenSettings settings) : ISettingsStore
{
    public int SaveCount { get; private set; }

    public string ConfigPath => Path.Combine(Path.GetTempPath(), "ros2appgen-tests", "App.config");

    public AppGenSettings Load() => settings;

    public void Save(AppGenSettings s)
    {
        settings = s;
        SaveCount++;
    }
}

internal sealed class FsWorkspace : IWorkspace
{
    public string? RootPath { get; private set; }

    public event Action<string>? FileChanged;

    public event Action? ProjectChanged;

    public string? ActiveFile { get; set; }

    public string? ActiveSelection { get; set; }

    public void Open(string folderOrProjectFile)
    {
        RootPath = Directory.Exists(folderOrProjectFile) ? Path.GetFullPath(folderOrProjectFile) : Path.GetDirectoryName(Path.GetFullPath(folderOrProjectFile));
        ProjectChanged?.Invoke();
    }

    public void Close()
    {
        RootPath = null;
        ProjectChanged?.Invoke();
    }

    public IReadOnlyList<string> ListFiles(string? relativeFolder = null, bool recursive = true)
    {
        if (RootPath is null)
        {
            return [];
        }

        var folder = Path.Combine(RootPath, relativeFolder ?? string.Empty);
        return Directory.EnumerateFiles(folder, "*", recursive ? SearchOption.AllDirectories : SearchOption.TopDirectoryOnly)
            .Select(f => Path.GetRelativePath(RootPath, f).Replace('\\', '/'))
            .Where(f => !f.StartsWith("bin/", StringComparison.Ordinal) && !f.StartsWith("obj/", StringComparison.Ordinal))
            .Order(StringComparer.Ordinal)
            .ToList();
    }

    public string ReadFile(string relativePath) => File.ReadAllText(Full(relativePath));

    public void WriteFile(string relativePath, string content)
    {
        var full = Full(relativePath);
        Directory.CreateDirectory(Path.GetDirectoryName(full)!);
        File.WriteAllText(full, content);
        FileChanged?.Invoke(full);
    }

    public void DeleteFile(string relativePath)
    {
        var full = Full(relativePath);
        File.Delete(full);
        FileChanged?.Invoke(full);
    }

    public void RenameFile(string from, string to)
    {
        File.Move(Full(from), Full(to));
        FileChanged?.Invoke(Full(from));
        FileChanged?.Invoke(Full(to));
    }

    public void CreateFolder(string relativePath)
    {
        Directory.CreateDirectory(Full(relativePath));
        FileChanged?.Invoke(Full(relativePath));
    }

    private string Full(string rel) => Path.GetFullPath(Path.Combine(RootPath ?? throw new InvalidOperationException("No project open."), rel));
}

internal sealed class SampleProjects : IProjectService
{
    public IReadOnlyList<ProjectTemplate> Templates { get; } =
    [
        new("warehouse-fleet", "Warehouse fleet monitor", "Industry", "Two simulated AMRs in a warehouse with LiDAR safety stops, a fleet dashboard and NavigateToPose goals.", "avalonia", ["nav2", "fleet", "simulation"]),
        new("turtle-classroom", "Classroom turtle lab", "Education", "A turtlesim-compatible playground for teaching topics, services and actions, with step-by-step lessons.", "avalonia", ["turtlesim", "lessons"]),
        new("lab-logger", "Lab instrument logger", "Science", "Record IMU, temperature and custom sensor topics to MCAP bags and export CSV for analysis.", "console", ["rosbag2", "csv"]),
        new("vacuum-bot", "Home vacuum robot", "Housekeeping", "Coverage planning over an occupancy grid with a battery model and docking behavior tree.", "avalonia", ["coverage", "bt"]),
        new("drone-search", "Search & rescue drone", "Disaster & safety", "A simulated drone sweeping a grid pattern, streaming detections and GPS fixes to a web console.", "blazor", ["gps", "web"]),
        new("arm-pick", "Pick-and-place cell", "Industry", "A UR5-like arm planning around obstacles with RRT-Connect and a gripper action server.", "avalonia", ["moveit", "urdf"]),
        new("robot-pet", "Robot pet", "Entertainment", "An expressive mobile robot that follows people and reacts to voice commands through Jack's LLM tools.", "console", ["llm", "behavior"]),
        new("office-runner", "Office delivery runner", "Office", "Deliver parcels between desks using named waypoints, with a Blazor booking page.", "blazor", ["waypoints"]),
        new("topic-inspector", "Topic inspector", "Tools", "A desktop tool listing topics with rates, echoing messages and plotting numeric fields.", "avalonia", ["introspection"]),
    ];

    public IReadOnlyList<string> Categories => Templates.Select(t => t.Category).Distinct().Order().ToList();

    public string CreateBlank(string parentFolder, string projectName, string appKind = "console") => Write(parentFolder, projectName, $"// Blank {appKind} ROS2.Net app\n");

    public string CreateFromTemplate(string templateId, string parentFolder, string projectName) => Write(parentFolder, projectName, $"// From template {templateId}\n");

    private static string Write(string parent, string name, string program)
    {
        var root = Path.Combine(parent, name);
        Directory.CreateDirectory(root);
        File.WriteAllText(Path.Combine(root, name + ".csproj"), "<Project Sdk=\"Microsoft.NET.Sdk\" />\n");
        File.WriteAllText(Path.Combine(root, "Program.cs"), program);
        return root;
    }
}

internal sealed class FakeBuild : IBuildService
{
    public bool IsRunning { get; private set; }

    /// <summary>The target of the last run (tests).</summary>
    public RunTarget? LastTarget { get; private set; }

    public Task<ProcessResult> RunAsync(string projectFolder, RunTarget target, Action<ProcessOutput>? output, CancellationToken cancellationToken)
    {
        LastTarget = target;
        return RunAsync(projectFolder, output, cancellationToken);
    }

    public Task<ProcessResult> BuildAsync(string projectFolder, Action<ProcessOutput>? output, CancellationToken cancellationToken)
    {
        output?.Invoke(new ProcessOutput("  Determining projects to restore...", false));
        output?.Invoke(new ProcessOutput($"  {Path.GetFileName(projectFolder)} -> bin/Debug/net10.0/{Path.GetFileName(projectFolder)}.dll", false));
        output?.Invoke(new ProcessOutput("Build succeeded.    0 Warning(s)    0 Error(s)", false));
        return Task.FromResult(new ProcessResult(0, "ok", TimeSpan.FromSeconds(3.2)));
    }

    public async Task<ProcessResult> RunAsync(string projectFolder, Action<ProcessOutput>? output, CancellationToken cancellationToken)
    {
        IsRunning = true;
        output?.Invoke(new ProcessOutput("Warehouse running. Ctrl+C to stop.", false));
        await Task.Delay(50, cancellationToken);
        IsRunning = false;
        return new ProcessResult(0, string.Empty, TimeSpan.FromMilliseconds(50));
    }

    public void StopRun() => IsRunning = false;

    public Task<ProcessResult> DeployAsync(string projectFolder, string outputFolder, string runtime, bool selfContained, Action<ProcessOutput>? output, CancellationToken cancellationToken) =>
        Task.FromResult(new ProcessResult(0, string.Empty, TimeSpan.FromSeconds(1)));
}

internal sealed class TrimFormatter : ICodeFormatter
{
    public bool CanFormat(string fileName) => fileName.EndsWith(".cs", StringComparison.OrdinalIgnoreCase);

    public string Format(string fileName, string text) =>
        string.Join("\n", text.Replace("\r\n", "\n").Split('\n').Select(l => l.TrimEnd()));
}

/// <summary>Plays back a fixed answer with tool calls, editing a real file like the real Jack would.</summary>
internal sealed class ScriptedJack(IWorkspace workspace) : ICodeAssistant
{
    private readonly List<ChatEntry> _thread = [];

    public string Name => "Jack - The Code Bender";

    public IReadOnlyList<ChatEntry> Thread => _thread;

    public string Profile { get; private set; } = "Claude Sonnet";

    public int Delay { get; set; } = 0;

    public async IAsyncEnumerable<AssistantEvent> SendAsync(string message, IReadOnlyList<ImageAttachment> images, [EnumeratorCancellation] CancellationToken cancellationToken)
    {
        _thread.Add(new ChatEntry("user", message, images, DateTimeOffset.Now));
        yield return new AssistantEvent(AssistantEventKind.ToolStarted, "Project.ListFiles()", "Project.ListFiles");
        yield return new AssistantEvent(AssistantEventKind.ToolCompleted, "8 files", "Project.ListFiles");
        yield return new AssistantEvent(AssistantEventKind.ToolStarted, "Ros2Net.GetApiReference(\"Subscription QoS\")", "Ros2Net.GetApiReference");
        yield return new AssistantEvent(AssistantEventKind.ToolCompleted, "QosProfile.SensorData: keep last 5, best effort", "Ros2Net.GetApiReference");
        yield return new AssistantEvent(AssistantEventKind.ToolStarted, "Project.WriteFile(\"Nodes/FleetStatusNode.cs\")", "Project.WriteFile");
        if (workspace.RootPath is not null)
        {
            workspace.WriteFile("Nodes/FleetStatusNode.cs", "using ROS2.Net;\nusing ROS2.Net.Msgs.NavMsgs;\n\nnamespace WarehouseMonitor.Nodes;\n\n/// <summary>Collects odometry from every robot for the dashboard.</summary>\npublic sealed class FleetStatusNode : Node\n{\n    public FleetStatusNode(Ros2Context context, string[] robots)\n        : base(context, \"fleet_status\")\n    {\n        foreach (var robot in robots)\n        {\n            CreateSubscription<Odometry>($\"/{robot}/odom\", odom => Latest[robot] = odom);\n        }\n    }\n\n    public Dictionary<string, Odometry> Latest { get; } = [];\n}\n");
        }

        yield return new AssistantEvent(AssistantEventKind.ToolCompleted, "wrote 512 bytes", "Project.WriteFile");
        yield return new AssistantEvent(AssistantEventKind.ToolStarted, "Build.BuildProject()", "Build.BuildProject");
        yield return new AssistantEvent(AssistantEventKind.ToolCompleted, "Build succeeded, 0 errors", "Build.BuildProject");
        var reply = """
            Done — `FleetStatusNode` now subscribes to **each robot's `/odom`** and keeps the latest pose for the dashboard.

            What changed:
            - `Nodes/FleetStatusNode.cs`: one `Subscription<Odometry>` per robot, stored in `Latest`.
            - The safety monitor keeps `QosProfile.SensorData` for `/scan`, so a slow consumer never blocks the LiDAR.

            ```csharp
            CreateSubscription<Odometry>($"/{robot}/odom", odom => Latest[robot] = odom);
            ```

            The project builds cleanly. Want me to add a fleet map to `Views/Dashboard.cs` next?
            """;
        foreach (var chunk in Chunks(reply, 24))
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (Delay > 0)
            {
                await Task.Delay(Delay, cancellationToken);
            }

            yield return new AssistantEvent(AssistantEventKind.TextDelta, chunk);
        }

        _thread.Add(new ChatEntry("assistant", reply, [], DateTimeOffset.Now));
        yield return new AssistantEvent(AssistantEventKind.Completed, string.Empty);
        await Task.CompletedTask;
    }

    public void ClearThread() => _thread.Clear();

    public void UseProfile(string profileName) => Profile = profileName;

    public Task<string> PingAsync(LlmProfile profile, CancellationToken cancellationToken) => Task.FromResult("pong");

    private static IEnumerable<string> Chunks(string s, int size)
    {
        for (var i = 0; i < s.Length; i += size)
        {
            yield return s.Substring(i, Math.Min(size, s.Length - i));
        }
    }
}
