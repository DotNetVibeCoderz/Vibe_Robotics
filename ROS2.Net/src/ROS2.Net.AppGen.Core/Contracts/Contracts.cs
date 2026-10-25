// Shared contract between the AppGen backend (this project) and the Avalonia UI (apps/ROS2.Net.AppGen).
// Keep these shapes stable; implementations live elsewhere in this project.
namespace ROS2.Net.AppGen;

/// <summary>LLM providers Jack can run on.</summary>
public enum LlmProviderKind
{
    OpenAI,
    AzureOpenAI,
    Claude,
    Gemini,
    Ollama,
    DeepSeek,
}

/// <summary>One configured model endpoint.</summary>
public sealed class LlmProfile
{
    /// <summary>Display name shown in the chat panel's model picker, e.g. "Claude Sonnet".</summary>
    public string Name { get; set; } = string.Empty;

    public LlmProviderKind Provider { get; set; }

    /// <summary>Model / deployment id, e.g. <c>claude-sonnet-5-5</c>, <c>gpt-5-mini</c>, <c>llama3.2</c>.</summary>
    public string Model { get; set; } = string.Empty;

    public string ApiKey { get; set; } = string.Empty;

    /// <summary>Base endpoint; empty uses the provider default.</summary>
    public string Endpoint { get; set; } = string.Empty;

    public double Temperature { get; set; } = 0.2;

    /// <summary>Optional per-profile max output tokens (0 = provider default).</summary>
    public int MaxTokens { get; set; }

    /// <summary>Whether the model accepts image input.</summary>
    public bool SupportsVision { get; set; } = true;

    /// <summary>
    /// Extra HTTP headers sent with every request, one <c>Name: value</c> per line (or separated by ';').
    /// Example for Anthropic keys that are not workspace-scoped: <c>anthropic-workspace-id: wrkspc_…</c>.
    /// Values may use <c>env:VARIABLE</c>.
    /// </summary>
    public string ExtraHeaders { get; set; } = string.Empty;
}

/// <summary>Where Run sends the generated app.</summary>
public enum RunTargetKind
{
    /// <summary>Software only: the app starts a ROS2.Net.Simulation world (or needs no robot at all).</summary>
    Simulator,
    /// <summary>A real robot: no simulator; the app talks to the robot's topics over RTPS or rosbridge.</summary>
    PhysicalRobot,
}

/// <summary>
/// A named run configuration. Passed to the app as environment variables (see
/// <c>RunTargetEnvironment</c>): <c>ROS2NET_TARGET=sim|robot</c>, <c>ROS2NET_RMW</c>, <c>ROS_DOMAIN_ID</c>,
/// <c>ROS2NET_ROSBRIDGE_URL</c>, <c>ROS2NET_ROBOT_NS</c>, <c>ROS_LOCALHOST_ONLY</c>,
/// <c>ROS2NET_NETWORK_INTERFACE</c>, <c>ROS2NET_SIM_SCENARIO</c>, <c>ROS2NET_MAX_LINEAR</c>, <c>ROS2NET_MAX_ANGULAR</c>.
/// </summary>
public sealed class RunTarget
{
    /// <summary>Display name, e.g. "Simulator (in-process)".</summary>
    public string Name { get; set; } = string.Empty;

    public RunTargetKind Kind { get; set; }

    /// <summary>Middleware: <c>inprocess</c> (simulator only), <c>rtps</c> or <c>rosbridge</c>.</summary>
    public string Rmw { get; set; } = "inprocess";

    /// <summary>DDS domain id (ROS_DOMAIN_ID); null keeps the environment's value.</summary>
    public int? DomainId { get; set; }

    /// <summary>rosbridge URL when <see cref="Rmw"/> is <c>rosbridge</c>.</summary>
    public string RosbridgeUrl { get; set; } = "ws://localhost:9090";

    /// <summary>Robot namespace, e.g. <c>/robot1</c>; empty for un-namespaced topics (/cmd_vel, /odom, /scan).</summary>
    public string RobotNamespace { get; set; } = string.Empty;

    /// <summary>Network interface (name or IP) for RTPS; empty = all.</summary>
    public string NetworkInterface { get; set; } = string.Empty;

    /// <summary>Restrict discovery to this machine.</summary>
    public bool LocalhostOnly { get; set; }

    /// <summary>Simulator scenario (Empty, Warehouse, Office, Maze, CorridorLoop, AgricultureRows); empty = the app's default.</summary>
    public string Scenario { get; set; } = string.Empty;

    /// <summary>Safety velocity limits for physical robots (0 = the app's default).</summary>
    public double MaxLinearSpeed { get; set; }

    public double MaxAngularSpeed { get; set; }

    /// <summary>Additional environment variables for the app.</summary>
    public Dictionary<string, string> ExtraEnvironment { get; set; } = new(StringComparer.Ordinal);
}

/// <summary>All persisted settings (stored in App.config next to the executable).</summary>
public sealed class AppGenSettings
{
    public List<LlmProfile> Profiles { get; set; } = [];

    public string ActiveProfile { get; set; } = string.Empty;

    /// <summary>System prompt for Jack - The Code Bender; empty uses the built-in default.</summary>
    public string SystemPrompt { get; set; } = string.Empty;

    public string TavilyApiKey { get; set; } = string.Empty;

    /// <summary>Where new projects are created by default.</summary>
    public string ProjectsFolder { get; set; } = string.Empty;

    /// <summary>Where ROS2.Net NuGet packages come from: empty = nuget.org, or a local folder feed (e.g. ROS2.Net/artifacts/packages).</summary>
    public string PackageSource { get; set; } = string.Empty;

    public string Ros2NetVersion { get; set; } = "0.1.0";

    public List<string> RecentProjects { get; set; } = [];

    // Editor / layout preferences.
    public bool ShowLineNumbers { get; set; } = true;

    public double EditorFontSize { get; set; } = 13;

    public bool WordWrap { get; set; }

    public string Theme { get; set; } = "Dark";

    public double ChatPanelWidth { get; set; } = 420;

    public bool ChatPanelVisible { get; set; } = true;

    public double ExplorerWidth { get; set; } = 260;

    public bool LogsPanelVisible { get; set; } = true;

    /// <summary>Let Jack run builds/commands without asking.</summary>
    public bool AllowAutoBuild { get; set; } = true;

    /// <summary>Max tool-call rounds per user message.</summary>
    public int MaxToolRounds { get; set; } = 24;

    /// <summary>Saved run targets (simulator and physical robots).</summary>
    public List<RunTarget> RunTargets { get; set; } = [];

    /// <summary>Name of the run target Run uses.</summary>
    public string ActiveRunTarget { get; set; } = string.Empty;
}

/// <summary>Loads and saves <see cref="AppGenSettings"/> in App.config (appSettings section).</summary>
public interface ISettingsStore
{
    AppGenSettings Load();

    void Save(AppGenSettings settings);

    /// <summary>Full path of the config file in use.</summary>
    string ConfigPath { get; }
}

/// <summary>An image attached to a chat message.</summary>
public sealed record ImageAttachment(string FileName, string MediaType, byte[] Data);

/// <summary>Kinds of events streamed by the assistant while answering.</summary>
public enum AssistantEventKind
{
    /// <summary>A chunk of assistant text (append to the current reply).</summary>
    TextDelta,
    /// <summary>A kernel function is about to run (<see cref="AssistantEvent.Text"/> = "Plugin.Function(args)").</summary>
    ToolStarted,
    /// <summary>A kernel function finished (<see cref="AssistantEvent.Text"/> = short result summary).</summary>
    ToolCompleted,
    /// <summary>The reply is complete.</summary>
    Completed,
    /// <summary>An error ended the reply (<see cref="AssistantEvent.Text"/> = message).</summary>
    Error,
}

public sealed record AssistantEvent(AssistantEventKind Kind, string Text, string? ToolName = null);

/// <summary>A message kept in the chat thread.</summary>
public sealed record ChatEntry(string Role, string Text, IReadOnlyList<ImageAttachment> Images, DateTimeOffset Time);

/// <summary>Jack - The Code Bender: the AI assistant that writes ROS2.Net apps.</summary>
public interface ICodeAssistant
{
    string Name { get; }

    IReadOnlyList<ChatEntry> Thread { get; }

    /// <summary>Sends a user message (with optional images) and streams the answer, running kernel functions as needed.</summary>
    IAsyncEnumerable<AssistantEvent> SendAsync(string message, IReadOnlyList<ImageAttachment> images, CancellationToken cancellationToken);

    /// <summary>Clears the conversation.</summary>
    void ClearThread();

    /// <summary>Switches model profile (keeps the thread).</summary>
    void UseProfile(string profileName);

    /// <summary>Tests connectivity of a profile with a tiny prompt; returns the reply or throws.</summary>
    Task<string> PingAsync(LlmProfile profile, CancellationToken cancellationToken);
}

/// <summary>The open project folder. All file edits by Jack go through it so the UI can refresh.</summary>
public interface IWorkspace
{
    /// <summary>Root folder of the open project, or null when none is open.</summary>
    string? RootPath { get; }

    /// <summary>Raised when a file is created, changed, deleted or renamed (path is absolute).</summary>
    event Action<string>? FileChanged;

    /// <summary>Raised when the open project changes.</summary>
    event Action? ProjectChanged;

    void Open(string folderOrProjectFile);

    void Close();

    IReadOnlyList<string> ListFiles(string? relativeFolder = null, bool recursive = true);

    string ReadFile(string relativePath);

    void WriteFile(string relativePath, string content);

    void DeleteFile(string relativePath);

    void RenameFile(string from, string to);

    void CreateFolder(string relativePath);

    /// <summary>Path of the file currently focused in the editor (relative), if any; set by the UI.</summary>
    string? ActiveFile { get; set; }

    /// <summary>The text currently selected in the editor; set by the UI.</summary>
    string? ActiveSelection { get; set; }
}

/// <summary>A project template. <c>AppKind</c> is "console", "avalonia" or "blazor".</summary>
public sealed record ProjectTemplate(
    string Id,
    string Name,
    string Category,
    string Description,
    string AppKind,
    IReadOnlyList<string> Tags);

/// <summary>Creates projects.</summary>
public interface IProjectService
{
    IReadOnlyList<ProjectTemplate> Templates { get; }

    IReadOnlyList<string> Categories { get; }

    /// <summary>Creates a blank ROS2.Net console project. Returns the project folder.</summary>
    string CreateBlank(string parentFolder, string projectName, string appKind = "console");

    /// <summary>Creates a project from a template. Returns the project folder.</summary>
    string CreateFromTemplate(string templateId, string parentFolder, string projectName);
}

/// <summary>Output line from a build/run/deploy process.</summary>
public sealed record ProcessOutput(string Line, bool IsError);

/// <summary>Result of a dotnet command.</summary>
public sealed record ProcessResult(int ExitCode, string Output, TimeSpan Duration)
{
    public bool Success => ExitCode == 0;
}

/// <summary>Runs dotnet build / run / publish for the open project.</summary>
public interface IBuildService
{
    Task<ProcessResult> BuildAsync(string projectFolder, Action<ProcessOutput>? output, CancellationToken cancellationToken);

    /// <summary>Starts the app against the default simulator target; completes when it exits or is stopped.</summary>
    Task<ProcessResult> RunAsync(string projectFolder, Action<ProcessOutput>? output, CancellationToken cancellationToken);

    /// <summary>Starts the app against <paramref name="target"/> (simulator or physical robot), passed as environment variables.</summary>
    Task<ProcessResult> RunAsync(string projectFolder, RunTarget target, Action<ProcessOutput>? output, CancellationToken cancellationToken);

    bool IsRunning { get; }

    void StopRun();

    /// <summary>dotnet publish to <paramref name="outputFolder"/> (self-contained single file optional).</summary>
    Task<ProcessResult> DeployAsync(string projectFolder, string outputFolder, string runtime, bool selfContained, Action<ProcessOutput>? output, CancellationToken cancellationToken);
}

/// <summary>Formats source code by file extension (.cs via Roslyn, .json, .xml/.csproj/.axaml, .razor best effort).</summary>
public interface ICodeFormatter
{
    bool CanFormat(string fileName);

    string Format(string fileName, string text);
}
