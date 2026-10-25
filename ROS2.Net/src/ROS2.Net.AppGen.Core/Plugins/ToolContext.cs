namespace ROS2.Net.AppGen.Plugins;

/// <summary>What the kernel plugins operate on: the open workspace, build tooling, templates and settings.</summary>
public sealed class ToolContext
{
    public required FileSystemWorkspace Workspace { get; init; }

    public required BuildService Build { get; init; }

    public required ProjectService Projects { get; init; }

    public required Func<AppGenSettings> Settings { get; init; }

    /// <summary>Log sink (logs panel).</summary>
    public Action<string> Log { get; init; } = _ => { };

    /// <summary>Shared HTTP client for web tools.</summary>
    public HttpClient Http { get; init; } = CreateHttpClient();

    /// <summary>Maximum characters a tool returns to the model; longer results are truncated with a note.</summary>
    public int MaxToolOutput { get; init; } = 24_000;

    public string RequireRoot() => Workspace.RootPath ?? throw new InvalidOperationException("No project is open. Create one with create_project / create_project_from_template, or ask the user to open one.");

    public string Truncate(string text, int? max = null)
    {
        var limit = max ?? MaxToolOutput;
        return text.Length <= limit ? text : text[..limit] + $"\n… [truncated {text.Length - limit} characters]";
    }

    private static HttpClient CreateHttpClient()
    {
        var http = new HttpClient { Timeout = TimeSpan.FromSeconds(30) };
        http.DefaultRequestHeaders.UserAgent.ParseAdd("ROS2AppGen/0.1 (+https://github.com/DotNetVibeCoderz/Vibe_Robotics)");
        return http;
    }
}
