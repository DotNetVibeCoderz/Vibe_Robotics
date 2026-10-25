namespace ROS2.Net.AppGen;

/// <summary>
/// Composition root shared by the UI: one instance per app, owning settings and services.
/// The backend provides the static <c>Create</c> factory in <c>AppGenHost.Factory.cs</c>.
/// </summary>
public sealed partial class AppGenHost
{
    public required ISettingsStore SettingsStore { get; init; }

    /// <summary>Live settings; call <see cref="SaveSettings"/> after changing them.</summary>
    public required AppGenSettings Settings { get; set; }

    public required IWorkspace Workspace { get; init; }

    public required IProjectService Projects { get; init; }

    public required IBuildService Build { get; init; }

    public required ICodeFormatter Formatter { get; init; }

    public required ICodeAssistant Assistant { get; init; }

    /// <summary>Diagnostic log lines from services (assistant, tools, builds) for the logs panel.</summary>
    public event Action<string>? Log;

    public void WriteLog(string line) => Log?.Invoke(line);

    public void SaveSettings() => SettingsStore.Save(Settings);
}
