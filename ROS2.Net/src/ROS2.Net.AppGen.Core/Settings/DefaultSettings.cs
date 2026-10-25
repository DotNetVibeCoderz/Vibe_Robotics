namespace ROS2.Net.AppGen;

/// <summary>Factory defaults: one profile per supported provider, all with empty API keys.</summary>
public static class DefaultSettings
{
    /// <summary>Creates the settings a first run starts with.</summary>
    public static AppGenSettings Create() => new()
    {
        Profiles =
        [
            new LlmProfile { Name = "Claude Sonnet", Provider = LlmProviderKind.Claude, Model = "claude-sonnet-5-5", Temperature = 0.2, MaxTokens = 16000 },
            new LlmProfile { Name = "OpenAI GPT-5 mini", Provider = LlmProviderKind.OpenAI, Model = "gpt-5-mini", Temperature = 1.0 },
            new LlmProfile { Name = "Azure OpenAI", Provider = LlmProviderKind.AzureOpenAI, Model = "gpt-5-mini", Endpoint = "https://<resource>.openai.azure.com/", Temperature = 1.0 },
            new LlmProfile { Name = "Gemini Flash", Provider = LlmProviderKind.Gemini, Model = "gemini-2.5-flash", Temperature = 0.2 },
            new LlmProfile { Name = "Ollama (local)", Provider = LlmProviderKind.Ollama, Model = "llama3.2", Endpoint = "http://localhost:11434", Temperature = 0.2, SupportsVision = false },
            new LlmProfile { Name = "DeepSeek", Provider = LlmProviderKind.DeepSeek, Model = "deepseek-v4-flash", Endpoint = "https://api.deepseek.com", Temperature = 0.2, SupportsVision = false },
        ],
        ActiveProfile = "Claude Sonnet",
        RunTargets = RunTargetEnvironment.Defaults(),
        ActiveRunTarget = RunTargetEnvironment.DefaultSimulator.Name,
        ProjectsFolder = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "ROS2AppGen"),
    };

    /// <summary>
    /// The content of a fresh <c>App.config</c> (and of the shipped <c>App.config.template</c>):
    /// defaults only, no secrets.
    /// </summary>
    public static string ConfigTemplateXml() => AppConfigSettingsStore.ToXml(Create());

    /// <summary>Default provider endpoint when the profile leaves it empty.</summary>
    public static string DefaultEndpoint(LlmProviderKind kind) => kind switch
    {
        LlmProviderKind.Ollama => "http://localhost:11434",
        LlmProviderKind.DeepSeek => "https://api.deepseek.com",
        LlmProviderKind.OpenAI => "https://api.openai.com/v1",
        LlmProviderKind.Claude => "https://api.anthropic.com/v1/",
        _ => string.Empty,
    };
}
