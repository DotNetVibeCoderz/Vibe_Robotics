namespace ROS2.Net.AppGen.Tests.Live;

/// <summary>
/// Reads provider credentials for opt-in live tests from a local key file that lives OUTSIDE the
/// repository (default <c>C:/Users/mifma/Documents/CodeSandbox/testkey.txt</c>, override with
/// <c>ROS2NET_APPGEN_KEYFILE</c>). Live tests run only when <c>ROS2NET_APPGEN_LIVE=1</c>.
/// Keys are never written anywhere or printed.
/// </summary>
internal static class TestKeys
{
    public static bool LiveEnabled => Environment.GetEnvironmentVariable("ROS2NET_APPGEN_LIVE") == "1";

    public static string KeyFile => Environment.GetEnvironmentVariable("ROS2NET_APPGEN_KEYFILE") ?? "C:/Users/mifma/Documents/CodeSandbox/testkey.txt";

    private static readonly Lazy<Dictionary<string, Dictionary<string, string>>> s_sections = new(Load);

    /// <summary>Value of <paramref name="key"/> in <paramref name="section"/> (case-insensitive), or null.</summary>
    public static string? Get(string section, params string[] keys)
    {
        if (!s_sections.Value.TryGetValue(section, out var values))
        {
            return null;
        }

        foreach (var k in keys)
        {
            if (values.TryGetValue(k, out var v) && v.Length > 0)
            {
                return v;
            }
        }

        return null;
    }

    public static LlmProfile? Azure() =>
        Get("Azure OpenAI", "apikey") is { } key && Get("Azure OpenAI", "endpoint") is { } endpoint
            ? new LlmProfile { Name = "Azure", Provider = LlmProviderKind.AzureOpenAI, ApiKey = key, Endpoint = endpoint, Model = First(Get("Azure OpenAI", "model")) ?? "gpt-5-mini", Temperature = 1.0 }
            : null;

    public static LlmProfile? DeepSeek() =>
        Get("DeepSeek", "Deep Seek Api Key", "apikey") is { } key
            ? new LlmProfile { Name = "DeepSeek", Provider = LlmProviderKind.DeepSeek, ApiKey = key, Endpoint = Get("DeepSeek", "Deep Seek Endpoint", "endpoint") ?? "https://api.deepseek.com", Model = First(Get("DeepSeek", "Model Id", "model")) ?? "deepseek-v4-flash", SupportsVision = false }
            : null;

    public static LlmProfile? Claude() =>
        Get("Anthropic", "ApiKey", "apikey") is { } key
            ? new LlmProfile
            {
                Name = "Claude",
                Provider = LlmProviderKind.Claude,
                ApiKey = key,
                Model = First(Get("Anthropic", "model")) ?? "claude-sonnet-5-5",
                Temperature = 0.2,
                MaxTokens = 16000,
                ExtraHeaders = Get("Anthropic", "workspaceId") is { } ws ? $"anthropic-workspace-id: {ws}" : string.Empty,
            }
            : null;

    public static string? Tavily() => Get("Tavily", "apikey");

    private static string? First(string? csv) => csv?.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries).FirstOrDefault();

    /// <summary>
    /// Parses free-text sections: a line without ':' starts a section ("Azure OpenAI"); following
    /// "key: value" lines belong to it. Lines starting with '#' or 'export' are ignored.
    /// </summary>
    internal static Dictionary<string, Dictionary<string, string>> Parse(IEnumerable<string> lines)
    {
        var result = new Dictionary<string, Dictionary<string, string>>(StringComparer.OrdinalIgnoreCase);
        Dictionary<string, string>? current = null;
        foreach (var raw in lines)
        {
            var line = raw.Trim();
            if (line.Length == 0 || line.StartsWith('#') || line.StartsWith("export ", StringComparison.Ordinal))
            {
                continue;
            }

            var colon = line.IndexOf(':');
            var looksLikeUrlOnly = line.StartsWith("http", StringComparison.OrdinalIgnoreCase);
            if (colon < 0 || looksLikeUrlOnly)
            {
                if (!looksLikeUrlOnly)
                {
                    current = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                    result[line] = current;
                }

                continue;
            }

            if (current is null)
            {
                continue;
            }

            var key = line[..colon].Trim();
            var value = line[(colon + 1)..].Trim();
            current.TryAdd(key, value);
        }

        return result;
    }

    private static Dictionary<string, Dictionary<string, string>> Load() =>
        File.Exists(KeyFile) ? Parse(File.ReadAllLines(KeyFile)) : new(StringComparer.OrdinalIgnoreCase);
}
