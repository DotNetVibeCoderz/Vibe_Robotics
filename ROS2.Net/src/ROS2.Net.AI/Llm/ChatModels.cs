using System.Globalization;
using System.Net.Http.Headers;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;

namespace ROS2.Net.AI.Llm;

/// <summary>A chat message (<c>system</c>, <c>user</c>, <c>assistant</c> or <c>tool</c>).</summary>
public sealed record ChatMessage(string Role, string? Content, IReadOnlyList<ToolCall>? ToolCalls = null, string? ToolCallId = null)
{
    public static ChatMessage System(string text) => new("system", text);

    public static ChatMessage User(string text) => new("user", text);
}

/// <summary>A tool invocation requested by the model; arguments are a JSON object.</summary>
public sealed record ToolCall(string Id, string Name, string ArgumentsJson)
{
    public double GetDouble(string name, double fallback = 0)
    {
        using var doc = JsonDocument.Parse(string.IsNullOrWhiteSpace(ArgumentsJson) ? "{}" : ArgumentsJson);
        if (!doc.RootElement.TryGetProperty(name, out var v))
        {
            return fallback;
        }

        return v.ValueKind == JsonValueKind.Number ? v.GetDouble()
            : v.ValueKind == JsonValueKind.String && double.TryParse(v.GetString(), NumberStyles.Float, CultureInfo.InvariantCulture, out var d) ? d
            : fallback;
    }

    public string? GetString(string name)
    {
        using var doc = JsonDocument.Parse(string.IsNullOrWhiteSpace(ArgumentsJson) ? "{}" : ArgumentsJson);
        return doc.RootElement.TryGetProperty(name, out var v) ? (v.ValueKind == JsonValueKind.String ? v.GetString() : v.GetRawText()) : null;
    }
}

/// <summary>A tool the model may call, described by a JSON Schema for its parameters.</summary>
public sealed record ToolDefinition(string Name, string Description, string ParametersJsonSchema);

/// <summary>A model reply: text and/or tool calls.</summary>
public sealed record ChatResult(string? Content, IReadOnlyList<ToolCall> ToolCalls);

/// <summary>Minimal chat-completion abstraction with tool calling.</summary>
public interface IChatModel
{
    Task<ChatResult> CompleteAsync(IReadOnlyList<ChatMessage> messages, IReadOnlyList<ToolDefinition> tools, CancellationToken cancellationToken = default);
}

/// <summary>Settings of <see cref="OpenAICompatibleChatModel"/>.</summary>
public sealed class OpenAICompatibleOptions
{
    /// <summary>Base URL including <c>/v1</c>. Default: a local Ollama server.</summary>
    public string Endpoint { get; set; } = Environment.GetEnvironmentVariable("ROS2NET_LLM_ENDPOINT") ?? "http://localhost:11434/v1";

    public string Model { get; set; } = Environment.GetEnvironmentVariable("ROS2NET_LLM_MODEL") ?? "llama3.2";

    /// <summary>API key; defaults to <c>ROS2NET_LLM_API_KEY</c>, then <c>OPENAI_API_KEY</c>. Local servers need none.</summary>
    public string? ApiKey { get; set; } = Environment.GetEnvironmentVariable("ROS2NET_LLM_API_KEY") ?? Environment.GetEnvironmentVariable("OPENAI_API_KEY");

    /// <summary>Send the key as Azure's <c>api-key</c> header instead of a bearer token.</summary>
    public bool UseAzureApiKeyHeader { get; set; }

    /// <summary>Optional query string, e.g. <c>api-version=2024-10-21</c> for Azure OpenAI.</summary>
    public string? QueryString { get; set; }

    public double Temperature { get; set; } = 0.1;

    public TimeSpan Timeout { get; set; } = TimeSpan.FromSeconds(60);
}

/// <summary>
/// A client for any OpenAI-compatible <c>/chat/completions</c> endpoint: OpenAI, Azure OpenAI,
/// Ollama, llama.cpp server, LM Studio, vLLM. No keys are stored in code.
/// </summary>
public sealed class OpenAICompatibleChatModel : IChatModel, IDisposable
{
    private readonly HttpClient _http;
    private readonly bool _ownsHttp;
    private readonly OpenAICompatibleOptions _options;

    public OpenAICompatibleChatModel(OpenAICompatibleOptions? options = null, HttpClient? http = null)
    {
        _options = options ?? new OpenAICompatibleOptions();
        _ownsHttp = http is null;
        _http = http ?? new HttpClient { Timeout = _options.Timeout };
    }

    public async Task<ChatResult> CompleteAsync(IReadOnlyList<ChatMessage> messages, IReadOnlyList<ToolDefinition> tools, CancellationToken cancellationToken = default)
    {
        var url = _options.Endpoint.TrimEnd('/') + "/chat/completions" + (string.IsNullOrEmpty(_options.QueryString) ? string.Empty : "?" + _options.QueryString);
        using var request = new HttpRequestMessage(HttpMethod.Post, url)
        {
            Content = new StringContent(BuildRequest(messages, tools), Encoding.UTF8, "application/json"),
        };
        if (!string.IsNullOrEmpty(_options.ApiKey))
        {
            if (_options.UseAzureApiKeyHeader)
            {
                request.Headers.Add("api-key", _options.ApiKey);
            }
            else
            {
                request.Headers.Authorization = new AuthenticationHeaderValue("Bearer", _options.ApiKey);
            }
        }

        using var response = await _http.SendAsync(request, cancellationToken).ConfigureAwait(false);
        var body = await response.Content.ReadAsStringAsync(cancellationToken).ConfigureAwait(false);
        if (!response.IsSuccessStatusCode)
        {
            throw new HttpRequestException($"Chat completion failed ({(int)response.StatusCode}): {body}");
        }

        return ParseResponse(body);
    }

    internal string BuildRequest(IReadOnlyList<ChatMessage> messages, IReadOnlyList<ToolDefinition> tools)
    {
        using var stream = new MemoryStream();
        using (var w = new Utf8JsonWriter(stream))
        {
            w.WriteStartObject();
            w.WriteString("model", _options.Model);
            w.WriteNumber("temperature", _options.Temperature);
            w.WriteStartArray("messages");
            foreach (var m in messages)
            {
                w.WriteStartObject();
                w.WriteString("role", m.Role);
                if (m.Content is not null)
                {
                    w.WriteString("content", m.Content);
                }

                if (m.ToolCallId is not null)
                {
                    w.WriteString("tool_call_id", m.ToolCallId);
                }

                if (m.ToolCalls is { Count: > 0 })
                {
                    w.WriteStartArray("tool_calls");
                    foreach (var c in m.ToolCalls)
                    {
                        w.WriteStartObject();
                        w.WriteString("id", c.Id);
                        w.WriteString("type", "function");
                        w.WriteStartObject("function");
                        w.WriteString("name", c.Name);
                        w.WriteString("arguments", c.ArgumentsJson);
                        w.WriteEndObject();
                        w.WriteEndObject();
                    }

                    w.WriteEndArray();
                }

                w.WriteEndObject();
            }

            w.WriteEndArray();
            if (tools.Count > 0)
            {
                w.WriteStartArray("tools");
                foreach (var t in tools)
                {
                    w.WriteStartObject();
                    w.WriteString("type", "function");
                    w.WriteStartObject("function");
                    w.WriteString("name", t.Name);
                    w.WriteString("description", t.Description);
                    w.WritePropertyName("parameters");
                    using (var schema = JsonDocument.Parse(t.ParametersJsonSchema))
                    {
                        schema.RootElement.WriteTo(w);
                    }

                    w.WriteEndObject();
                    w.WriteEndObject();
                }

                w.WriteEndArray();
            }

            w.WriteEndObject();
        }

        return Encoding.UTF8.GetString(stream.ToArray());
    }

    internal static ChatResult ParseResponse(string body)
    {
        using var doc = JsonDocument.Parse(body);
        var message = doc.RootElement.GetProperty("choices")[0].GetProperty("message");
        var content = message.TryGetProperty("content", out var c) && c.ValueKind == JsonValueKind.String ? c.GetString() : null;
        var calls = new List<ToolCall>();
        if (message.TryGetProperty("tool_calls", out var tc) && tc.ValueKind == JsonValueKind.Array)
        {
            foreach (var call in tc.EnumerateArray())
            {
                var fn = call.GetProperty("function");
                var args = fn.TryGetProperty("arguments", out var a) ? (a.ValueKind == JsonValueKind.String ? a.GetString() ?? "{}" : a.GetRawText()) : "{}";
                calls.Add(new ToolCall(call.TryGetProperty("id", out var id) ? id.GetString() ?? string.Empty : Guid.NewGuid().ToString("N"), fn.GetProperty("name").GetString() ?? string.Empty, args));
            }
        }

        return new ChatResult(content, calls);
    }

    public void Dispose()
    {
        if (_ownsHttp)
        {
            _http.Dispose();
        }
    }
}

/// <summary>
/// An offline, deterministic "model" that understands simple English and Bahasa Indonesia commands
/// ("forward 1 m", "maju 50 cm", "turn left 90", "belok kanan 45 derajat", "go to dock", "pergi ke rak A1",
/// "stop"/"berhenti"). Useful without an LLM server, for tests and as a fallback.
/// </summary>
public sealed partial class RuleBasedChatModel : IChatModel
{
    public Task<ChatResult> CompleteAsync(IReadOnlyList<ChatMessage> messages, IReadOnlyList<ToolDefinition> tools, CancellationToken cancellationToken = default)
    {
        var text = messages.LastOrDefault(m => m.Role == "user")?.Content ?? string.Empty;
        var calls = new List<ToolCall>();
        foreach (var clause in ClauseSplit().Split(text.ToLowerInvariant()))
        {
            if (Parse(clause.Trim()) is { } call)
            {
                calls.Add(call with { Id = "call_" + calls.Count });
            }
        }

        return Task.FromResult(new ChatResult(calls.Count == 0 ? "Sorry, I did not understand that command." : null, calls));
    }

    private static ToolCall? Parse(string c)
    {
        if (c.Length == 0)
        {
            return null;
        }

        if (StopWords().IsMatch(c))
        {
            return new ToolCall(string.Empty, "stop", "{}");
        }

        var go = GoTo().Match(c);
        if (go.Success)
        {
            return new ToolCall(string.Empty, "go_to_waypoint", JsonSerializer.Serialize(new { name = go.Groups["name"].Value.Trim().Replace(' ', '_') }));
        }

        var number = Number().Match(c);
        var value = number.Success ? double.Parse(number.Groups["n"].Value.Replace(',', '.'), CultureInfo.InvariantCulture) : double.NaN;

        if (Turn().IsMatch(c))
        {
            var deg = double.IsNaN(value) ? 90 : value;
            if (Radians().IsMatch(c))
            {
                deg = deg * 180 / Math.PI;
            }

            var right = RightWords().IsMatch(c);
            return new ToolCall(string.Empty, "rotate", Json(("angle_deg", right ? -deg : deg)));
        }

        var backward = Backward().IsMatch(c);
        if (backward || Forward().IsMatch(c))
        {
            var meters = double.IsNaN(value) ? 0.5 : value;
            if (Centimeters().IsMatch(c))
            {
                meters /= 100;
            }

            return new ToolCall(string.Empty, "move_forward", Json(("distance_m", backward ? -meters : meters)));
        }

        return null;
    }

    private static string Json(params (string Key, double Value)[] props) =>
        "{" + string.Join(",", props.Select(p => $"\"{p.Key}\":{p.Value.ToString("R", CultureInfo.InvariantCulture)}")) + "}";

    [GeneratedRegex(@"\b(then|and then|lalu|kemudian|terus|setelah itu)\b|;|,(?=\s)|\.(?=\s|$)")]
    private static partial Regex ClauseSplit();

    [GeneratedRegex(@"\b(stop|halt|berhenti|stop!|diam)\b")]
    private static partial Regex StopWords();

    [GeneratedRegex(@"\b(go to|navigate to|move to|pergi ke|menuju|ke titik)\s+(the\s+)?(?<name>[a-z0-9_ ]+)")]
    private static partial Regex GoTo();

    [GeneratedRegex(@"(?<n>-?\d+([.,]\d+)?)")]
    private static partial Regex Number();

    [GeneratedRegex(@"\b(turn|rotate|putar|belok)\b")]
    private static partial Regex Turn();

    [GeneratedRegex(@"\b(right|kanan|clockwise)\b")]
    private static partial Regex RightWords();

    [GeneratedRegex(@"\b(rad|radian|radians)\b")]
    private static partial Regex Radians();

    [GeneratedRegex(@"\b(back|backward|backwards|reverse|mundur)\b")]
    private static partial Regex Backward();

    [GeneratedRegex(@"\b(forward|ahead|go|move|drive|maju|jalan)\b")]
    private static partial Regex Forward();

    [GeneratedRegex(@"\b(cm|centimeter|centimeters|senti|sentimeter)\b")]
    private static partial Regex Centimeters();
}
