using Microsoft.Extensions.DependencyInjection;
using Microsoft.SemanticKernel;
using Microsoft.SemanticKernel.ChatCompletion;
using Microsoft.SemanticKernel.Connectors.AzureOpenAI;
using Microsoft.SemanticKernel.Connectors.Google;
using Microsoft.SemanticKernel.Connectors.Ollama;
using Microsoft.SemanticKernel.Connectors.OpenAI;
using ROS2.Net.AppGen.Plugins;

namespace ROS2.Net.AppGen.Ai;

/// <summary>Raised when a profile is missing something it needs (API key, endpoint, model).</summary>
public sealed class LlmConfigurationException(string message) : InvalidOperationException(message);

/// <summary>
/// Builds a Semantic Kernel for an <see cref="LlmProfile"/>. OpenAI, Azure OpenAI, Gemini and Ollama use
/// their first-party SK connectors; DeepSeek and Claude use the OpenAI connector against their
/// OpenAI-compatible endpoints (<c>https://api.deepseek.com</c>, <c>https://api.anthropic.com/v1/</c>).
/// </summary>
public static class KernelFactory
{
    /// <summary>Creates a kernel; when <paramref name="tools"/> is given, Jack's plugins are registered.</summary>
    public static Kernel Create(LlmProfile profile, ToolContext? tools = null, IEnumerable<object>? filters = null)
    {
        Validate(profile);
        var builder = Kernel.CreateBuilder();
        var key = AppConfigSettingsStore.ResolveSecret(profile.ApiKey);
        var endpoint = string.IsNullOrWhiteSpace(profile.Endpoint) ? DefaultSettings.DefaultEndpoint(profile.Provider) : profile.Endpoint.Trim();
        var http = CreateHttpClient(profile);

        switch (profile.Provider)
        {
            case LlmProviderKind.OpenAI:
                if (string.IsNullOrWhiteSpace(profile.Endpoint))
                {
                    builder.AddOpenAIChatCompletion(profile.Model, key, httpClient: http);
                }
                else
                {
                    builder.AddOpenAIChatCompletion(profile.Model, new Uri(endpoint), key, httpClient: http);
                }

                break;

            case LlmProviderKind.DeepSeek:
                builder.AddOpenAIChatCompletion(profile.Model, new Uri(endpoint), key, httpClient: http);
                break;

            case LlmProviderKind.AzureOpenAI:
                builder.AddAzureOpenAIChatCompletion(profile.Model, endpoint, key, httpClient: http);
                break;

            case LlmProviderKind.Gemini:
                builder.AddGoogleAIGeminiChatCompletion(profile.Model, key, GoogleAIVersion.V1_Beta, httpClient: http);
                break;

            case LlmProviderKind.Ollama:
                builder.AddOllamaChatCompletion(profile.Model, new Uri(endpoint));
                break;

            case LlmProviderKind.Claude:
                // Anthropic's OpenAI-compatible Messages endpoint: streaming, tool use and images through the
                // first-party SK OpenAI connector (no extra SDK whose Microsoft.Extensions.AI version could clash).
                builder.AddOpenAIChatCompletion(profile.Model, new Uri(endpoint), key, httpClient: http);
                break;

            default:
                throw new LlmConfigurationException($"Unknown provider {profile.Provider}.");
        }

        if (tools is not null)
        {
            builder.Plugins.AddFromObject(new ProjectPlugin(tools), "project");
            builder.Plugins.AddFromObject(new BuildPlugin(tools), "build");
            builder.Plugins.AddFromObject(new Ros2NetKnowledgePlugin(), "ros2net");
            builder.Plugins.AddFromObject(new WebPlugin(tools), "web");
            builder.Plugins.AddFromObject(new NuGetPlugin(tools), "nuget");
            builder.Plugins.AddFromObject(new MathPlugin(), "math");
            builder.Plugins.AddFromObject(new DateTimePlugin(), "time");
        }

        foreach (var f in filters ?? [])
        {
            switch (f)
            {
                case IFunctionInvocationFilter fi:
                    builder.Services.AddSingleton(fi);
                    break;
            }

            if (f is IAutoFunctionInvocationFilter af)
            {
                builder.Services.AddSingleton(af);
            }
        }

        return builder.Build();
    }

    /// <summary>An HTTP client carrying the profile's <see cref="LlmProfile.ExtraHeaders"/>.</summary>
    public static HttpClient CreateHttpClient(LlmProfile profile)
    {
        var http = new HttpClient(new HeaderHandler(ParseHeaders(profile.ExtraHeaders)) { InnerHandler = new SocketsHttpHandler() })
        {
            Timeout = TimeSpan.FromMinutes(5),
        };
        return http;
    }

    /// <summary>Parses <c>Name: value</c> pairs separated by newlines or ';'.</summary>
    public static IReadOnlyList<KeyValuePair<string, string>> ParseHeaders(string? text)
    {
        if (string.IsNullOrWhiteSpace(text))
        {
            return [];
        }

        var list = new List<KeyValuePair<string, string>>();
        foreach (var part in text.Split([(char)10, (char)13, ';'], StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
        {
            var colon = part.IndexOf(':');
            if (colon > 0)
            {
                list.Add(new(part[..colon].Trim(), AppConfigSettingsStore.ResolveSecret(part[(colon + 1)..].Trim())));
            }
        }

        return list;
    }

    private sealed class HeaderHandler(IReadOnlyList<KeyValuePair<string, string>> headers) : DelegatingHandler
    {
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            foreach (var (name, value) in headers)
            {
                request.Headers.Remove(name);
                request.Headers.TryAddWithoutValidation(name, value);
            }

            return base.SendAsync(request, cancellationToken);
        }
    }

    /// <summary>Execution settings for the profile: temperature, token limit and automatic function calling.</summary>
    public static PromptExecutionSettings CreateSettings(LlmProfile profile, bool tools)
    {
        var choice = tools ? FunctionChoiceBehavior.Auto() : FunctionChoiceBehavior.None();
        var temperature = SupportsTemperature(profile) ? profile.Temperature : (double?)null;
        int? maxTokens = profile.MaxTokens > 0 ? profile.MaxTokens : null;
        return profile.Provider switch
        {
            LlmProviderKind.AzureOpenAI => new AzureOpenAIPromptExecutionSettings
            {
                Temperature = temperature,
                MaxTokens = IsReasoningModel(profile.Model) ? null : maxTokens,
                FunctionChoiceBehavior = choice,
            },
            LlmProviderKind.OpenAI or LlmProviderKind.DeepSeek or LlmProviderKind.Claude => new OpenAIPromptExecutionSettings
            {
                Temperature = temperature,
                MaxTokens = IsReasoningModel(profile.Model) ? null : maxTokens,
                FunctionChoiceBehavior = choice,
            },
            LlmProviderKind.Gemini => new GeminiPromptExecutionSettings
            {
                Temperature = temperature,
                MaxTokens = maxTokens,
                FunctionChoiceBehavior = choice,
            },
            LlmProviderKind.Ollama => new OllamaPromptExecutionSettings
            {
                Temperature = (float?)temperature,
                NumPredict = maxTokens,
                FunctionChoiceBehavior = choice,
            },
            _ => new PromptExecutionSettings
            {
                ModelId = profile.Model,
                FunctionChoiceBehavior = choice,
                ExtensionData = new Dictionary<string, object>(StringComparer.Ordinal)
                {
                    ["temperature"] = temperature ?? 1.0,
                    ["max_tokens"] = maxTokens ?? 16000,
                },
            },
        };
    }

    /// <summary>Checks a profile has what its provider needs.</summary>
    public static void Validate(LlmProfile profile)
    {
        if (string.IsNullOrWhiteSpace(profile.Model))
        {
            throw new LlmConfigurationException($"Profile '{profile.Name}' has no model.");
        }

        var needsKey = profile.Provider != LlmProviderKind.Ollama;
        if (needsKey && string.IsNullOrWhiteSpace(AppConfigSettingsStore.ResolveSecret(profile.ApiKey)))
        {
            throw new LlmConfigurationException($"Profile '{profile.Name}' ({profile.Provider}) has no API key. Open Settings to add one.");
        }

        if (profile.Provider == LlmProviderKind.AzureOpenAI && (string.IsNullOrWhiteSpace(profile.Endpoint) || profile.Endpoint.Contains('<')))
        {
            throw new LlmConfigurationException($"Profile '{profile.Name}' needs the Azure OpenAI endpoint (https://<resource>.openai.azure.com/).");
        }

        if (!string.IsNullOrWhiteSpace(profile.Endpoint) && !Uri.TryCreate(profile.Endpoint, UriKind.Absolute, out _))
        {
            throw new LlmConfigurationException($"Profile '{profile.Name}' has an invalid endpoint '{profile.Endpoint}'.");
        }
    }

    /// <summary>OpenAI reasoning models (gpt-5*, o-series) only accept the default temperature and no max_tokens.</summary>
    public static bool IsReasoningModel(string model)
    {
        var m = model.ToLowerInvariant();
        return m.StartsWith("gpt-5", StringComparison.Ordinal) || m.StartsWith("o1", StringComparison.Ordinal) ||
               m.StartsWith("o3", StringComparison.Ordinal) || m.StartsWith("o4", StringComparison.Ordinal);
    }

    private static bool SupportsTemperature(LlmProfile p) => p.Provider switch
    {
        LlmProviderKind.OpenAI or LlmProviderKind.AzureOpenAI => !IsReasoningModel(p.Model),

        // Claude 5-generation models reject a temperature ("deprecated for this model").
        LlmProviderKind.Claude => !System.Text.RegularExpressions.Regex.IsMatch(p.Model, @"claude-(?:[a-z]+-)?(?:[5-9]|\d{2,})", System.Text.RegularExpressions.RegexOptions.IgnoreCase),
        _ => true,
    };
}
