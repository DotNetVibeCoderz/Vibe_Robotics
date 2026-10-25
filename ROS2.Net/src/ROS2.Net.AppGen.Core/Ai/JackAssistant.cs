using System.Net;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Channels;
using Microsoft.SemanticKernel;
using Microsoft.SemanticKernel.ChatCompletion;
using ROS2.Net.AppGen.Plugins;

namespace ROS2.Net.AppGen.Ai;

/// <summary>
/// Jack - The Code Bender: a Semantic Kernel chat agent with project, build, ROS2.Net knowledge,
/// web, math and time tools. Answers stream as <see cref="AssistantEvent"/>s; tool calls are reported
/// as they happen.
/// </summary>
public sealed class JackAssistant : ICodeAssistant
{
    private readonly Func<AppGenSettings> _settings;
    private readonly ToolContext _tools;
    private readonly List<ChatEntry> _thread = [];
    private readonly ChatHistory _history = [];
    private readonly object _gate = new();
    private string? _profileName;
    private (string Key, Kernel Kernel, ToolEventFilter Filter)? _kernel;

    public JackAssistant(Func<AppGenSettings> settings, ToolContext tools)
    {
        _settings = settings;
        _tools = tools;
    }

    public string Name => JackPrompt.AssistantName;

    public IReadOnlyList<ChatEntry> Thread
    {
        get
        {
            lock (_gate)
            {
                return _thread.ToList();
            }
        }
    }

    /// <summary>The profile currently used for answers.</summary>
    public LlmProfile? ActiveProfile
    {
        get
        {
            var s = _settings();
            var name = _profileName ?? s.ActiveProfile;
            return s.Profiles.FirstOrDefault(p => p.Name == name) ?? s.Profiles.FirstOrDefault();
        }
    }

    public void ClearThread()
    {
        lock (_gate)
        {
            _thread.Clear();
            _history.Clear();
        }
    }

    public void UseProfile(string profileName)
    {
        lock (_gate)
        {
            if (_profileName == profileName)
            {
                return;
            }

            _profileName = profileName;

            // Tool-call messages are provider specific (ids, formats): keep only the plain conversation.
            var text = _history.Where(m => (m.Role == AuthorRole.User || m.Role == AuthorRole.Assistant) && !string.IsNullOrEmpty(m.Content)).ToList();
            _history.Clear();
            foreach (var m in text)
            {
                _history.Add(new ChatMessageContent(m.Role, m.Content));
            }
        }
    }

    public async Task<string> PingAsync(LlmProfile profile, CancellationToken cancellationToken)
    {
        var kernel = KernelFactory.Create(profile);
        var chat = kernel.GetRequiredService<IChatCompletionService>();
        var history = new ChatHistory();
        history.AddUserMessage("Reply with exactly the word: pong");
        var settings = KernelFactory.CreateSettings(profile, tools: false);
        var reply = await chat.GetChatMessageContentAsync(history, settings, kernel, cancellationToken).ConfigureAwait(false);
        return reply.Content?.Trim() ?? string.Empty;
    }

    public async IAsyncEnumerable<AssistantEvent> SendAsync(string message, IReadOnlyList<ImageAttachment> images, [EnumeratorCancellation] CancellationToken cancellationToken)
    {
        var settings = _settings();
        var profile = ActiveProfile;
        if (profile is null)
        {
            yield return new AssistantEvent(AssistantEventKind.Error, "No model profile is configured. Open Settings and add one.");
            yield break;
        }

        Kernel? kernel = null;
        ToolEventFilter? filter = null;
        string? configError = null;
        try
        {
            (kernel, filter) = GetKernel(profile);
        }
        catch (Exception ex) when (ex is LlmConfigurationException or UriFormatException or ArgumentException)
        {
            configError = ex.Message;
        }

        if (configError is not null || kernel is null || filter is null)
        {
            yield return new AssistantEvent(AssistantEventKind.Error, configError ?? "The model could not be initialized.");
            yield break;
        }

        var userEntry = new ChatEntry("user", message, images, DateTimeOffset.Now);
        var userContent = new ChatMessageContentItemCollection { new TextContent(message) };
        var attachImages = images.Count > 0 && profile.SupportsVision;
        if (attachImages)
        {
            foreach (var img in images)
            {
                userContent.Add(new ImageContent(img.Data, img.MediaType));
            }
        }

        ChatHistory working;
        lock (_gate)
        {
            _thread.Add(userEntry);
            _history.Add(new ChatMessageContent(AuthorRole.User, userContent));
            working = new ChatHistory(BuildSystemPrompt(settings));
            working.AddRange(_history);
        }

        if (images.Count > 0 && !attachImages)
        {
            yield return new AssistantEvent(AssistantEventKind.TextDelta, $"(Note: {profile.Name} does not accept images; they were not sent.)\n\n");
        }

        var channel = Channel.CreateUnbounded<AssistantEvent>(new UnboundedChannelOptions { SingleReader = true });
        filter.Reset();
        filter.MaxRounds = Math.Max(1, settings.MaxToolRounds);
        filter.MaxCalls = Math.Max(4, settings.MaxToolRounds * 4);
        filter.Sink = e =>
        {
            channel.Writer.TryWrite(e);
            _tools.Log(e.Kind == AssistantEventKind.ToolStarted ? "Jack → " + e.Text : "Jack ← " + e.Text);
        };

        var baseCount = working.Count;
        var reply = new StringBuilder();
        var producer = Task.Run(async () =>
        {
            try
            {
                var chat = kernel.GetRequiredService<IChatCompletionService>();
                var execution = KernelFactory.CreateSettings(profile, tools: true);
                await foreach (var chunk in chat.GetStreamingChatMessageContentsAsync(working, execution, kernel, cancellationToken).ConfigureAwait(false))
                {
                    if (!string.IsNullOrEmpty(chunk.Content))
                    {
                        reply.Append(chunk.Content);
                        channel.Writer.TryWrite(new AssistantEvent(AssistantEventKind.TextDelta, chunk.Content));
                    }
                }

                channel.Writer.TryWrite(new AssistantEvent(AssistantEventKind.Completed, reply.ToString()));
            }
            catch (OperationCanceledException)
            {
                channel.Writer.TryWrite(new AssistantEvent(AssistantEventKind.Error, "Stopped."));
            }
            catch (Exception ex)
            {
                channel.Writer.TryWrite(new AssistantEvent(AssistantEventKind.Error, Describe(ex, profile)));
            }
            finally
            {
                filter.Sink = null;
                channel.Writer.TryComplete();
            }
        }, CancellationToken.None);

        await foreach (var e in channel.Reader.ReadAllAsync(CancellationToken.None).ConfigureAwait(false))
        {
            yield return e;
        }

        await producer.ConfigureAwait(false);

        lock (_gate)
        {
            // Keep tool-call messages SK added to the working copy, then the final assistant text.
            for (var i = baseCount; i < working.Count; i++)
            {
                _history.Add(working[i]);
            }

            var text = reply.ToString();
            if (text.Length > 0)
            {
                _history.AddAssistantMessage(text);
                _thread.Add(new ChatEntry("assistant", text, [], DateTimeOffset.Now));
            }
        }
    }

    private (Kernel Kernel, ToolEventFilter Filter) GetKernel(LlmProfile profile)
    {
        var key = $"{profile.Provider}|{profile.Model}|{profile.Endpoint}|{profile.ApiKey}|{profile.Name}";
        lock (_gate)
        {
            if (_kernel is { } k && k.Key == key)
            {
                return (k.Kernel, k.Filter);
            }

            var filter = new ToolEventFilter();
            var kernel = KernelFactory.Create(profile, _tools, [filter]);
            _kernel = (key, kernel, filter);
            return (kernel, filter);
        }
    }

    private string BuildSystemPrompt(AppGenSettings settings)
    {
        var sb = new StringBuilder(string.IsNullOrWhiteSpace(settings.SystemPrompt) ? JackPrompt.Default : settings.SystemPrompt);
        sb.AppendLine().AppendLine().AppendLine("Current context:");
        sb.AppendLine($"- Date/time: {DateTimeOffset.Now:yyyy-MM-dd HH:mm zzz}; OS: {System.Runtime.InteropServices.RuntimeInformation.OSDescription}");
        sb.AppendLine($"- ROS2.Net package version: {settings.Ros2NetVersion}{(string.IsNullOrWhiteSpace(settings.PackageSource) ? string.Empty : $" (local feed: {settings.PackageSource})")}");
        var root = _tools.Workspace.RootPath;
        sb.AppendLine(root is null ? "- No project is open. Create one with build.create_project or build.create_project_from_template when the user asks for an app." : $"- Open project: {root}");
        if (!string.IsNullOrEmpty(_tools.Workspace.ActiveFile))
        {
            sb.AppendLine($"- File open in the editor: {_tools.Workspace.ActiveFile}");
        }

        if (!string.IsNullOrEmpty(_tools.Workspace.ActiveSelection))
        {
            sb.AppendLine($"- The user has text selected in the editor (call project.get_selection to read it).");
        }

        return sb.ToString();
    }

    private static string Describe(Exception ex, LlmProfile profile)
    {
        var inner = ex;
        while (inner.InnerException is not null && inner is not HttpOperationException)
        {
            inner = inner.InnerException;
        }

        if (inner is HttpOperationException http)
        {
            var detail = http.ResponseContent is { Length: > 0 } c ? c.Length > 400 ? c[..400] : c : http.Message;
            return http.StatusCode switch
            {
                HttpStatusCode.Unauthorized or HttpStatusCode.Forbidden => $"{profile.Name}: the API key was rejected ({(int)http.StatusCode}). Check it in Settings.",
                HttpStatusCode.NotFound => $"{profile.Name}: model or endpoint not found ({profile.Model} at {profile.Endpoint}). {detail}",
                HttpStatusCode.TooManyRequests => $"{profile.Name}: rate limited, try again shortly. {detail}",
                _ => $"{profile.Name}: {(int?)http.StatusCode} {detail}",
            };
        }

        if (inner is HttpRequestException hre)
        {
            return $"{profile.Name}: cannot reach the endpoint ({hre.Message}).";
        }

        return $"{profile.Name}: {ex.GetType().Name}: {ex.Message}";
    }
}
