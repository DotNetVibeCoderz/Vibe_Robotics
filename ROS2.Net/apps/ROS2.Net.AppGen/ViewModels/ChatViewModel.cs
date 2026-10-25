using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Text;
using ROS2.Net.AppGen;

namespace Ros2AppGen.ViewModels;

/// <summary>One kernel function call made while Jack answered.</summary>
public sealed class ToolActivity
{
    private readonly Stopwatch _watch = Stopwatch.StartNew();

    public ToolActivity(string name, string detail)
    {
        Name = name;
        Detail = detail;
    }

    public string Name { get; }

    public string Detail { get; }

    public bool IsDone { get; private set; }

    public string? Result { get; private set; }

    public TimeSpan Elapsed => _watch.Elapsed;

    internal void Complete(string result)
    {
        _watch.Stop();
        IsDone = true;
        Result = result;
    }
}

/// <summary>A message in the chat thread as the UI shows it.</summary>
public sealed class ChatMessage
{
    private readonly StringBuilder _text = new();

    public ChatMessage(string role, string text = "", IReadOnlyList<ImageAttachment>? images = null)
    {
        Role = role;
        _text.Append(text);
        Images = images ?? [];
        Time = DateTimeOffset.Now;
    }

    /// <summary><c>user</c>, <c>assistant</c> or <c>system</c> (errors and notices).</summary>
    public string Role { get; }

    public string Text => _text.ToString();

    public IReadOnlyList<ImageAttachment> Images { get; }

    public List<ToolActivity> Tools { get; } = [];

    public bool IsStreaming { get; internal set; }

    public bool IsError { get; internal set; }

    public DateTimeOffset Time { get; }

    /// <summary>Raised when text, tools or state change.</summary>
    public event Action? Changed;

    internal void Append(string delta)
    {
        _text.Append(delta);
        Changed?.Invoke();
    }

    internal void Notify() => Changed?.Invoke();
}

/// <summary>Drives the chat panel: sends messages to Jack and turns the event stream into messages.</summary>
public sealed class ChatViewModel
{
    private readonly AppGenHost _host;
    private CancellationTokenSource? _cts;

    public ChatViewModel(AppGenHost host) => _host = host;

    public ObservableCollection<ChatMessage> Messages { get; } = [];

    /// <summary>Images waiting to be sent with the next message.</summary>
    public ObservableCollection<ImageAttachment> PendingImages { get; } = [];

    public bool IsBusy { get; private set; }

    /// <summary>Short description of what Jack is doing right now, for the status bar.</summary>
    public string Activity { get; private set; } = "Idle";

    /// <summary>Raised when <see cref="IsBusy"/> or <see cref="Activity"/> change.</summary>
    public event Action? StateChanged;

    /// <summary>Raised for each tool call or error, for the Jack log tab.</summary>
    public event Action<string>? ToolLog;

    public string AssistantName => _host.Assistant.Name;

    public void AttachImage(string path)
    {
        var bytes = File.ReadAllBytes(path);
        var ext = Path.GetExtension(path).ToLowerInvariant();
        var media = ext switch
        {
            ".jpg" or ".jpeg" => "image/jpeg",
            ".gif" => "image/gif",
            ".webp" => "image/webp",
            ".bmp" => "image/bmp",
            _ => "image/png",
        };
        PendingImages.Add(new ImageAttachment(Path.GetFileName(path), media, bytes));
    }

    public void AttachImage(ImageAttachment image) => PendingImages.Add(image);

    /// <summary>Sends <paramref name="text"/> with the pending images and streams Jack's reply.</summary>
    public async Task SendAsync(string text)
    {
        text = text.Trim();
        if (IsBusy || (text.Length == 0 && PendingImages.Count == 0))
        {
            return;
        }

        var images = PendingImages.ToList();
        PendingImages.Clear();
        Messages.Add(new ChatMessage("user", text, images));
        var reply = new ChatMessage("assistant") { IsStreaming = true };
        Messages.Add(reply);

        _cts = new CancellationTokenSource();
        SetState(true, "Thinking…");
        try
        {
            await foreach (var e in _host.Assistant.SendAsync(text, images, _cts.Token))
            {
                switch (e.Kind)
                {
                    case AssistantEventKind.TextDelta:
                        if (Activity != "Writing…")
                        {
                            SetState(true, "Writing…");
                        }

                        reply.Append(e.Text);
                        break;
                    case AssistantEventKind.ToolStarted:
                        reply.Tools.Add(new ToolActivity(e.ToolName ?? e.Text, e.Text));
                        reply.Notify();
                        SetState(true, $"Running {e.ToolName ?? e.Text}");
                        ToolLog?.Invoke($"▶ {e.Text}");
                        break;
                    case AssistantEventKind.ToolCompleted:
                        var running = reply.Tools.LastOrDefault(t => !t.IsDone && (e.ToolName is null || t.Name == e.ToolName))
                                      ?? reply.Tools.LastOrDefault(t => !t.IsDone);
                        running?.Complete(e.Text);
                        reply.Notify();
                        ToolLog?.Invoke($"✓ {e.ToolName ?? running?.Name}: {e.Text}");
                        SetState(true, "Thinking…");
                        break;
                    case AssistantEventKind.Error:
                        reply.IsError = true;
                        reply.Append((reply.Text.Length > 0 ? "\n\n" : string.Empty) + "⚠ " + e.Text);
                        ToolLog?.Invoke($"✗ {e.Text}");
                        break;
                    case AssistantEventKind.Completed:
                        break;
                }
            }
        }
        catch (OperationCanceledException)
        {
            reply.Append(reply.Text.Length > 0 ? "\n\n_(stopped)_" : "_(stopped)_");
        }
        catch (Exception ex)
        {
            reply.IsError = true;
            reply.Append((reply.Text.Length > 0 ? "\n\n" : string.Empty) + "⚠ " + ex.Message);
            ToolLog?.Invoke($"✗ {ex.Message}");
        }
        finally
        {
            foreach (var t in reply.Tools.Where(t => !t.IsDone))
            {
                t.Complete("(no result)");
            }

            reply.IsStreaming = false;
            reply.Notify();
            _cts.Dispose();
            _cts = null;
            SetState(false, "Idle");
        }
    }

    /// <summary>Stops the reply in progress.</summary>
    public void Stop() => _cts?.Cancel();

    /// <summary>Clears the thread here and in the assistant.</summary>
    public void Clear()
    {
        Stop();
        _host.Assistant.ClearThread();
        Messages.Clear();
        PendingImages.Clear();
    }

    public void UseProfile(string name)
    {
        _host.Assistant.UseProfile(name);
        _host.Settings.ActiveProfile = name;
        _host.SaveSettings();
    }

    private void SetState(bool busy, string activity)
    {
        IsBusy = busy;
        Activity = activity;
        StateChanged?.Invoke();
    }
}
