using System.Text;
using System.Text.Json;
using Microsoft.SemanticKernel;

namespace ROS2.Net.AppGen.Ai;

/// <summary>
/// Reports every kernel function call as Started/Completed events and enforces the per-message tool
/// budget. Implemented as a function-invocation filter, so it sees calls made through both the SK
/// connectors and the Microsoft.Extensions.AI bridge (Claude).
/// </summary>
internal sealed class ToolEventFilter : IFunctionInvocationFilter, IAutoFunctionInvocationFilter
{
    private int _calls;

    public Action<AssistantEvent>? Sink { get; set; }

    /// <summary>Maximum tool calls per user message (0 = unlimited).</summary>
    public int MaxCalls { get; set; }

    /// <summary>Maximum model round trips per user message (SK connectors only).</summary>
    public int MaxRounds { get; set; }

    public void Reset() => Interlocked.Exchange(ref _calls, 0);

    public async Task OnFunctionInvocationAsync(FunctionInvocationContext context, Func<FunctionInvocationContext, Task> next)
    {
        var name = $"{context.Function.PluginName}.{context.Function.Name}";
        var calls = Interlocked.Increment(ref _calls);
        if (MaxCalls > 0 && calls > MaxCalls)
        {
            context.Result = new FunctionResult(context.Function, "Error: the tool budget for this message is exhausted. Stop calling tools and summarize what is done and what remains.");
            Sink?.Invoke(new AssistantEvent(AssistantEventKind.ToolCompleted, "skipped: tool budget exhausted", name));
            return;
        }

        Sink?.Invoke(new AssistantEvent(AssistantEventKind.ToolStarted, $"{name}({Summarize(context.Arguments)})", name));
        try
        {
            await next(context).ConfigureAwait(false);
            var text = context.Result.GetValue<object>()?.ToString() ?? string.Empty;
            Sink?.Invoke(new AssistantEvent(AssistantEventKind.ToolCompleted, FirstLine(text), name));
        }
        catch (Exception ex) when (ex is not OperationCanceledException)
        {
            // Return the error to the model instead of aborting the whole answer.
            context.Result = new FunctionResult(context.Function, $"Error: {ex.GetType().Name}: {ex.Message}");
            Sink?.Invoke(new AssistantEvent(AssistantEventKind.ToolCompleted, "error: " + ex.Message, name));
        }
    }

    public async Task OnAutoFunctionInvocationAsync(AutoFunctionInvocationContext context, Func<AutoFunctionInvocationContext, Task> next)
    {
        await next(context).ConfigureAwait(false);
        if (MaxRounds > 0 && context.RequestSequenceIndex + 1 >= MaxRounds)
        {
            context.Terminate = true;
        }
    }

    internal static string Summarize(KernelArguments? args)
    {
        if (args is null || args.Count == 0)
        {
            return string.Empty;
        }

        var sb = new StringBuilder();
        foreach (var (k, v) in args)
        {
            if (sb.Length > 0)
            {
                sb.Append(", ");
            }

            var text = v switch
            {
                null => "null",
                string s => s,
                _ => JsonSerializer.Serialize(v),
            };
            text = text.Replace('\n', ' ').Replace('\r', ' ');
            sb.Append(k).Append('=').Append(text.Length > 60 ? text[..60] + "…" : text);
        }

        return sb.ToString();
    }

    private static string FirstLine(string text)
    {
        var line = text.Split('\n', 2)[0].Trim();
        return line.Length > 160 ? line[..160] + "…" : line;
    }
}
