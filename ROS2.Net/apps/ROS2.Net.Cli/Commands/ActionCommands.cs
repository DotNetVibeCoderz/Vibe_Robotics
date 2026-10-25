using System.CommandLine;
using System.Reflection;
using System.Text.Json;
using ROS2.Net.Rmw;
using ROS2.Net.Serialization;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net action ...</c></summary>
internal static class ActionCommands
{
    private const string SendGoalSuffix = "/_action/send_goal";

    public static Command Build(CliEnvironment env)
    {
        var action = new Command("action", "Inspect and send goals to actions.");

        var showTypes = new Option<bool>("--show-types", "-t") { Description = "Show action types." };
        var list = new Command("list", "List actions.") { showTypes };
        list.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            var endpoints = s.Context.Rmw.Graph.GetEndpoints().Where(e => e.Kind == EndpointKind.ServiceServer);
            foreach (var (name, type) in FindActions(endpoints))
            {
                env.Out.WriteLine(p.GetValue(showTypes) ? $"{name} [{type}]" : name);
            }

            return 0;
        });

        var name = new Argument<string>("action_name");
        var type = new Argument<string?>("action_type") { Arity = ArgumentArity.ZeroOrOne, Description = "e.g. example_interfaces/action/Fibonacci (discovered when omitted)." };
        var goal = new Argument<string?>("goal") { Arity = ArgumentArity.ZeroOrOne, Description = "Goal as YAML or JSON." };
        var feedback = new Option<bool>("--feedback", "-f") { Description = "Print feedback messages." };
        var timeout = new Option<double>("--timeout") { Description = "Seconds to wait for the action server.", DefaultValueFactory = _ => 10 };
        var send = new Command("send_goal", "Send a goal and wait for the result.") { name, type, goal, feedback, timeout };
        send.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            var actionName = s.Node.ResolveTopicName(p.GetValue(name)!);
            var t = p.GetValue(type);
            var goalText = p.GetValue(goal);
            if (t is not null && (t.TrimStart().StartsWith('{') || !t.Contains('/')))
            {
                goalText = t;
                t = null;
            }

            if (t is null)
            {
                var svc = await s.WaitForTopicTypeAsync(actionName + SendGoalSuffix, TimeSpan.FromSeconds(p.GetValue(timeout)), ct, services: true)
                    ?? throw new CliException($"Could not determine the type of action '{actionName}'. Pass it explicitly.");
                t = svc.EndsWith("_SendGoal", StringComparison.Ordinal) ? svc[..^"_SendGoal".Length] : svc;
            }

            var normalized = t.Split('/') is { Length: 2 } parts ? $"{parts[0]}/action/{parts[1]}" : t;
            if (!RosTypeRegistry.TryGetAction(normalized, out var info))
            {
                throw new CliException($"Action type '{t}' is not known to ros2net (only compiled-in types can be used).");
            }

            var method = typeof(ActionCommands).GetMethod(nameof(SendTypedAsync), BindingFlags.NonPublic | BindingFlags.Static)!
                .MakeGenericMethod(info.Goal.ClrType, info.Result.ClrType, info.Feedback.ClrType);
            try
            {
                return await (Task<int>)method.Invoke(null, [env, s.Node, actionName, YamlJson.ToJson(goalText), p.GetValue(feedback), TimeSpan.FromSeconds(p.GetValue(timeout)), ct])!;
            }
            catch (TargetInvocationException ex) when (ex.InnerException is not null)
            {
                throw ex.InnerException;
            }
        });

        action.Add(list);
        action.Add(send);
        return action;
    }

    /// <summary>Actions inferred from their <c>_action/send_goal</c> service endpoints.</summary>
    internal static IEnumerable<(string Name, string Type)> FindActions(IEnumerable<EndpointInfo> serviceEndpoints) =>
        serviceEndpoints
            .Where(e => e.Name.EndsWith(SendGoalSuffix, StringComparison.Ordinal))
            .Select(e => (e.Name[..^SendGoalSuffix.Length], e.TypeName.EndsWith("_SendGoal", StringComparison.Ordinal) ? e.TypeName[..^"_SendGoal".Length] : e.TypeName))
            .Distinct()
            .OrderBy(a => a.Item1, StringComparer.Ordinal);

    private static async Task<int> SendTypedAsync<TGoal, TResult, TFeedback>(CliEnvironment env, Node node, string actionName, string goalJson, bool showFeedback, TimeSpan timeout, CancellationToken ct)
        where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
        where TResult : class, IRosMessage<TResult>, new()
        where TFeedback : class, IRosMessage<TFeedback>, new()
    {
        using var client = node.CreateActionClient<TGoal, TResult, TFeedback>(actionName);
        env.Out.WriteLine("Waiting for an action server to become available...");
        if (!await client.WaitForServerAsync(timeout, ct))
        {
            throw new CliException($"Action server '{actionName}' is not available.");
        }

        var goal = new TGoal();
        using (var doc = JsonDocument.Parse(goalJson))
        {
            goal.ReadJson(doc.RootElement);
        }

        env.Out.WriteLine($"Sending goal:\n{YamlJson.ToYaml(RosJson.ToJsonString(goal))}");
        ClientGoalHandle<TGoal, TResult, TFeedback> handle;
        try
        {
            handle = await client.SendGoalAsync(goal, showFeedback
                ? f =>
                {
                    lock (env.Out)
                    {
                        env.Out.WriteLine($"Feedback:\n{YamlJson.ToYaml(RosJson.ToJsonString(f))}");
                    }
                }
                : null, ct);
        }
        catch (GoalRejectedException)
        {
            env.Out.WriteLine("Goal was rejected.");
            return 1;
        }

        env.Out.WriteLine($"Goal accepted with ID: {handle.GoalId:N}");
        using var reg = ct.Register(() => _ = handle.CancelAsync(CancellationToken.None));
        var result = await handle.GetResultAsync(CancellationToken.None);
        lock (env.Out)
        {
            env.Out.WriteLine($"Result:\n{YamlJson.ToYaml(RosJson.ToJsonString(result.Result))}");
            env.Out.WriteLine($"Goal finished with status: {result.Status.ToString().ToUpperInvariant()}");
        }

        return result.Succeeded ? 0 : 1;
    }
}
