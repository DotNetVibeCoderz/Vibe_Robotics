using System.CommandLine;
using ROS2.Net.Rmw;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net node ...</c></summary>
internal static class NodeCommands
{
    public static Command Build(CliEnvironment env)
    {
        var node = new Command("node", "Inspect nodes.");

        var all = new Option<bool>("--all", "-a") { Description = "Include hidden nodes (names starting with '_')." };
        var count = new Option<bool>("--count-nodes", "-c") { Description = "Only print the number of nodes." };
        var list = new Command("list", "List nodes.") { all, count };
        list.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            var nodes = s.Node.GetNodeNames()
                .Where(n => p.GetValue(all) || !n.Name.StartsWith('_'))
                .Select(n => n.FullyQualifiedName)
                .Distinct()
                .Order(StringComparer.Ordinal)
                .ToList();
            if (p.GetValue(count))
            {
                env.Out.WriteLine(nodes.Count);
            }
            else
            {
                nodes.ForEach(env.Out.WriteLine);
            }

            return 0;
        });

        var name = new Argument<string>("node_name");
        var info = new Command("info", "Print the publishers, subscriptions, services and actions of a node.") { name };
        info.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            var fqn = p.GetValue(name)!;
            fqn = fqn.StartsWith('/') ? fqn : "/" + fqn;
            var target = s.Node.GetNodeNames().FirstOrDefault(n => n.FullyQualifiedName == fqn)
                ?? throw new CliException($"Unable to find node '{fqn}'.");
            var eps = s.Node.GetNodeEndpoints(target.Name, target.Namespace);
            env.Out.WriteLine(fqn);
            Section(env, "Subscribers", eps.Where(e => e.Kind == EndpointKind.Subscription && !e.Name.Contains("/_action/", StringComparison.Ordinal)));
            Section(env, "Publishers", eps.Where(e => e.Kind == EndpointKind.Publisher && !e.Name.Contains("/_action/", StringComparison.Ordinal)));
            Section(env, "Service Servers", eps.Where(e => e.Kind == EndpointKind.ServiceServer && !e.Name.Contains("/_action/", StringComparison.Ordinal)));
            Section(env, "Service Clients", eps.Where(e => e.Kind == EndpointKind.ServiceClient && !e.Name.Contains("/_action/", StringComparison.Ordinal)));
            Actions(env, "Action Servers", eps, EndpointKind.ServiceServer);
            Actions(env, "Action Clients", eps, EndpointKind.ServiceClient);
            return 0;
        });

        node.Add(list);
        node.Add(info);
        return node;
    }

    private static void Section(CliEnvironment env, string title, IEnumerable<EndpointInfo> endpoints)
    {
        env.Out.WriteLine($"  {title}:");
        foreach (var e in endpoints.DistinctBy(e => (e.Name, e.TypeName)).OrderBy(e => e.Name, StringComparer.Ordinal))
        {
            env.Out.WriteLine($"    {e.Name}: {e.TypeName}");
        }
    }

    private static void Actions(CliEnvironment env, string title, IEnumerable<EndpointInfo> endpoints, EndpointKind kind)
    {
        env.Out.WriteLine($"  {title}:");
        foreach (var (actionName, type) in ActionCommands.FindActions(endpoints.Where(e => e.Kind == kind)))
        {
            env.Out.WriteLine($"    {actionName}: {type}");
        }
    }
}
