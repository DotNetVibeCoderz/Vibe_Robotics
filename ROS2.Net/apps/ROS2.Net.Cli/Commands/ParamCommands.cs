using System.CommandLine;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net param ...</c>, through the standard parameter services of the target node.</summary>
internal static class ParamCommands
{
    public static Command Build(CliEnvironment env)
    {
        var param = new Command("param", "Get, set, list, dump and load parameters of other nodes.");
        var timeout = new Option<double>("--timeout") { Description = "Seconds to wait for the node's parameter services.", DefaultValueFactory = _ => 5, Recursive = true };
        param.Add(timeout);

        var listNode = new Argument<string?>("node_name") { Arity = ArgumentArity.ZeroOrOne, Description = "Node to list; all nodes when omitted." };
        var list = new Command("list", "List parameter names.") { listNode };
        list.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: p.GetValue(listNode) is null);
            var targets = p.GetValue(listNode) is { } n
                ? [Fqn(n)]
                : s.Node.GetServiceNamesAndTypes().Where(x => x.Name.EndsWith("/list_parameters", StringComparison.Ordinal))
                    .Select(x => x.Name[..^"/list_parameters".Length]).Order(StringComparer.Ordinal).ToList();
            foreach (var target in targets)
            {
                using var client = await ConnectAsync(s.Node, target, p.GetValue(timeout), ct);
                env.Out.WriteLine($"{target}:");
                foreach (var name in (await client.ListAsync(cancellationToken: ct)).Order(StringComparer.Ordinal))
                {
                    env.Out.WriteLine($"  {name}");
                }
            }

            return 0;
        });

        var getNode = new Argument<string>("node_name");
        var getName = new Argument<string>("parameter_name");
        var get = new Command("get", "Print a parameter value.") { getNode, getName };
        get.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            using var client = await ConnectAsync(s.Node, Fqn(p.GetValue(getNode)!), p.GetValue(timeout), ct);
            var value = await client.GetAsync(p.GetValue(getName)!, ct);
            if (!value.IsSet)
            {
                env.Out.WriteLine("Parameter not set.");
                return 1;
            }

            env.Out.WriteLine($"{TypeLabel(value.Type)} value is: {value}");
            return 0;
        });

        var setNode = new Argument<string>("node_name");
        var setName = new Argument<string>("parameter_name");
        var setValue = new Argument<string>("value") { Description = "YAML value: 1, 2.5, true, hello, [1, 2]." };
        var set = new Command("set", "Set a parameter.") { setNode, setName, setValue };
        set.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            using var client = await ConnectAsync(s.Node, Fqn(p.GetValue(setNode)!), p.GetValue(timeout), ct);
            var result = (await client.SetAsync([new Parameter(p.GetValue(setName)!, ParameterValue.Parse(p.GetValue(setValue)!))], ct))[0];
            env.Out.WriteLine(result.Successful ? "Set parameter successful" : $"Setting parameter failed: {result.Reason}");
            return result.Successful ? 0 : 1;
        });

        var dumpNode = new Argument<string>("node_name");
        var output = new Option<string?>("--output", "-o") { Description = "Write to a file instead of stdout." };
        var dump = new Command("dump", "Print all parameters of a node as a ROS 2 parameter file.") { dumpNode, output };
        dump.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            var target = Fqn(p.GetValue(dumpNode)!);
            using var client = await ConnectAsync(s.Node, target, p.GetValue(timeout), ct);
            var all = await client.GetAsync(await client.ListAsync(cancellationToken: ct), ct);
            var yaml = ParameterYaml.Dump(target, all);
            if (p.GetValue(output) is { } file)
            {
                await File.WriteAllTextAsync(file, yaml, ct);
                env.Out.WriteLine($"Saved parameters to {file}");
            }
            else
            {
                env.Out.Write(yaml);
            }

            return 0;
        });

        var loadNode = new Argument<string>("node_name");
        var loadFile = new Argument<string>("parameter_file");
        var load = new Command("load", "Load parameters from a ROS 2 parameter file.") { loadNode, loadFile };
        load.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            var target = Fqn(p.GetValue(loadNode)!);
            var parameters = ParameterYaml.LoadFile(p.GetValue(loadFile)!, target);
            using var client = await ConnectAsync(s.Node, target, p.GetValue(timeout), ct);
            var results = await client.SetAsync(parameters, ct);
            var failed = 0;
            for (var i = 0; i < results.Count; i++)
            {
                env.Out.WriteLine(results[i].Successful
                    ? $"Set parameter {parameters[i].Name} successful"
                    : $"Set parameter {parameters[i].Name} failed: {results[i].Reason}");
                failed += results[i].Successful ? 0 : 1;
            }

            return failed == 0 ? 0 : 1;
        });

        var descNode = new Argument<string>("node_name");
        var descNames = new Argument<string[]>("parameter_names") { Arity = ArgumentArity.OneOrMore };
        var describe = new Command("describe", "Describe parameters.") { descNode, descNames };
        describe.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            using var client = await ConnectAsync(s.Node, Fqn(p.GetValue(descNode)!), p.GetValue(timeout), ct);
            foreach (var d in await client.DescribeAsync(p.GetValue(descNames)!, ct))
            {
                env.Out.WriteLine($"Parameter name: {d.Name}");
                env.Out.WriteLine($"  Type: {TypeLabel(d.Type).ToLowerInvariant()}");
                if (d.Description.Length > 0)
                {
                    env.Out.WriteLine($"  Description: {d.Description}");
                }

                env.Out.WriteLine($"  Read only: {d.ReadOnly}");
                if (d.FloatingPointRange is { } fr)
                {
                    env.Out.WriteLine($"  Floating point range: [{fr.From}, {fr.To}] step {fr.Step}");
                }

                if (d.IntegerRange is { } ir)
                {
                    env.Out.WriteLine($"  Integer range: [{ir.From}, {ir.To}] step {ir.Step}");
                }
            }

            return 0;
        });

        param.Add(list);
        param.Add(get);
        param.Add(set);
        param.Add(dump);
        param.Add(load);
        param.Add(describe);
        return param;
    }

    private static string Fqn(string node) => node.StartsWith('/') ? node : "/" + node;

    private static async Task<ParameterClient> ConnectAsync(Node node, string target, double timeout, CancellationToken ct)
    {
        var client = new ParameterClient(node, target) { Timeout = TimeSpan.FromSeconds(timeout) };
        if (!await client.WaitForServiceAsync(TimeSpan.FromSeconds(timeout), ct))
        {
            client.Dispose();
            throw new CliException($"Node '{target}' not found (no parameter services).");
        }

        return client;
    }

    private static string TypeLabel(ParameterType type) => type switch
    {
        ParameterType.Bool => "Boolean",
        ParameterType.Integer => "Integer",
        ParameterType.Double => "Double",
        ParameterType.String => "String",
        ParameterType.ByteArray => "Byte array",
        ParameterType.BoolArray => "Boolean array",
        ParameterType.IntegerArray => "Integer array",
        ParameterType.DoubleArray => "Double array",
        ParameterType.StringArray => "String array",
        _ => "Not set",
    };
}
