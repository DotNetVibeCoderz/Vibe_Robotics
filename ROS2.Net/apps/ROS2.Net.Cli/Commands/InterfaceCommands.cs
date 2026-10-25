using System.CommandLine;
using ROS2.Net.Msgs;
using ROS2.Net.Rosidl;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net interface ...</c>: browse compiled-in interface types and generate C# from .msg/.srv/.action files.</summary>
internal static class InterfaceCommands
{
    public static Command Build(CliEnvironment env)
    {
        var iface = new Command("interface", "Show and generate ROS 2 interface types.");

        var onlyMsgs = new Option<bool>("--only-msgs", "-m");
        var onlySrvs = new Option<bool>("--only-srvs", "-s");
        var onlyActions = new Option<bool>("--only-actions", "-a");
        var list = new Command("list", "List interface types known to ros2net.") { onlyMsgs, onlySrvs, onlyActions };
        list.Handle(env, (p, _) =>
        {
            MessageExtensions.EnsureRegistered();
            var any = p.GetValue(onlyMsgs) || p.GetValue(onlySrvs) || p.GetValue(onlyActions);
            if (!any || p.GetValue(onlyMsgs))
            {
                env.Out.WriteLine("Messages:");
                foreach (var m in RosTypeRegistry.Messages.Where(m => m.Name.Contains("/msg/", StringComparison.Ordinal)))
                {
                    env.Out.WriteLine($"    {m.Name}");
                }
            }

            if (!any || p.GetValue(onlySrvs))
            {
                env.Out.WriteLine("Services:");
                foreach (var s in RosTypeRegistry.Services.DistinctBy(s => s.Name))
                {
                    env.Out.WriteLine($"    {s.Name}");
                }
            }

            if (!any || p.GetValue(onlyActions))
            {
                env.Out.WriteLine("Actions:");
                foreach (var a in RosTypeRegistry.Actions.DistinctBy(a => a.Name))
                {
                    env.Out.WriteLine($"    {a.Name}");
                }
            }

            return Task.FromResult(0);
        });

        var pkgName = new Argument<string>("package_name");
        var package = new Command("package", "List the interfaces of a package.") { pkgName };
        package.Handle(env, (p, _) =>
        {
            MessageExtensions.EnsureRegistered();
            var prefix = p.GetValue(pkgName) + "/";
            var names = RosTypeRegistry.Messages.Where(m => m.Name.StartsWith(prefix, StringComparison.Ordinal) && m.Name.Contains("/msg/", StringComparison.Ordinal)).Select(m => m.Name)
                .Concat(RosTypeRegistry.Services.Select(s => s.Name).Where(n => n.StartsWith(prefix, StringComparison.Ordinal)))
                .Concat(RosTypeRegistry.Actions.Select(a => a.Name).Where(n => n.StartsWith(prefix, StringComparison.Ordinal)))
                .Distinct()
                .Order(StringComparer.Ordinal)
                .ToList();
            if (names.Count == 0)
            {
                throw new CliException($"Unknown package '{p.GetValue(pkgName)}'.");
            }

            names.ForEach(env.Out.WriteLine);
            return Task.FromResult(0);
        });

        var typeName = new Argument<string>("type") { Description = "e.g. geometry_msgs/msg/Twist, std_srvs/srv/SetBool, example_interfaces/action/Fibonacci" };
        var show = new Command("show", "Print an interface definition.") { typeName };
        show.Handle(env, (p, _) =>
        {
            env.Out.Write(Show(p.GetValue(typeName)!));
            return Task.FromResult(0);
        });

        var input = new Argument<string>("input_directory") { Description = "Folder containing <package>/msg|srv|action/*.{msg,srv,action}." };
        var output = new Option<string>("--output", "-o") { Description = "Folder for the generated .cs files.", DefaultValueFactory = _ => "Generated" };
        var rootNs = new Option<string>("--root-namespace") { Description = "Root namespace of generated packages.", DefaultValueFactory = _ => Naming.DefaultRootNamespace };
        var generate = new Command("generate", "Generate C# sources from interface files (no source generator needed).") { input, output, rootNs };
        generate.Handle(env, async (p, ct) =>
        {
            var count = await GenerateAsync(p.GetValue(input)!, p.GetValue(output)!, p.GetValue(rootNs)!, env.Out, ct);
            env.Out.WriteLine($"Generated {count} interface file(s) into {Path.GetFullPath(p.GetValue(output)!)}");
            return count > 0 ? 0 : 1;
        });

        iface.Add(list);
        iface.Add(package);
        iface.Add(show);
        iface.Add(generate);
        return iface;
    }

    /// <summary>The definition text of a message, service or action type.</summary>
    internal static string Show(string type)
    {
        MessageExtensions.EnsureRegistered();
        var parts = type.Split('/');
        var kind = parts.Length == 3 ? parts[1] : null;
        var shortName = parts.Length == 3 ? $"{parts[0]}/{parts[2]}" : type;

        if (kind is null or "srv" && RosTypeRegistry.TryGetService(parts.Length == 3 ? type : $"{parts[0]}/srv/{parts[^1]}", out var svc))
        {
            return Text(svc.Request.Definition) + "---\n" + Text(svc.Response.Definition);
        }

        if (kind is null or "action" && RosTypeRegistry.TryGetAction(parts.Length == 3 ? type : $"{parts[0]}/action/{parts[^1]}", out var act))
        {
            return Text(act.Goal.Definition) + "---\n" + Text(act.Result.Definition) + "---\n" + Text(act.Feedback.Definition);
        }

        if (RosTypeRegistry.TryGetMessage(kind is null ? shortName : type, out var msg))
        {
            return Text(msg.Definition);
        }

        throw new CliException($"Unknown interface type '{type}'.");
    }

    private static string Text(string definition) => definition.Length == 0 || definition.EndsWith('\n') ? definition : definition + "\n";

    internal static async Task<int> GenerateAsync(string inputDirectory, string outputDirectory, string rootNamespace, TextWriter log, CancellationToken ct)
    {
        if (!Directory.Exists(inputDirectory))
        {
            throw new CliException($"Directory '{inputDirectory}' does not exist.");
        }

        Directory.CreateDirectory(outputDirectory);
        var options = new EmitOptions { RootNamespace = rootNamespace };
        var messages = new List<MessageDef>();
        var services = new List<ServiceDef>();
        var actions = new List<ActionDef>();
        var count = 0;
        foreach (var file in Directory.EnumerateFiles(inputDirectory, "*", SearchOption.AllDirectories).Order(StringComparer.Ordinal))
        {
            if (!RosidlParser.TryParsePath(file, out var pkg, out var kind, out var name))
            {
                continue;
            }

            var text = await File.ReadAllTextAsync(file, ct);
            string source;
            switch (kind)
            {
                case InterfaceKind.Message:
                    var m = RosidlParser.ParseMessage(pkg, name, text, file);
                    messages.Add(m);
                    source = CSharpEmitter.EmitMessage(m, options);
                    break;
                case InterfaceKind.Service:
                    var s = RosidlParser.ParseService(pkg, name, text, file);
                    services.Add(s);
                    source = CSharpEmitter.EmitService(s, options);
                    break;
                default:
                    var a = RosidlParser.ParseAction(pkg, name, text, file);
                    actions.Add(a);
                    source = CSharpEmitter.EmitAction(a, options);
                    break;
            }

            var outFile = Path.Combine(outputDirectory, $"{pkg}.{MessageDef.KindDirectory(kind)}.{name}.g.cs");
            await File.WriteAllTextAsync(outFile, source, ct);
            log.WriteLine($"  {pkg}/{MessageDef.KindDirectory(kind)}/{name} -> {Path.GetFileName(outFile)}");
            count++;
        }

        if (count > 0)
        {
            var asm = new DirectoryInfo(Path.GetFullPath(inputDirectory)).Name;
            await File.WriteAllTextAsync(Path.Combine(outputDirectory, "RosTypeRegistration.g.cs"),
                CSharpEmitter.EmitRegistration(asm, messages, services, actions, options), ct);
        }

        return count;
    }
}
