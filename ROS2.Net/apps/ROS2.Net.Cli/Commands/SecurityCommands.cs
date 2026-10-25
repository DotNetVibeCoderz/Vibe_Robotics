using System.CommandLine;
using ROS2.Net.Security;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net security ...</c>: SROS2 keystores in the <c>ros2 security</c> layout.</summary>
internal static class SecurityCommands
{
    public static Command Build(CliEnvironment env)
    {
        var security = new Command("security", "Create SROS2 keystores and enclaves.");

        var ksDir = new Argument<string>("keystore_path");
        var create = new Command("create_keystore", "Create a keystore (CA, governance).") { ksDir };
        create.Handle(env, (p, _) =>
        {
            var ks = SrosKeystore.Create(p.GetValue(ksDir)!);
            env.Out.WriteLine($"Created keystore at {ks.Root}");
            return Task.FromResult(0);
        });

        var encDir = new Argument<string>("keystore_path");
        var encName = new Argument<string>("enclave_name") { Description = "e.g. /talker_listener/talker" };
        var enclave = new Command("create_enclave", "Create an enclave (identity + permissions) in a keystore.") { encDir, encName };
        enclave.Handle(env, (p, _) =>
        {
            var dir = SrosKeystore.Open(p.GetValue(encDir)!).CreateEnclave(p.GetValue(encName)!);
            env.Out.WriteLine($"Created enclave at {dir}");
            return Task.FromResult(0);
        });

        var listDir = new Argument<string>("keystore_path");
        var list = new Command("list_enclaves", "List the enclaves of a keystore.") { listDir };
        list.Handle(env, (p, _) =>
        {
            foreach (var e in SrosKeystore.Open(p.GetValue(listDir)!).ListEnclaves())
            {
                env.Out.WriteLine(e);
            }

            return Task.FromResult(0);
        });

        security.Add(create);
        security.Add(enclave);
        security.Add(list);
        return security;
    }
}
