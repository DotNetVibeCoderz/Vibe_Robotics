using System.CommandLine;
using ROS2.Net.Cli.Commands;

namespace ROS2.Net.Cli;

/// <summary>Builds and runs the <c>ros2net</c> command tree.</summary>
public static class CliApp
{
    /// <summary>Runs the CLI. Output goes to <paramref name="output"/> (stdout when null), errors to <paramref name="error"/>.</summary>
    public static async Task<int> RunAsync(string[] args, TextWriter? output = null, TextWriter? error = null, CancellationToken cancellationToken = default)
    {
        var globals = new GlobalOptions();
        var root = BuildRoot(globals, output ?? Console.Out, error ?? Console.Error);
        var parse = root.Parse(args);
        var config = new InvocationConfiguration
        {
            Output = output ?? Console.Out,
            Error = error ?? Console.Error,
        };
        try
        {
            return await parse.InvokeAsync(config, cancellationToken).ConfigureAwait(false);
        }
        catch (OperationCanceledException)
        {
            return 130;
        }
    }

    internal static RootCommand BuildRoot(GlobalOptions globals, TextWriter output, TextWriter error)
    {
        var root = new RootCommand(
            "ros2net — the ROS2.Net command-line tool (made by Gravicode Studios, led by Kang Fadhil).\n" +
            "Works without a ROS 2 installation: talks DDS-RTPS, rosbridge or the in-process middleware.");
        globals.AddTo(root);

        var env = new CliEnvironment(globals, output, error);
        root.Add(TopicCommands.Build(env));
        root.Add(NodeCommands.Build(env));
        root.Add(ServiceCommands.Build(env));
        root.Add(ActionCommands.Build(env));
        root.Add(ParamCommands.Build(env));
        root.Add(InterfaceCommands.Build(env));
        root.Add(BagCommands.Build(env));
        root.Add(SecurityCommands.Build(env));
        root.Add(DoctorCommand.Build(env));
        root.Add(DemoCommands.Build(env));
        return root;
    }
}
