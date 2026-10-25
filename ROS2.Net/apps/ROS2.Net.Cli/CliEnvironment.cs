using System.CommandLine;
using ROS2.Net.Msgs;
using ROS2.Net.Rmw.InProcess;
using ROS2.Net.Rmw.Rosbridge;
using ROS2.Net.Rmw.Rtps;

namespace ROS2.Net.Cli;

/// <summary>Options shared by every command (<c>--rmw</c>, <c>--domain</c>, ...).</summary>
internal sealed class GlobalOptions
{
    public Option<string?> Rmw { get; } = new("--rmw")
    {
        Description = "Middleware: rtps (default, talks to ROS 2 DDS networks), rosbridge or inprocess. Defaults to ROS2NET_RMW.",
        Recursive = true,
    };

    public Option<int?> Domain { get; } = new("--domain")
    {
        Description = "ROS domain id (defaults to ROS_DOMAIN_ID, else 0).",
        Recursive = true,
    };

    public Option<string?> Url { get; } = new("--url")
    {
        Description = "rosbridge WebSocket URL (implies --rmw rosbridge), e.g. ws://robot:9090.",
        Recursive = true,
    };

    public Option<bool> Localhost { get; } = new("--localhost")
    {
        Description = "Restrict discovery to this machine.",
        Recursive = true,
    };

    public Option<double?> SpinTime { get; } = new("--spin-time")
    {
        Description = "Seconds to wait for discovery before reading the graph (default: 2 for rtps, 1 for rosbridge, 0 in-process).",
        Recursive = true,
    };

    public void AddTo(Command root)
    {
        root.Add(Rmw);
        root.Add(Domain);
        root.Add(Url);
        root.Add(Localhost);
        root.Add(SpinTime);
    }
}

/// <summary>Per-invocation services for commands: output writers and middleware sessions.</summary>
internal sealed class CliEnvironment(GlobalOptions globals, TextWriter output, TextWriter error)
{
    public GlobalOptions Globals { get; } = globals;

    public TextWriter Out { get; } = output;

    public TextWriter Error { get; } = error;

    /// <summary>Starts a context and a hidden helper node, then waits for discovery.</summary>
    public async Task<CliSession> OpenAsync(ParseResult parse, CancellationToken cancellationToken, bool waitForDiscovery = true)
    {
        MessageExtensions.EnsureRegistered();
        var url = parse.GetValue(Globals.Url);
        var rmw = (parse.GetValue(Globals.Rmw) ?? (url is not null ? "rosbridge" : Environment.GetEnvironmentVariable("ROS2NET_RMW")) ?? "rtps").ToLowerInvariant();
        var localhost = parse.GetValue(Globals.Localhost);
        var options = new Ros2ContextOptions { DomainId = parse.GetValue(Globals.Domain) };
        if (localhost)
        {
            options.LocalhostOnly = true;
        }

        switch (rmw)
        {
            case "inprocess":
                options.Rmw = new InProcessRmwFactory();
                break;
            case "rtps":
                options.UseRtps(o =>
                {
                    if (localhost)
                    {
                        o.LocalhostOnly = true;
                    }
                });
                break;
            case "rosbridge":
                options.UseRosbridge(url ?? "ws://localhost:9090");
                break;
            default:
                throw new CliException($"Unknown middleware '{rmw}'. Use rtps, rosbridge or inprocess.");
        }

        var context = await Ros2Context.CreateAsync(options, cancellationToken).ConfigureAwait(false);
        var node = context.CreateNode($"_ros2net_cli_{Environment.ProcessId}_{Random.Shared.Next(100000)}", options: new NodeOptions
        {
            StartParameterServices = false,
            StartParameterEventPublisher = false,
            EnableRosout = false,
            UseGlobalArguments = false,
        });

        var session = new CliSession(context, node, rmw);
        var spin = parse.GetValue(Globals.SpinTime) ?? rmw switch
        {
            "rtps" => 2.0,
            "rosbridge" => 1.0,
            _ => 0.0,
        };
        if (waitForDiscovery && spin > 0)
        {
            await Task.Delay(TimeSpan.FromSeconds(spin), cancellationToken).ConfigureAwait(false);
        }

        return session;
    }
}

/// <summary>A running context plus the CLI's own (hidden) node.</summary>
internal sealed class CliSession(Ros2Context context, Node node, string rmw) : IAsyncDisposable
{
    public Ros2Context Context { get; } = context;

    public Node Node { get; } = node;

    public string Rmw { get; } = rmw;

    /// <summary>Waits until a topic shows up in the graph and returns its type.</summary>
    public async Task<string?> WaitForTopicTypeAsync(string topic, TimeSpan timeout, CancellationToken ct, bool services = false)
    {
        var fqn = Node.ResolveTopicName(topic);
        var deadline = DateTime.UtcNow + timeout;
        while (true)
        {
            var list = services ? Node.GetServiceNamesAndTypes(includeHidden: true) : Node.GetTopicNamesAndTypes(includeHidden: true);
            var found = list.FirstOrDefault(t => t.Name == fqn);
            if (found is { Types.Count: > 0 })
            {
                return found.Types[0];
            }

            if (DateTime.UtcNow >= deadline)
            {
                return null;
            }

            await Task.Delay(50, ct).ConfigureAwait(false);
        }
    }

    public async ValueTask DisposeAsync() => await Context.DisposeAsync().ConfigureAwait(false);
}

/// <summary>A user-facing error: printed without a stack trace.</summary>
internal sealed class CliException(string message) : Exception(message);
