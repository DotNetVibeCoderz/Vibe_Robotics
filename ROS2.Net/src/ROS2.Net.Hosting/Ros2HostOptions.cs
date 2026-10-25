using ROS2.Net.Rmw;

namespace ROS2.Net.Hosting;

/// <summary>Which executor the hosted service spins the context's nodes on.</summary>
public enum Ros2ExecutorKind
{
    SingleThreaded,
    MultiThreaded,
    Realtime,
}

/// <summary>
/// Options for <see cref="Ros2ServiceCollectionExtensions.AddRos2(Microsoft.Extensions.DependencyInjection.IServiceCollection, Action{Ros2HostOptions}?)"/>.
/// Bindable from configuration (e.g. an <c>appsettings.json</c> <c>"Ros2"</c> section).
/// </summary>
public sealed class Ros2HostOptions
{
    /// <summary>DDS domain id. Null uses <c>ROS_DOMAIN_ID</c> (else 0).</summary>
    public int? DomainId { get; set; }

    /// <summary>
    /// Middleware identifier (<c>inprocess</c>, <c>rtps</c>, <c>rosbridge</c>). The middleware's assembly must be
    /// referenced and loaded so it has registered itself; null uses <c>ROS2NET_RMW</c>, else in-process.
    /// </summary>
    public string? Rmw { get; set; }

    /// <summary>A middleware factory, taking precedence over <see cref="Rmw"/>.</summary>
    public IRmwFactory? RmwFactory { get; set; }

    /// <summary>Middleware-specific settings, e.g. <c>url</c> for rosbridge.</summary>
    public Dictionary<string, string> RmwSettings { get; set; } = new(StringComparer.OrdinalIgnoreCase);

    public bool UseIntraProcessComms { get; set; }

    /// <summary><c>--ros-args</c> style arguments (remaps, parameters, log levels).</summary>
    public List<string> Arguments { get; set; } = [];

    public bool? LocalhostOnly { get; set; }

    public string? Enclave { get; set; }

    public Ros2ExecutorKind Executor { get; set; } = Ros2ExecutorKind.SingleThreaded;

    /// <summary>Worker count for <see cref="Ros2ExecutorKind.MultiThreaded"/> (0 = processor count).</summary>
    public int ThreadCount { get; set; }

    /// <summary>Stop the application when the ROS context shuts down (e.g. a node calls <c>Context.Shutdown()</c>).</summary>
    public bool StopApplicationOnShutdown { get; set; } = true;

    /// <summary>Last chance to adjust the context options in code.</summary>
    public Action<Ros2ContextOptions>? ConfigureContext { get; set; }

    internal Executor CreateExecutor() => Executor switch
    {
        Ros2ExecutorKind.MultiThreaded => new MultiThreadedExecutor(ThreadCount),
        Ros2ExecutorKind.Realtime => new RealtimeExecutor(),
        _ => new SingleThreadedExecutor(),
    };
}
