using System.Reflection;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.DependencyInjection.Extensions;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;
using ROS2.Net.Rmw;

namespace ROS2.Net.Hosting;

/// <summary>A node registered with <c>AddRos2Node</c>; created when the host starts.</summary>
public sealed record Ros2NodeRegistration(Func<IServiceProvider, Node> Factory);

/// <summary>Dependency injection registration for ROS2.Net.</summary>
public static class Ros2ServiceCollectionExtensions
{
    /// <summary>
    /// Registers a singleton <see cref="Ros2Context"/> (created at host start, using the host's
    /// <see cref="ILoggerFactory"/>) and a hosted service that spins every node of it until the host stops.
    /// </summary>
    public static IServiceCollection AddRos2(this IServiceCollection services, Action<Ros2HostOptions>? configure = null)
    {
        ArgumentNullException.ThrowIfNull(services);
        var builder = services.AddOptions<Ros2HostOptions>();
        if (configure is not null)
        {
            builder.Configure(configure);
        }

        services.TryAddSingleton(sp => CreateContext(sp.GetRequiredService<IOptions<Ros2HostOptions>>().Value, sp.GetService<ILoggerFactory>()));
        services.TryAddEnumerable(ServiceDescriptor.Singleton<IHostedService, Ros2HostedService>());
        return services;
    }

    /// <summary>Like <see cref="AddRos2(IServiceCollection, Action{Ros2HostOptions}?)"/>, binding options from a configuration section.</summary>
    public static IServiceCollection AddRos2(this IServiceCollection services, IConfiguration configuration, Action<Ros2HostOptions>? configure = null)
    {
        ArgumentNullException.ThrowIfNull(configuration);
        services.AddOptions<Ros2HostOptions>().Bind(configuration);
        return services.AddRos2(configure);
    }

    /// <summary>
    /// Registers a node type built from DI. Its constructor may take <see cref="Ros2Context"/>,
    /// <see cref="NodeOptions"/> (from <paramref name="options"/>) and any registered service.
    /// The node is created when the host starts and is also resolvable as <typeparamref name="TNode"/>.
    /// </summary>
    public static IServiceCollection AddRos2Node<TNode>(this IServiceCollection services, Func<IServiceProvider, NodeOptions>? options = null)
        where TNode : Node
    {
        services.AddSingleton(sp => CreateNode<TNode>(sp, options?.Invoke(sp) ?? new NodeOptions()));
        services.AddSingleton(new Ros2NodeRegistration(sp => sp.GetRequiredService<TNode>()));
        return services;
    }

    /// <summary>Registers a plain node configured by a callback (create publishers, timers... in <paramref name="configure"/>).</summary>
    public static IServiceCollection AddRos2Node(this IServiceCollection services, string name, Action<IServiceProvider, Node> configure, string? @namespace = null, NodeOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(configure);
        services.AddSingleton(new Ros2NodeRegistration(sp =>
        {
            var node = sp.GetRequiredService<Ros2Context>().CreateNode(name, @namespace, options);
            configure(sp, node);
            return node;
        }));
        return services;
    }

    internal static Ros2Context CreateContext(Ros2HostOptions host, ILoggerFactory? loggerFactory)
    {
        var options = new Ros2ContextOptions
        {
            DomainId = host.DomainId,
            UseIntraProcessComms = host.UseIntraProcessComms,
            LoggerFactory = loggerFactory,
            Arguments = host.Arguments.Count > 0 ? (host.Arguments[0] == "--ros-args" ? host.Arguments : ["--ros-args", .. host.Arguments]) : null,
            LocalhostOnly = host.LocalhostOnly,
            Enclave = host.Enclave,
        };
        foreach (var (k, v) in host.RmwSettings)
        {
            options.RmwSettings[k] = v;
        }

        if (host.RmwFactory is not null)
        {
            options.Rmw = host.RmwFactory;
        }
        else if (!string.IsNullOrWhiteSpace(host.Rmw))
        {
            options.Rmw = RmwRegistry.TryGet(host.Rmw, out var f)
                ? f
                : throw new InvalidOperationException($"Middleware '{host.Rmw}' is not registered (known: {string.Join(", ", RmwRegistry.Identifiers)}). Reference its package and load it (for example by calling its Use…() extension in ConfigureContext).");
        }

        host.ConfigureContext?.Invoke(options);
        return Ros2Context.Create(options);
    }

    private static TNode CreateNode<TNode>(IServiceProvider sp, NodeOptions nodeOptions)
        where TNode : Node
    {
        // Pass NodeOptions explicitly only when a constructor asks for it; ActivatorUtilities rejects unused arguments.
        var wantsOptions = typeof(TNode).GetConstructors(BindingFlags.Public | BindingFlags.Instance)
            .Any(c => c.GetParameters().Any(p => p.ParameterType == typeof(NodeOptions)));
        return wantsOptions
            ? ActivatorUtilities.CreateInstance<TNode>(sp, nodeOptions)
            : ActivatorUtilities.CreateInstance<TNode>(sp);
    }
}
