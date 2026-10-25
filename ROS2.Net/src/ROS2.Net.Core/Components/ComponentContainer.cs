using System.Collections.Concurrent;
using System.Reflection;
using System.Runtime.Loader;
using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs.CompositionInterfaces;

namespace ROS2.Net;

/// <summary>
/// Marks a <see cref="Node"/> subclass as a loadable component. The class needs a public
/// constructor <c>(Ros2Context, NodeOptions)</c>.
/// </summary>
[AttributeUsage(AttributeTargets.Class, AllowMultiple = false, Inherited = false)]
public sealed class RosComponentAttribute(string? name = null) : Attribute
{
    /// <summary>Plugin name, e.g. <c>demo_nodes::Talker</c>. Defaults to the type's full name.</summary>
    public string? Name { get; } = name;
}

/// <summary>A component loaded into a container.</summary>
public sealed record LoadedComponent(ulong UniqueId, string FullNodeName, string PluginName, Node Node);

/// <summary>
/// Hosts component nodes in one process, like <c>rclcpp_components::ComponentManager</c>. Components
/// share the container's context, so with intra-process comms enabled they exchange messages by
/// reference. Exposes <c>~/_container/load_node</c>, <c>unload_node</c> and <c>list_nodes</c>, so
/// <c>ros2 component load</c> works against it.
/// </summary>
public sealed class ComponentContainer : Node
{
    private readonly ConcurrentDictionary<ulong, LoadedComponent> _components = new();
    private readonly ConcurrentDictionary<string, Type> _catalog = new(StringComparer.Ordinal);
    private long _nextId;

    public ComponentContainer(Ros2Context context, string name = "ComponentManager", NodeOptions? options = null)
        : base(context, name, options)
    {
        CreateService<LoadNodeRequest, LoadNodeResponse>("~/_container/load_node", HandleLoad);
        CreateService<UnloadNodeRequest, UnloadNodeResponse>("~/_container/unload_node", req =>
        {
            var ok = Unload(req.UniqueId);
            return new UnloadNodeResponse { Success = ok, ErrorMessage = ok ? string.Empty : $"No node found with unique_id: {req.UniqueId}" };
        });
        CreateService<ListNodesRequest, ListNodesResponse>("~/_container/list_nodes", _ =>
        {
            var list = _components.Values.OrderBy(c => c.UniqueId).ToList();
            return new ListNodesResponse
            {
                FullNodeNames = list.Select(c => c.FullNodeName).ToArray(),
                UniqueIds = list.Select(c => c.UniqueId).ToArray(),
            };
        });
    }

    public IReadOnlyList<LoadedComponent> Components => _components.Values.OrderBy(c => c.UniqueId).ToList();

    /// <summary>Makes every <see cref="RosComponentAttribute"/> type in <paramref name="assembly"/> loadable by plugin name.</summary>
    public void RegisterAssembly(Assembly assembly)
    {
        foreach (var type in assembly.GetExportedTypes())
        {
            if (type.GetCustomAttribute<RosComponentAttribute>() is { } attr && typeof(Node).IsAssignableFrom(type))
            {
                _catalog[attr.Name ?? type.FullName!] = type;
                _catalog[type.FullName!] = type;
            }
        }
    }

    /// <summary>Loads a component type directly.</summary>
    public LoadedComponent Load(Type componentType, NodeOptions? options = null)
    {
        if (!typeof(Node).IsAssignableFrom(componentType))
        {
            throw new ArgumentException($"{componentType} does not derive from Node.", nameof(componentType));
        }

        var node = (Node)(Activator.CreateInstance(componentType, Context, options ?? new NodeOptions())
            ?? throw new InvalidOperationException($"Could not create {componentType}."));
        var id = (ulong)Interlocked.Increment(ref _nextId);
        var plugin = componentType.GetCustomAttribute<RosComponentAttribute>()?.Name ?? componentType.FullName!;
        var loaded = new LoadedComponent(id, node.FullyQualifiedName, plugin, node);
        _components[id] = loaded;
        Executor?.AddNode(node);
        Logger.LogInformation("Loaded component {Plugin} as {Node} (id {Id})", plugin, node.FullyQualifiedName, id);
        return loaded;
    }

    public LoadedComponent Load<TComponent>(NodeOptions? options = null)
        where TComponent : Node => Load(typeof(TComponent), options);

    /// <summary>Loads a component by plugin name; <paramref name="package"/> may be an assembly name or a path to a .dll.</summary>
    public LoadedComponent Load(string package, string pluginName, NodeOptions? options = null)
    {
        if (!_catalog.TryGetValue(pluginName, out var type))
        {
            var assembly = package.EndsWith(".dll", StringComparison.OrdinalIgnoreCase) && File.Exists(package)
                ? AssemblyLoadContext.Default.LoadFromAssemblyPath(Path.GetFullPath(package))
                : Assembly.Load(new AssemblyName(package));
            RegisterAssembly(assembly);
            if (!_catalog.TryGetValue(pluginName, out type))
            {
                throw new InvalidOperationException($"Component '{pluginName}' not found in '{package}'.");
            }
        }

        return Load(type, options);
    }

    public bool Unload(ulong uniqueId)
    {
        if (!_components.TryRemove(uniqueId, out var c))
        {
            return false;
        }

        c.Node.Dispose();
        return true;
    }

    private LoadNodeResponse HandleLoad(LoadNodeRequest req)
    {
        try
        {
            var options = new NodeOptions();
            if (!string.IsNullOrEmpty(req.NodeNamespace))
            {
                options.Namespace = req.NodeNamespace;
            }

            if (!string.IsNullOrEmpty(req.NodeName))
            {
                options.Remappings.Add(new RemapRule("__node", req.NodeName));
            }

            foreach (var rule in req.RemapRules)
            {
                options.Remappings.Add(RemapRule.Parse(rule));
            }

            foreach (var p in req.Parameters)
            {
                options.ParameterOverrides.Add(Parameter.FromMsg(p));
            }

            foreach (var extra in req.ExtraArguments)
            {
                if (extra.Name == "use_intra_process_comms" && extra.Value.Type == (byte)ParameterType.Bool)
                {
                    options.UseIntraProcessComms = extra.Value.BoolValue;
                }
            }

            var loaded = Load(req.PackageName, req.PluginName, options);
            return new LoadNodeResponse { Success = true, FullNodeName = loaded.FullNodeName, UniqueId = loaded.UniqueId };
        }
        catch (Exception ex) when (ex is not OutOfMemoryException)
        {
            Logger.LogError(ex, "Failed to load component {Plugin}", req.PluginName);
            return new LoadNodeResponse { Success = false, ErrorMessage = ex.Message };
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            foreach (var id in _components.Keys.ToList())
            {
                Unload(id);
            }
        }

        base.Dispose(disposing);
    }
}
