using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs.RclInterfaces;
using ROS2.Net.Msgs.RosgraphMsgs;
using ROS2.Net.Rmw;

namespace ROS2.Net;

/// <summary>
/// A ROS 2 node: the unit of composition that owns publishers, subscriptions, timers, services,
/// actions and parameters. Equivalent to <c>rclcpp::Node</c>; derive from it to build components.
/// </summary>
public class Node : IDisposable
{
    private readonly object _gate = new();
    private readonly HashSet<Waitable> _waitables = [];
    private readonly HashSet<IDisposable> _owned = [];
    private readonly ParameterStore _parameters;
    private readonly IReadOnlyList<RemapRule> _remaps;
    private Executor? _executor;
    private Publisher<ParameterEvent>? _parameterEvents;
    private Subscription<Clock>? _clockSubscription;
    private int _disposed;

    /// <summary>Creates a node. Prefer <see cref="Ros2Context.CreateNode"/> unless deriving.</summary>
    public Node(Ros2Context context, string name, NodeOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(context);
        Context = context;
        Options = options ?? new NodeOptions();

        var nodeArgs = RosArguments.Parse(Options.Arguments.Count > 0 ? ["--ros-args", .. Options.Arguments] : null);
        var remaps = new List<RemapRule>(Options.Remappings);
        remaps.AddRange(nodeArgs.Remappings);
        if (Options.UseGlobalArguments)
        {
            remaps.AddRange(context.Arguments.Remappings);
        }

        var ns = Options.Namespace;
        foreach (var r in remaps)
        {
            if (r.NodeName is { } target && target != name)
            {
                continue;
            }

            if (r.From == "__node" || r.From == "__name")
            {
                name = r.To;
            }
            else if (r.From == "__ns")
            {
                ns = r.To;
            }
        }

        RosNames.ValidateNodeName(name);
        Name = name;
        Namespace = RosNames.NormalizeNamespace(ns);
        FullyQualifiedName = RosNames.FullyQualifiedNodeName(Namespace, Name);
        _remaps = remaps;

        RmwNode = context.Rmw.CreateNode(Name, Namespace);
        DefaultCallbackGroup = new CallbackGroup(CallbackGroupType.MutuallyExclusive);
        Clock = new RosClock(ClockType.RosTime);

        var loggerName = (Namespace == "/" ? string.Empty : Namespace.TrimStart('/').Replace('/', '.') + ".") + Name;
        var level = (Options.UseGlobalArguments ? context.Arguments.LoggerLevels.GetValueOrDefault(loggerName, context.Arguments.LogLevel ?? LogLevel.Trace) : LogLevel.Trace);
        if (nodeArgs.LogLevel is { } nl)
        {
            level = nl;
        }

        Logger = new NodeLogger(context.LoggerFactory.CreateLogger(loggerName), loggerName, level);

        // Parameter overrides, lowest to highest precedence: global args, node args, explicit overrides.
        var overrides = new Dictionary<string, ParameterValue>(StringComparer.Ordinal);
        if (Options.UseGlobalArguments)
        {
            foreach (var kv in context.Arguments.ParametersFor(Name, FullyQualifiedName))
            {
                overrides[kv.Key] = kv.Value;
            }
        }

        foreach (var kv in nodeArgs.ParametersFor(Name, FullyQualifiedName))
        {
            overrides[kv.Key] = kv.Value;
        }

        foreach (var p in Options.ParameterOverrides)
        {
            overrides[p.Name] = p.Value;
        }

        _parameters = new ParameterStore(overrides, Options.AllowUndeclaredParameters);
        _parameters.Changed += OnParametersChanged;

        context.Register(this);

        if (Options.EnableRosout)
        {
            Logger.AttachRosout(CreatePublisher<Msgs.RclInterfaces.Log>("/rosout", QosProfile.Rosout));
        }

        if (Options.StartParameterEventPublisher)
        {
            _parameterEvents = CreatePublisher<ParameterEvent>("/parameter_events", Options.ParameterEventQos);
        }

        var useSimTime = DeclareParameter("use_sim_time", ParameterValue.From(false), new ParameterDescriptor
        {
            Description = "Follow /clock instead of the system clock.",
        });
        if (useSimTime.AsBool())
        {
            EnableSimTime();
        }

        if (Options.AutomaticallyDeclareParametersFromOverrides)
        {
            foreach (var (k, v) in overrides)
            {
                if (!_parameters.Has(k))
                {
                    _parameters.Declare(k, v, null, ignoreOverride: false);
                }
            }
        }

        if (Options.StartParameterServices)
        {
            ParameterServices.Install(this);
        }
    }

    public Ros2Context Context { get; }

    public NodeOptions Options { get; }

    /// <summary>Base name, e.g. <c>talker</c>.</summary>
    public string Name { get; }

    /// <summary>Namespace, e.g. <c>/robot1</c> or <c>/</c>.</summary>
    public string Namespace { get; }

    /// <summary>Fully qualified name, e.g. <c>/robot1/talker</c>.</summary>
    public string FullyQualifiedName { get; }

    /// <summary>The node's logger; also publishes to <c>/rosout</c> when enabled.</summary>
    public NodeLogger Logger { get; }

    /// <summary>The node's ROS clock (follows <c>/clock</c> when <c>use_sim_time</c> is true).</summary>
    public RosClock Clock { get; }

    public CallbackGroup DefaultCallbackGroup { get; }

    public bool IsDisposed => Volatile.Read(ref _disposed) != 0;

    internal IRmwNode RmwNode { get; }

    /// <summary>The executor spinning this node, if any.</summary>
    public Executor? Executor => Volatile.Read(ref _executor);

    // ---------------------------------------------------------------- names

    /// <summary>Expands and remaps a topic name the way this node sees it.</summary>
    public string ResolveTopicName(string name, bool remap = true)
    {
        var expanded = RosNames.ExpandTopicName(name, Name, Namespace);
        return remap ? RosNames.Remap(expanded, name, Name, Namespace, _remaps) : expanded;
    }

    /// <summary>Expands and remaps a service name the way this node sees it.</summary>
    public string ResolveServiceName(string name, bool remap = true) => ResolveTopicName(name, remap);

    // ---------------------------------------------------------------- publishers & subscriptions

    public Publisher<T> CreatePublisher<T>(string topic, QosProfile? qos = null, PublisherOptions? options = null)
        where T : class, IRosMessage<T>, new() =>
        Track(new Publisher<T>(this, ResolveTopicName(topic), qos ?? QosProfile.Default, options));

    public Subscription<T> CreateSubscription<T>(string topic, Action<T> callback, QosProfile? qos = null, SubscriptionOptions? options = null, CallbackGroup? group = null)
        where T : class, IRosMessage<T>, new()
    {
        ArgumentNullException.ThrowIfNull(callback);
        return CreateSubscriptionCore<T>(topic, (m, _) =>
        {
            callback(m);
            return ValueTask.CompletedTask;
        }, qos, options, group);
    }

    public Subscription<T> CreateSubscription<T>(string topic, Action<T, MessageInfo> callback, QosProfile? qos = null, SubscriptionOptions? options = null, CallbackGroup? group = null)
        where T : class, IRosMessage<T>, new()
    {
        ArgumentNullException.ThrowIfNull(callback);
        return CreateSubscriptionCore<T>(topic, (m, i) =>
        {
            callback(m, i);
            return ValueTask.CompletedTask;
        }, qos, options, group);
    }

    /// <summary>Subscription with an asynchronous callback; the executor awaits it before running the next callback of the same group.</summary>
    public Subscription<T> CreateSubscription<T>(string topic, Func<T, ValueTask> callback, QosProfile? qos = null, SubscriptionOptions? options = null, CallbackGroup? group = null)
        where T : class, IRosMessage<T>, new()
    {
        ArgumentNullException.ThrowIfNull(callback);
        return CreateSubscriptionCore<T>(topic, (m, _) => callback(m), qos, options, group);
    }

    /// <summary>
    /// Subscription without a callback: read it with <see cref="Subscription{T}.ReadAllAsync"/>
    /// (<c>await foreach</c>). No executor is needed.
    /// </summary>
    public Subscription<T> CreateSubscription<T>(string topic, QosProfile? qos = null, SubscriptionOptions? options = null)
        where T : class, IRosMessage<T>, new() =>
        Track(new Subscription<T>(this, ResolveTopicName(topic), qos ?? QosProfile.Default, null, options, null));

    private Subscription<T> CreateSubscriptionCore<T>(string topic, Func<T, MessageInfo, ValueTask> callback, QosProfile? qos, SubscriptionOptions? options, CallbackGroup? group)
        where T : class, IRosMessage<T>, new() =>
        Track(new Subscription<T>(this, ResolveTopicName(topic), qos ?? QosProfile.Default, callback, options, group));

    /// <summary>Publisher of raw CDR for a type known only at runtime.</summary>
    public SerializedPublisher CreateSerializedPublisher(string topic, MessageTypeInfo type, QosProfile? qos = null) =>
        Track(new SerializedPublisher(this, ResolveTopicName(topic), type, qos ?? QosProfile.Default));

    /// <summary>Subscription receiving raw CDR. The handler runs on the middleware's thread, not the executor.</summary>
    public SerializedSubscription CreateSerializedSubscription(string topic, MessageTypeInfo type, RmwSerializedHandler handler, QosProfile? qos = null) =>
        Track(new SerializedSubscription(this, ResolveTopicName(topic), type, qos ?? QosProfile.Default, handler));

    // ---------------------------------------------------------------- timers

    /// <summary>Creates a timer on the node's ROS clock (simulation-time aware).</summary>
    public RosTimer CreateTimer(TimeSpan period, Action callback, CallbackGroup? group = null, bool autostart = true)
    {
        ArgumentNullException.ThrowIfNull(callback);
        return CreateTimerCore(period, Clock, _ =>
        {
            callback();
            return ValueTask.CompletedTask;
        }, group, autostart);
    }

    public RosTimer CreateTimer(TimeSpan period, Action<TimerInfo> callback, CallbackGroup? group = null, bool autostart = true)
    {
        ArgumentNullException.ThrowIfNull(callback);
        return CreateTimerCore(period, Clock, i =>
        {
            callback(i);
            return ValueTask.CompletedTask;
        }, group, autostart);
    }

    public RosTimer CreateTimer(TimeSpan period, Func<ValueTask> callback, CallbackGroup? group = null, bool autostart = true)
    {
        ArgumentNullException.ThrowIfNull(callback);
        return CreateTimerCore(period, Clock, _ => callback(), group, autostart);
    }

    /// <summary>Creates a timer on the steady clock, unaffected by simulation time.</summary>
    public RosTimer CreateWallTimer(TimeSpan period, Action callback, CallbackGroup? group = null, bool autostart = true)
    {
        ArgumentNullException.ThrowIfNull(callback);
        return CreateTimerCore(period, new RosClock(ClockType.SteadyTime), _ =>
        {
            callback();
            return ValueTask.CompletedTask;
        }, group, autostart);
    }

    private RosTimer CreateTimerCore(TimeSpan period, RosClock clock, Func<TimerInfo, ValueTask> callback, CallbackGroup? group, bool autostart)
    {
        var timer = new RosTimer(this, period, clock, callback, group, autostart: false);
        Register(timer);
        if (autostart)
        {
            timer.Reset();
        }

        return timer;
    }

    public GuardCondition CreateGuardCondition(Action callback, CallbackGroup? group = null)
    {
        var gc = new GuardCondition(this, () =>
        {
            callback();
            return ValueTask.CompletedTask;
        }, group);
        Register(gc);
        return gc;
    }

    public CallbackGroup CreateCallbackGroup(CallbackGroupType type) => new(type);

    // ---------------------------------------------------------------- services

    /// <summary>Creates a service with a synchronous handler.</summary>
    public ServiceServer<TRequest, TResponse> CreateService<TRequest, TResponse>(string name, Func<TRequest, TResponse> handler, QosProfile? qos = null, CallbackGroup? group = null)
        where TRequest : class, IRosMessage<TRequest>, IRosServiceRequest<TRequest, TResponse>, new()
        where TResponse : class, IRosMessage<TResponse>, new()
    {
        ArgumentNullException.ThrowIfNull(handler);
        return CreateServiceCore<TRequest, TResponse>(name, ServiceInfo<TRequest, TResponse>(), (req, _, _) => ValueTask.FromResult(handler(req)), qos, group);
    }

    /// <summary>Creates a service with an asynchronous handler.</summary>
    public ServiceServer<TRequest, TResponse> CreateService<TRequest, TResponse>(string name, Func<TRequest, CancellationToken, ValueTask<TResponse>> handler, QosProfile? qos = null, CallbackGroup? group = null)
        where TRequest : class, IRosMessage<TRequest>, IRosServiceRequest<TRequest, TResponse>, new()
        where TResponse : class, IRosMessage<TResponse>, new()
    {
        ArgumentNullException.ThrowIfNull(handler);
        return CreateServiceCore<TRequest, TResponse>(name, ServiceInfo<TRequest, TResponse>(), (req, _, ct) => handler(req, ct), qos, group);
    }

    public ServiceClient<TRequest, TResponse> CreateClient<TRequest, TResponse>(string name, QosProfile? qos = null)
        where TRequest : class, IRosMessage<TRequest>, IRosServiceRequest<TRequest, TResponse>, new()
        where TResponse : class, IRosMessage<TResponse>, new() =>
        CreateClientCore<TRequest, TResponse>(name, ServiceInfo<TRequest, TResponse>(), qos);

    /// <summary>Creates a service whose wire types are only known at runtime (used by actions and bridges).</summary>
    public ServiceServer<TRequest, TResponse> CreateServiceCore<TRequest, TResponse>(string name, ServiceTypeInfo type, Func<TRequest, RequestId, CancellationToken, ValueTask<TResponse>> handler, QosProfile? qos = null, CallbackGroup? group = null)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new()
    {
        var server = new ServiceServer<TRequest, TResponse>(this, ResolveServiceName(name), type, qos ?? QosProfile.ServicesDefault, handler, group);
        Register(server);
        return server;
    }

    public ServiceClient<TRequest, TResponse> CreateClientCore<TRequest, TResponse>(string name, ServiceTypeInfo type, QosProfile? qos = null)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new() =>
        Track(new ServiceClient<TRequest, TResponse>(this, ResolveServiceName(name), type, qos ?? QosProfile.ServicesDefault));

    internal static ServiceTypeInfo ServiceInfo<TRequest, TResponse>()
        where TRequest : class, IRosMessage<TRequest>, IRosServiceRequest<TRequest, TResponse>, new()
        where TResponse : class, IRosMessage<TResponse>, new() =>
        new(TRequest.ServiceTypeName, MessageTypeInfo.Of<TRequest>(), MessageTypeInfo.Of<TResponse>());

    // ---------------------------------------------------------------- actions

    /// <summary>Creates an action server. <paramref name="execute"/> runs for each accepted goal; return the result, or throw <see cref="OperationCanceledException"/> after honoring a cancel request.</summary>
    public ActionServer<TGoal, TResult, TFeedback> CreateActionServer<TGoal, TResult, TFeedback>(
        string name,
        Func<ServerGoalHandle<TGoal, TResult, TFeedback>, CancellationToken, Task<TResult>> execute,
        ActionServerOptions<TGoal, TResult, TFeedback>? options = null)
        where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
        where TResult : class, IRosMessage<TResult>, new()
        where TFeedback : class, IRosMessage<TFeedback>, new() =>
        Track(new ActionServer<TGoal, TResult, TFeedback>(this, name, execute, options ?? new()));

    public ActionClient<TGoal, TResult, TFeedback> CreateActionClient<TGoal, TResult, TFeedback>(string name)
        where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
        where TResult : class, IRosMessage<TResult>, new()
        where TFeedback : class, IRosMessage<TFeedback>, new() =>
        Track(new ActionClient<TGoal, TResult, TFeedback>(this, name));

    // ---------------------------------------------------------------- parameters

    /// <summary>Raised after parameters are declared, changed or undeclared.</summary>
    public event Action<IReadOnlyList<ParameterChange>>? ParametersChanged;

    /// <summary>Declares a parameter; an override (command line, YAML, <see cref="NodeOptions.ParameterOverrides"/>) replaces the default.</summary>
    public ParameterValue DeclareParameter(string name, ParameterValue defaultValue, ParameterDescriptor? descriptor = null, bool ignoreOverride = false) =>
        _parameters.Declare(name, defaultValue, descriptor, ignoreOverride);

    /// <summary>Declares a parameter and returns its effective value as <typeparamref name="T"/>.</summary>
    public T DeclareParameter<T>(string name, T defaultValue, string? description = null, bool readOnly = false)
    {
        var value = _parameters.Declare(name, ParameterValue.FromObject(defaultValue), new ParameterDescriptor
        {
            Description = description ?? string.Empty,
            ReadOnly = readOnly,
        }, ignoreOverride: false);
        return value.As<T>();
    }

    /// <summary>Declares a parameter with a full descriptor (ranges, dynamic typing).</summary>
    public T DeclareParameter<T>(string name, T defaultValue, ParameterDescriptor descriptor) =>
        _parameters.Declare(name, ParameterValue.FromObject(defaultValue), descriptor, ignoreOverride: false).As<T>();

    public void UndeclareParameter(string name) => _parameters.Undeclare(name);

    public bool HasParameter(string name) => _parameters.Has(name);

    public ParameterValue GetParameter(string name) => _parameters.Get(name);

    public T GetParameter<T>(string name) => _parameters.Get(name).As<T>();

    public bool TryGetParameter(string name, out ParameterValue value) => _parameters.TryGet(name, out value);

    public T GetParameterOr<T>(string name, T fallback) => _parameters.TryGet(name, out var v) && v.IsSet ? v.As<T>() : fallback;

    public ParameterDescriptor DescribeParameter(string name) => _parameters.Describe(name);

    /// <summary>Every declared parameter.</summary>
    public IReadOnlyList<Parameter> Parameters => _parameters.All;

    public IReadOnlyList<string> ParameterNames => _parameters.Names;

    /// <summary>Overrides supplied for this node, declared or not.</summary>
    public IReadOnlyDictionary<string, ParameterValue> ParameterOverrides => _parameters.Overrides;

    public SetParametersResult SetParameter(Parameter parameter) => _parameters.Set([parameter], atomic: false)[0];

    public SetParametersResult SetParameter(string name, object? value) => SetParameter(new Parameter(name, value));

    public IReadOnlyList<SetParametersResult> SetParameters(IEnumerable<Parameter> parameters) =>
        _parameters.Set(parameters.ToList(), atomic: false);

    public SetParametersResult SetParametersAtomically(IEnumerable<Parameter> parameters)
    {
        var results = _parameters.Set(parameters.ToList(), atomic: true);
        return results.FirstOrDefault(r => !r.Successful) ?? SetParametersResult.Success;
    }

    /// <summary>
    /// Registers a validator that runs before parameters change (<c>add_on_set_parameters_callback</c>).
    /// Return a failure to reject the change. Dispose the result to remove it.
    /// </summary>
    public IDisposable AddOnSetParametersCallback(Func<IReadOnlyList<Parameter>, SetParametersResult> callback)
    {
        _parameters.AddValidator(callback);
        return new CallbackHandle(() => _parameters.RemoveValidator(callback));
    }

    /// <summary>Lists parameter names and prefixes under <paramref name="prefixes"/> to <paramref name="depth"/> levels (0 = unlimited).</summary>
    public (IReadOnlyList<string> Names, IReadOnlyList<string> Prefixes) ListParameters(IReadOnlyList<string> prefixes, ulong depth)
    {
        var names = new List<string>();
        var foundPrefixes = new SortedSet<string>(StringComparer.Ordinal);
        foreach (var name in _parameters.Names)
        {
            var matches = prefixes.Count == 0 || prefixes.Any(p => name == p || name.StartsWith(p + ".", StringComparison.Ordinal));
            if (!matches)
            {
                continue;
            }

            var dots = name.Count(c => c == '.');
            var baseDepth = prefixes.Count == 0 ? 0 : prefixes.Where(p => name.StartsWith(p, StringComparison.Ordinal)).Max(p => p.Count(c => c == '.') + 1);
            if (depth == 0 || (ulong)(dots - baseDepth + 1) <= depth || (prefixes.Count == 0 && (ulong)dots < depth))
            {
                names.Add(name);
                var last = name.LastIndexOf('.');
                if (last > 0)
                {
                    foundPrefixes.Add(name[..last]);
                }
            }
        }

        return (names, foundPrefixes.ToList());
    }

    private void OnParametersChanged(IReadOnlyList<ParameterChange> changes)
    {
        foreach (var c in changes)
        {
            if (c.Name == "use_sim_time" && c.NewValue.Type == ParameterType.Bool)
            {
                if (c.NewValue.AsBool())
                {
                    EnableSimTime();
                }
                else
                {
                    DisableSimTime();
                }
            }
        }

        if (_parameterEvents is { } pub)
        {
            var ev = new ParameterEvent
            {
                Stamp = Clock.Now().ToMsg(),
                Node = FullyQualifiedName,
                NewParameters = changes.Where(c => c.IsNew).Select(c => new Parameter(c.Name, c.NewValue).ToMsg()).ToArray(),
                ChangedParameters = changes.Where(c => !c.IsNew && !c.IsDeleted).Select(c => new Parameter(c.Name, c.NewValue).ToMsg()).ToArray(),
                DeletedParameters = changes.Where(c => c.IsDeleted).Select(c => new Parameter(c.Name, c.OldValue ?? ParameterValue.NotSet).ToMsg()).ToArray(),
            };
            pub.Publish(ev);
        }

        ParametersChanged?.Invoke(changes);
    }

    private void EnableSimTime()
    {
        if (Clock.IsSimTimeActive)
        {
            return;
        }

        Clock.EnableSimTime();
        _clockSubscription ??= CreateSubscription<Clock>("/clock", OnClock, QosProfile.SensorData.KeepLast(1), new SubscriptionOptions());
    }

    private void DisableSimTime()
    {
        Clock.DisableSimTime();
        _clockSubscription?.Dispose();
        _clockSubscription = null;
    }

    private void OnClock(Clock msg) => Clock.SetSimTime(RosTime.FromMsg(msg.ClockValue));

    // ---------------------------------------------------------------- graph

    /// <summary>Nodes visible in the graph.</summary>
    public IReadOnlyList<NodeInfo> GetNodeNames() => Context.Rmw.Graph.GetNodes();

    /// <summary>Topics with their types.</summary>
    public IReadOnlyList<TopicInfo> GetTopicNamesAndTypes(bool includeHidden = false) =>
        Group(Context.Rmw.Graph.GetEndpoints().Where(e => e.Kind is EndpointKind.Publisher or EndpointKind.Subscription), includeHidden);

    /// <summary>Services with their types.</summary>
    public IReadOnlyList<TopicInfo> GetServiceNamesAndTypes(bool includeHidden = false) =>
        Group(Context.Rmw.Graph.GetEndpoints().Where(e => e.Kind == EndpointKind.ServiceServer), includeHidden);

    public IReadOnlyList<EndpointInfo> GetPublishersInfo(string topic) => Endpoints(topic, EndpointKind.Publisher);

    public IReadOnlyList<EndpointInfo> GetSubscriptionsInfo(string topic) => Endpoints(topic, EndpointKind.Subscription);

    public int CountPublishers(string topic) => GetPublishersInfo(topic).Count;

    public int CountSubscribers(string topic) => GetSubscriptionsInfo(topic).Count;

    /// <summary>Endpoints owned by a node (for <c>node info</c>).</summary>
    public IReadOnlyList<EndpointInfo> GetNodeEndpoints(string nodeName, string nodeNamespace) =>
        Context.Rmw.Graph.GetEndpoints().Where(e => e.NodeName == nodeName && e.NodeNamespace == RosNames.NormalizeNamespace(nodeNamespace)).ToList();

    /// <summary>Waits until the graph changes or the timeout elapses.</summary>
    public async Task<bool> WaitForGraphChangeAsync(TimeSpan timeout, CancellationToken cancellationToken = default)
    {
        var tcs = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        void Handler() => tcs.TrySetResult();
        Context.Rmw.Graph.Changed += Handler;
        try
        {
            await tcs.Task.WaitAsync(timeout, cancellationToken).ConfigureAwait(false);
            return true;
        }
        catch (TimeoutException)
        {
            return false;
        }
        finally
        {
            Context.Rmw.Graph.Changed -= Handler;
        }
    }

    private IReadOnlyList<EndpointInfo> Endpoints(string topic, EndpointKind kind)
    {
        var fqn = ResolveTopicName(topic);
        return Context.Rmw.Graph.GetEndpoints().Where(e => e.Kind == kind && e.Name == fqn).ToList();
    }

    private static IReadOnlyList<TopicInfo> Group(IEnumerable<EndpointInfo> endpoints, bool includeHidden) =>
        endpoints.Where(e => includeHidden || !RosNames.IsHidden(e.Name))
            .GroupBy(e => e.Name, StringComparer.Ordinal)
            .OrderBy(g => g.Key, StringComparer.Ordinal)
            .Select(g => new TopicInfo(g.Key, g.Select(e => e.TypeName).Distinct().Order(StringComparer.Ordinal).ToList()))
            .ToList();

    // ---------------------------------------------------------------- spinning & lifetime

    /// <summary>Spins only this node on a single-threaded executor.</summary>
    public async Task SpinAsync(CancellationToken cancellationToken = default)
    {
        using var executor = new SingleThreadedExecutor();
        executor.AddNode(this);
        await executor.SpinAsync(cancellationToken).ConfigureAwait(false);
    }

    internal void AttachExecutor(Executor executor)
    {
        if (Interlocked.CompareExchange(ref _executor, executor, null) is { } existing && existing != executor)
        {
            throw new InvalidOperationException($"Node '{FullyQualifiedName}' is already added to another executor.");
        }

        List<Waitable> waitables;
        lock (_gate)
        {
            waitables = _waitables.ToList();
        }

        foreach (var w in waitables)
        {
            w.Rearm();
        }
    }

    internal void DetachExecutor(Executor executor) => Interlocked.CompareExchange(ref _executor, null, executor);

    internal void Schedule(Waitable waitable) => Volatile.Read(ref _executor)?.Enqueue(waitable);

    internal void Register(Waitable waitable)
    {
        lock (_gate)
        {
            _waitables.Add(waitable);
        }

        if (waitable.HasWork)
        {
            waitable.Rearm();
        }
    }

    internal void Unregister(Waitable waitable)
    {
        lock (_gate)
        {
            _waitables.Remove(waitable);
        }
    }

    internal T Track<T>(T disposable)
        where T : class, IDisposable
    {
        if (disposable is Waitable w)
        {
            Register(w);
            return disposable;
        }

        lock (_gate)
        {
            _owned.Add(disposable);
        }

        return disposable;
    }

    internal void Untrack(IDisposable disposable)
    {
        lock (_gate)
        {
            _owned.Remove(disposable);
        }
    }

    public override string ToString() => FullyQualifiedName;

    protected virtual void Dispose(bool disposing)
    {
        if (!disposing)
        {
            return;
        }

        Logger.DetachRosout();
        List<IDisposable> all;
        lock (_gate)
        {
            all = [.. _waitables, .. _owned];
            _waitables.Clear();
            _owned.Clear();
        }

        foreach (var d in all)
        {
            try
            {
                d.Dispose();
            }
            catch (ObjectDisposedException)
            {
            }
        }

        _executor?.RemoveNode(this);
        RmwNode.Dispose();
        Context.Unregister(this);
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
        {
            Dispose(true);
        }

        GC.SuppressFinalize(this);
    }

    private sealed class CallbackHandle(Action dispose) : IDisposable
    {
        private Action? _dispose = dispose;

        public void Dispose() => Interlocked.Exchange(ref _dispose, null)?.Invoke();
    }
}
