using System.Runtime.InteropServices;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Logging.Abstractions;
using ROS2.Net.Rmw;

namespace ROS2.Net;

/// <summary>Options for <see cref="Ros2Context"/>.</summary>
public sealed class Ros2ContextOptions
{
    /// <summary>DDS domain id. Defaults to <c>ROS_DOMAIN_ID</c>, else 0.</summary>
    public int? DomainId { get; set; }

    /// <summary>The middleware. Defaults to <see cref="RmwRegistry.GetDefault"/> (<c>ROS2NET_RMW</c>, else in-process).</summary>
    public IRmwFactory? Rmw { get; set; }

    /// <summary>Middleware-specific settings, e.g. <c>url</c> for rosbridge or <c>serialize</c> for in-process.</summary>
    public Dictionary<string, string> RmwSettings { get; } = new(StringComparer.OrdinalIgnoreCase);

    /// <summary>
    /// Deliver messages between publishers and subscriptions of this context by reference (zero copy),
    /// bypassing serialization. Requires a middleware that can ignore local publications.
    /// </summary>
    public bool UseIntraProcessComms { get; set; }

    public ILoggerFactory? LoggerFactory { get; set; }

    /// <summary>Command-line arguments; <c>--ros-args</c> blocks are parsed (remaps, parameters, log level, enclave).</summary>
    public IEnumerable<string>? Arguments { get; set; }

    /// <summary>Restrict discovery to this machine. Defaults to <c>ROS_LOCALHOST_ONLY=1</c> / <c>ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST</c>.</summary>
    public bool? LocalhostOnly { get; set; }

    /// <summary>SROS2 security enclave.</summary>
    public string? Enclave { get; set; }

    /// <summary>Shut the context down on Ctrl+C / SIGTERM.</summary>
    public bool HandleShutdownSignals { get; set; }
}

/// <summary>
/// The root of a ROS 2 application, equivalent to <c>rclcpp::Context</c> plus <c>rclcpp::init</c>:
/// owns the middleware, the nodes and the shutdown signal.
/// </summary>
public sealed class Ros2Context : IAsyncDisposable, IDisposable
{
    private readonly CancellationTokenSource _shutdown = new();
    private readonly List<Node> _nodes = [];
    private readonly object _gate = new();
    private readonly List<PosixSignalRegistration> _signals = [];
    private Executor? _defaultExecutor;
    private int _disposed;

    private Ros2Context(Ros2ContextOptions options, IRmwImplementation rmw, ILoggerFactory loggerFactory, RosArguments arguments, int domainId)
    {
        Options = options;
        Rmw = rmw;
        LoggerFactory = loggerFactory;
        Arguments = arguments;
        DomainId = domainId;
        UseIntraProcessComms = options.UseIntraProcessComms && rmw.SupportsIgnoreLocalPublications && rmw.Identifier != "inprocess";
        IntraProcess = UseIntraProcessComms ? new IntraProcessManager() : null;
        Logger = loggerFactory.CreateLogger("ros2net");

        if (options.HandleShutdownSignals)
        {
            foreach (var signal in new[] { PosixSignal.SIGINT, PosixSignal.SIGTERM })
            {
                try
                {
                    _signals.Add(PosixSignalRegistration.Create(signal, ctx =>
                    {
                        ctx.Cancel = true;
                        Shutdown($"signal {ctx.Signal}");
                    }));
                }
                catch (PlatformNotSupportedException)
                {
                }
            }
        }
    }

    public Ros2ContextOptions Options { get; }

    public IRmwImplementation Rmw { get; }

    public int DomainId { get; }

    public ILoggerFactory LoggerFactory { get; }

    public RosArguments Arguments { get; }

    public bool UseIntraProcessComms { get; }

    internal IntraProcessManager? IntraProcess { get; }

    internal ILogger Logger { get; }

    /// <summary>Cancelled when the context shuts down.</summary>
    public CancellationToken ShutdownToken => _shutdown.Token;

    public bool IsShutdown => _shutdown.IsCancellationRequested;

    public IReadOnlyList<Node> Nodes
    {
        get
        {
            lock (_gate)
            {
                return _nodes.ToList();
            }
        }
    }

    /// <summary>Creates and starts a context.</summary>
    public static async Task<Ros2Context> CreateAsync(Ros2ContextOptions? options = null, CancellationToken cancellationToken = default)
    {
        options ??= new Ros2ContextOptions();
        var args = RosArguments.Parse(options.Arguments);
        var loggerFactory = options.LoggerFactory ?? NullLoggerFactory.Instance;
        var domainId = options.DomainId ?? (int.TryParse(Environment.GetEnvironmentVariable("ROS_DOMAIN_ID"), out var d) ? d : 0);
        if (domainId is < 0 or > 232)
        {
            throw new ArgumentOutOfRangeException(nameof(options), $"Domain id {domainId} is outside 0..232.");
        }

        var localhost = options.LocalhostOnly ??
            (Environment.GetEnvironmentVariable("ROS_LOCALHOST_ONLY") == "1" ||
             string.Equals(Environment.GetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE"), "LOCALHOST", StringComparison.OrdinalIgnoreCase));

        var factory = options.Rmw ?? RmwRegistry.GetDefault();
        var rmw = await factory.CreateAsync(new RmwContextOptions
        {
            DomainId = domainId,
            Enclave = options.Enclave ?? args.Enclave ?? Environment.GetEnvironmentVariable("ROS_SECURITY_ENCLAVE_OVERRIDE") ?? "/",
            LocalhostOnly = localhost,
            LoggerFactory = loggerFactory,
            Settings = options.RmwSettings,
        }, cancellationToken).ConfigureAwait(false);

        return new Ros2Context(options, rmw, loggerFactory, args, domainId);
    }

    /// <summary>Synchronous <see cref="CreateAsync"/>.</summary>
    public static Ros2Context Create(Ros2ContextOptions? options = null) =>
        CreateAsync(options).GetAwaiter().GetResult();

    /// <summary>Creates a node in this context.</summary>
    public Node CreateNode(string name, string? @namespace = null, NodeOptions? options = null) =>
        new(this, name, WithNamespace(options, @namespace));

    /// <summary>Creates a managed (lifecycle) node in this context.</summary>
    public LifecycleNode CreateLifecycleNode(string name, string? @namespace = null, NodeOptions? options = null) =>
        new(this, name, WithNamespace(options, @namespace));

    private static NodeOptions? WithNamespace(NodeOptions? options, string? ns) =>
        ns is null ? options : (options ?? new NodeOptions()).WithNamespace(ns);

    internal void Register(Node node)
    {
        ObjectDisposedException.ThrowIf(Volatile.Read(ref _disposed) != 0, this);
        Executor? executor;
        lock (_gate)
        {
            _nodes.Add(node);
            executor = _defaultExecutor;
        }

        executor?.AddNode(node);
    }

    internal void Unregister(Node node)
    {
        lock (_gate)
        {
            _nodes.Remove(node);
        }
    }

    /// <summary>
    /// Spins every node of this context (including nodes created later) until cancellation or
    /// shutdown, on a single-threaded executor unless <paramref name="executor"/> is given.
    /// </summary>
    public async Task SpinAsync(CancellationToken cancellationToken = default, Executor? executor = null)
    {
        executor ??= new SingleThreadedExecutor();
        lock (_gate)
        {
            _defaultExecutor = executor;
        }

        foreach (var n in Nodes)
        {
            executor.AddNode(n);
        }

        try
        {
            await executor.SpinAsync(cancellationToken).ConfigureAwait(false);
        }
        finally
        {
            lock (_gate)
            {
                if (_defaultExecutor == executor)
                {
                    _defaultExecutor = null;
                }
            }

            executor.Dispose();
        }
    }

    /// <summary>Raised once when the context starts shutting down.</summary>
    public event Action<string>? ShuttingDown;

    /// <summary>Requests shutdown: spinning stops and pending waits are cancelled.</summary>
    public void Shutdown(string reason = "user request")
    {
        if (_shutdown.IsCancellationRequested)
        {
            return;
        }

        Logger.LogInformation("Shutting down: {Reason}", reason);
        ShuttingDown?.Invoke(reason);
        _shutdown.Cancel();
    }

    /// <summary>Waits until the context shuts down.</summary>
    public Task WaitForShutdownAsync(CancellationToken cancellationToken = default) =>
        Task.Delay(Timeout.Infinite, CancellationTokenSource.CreateLinkedTokenSource(ShutdownToken, cancellationToken).Token)
            .ContinueWith(_ => { }, TaskScheduler.Default);

    public async ValueTask DisposeAsync()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0)
        {
            return;
        }

        Shutdown("context disposed");
        foreach (var node in Nodes)
        {
            node.Dispose();
        }

        foreach (var s in _signals)
        {
            s.Dispose();
        }

        await Rmw.DisposeAsync().ConfigureAwait(false);
        _shutdown.Dispose();
    }

    public void Dispose() => DisposeAsync().AsTask().GetAwaiter().GetResult();
}
