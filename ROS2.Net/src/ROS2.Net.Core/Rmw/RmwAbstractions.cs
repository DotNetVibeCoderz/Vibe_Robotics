using System.Buffers.Binary;
using System.Security.Cryptography;
using Microsoft.Extensions.Logging;

namespace ROS2.Net.Rmw;

/// <summary>A 16-byte global endpoint identifier (an RTPS GUID: 12-byte participant prefix + 4-byte entity id).</summary>
public readonly record struct Gid(ulong High, ulong Low)
{
    public static Gid Empty => default;

    /// <summary>Creates a GID from a participant prefix and an entity id.</summary>
    public static Gid Create(ReadOnlySpan<byte> prefix12, uint entityId)
    {
        Span<byte> bytes = stackalloc byte[16];
        prefix12[..12].CopyTo(bytes);
        BinaryPrimitives.WriteUInt32BigEndian(bytes[12..], entityId);
        return FromBytes(bytes);
    }

    public static Gid FromBytes(ReadOnlySpan<byte> bytes16) =>
        new(BinaryPrimitives.ReadUInt64BigEndian(bytes16), BinaryPrimitives.ReadUInt64BigEndian(bytes16[8..]));

    public static Gid NewRandom()
    {
        Span<byte> bytes = stackalloc byte[16];
        RandomNumberGenerator.Fill(bytes);
        return FromBytes(bytes);
    }

    public void WriteTo(Span<byte> destination)
    {
        BinaryPrimitives.WriteUInt64BigEndian(destination, High);
        BinaryPrimitives.WriteUInt64BigEndian(destination[8..], Low);
    }

    public byte[] ToArray()
    {
        var b = new byte[16];
        WriteTo(b);
        return b;
    }

    /// <summary>True when both GIDs share the same 12-byte participant prefix.</summary>
    public bool SameParticipant(Gid other) => High == other.High && (Low >> 32) == (other.Low >> 32);

    public override string ToString() => $"{High:x16}{Low:x16}";
}

/// <summary>Metadata delivered with each received message.</summary>
public readonly record struct MessageInfo(
    RosTime SourceTimestamp,
    RosTime ReceivedTimestamp,
    Gid PublisherGid,
    long SequenceNumber,
    bool FromIntraProcess = false);

/// <summary>Identifies one service request: the client's GID and its sequence number.</summary>
public readonly record struct RequestId(Gid ClientGid, long SequenceNumber);

/// <summary>Kind of a graph endpoint.</summary>
public enum EndpointKind
{
    Publisher,
    Subscription,
    ServiceServer,
    ServiceClient,
}

/// <summary>A node visible in the ROS graph.</summary>
public sealed record NodeInfo(string Name, string Namespace, string Enclave = "/")
{
    public string FullyQualifiedName => RosNames.FullyQualifiedNodeName(Namespace, Name);
}

/// <summary>A topic or service with the types it is used with.</summary>
public sealed record TopicInfo(string Name, IReadOnlyList<string> Types);

/// <summary>One endpoint in the graph.</summary>
public sealed record EndpointInfo(
    string NodeName,
    string NodeNamespace,
    string Name,
    string TypeName,
    EndpointKind Kind,
    Gid Gid,
    QosProfile Qos);

/// <summary>Read-only view of the ROS graph as seen by the middleware's discovery.</summary>
public interface IRmwGraph
{
    IReadOnlyList<NodeInfo> GetNodes();

    /// <summary>All endpoints currently known, local and remote.</summary>
    IReadOnlyList<EndpointInfo> GetEndpoints();

    /// <summary>Raised (on an arbitrary thread) whenever nodes or endpoints appear or disappear.</summary>
    event Action? Changed;
}

/// <summary>Options passed to a middleware when a context starts.</summary>
public sealed class RmwContextOptions
{
    public int DomainId { get; init; }

    /// <summary>Security enclave (SROS2), e.g. <c>/talker_listener/talker</c>.</summary>
    public string Enclave { get; init; } = "/";

    /// <summary>Restrict discovery to this host.</summary>
    public bool LocalhostOnly { get; init; }

    public ILoggerFactory LoggerFactory { get; init; } = Microsoft.Extensions.Logging.Abstractions.NullLoggerFactory.Instance;

    /// <summary>Middleware-specific settings (<c>key=value</c>).</summary>
    public IReadOnlyDictionary<string, string> Settings { get; init; } = new Dictionary<string, string>();
}

/// <summary>Creates middleware instances. Register factories with <see cref="RmwRegistry"/>.</summary>
public interface IRmwFactory
{
    /// <summary>Identifier used by <c>ROS2NET_RMW</c>, e.g. <c>inprocess</c>, <c>rtps</c>, <c>rosbridge</c>.</summary>
    string Identifier { get; }

    ValueTask<IRmwImplementation> CreateAsync(RmwContextOptions options, CancellationToken cancellationToken);
}

/// <summary>
/// A ROS middleware: the layer that moves messages between nodes. Equivalent to the C <c>rmw</c> API,
/// but typed: publishers and subscriptions are generic over the message, so each middleware can pick
/// its own representation (object references in-process, CDR over RTPS, JSON over rosbridge).
/// </summary>
public interface IRmwImplementation : IAsyncDisposable
{
    string Identifier { get; }

    /// <summary>
    /// Whether subscriptions can ignore messages published from the same context. Intra-process
    /// delivery relies on this to avoid receiving every local message twice.
    /// </summary>
    bool SupportsIgnoreLocalPublications { get; }

    IRmwGraph Graph { get; }

    IRmwNode CreateNode(string name, string @namespace);
}

/// <summary>Per-endpoint middleware options.</summary>
public sealed record RmwEndpointOptions
{
    public static RmwEndpointOptions Default { get; } = new();

    /// <summary>Drop messages that were published from the same context.</summary>
    public bool IgnoreLocalPublications { get; init; }
}

/// <summary>Everything a middleware needs to create a publisher or subscription.</summary>
public sealed record RmwTopicSpec(string TopicName, MessageTypeInfo Type, QosProfile Qos, RmwEndpointOptions Options);

/// <summary>Everything a middleware needs to create a service server or client.</summary>
public sealed record RmwServiceSpec(string ServiceName, ServiceTypeInfo Type, QosProfile Qos);

/// <summary>Delivers a decoded message.</summary>
public delegate void RmwMessageHandler<in T>(T message, MessageInfo info);

/// <summary>Delivers raw CDR (encapsulation header included). The span is only valid during the call.</summary>
public delegate void RmwSerializedHandler(ReadOnlySpan<byte> cdr, MessageInfo info);

/// <summary>A middleware node: the factory for endpoints.</summary>
public interface IRmwNode : IDisposable
{
    string Name { get; }

    string Namespace { get; }

    IRmwPublisher<T> CreatePublisher<T>(RmwTopicSpec spec)
        where T : class, IRosMessage, new();

    IRmwSubscription CreateSubscription<T>(RmwTopicSpec spec, RmwMessageHandler<T> handler)
        where T : class, IRosMessage, new();

    /// <summary>A publisher of pre-serialized CDR, for bag playback and bridges.</summary>
    IRmwSerializedPublisher CreateSerializedPublisher(RmwTopicSpec spec);

    /// <summary>A subscription that receives raw CDR, for recording and introspection.</summary>
    IRmwSubscription CreateSerializedSubscription(RmwTopicSpec spec, RmwSerializedHandler handler);

    IRmwServiceServer<TResponse> CreateServiceServer<TRequest, TResponse>(RmwServiceSpec spec, Action<RequestId, TRequest> onRequest)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new();

    IRmwServiceClient<TRequest> CreateServiceClient<TRequest, TResponse>(RmwServiceSpec spec, Action<long, TResponse> onResponse)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new();
}

/// <summary>Common endpoint surface.</summary>
public interface IRmwEndpoint : IDisposable
{
    Gid Gid { get; }

    /// <summary>Number of matched remote endpoints (subscriptions for a publisher, publishers for a subscription).</summary>
    int MatchedCount { get; }

    /// <summary>QoS events raised by the middleware itself (matching, incompatible QoS, liveliness).</summary>
    event Action<QosEvent>? QosEventRaised;
}

public interface IRmwPublisher<in T> : IRmwEndpoint
    where T : class, IRosMessage
{
    void Publish(T message);

    void AssertLiveliness();
}

public interface IRmwSerializedPublisher : IRmwEndpoint
{
    void Publish(ReadOnlySpan<byte> cdr);
}

public interface IRmwSubscription : IRmwEndpoint;

public interface IRmwServiceServer<in TResponse> : IRmwEndpoint
    where TResponse : class, IRosMessage
{
    void SendResponse(RequestId requestId, TResponse response);
}

public interface IRmwServiceClient<in TRequest> : IRmwEndpoint
    where TRequest : class, IRosMessage
{
    /// <summary>Sends a request and returns its sequence number.</summary>
    long SendRequest(TRequest request);

    /// <summary>True once a matching server has been discovered.</summary>
    bool IsServiceAvailable { get; }
}

/// <summary>
/// Process-wide registry of middleware factories. Middleware assemblies register themselves; the
/// default is chosen by <c>ROS2NET_RMW</c> (falling back to <c>RMW_IMPLEMENTATION</c> name hints),
/// then <c>inprocess</c>.
/// </summary>
public static class RmwRegistry
{
    private static readonly Dictionary<string, IRmwFactory> s_factories = new(StringComparer.OrdinalIgnoreCase);
    private static readonly object s_gate = new();

    static RmwRegistry() => Register(new InProcess.InProcessRmwFactory());

    public static void Register(IRmwFactory factory)
    {
        lock (s_gate)
        {
            s_factories[factory.Identifier] = factory;
        }
    }

    public static bool TryGet(string identifier, [System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out IRmwFactory? factory)
    {
        lock (s_gate)
        {
            return s_factories.TryGetValue(identifier, out factory);
        }
    }

    public static IReadOnlyList<string> Identifiers
    {
        get
        {
            lock (s_gate)
            {
                return s_factories.Keys.Order(StringComparer.OrdinalIgnoreCase).ToList();
            }
        }
    }

    /// <summary>The factory selected by the environment, or the in-process middleware.</summary>
    public static IRmwFactory GetDefault()
    {
        var requested = Environment.GetEnvironmentVariable("ROS2NET_RMW");
        if (string.IsNullOrWhiteSpace(requested))
        {
            // Accept the stock variable too: any DDS implementation maps onto the managed RTPS stack.
            var stock = Environment.GetEnvironmentVariable("RMW_IMPLEMENTATION");
            if (stock is not null && (stock.Contains("fastrtps", StringComparison.OrdinalIgnoreCase) || stock.Contains("cyclonedds", StringComparison.OrdinalIgnoreCase) || stock.Contains("connext", StringComparison.OrdinalIgnoreCase)))
            {
                requested = "rtps";
            }
        }

        if (!string.IsNullOrWhiteSpace(requested))
        {
            if (TryGet(requested, out var f))
            {
                return f;
            }

            throw new InvalidOperationException(
                $"ROS2NET_RMW='{requested}' but no such middleware is registered (known: {string.Join(", ", Identifiers)}). " +
                "Reference the middleware package and call its Use…() extension, or touch one of its types, so it registers.");
        }

        return s_factories["inprocess"];
    }
}
