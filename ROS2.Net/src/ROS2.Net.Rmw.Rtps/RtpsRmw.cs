using Microsoft.Extensions.Logging;
using ROS2.Net.Rmw.Rtps.Discovery;
using ROS2.Net.Rmw.Rtps.Protocol;
using ROS2.Net.Rmw.Rtps.Wire;

namespace ROS2.Net.Rmw.Rtps;

/// <summary>ROS 2 ↔ DDS name mangling, as done by rmw_fastrtps and rmw_cyclonedds.</summary>
public static class RosDdsNames
{
    public const string TopicPrefix = "rt";
    public const string RequestPrefix = "rq";
    public const string ReplyPrefix = "rr";

    /// <summary><c>/chatter</c> → <c>rt/chatter</c> (verbatim when avoiding ROS conventions).</summary>
    public static string Topic(string rosName, bool avoidRosNamespaceConventions = false) =>
        avoidRosNamespaceConventions ? rosName : TopicPrefix + rosName;

    /// <summary><c>/add_two_ints</c> → <c>rq/add_two_intsRequest</c>.</summary>
    public static string Request(string rosService, bool avoid = false) => avoid ? rosService + "Request" : RequestPrefix + rosService + "Request";

    /// <summary><c>/add_two_ints</c> → <c>rr/add_two_intsReply</c>.</summary>
    public static string Reply(string rosService, bool avoid = false) => avoid ? rosService + "Reply" : ReplyPrefix + rosService + "Reply";

    /// <summary>Maps a DDS endpoint back to a ROS graph entry, or null when it is not a ROS endpoint.</summary>
    public static (string Name, string Type, EndpointKind Kind)? ToRos(string ddsTopic, string ddsType, bool isWriter)
    {
        var rosType = MessageTypeInfo.FromDdsName(ddsType);
        if (ddsTopic.StartsWith("rt/", StringComparison.Ordinal))
        {
            return (ddsTopic[2..], rosType, isWriter ? EndpointKind.Publisher : EndpointKind.Subscription);
        }

        if (ddsTopic.StartsWith("rq/", StringComparison.Ordinal) && ddsTopic.EndsWith("Request", StringComparison.Ordinal))
        {
            var service = ddsTopic[2..^"Request".Length];
            var type = rosType.EndsWith("_Request", StringComparison.Ordinal) ? rosType[..^"_Request".Length] : rosType;
            return (service, type, isWriter ? EndpointKind.ServiceClient : EndpointKind.ServiceServer);
        }

        if (ddsTopic.StartsWith('/'))
        {
            return (ddsTopic, rosType, isWriter ? EndpointKind.Publisher : EndpointKind.Subscription);
        }

        return null;
    }
}

/// <summary>
/// The managed RTPS middleware (identifier <c>rtps</c>). One instance is one DDS domain participant.
/// </summary>
public sealed class RtpsRmw : IRmwImplementation, IRmwGraph
{
    internal const string UnknownNodeName = "_NODE_NAME_UNKNOWN_";
    internal const string UnknownNodeNamespace = "_NODE_NAMESPACE_UNKNOWN_";

    private readonly RtpsParticipant _participant;
    private readonly ILogger _logger;

    public RtpsRmw(int domainId, string enclave, RtpsOptions options, ILoggerFactory loggerFactory)
    {
        Options = options;
        _logger = loggerFactory.CreateLogger("ros2net.rtps");
        _participant = new RtpsParticipant(domainId, enclave, options, loggerFactory);
        _participant.Changed += () => Changed?.Invoke();
    }

    public string Identifier => "rtps";

    public bool SupportsIgnoreLocalPublications => true;

    public IRmwGraph Graph => this;

    public RtpsOptions Options { get; }

    public int DomainId => _participant.DomainId;

    /// <summary>This participant's GUID prefix.</summary>
    public GuidPrefix GuidPrefix => _participant.Prefix;

    /// <summary>RTPS participant id (selects the unicast ports).</summary>
    public int ParticipantId => _participant.ParticipantId;

    /// <summary>Number of remote participants currently discovered.</summary>
    public int DiscoveredParticipantCount => _participant.RemoteParticipantCount;

    public event Action? Changed;

    internal RtpsParticipant Participant => _participant;

    internal ILogger Logger => _logger;

    public ValueTask StartAsync(CancellationToken cancellationToken)
    {
        _participant.Start();
        return ValueTask.CompletedTask;
    }

    public IRmwNode CreateNode(string name, string @namespace) => new RtpsNode(this, name, @namespace);

    public IReadOnlyList<NodeInfo> GetNodes()
    {
        var nodes = new List<NodeInfo>(_participant.LocalNodes.Select(n => n with { Enclave = _participant.Enclave }));
        foreach (var info in _participant.Snapshot().Infos)
        {
            nodes.AddRange(info.Nodes.Select(n => new NodeInfo(n.Name, n.Namespace)));
        }

        return nodes.Distinct().ToList();
    }

    public IReadOnlyList<EndpointInfo> GetEndpoints()
    {
        var (locals, remotes, infos) = _participant.Snapshot();
        var result = new List<EndpointInfo>();
        foreach (var l in locals)
        {
            if (RosDdsNames.ToRos(l.Topic, l.TypeName, l.IsWriter) is { } m)
            {
                result.Add(new EndpointInfo(l.NodeName, l.NodeNamespace, m.Name, m.Type, m.Kind, l.Guid.ToGid(), l.Qos));
            }
        }

        var owners = new Dictionary<Gid, (string Name, string Ns)>();
        foreach (var info in infos)
        {
            foreach (var n in info.Nodes)
            {
                foreach (var g in n.Readers.Concat(n.Writers))
                {
                    owners[g] = (n.Name, n.Namespace);
                }
            }
        }

        foreach (var r in remotes)
        {
            if (RosDdsNames.ToRos(r.TopicName, r.TypeName, r.IsWriter) is { } m)
            {
                var gid = r.Guid.ToGid();
                var (name, ns) = owners.TryGetValue(gid, out var o) ? o : (UnknownNodeName, UnknownNodeNamespace);
                result.Add(new EndpointInfo(name, ns, m.Name, m.Type, m.Kind, gid, r.Qos));
            }
        }

        return result;
    }

    public ValueTask DisposeAsync() => _participant.DisposeAsync();
}

internal sealed class RtpsNode : IRmwNode
{
    private readonly RtpsRmw _rmw;
    private readonly NodeInfo _info;
    private int _disposed;

    public RtpsNode(RtpsRmw rmw, string name, string @namespace)
    {
        _rmw = rmw;
        Name = name;
        Namespace = @namespace;
        _info = new NodeInfo(name, @namespace);
        rmw.Participant.AddNode(_info);
    }

    public string Name { get; }

    public string Namespace { get; }

    internal RtpsParticipant Participant => _rmw.Participant;

    internal RtpsRmw Rmw => _rmw;

    public IRmwPublisher<T> CreatePublisher<T>(RmwTopicSpec spec)
        where T : class, IRosMessage, new() => new RtpsPublisher<T>(this, spec);

    public IRmwSerializedPublisher CreateSerializedPublisher(RmwTopicSpec spec) => new RtpsSerializedPublisher(this, spec);

    public IRmwSubscription CreateSubscription<T>(RmwTopicSpec spec, RmwMessageHandler<T> handler)
        where T : class, IRosMessage, new()
    {
        var logger = _rmw.Logger;
        return new RtpsSubscription(this, spec, (in RtpsSample s) =>
        {
            if (s.Payload.IsEmpty)
            {
                return;
            }

            T message;
            try
            {
                message = Serialization.CdrReader.Deserialize<T>(s.Payload);
            }
            catch (Serialization.CdrException ex)
            {
                logger.LogWarning(ex, "Dropping undecodable sample on {Topic}", spec.TopicName);
                return;
            }

            handler(message, RtpsSubscription.Info(in s));
        });
    }

    public IRmwSubscription CreateSerializedSubscription(RmwTopicSpec spec, RmwSerializedHandler handler) =>
        new RtpsSubscription(this, spec, (in RtpsSample s) =>
        {
            if (!s.Payload.IsEmpty)
            {
                handler(s.Payload, RtpsSubscription.Info(in s));
            }
        });

    public IRmwServiceServer<TResponse> CreateServiceServer<TRequest, TResponse>(RmwServiceSpec spec, Action<RequestId, TRequest> onRequest)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new() => new RtpsServiceServer<TRequest, TResponse>(this, spec, onRequest);

    public IRmwServiceClient<TRequest> CreateServiceClient<TRequest, TResponse>(RmwServiceSpec spec, Action<long, TResponse> onResponse)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new() => new RtpsServiceClient<TRequest, TResponse>(this, spec, onResponse);

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
        {
            _rmw.Participant.RemoveNode(_info);
        }
    }
}
