using System.Runtime.CompilerServices;
using ROS2.Net.Rmw.Rtps.Protocol;
using ROS2.Net.Rmw.Rtps.Wire;
using ROS2.Net.Serialization;

namespace ROS2.Net.Rmw.Rtps;

internal abstract class RtpsEndpointBase : IRmwEndpoint
{
    private int _disposed;

    protected RtpsEndpointBase(RtpsNode node) => Node = node;

    protected RtpsNode Node { get; }

    public abstract Gid Gid { get; }

    public abstract int MatchedCount { get; }

    public event Action<QosEvent>? QosEventRaised;

    protected void Forward(QosEvent e) => QosEventRaised?.Invoke(e);

    protected abstract void DisposeCore();

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposed, 1) == 0)
        {
            DisposeCore();
        }
    }
}

internal class RtpsPublisherBase : RtpsEndpointBase
{
    protected RtpsPublisherBase(RtpsNode node, RmwTopicSpec spec)
        : base(node)
    {
        Endpoint = node.Participant.CreateWriter(RosDdsNames.Topic(spec.TopicName, spec.Qos.AvoidRosNamespaceConventions), spec.Type.DdsName, spec.Qos, node.Name, node.Namespace);
        Endpoint.QosEvent += Forward;
    }

    protected LocalEndpoint Endpoint { get; }

    public override Gid Gid => Endpoint.Guid.ToGid();

    public override int MatchedCount => Endpoint.MatchedCount;

    public void AssertLiveliness()
    {
    }

    protected override void DisposeCore() => Node.Participant.Unregister(Endpoint);
}

internal sealed class RtpsPublisher<T>(RtpsNode node, RmwTopicSpec spec) : RtpsPublisherBase(node, spec), IRmwPublisher<T>
    where T : class, IRosMessage
{
    public void Publish(T message) => CdrWriter.SerializeWith(message, Endpoint.Writer!, static (span, w) => w.Write(span));
}

internal sealed class RtpsSerializedPublisher(RtpsNode node, RmwTopicSpec spec) : RtpsPublisherBase(node, spec), IRmwSerializedPublisher
{
    public void Publish(ReadOnlySpan<byte> cdr) => Endpoint.Writer!.Write(cdr);
}

internal sealed class RtpsSubscription : RtpsEndpointBase, IRmwSubscription
{
    private readonly LocalEndpoint _endpoint;

    public RtpsSubscription(RtpsNode node, RmwTopicSpec spec, RtpsSampleHandler handler)
        : base(node)
    {
        _endpoint = node.Participant.CreateReader(RosDdsNames.Topic(spec.TopicName, spec.Qos.AvoidRosNamespaceConventions), spec.Type.DdsName, spec.Qos,
            node.Name, node.Namespace, spec.Options.IgnoreLocalPublications, handler);
        _endpoint.QosEvent += Forward;
    }

    public override Gid Gid => _endpoint.Guid.ToGid();

    public override int MatchedCount => _endpoint.MatchedCount;

    internal static MessageInfo Info(in RtpsSample s) =>
        new(new RosTime(s.Timestamp, ClockType.SystemTime), RosTime.Now(), s.Writer.ToGid(), s.SequenceNumber);

    protected override void DisposeCore() => Node.Participant.Unregister(_endpoint);
}

/// <summary>Correlation data carried with service requests and replies.</summary>
internal static class ServiceHeaders
{
    /// <summary>Inline QoS with the related sample identity under both the eProsima (0x800f) and standard (0x0083) ids.</summary>
    public static byte[] RelatedIdentity(RtpsGuid writer, long sn)
    {
        using var buffer = new RtpsBuffer(80);
        var w = new ParameterListWriter(buffer);
        foreach (var pid in new[] { ParameterId.FastDdsRelatedSampleIdentity, ParameterId.RelatedSampleIdentity })
        {
            w.Begin(pid);
            buffer.WriteGuid(writer);
            buffer.WriteSequenceNumber(sn);
            w.End();
        }

        w.WriteSentinel();
        return buffer.Span.ToArray();
    }

    public static bool TryReadRelatedIdentity(in RtpsSample s, out RtpsGuid writer, out long sn)
    {
        writer = default;
        sn = 0;
        if (s.InlineQos.IsEmpty)
        {
            return false;
        }

        var qos = ParameterList.Parse(s.InlineQos, s.InlineQosLittleEndian, out _);
        if ((qos.TryGet(ParameterId.FastDdsRelatedSampleIdentity, out var v) || qos.TryGet(ParameterId.RelatedSampleIdentity, out v)) && v.Length >= 24)
        {
            var r = new SpanReader(v, qos.LittleEndian);
            writer = r.ReadGuid();
            sn = r.ReadSequenceNumber();
            return true;
        }

        return false;
    }

    /// <summary>The 64-bit client id rmw_cyclonedds-style headers carry (FNV-1a of the request writer GUID).</summary>
    public static ulong ClientId(RtpsGuid writer)
    {
        Span<byte> b = stackalloc byte[16];
        writer.Write(b);
        var h = 14695981039346656037UL;
        foreach (var x in b)
        {
            h = (h ^ x) * 1099511628211UL;
        }

        return h;
    }

    /// <summary>Serializes <c>{ uint64 guid; int64 seq; }</c> followed by the message in one CDR stream.</summary>
    public static void WithCycloneHeader(IRosMessage message, ulong clientId, long seq, RtpsWriter writer)
    {
        using var cdr = new CdrWriter(512);
        cdr.WriteUInt64(clientId);
        cdr.WriteInt64(seq);
        message.Serialize(cdr);
        writer.Write(cdr.WrittenSpan);
    }

    public static T ReadCycloneHeader<T>(ReadOnlySpan<byte> payload, out ulong clientId, out long seq)
        where T : class, IRosMessage, new()
    {
        var reader = new CdrReader(payload);
        clientId = reader.ReadUInt64();
        seq = reader.ReadInt64();
        var m = new T();
        m.Deserialize(ref reader);
        return m;
    }
}

internal sealed class RtpsServiceServer<TRequest, TResponse> : RtpsEndpointBase, IRmwServiceServer<TResponse>
    where TRequest : class, IRosMessage, new()
    where TResponse : class, IRosMessage, new()
{
    private readonly LocalEndpoint _requests;
    private readonly LocalEndpoint _replies;
    private readonly ServiceMapping _mapping;

    public RtpsServiceServer(RtpsNode node, RmwServiceSpec spec, Action<RequestId, TRequest> onRequest)
        : base(node)
    {
        _mapping = node.Rmw.Options.ServiceMapping;
        var avoid = spec.Qos.AvoidRosNamespaceConventions;
        var p = node.Participant;
        _replies = p.CreateWriter(RosDdsNames.Reply(spec.ServiceName, avoid), spec.Type.Response.DdsName, spec.Qos, node.Name, node.Namespace);
        _requests = p.CreateReader(RosDdsNames.Request(spec.ServiceName, avoid), spec.Type.Request.DdsName, spec.Qos, node.Name, node.Namespace, false, (in RtpsSample s) =>
        {
            if (s.Payload.IsEmpty)
            {
                return;
            }

            try
            {
                if (_mapping == ServiceMapping.CycloneDds)
                {
                    var req = ServiceHeaders.ReadCycloneHeader<TRequest>(s.Payload, out var clientId, out var seq);
                    onRequest(new RequestId(new Gid(clientId, 0), seq), req);
                }
                else
                {
                    onRequest(new RequestId(s.Writer.ToGid(), s.SequenceNumber), CdrReader.Deserialize<TRequest>(s.Payload));
                }
            }
            catch (CdrException)
            {
                // Undecodable request (e.g. mismatched service mapping): drop it.
            }
        });
        _requests.QosEvent += Forward;
    }

    public override Gid Gid => _requests.Guid.ToGid();

    public override int MatchedCount => _requests.MatchedCount;

    public void SendResponse(RequestId requestId, TResponse response)
    {
        if (_mapping == ServiceMapping.CycloneDds)
        {
            ServiceHeaders.WithCycloneHeader(response, requestId.ClientGid.High, requestId.SequenceNumber, _replies.Writer!);
            return;
        }

        var qos = ServiceHeaders.RelatedIdentity(RtpsGuid.FromGid(requestId.ClientGid), requestId.SequenceNumber);
        CdrWriter.SerializeWith(response, (_replies.Writer!, qos), static (span, st) => st.Item1.Write(span, st.qos));
    }

    protected override void DisposeCore()
    {
        Node.Participant.Unregister(_requests);
        Node.Participant.Unregister(_replies);
    }
}

internal sealed class RtpsServiceClient<TRequest, TResponse> : RtpsEndpointBase, IRmwServiceClient<TRequest>
    where TRequest : class, IRosMessage, new()
    where TResponse : class, IRosMessage, new()
{
    private readonly LocalEndpoint _requests;
    private readonly LocalEndpoint _replies;
    private readonly ServiceMapping _mapping;
    private readonly ulong _clientId;
    private long _sequence;

    public RtpsServiceClient(RtpsNode node, RmwServiceSpec spec, Action<long, TResponse> onResponse)
        : base(node)
    {
        _mapping = node.Rmw.Options.ServiceMapping;
        var avoid = spec.Qos.AvoidRosNamespaceConventions;
        var p = node.Participant;
        _requests = p.CreateWriter(RosDdsNames.Request(spec.ServiceName, avoid), spec.Type.Request.DdsName, spec.Qos, node.Name, node.Namespace);
        _clientId = ServiceHeaders.ClientId(_requests.Guid);
        var requestWriter = _requests.Guid;
        _replies = p.CreateReader(RosDdsNames.Reply(spec.ServiceName, avoid), spec.Type.Response.DdsName, spec.Qos, node.Name, node.Namespace, false, (in RtpsSample s) =>
        {
            if (s.Payload.IsEmpty)
            {
                return;
            }

            try
            {
                if (_mapping == ServiceMapping.CycloneDds)
                {
                    var res = ServiceHeaders.ReadCycloneHeader<TResponse>(s.Payload, out var clientId, out var seq);
                    if (clientId == _clientId)
                    {
                        onResponse(seq, res);
                    }
                }
                else if (ServiceHeaders.TryReadRelatedIdentity(in s, out var writer, out var sn) && writer == requestWriter)
                {
                    onResponse(sn, CdrReader.Deserialize<TResponse>(s.Payload));
                }
            }
            catch (CdrException)
            {
            }
        });
        _replies.QosEvent += Forward;
    }

    public override Gid Gid => _requests.Guid.ToGid();

    public override int MatchedCount => IsServiceAvailable ? 1 : 0;

    /// <summary>Like rmw_fastrtps: a server is available once both the request and the reply topics are matched.</summary>
    public bool IsServiceAvailable => _requests.MatchedCount > 0 && _replies.MatchedCount > 0;

    public long SendRequest(TRequest request)
    {
        if (_mapping == ServiceMapping.CycloneDds)
        {
            var seq = Interlocked.Increment(ref _sequence);
            ServiceHeaders.WithCycloneHeader(request, _clientId, seq, _requests.Writer!);
            return seq;
        }

        // The reply reader's GUID rides along, as rmw_fastrtps does, so servers can direct replies.
        var qos = ServiceHeaders.RelatedIdentity(_replies.Guid, -1L << 32);
        var sn = new StrongBox<long>();
        CdrWriter.SerializeWith(request, (Writer: _requests.Writer!, Qos: qos, Sn: sn), static (span, st) => st.Sn.Value = st.Writer.Write(span, st.Qos));
        return sn.Value;
    }

    protected override void DisposeCore()
    {
        Node.Participant.Unregister(_requests);
        Node.Participant.Unregister(_replies);
    }
}
