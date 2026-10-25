using System.Diagnostics;
using System.Net;
using System.Text;
using Microsoft.Extensions.Logging;
using ROS2.Net.Rmw.Rtps.Discovery;
using ROS2.Net.Rmw.Rtps.Protocol;
using ROS2.Net.Rmw.Rtps.Transport;
using ROS2.Net.Rmw.Rtps.Wire;
using ROS2.Net.Serialization;

namespace ROS2.Net.Rmw.Rtps;

/// <summary>A user endpoint owned by this participant.</summary>
internal sealed class LocalEndpoint
{
    private int _lastMatched;
    private int _matchedTotal;
    private int _incompatibleTotal;

    public required RtpsGuid Guid { get; init; }

    public required string Topic { get; init; }

    public required string TypeName { get; init; }

    public required QosProfile Qos { get; init; }

    public required string NodeName { get; init; }

    public required string NodeNamespace { get; init; }

    public RtpsWriter? Writer { get; init; }

    public RtpsReader? Reader { get; init; }

    public bool IsWriter => Writer is not null;

    /// <summary>Middleware-internal endpoint (ros_discovery_info): hidden from the graph and node lists.</summary>
    public bool Internal { get; init; }

    public bool IgnoreLocalPublications { get; init; }

    public event Action<QosEvent>? QosEvent;

    public int MatchedCount => Writer?.MatchedCount ?? Reader!.MatchedCount;

    public void RefreshMatched()
    {
        var current = MatchedCount;
        var change = current - Interlocked.Exchange(ref _lastMatched, current);
        if (change != 0)
        {
            var total = change > 0 ? Interlocked.Add(ref _matchedTotal, change) : _matchedTotal;
            QosEvent?.Invoke(new QosEvent(QosEventKind.Matched, total, change, current));
        }
    }

    public void RaiseIncompatible(string reason)
    {
        var total = Interlocked.Increment(ref _incompatibleTotal);
        QosEvent?.Invoke(new QosEvent(IsWriter ? QosEventKind.OfferedIncompatibleQos : QosEventKind.RequestedIncompatibleQos, total, 1, Detail: reason));
    }

    public void RaiseLost(int count) =>
        QosEvent?.Invoke(new QosEvent(QosEventKind.MessageLost, count, count));
}

internal sealed class RemoteParticipant
{
    public required ParticipantData Data { get; set; }

    public long LeaseDeadline;

    public List<IPEndPoint> MetaEndpoints { get; set; } = [];

    public List<IPEndPoint> UserEndpoints { get; set; } = [];

    public ParticipantEntitiesInfo? Info { get; set; }
}

/// <summary>
/// One RTPS domain participant: SPDP participant discovery, SEDP endpoint discovery, endpoint
/// matching, datagram routing and the periodic timer (announcements, heartbeats, leases).
/// </summary>
internal sealed class RtpsParticipant : IRtpsMessageHandler, IRtpsSender, IAsyncDisposable
{
    private static readonly byte[] s_statusDisposedUnregistered = [0, 0, 0, 3];

    private readonly object _gate = new();
    private readonly RtpsOptions _options;
    private readonly ILogger _logger;
    private readonly UdpTransport _transport;
    private readonly Dictionary<GuidPrefix, RemoteParticipant> _participants = [];
    private readonly Dictionary<RtpsGuid, EndpointData> _remoteEndpoints = [];
    private readonly Dictionary<EntityId, LocalEndpoint> _locals = [];
    private readonly Dictionary<EntityId, RtpsWriter> _writersById = [];
    private readonly Dictionary<EntityId, RtpsReader> _readersById = [];
    private readonly Dictionary<RtpsGuid, List<RtpsReader>> _readersByRemoteWriter = [];
    private readonly List<NodeInfo> _nodes = [];
    private readonly RtpsMessageBuilder _spdpBuilder;
    private readonly CancellationTokenSource _cts = new();
    private RtpsWriter _sedpPubWriter = null!;
    private RtpsWriter _sedpSubWriter = null!;
    private RtpsReader _sedpPubReader = null!;
    private RtpsReader _sedpSubReader = null!;
    private LocalEndpoint? _infoWriter;
    private LocalEndpoint? _infoReader;
    private Task? _timer;
    private long _spdpSn;
    private uint _entityCounter;
    private int _announcements;
    private int _disposed;

    public RtpsParticipant(int domainId, string enclave, RtpsOptions options, ILoggerFactory loggerFactory)
    {
        DomainId = domainId;
        Enclave = string.IsNullOrEmpty(enclave) ? "/" : enclave;
        _options = options;
        _logger = loggerFactory.CreateLogger("ros2net.rtps");
        _transport = new UdpTransport(domainId, options, _logger);
        Prefix = GuidPrefix.Create(Vendor);
        _spdpBuilder = new RtpsMessageBuilder(Prefix, Vendor);
    }

    public int DomainId { get; }

    public string Enclave { get; }

    public GuidPrefix Prefix { get; }

    public VendorId Vendor => VendorId.Ros2Net;

    public RtpsGuid Guid => new(Prefix, EntityId.Participant);

    public int MaxFragmentSize => _options.MaxFragmentSize;

    public RtpsOptions Options => _options;

    public int ParticipantId => _transport.ParticipantId;

    public UdpTransport Transport => _transport;

    /// <summary>Raised when participants, endpoints or node information change.</summary>
    public event Action? Changed;

    public void Start()
    {
        _transport.Open();
        var hb = _options.HeartbeatPeriod;
        _sedpPubWriter = new RtpsWriter(this, new RtpsGuid(Prefix, EntityId.SedpPublicationsWriter), true, true, int.MaxValue, keyed: true, hb);
        _sedpSubWriter = new RtpsWriter(this, new RtpsGuid(Prefix, EntityId.SedpSubscriptionsWriter), true, true, int.MaxValue, keyed: true, hb);
        _sedpPubReader = new RtpsReader(this, new RtpsGuid(Prefix, EntityId.SedpPublicationsReader), true, OnSedpSample);
        _sedpSubReader = new RtpsReader(this, new RtpsGuid(Prefix, EntityId.SedpSubscriptionsReader), true, OnSedpSample);
        foreach (var w in new[] { _sedpPubWriter, _sedpSubWriter })
        {
            _writersById[w.Guid.EntityId] = w;
        }

        foreach (var r in new[] { _sedpPubReader, _sedpSubReader })
        {
            _readersById[r.Guid.EntityId] = r;
        }

        if (_options.PublishDiscoveryInfo)
        {
            var qos = new QosProfile(1) { Durability = DurabilityPolicy.TransientLocal };
            const string dds = "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_";
            _infoWriter = CreateWriter(ParticipantEntitiesInfo.TopicName, dds, qos, "_ros2net", "/", isInternal: true);
            _infoReader = CreateReader(ParticipantEntitiesInfo.TopicName, dds, qos, "_ros2net", "/", ignoreLocal: true, OnDiscoveryInfo, isInternal: true);
        }

        _transport.Start(OnDatagram);
        AnnounceParticipant();
        PublishDiscoveryInfo();
        _timer = Task.Run(() => TimerLoop(_cts.Token));
    }

    // ------------------------------------------------------------------ nodes and endpoints

    public void AddNode(NodeInfo node)
    {
        lock (_gate)
        {
            _nodes.Add(node);
        }

        PublishDiscoveryInfo();
        Changed?.Invoke();
    }

    public void RemoveNode(NodeInfo node)
    {
        lock (_gate)
        {
            _nodes.Remove(node);
        }

        PublishDiscoveryInfo();
        Changed?.Invoke();
    }

    public IReadOnlyList<NodeInfo> LocalNodes
    {
        get
        {
            lock (_gate)
            {
                return _nodes.ToList();
            }
        }
    }

    public LocalEndpoint CreateWriter(string topic, string typeName, QosProfile qos, string nodeName, string nodeNamespace, bool isInternal = false)
    {
        var q = qos.Resolve();
        RtpsGuid guid;
        lock (_gate)
        {
            guid = new RtpsGuid(Prefix, EntityId.Create(++_entityCounter, EntityId.KindWriterNoKey));
        }

        var writer = new RtpsWriter(this, guid, q.Reliability == ReliabilityPolicy.Reliable, q.Durability == DurabilityPolicy.TransientLocal,
            q.History == HistoryPolicy.KeepAll ? 100_000 : q.Capacity, keyed: false, _options.HeartbeatPeriod);
        var ep = new LocalEndpoint { Guid = guid, Topic = topic, TypeName = typeName, Qos = qos, NodeName = nodeName, NodeNamespace = nodeNamespace, Writer = writer, Internal = isInternal };
        Register(ep);
        return ep;
    }

    public LocalEndpoint CreateReader(string topic, string typeName, QosProfile qos, string nodeName, string nodeNamespace, bool ignoreLocal, RtpsSampleHandler handler, bool isInternal = false)
    {
        var q = qos.Resolve();
        RtpsGuid guid;
        lock (_gate)
        {
            guid = new RtpsGuid(Prefix, EntityId.Create(++_entityCounter, EntityId.KindReaderNoKey));
        }

        var reader = new RtpsReader(this, guid, q.Reliability == ReliabilityPolicy.Reliable, handler);
        var ep = new LocalEndpoint { Guid = guid, Topic = topic, TypeName = typeName, Qos = qos, NodeName = nodeName, NodeNamespace = nodeNamespace, Reader = reader, Internal = isInternal, IgnoreLocalPublications = ignoreLocal };
        reader.SamplesLost += ep.RaiseLost;
        Register(ep);
        return ep;
    }

    private void Register(LocalEndpoint ep)
    {
        List<LocalEndpoint> localPeers;
        List<EndpointData> remotes;
        lock (_gate)
        {
            _locals[ep.Guid.EntityId] = ep;
            if (ep.Writer is not null)
            {
                _writersById[ep.Guid.EntityId] = ep.Writer;
            }
            else
            {
                _readersById[ep.Guid.EntityId] = ep.Reader!;
            }

            localPeers = _locals.Values.Where(o => o != ep && o.IsWriter != ep.IsWriter && o.Topic == ep.Topic && o.TypeName == ep.TypeName).ToList();
            remotes = _remoteEndpoints.Values.Where(r => r.IsWriter != ep.IsWriter && r.TopicName == ep.Topic && r.TypeName == ep.TypeName).ToList();
        }

        Announce(ep);
        foreach (var peer in localPeers)
        {
            if (ep.IsWriter)
            {
                MatchLocal(ep, peer);
            }
            else
            {
                MatchLocal(peer, ep);
            }
        }

        ep.RefreshMatched();

        // Remote matches are applied off the creating thread: the endpoint's owner subscribes to QoS
        // events only after construction returns, and incompatible-QoS events must not be lost.
        if (remotes.Count > 0)
        {
            ThreadPool.QueueUserWorkItem(_ =>
            {
                foreach (var r in remotes)
                {
                    lock (_gate)
                    {
                        if (!_locals.ContainsKey(ep.Guid.EntityId))
                        {
                            return;
                        }
                    }

                    MatchRemote(ep, r);
                }
            });
        }
        if (!ep.Internal)
        {
            PublishDiscoveryInfo();
            Changed?.Invoke();
        }
    }

    public void Unregister(LocalEndpoint ep)
    {
        List<LocalEndpoint> affected;
        lock (_gate)
        {
            if (!_locals.Remove(ep.Guid.EntityId))
            {
                return;
            }

            _writersById.Remove(ep.Guid.EntityId);
            _readersById.Remove(ep.Guid.EntityId);
            foreach (var list in _readersByRemoteWriter.Values)
            {
                if (ep.Reader is not null)
                {
                    list.Remove(ep.Reader);
                }
            }

            affected = _locals.Values.Where(o => o.IsWriter != ep.IsWriter && o.Topic == ep.Topic).ToList();
        }

        foreach (var o in affected)
        {
            if (o.Writer?.RemoveReader(ep.Guid) == true || o.Reader?.RemoveWriter(ep.Guid) == true)
            {
                if (o.Reader is not null)
                {
                    lock (_gate)
                    {
                        if (_readersByRemoteWriter.TryGetValue(ep.Guid, out var l))
                        {
                            l.Remove(o.Reader);
                        }
                    }
                }

                o.RefreshMatched();
            }
        }

        var sedp = ep.IsWriter ? _sedpPubWriter : _sedpSubWriter;
        if (Volatile.Read(ref _disposed) == 0)
        {
            sedp.Write(default, BuildInlineQos(ep.Guid, disposed: true), ep.Guid);
        }

        ep.Writer?.Dispose();
        ep.Reader?.Dispose();
        if (!ep.Internal)
        {
            PublishDiscoveryInfo();
            Changed?.Invoke();
        }
    }

    private void Announce(LocalEndpoint ep)
    {
        var data = new EndpointData
        {
            Guid = ep.Guid,
            ParticipantGuid = Guid,
            TopicName = ep.Topic,
            TypeName = ep.TypeName,
            Qos = ep.Qos,
        };
        using var buffer = new RtpsBuffer(512);
        data.Write(buffer);
        var writer = ep.IsWriter ? _sedpPubWriter : _sedpSubWriter;
        writer.Write(buffer.Span, BuildInlineQos(ep.Guid, disposed: false), ep.Guid);
    }

    private static byte[] BuildInlineQos(RtpsGuid key, bool disposed)
    {
        using var buffer = new RtpsBuffer(64);
        var w = new ParameterListWriter(buffer);
        w.WriteGuid(ParameterId.KeyHash, key);
        if (disposed)
        {
            w.WriteRaw(ParameterId.StatusInfo, s_statusDisposedUnregistered);
        }

        w.WriteSentinel();
        return buffer.Span.ToArray();
    }

    private void MatchLocal(LocalEndpoint writer, LocalEndpoint reader)
    {
        if (reader.IgnoreLocalPublications)
        {
            return;
        }

        if (!QosMatching.IsCompatible(writer.Qos, reader.Qos))
        {
            QosMatching.Check(writer.Qos, reader.Qos, out var reason);
            writer.RaiseIncompatible(reason);
            reader.RaiseIncompatible(reason);
            return;
        }

        var rq = reader.Qos.Resolve();
        reader.Reader!.AddWriter(new WriterProxy { Guid = writer.Guid, IsLocal = true, Reliable = false });
        lock (_gate)
        {
            AddRoute(writer.Guid, reader.Reader);
        }

        writer.Writer!.AddReader(new ReaderProxy
        {
            Guid = reader.Guid,
            Local = reader.Reader,
            Reliable = rq.Reliability == ReliabilityPolicy.Reliable,
            TransientLocal = rq.Durability == DurabilityPolicy.TransientLocal,
        });
        writer.RefreshMatched();
        reader.RefreshMatched();
    }

    private void MatchRemote(LocalEndpoint local, EndpointData remote)
    {
        var offered = local.IsWriter ? local.Qos : remote.Qos;
        var requested = local.IsWriter ? remote.Qos : local.Qos;
        if (!QosMatching.IsCompatible(offered, requested))
        {
            QosMatching.Check(offered, requested, out var reason);
            local.RaiseIncompatible(reason);
            return;
        }

        List<IPEndPoint> endpoints;
        lock (_gate)
        {
            endpoints = remote.UnicastLocators.Count > 0
                ? _transport.Select(remote.UnicastLocators)
                : _participants.TryGetValue(remote.Guid.Prefix, out var p) ? p.UserEndpoints : [];
        }

        if (local.Writer is { } w)
        {
            var rq = remote.Qos.Resolve();
            if (!w.HasReader(remote.Guid))
            {
                w.AddReader(new ReaderProxy
                {
                    Guid = remote.Guid,
                    Endpoints = endpoints,
                    Reliable = rq.Reliability == ReliabilityPolicy.Reliable && w.Reliable,
                    TransientLocal = rq.Durability == DurabilityPolicy.TransientLocal,
                });
            }
        }
        else
        {
            var reader = local.Reader!;
            if (local.IgnoreLocalPublications && remote.Guid.Prefix == Prefix)
            {
                return;
            }

            if (!reader.HasWriter(remote.Guid))
            {
                reader.AddWriter(new WriterProxy
                {
                    Guid = remote.Guid,
                    Endpoints = endpoints,
                    Reliable = reader.Reliable && remote.Qos.Resolve().Reliability == ReliabilityPolicy.Reliable,
                });
                lock (_gate)
                {
                    AddRoute(remote.Guid, reader);
                }
            }
        }

        local.RefreshMatched();
    }

    private void AddRoute(RtpsGuid writer, RtpsReader reader)
    {
        if (!_readersByRemoteWriter.TryGetValue(writer, out var list))
        {
            _readersByRemoteWriter[writer] = list = [];
        }

        if (!list.Contains(reader))
        {
            list.Add(reader);
        }
    }

    // ------------------------------------------------------------------ SPDP

    private void AnnounceParticipant(IReadOnlyList<IPEndPoint>? only = null)
    {
        if (Volatile.Read(ref _disposed) != 0)
        {
            return;
        }

        var data = new ParticipantData
        {
            Guid = Guid,
            Vendor = Vendor,
            DomainId = DomainId,
            LeaseDuration = _options.ParticipantLeaseDuration,
            BuiltinEndpoints = BuiltinEndpoints.ParticipantAnnouncer | BuiltinEndpoints.ParticipantDetector |
                               BuiltinEndpoints.PublicationsAnnouncer | BuiltinEndpoints.PublicationsDetector |
                               BuiltinEndpoints.SubscriptionsAnnouncer | BuiltinEndpoints.SubscriptionsDetector,
            UserData = Encoding.UTF8.GetBytes($"enclave={Enclave};"),
            EntityName = "ros2net",
        };
        data.MetatrafficUnicast.AddRange(_transport.MetatrafficUnicast);
        data.MetatrafficMulticast.AddRange(_transport.MetatrafficMulticast);
        data.DefaultUnicast.AddRange(_transport.UserUnicast);
        data.DefaultMulticast.AddRange(_transport.UserMulticast);

        using var payload = new RtpsBuffer(512);
        data.Write(payload);
        var qos = BuildInlineQos(Guid, disposed: false);
        lock (_spdpBuilder)
        {
            _spdpBuilder.Begin().InfoTs(RosTime.Now().Nanoseconds)
                .Data(EntityId.SpdpReader, EntityId.SpdpWriter, ++_spdpSn, qos, payload.Span);
            if (only is not null)
            {
                Send(_spdpBuilder.Buffer.Span, only);
                return;
            }

            Send(_spdpBuilder.Buffer.Span, _transport.DiscoveryPeers);
            List<IPEndPoint> known;
            lock (_gate)
            {
                known = _participants.Values.SelectMany(p => p.MetaEndpoints).Distinct().ToList();
            }

            Send(_spdpBuilder.Buffer.Span, known);
        }
    }

    private void AnnounceDisposal()
    {
        lock (_spdpBuilder)
        {
            _spdpBuilder.Begin().InfoTs(RosTime.Now().Nanoseconds)
                .Data(EntityId.SpdpReader, EntityId.SpdpWriter, ++_spdpSn, BuildInlineQos(Guid, disposed: true), default);
            Send(_spdpBuilder.Buffer.Span, _transport.DiscoveryPeers);
            List<IPEndPoint> known;
            lock (_gate)
            {
                known = _participants.Values.SelectMany(p => p.MetaEndpoints).Distinct().ToList();
            }

            Send(_spdpBuilder.Buffer.Span, known);
        }
    }

    private void HandleSpdp(in ReceiveState state, in DataSubmessage data)
    {
        if (state.SourcePrefix == Prefix)
        {
            return;
        }

        if (!data.InlineQos.IsEmpty)
        {
            var qos = ParameterList.Parse(data.InlineQos, data.LittleEndian, out _);
            if (qos.TryGet(ParameterId.StatusInfo, out var status) && status.Length >= 4 && (status[3] & 0x03) != 0)
            {
                RemoveParticipant(qos.GetGuid(ParameterId.KeyHash)?.Prefix ?? state.SourcePrefix, "disposed");
                return;
            }
        }

        if (data.Payload.IsEmpty)
        {
            return;
        }

        var pd = ParticipantData.Read(ParameterList.ParsePayload(data.Payload));
        if (pd.Guid.Prefix == Prefix || (pd.DomainId is { } d && d != DomainId))
        {
            return;
        }

        var lease = (long)(Math.Min(pd.LeaseDuration.TotalSeconds, 3600) * Stopwatch.Frequency);
        bool isNew;
        RemoteParticipant rp;
        lock (_gate)
        {
            isNew = !_participants.TryGetValue(pd.Guid.Prefix, out rp!);
            if (isNew)
            {
                rp = new RemoteParticipant { Data = pd };
                _participants[pd.Guid.Prefix] = rp;
            }

            rp.Data = pd;
            rp.LeaseDeadline = Stopwatch.GetTimestamp() + lease;
            rp.MetaEndpoints = _transport.Select(pd.MetatrafficUnicast);
            rp.UserEndpoints = _transport.Select(pd.DefaultUnicast.Count > 0 ? pd.DefaultUnicast : pd.MetatrafficUnicast);
        }

        if (!isNew)
        {
            return;
        }

        _logger.LogDebug("Discovered participant {Prefix} (vendor {Vendor}) at {Endpoints}", pd.Guid.Prefix, pd.Vendor, string.Join(", ", rp.MetaEndpoints));
        var p = pd.Guid.Prefix;
        var be = pd.BuiltinEndpoints;
        if (be.HasFlag(BuiltinEndpoints.PublicationsDetector))
        {
            _sedpPubWriter.AddReader(new ReaderProxy { Guid = new RtpsGuid(p, EntityId.SedpPublicationsReader), Endpoints = rp.MetaEndpoints, Reliable = true, TransientLocal = true });
        }

        if (be.HasFlag(BuiltinEndpoints.SubscriptionsDetector))
        {
            _sedpSubWriter.AddReader(new ReaderProxy { Guid = new RtpsGuid(p, EntityId.SedpSubscriptionsReader), Endpoints = rp.MetaEndpoints, Reliable = true, TransientLocal = true });
        }

        if (be.HasFlag(BuiltinEndpoints.PublicationsAnnouncer))
        {
            _sedpPubReader.AddWriter(new WriterProxy { Guid = new RtpsGuid(p, EntityId.SedpPublicationsWriter), Endpoints = rp.MetaEndpoints, Reliable = true });
        }

        if (be.HasFlag(BuiltinEndpoints.SubscriptionsAnnouncer))
        {
            _sedpSubReader.AddWriter(new WriterProxy { Guid = new RtpsGuid(p, EntityId.SedpSubscriptionsWriter), Endpoints = rp.MetaEndpoints, Reliable = true });
        }

        // Endpoints announced before the participant itself (SEDP raced SPDP) can match now.
        List<EndpointData> early;
        lock (_gate)
        {
            early = _remoteEndpoints.Values.Where(e => e.Guid.Prefix == p).ToList();
        }

        foreach (var e in early)
        {
            OnRemoteEndpoint(e);
        }

        AnnounceParticipant(rp.MetaEndpoints);
        Changed?.Invoke();
    }

    private void RemoveParticipant(GuidPrefix prefix, string reason)
    {
        List<LocalEndpoint> locals;
        lock (_gate)
        {
            if (!_participants.Remove(prefix))
            {
                return;
            }

            foreach (var k in _remoteEndpoints.Keys.Where(k => k.Prefix == prefix).ToList())
            {
                _remoteEndpoints.Remove(k);
            }

            foreach (var k in _readersByRemoteWriter.Keys.Where(k => k.Prefix == prefix).ToList())
            {
                _readersByRemoteWriter.Remove(k);
            }

            locals = _locals.Values.ToList();
        }

        _logger.LogDebug("Participant {Prefix} removed ({Reason})", prefix, reason);
        foreach (var w in new[] { _sedpPubWriter, _sedpSubWriter })
        {
            w.RemoveReadersOf(prefix);
        }

        foreach (var r in new[] { _sedpPubReader, _sedpSubReader })
        {
            r.RemoveWritersOf(prefix);
        }

        foreach (var l in locals)
        {
            var removed = l.Writer?.RemoveReadersOf(prefix) ?? l.Reader!.RemoveWritersOf(prefix);
            if (removed > 0)
            {
                l.RefreshMatched();
            }
        }

        Changed?.Invoke();
    }

    // ------------------------------------------------------------------ SEDP

    private void OnSedpSample(in RtpsSample sample)
    {
        RtpsGuid? key = null;
        var disposed = false;
        if (!sample.InlineQos.IsEmpty)
        {
            var qos = ParameterList.Parse(sample.InlineQos, sample.InlineQosLittleEndian, out _);
            key = qos.GetGuid(ParameterId.KeyHash);
            disposed = qos.TryGet(ParameterId.StatusInfo, out var status) && status.Length >= 4 && (status[3] & 0x03) != 0;
        }

        if (disposed || sample.Payload.IsEmpty)
        {
            if (key is { } k)
            {
                OnRemoteEndpointRemoved(k);
            }

            return;
        }

        EndpointData data;
        try
        {
            data = EndpointData.Read(ParameterList.ParsePayload(sample.Payload), key);
        }
        catch (RtpsFormatException ex)
        {
            _logger.LogDebug(ex, "Ignoring malformed SEDP sample");
            return;
        }

        if (data.Guid.Prefix != Prefix)
        {
            OnRemoteEndpoint(data);
        }
    }

    private void OnRemoteEndpoint(EndpointData data)
    {
        List<LocalEndpoint> candidates;
        bool participantKnown;
        lock (_gate)
        {
            _remoteEndpoints[data.Guid] = data;
            participantKnown = _participants.ContainsKey(data.Guid.Prefix);
            candidates = _locals.Values.Where(l => l.IsWriter != data.IsWriter && l.Topic == data.TopicName && l.TypeName == data.TypeName).ToList();
        }

        if (participantKnown)
        {
            foreach (var l in candidates)
            {
                MatchRemote(l, data);
            }
        }

        Changed?.Invoke();
    }

    private void OnRemoteEndpointRemoved(RtpsGuid guid)
    {
        List<LocalEndpoint> locals;
        lock (_gate)
        {
            if (!_remoteEndpoints.Remove(guid))
            {
                return;
            }

            _readersByRemoteWriter.Remove(guid);
            locals = _locals.Values.ToList();
        }

        foreach (var l in locals)
        {
            if (l.Writer?.RemoveReader(guid) == true || l.Reader?.RemoveWriter(guid) == true)
            {
                l.RefreshMatched();
            }
        }

        Changed?.Invoke();
    }

    // ------------------------------------------------------------------ ros_discovery_info

    private void PublishDiscoveryInfo()
    {
        if (_infoWriter is null || Volatile.Read(ref _disposed) != 0)
        {
            return;
        }

        ParticipantEntitiesInfo info;
        lock (_gate)
        {
            info = new ParticipantEntitiesInfo(_options.GidSize) { ParticipantGid = Guid.ToGid() };
            foreach (var n in _nodes)
            {
                var eps = _locals.Values.Where(e => !e.Internal && e.NodeName == n.Name && e.NodeNamespace == n.Namespace).ToList();
                info.Nodes.Add(new NodeEntitiesInfo(n.Namespace, n.Name,
                    eps.Where(e => !e.IsWriter).Select(e => e.Guid.ToGid()).ToList(),
                    eps.Where(e => e.IsWriter).Select(e => e.Guid.ToGid()).ToList()));
            }
        }

        CdrWriter.SerializeWith(info, _infoWriter.Writer!, static (span, w) => w.Write(span));
    }

    private void OnDiscoveryInfo(in RtpsSample sample)
    {
        if (sample.Payload.IsEmpty || sample.Writer.Prefix == Prefix)
        {
            return;
        }

        var info = new ParticipantEntitiesInfo(_options.GidSize);
        try
        {
            var reader = new CdrReader(sample.Payload);
            info.Deserialize(ref reader);
        }
        catch (CdrException ex)
        {
            _logger.LogDebug(ex, "Ignoring ros_discovery_info sample (Gid size mismatch? configure RtpsOptions.Distro)");
            return;
        }

        lock (_gate)
        {
            var prefix = RtpsGuid.FromGid(info.ParticipantGid).Prefix;
            if (_participants.TryGetValue(prefix, out var rp))
            {
                rp.Info = info;
            }
            else if (_participants.TryGetValue(sample.Writer.Prefix, out rp))
            {
                rp.Info = info;
            }
        }

        Changed?.Invoke();
    }

    // ------------------------------------------------------------------ graph snapshot

    public (List<LocalEndpoint> Locals, List<EndpointData> Remotes, List<ParticipantEntitiesInfo> Infos) Snapshot()
    {
        lock (_gate)
        {
            return (_locals.Values.Where(l => !l.Internal).ToList(), _remoteEndpoints.Values.ToList(),
                _participants.Values.Where(p => p.Info is not null).Select(p => p.Info!).ToList());
        }
    }

    public int RemoteParticipantCount
    {
        get
        {
            lock (_gate)
            {
                return _participants.Count;
            }
        }
    }

    // ------------------------------------------------------------------ receive path

    private void OnDatagram(ReadOnlyMemory<byte> datagram) => RtpsMessageParser.Parse(datagram.Span, Prefix, this);

    private RtpsReader[] RouteToReaders(in ReceiveState state, EntityId readerId, EntityId writerId)
    {
        lock (_gate)
        {
            if (!readerId.Equals(EntityId.Unknown))
            {
                return _readersById.TryGetValue(readerId, out var r) ? [r] : [];
            }

            if (writerId == EntityId.SedpPublicationsWriter)
            {
                return [_sedpPubReader];
            }

            if (writerId == EntityId.SedpSubscriptionsWriter)
            {
                return [_sedpSubReader];
            }

            return _readersByRemoteWriter.TryGetValue(new RtpsGuid(state.SourcePrefix, writerId), out var list) ? [.. list] : [];
        }
    }

    public void OnData(in ReceiveState state, in DataSubmessage data)
    {
        if (data.WriterId == EntityId.SpdpWriter)
        {
            try
            {
                HandleSpdp(in state, in data);
            }
            catch (RtpsFormatException ex)
            {
                _logger.LogDebug(ex, "Ignoring malformed SPDP data");
            }

            return;
        }

        foreach (var r in RouteToReaders(in state, data.ReaderId, data.WriterId))
        {
            r.OnData(in state, in data);
        }
    }

    public void OnDataFrag(in ReceiveState state, in DataFragSubmessage frag)
    {
        foreach (var r in RouteToReaders(in state, frag.ReaderId, frag.WriterId))
        {
            r.OnDataFrag(in state, in frag);
        }
    }

    public void OnHeartbeat(in ReceiveState state, HeartbeatSubmessage heartbeat)
    {
        foreach (var r in RouteToReaders(in state, heartbeat.ReaderId, heartbeat.WriterId))
        {
            r.OnHeartbeat(in state, heartbeat);
        }
    }

    public void OnGap(in ReceiveState state, GapSubmessage gap)
    {
        foreach (var r in RouteToReaders(in state, gap.ReaderId, gap.WriterId))
        {
            r.OnGap(in state, gap);
        }
    }

    public void OnAckNack(in ReceiveState state, AckNackSubmessage ackNack)
    {
        RtpsWriter? w;
        lock (_gate)
        {
            _writersById.TryGetValue(ackNack.WriterId, out w);
        }

        w?.OnAckNack(new RtpsGuid(state.SourcePrefix, ackNack.ReaderId), ackNack);
    }

    public void OnNackFrag(in ReceiveState state, NackFragSubmessage nackFrag)
    {
        RtpsWriter? w;
        lock (_gate)
        {
            _writersById.TryGetValue(nackFrag.WriterId, out w);
        }

        w?.OnNackFrag(new RtpsGuid(state.SourcePrefix, nackFrag.ReaderId), nackFrag);
    }

    public void Send(ReadOnlySpan<byte> datagram, IReadOnlyList<IPEndPoint> destinations)
    {
        for (var i = 0; i < destinations.Count; i++)
        {
            _transport.Send(datagram, destinations[i]);
        }
    }

    // ------------------------------------------------------------------ timer

    private async Task TimerLoop(CancellationToken token)
    {
        var nextAnnounce = Stopwatch.GetTimestamp();
        var nextLeaseCheck = Stopwatch.GetTimestamp();
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(20));
        try
        {
            while (await timer.WaitForNextTickAsync(token).ConfigureAwait(false))
            {
                var now = Stopwatch.GetTimestamp();
                if (now >= nextAnnounce)
                {
                    AnnounceParticipant();
                    var fast = _announcements++ < 5;
                    nextAnnounce = now + (long)((fast ? 0.2 : _options.AnnouncementPeriod.TotalSeconds) * Stopwatch.Frequency);
                }

                List<RtpsWriter> writers;
                lock (_gate)
                {
                    writers = _writersById.Values.ToList();
                }

                foreach (var w in writers)
                {
                    w.Tick(now);
                }

                if (now >= nextLeaseCheck)
                {
                    nextLeaseCheck = now + (Stopwatch.Frequency / 2);
                    List<GuidPrefix> expired;
                    lock (_gate)
                    {
                        expired = _participants.Where(p => p.Value.LeaseDeadline < now).Select(p => p.Key).ToList();
                    }

                    foreach (var p in expired)
                    {
                        RemoveParticipant(p, "lease expired");
                    }
                }
            }
        }
        catch (OperationCanceledException)
        {
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "RTPS timer loop failed");
        }
    }

    public async ValueTask DisposeAsync()
    {
        if (Interlocked.Exchange(ref _disposed, 1) != 0)
        {
            return;
        }

        try
        {
            AnnounceDisposal();
        }
        catch (ObjectDisposedException)
        {
        }

        _cts.Cancel();
        if (_timer is not null)
        {
            try
            {
                await _timer.ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
            }
        }

        await _transport.DisposeAsync().ConfigureAwait(false);
        List<LocalEndpoint> locals;
        lock (_gate)
        {
            locals = _locals.Values.ToList();
        }

        foreach (var l in locals)
        {
            l.Writer?.Dispose();
            l.Reader?.Dispose();
        }

        _sedpPubWriter?.Dispose();
        _sedpSubWriter?.Dispose();
        _sedpPubReader?.Dispose();
        _sedpSubReader?.Dispose();
        _spdpBuilder.Dispose();
        _cts.Dispose();
    }
}
