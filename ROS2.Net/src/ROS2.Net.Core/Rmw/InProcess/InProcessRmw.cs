using System.Collections.Concurrent;
using ROS2.Net.Serialization;

namespace ROS2.Net.Rmw.InProcess;

/// <summary>Factory for the in-process middleware (identifier <c>inprocess</c>).</summary>
public sealed class InProcessRmwFactory : IRmwFactory
{
    public string Identifier => "inprocess";

    public ValueTask<IRmwImplementation> CreateAsync(RmwContextOptions options, CancellationToken cancellationToken)
    {
        var serialize = options.Settings.TryGetValue("serialize", out var s) && bool.TryParse(s, out var b) && b;
        return ValueTask.FromResult<IRmwImplementation>(new InProcessRmw(options.DomainId, serialize));
    }
}

/// <summary>
/// A middleware that connects every context in the current process on the same domain id. Messages
/// are delivered by reference — zero copy, zero serialization — which makes it the fastest transport
/// for composed systems, simulators and tests. Set <c>serialize=true</c> to round-trip every message
/// through CDR instead, which isolates instances and exercises the wire format.
/// </summary>
/// <remarks>Subscribers receive the publisher's instance and must treat it as read-only.</remarks>
public sealed class InProcessRmw : IRmwImplementation, IRmwGraph
{
    private static long s_contextCounter;
    private readonly InProcessDomain _domain;
    private readonly long _contextId = Interlocked.Increment(ref s_contextCounter);
    private readonly byte[] _prefix = new byte[12];
    private int _entityCounter;

    public InProcessRmw(int domainId = 0, bool serializeMessages = false)
    {
        _domain = InProcessDomain.Get(domainId);
        SerializeMessages = serializeMessages;
        System.Security.Cryptography.RandomNumberGenerator.Fill(_prefix);
        _domain.Changed += OnDomainChanged;
    }

    public string Identifier => "inprocess";

    public bool SupportsIgnoreLocalPublications => true;

    public bool SerializeMessages { get; }

    public IRmwGraph Graph => this;

    public event Action? Changed;

    internal long ContextId => _contextId;

    internal InProcessDomain Domain => _domain;

    internal Gid NextGid() => Gid.Create(_prefix, (uint)Interlocked.Increment(ref _entityCounter));

    public IRmwNode CreateNode(string name, string @namespace) => new InProcessNode(this, name, @namespace);

    public IReadOnlyList<NodeInfo> GetNodes() => _domain.GetNodes();

    public IReadOnlyList<EndpointInfo> GetEndpoints() => _domain.GetEndpoints();

    private void OnDomainChanged() => Changed?.Invoke();

    public ValueTask DisposeAsync()
    {
        _domain.Changed -= OnDomainChanged;
        _domain.RemoveContext(_contextId);
        return ValueTask.CompletedTask;
    }
}

/// <summary>A sample in flight: the message object, its CDR encoding, or both (computed lazily).</summary>
internal sealed class InProcessSample(IRosMessage? message, byte[]? cdr)
{
    private IRosMessage? _message = message;
    private byte[]? _cdr = cdr;

    public byte[] Cdr => _cdr ??= CdrWriter.Serialize(_message!);

    public T Get<T>(bool forceCopy)
        where T : class, IRosMessage, new()
    {
        if (!forceCopy && _message is T typed)
        {
            return typed;
        }

        return CdrReader.Deserialize<T>(Cdr);
    }

    public IRosMessage? Message => _message;
}

internal sealed class InProcessDomain
{
    private static readonly ConcurrentDictionary<int, InProcessDomain> s_domains = new();

    private readonly object _gate = new();
    private readonly List<NodeEntry> _nodes = [];
    private readonly List<PublisherEntry> _publishers = [];
    private readonly List<SubscriptionEntry> _subscriptions = [];
    private readonly List<ServerEntry> _servers = [];
    private readonly List<ClientEntry> _clients = [];

    public static InProcessDomain Get(int domainId) => s_domains.GetOrAdd(domainId, _ => new InProcessDomain());

    public event Action? Changed;

    public sealed record NodeEntry(long ContextId, NodeInfo Info);

    public sealed class PublisherEntry
    {
        public required long ContextId { get; init; }
        public required RmwTopicSpec Spec { get; init; }
        public required Gid Gid { get; init; }
        public required NodeInfo Node { get; init; }
        public required Action<int> OnMatched { get; init; }
        public Queue<(InProcessSample Sample, MessageInfo Info)> History { get; } = new();
        public long Sequence;
    }

    public sealed class SubscriptionEntry
    {
        public required long ContextId { get; init; }
        public required RmwTopicSpec Spec { get; init; }
        public required Gid Gid { get; init; }
        public required NodeInfo Node { get; init; }
        public required Action<InProcessSample, MessageInfo> Deliver { get; init; }
        public required Action<int> OnMatched { get; init; }
    }

    public sealed class ServerEntry
    {
        public required long ContextId { get; init; }
        public required RmwServiceSpec Spec { get; init; }
        public required Gid Gid { get; init; }
        public required NodeInfo Node { get; init; }
        public required Action<RequestId, InProcessSample> Deliver { get; init; }
    }

    public sealed class ClientEntry
    {
        public required long ContextId { get; init; }
        public required RmwServiceSpec Spec { get; init; }
        public required Gid Gid { get; init; }
        public required NodeInfo Node { get; init; }
        public required Action<long, InProcessSample> Deliver { get; init; }
        public long Sequence;
    }

    public void AddNode(NodeEntry node)
    {
        lock (_gate)
        {
            _nodes.Add(node);
        }

        Changed?.Invoke();
    }

    public void RemoveNode(NodeEntry node)
    {
        lock (_gate)
        {
            _nodes.Remove(node);
        }

        Changed?.Invoke();
    }

    public void RemoveContext(long contextId)
    {
        lock (_gate)
        {
            _nodes.RemoveAll(n => n.ContextId == contextId);
            _publishers.RemoveAll(p => p.ContextId == contextId);
            _subscriptions.RemoveAll(s => s.ContextId == contextId);
            _servers.RemoveAll(s => s.ContextId == contextId);
            _clients.RemoveAll(c => c.ContextId == contextId);
        }

        Changed?.Invoke();
    }

    private static bool Matches(RmwTopicSpec pub, RmwTopicSpec sub) =>
        pub.TopicName == sub.TopicName && pub.Type.Name == sub.Type.Name && QosMatching.IsCompatible(pub.Qos, sub.Qos);

    public void AddPublisher(PublisherEntry pub)
    {
        List<SubscriptionEntry> matched;
        lock (_gate)
        {
            _publishers.Add(pub);
            matched = _subscriptions.Where(s => Matches(pub.Spec, s.Spec)).ToList();
        }

        pub.OnMatched(matched.Count);
        foreach (var s in matched)
        {
            s.OnMatched(CountPublishersFor(s));
        }

        Changed?.Invoke();
    }

    public void RemovePublisher(PublisherEntry pub)
    {
        List<SubscriptionEntry> matched;
        lock (_gate)
        {
            _publishers.Remove(pub);
            matched = _subscriptions.Where(s => Matches(pub.Spec, s.Spec)).ToList();
        }

        foreach (var s in matched)
        {
            s.OnMatched(CountPublishersFor(s));
        }

        Changed?.Invoke();
    }

    public void AddSubscription(SubscriptionEntry sub)
    {
        List<PublisherEntry> matched;
        List<(InProcessSample, MessageInfo)> late = [];
        lock (_gate)
        {
            _subscriptions.Add(sub);
            matched = _publishers.Where(p => Matches(p.Spec, sub.Spec)).ToList();
            if (sub.Spec.Qos.Resolve().Durability == DurabilityPolicy.TransientLocal)
            {
                foreach (var p in matched)
                {
                    if (sub.Spec.Options.IgnoreLocalPublications && p.ContextId == sub.ContextId)
                    {
                        continue;
                    }

                    late.AddRange(p.History);
                }
            }
        }

        sub.OnMatched(matched.Count);
        foreach (var p in matched)
        {
            p.OnMatched(CountSubscriptionsFor(p));
        }

        foreach (var (sample, info) in late)
        {
            sub.Deliver(sample, info);
        }

        Changed?.Invoke();
    }

    public void RemoveSubscription(SubscriptionEntry sub)
    {
        List<PublisherEntry> matched;
        lock (_gate)
        {
            _subscriptions.Remove(sub);
            matched = _publishers.Where(p => Matches(p.Spec, sub.Spec)).ToList();
        }

        foreach (var p in matched)
        {
            p.OnMatched(CountSubscriptionsFor(p));
        }

        Changed?.Invoke();
    }

    private int CountPublishersFor(SubscriptionEntry s)
    {
        lock (_gate)
        {
            return _publishers.Count(p => Matches(p.Spec, s.Spec));
        }
    }

    private int CountSubscriptionsFor(PublisherEntry p)
    {
        lock (_gate)
        {
            return _subscriptions.Count(s => Matches(p.Spec, s.Spec));
        }
    }

    public int MatchedSubscriptions(PublisherEntry p) => CountSubscriptionsFor(p);

    public int MatchedPublishers(SubscriptionEntry s) => CountPublishersFor(s);

    [ThreadStatic]
    private static List<SubscriptionEntry>? t_targets;

    public void Publish(PublisherEntry pub, InProcessSample sample)
    {
        var info = new MessageInfo(RosTime.Now(), RosTime.Now(), pub.Gid, Interlocked.Increment(ref pub.Sequence));
        var targets = t_targets ??= [];
        targets.Clear();
        lock (_gate)
        {
            var qos = pub.Spec.Qos.Resolve();
            if (qos.Durability == DurabilityPolicy.TransientLocal)
            {
                pub.History.Enqueue((sample, info));
                while (pub.History.Count > qos.Capacity)
                {
                    pub.History.Dequeue();
                }
            }

            foreach (var s in _subscriptions)
            {
                if (Matches(pub.Spec, s.Spec) && !(s.Spec.Options.IgnoreLocalPublications && s.ContextId == pub.ContextId))
                {
                    targets.Add(s);
                }
            }
        }

        // Deliver outside the lock: handlers may create entities or publish.
        var count = targets.Count;
        for (var i = 0; i < count; i++)
        {
            targets[i].Deliver(sample, info);
        }

        targets.Clear();
    }

    public void AddServer(ServerEntry s)
    {
        lock (_gate)
        {
            _servers.Add(s);
        }

        Changed?.Invoke();
    }

    public void RemoveServer(ServerEntry s)
    {
        lock (_gate)
        {
            _servers.Remove(s);
        }

        Changed?.Invoke();
    }

    public void AddClient(ClientEntry c)
    {
        lock (_gate)
        {
            _clients.Add(c);
        }

        Changed?.Invoke();
    }

    public void RemoveClient(ClientEntry c)
    {
        lock (_gate)
        {
            _clients.Remove(c);
        }

        Changed?.Invoke();
    }

    public bool HasServer(RmwServiceSpec spec)
    {
        lock (_gate)
        {
            return _servers.Exists(s => s.Spec.ServiceName == spec.ServiceName && s.Spec.Type.Name == spec.Type.Name);
        }
    }

    public long SendRequest(ClientEntry client, InProcessSample request)
    {
        var seq = Interlocked.Increment(ref client.Sequence);
        List<ServerEntry> servers;
        lock (_gate)
        {
            servers = _servers.Where(s => s.Spec.ServiceName == client.Spec.ServiceName && s.Spec.Type.Name == client.Spec.Type.Name).ToList();
        }

        // Like DDS, every matched server receives the request; the client keeps the first response.
        foreach (var s in servers)
        {
            s.Deliver(new RequestId(client.Gid, seq), request);
        }

        return seq;
    }

    public void SendResponse(RequestId id, InProcessSample response)
    {
        ClientEntry? client;
        lock (_gate)
        {
            client = _clients.Find(c => c.Gid == id.ClientGid);
        }

        client?.Deliver(id.SequenceNumber, response);
    }

    public IReadOnlyList<NodeInfo> GetNodes()
    {
        lock (_gate)
        {
            return _nodes.Select(n => n.Info).ToList();
        }
    }

    public IReadOnlyList<EndpointInfo> GetEndpoints()
    {
        lock (_gate)
        {
            var list = new List<EndpointInfo>();
            list.AddRange(_publishers.Select(p => new EndpointInfo(p.Node.Name, p.Node.Namespace, p.Spec.TopicName, p.Spec.Type.Name, EndpointKind.Publisher, p.Gid, p.Spec.Qos)));
            list.AddRange(_subscriptions.Select(s => new EndpointInfo(s.Node.Name, s.Node.Namespace, s.Spec.TopicName, s.Spec.Type.Name, EndpointKind.Subscription, s.Gid, s.Spec.Qos)));
            list.AddRange(_servers.Select(s => new EndpointInfo(s.Node.Name, s.Node.Namespace, s.Spec.ServiceName, s.Spec.Type.Name, EndpointKind.ServiceServer, s.Gid, s.Spec.Qos)));
            list.AddRange(_clients.Select(c => new EndpointInfo(c.Node.Name, c.Node.Namespace, c.Spec.ServiceName, c.Spec.Type.Name, EndpointKind.ServiceClient, c.Gid, c.Spec.Qos)));
            return list;
        }
    }
}

internal sealed class InProcessNode : IRmwNode
{
    private readonly InProcessRmw _rmw;
    private readonly InProcessDomain.NodeEntry _entry;

    public InProcessNode(InProcessRmw rmw, string name, string @namespace)
    {
        _rmw = rmw;
        Name = name;
        Namespace = @namespace;
        _entry = new InProcessDomain.NodeEntry(rmw.ContextId, new NodeInfo(name, @namespace));
        rmw.Domain.AddNode(_entry);
    }

    public string Name { get; }

    public string Namespace { get; }

    private NodeInfo Info => _entry.Info;

    public IRmwPublisher<T> CreatePublisher<T>(RmwTopicSpec spec)
        where T : class, IRosMessage, new() => new Publisher<T>(this, spec);

    public IRmwSerializedPublisher CreateSerializedPublisher(RmwTopicSpec spec) => new SerializedPublisher(this, spec);

    public IRmwSubscription CreateSubscription<T>(RmwTopicSpec spec, RmwMessageHandler<T> handler)
        where T : class, IRosMessage, new()
    {
        var copy = _rmw.SerializeMessages;
        return new Subscription(this, spec, (sample, info) => handler(sample.Get<T>(copy), info));
    }

    public IRmwSubscription CreateSerializedSubscription(RmwTopicSpec spec, RmwSerializedHandler handler) =>
        new Subscription(this, spec, (sample, info) => handler(sample.Cdr, info));

    public IRmwServiceServer<TResponse> CreateServiceServer<TRequest, TResponse>(RmwServiceSpec spec, Action<RequestId, TRequest> onRequest)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new() =>
        new Server<TResponse>(this, spec, (id, sample) => onRequest(id, sample.Get<TRequest>(_rmw.SerializeMessages)));

    public IRmwServiceClient<TRequest> CreateServiceClient<TRequest, TResponse>(RmwServiceSpec spec, Action<long, TResponse> onResponse)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new() =>
        new Client<TRequest>(this, spec, (seq, sample) => onResponse(seq, sample.Get<TResponse>(_rmw.SerializeMessages)));

    public void Dispose() => _rmw.Domain.RemoveNode(_entry);

    private abstract class Endpoint : IRmwEndpoint
    {
        protected Endpoint(InProcessNode node)
        {
            Node = node;
            Gid = node._rmw.NextGid();
        }

        protected InProcessNode Node { get; }

        public Gid Gid { get; }

        public abstract int MatchedCount { get; }

        public event Action<QosEvent>? QosEventRaised;

        private int _matchedTotal;
        private int _lastMatched;

        protected void RaiseMatched(int current)
        {
            var change = current - Interlocked.Exchange(ref _lastMatched, current);
            if (change == 0)
            {
                return;
            }

            var total = change > 0 ? Interlocked.Add(ref _matchedTotal, change) : _matchedTotal;
            QosEventRaised?.Invoke(new QosEvent(QosEventKind.Matched, total, change, current));
        }

        public abstract void Dispose();
    }

    private class PublisherBase : Endpoint
    {
        protected readonly InProcessDomain.PublisherEntry Entry;
        private int _disposed;

        protected PublisherBase(InProcessNode node, RmwTopicSpec spec)
            : base(node)
        {
            Entry = new InProcessDomain.PublisherEntry
            {
                ContextId = node._rmw.ContextId,
                Spec = spec,
                Gid = Gid,
                Node = node.Info,
                OnMatched = RaiseMatched,
            };
            node._rmw.Domain.AddPublisher(Entry);
        }

        public override int MatchedCount => Node._rmw.Domain.MatchedSubscriptions(Entry);

        public void AssertLiveliness()
        {
        }

        public override void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) == 0)
            {
                Node._rmw.Domain.RemovePublisher(Entry);
            }
        }
    }

    private sealed class Publisher<T>(InProcessNode node, RmwTopicSpec spec) : PublisherBase(node, spec), IRmwPublisher<T>
        where T : class, IRosMessage
    {
        public void Publish(T message) => Node._rmw.Domain.Publish(Entry, new InProcessSample(message, null));
    }

    private sealed class SerializedPublisher(InProcessNode node, RmwTopicSpec spec) : PublisherBase(node, spec), IRmwSerializedPublisher
    {
        public void Publish(ReadOnlySpan<byte> cdr) => Node._rmw.Domain.Publish(Entry, new InProcessSample(null, cdr.ToArray()));
    }

    private sealed class Subscription : Endpoint, IRmwSubscription
    {
        private readonly InProcessDomain.SubscriptionEntry _entry;
        private int _disposed;

        public Subscription(InProcessNode node, RmwTopicSpec spec, Action<InProcessSample, MessageInfo> deliver)
            : base(node)
        {
            _entry = new InProcessDomain.SubscriptionEntry
            {
                ContextId = node._rmw.ContextId,
                Spec = spec,
                Gid = Gid,
                Node = node.Info,
                Deliver = deliver,
                OnMatched = RaiseMatched,
            };
            node._rmw.Domain.AddSubscription(_entry);
        }

        public override int MatchedCount => Node._rmw.Domain.MatchedPublishers(_entry);

        public override void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) == 0)
            {
                Node._rmw.Domain.RemoveSubscription(_entry);
            }
        }
    }

    private sealed class Server<TResponse> : Endpoint, IRmwServiceServer<TResponse>
        where TResponse : class, IRosMessage
    {
        private readonly InProcessDomain.ServerEntry _entry;
        private int _disposed;

        public Server(InProcessNode node, RmwServiceSpec spec, Action<RequestId, InProcessSample> deliver)
            : base(node)
        {
            _entry = new InProcessDomain.ServerEntry { ContextId = node._rmw.ContextId, Spec = spec, Gid = Gid, Node = node.Info, Deliver = deliver };
            node._rmw.Domain.AddServer(_entry);
        }

        public override int MatchedCount => 0;

        public void SendResponse(RequestId requestId, TResponse response) =>
            Node._rmw.Domain.SendResponse(requestId, new InProcessSample(response, null));

        public override void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) == 0)
            {
                Node._rmw.Domain.RemoveServer(_entry);
            }
        }
    }

    private sealed class Client<TRequest> : Endpoint, IRmwServiceClient<TRequest>
        where TRequest : class, IRosMessage
    {
        private readonly InProcessDomain.ClientEntry _entry;
        private int _disposed;

        public Client(InProcessNode node, RmwServiceSpec spec, Action<long, InProcessSample> deliver)
            : base(node)
        {
            _entry = new InProcessDomain.ClientEntry { ContextId = node._rmw.ContextId, Spec = spec, Gid = Gid, Node = node.Info, Deliver = deliver };
            node._rmw.Domain.AddClient(_entry);
        }

        public override int MatchedCount => IsServiceAvailable ? 1 : 0;

        public bool IsServiceAvailable => Node._rmw.Domain.HasServer(_entry.Spec);

        public long SendRequest(TRequest request) => Node._rmw.Domain.SendRequest(_entry, new InProcessSample(request, null));

        public override void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) == 0)
            {
                Node._rmw.Domain.RemoveClient(_entry);
            }
        }
    }
}
