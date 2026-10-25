using System.Collections.Concurrent;
using System.Text.Json;
using ROS2.Net.Serialization;

namespace ROS2.Net.Rmw.Rosbridge;

/// <summary>A node on the rosbridge middleware. Endpoints are bookkept locally and multiplexed over the shared link.</summary>
internal sealed class RosbridgeNode : IRmwNode
{
    private readonly RosbridgeRmw _rmw;
    private readonly NodeInfo _info;

    public RosbridgeNode(RosbridgeRmw rmw, NodeInfo info)
    {
        _rmw = rmw;
        _info = info;
    }

    public string Name => _info.Name;

    public string Namespace => _info.Namespace;

    public IRmwPublisher<T> CreatePublisher<T>(RmwTopicSpec spec)
        where T : class, IRosMessage, new() => new Publisher<T>(this, spec);

    public IRmwSerializedPublisher CreateSerializedPublisher(RmwTopicSpec spec) => new SerializedPublisher(this, spec);

    public IRmwSubscription CreateSubscription<T>(RmwTopicSpec spec, RmwMessageHandler<T> handler)
        where T : class, IRosMessage, new() =>
        new Subscription(this, spec, (json, info) =>
        {
            var message = new T();
            message.ReadJson(json);
            handler(message, info);
        });

    public IRmwSubscription CreateSerializedSubscription(RmwTopicSpec spec, RmwSerializedHandler handler) =>
        new Subscription(this, spec, (json, info) =>
        {
            // rosbridge carries JSON; rebuild CDR through the type's generated (or runtime) factory.
            var message = spec.Type.Factory();
            message.ReadJson(json);
            CdrWriter.SerializeWith(message, (handler, info), static (cdr, s) => s.handler(cdr, s.info));
        });

    public IRmwServiceServer<TResponse> CreateServiceServer<TRequest, TResponse>(RmwServiceSpec spec, Action<RequestId, TRequest> onRequest)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new() => new Server<TRequest, TResponse>(this, spec, onRequest);

    public IRmwServiceClient<TRequest> CreateServiceClient<TRequest, TResponse>(RmwServiceSpec spec, Action<long, TResponse> onResponse)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new() => new Client<TRequest, TResponse>(this, spec, onResponse);

    public void Dispose() => _rmw.RemoveNode(_info);

    private abstract class Endpoint : IRmwEndpoint
    {
        private int _disposed;

        protected Endpoint(RosbridgeNode node, string name, string type, EndpointKind kind, QosProfile qos)
        {
            Node = node;
            Gid = node._rmw.NextGid();
            Info = new EndpointInfo(node.Name, node.Namespace, name, type, kind, Gid, qos);
            node._rmw.AddEndpoint(Info);
        }

        protected RosbridgeNode Node { get; }

        protected RosbridgeRmw Rmw => Node._rmw;

        protected EndpointInfo Info { get; }

        public Gid Gid { get; }

        public abstract int MatchedCount { get; }

        // rosbridge does not report matching or QoS events.
        public event Action<QosEvent>? QosEventRaised
        {
            add { }
            remove { }
        }

        public void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) == 0)
            {
                Rmw.RemoveEndpoint(Info);
                Close();
            }
        }

        protected abstract void Close();
    }

    private class PublisherBase : Endpoint
    {
        protected PublisherBase(RosbridgeNode node, RmwTopicSpec spec)
            : base(node, spec.TopicName, spec.Type.Name, EndpointKind.Publisher, spec.Qos)
        {
            Spec = spec;
            Rmw.Advertise(spec.TopicName, spec);
        }

        protected RmwTopicSpec Spec { get; }

        public override int MatchedCount => Rmw.CountEndpoints(Spec.TopicName, EndpointKind.Subscription, Gid);

        public void AssertLiveliness()
        {
        }

        protected override void Close() => Rmw.Unadvertise(Spec.TopicName);
    }

    private sealed class Publisher<T>(RosbridgeNode node, RmwTopicSpec spec) : PublisherBase(node, spec), IRmwPublisher<T>
        where T : class, IRosMessage
    {
        public void Publish(T message) => Rmw.Publish(Spec.TopicName, message);
    }

    private sealed class SerializedPublisher(RosbridgeNode node, RmwTopicSpec spec) : PublisherBase(node, spec), IRmwSerializedPublisher
    {
        public void Publish(ReadOnlySpan<byte> cdr)
        {
            var message = Spec.Type.Factory();
            var reader = new CdrReader(cdr);
            message.Deserialize(ref reader);
            Rmw.Publish(Spec.TopicName, message);
        }
    }

    private sealed class Subscription : Endpoint, IRmwSubscription
    {
        private readonly RmwTopicSpec _spec;
        private readonly RosbridgeRmw.LocalSubscriber _local;

        public Subscription(RosbridgeNode node, RmwTopicSpec spec, Action<JsonElement, MessageInfo> deliver)
            : base(node, spec.TopicName, spec.Type.Name, EndpointKind.Subscription, spec.Qos)
        {
            _spec = spec;
            _local = Rmw.Subscribe(spec.TopicName, spec, deliver);
        }

        public override int MatchedCount => Rmw.CountEndpoints(_spec.TopicName, EndpointKind.Publisher, Gid);

        protected override void Close() => Rmw.Unsubscribe(_spec.TopicName, _local);
    }

    private sealed class Server<TRequest, TResponse> : Endpoint, IRmwServiceServer<TResponse>
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new()
    {
        private readonly RmwServiceSpec _spec;
        private readonly RosbridgeRmw.ServiceAdvertisement _advertisement;
        private readonly ConcurrentDictionary<long, string> _callIds = new();
        private long _sequence;

        public Server(RosbridgeNode node, RmwServiceSpec spec, Action<RequestId, TRequest> onRequest)
            : base(node, spec.ServiceName, spec.Type.Name, EndpointKind.ServiceServer, spec.Qos)
        {
            _spec = spec;
            _advertisement = new RosbridgeRmw.ServiceAdvertisement(spec.ServiceName, spec.Type.Name, (callId, args) =>
            {
                var request = new TRequest();
                if (args.ValueKind == JsonValueKind.Object)
                {
                    request.ReadJson(args);
                }

                var seq = Interlocked.Increment(ref _sequence);
                _callIds[seq] = callId;
                onRequest(new RequestId(Gid, seq), request);
            });
            Rmw.AdvertiseService(_advertisement);
        }

        public override int MatchedCount => 0;

        public void SendResponse(RequestId requestId, TResponse response)
        {
            if (_callIds.TryRemove(requestId.SequenceNumber, out var callId))
            {
                Rmw.SendServiceResponse(_spec.ServiceName, callId, response);
            }
        }

        protected override void Close() => Rmw.UnadvertiseService(_advertisement);
    }

    private sealed class Client<TRequest, TResponse> : Endpoint, IRmwServiceClient<TRequest>
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new()
    {
        private readonly RmwServiceSpec _spec;
        private readonly Action<long, TResponse> _onResponse;
        private long _sequence;
        private volatile bool _closed;

        public Client(RosbridgeNode node, RmwServiceSpec spec, Action<long, TResponse> onResponse)
            : base(node, spec.ServiceName, spec.Type.Name, EndpointKind.ServiceClient, spec.Qos)
        {
            _spec = spec;
            _onResponse = onResponse;
        }

        public override int MatchedCount => IsServiceAvailable ? 1 : 0;

        public bool IsServiceAvailable => Rmw.IsServiceAvailable(_spec.ServiceName);

        public long SendRequest(TRequest request) =>
            Rmw.CallService<TResponse>(_spec.ServiceName, _spec.Type.Name, request, (seq, res) =>
            {
                if (!_closed)
                {
                    _onResponse(seq, res);
                }
            }, Interlocked.Increment(ref _sequence));

        protected override void Close() => _closed = true;
    }
}
