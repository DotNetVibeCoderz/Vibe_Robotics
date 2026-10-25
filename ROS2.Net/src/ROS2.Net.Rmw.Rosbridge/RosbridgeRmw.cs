using System.Buffers;
using System.Collections.Concurrent;
using System.Globalization;
using System.Text.Json;
using Microsoft.Extensions.Logging;
using ROS2.Net.Serialization;

namespace ROS2.Net.Rmw.Rosbridge;

/// <summary>
/// A middleware that speaks the rosbridge v2 protocol to a <c>rosbridge_server</c>. Every node of
/// the context shares one WebSocket; topics are advertised and subscribed once per topic and fanned
/// out locally. Messages travel as JSON produced by the generated <c>WriteJson</c>/<c>ReadJson</c>.
/// The graph comes from the <c>rosapi</c> services, polled, plus this context's own endpoints.
/// </summary>
/// <remarks>
/// rosbridge echoes a context's own publications back to its subscriptions, so
/// <see cref="SupportsIgnoreLocalPublications"/> is false and intra-process delivery stays off.
/// </remarks>
public sealed class RosbridgeRmw : IRmwImplementation, IRmwGraph
{
    private static long s_instances;

    private readonly RosbridgeOptions _options;
    private readonly ILogger _logger;
    private readonly RosbridgeConnection _connection;
    private readonly object _gate = new();
    private readonly Dictionary<string, TopicSubscription> _subscriptions = new(StringComparer.Ordinal);
    private readonly Dictionary<string, TopicAdvertisement> _advertisements = new(StringComparer.Ordinal);
    private readonly Dictionary<string, ServiceAdvertisement> _services = new(StringComparer.Ordinal);
    private readonly ConcurrentDictionary<string, Action<JsonElement, bool>> _calls = new(StringComparer.Ordinal);
    private readonly List<NodeInfo> _localNodes = [];
    private readonly List<EndpointInfo> _localEndpoints = [];
    private readonly byte[] _prefix = new byte[12];
    private readonly CancellationTokenSource _cts = new();
    private readonly string _clientTag;
    private IReadOnlyList<NodeInfo> _remoteNodes = [];
    private IReadOnlyList<EndpointInfo> _remoteEndpoints = [];
    private HashSet<string> _remoteServices = new(StringComparer.Ordinal);
    private string _graphSignature = string.Empty;
    private volatile bool _rosapiAvailable;
    private volatile bool _rosapiChecked;
    private Task? _graphLoop;
    private long _idCounter;
    private int _entityCounter;

    internal RosbridgeRmw(RosbridgeOptions options, ILogger logger)
    {
        _options = options;
        _logger = logger;
        _clientTag = "ros2net" + Interlocked.Increment(ref s_instances).ToString(CultureInfo.InvariantCulture) + "_" + Environment.ProcessId.ToString(CultureInfo.InvariantCulture);
        System.Security.Cryptography.RandomNumberGenerator.Fill(_prefix);
        _connection = new RosbridgeConnection(options, logger, OnOp);
        _connection.Reconnected += Resync;
    }

    public string Identifier => "rosbridge";

    public bool SupportsIgnoreLocalPublications => false;

    public IRmwGraph Graph => this;

    /// <summary>The rosbridge endpoint.</summary>
    public Uri Url => _options.Url;

    public bool IsConnected => _connection.IsConnected;

    /// <summary>True once a rosapi query has succeeded; without rosapi, service availability is assumed.</summary>
    public bool IsRosapiAvailable => _rosapiAvailable;

    public event Action? Changed;

    internal async Task StartAsync(CancellationToken cancellationToken)
    {
        await _connection.StartAsync(cancellationToken).ConfigureAwait(false);
        if (_options.GraphPollInterval > TimeSpan.Zero)
        {
            _graphLoop = Task.Run(() => GraphLoopAsync(_cts.Token));
        }
        else
        {
            _rosapiChecked = true;
        }
    }

    public IRmwNode CreateNode(string name, string @namespace)
    {
        var info = new NodeInfo(name, @namespace);
        lock (_gate)
        {
            _localNodes.Add(info);
        }

        RaiseChanged();
        return new RosbridgeNode(this, info);
    }

    internal void RemoveNode(NodeInfo info)
    {
        lock (_gate)
        {
            _localNodes.Remove(info);
        }

        RaiseChanged();
    }

    internal Gid NextGid() => Gid.Create(_prefix, (uint)Interlocked.Increment(ref _entityCounter));

    private string NextId(string kind, string name) =>
        $"{kind}:{name}:{_clientTag}:{Interlocked.Increment(ref _idCounter).ToString(CultureInfo.InvariantCulture)}";

    // ------------------------------------------------------------------ graph

    public IReadOnlyList<NodeInfo> GetNodes()
    {
        lock (_gate)
        {
            return _localNodes.Concat(_remoteNodes).Distinct().ToList();
        }
    }

    public IReadOnlyList<EndpointInfo> GetEndpoints()
    {
        lock (_gate)
        {
            return _localEndpoints.Concat(_remoteEndpoints).ToList();
        }
    }

    internal void AddEndpoint(EndpointInfo e)
    {
        lock (_gate)
        {
            _localEndpoints.Add(e);
        }

        RaiseChanged();
    }

    internal void RemoveEndpoint(EndpointInfo e)
    {
        lock (_gate)
        {
            _localEndpoints.Remove(e);
        }

        RaiseChanged();
    }

    internal int CountEndpoints(string name, EndpointKind kind, Gid exclude)
    {
        lock (_gate)
        {
            return _localEndpoints.Concat(_remoteEndpoints).Count(e => e.Name == name && e.Kind == kind && e.Gid != exclude);
        }
    }

    internal bool IsServiceAvailable(string service)
    {
        if (!_rosapiChecked)
        {
            return false;
        }

        if (!_rosapiAvailable)
        {
            return true; // No rosapi: nothing to ask, so assume the service exists.
        }

        lock (_gate)
        {
            return _remoteServices.Contains(service) || _services.ContainsKey(service);
        }
    }

    private void RaiseChanged()
    {
        try
        {
            Changed?.Invoke();
        }
        catch (Exception ex) when (ex is not OutOfMemoryException)
        {
            _logger.LogWarning(ex, "Graph change handler threw");
        }
    }

    /// <summary>Forces a graph refresh from rosapi now.</summary>
    public async Task RefreshGraphAsync(CancellationToken cancellationToken = default)
    {
        var timeout = TimeSpan.FromSeconds(3);
        var topicsTask = CallRawAsync("/rosapi/topics", null, timeout, cancellationToken);
        var nodesTask = CallRawAsync("/rosapi/nodes", null, timeout, cancellationToken);
        var servicesTask = CallRawAsync("/rosapi/services", null, timeout, cancellationToken);
        var topics = await topicsTask.ConfigureAwait(false);
        var nodes = await nodesTask.ConfigureAwait(false);
        var services = await servicesTask.ConfigureAwait(false);

        if (topics is null && nodes is null && services is null)
        {
            if (!_rosapiAvailable && !_rosapiChecked)
            {
                _logger.LogInformation("rosapi is not reachable through rosbridge; graph introspection is limited to this process and services are assumed available.");
            }

            _rosapiChecked = true;
            return;
        }

        var topicTypes = new Dictionary<string, string>(StringComparer.Ordinal);
        if (topics is { } t && t.TryGetProperty("topics", out var tn) && tn.ValueKind == JsonValueKind.Array)
        {
            var names = tn.EnumerateArray().Select(x => x.GetString() ?? string.Empty).ToList();
            var types = t.TryGetProperty("types", out var tt) && tt.ValueKind == JsonValueKind.Array
                ? tt.EnumerateArray().Select(x => x.GetString() ?? string.Empty).ToList()
                : [];
            for (var i = 0; i < names.Count; i++)
            {
                topicTypes[names[i]] = i < types.Count ? NormalizeType(types[i]) : string.Empty;
            }
        }

        var serviceNames = services is { } s && s.TryGetProperty("services", out var sn) && sn.ValueKind == JsonValueKind.Array
            ? sn.EnumerateArray().Select(x => x.GetString() ?? string.Empty).Where(x => x.Length > 0).ToList()
            : [];

        var nodeNames = nodes is { } n && n.TryGetProperty("nodes", out var nn) && nn.ValueKind == JsonValueKind.Array
            ? nn.EnumerateArray().Select(x => x.GetString() ?? string.Empty).Where(x => x.Length > 0).ToList()
            : [];

        var remoteNodes = nodeNames.Select(SplitNode).ToList();
        var endpoints = new List<EndpointInfo>();
        var attributedTopics = new HashSet<string>(StringComparer.Ordinal);
        var attributedServices = new HashSet<string>(StringComparer.Ordinal);
        var details = await Task.WhenAll(nodeNames.Select(async name =>
        {
            var args = JsonSerializer.SerializeToElement(new Dictionary<string, string> { ["node"] = name });
            return (Name: name, Details: await CallRawAsync("/rosapi/node_details", args, timeout, cancellationToken).ConfigureAwait(false));
        })).ConfigureAwait(false);

        foreach (var (name, d) in details)
        {
            if (d is not { } detail)
            {
                continue;
            }

            var node = SplitNode(name);
            foreach (var (prop, kind) in new[] { ("publishing", EndpointKind.Publisher), ("subscribing", EndpointKind.Subscription), ("services", EndpointKind.ServiceServer) })
            {
                if (!detail.TryGetProperty(prop, out var list) || list.ValueKind != JsonValueKind.Array)
                {
                    continue;
                }

                foreach (var item in list.EnumerateArray())
                {
                    var endpoint = item.GetString() ?? string.Empty;
                    if (endpoint.Length == 0)
                    {
                        continue;
                    }

                    (kind == EndpointKind.ServiceServer ? attributedServices : attributedTopics).Add(endpoint);
                    endpoints.Add(new EndpointInfo(node.Name, node.Namespace, endpoint, kind == EndpointKind.ServiceServer ? string.Empty : topicTypes.GetValueOrDefault(endpoint, string.Empty), kind, Gid.Empty, QosProfile.Default));
                }
            }
        }

        // Topics and services rosapi lists without an owner (rosapi without node_details, or bridges).
        foreach (var (topic, type) in topicTypes)
        {
            if (!attributedTopics.Contains(topic))
            {
                endpoints.Add(new EndpointInfo(string.Empty, "/", topic, type, EndpointKind.Publisher, Gid.Empty, QosProfile.Default));
            }
        }

        foreach (var svc in serviceNames)
        {
            if (!attributedServices.Contains(svc))
            {
                endpoints.Add(new EndpointInfo(string.Empty, "/", svc, string.Empty, EndpointKind.ServiceServer, Gid.Empty, QosProfile.Default));
            }
        }

        var signature = string.Join('|', remoteNodes.Select(x => x.FullyQualifiedName)) + "#" +
                        string.Join('|', endpoints.Select(e => $"{e.Kind}:{e.Name}:{e.TypeName}:{e.NodeNamespace}/{e.NodeName}").Order(StringComparer.Ordinal));
        bool changed;
        lock (_gate)
        {
            _remoteNodes = remoteNodes;
            _remoteEndpoints = endpoints;
            _remoteServices = new HashSet<string>(serviceNames.Concat(attributedServices), StringComparer.Ordinal);
            changed = signature != _graphSignature;
            _graphSignature = signature;
        }

        _rosapiAvailable = true;
        _rosapiChecked = true;
        if (changed)
        {
            RaiseChanged();
        }
    }

    private async Task GraphLoopAsync(CancellationToken token)
    {
        while (!token.IsCancellationRequested)
        {
            try
            {
                await RefreshGraphAsync(token).ConfigureAwait(false);
                await Task.Delay(_options.GraphPollInterval, token).ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
                return;
            }
            catch (Exception ex) when (ex is not OutOfMemoryException)
            {
                _logger.LogDebug(ex, "rosapi graph refresh failed");
            }
        }
    }

    private static NodeInfo SplitNode(string fqn)
    {
        var slash = fqn.LastIndexOf('/');
        var ns = slash <= 0 ? "/" : fqn[..slash];
        return new NodeInfo(fqn[(slash + 1)..], ns);
    }

    /// <summary>rosapi reports <c>pkg/Name</c> on some distros; the graph uses <c>pkg/msg/Name</c>.</summary>
    private static string NormalizeType(string type)
    {
        var parts = type.Split('/');
        return parts.Length == 2 ? $"{parts[0]}/msg/{parts[1]}" : type;
    }

    // ------------------------------------------------------------------ protocol

    internal static byte[] Op(Action<Utf8JsonWriter> body)
    {
        var buffer = new ArrayBufferWriter<byte>(256);
        using (var writer = new Utf8JsonWriter(buffer, new JsonWriterOptions { SkipValidation = true }))
        {
            writer.WriteStartObject();
            body(writer);
            writer.WriteEndObject();
        }

        return buffer.WrittenSpan.ToArray();
    }

    private void OnOp(JsonElement root)
    {
        var op = root.TryGetProperty("op", out var o) ? o.GetString() : null;
        switch (op)
        {
            case "publish":
                OnPublish(root);
                break;
            case "service_response":
                OnServiceResponse(root);
                break;
            case "call_service":
                OnCallService(root);
                break;
            case "status":
                var level = root.TryGetProperty("level", out var l) ? l.GetString() : "info";
                var msg = root.TryGetProperty("msg", out var m) ? m.GetString() : string.Empty;
                if (level is "error" or "warning")
                {
                    _logger.LogWarning("rosbridge {Level}: {Message}", level, msg);
                }
                else
                {
                    _logger.LogDebug("rosbridge {Level}: {Message}", level, msg);
                }

                break;
        }
    }

    private void OnPublish(JsonElement root)
    {
        var topic = root.GetProperty("topic").GetString() ?? string.Empty;
        TopicSubscription? sub;
        lock (_gate)
        {
            _subscriptions.TryGetValue(topic, out sub);
        }

        if (sub is null || !root.TryGetProperty("msg", out var msg))
        {
            return;
        }

        var now = RosTime.Now();
        var info = new MessageInfo(now, now, Gid.Empty, Interlocked.Increment(ref sub.Sequence));
        foreach (var local in sub.Snapshot())
        {
            try
            {
                local.Deliver(msg, info);
            }
            catch (Exception ex) when (ex is not OutOfMemoryException)
            {
                _logger.LogWarning(ex, "Could not deliver a message on {Topic}", topic);
            }
        }
    }

    private void OnServiceResponse(JsonElement root)
    {
        var id = root.TryGetProperty("id", out var i) ? i.GetString() : null;
        if (id is null || !_calls.TryRemove(id, out var callback))
        {
            return;
        }

        var ok = !root.TryGetProperty("result", out var r) || r.ValueKind != JsonValueKind.False;
        var values = root.TryGetProperty("values", out var v) ? v : default;
        callback(values, ok);
    }

    private void OnCallService(JsonElement root)
    {
        var service = root.GetProperty("service").GetString() ?? string.Empty;
        var id = root.TryGetProperty("id", out var i) ? i.GetString() ?? string.Empty : string.Empty;
        ServiceAdvertisement? adv;
        lock (_gate)
        {
            _services.TryGetValue(service, out adv);
        }

        if (adv is null)
        {
            _connection.Send(Op(w =>
            {
                w.WriteString("op", "service_response");
                w.WriteString("id", id);
                w.WriteString("service", service);
                w.WriteBoolean("result", false);
                w.WriteString("values", $"service {service} is not served by this client");
            }));
            return;
        }

        var args = root.TryGetProperty("args", out var a) ? a : default;
        adv.OnCall(id, args);
    }

    /// <summary>Calls a service with raw JSON args and returns the response values, or null on failure/timeout.</summary>
    internal async Task<JsonElement?> CallRawAsync(string service, JsonElement? args, TimeSpan timeout, CancellationToken cancellationToken)
    {
        var id = NextId("call_service", service);
        var tcs = new TaskCompletionSource<JsonElement?>(TaskCreationOptions.RunContinuationsAsynchronously);
        _calls[id] = (values, ok) => tcs.TrySetResult(ok && values.ValueKind != JsonValueKind.Undefined ? values.Clone() : null);
        _connection.Send(Op(w =>
        {
            w.WriteString("op", "call_service");
            w.WriteString("id", id);
            w.WriteString("service", service);
            if (args is { } x)
            {
                w.WritePropertyName("args");
                x.WriteTo(w);
            }
        }));
        try
        {
            return await tcs.Task.WaitAsync(timeout, cancellationToken).ConfigureAwait(false);
        }
        catch (TimeoutException)
        {
            return null;
        }
        finally
        {
            _calls.TryRemove(id, out _);
        }
    }

    internal long CallService<TResponse>(string service, string type, IRosMessage request, Action<long, TResponse> onResponse, long sequence)
        where TResponse : class, IRosMessage, new()
    {
        var id = NextId("call_service", service);
        _calls[id] = (values, ok) =>
        {
            if (!ok)
            {
                _logger.LogWarning("Service call to {Service} failed: {Detail}", service, values.ValueKind == JsonValueKind.String ? values.GetString() : values.ToString());
                return;
            }

            var response = new TResponse();
            if (values.ValueKind == JsonValueKind.Object)
            {
                response.ReadJson(values);
            }

            onResponse(sequence, response);
        };
        _connection.Send(Op(w =>
        {
            w.WriteString("op", "call_service");
            w.WriteString("id", id);
            w.WriteString("service", service);
            w.WriteString("type", type);
            w.WritePropertyName("args");
            request.WriteJson(w);
        }));
        return sequence;
    }

    // ------------------------------------------------------------------ topics

    internal void Advertise(string topic, RmwTopicSpec spec)
    {
        TopicAdvertisement adv;
        lock (_gate)
        {
            if (_advertisements.TryGetValue(topic, out var existing))
            {
                existing.Count++;
                return;
            }

            adv = new TopicAdvertisement(NextId("advertise", topic), topic, spec.Type.Name, spec.Qos) { Count = 1 };
            _advertisements[topic] = adv;
        }

        _connection.Send(adv.AdvertiseOp());
    }

    internal void Unadvertise(string topic)
    {
        TopicAdvertisement? adv = null;
        lock (_gate)
        {
            if (_advertisements.TryGetValue(topic, out var existing) && --existing.Count == 0)
            {
                _advertisements.Remove(topic);
                adv = existing;
            }
        }

        if (adv is not null)
        {
            _connection.Send(Op(w =>
            {
                w.WriteString("op", "unadvertise");
                w.WriteString("id", adv.Id);
                w.WriteString("topic", topic);
            }));
        }
    }

    internal void Publish(string topic, IRosMessage message)
    {
        _connection.Send(Op(w =>
        {
            w.WriteString("op", "publish");
            w.WriteString("topic", topic);
            w.WritePropertyName("msg");
            message.WriteJson(w);
        }));
    }

    internal LocalSubscriber Subscribe(string topic, RmwTopicSpec spec, Action<JsonElement, MessageInfo> deliver)
    {
        var local = new LocalSubscriber(deliver);
        TopicSubscription? created = null;
        lock (_gate)
        {
            if (!_subscriptions.TryGetValue(topic, out var sub))
            {
                created = sub = new TopicSubscription(NextId("subscribe", topic), topic, spec.Type.Name, spec.Qos);
                _subscriptions[topic] = sub;
            }

            sub.Add(local);
        }

        if (created is not null)
        {
            _connection.Send(created.SubscribeOp(_options));
        }

        return local;
    }

    internal void Unsubscribe(string topic, LocalSubscriber local)
    {
        TopicSubscription? removed = null;
        lock (_gate)
        {
            if (_subscriptions.TryGetValue(topic, out var sub) && sub.Remove(local) == 0)
            {
                _subscriptions.Remove(topic);
                removed = sub;
            }
        }

        if (removed is not null)
        {
            _connection.Send(Op(w =>
            {
                w.WriteString("op", "unsubscribe");
                w.WriteString("id", removed.Id);
                w.WriteString("topic", topic);
            }));
        }
    }

    // ------------------------------------------------------------------ services

    internal void AdvertiseService(ServiceAdvertisement adv)
    {
        lock (_gate)
        {
            if (_services.ContainsKey(adv.Service))
            {
                throw new InvalidOperationException($"Service '{adv.Service}' is already served by this context; rosbridge allows one provider per client.");
            }

            _services[adv.Service] = adv;
        }

        _connection.Send(adv.AdvertiseOp());
    }

    internal void UnadvertiseService(ServiceAdvertisement adv)
    {
        lock (_gate)
        {
            if (!_services.Remove(adv.Service))
            {
                return;
            }
        }

        _connection.Send(Op(w =>
        {
            w.WriteString("op", "unadvertise_service");
            w.WriteString("service", adv.Service);
        }));
    }

    internal void SendServiceResponse(string service, string callId, IRosMessage response)
    {
        _connection.Send(Op(w =>
        {
            w.WriteString("op", "service_response");
            w.WriteString("id", callId);
            w.WriteString("service", service);
            w.WriteBoolean("result", true);
            w.WritePropertyName("values");
            response.WriteJson(w);
        }));
    }

    /// <summary>After a reconnect, re-creates every advertisement and subscription on the new link.</summary>
    private void Resync()
    {
        List<byte[]> ops;
        lock (_gate)
        {
            ops = [
                .. _advertisements.Values.Select(a => a.AdvertiseOp()),
                .. _subscriptions.Values.Select(s => s.SubscribeOp(_options)),
                .. _services.Values.Select(s => s.AdvertiseOp()),
            ];
        }

        foreach (var op in ops)
        {
            _connection.Send(op);
        }

        _logger.LogInformation("Re-established {Count} rosbridge advertisements and subscriptions", ops.Count);
    }

    public async ValueTask DisposeAsync()
    {
        _cts.Cancel();
        if (_graphLoop is not null)
        {
            try
            {
                await _graphLoop.WaitAsync(TimeSpan.FromSeconds(2)).ConfigureAwait(false);
            }
            catch (Exception)
            {
            }
        }

        await _connection.DisposeAsync().ConfigureAwait(false);
        _cts.Dispose();
    }

    // ------------------------------------------------------------------ bookkeeping

    internal sealed class LocalSubscriber(Action<JsonElement, MessageInfo> deliver)
    {
        public Action<JsonElement, MessageInfo> Deliver { get; } = deliver;
    }

    private sealed class TopicSubscription(string id, string topic, string type, QosProfile qos)
    {
        private LocalSubscriber[] _locals = [];

        public string Id { get; } = id;

        public long Sequence;

        public LocalSubscriber[] Snapshot() => Volatile.Read(ref _locals);

        public void Add(LocalSubscriber s) => _locals = [.. _locals, s];

        public int Remove(LocalSubscriber s)
        {
            _locals = _locals.Where(x => x != s).ToArray();
            return _locals.Length;
        }

        public byte[] SubscribeOp(RosbridgeOptions options) => Op(w =>
        {
            w.WriteString("op", "subscribe");
            w.WriteString("id", Id);
            w.WriteString("topic", topic);
            w.WriteString("type", type);
            w.WriteNumber("queue_length", Math.Min(qos.Capacity, 1000));
            if (options.ThrottleRateMs > 0)
            {
                w.WriteNumber("throttle_rate", options.ThrottleRateMs);
            }

            if (options.FragmentSize > 0)
            {
                w.WriteNumber("fragment_size", options.FragmentSize);
            }
        });
    }

    private sealed class TopicAdvertisement(string id, string topic, string type, QosProfile qos)
    {
        public string Id { get; } = id;

        public int Count { get; set; }

        public byte[] AdvertiseOp() => Op(w =>
        {
            w.WriteString("op", "advertise");
            w.WriteString("id", Id);
            w.WriteString("topic", topic);
            w.WriteString("type", type);
            w.WriteBoolean("latch", qos.Resolve().Durability == DurabilityPolicy.TransientLocal);
            w.WriteNumber("queue_size", Math.Min(qos.Capacity, 1000));
        });
    }

    internal sealed class ServiceAdvertisement(string service, string type, Action<string, JsonElement> onCall)
    {
        public string Service { get; } = service;

        public void OnCall(string id, JsonElement args) => onCall(id, args);

        public byte[] AdvertiseOp() => Op(w =>
        {
            w.WriteString("op", "advertise_service");
            w.WriteString("service", Service);
            w.WriteString("type", type);
        });
    }
}
