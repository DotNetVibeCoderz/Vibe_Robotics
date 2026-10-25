using System.Buffers;
using System.Collections.Concurrent;
using System.Net;
using System.Net.Sockets;
using System.Net.WebSockets;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;

namespace ROS2.Net.Integration.Tests.Rosbridge;

/// <summary>
/// A minimal rosbridge_server stand-in: routes publish to subscribers (including the sender, as the
/// real server does), forwards call_service to the client that advertised the service, and answers
/// the rosapi topics/services/nodes queries from what clients advertised.
/// </summary>
internal sealed class FakeRosbridgeServer : IAsyncDisposable
{
    private readonly TcpListener _listener = new(IPAddress.Loopback, 0);
    private readonly CancellationTokenSource _cts = new();
    private readonly ConcurrentDictionary<Client, byte> _clients = new();
    private readonly ConcurrentDictionary<string, Client> _services = new(StringComparer.Ordinal);
    private readonly ConcurrentDictionary<string, string> _topics = new(StringComparer.Ordinal);
    private readonly ConcurrentDictionary<string, (Client Caller, string Id, string Service)> _pending = new(StringComparer.Ordinal);
    private long _forwardId;
    private Task? _accept;

    public FakeRosbridgeServer()
    {
        _listener.Start();
        Url = $"ws://127.0.0.1:{((IPEndPoint)_listener.LocalEndpoint).Port}";
        _accept = Task.Run(AcceptLoopAsync);
    }

    public string Url { get; }

    /// <summary>Every op received, for assertions.</summary>
    public ConcurrentQueue<string> Ops { get; } = new();

    public int ClientCount => _clients.Count;

    /// <summary>Drops every connection (to exercise client reconnection).</summary>
    public void DisconnectAll()
    {
        foreach (var c in _clients.Keys)
        {
            c.Socket.Abort();
        }
    }

    private async Task AcceptLoopAsync()
    {
        while (!_cts.IsCancellationRequested)
        {
            TcpClient tcp;
            try
            {
                tcp = await _listener.AcceptTcpClientAsync(_cts.Token);
            }
            catch (Exception)
            {
                return;
            }

            _ = Task.Run(() => ServeAsync(tcp));
        }
    }

    private async Task ServeAsync(TcpClient tcp)
    {
        var stream = tcp.GetStream();
        if (!await HandshakeAsync(stream))
        {
            tcp.Dispose();
            return;
        }

        var ws = WebSocket.CreateFromStream(stream, isServer: true, subProtocol: null, keepAliveInterval: TimeSpan.FromSeconds(30));
        var client = new Client(ws);
        _clients[client] = 0;
        var buffer = new byte[64 * 1024];
        var message = new ArrayBufferWriter<byte>();
        try
        {
            while (ws.State == WebSocketState.Open && !_cts.IsCancellationRequested)
            {
                message.ResetWrittenCount();
                ValueWebSocketReceiveResult r;
                do
                {
                    r = await ws.ReceiveAsync(buffer.AsMemory(), _cts.Token);
                    if (r.MessageType == WebSocketMessageType.Close)
                    {
                        return;
                    }

                    message.Write(buffer.AsSpan(0, r.Count));
                }
                while (!r.EndOfMessage);

                await HandleAsync(client, message.WrittenMemory);
            }
        }
        catch (Exception)
        {
        }
        finally
        {
            _clients.TryRemove(client, out _);
            foreach (var (svc, owner) in _services)
            {
                if (owner == client)
                {
                    _services.TryRemove(svc, out _);
                }
            }

            ws.Dispose();
            tcp.Dispose();
        }
    }

    private static async Task<bool> HandshakeAsync(NetworkStream stream)
    {
        var buffer = new byte[8192];
        var total = 0;
        while (true)
        {
            var n = await stream.ReadAsync(buffer.AsMemory(total));
            if (n == 0)
            {
                return false;
            }

            total += n;
            var text = Encoding.ASCII.GetString(buffer, 0, total);
            if (!text.Contains("\r\n\r\n", StringComparison.Ordinal))
            {
                continue;
            }

            var keyLine = text.Split("\r\n").FirstOrDefault(l => l.StartsWith("Sec-WebSocket-Key:", StringComparison.OrdinalIgnoreCase));
            if (keyLine is null)
            {
                return false;
            }

            var key = keyLine[(keyLine.IndexOf(':') + 1)..].Trim();
            var accept = Convert.ToBase64String(SHA1.HashData(Encoding.ASCII.GetBytes(key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11")));
            var response = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n" +
                           $"Sec-WebSocket-Accept: {accept}\r\n\r\n";
            await stream.WriteAsync(Encoding.ASCII.GetBytes(response));
            return true;
        }
    }

    private async Task HandleAsync(Client from, ReadOnlyMemory<byte> json)
    {
        using var doc = JsonDocument.Parse(json);
        var root = doc.RootElement;
        var op = root.GetProperty("op").GetString();
        Ops.Enqueue(op + ":" + (root.TryGetProperty("topic", out var tp) ? tp.GetString() : root.TryGetProperty("service", out var sp) ? sp.GetString() : string.Empty));
        switch (op)
        {
            case "advertise":
            case "subscribe":
                var topic = root.GetProperty("topic").GetString()!;
                _topics[topic] = root.TryGetProperty("type", out var ty) ? ty.GetString() ?? string.Empty : string.Empty;
                if (op == "subscribe")
                {
                    from.Topics[topic] = 0;
                }

                break;
            case "unsubscribe":
                from.Topics.TryRemove(root.GetProperty("topic").GetString()!, out _);
                break;
            case "publish":
                var t = root.GetProperty("topic").GetString()!;
                var payload = Encoding.UTF8.GetBytes($"{{\"op\":\"publish\",\"topic\":\"{t}\",\"msg\":{root.GetProperty("msg").GetRawText()}}}");
                foreach (var c in _clients.Keys.Where(c => c.Topics.ContainsKey(t)))
                {
                    await c.SendAsync(payload);
                }

                break;
            case "advertise_service":
                _services[root.GetProperty("service").GetString()!] = from;
                break;
            case "unadvertise_service":
                _services.TryRemove(root.GetProperty("service").GetString()!, out _);
                break;
            case "call_service":
                await CallServiceAsync(from, root);
                break;
            case "service_response":
                var id = root.GetProperty("id").GetString()!;
                if (_pending.TryRemove(id, out var p))
                {
                    var values = root.TryGetProperty("values", out var v) ? v.GetRawText() : "{}";
                    var result = !root.TryGetProperty("result", out var rr) || rr.ValueKind != JsonValueKind.False;
                    await p.Caller.SendAsync(Encoding.UTF8.GetBytes(
                        $"{{\"op\":\"service_response\",\"id\":{JsonSerializer.Serialize(p.Id)},\"service\":{JsonSerializer.Serialize(p.Service)},\"values\":{values},\"result\":{(result ? "true" : "false")}}}"));
                }

                break;
        }
    }

    private async Task CallServiceAsync(Client from, JsonElement root)
    {
        var service = root.GetProperty("service").GetString()!;
        var id = root.TryGetProperty("id", out var i) ? i.GetString() ?? string.Empty : string.Empty;
        string? values = service switch
        {
            "/rosapi/topics" => JsonSerializer.Serialize(new { topics = _topics.Keys.ToArray(), types = _topics.Values.ToArray() }),
            "/rosapi/services" => JsonSerializer.Serialize(new { services = _services.Keys.ToArray() }),
            "/rosapi/nodes" => JsonSerializer.Serialize(new { nodes = new[] { "/rosbridge_websocket" } }),
            "/rosapi/node_details" => JsonSerializer.Serialize(new
            {
                publishing = _topics.Keys.ToArray(),
                subscribing = Array.Empty<string>(),
                services = _services.Keys.ToArray(),
            }),
            _ => null,
        };

        if (values is not null)
        {
            await from.SendAsync(Encoding.UTF8.GetBytes($"{{\"op\":\"service_response\",\"id\":{JsonSerializer.Serialize(id)},\"service\":\"{service}\",\"values\":{values},\"result\":true}}"));
            return;
        }

        if (!_services.TryGetValue(service, out var provider))
        {
            await from.SendAsync(Encoding.UTF8.GetBytes($"{{\"op\":\"service_response\",\"id\":{JsonSerializer.Serialize(id)},\"service\":\"{service}\",\"values\":\"no provider\",\"result\":false}}"));
            return;
        }

        var forward = "fwd_" + Interlocked.Increment(ref _forwardId);
        _pending[forward] = (from, id, service);
        var args = root.TryGetProperty("args", out var a) ? a.GetRawText() : "{}";
        await provider.SendAsync(Encoding.UTF8.GetBytes($"{{\"op\":\"call_service\",\"id\":\"{forward}\",\"service\":\"{service}\",\"args\":{args}}}"));
    }

    public async ValueTask DisposeAsync()
    {
        _cts.Cancel();
        _listener.Stop();
        DisconnectAll();
        if (_accept is not null)
        {
            await _accept.WaitAsync(TimeSpan.FromSeconds(2)).ContinueWith(_ => { });
        }

        _cts.Dispose();
    }

    private sealed class Client(WebSocket socket)
    {
        private readonly SemaphoreSlim _send = new(1, 1);

        public WebSocket Socket { get; } = socket;

        public ConcurrentDictionary<string, byte> Topics { get; } = new(StringComparer.Ordinal);

        public async Task SendAsync(byte[] payload)
        {
            await _send.WaitAsync();
            try
            {
                if (Socket.State == WebSocketState.Open)
                {
                    await Socket.SendAsync(payload, WebSocketMessageType.Text, true, CancellationToken.None);
                }
            }
            catch (Exception)
            {
            }
            finally
            {
                _send.Release();
            }
        }
    }
}
