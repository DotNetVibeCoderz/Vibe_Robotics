using System.Buffers;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading.Channels;
using Microsoft.Extensions.Logging;

namespace ROS2.Net.Rmw.Rosbridge;

/// <summary>
/// One WebSocket link to rosbridge_server: a single sender loop (WebSocket sends must not overlap),
/// a receive loop that reassembles frames and <c>fragment</c> ops, and automatic reconnection.
/// </summary>
internal sealed class RosbridgeConnection : IAsyncDisposable
{
    private readonly RosbridgeOptions _options;
    private readonly ILogger _logger;
    private readonly Action<JsonElement> _onOp;
    private readonly Channel<byte[]> _outgoing = Channel.CreateBounded<byte[]>(new BoundedChannelOptions(8192)
    {
        FullMode = BoundedChannelFullMode.DropOldest,
        SingleReader = true,
    });

    private readonly CancellationTokenSource _cts = new();
    private readonly Dictionary<string, (string?[] Parts, int Received)> _fragments = new(StringComparer.Ordinal);
    private ClientWebSocket? _socket;
    private Task? _receiveLoop;
    private Task? _sendLoop;

    public RosbridgeConnection(RosbridgeOptions options, ILogger logger, Action<JsonElement> onOp)
    {
        _options = options;
        _logger = logger;
        _onOp = onOp;
    }

    public bool IsConnected => _socket?.State == WebSocketState.Open;

    /// <summary>Raised after a dropped link is re-established; subscribers re-send their state.</summary>
    public event Action? Reconnected;

    public async Task StartAsync(CancellationToken cancellationToken)
    {
        using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, _cts.Token);
        timeout.CancelAfter(_options.ConnectTimeout);
        try
        {
            _socket = await ConnectAsync(timeout.Token).ConfigureAwait(false);
        }
        catch (Exception ex) when (ex is WebSocketException or OperationCanceledException or HttpRequestException)
        {
            throw new InvalidOperationException($"Could not connect to rosbridge at {_options.Url}: {ex.Message}", ex);
        }

        _receiveLoop = Task.Run(ReceiveLoopAsync);
        _sendLoop = Task.Run(SendLoopAsync);
    }

    private async Task<ClientWebSocket> ConnectAsync(CancellationToken cancellationToken)
    {
        var socket = new ClientWebSocket();
        socket.Options.KeepAliveInterval = TimeSpan.FromSeconds(15);
        await socket.ConnectAsync(_options.Url, cancellationToken).ConfigureAwait(false);
        _logger.LogInformation("Connected to rosbridge at {Url}", _options.Url);
        return socket;
    }

    /// <summary>Queues an op. Ops are dropped (oldest first) only if 8192 are already waiting.</summary>
    public void Send(byte[] op) => _outgoing.Writer.TryWrite(op);

    private async Task SendLoopAsync()
    {
        var token = _cts.Token;
        try
        {
            await foreach (var op in _outgoing.Reader.ReadAllAsync(token).ConfigureAwait(false))
            {
                while (!IsConnected)
                {
                    await Task.Delay(20, token).ConfigureAwait(false);
                }

                try
                {
                    await _socket!.SendAsync(op, WebSocketMessageType.Text, true, token).ConfigureAwait(false);
                }
                catch (Exception ex) when (ex is WebSocketException or ObjectDisposedException or InvalidOperationException)
                {
                    _logger.LogDebug(ex, "rosbridge send failed; the op is dropped");
                }
            }
        }
        catch (OperationCanceledException)
        {
        }
    }

    private async Task ReceiveLoopAsync()
    {
        var token = _cts.Token;
        var buffer = ArrayPool<byte>.Shared.Rent(64 * 1024);
        var message = new ArrayBufferWriter<byte>(64 * 1024);
        try
        {
            while (!token.IsCancellationRequested)
            {
                try
                {
                    var socket = _socket!;
                    while (socket.State == WebSocketState.Open)
                    {
                        message.ResetWrittenCount();
                        ValueWebSocketReceiveResult result;
                        do
                        {
                            result = await socket.ReceiveAsync(buffer.AsMemory(), token).ConfigureAwait(false);
                            if (result.MessageType == WebSocketMessageType.Close)
                            {
                                throw new WebSocketException("rosbridge closed the connection.");
                            }

                            message.Write(buffer.AsSpan(0, result.Count));
                        }
                        while (!result.EndOfMessage);

                        Dispatch(message.WrittenMemory);
                    }
                }
                catch (OperationCanceledException) when (token.IsCancellationRequested)
                {
                    return;
                }
                catch (Exception ex) when (ex is WebSocketException or IOException or InvalidOperationException)
                {
                    _logger.LogWarning("rosbridge link lost: {Message}", ex.Message);
                }

                await ReconnectAsync(token).ConfigureAwait(false);
            }
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(buffer);
        }
    }

    private async Task ReconnectAsync(CancellationToken token)
    {
        _socket?.Dispose();
        while (!token.IsCancellationRequested)
        {
            try
            {
                await Task.Delay(_options.ReconnectDelay, token).ConfigureAwait(false);
                _socket = await ConnectAsync(token).ConfigureAwait(false);
                lock (_fragments)
                {
                    _fragments.Clear();
                }

                Reconnected?.Invoke();
                return;
            }
            catch (OperationCanceledException)
            {
                return;
            }
            catch (Exception ex) when (ex is WebSocketException or HttpRequestException or IOException)
            {
                _logger.LogDebug("rosbridge reconnect failed: {Message}", ex.Message);
            }
        }
    }

    private void Dispatch(ReadOnlyMemory<byte> json)
    {
        JsonDocument doc;
        try
        {
            doc = JsonDocument.Parse(json);
        }
        catch (JsonException ex)
        {
            _logger.LogWarning("Ignoring malformed rosbridge message: {Message}", ex.Message);
            return;
        }

        using (doc)
        {
            var root = doc.RootElement;
            if (root.ValueKind == JsonValueKind.Object && root.TryGetProperty("op", out var op) && op.GetString() == "fragment")
            {
                OnFragment(root);
                return;
            }

            try
            {
                _onOp(root);
            }
            catch (Exception ex) when (ex is not OutOfMemoryException)
            {
                _logger.LogError(ex, "Error handling rosbridge op");
            }
        }
    }

    private void OnFragment(JsonElement root)
    {
        var id = root.GetProperty("id").GetString() ?? string.Empty;
        var num = root.GetProperty("num").GetInt32();
        var total = root.GetProperty("total").GetInt32();
        var data = root.GetProperty("data").GetString() ?? string.Empty;
        string? complete = null;
        lock (_fragments)
        {
            if (!_fragments.TryGetValue(id, out var entry) || entry.Parts.Length != total)
            {
                entry = (new string?[total], 0);
            }

            if (num >= 0 && num < total && entry.Parts[num] is null)
            {
                entry.Parts[num] = data;
                entry.Received++;
            }

            if (entry.Received == total)
            {
                complete = string.Concat(entry.Parts);
                _fragments.Remove(id);
            }
            else
            {
                _fragments[id] = entry;
            }
        }

        if (complete is not null)
        {
            Dispatch(Encoding.UTF8.GetBytes(complete));
        }
    }

    public async ValueTask DisposeAsync()
    {
        _cts.Cancel();
        _outgoing.Writer.TryComplete();
        var socket = _socket;
        if (socket is { State: WebSocketState.Open })
        {
            try
            {
                using var close = new CancellationTokenSource(TimeSpan.FromSeconds(1));
                await socket.CloseOutputAsync(WebSocketCloseStatus.NormalClosure, "bye", close.Token).ConfigureAwait(false);
            }
            catch (Exception ex) when (ex is WebSocketException or OperationCanceledException or ObjectDisposedException)
            {
            }
        }

        foreach (var t in new[] { _receiveLoop, _sendLoop })
        {
            if (t is not null)
            {
                try
                {
                    await t.WaitAsync(TimeSpan.FromSeconds(2)).ConfigureAwait(false);
                }
                catch (Exception)
                {
                }
            }
        }

        socket?.Dispose();
        _cts.Dispose();
    }
}
