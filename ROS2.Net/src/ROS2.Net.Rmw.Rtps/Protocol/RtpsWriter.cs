using System.Buffers;
using System.Diagnostics;
using System.Net;
using ROS2.Net.Rmw.Rtps.Wire;

namespace ROS2.Net.Rmw.Rtps.Protocol;

/// <summary>Sends datagrams for protocol endpoints.</summary>
internal interface IRtpsSender
{
    GuidPrefix Prefix { get; }

    VendorId Vendor { get; }

    int MaxFragmentSize { get; }

    void Send(ReadOnlySpan<byte> datagram, IReadOnlyList<IPEndPoint> destinations);
}

/// <summary>Receives samples from a writer in the same participant without going through sockets.</summary>
internal interface ILocalReader
{
    RtpsGuid Guid { get; }

    void DeliverLocal(RtpsGuid writer, long sn, ReadOnlySpan<byte> payload, ReadOnlySpan<byte> inlineQos, long timestamp);
}

/// <summary>A sample in a writer's history.</summary>
internal sealed class CacheChange
{
    public long Sn;
    public byte[] Buffer = [];
    public int Length;
    public byte[] InlineQos = [];
    public long Timestamp;
    public RtpsGuid? Key;

    public ReadOnlySpan<byte> Payload => Buffer.AsSpan(0, Length);

    public void Release()
    {
        if (Buffer.Length > 0)
        {
            ArrayPool<byte>.Shared.Return(Buffer);
            Buffer = [];
        }
    }
}

/// <summary>A writer's view of a matched reader.</summary>
internal sealed class ReaderProxy
{
    public required RtpsGuid Guid { get; init; }

    public List<IPEndPoint> Endpoints { get; set; } = [];

    public bool Reliable { get; init; }

    public bool TransientLocal { get; init; }

    public ILocalReader? Local { get; init; }

    /// <summary>Highest sequence number the reader acknowledged (everything up to it is received or irrelevant).</summary>
    public long Acked;

    /// <summary>Samples up to this number predate the match and are irrelevant to a volatile reader.</summary>
    public long StartSn;

    public int LastAckNackCount = int.MinValue;
    public int LastNackFragCount = int.MinValue;
    public long NextHeartbeat;
    public int HeartbeatBackoff = 1;
}

/// <summary>
/// An RTPS stateful writer: history cache, matched reader proxies, heartbeats and repair of lost
/// samples (ACKNACK → resend or GAP, NACK_FRAG → fragment resend). Used for user and SEDP endpoints.
/// </summary>
internal sealed class RtpsWriter : IDisposable
{
    private readonly object _gate = new();
    private readonly IRtpsSender _sender;
    private readonly RtpsMessageBuilder _builder;
    private readonly SortedDictionary<long, CacheChange> _history = [];
    private readonly Dictionary<RtpsGuid, ReaderProxy> _readers = [];
    private readonly int _depth;
    private readonly TimeSpan _heartbeatPeriod;
    private long _lastSn;
    private int _heartbeatCount;
    private bool _disposed;

    public RtpsWriter(IRtpsSender sender, RtpsGuid guid, bool reliable, bool transientLocal, int depth, bool keyed, TimeSpan heartbeatPeriod)
    {
        _sender = sender;
        Guid = guid;
        Reliable = reliable;
        TransientLocal = transientLocal;
        _depth = Math.Max(1, depth);
        Keyed = keyed;
        _heartbeatPeriod = heartbeatPeriod;
        _builder = new RtpsMessageBuilder(sender.Prefix, sender.Vendor);
    }

    public RtpsGuid Guid { get; }

    public bool Reliable { get; }

    public bool TransientLocal { get; }

    /// <summary>Keyed history keeps only the latest sample per key (SEDP).</summary>
    public bool Keyed { get; }

    public long LastSequenceNumber
    {
        get
        {
            lock (_gate)
            {
                return _lastSn;
            }
        }
    }

    public int MatchedCount
    {
        get
        {
            lock (_gate)
            {
                return _readers.Count;
            }
        }
    }

    public int HistoryCount
    {
        get
        {
            lock (_gate)
            {
                return _history.Count;
            }
        }
    }

    public bool HasReader(RtpsGuid guid)
    {
        lock (_gate)
        {
            return _readers.ContainsKey(guid);
        }
    }

    /// <summary>True when every reliable remote reader acknowledged everything.</summary>
    public bool IsFullyAcknowledged
    {
        get
        {
            lock (_gate)
            {
                return _readers.Values.All(r => r.Local is not null || !r.Reliable || r.Acked >= _lastSn);
            }
        }
    }

    /// <summary>Writes a sample and returns its sequence number.</summary>
    public long Write(ReadOnlySpan<byte> payload, ReadOnlySpan<byte> inlineQos = default, RtpsGuid? key = null)
    {
        var ts = RosTime.Now().Nanoseconds;
        List<ILocalReader>? locals = null;
        long sn;
        lock (_gate)
        {
            ObjectDisposedException.ThrowIf(_disposed, this);
            sn = ++_lastSn;
            var store = TransientLocal || Keyed || _readers.Values.Any(r => r.Reliable && r.Local is null);
            CacheChange? change = null;
            if (store)
            {
                change = new CacheChange { Sn = sn, Length = payload.Length, InlineQos = inlineQos.ToArray(), Timestamp = ts, Key = key };
                if (payload.Length > 0)
                {
                    change.Buffer = ArrayPool<byte>.Shared.Rent(payload.Length);
                    payload.CopyTo(change.Buffer);
                }

                if (Keyed && key is { } k)
                {
                    foreach (var old in _history.Values.Where(c => c.Key == k).ToList())
                    {
                        _history.Remove(old.Sn);
                        old.Release();
                    }
                }

                _history[sn] = change;
                while (!Keyed && _history.Count > _depth)
                {
                    var first = _history.First();
                    _history.Remove(first.Key);
                    first.Value.Release();
                }
            }

            var remote = new List<ReaderProxy>();
            foreach (var r in _readers.Values)
            {
                if (r.Local is not null)
                {
                    (locals ??= []).Add(r.Local);
                }
                else
                {
                    remote.Add(r);
                    if (r.Reliable)
                    {
                        // New data: heartbeat soon again in case the sample (or its piggybacked heartbeat) is lost.
                        r.HeartbeatBackoff = 1;
                        r.NextHeartbeat = Stopwatch.GetTimestamp() + (long)(_heartbeatPeriod.TotalSeconds * Stopwatch.Frequency);
                    }
                }
            }

            if (remote.Count > 0)
            {
                SendSample(sn, payload, inlineQos, ts, remote, piggybackHeartbeat: Reliable);
            }
        }

        if (locals is not null)
        {
            foreach (var l in locals)
            {
                l.DeliverLocal(Guid, sn, payload, inlineQos, ts);
            }
        }

        return sn;
    }

    /// <summary>Matches a reader. Transient-local writers push their history to it immediately.</summary>
    public void AddReader(ReaderProxy proxy)
    {
        List<CacheChange>? replay = null;
        lock (_gate)
        {
            if (_disposed)
            {
                return;
            }

            proxy.StartSn = TransientLocal && proxy.TransientLocal ? 0 : _lastSn;
            proxy.Acked = proxy.StartSn;
            proxy.NextHeartbeat = 0;
            _readers[proxy.Guid] = proxy;
            if (TransientLocal && proxy.TransientLocal && _history.Count > 0)
            {
                if (proxy.Local is not null)
                {
                    replay = _history.Values.ToList();
                }
                else
                {
                    foreach (var c in _history.Values)
                    {
                        SendSample(c.Sn, c.Payload, c.InlineQos, c.Timestamp, [proxy], piggybackHeartbeat: false);
                    }

                    if (proxy.Reliable)
                    {
                        SendHeartbeat(proxy, final: false);
                    }
                }
            }
            else if (proxy.Reliable && proxy.Local is null)
            {
                SendHeartbeat(proxy, final: false);
            }
        }

        if (replay is not null)
        {
            foreach (var c in replay)
            {
                proxy.Local!.DeliverLocal(Guid, c.Sn, c.Payload, c.InlineQos, c.Timestamp);
            }
        }
    }

    public bool RemoveReader(RtpsGuid guid)
    {
        lock (_gate)
        {
            return _readers.Remove(guid);
        }
    }

    public int RemoveReadersOf(GuidPrefix prefix)
    {
        lock (_gate)
        {
            var gone = _readers.Keys.Where(k => k.Prefix == prefix).ToList();
            foreach (var g in gone)
            {
                _readers.Remove(g);
            }

            return gone.Count;
        }
    }

    public void UpdateReaderEndpoints(GuidPrefix prefix, List<IPEndPoint> endpoints)
    {
        lock (_gate)
        {
            foreach (var r in _readers.Values.Where(r => r.Guid.Prefix == prefix && r.Endpoints.Count == 0))
            {
                r.Endpoints = endpoints;
            }
        }
    }

    public void OnAckNack(RtpsGuid reader, AckNackSubmessage ack)
    {
        lock (_gate)
        {
            if (!_readers.TryGetValue(reader, out var proxy) || ack.Count <= proxy.LastAckNackCount)
            {
                return;
            }

            proxy.LastAckNackCount = ack.Count;
            proxy.Acked = Math.Max(proxy.Acked, ack.State.Base - 1);
            var gaps = new List<long>();
            var resent = false;
            foreach (var sn in ack.State.Members())
            {
                if (sn > _lastSn)
                {
                    break;
                }

                if (sn <= proxy.StartSn || !_history.TryGetValue(sn, out var c))
                {
                    gaps.Add(sn);
                }
                else
                {
                    SendSample(c.Sn, c.Payload, c.InlineQos, c.Timestamp, [proxy], piggybackHeartbeat: false);
                    resent = true;
                }
            }

            if (gaps.Count > 0)
            {
                SendGaps(proxy, gaps);
            }

            if (resent || gaps.Count > 0)
            {
                SendHeartbeat(proxy, final: true);
            }

            if (proxy.Acked >= _lastSn)
            {
                proxy.HeartbeatBackoff = 1;
            }
        }
    }

    public void OnNackFrag(RtpsGuid reader, NackFragSubmessage nack)
    {
        lock (_gate)
        {
            if (!_readers.TryGetValue(reader, out var proxy) || nack.Count <= proxy.LastNackFragCount)
            {
                return;
            }

            proxy.LastNackFragCount = nack.Count;
            if (!_history.TryGetValue(nack.SequenceNumber, out var c))
            {
                SendGaps(proxy, [nack.SequenceNumber]);
                return;
            }

            var fragSize = _sender.MaxFragmentSize;
            foreach (var f in nack.State.Members())
            {
                var offset = (long)(f - 1) * fragSize;
                if (f == 0 || offset >= c.Length)
                {
                    continue;
                }

                var len = (int)Math.Min(fragSize, c.Length - offset);
                _builder.Begin().InfoDst(proxy.Guid.Prefix).InfoTs(c.Timestamp)
                    .DataFrag(proxy.Guid.EntityId, Guid.EntityId, c.Sn, f, (ushort)fragSize, (uint)c.Length, f == 1 ? c.InlineQos : default, c.Payload.Slice((int)offset, len));
                _sender.Send(_builder.Buffer.Span, proxy.Endpoints);
            }
        }
    }

    /// <summary>Periodic work: heartbeats to reliable readers with unacknowledged samples (with backoff).</summary>
    public void Tick(long nowTicks)
    {
        lock (_gate)
        {
            if (_disposed || !Reliable)
            {
                return;
            }

            foreach (var r in _readers.Values)
            {
                if (r.Local is not null || !r.Reliable || r.Acked >= _lastSn || nowTicks < r.NextHeartbeat)
                {
                    continue;
                }

                SendHeartbeat(r, final: false);
                r.NextHeartbeat = nowTicks + (long)(_heartbeatPeriod.TotalSeconds * Stopwatch.Frequency * r.HeartbeatBackoff);
                r.HeartbeatBackoff = Math.Min(r.HeartbeatBackoff * 2, 32);
            }
        }
    }

    private void SendSample(long sn, ReadOnlySpan<byte> payload, ReadOnlySpan<byte> inlineQos, long ts, List<ReaderProxy> proxies, bool piggybackHeartbeat)
    {
        foreach (var group in proxies.GroupBy(p => p.Guid.Prefix))
        {
            var list = group.ToList();
            var endpoints = list.SelectMany(p => p.Endpoints).Distinct().ToList();
            if (endpoints.Count == 0)
            {
                continue;
            }

            var readerId = list.Count == 1 ? list[0].Guid.EntityId : EntityId.Unknown;
            var maxFrag = _sender.MaxFragmentSize;
            if (payload.Length <= maxFrag)
            {
                _builder.Begin().InfoDst(group.Key).InfoTs(ts).Data(readerId, Guid.EntityId, sn, inlineQos, payload);
                if (piggybackHeartbeat)
                {
                    foreach (var p in list.Where(p => p.Reliable))
                    {
                        AppendHeartbeat(p, final: true);
                    }
                }

                _sender.Send(_builder.Buffer.Span, endpoints);
                continue;
            }

            var fragments = (payload.Length + maxFrag - 1) / maxFrag;
            for (var i = 0; i < fragments; i++)
            {
                var offset = i * maxFrag;
                var len = Math.Min(maxFrag, payload.Length - offset);
                _builder.Begin().InfoDst(group.Key).InfoTs(ts)
                    .DataFrag(readerId, Guid.EntityId, sn, (uint)(i + 1), (ushort)maxFrag, (uint)payload.Length, i == 0 ? inlineQos : default, payload.Slice(offset, len));
                _sender.Send(_builder.Buffer.Span, endpoints);
            }

            if (piggybackHeartbeat)
            {
                _builder.Begin().InfoDst(group.Key);
                foreach (var p in list.Where(p => p.Reliable))
                {
                    AppendHeartbeat(p, final: true);
                }

                _sender.Send(_builder.Buffer.Span, endpoints);
            }
        }
    }

    private void SendHeartbeat(ReaderProxy proxy, bool final)
    {
        if (proxy.Endpoints.Count == 0)
        {
            return;
        }

        _builder.Begin().InfoDst(proxy.Guid.Prefix);
        AppendHeartbeat(proxy, final);
        _sender.Send(_builder.Buffer.Span, proxy.Endpoints);
    }

    private void AppendHeartbeat(ReaderProxy proxy, bool final)
    {
        var first = _history.Count > 0 ? _history.Keys.First() : _lastSn + 1;
        first = Math.Max(first, proxy.StartSn + 1);
        var last = _lastSn;
        if (first > last + 1)
        {
            first = last + 1;
        }

        _builder.Heartbeat(proxy.Guid.EntityId, Guid.EntityId, Math.Max(1, first), last, ++_heartbeatCount, final);
    }

    private void SendGaps(ReaderProxy proxy, List<long> gaps)
    {
        if (proxy.Endpoints.Count == 0)
        {
            return;
        }

        _builder.Begin().InfoDst(proxy.Guid.Prefix);
        var i = 0;
        while (i < gaps.Count)
        {
            var start = gaps[i];
            var end = start;
            while (i + 1 < gaps.Count && gaps[i + 1] == end + 1)
            {
                i++;
                end++;
            }

            _builder.Gap(proxy.Guid.EntityId, Guid.EntityId, start, new SequenceNumberSet(end + 1, 0));
            i++;
        }

        _sender.Send(_builder.Buffer.Span, proxy.Endpoints);
    }

    public void Dispose()
    {
        lock (_gate)
        {
            if (_disposed)
            {
                return;
            }

            _disposed = true;
            foreach (var c in _history.Values)
            {
                c.Release();
            }

            _history.Clear();
            _readers.Clear();
            _builder.Dispose();
        }
    }
}
