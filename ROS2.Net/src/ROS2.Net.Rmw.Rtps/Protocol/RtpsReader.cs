using System.Net;
using ROS2.Net.Rmw.Rtps.Wire;

namespace ROS2.Net.Rmw.Rtps.Protocol;

/// <summary>A sample handed to a reader's consumer. Spans are only valid during the callback.</summary>
internal readonly ref struct RtpsSample
{
    public RtpsGuid Writer { get; init; }

    public long SequenceNumber { get; init; }

    public ReadOnlySpan<byte> Payload { get; init; }

    public ReadOnlySpan<byte> InlineQos { get; init; }

    public bool InlineQosLittleEndian { get; init; }

    public long Timestamp { get; init; }

    public bool FromLocalWriter { get; init; }
}

internal delegate void RtpsSampleHandler(in RtpsSample sample);

/// <summary>A reader's view of a matched writer.</summary>
internal sealed class WriterProxy
{
    public required RtpsGuid Guid { get; init; }

    public List<IPEndPoint> Endpoints { get; set; } = [];

    /// <summary>Reliable when both the reader and the writer are reliable.</summary>
    public bool Reliable { get; init; }

    public bool IsLocal { get; init; }

    /// <summary>Next sequence number to deliver.</summary>
    public long Expected = 1;

    public readonly SortedDictionary<long, PendingSample?> Pending = [];
    public readonly Dictionary<long, FragmentAssembly> Fragments = [];
    public int LastHeartbeatCount = int.MinValue;
    public int AckNackCount;
    public int NackFragCount;
    public long HighestAnnounced;
}

internal sealed class PendingSample
{
    public required byte[] Payload { get; init; }

    public required byte[] InlineQos { get; init; }

    public bool LittleEndian { get; init; }

    public long Timestamp { get; init; }
}

internal sealed class FragmentAssembly
{
    public FragmentAssembly(uint sampleSize, ushort fragmentSize)
    {
        Data = new byte[sampleSize];
        FragmentSize = fragmentSize;
        Received = new bool[(sampleSize + fragmentSize - 1) / fragmentSize];
    }

    public byte[] Data { get; }

    public ushort FragmentSize { get; }

    public bool[] Received { get; }

    public int ReceivedCount;
    public byte[] InlineQos = [];
    public bool LittleEndian;
    public long Timestamp;

    public bool Complete => ReceivedCount == Received.Length;
}

/// <summary>
/// An RTPS stateful reader: per-writer in-order delivery for reliable writers (buffering, ACKNACK
/// and NACK_FRAG requests, GAP handling), latest-wins delivery for best-effort writers, and
/// DATA_FRAG reassembly.
/// </summary>
internal sealed class RtpsReader : ILocalReader, IDisposable
{
    private const int MaxPending = 10_000;
    private const long MaxSampleSize = 256L * 1024 * 1024;
    private readonly object _gate = new();
    private readonly IRtpsSender _sender;
    private readonly RtpsMessageBuilder _builder;
    private readonly Dictionary<RtpsGuid, WriterProxy> _writers = [];
    private readonly RtpsSampleHandler _handler;
    private bool _disposed;

    public RtpsReader(IRtpsSender sender, RtpsGuid guid, bool reliable, RtpsSampleHandler handler)
    {
        _sender = sender;
        Guid = guid;
        Reliable = reliable;
        _handler = handler;
        _builder = new RtpsMessageBuilder(sender.Prefix, sender.Vendor);
    }

    public RtpsGuid Guid { get; }

    public bool Reliable { get; }

    /// <summary>Accept samples from writers that were never matched (SPDP).</summary>
    public bool AcceptUnknownWriters { get; init; }

    /// <summary>Raised (under no lock guarantee) when samples were skipped because the writer no longer had them.</summary>
    public event Action<int>? SamplesLost;

    public int MatchedCount
    {
        get
        {
            lock (_gate)
            {
                return _writers.Count;
            }
        }
    }

    public bool HasWriter(RtpsGuid guid)
    {
        lock (_gate)
        {
            return _writers.ContainsKey(guid);
        }
    }

    public void AddWriter(WriterProxy proxy)
    {
        lock (_gate)
        {
            if (_disposed || _writers.ContainsKey(proxy.Guid))
            {
                return;
            }

            _writers[proxy.Guid] = proxy;
            if (proxy.Reliable && !proxy.IsLocal)
            {
                // Preemptive ACKNACK: tells the writer we exist and want everything from 1.
                SendAckNack(proxy, new SequenceNumberSet(proxy.Expected, 0), final: false);
            }
        }
    }

    public bool RemoveWriter(RtpsGuid guid)
    {
        lock (_gate)
        {
            return _writers.Remove(guid);
        }
    }

    public int RemoveWritersOf(GuidPrefix prefix)
    {
        lock (_gate)
        {
            var gone = _writers.Keys.Where(k => k.Prefix == prefix).ToList();
            foreach (var g in gone)
            {
                _writers.Remove(g);
            }

            return gone.Count;
        }
    }

    public void UpdateWriterEndpoints(GuidPrefix prefix, List<IPEndPoint> endpoints)
    {
        lock (_gate)
        {
            foreach (var w in _writers.Values.Where(w => w.Guid.Prefix == prefix && w.Endpoints.Count == 0))
            {
                w.Endpoints = endpoints;
            }
        }
    }

    public void DeliverLocal(RtpsGuid writer, long sn, ReadOnlySpan<byte> payload, ReadOnlySpan<byte> inlineQos, long timestamp)
    {
        lock (_gate)
        {
            if (_disposed || !_writers.TryGetValue(writer, out var proxy) || sn < proxy.Expected)
            {
                return;
            }

            proxy.Expected = sn + 1;
            Deliver(writer, sn, payload, inlineQos, true, timestamp, local: true);
        }
    }

    public void OnData(in ReceiveState state, in DataSubmessage data)
    {
        var writer = new RtpsGuid(state.SourcePrefix, data.WriterId);
        var ts = state.Timestamp ?? RosTime.Now().Nanoseconds;
        lock (_gate)
        {
            if (_disposed)
            {
                return;
            }

            if (!_writers.TryGetValue(writer, out var proxy))
            {
                if (AcceptUnknownWriters)
                {
                    Deliver(writer, data.SequenceNumber, data.Payload, data.InlineQos, data.LittleEndian, ts, local: false);
                }

                return;
            }

            Accept(proxy, data.SequenceNumber, data.Payload, data.InlineQos, data.LittleEndian, ts);
        }
    }

    public void OnDataFrag(in ReceiveState state, in DataFragSubmessage frag)
    {
        var writer = new RtpsGuid(state.SourcePrefix, frag.WriterId);
        lock (_gate)
        {
            if (_disposed || !_writers.TryGetValue(writer, out var proxy) || frag.FragmentSize == 0 || frag.SampleSize > MaxSampleSize)
            {
                return;
            }

            var sn = frag.SequenceNumber;
            if ((proxy.Reliable && (sn < proxy.Expected || proxy.Pending.ContainsKey(sn))) || (!proxy.Reliable && sn < proxy.Expected))
            {
                return;
            }

            if (!proxy.Fragments.TryGetValue(sn, out var asm))
            {
                if (proxy.Fragments.Count >= 32)
                {
                    proxy.Fragments.Remove(proxy.Fragments.Keys.Min());
                }

                asm = new FragmentAssembly(frag.SampleSize, frag.FragmentSize) { Timestamp = state.Timestamp ?? RosTime.Now().Nanoseconds };
                proxy.Fragments[sn] = asm;
            }

            if (!frag.InlineQos.IsEmpty)
            {
                asm.InlineQos = frag.InlineQos.ToArray();
                asm.LittleEndian = frag.LittleEndian;
            }

            var data = frag.Fragments;
            for (var i = 0; i < frag.FragmentsInSubmessage; i++)
            {
                var number = frag.FragmentStart + (uint)i;
                var index = (int)number - 1;
                if (index < 0 || index >= asm.Received.Length)
                {
                    break;
                }

                var offset = (long)index * asm.FragmentSize;
                var len = (int)Math.Min(asm.FragmentSize, asm.Data.Length - offset);
                if (data.Length < len)
                {
                    break;
                }

                if (!asm.Received[index])
                {
                    data[..len].CopyTo(asm.Data.AsSpan((int)offset));
                    asm.Received[index] = true;
                    asm.ReceivedCount++;
                }

                data = data[len..];
            }

            if (asm.Complete)
            {
                proxy.Fragments.Remove(sn);
                Accept(proxy, sn, asm.Data, asm.InlineQos, asm.LittleEndian, asm.Timestamp);
            }
        }
    }

    public void OnHeartbeat(in ReceiveState state, HeartbeatSubmessage hb)
    {
        var writer = new RtpsGuid(state.SourcePrefix, hb.WriterId);
        var lost = 0;
        lock (_gate)
        {
            if (_disposed || !_writers.TryGetValue(writer, out var proxy) || !proxy.Reliable || hb.Count <= proxy.LastHeartbeatCount)
            {
                return;
            }

            proxy.LastHeartbeatCount = hb.Count;
            proxy.HighestAnnounced = Math.Max(proxy.HighestAnnounced, hb.Last);
            if (hb.First > proxy.Expected)
            {
                // The writer no longer has older samples: skip them.
                foreach (var k in proxy.Pending.Keys.Where(k => k < hb.First).ToList())
                {
                    proxy.Pending.Remove(k);
                }

                lost = (int)Math.Min(int.MaxValue, hb.First - proxy.Expected);
                proxy.Expected = hb.First;
                Drain(proxy);
            }

            var missing = new SequenceNumberSet(proxy.Expected, (int)Math.Clamp(hb.Last - proxy.Expected + 1, 0, 256));
            var fragmented = new List<long>();
            for (var sn = proxy.Expected; sn <= hb.Last && sn < proxy.Expected + 256; sn++)
            {
                if (proxy.Pending.ContainsKey(sn))
                {
                    continue;
                }

                if (proxy.Fragments.ContainsKey(sn))
                {
                    fragmented.Add(sn);
                    continue;
                }

                missing.Add(sn);
            }

            var anyMissing = missing.Members().Any();
            if (!hb.Final || anyMissing)
            {
                SendAckNack(proxy, missing, final: !anyMissing);
            }

            foreach (var sn in fragmented)
            {
                var asm = proxy.Fragments[sn];
                var first = Array.IndexOf(asm.Received, false);
                if (first < 0)
                {
                    continue;
                }

                var set = new FragmentNumberSet((uint)first + 1, Math.Min(256, asm.Received.Length - first));
                for (var i = first; i < asm.Received.Length && i < first + 256; i++)
                {
                    if (!asm.Received[i])
                    {
                        set.Add((uint)i + 1);
                    }
                }

                _builder.Begin().InfoDst(proxy.Guid.Prefix).NackFrag(Guid.EntityId, proxy.Guid.EntityId, sn, set, ++proxy.NackFragCount);
                _sender.Send(_builder.Buffer.Span, proxy.Endpoints);
            }
        }

        if (lost > 0)
        {
            SamplesLost?.Invoke(lost);
        }
    }

    public void OnGap(in ReceiveState state, GapSubmessage gap)
    {
        var writer = new RtpsGuid(state.SourcePrefix, gap.WriterId);
        lock (_gate)
        {
            if (_disposed || !_writers.TryGetValue(writer, out var proxy) || !proxy.Reliable)
            {
                return;
            }

            var rangeEnd = gap.GapList.Base - 1;
            if (gap.GapStart <= proxy.Expected && rangeEnd >= proxy.Expected)
            {
                foreach (var k in proxy.Pending.Keys.Where(k => k <= rangeEnd).ToList())
                {
                    proxy.Pending.Remove(k);
                }

                proxy.Expected = rangeEnd + 1;
            }
            else
            {
                for (var sn = Math.Max(gap.GapStart, proxy.Expected); sn <= rangeEnd && proxy.Pending.Count < MaxPending; sn++)
                {
                    proxy.Pending.TryAdd(sn, null);
                }
            }

            foreach (var sn in gap.GapList.Members())
            {
                if (sn >= proxy.Expected)
                {
                    proxy.Pending.TryAdd(sn, null);
                }
            }

            Drain(proxy);
        }
    }

    private void Accept(WriterProxy proxy, long sn, ReadOnlySpan<byte> payload, ReadOnlySpan<byte> inlineQos, bool little, long ts)
    {
        if (!proxy.Reliable)
        {
            if (sn < proxy.Expected)
            {
                return;
            }

            proxy.Expected = sn + 1;
            Deliver(proxy.Guid, sn, payload, inlineQos, little, ts, proxy.IsLocal);
            return;
        }

        if (sn < proxy.Expected || proxy.Pending.ContainsKey(sn))
        {
            return;
        }

        if (sn == proxy.Expected)
        {
            proxy.Expected++;
            Deliver(proxy.Guid, sn, payload, inlineQos, little, ts, proxy.IsLocal);
            Drain(proxy);
            return;
        }

        if (proxy.Pending.Count < MaxPending)
        {
            proxy.Pending[sn] = new PendingSample { Payload = payload.ToArray(), InlineQos = inlineQos.ToArray(), LittleEndian = little, Timestamp = ts };
        }
    }

    private void Drain(WriterProxy proxy)
    {
        while (proxy.Pending.Remove(proxy.Expected, out var p))
        {
            var sn = proxy.Expected++;
            if (p is not null)
            {
                Deliver(proxy.Guid, sn, p.Payload, p.InlineQos, p.LittleEndian, p.Timestamp, proxy.IsLocal);
            }
        }
    }

    private void Deliver(RtpsGuid writer, long sn, ReadOnlySpan<byte> payload, ReadOnlySpan<byte> inlineQos, bool little, long ts, bool local)
    {
        _handler(new RtpsSample
        {
            Writer = writer,
            SequenceNumber = sn,
            Payload = payload,
            InlineQos = inlineQos,
            InlineQosLittleEndian = little,
            Timestamp = ts,
            FromLocalWriter = local,
        });
    }

    private void SendAckNack(WriterProxy proxy, SequenceNumberSet set, bool final)
    {
        if (proxy.Endpoints.Count == 0)
        {
            return;
        }

        _builder.Begin().InfoDst(proxy.Guid.Prefix).AckNack(Guid.EntityId, proxy.Guid.EntityId, set, ++proxy.AckNackCount, final);
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
            _writers.Clear();
            _builder.Dispose();
        }
    }
}
