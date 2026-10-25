using System.Buffers.Binary;

namespace ROS2.Net.Rmw.Rtps.Wire;

/// <summary>RTPS submessage ids.</summary>
public static class SubmessageId
{
    public const byte Pad = 0x01;
    public const byte AckNack = 0x06;
    public const byte Heartbeat = 0x07;
    public const byte Gap = 0x08;
    public const byte InfoTs = 0x09;
    public const byte InfoSrc = 0x0c;
    public const byte InfoReplyIp4 = 0x0d;
    public const byte InfoDst = 0x0e;
    public const byte InfoReply = 0x0f;
    public const byte NackFrag = 0x12;
    public const byte HeartbeatFrag = 0x13;
    public const byte Data = 0x15;
    public const byte DataFrag = 0x16;
}

/// <summary>A set of fragment numbers (<c>FragmentNumberSet</c>): 1-based base + up to 256 bits.</summary>
public sealed class FragmentNumberSet
{
    public FragmentNumberSet(uint bitmapBase, int numBits)
    {
        Base = bitmapBase;
        NumBits = Math.Clamp(numBits, 0, 256);
        Bitmap = new uint[(NumBits + 31) / 32];
    }

    public uint Base { get; }

    public int NumBits { get; }

    public uint[] Bitmap { get; }

    public void Add(uint fragment)
    {
        var i = (int)(fragment - Base);
        if (i >= 0 && i < NumBits)
        {
            Bitmap[i / 32] |= 1u << (31 - (i % 32));
        }
    }

    public IEnumerable<uint> Members()
    {
        for (var i = 0; i < NumBits; i++)
        {
            if ((Bitmap[i / 32] & (1u << (31 - (i % 32)))) != 0)
            {
                yield return Base + (uint)i;
            }
        }
    }
}

/// <summary>Builds RTPS messages (header + submessages), little-endian.</summary>
public sealed class RtpsMessageBuilder : IDisposable
{
    public const byte FlagEndianness = 0x01;
    private int _submessageStart;

    public RtpsMessageBuilder(GuidPrefix source, VendorId vendor, int capacity = 1500)
    {
        Source = source;
        Vendor = vendor;
        Buffer = new RtpsBuffer(capacity);
    }

    public GuidPrefix Source { get; }

    public VendorId Vendor { get; }

    public RtpsBuffer Buffer { get; }

    public ReadOnlyMemory<byte> Memory => Buffer.Memory;

    public int Length => Buffer.Position;

    /// <summary>Starts a new message: "RTPS", version 2.3, vendor id, source prefix.</summary>
    public RtpsMessageBuilder Begin()
    {
        Buffer.Reset();
        Buffer.WriteBytes("RTPS"u8);
        Buffer.WriteByte(2);
        Buffer.WriteByte(3);
        Buffer.WriteByte(Vendor.Major);
        Buffer.WriteByte(Vendor.Minor);
        Buffer.WriteGuidPrefix(Source);
        return this;
    }

    private void Start(byte id, byte flags)
    {
        Buffer.WriteByte(id);
        Buffer.WriteByte((byte)(flags | FlagEndianness));
        Buffer.WriteUInt16(0);
        _submessageStart = Buffer.Position;
    }

    private void Finish()
    {
        Buffer.Align4();
        Buffer.PatchUInt16(_submessageStart - 2, (ushort)(Buffer.Position - _submessageStart));
    }

    public RtpsMessageBuilder InfoDst(GuidPrefix destination)
    {
        Start(SubmessageId.InfoDst, 0);
        Buffer.WriteGuidPrefix(destination);
        Finish();
        return this;
    }

    /// <summary>INFO_TS with a source timestamp in Unix nanoseconds.</summary>
    public RtpsMessageBuilder InfoTs(long unixNanoseconds)
    {
        Start(SubmessageId.InfoTs, 0);
        var (s, f) = RtpsTime.FromNanoseconds(unixNanoseconds);
        Buffer.WriteInt32(s);
        Buffer.WriteUInt32(f);
        Finish();
        return this;
    }

    /// <summary>DATA. <paramref name="inlineQos"/> is a complete parameter list (sentinel included) or empty.</summary>
    public RtpsMessageBuilder Data(EntityId reader, EntityId writer, long sn, ReadOnlySpan<byte> inlineQos, ReadOnlySpan<byte> payload)
    {
        byte flags = 0;
        if (!inlineQos.IsEmpty)
        {
            flags |= 0x02;
        }

        if (!payload.IsEmpty)
        {
            flags |= 0x04;
        }

        Start(SubmessageId.Data, flags);
        Buffer.WriteUInt16(0); // extraFlags
        Buffer.WriteUInt16(16); // octetsToInlineQos
        Buffer.WriteEntityId(reader);
        Buffer.WriteEntityId(writer);
        Buffer.WriteSequenceNumber(sn);
        Buffer.WriteBytes(inlineQos);
        WritePayload(payload);
        Finish();
        return this;
    }

    /// <summary>DATA_FRAG carrying one fragment (<paramref name="fragmentNumber"/> is 1-based).</summary>
    public RtpsMessageBuilder DataFrag(EntityId reader, EntityId writer, long sn, uint fragmentNumber, ushort fragmentSize, uint sampleSize, ReadOnlySpan<byte> inlineQos, ReadOnlySpan<byte> fragment)
    {
        Start(SubmessageId.DataFrag, (byte)(inlineQos.IsEmpty ? 0 : 0x02));
        Buffer.WriteUInt16(0);
        Buffer.WriteUInt16(28);
        Buffer.WriteEntityId(reader);
        Buffer.WriteEntityId(writer);
        Buffer.WriteSequenceNumber(sn);
        Buffer.WriteUInt32(fragmentNumber);
        Buffer.WriteUInt16(1);
        Buffer.WriteUInt16(fragmentSize);
        Buffer.WriteUInt32(sampleSize);
        Buffer.WriteBytes(inlineQos);
        Buffer.WriteBytes(fragment);
        Finish();
        return this;
    }

    public RtpsMessageBuilder Heartbeat(EntityId reader, EntityId writer, long first, long last, int count, bool final, bool liveliness = false)
    {
        Start(SubmessageId.Heartbeat, (byte)((final ? 0x02 : 0) | (liveliness ? 0x04 : 0)));
        Buffer.WriteEntityId(reader);
        Buffer.WriteEntityId(writer);
        Buffer.WriteSequenceNumber(first);
        Buffer.WriteSequenceNumber(last);
        Buffer.WriteInt32(count);
        Finish();
        return this;
    }

    public RtpsMessageBuilder AckNack(EntityId reader, EntityId writer, SequenceNumberSet state, int count, bool final)
    {
        Start(SubmessageId.AckNack, (byte)(final ? 0x02 : 0));
        Buffer.WriteEntityId(reader);
        Buffer.WriteEntityId(writer);
        Buffer.WriteSet(state);
        Buffer.WriteInt32(count);
        Finish();
        return this;
    }

    public RtpsMessageBuilder Gap(EntityId reader, EntityId writer, long gapStart, SequenceNumberSet gapList)
    {
        Start(SubmessageId.Gap, 0);
        Buffer.WriteEntityId(reader);
        Buffer.WriteEntityId(writer);
        Buffer.WriteSequenceNumber(gapStart);
        Buffer.WriteSet(gapList);
        Finish();
        return this;
    }

    public RtpsMessageBuilder NackFrag(EntityId reader, EntityId writer, long sn, FragmentNumberSet state, int count)
    {
        Start(SubmessageId.NackFrag, 0);
        Buffer.WriteEntityId(reader);
        Buffer.WriteEntityId(writer);
        Buffer.WriteSequenceNumber(sn);
        Buffer.WriteUInt32(state.Base);
        Buffer.WriteUInt32((uint)state.NumBits);
        foreach (var w in state.Bitmap)
        {
            Buffer.WriteUInt32(w);
        }

        Buffer.WriteInt32(count);
        Finish();
        return this;
    }

    private void WritePayload(ReadOnlySpan<byte> payload)
    {
        if (payload.IsEmpty)
        {
            return;
        }

        var start = Buffer.Position;
        Buffer.WriteBytes(payload);
        var pad = (4 - (payload.Length & 3)) & 3;
        if (pad != 0 && payload.Length >= 4)
        {
            // Record the padding in the encapsulation options so receivers can strip it.
            var options = (ushort)(BinaryPrimitives.ReadUInt16BigEndian(payload[2..]) & ~0x3 | pad);
            Patch(start + 2, options);
        }
    }

    private void Patch(int offset, ushort valueBigEndian)
    {
        var current = Buffer.Position;
        Buffer.Position = offset;
        Buffer.WriteUInt16BigEndian(valueBigEndian);
        Buffer.Position = current;
    }

    public void Dispose() => Buffer.Dispose();
}

/// <summary>State carried between submessages of one received message.</summary>
public struct ReceiveState
{
    public GuidPrefix SourcePrefix;
    public GuidPrefix DestinationPrefix;
    public VendorId SourceVendor;
    public long? Timestamp;
}

/// <summary>A received DATA submessage. Spans point into the receive buffer and are only valid during the callback.</summary>
public readonly ref struct DataSubmessage
{
    public EntityId ReaderId { get; init; }

    public EntityId WriterId { get; init; }

    public long SequenceNumber { get; init; }

    public bool LittleEndian { get; init; }

    public ReadOnlySpan<byte> InlineQos { get; init; }

    /// <summary>Serialized payload with encapsulation header (padding stripped), empty for key-only/dispose samples.</summary>
    public ReadOnlySpan<byte> Payload { get; init; }
}

/// <summary>A received DATA_FRAG submessage.</summary>
public readonly ref struct DataFragSubmessage
{
    public EntityId ReaderId { get; init; }

    public EntityId WriterId { get; init; }

    public long SequenceNumber { get; init; }

    public uint FragmentStart { get; init; }

    public ushort FragmentsInSubmessage { get; init; }

    public ushort FragmentSize { get; init; }

    public uint SampleSize { get; init; }

    public bool LittleEndian { get; init; }

    public ReadOnlySpan<byte> InlineQos { get; init; }

    public ReadOnlySpan<byte> Fragments { get; init; }
}

public readonly record struct HeartbeatSubmessage(EntityId ReaderId, EntityId WriterId, long First, long Last, int Count, bool Final, bool Liveliness);

public sealed record AckNackSubmessage(EntityId ReaderId, EntityId WriterId, SequenceNumberSet State, int Count, bool Final);

public sealed record GapSubmessage(EntityId ReaderId, EntityId WriterId, long GapStart, SequenceNumberSet GapList);

public sealed record NackFragSubmessage(EntityId ReaderId, EntityId WriterId, long SequenceNumber, FragmentNumberSet State, int Count);

/// <summary>Receives parsed submessages.</summary>
public interface IRtpsMessageHandler
{
    void OnData(in ReceiveState state, in DataSubmessage data);

    void OnDataFrag(in ReceiveState state, in DataFragSubmessage frag);

    void OnHeartbeat(in ReceiveState state, HeartbeatSubmessage heartbeat);

    void OnAckNack(in ReceiveState state, AckNackSubmessage ackNack);

    void OnGap(in ReceiveState state, GapSubmessage gap);

    void OnNackFrag(in ReceiveState state, NackFragSubmessage nackFrag);
}

/// <summary>Parses RTPS messages and dispatches submessages, applying INFO_TS / INFO_DST / INFO_SRC.</summary>
public static class RtpsMessageParser
{
    public const int HeaderSize = 20;

    /// <summary>Parses one datagram. Submessages addressed (via INFO_DST) to another participant are skipped.</summary>
    /// <returns>False if the datagram is not an RTPS message.</returns>
    public static bool Parse(ReadOnlySpan<byte> datagram, GuidPrefix self, IRtpsMessageHandler handler)
    {
        if (datagram.Length < HeaderSize || !datagram[..4].SequenceEqual("RTPS"u8) || datagram[4] != 2)
        {
            return false;
        }

        var state = new ReceiveState
        {
            SourceVendor = new VendorId(datagram[6], datagram[7]),
            SourcePrefix = GuidPrefix.Read(datagram.Slice(8, 12)),
        };
        var forUs = true;
        var pos = HeaderSize;
        while (pos + 4 <= datagram.Length)
        {
            var id = datagram[pos];
            var flags = datagram[pos + 1];
            var little = (flags & 0x01) != 0;
            int length = little ? BinaryPrimitives.ReadUInt16LittleEndian(datagram[(pos + 2)..]) : BinaryPrimitives.ReadUInt16BigEndian(datagram[(pos + 2)..]);
            var bodyStart = pos + 4;
            if (length == 0 && id is not (SubmessageId.Pad or SubmessageId.InfoTs))
            {
                length = datagram.Length - bodyStart; // Last submessage extends to the end.
            }

            if (bodyStart + length > datagram.Length)
            {
                return true; // Truncated: ignore the rest.
            }

            var body = datagram.Slice(bodyStart, length);
            pos = bodyStart + length;
            try
            {
                switch (id)
                {
                    case SubmessageId.InfoTs:
                        if ((flags & 0x02) != 0)
                        {
                            state.Timestamp = null;
                        }
                        else
                        {
                            var r = new SpanReader(body, little);
                            state.Timestamp = RtpsTime.ToNanoseconds(r.ReadInt32(), r.ReadUInt32());
                        }

                        break;
                    case SubmessageId.InfoDst:
                        state.DestinationPrefix = GuidPrefix.Read(body);
                        forUs = state.DestinationPrefix.IsUnknown || state.DestinationPrefix == self;
                        break;
                    case SubmessageId.InfoSrc:
                        if (body.Length >= 20)
                        {
                            state.SourceVendor = new VendorId(body[6], body[7]);
                            state.SourcePrefix = GuidPrefix.Read(body.Slice(8, 12));
                        }

                        break;
                    case SubmessageId.Data when forUs:
                        ParseData(body, flags, little, in state, handler);
                        break;
                    case SubmessageId.DataFrag when forUs:
                        ParseDataFrag(body, flags, little, in state, handler);
                        break;
                    case SubmessageId.Heartbeat when forUs:
                    {
                        var r = new SpanReader(body, little);
                        handler.OnHeartbeat(in state, new HeartbeatSubmessage(r.ReadEntityId(), r.ReadEntityId(), r.ReadSequenceNumber(), r.ReadSequenceNumber(), r.ReadInt32(), (flags & 0x02) != 0, (flags & 0x04) != 0));
                        break;
                    }

                    case SubmessageId.AckNack when forUs:
                    {
                        var r = new SpanReader(body, little);
                        handler.OnAckNack(in state, new AckNackSubmessage(r.ReadEntityId(), r.ReadEntityId(), r.ReadSet(), r.ReadInt32(), (flags & 0x02) != 0));
                        break;
                    }

                    case SubmessageId.Gap when forUs:
                    {
                        var r = new SpanReader(body, little);
                        handler.OnGap(in state, new GapSubmessage(r.ReadEntityId(), r.ReadEntityId(), r.ReadSequenceNumber(), r.ReadSet()));
                        break;
                    }

                    case SubmessageId.NackFrag when forUs:
                    {
                        var r = new SpanReader(body, little);
                        var reader = r.ReadEntityId();
                        var writer = r.ReadEntityId();
                        var sn = r.ReadSequenceNumber();
                        var b = r.ReadUInt32();
                        var n = (int)r.ReadUInt32();
                        var set = new FragmentNumberSet(b, n);
                        for (var i = 0; i < set.Bitmap.Length; i++)
                        {
                            set.Bitmap[i] = r.ReadUInt32();
                        }

                        handler.OnNackFrag(in state, new NackFragSubmessage(reader, writer, sn, set, r.ReadInt32()));
                        break;
                    }
                }
            }
            catch (RtpsFormatException)
            {
                // Skip malformed submessages; the rest of the message may still be usable.
            }
        }

        return true;
    }

    private static void ParseData(ReadOnlySpan<byte> body, byte flags, bool little, in ReceiveState state, IRtpsMessageHandler handler)
    {
        var r = new SpanReader(body, little);
        r.ReadUInt16(); // extraFlags
        var toInlineQos = r.ReadUInt16();
        var reader = r.ReadEntityId();
        var writer = r.ReadEntityId();
        var sn = r.ReadSequenceNumber();
        r.Position = 4 + toInlineQos;
        ReadOnlySpan<byte> inlineQos = default;
        if ((flags & 0x02) != 0)
        {
            ParameterList.Parse(body[r.Position..], little, out var consumed);
            inlineQos = body.Slice(r.Position, consumed);
            r.Position += consumed;
        }

        ReadOnlySpan<byte> payload = default;
        if ((flags & 0x0c) != 0 && r.Remaining > 0)
        {
            payload = StripPadding(body[r.Position..]);
        }

        handler.OnData(in state, new DataSubmessage
        {
            ReaderId = reader,
            WriterId = writer,
            SequenceNumber = sn,
            LittleEndian = little,
            InlineQos = inlineQos,
            Payload = (flags & 0x04) != 0 ? payload : default,
        });
    }

    private static void ParseDataFrag(ReadOnlySpan<byte> body, byte flags, bool little, in ReceiveState state, IRtpsMessageHandler handler)
    {
        var r = new SpanReader(body, little);
        r.ReadUInt16();
        var toInlineQos = r.ReadUInt16();
        var reader = r.ReadEntityId();
        var writer = r.ReadEntityId();
        var sn = r.ReadSequenceNumber();
        var start = r.ReadUInt32();
        var count = r.ReadUInt16();
        var size = r.ReadUInt16();
        var sampleSize = r.ReadUInt32();
        r.Position = 4 + toInlineQos;
        ReadOnlySpan<byte> inlineQos = default;
        if ((flags & 0x02) != 0)
        {
            ParameterList.Parse(body[r.Position..], little, out var consumed);
            inlineQos = body.Slice(r.Position, consumed);
            r.Position += consumed;
        }

        handler.OnDataFrag(in state, new DataFragSubmessage
        {
            ReaderId = reader,
            WriterId = writer,
            SequenceNumber = sn,
            FragmentStart = start,
            FragmentsInSubmessage = count,
            FragmentSize = size,
            SampleSize = sampleSize,
            LittleEndian = little,
            InlineQos = inlineQos,
            Fragments = body[r.Position..],
        });
    }

    /// <summary>Removes trailing padding announced in the encapsulation options (low two bits).</summary>
    public static ReadOnlySpan<byte> StripPadding(ReadOnlySpan<byte> payload)
    {
        if (payload.Length < 4)
        {
            return payload;
        }

        var pad = payload[3] & 0x03;
        return pad > 0 && payload.Length - pad >= 4 ? payload[..^pad] : payload;
    }
}
