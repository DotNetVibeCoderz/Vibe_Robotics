using System.Buffers;
using System.Buffers.Binary;
using System.Net;
using System.Net.Sockets;

namespace ROS2.Net.Rmw.Rtps.Wire;

/// <summary>An RTPS locator (kind, port, 16-byte address).</summary>
public readonly record struct Locator(int Kind, uint Port, IPAddress Address)
{
    public const int KindUdpV4 = 1;
    public const int KindUdpV6 = 2;
    public const int Size = 24;

    public static Locator UdpV4(IPAddress address, int port) => new(KindUdpV4, (uint)port, address);

    public IPEndPoint ToEndPoint() => new(Address, (int)Port);

    /// <summary>Writes the 24-byte locator in the given byte order (address bytes are never swapped).</summary>
    public void Write(Span<byte> d, bool littleEndian = true)
    {
        if (littleEndian)
        {
            BinaryPrimitives.WriteInt32LittleEndian(d, Kind);
            BinaryPrimitives.WriteUInt32LittleEndian(d[4..], Port);
        }
        else
        {
            BinaryPrimitives.WriteInt32BigEndian(d, Kind);
            BinaryPrimitives.WriteUInt32BigEndian(d[4..], Port);
        }

        var addr = d.Slice(8, 16);
        addr.Clear();
        if (Address.AddressFamily == AddressFamily.InterNetwork)
        {
            Address.TryWriteBytes(addr[12..], out _);
        }
        else
        {
            Address.TryWriteBytes(addr, out _);
        }
    }

    public static Locator Read(ReadOnlySpan<byte> s, bool littleEndian)
    {
        var kind = littleEndian ? BinaryPrimitives.ReadInt32LittleEndian(s) : BinaryPrimitives.ReadInt32BigEndian(s);
        var port = littleEndian ? BinaryPrimitives.ReadUInt32LittleEndian(s[4..]) : BinaryPrimitives.ReadUInt32BigEndian(s[4..]);
        var address = kind == KindUdpV4 ? new IPAddress(s.Slice(20, 4)) : new IPAddress(s.Slice(8, 16));
        return new Locator(kind, port, address);
    }

    public override string ToString() => $"udp{(Kind == KindUdpV4 ? "v4" : "v6")}://{Address}:{Port}";
}

/// <summary>RTPS time helpers: <c>Time_t</c>/<c>Duration_t</c> are (int32 seconds, uint32 fraction of 2^-32 s).</summary>
public static class RtpsTime
{
    public static (int Seconds, uint Fraction) FromNanoseconds(long ns)
    {
        var seconds = Math.DivRem(ns, 1_000_000_000L, out var rem);
        if (rem < 0)
        {
            seconds--;
            rem += 1_000_000_000L;
        }

        return ((int)seconds, (uint)((rem << 32) / 1_000_000_000L));
    }

    public static long ToNanoseconds(int seconds, uint fraction) =>
        (seconds * 1_000_000_000L) + (long)(((ulong)fraction * 1_000_000_000UL) >> 32);

    /// <summary>Encodes a duration; null (infinite) becomes the RTPS infinite value.</summary>
    public static (int Seconds, uint Fraction) FromDuration(TimeSpan? d) =>
        d is null || d.Value == TimeSpan.MaxValue ? (0x7fffffff, 0xffffffff) : FromNanoseconds(d.Value.Ticks * 100);

    public static TimeSpan? ToDuration(int seconds, uint fraction) =>
        seconds == 0x7fffffff ? null : TimeSpan.FromTicks(ToNanoseconds(seconds, fraction) / 100);
}

/// <summary>A sequence number set (<c>SequenceNumberSet</c>): base + up to 256 bits.</summary>
public sealed class SequenceNumberSet
{
    public SequenceNumberSet(long bitmapBase, int numBits)
    {
        if (numBits is < 0 or > 256)
        {
            throw new ArgumentOutOfRangeException(nameof(numBits));
        }

        Base = bitmapBase;
        NumBits = numBits;
        Bitmap = new uint[(numBits + 31) / 32];
    }

    public long Base { get; }

    public int NumBits { get; }

    public uint[] Bitmap { get; }

    public bool Contains(long sn)
    {
        var i = sn - Base;
        return i >= 0 && i < NumBits && (Bitmap[i / 32] & (1u << (31 - (int)(i % 32)))) != 0;
    }

    public void Add(long sn)
    {
        var i = sn - Base;
        if (i < 0 || i >= NumBits)
        {
            throw new ArgumentOutOfRangeException(nameof(sn));
        }

        Bitmap[i / 32] |= 1u << (31 - (int)(i % 32));
    }

    public IEnumerable<long> Members()
    {
        for (var i = 0; i < NumBits; i++)
        {
            if ((Bitmap[i / 32] & (1u << (31 - (i % 32)))) != 0)
            {
                yield return Base + i;
            }
        }
    }

    public int WireSize => 12 + (4 * Bitmap.Length);
}

/// <summary>A growable pooled byte buffer for building RTPS messages (always little-endian).</summary>
public sealed class RtpsBuffer : IDisposable
{
    private byte[] _buffer;

    public RtpsBuffer(int capacity = 1500) => _buffer = ArrayPool<byte>.Shared.Rent(capacity);

    public int Position { get; set; }

    public ReadOnlySpan<byte> Span => _buffer.AsSpan(0, Position);

    public ReadOnlyMemory<byte> Memory => _buffer.AsMemory(0, Position);

    public void Reset() => Position = 0;

    public Span<byte> Reserve(int length)
    {
        Ensure(length);
        var s = _buffer.AsSpan(Position, length);
        Position += length;
        return s;
    }

    public void WriteByte(byte v) => Reserve(1)[0] = v;

    public void WriteUInt16(ushort v) => BinaryPrimitives.WriteUInt16LittleEndian(Reserve(2), v);

    public void WriteUInt16BigEndian(ushort v) => BinaryPrimitives.WriteUInt16BigEndian(Reserve(2), v);

    public void WriteInt32(int v) => BinaryPrimitives.WriteInt32LittleEndian(Reserve(4), v);

    public void WriteUInt32(uint v) => BinaryPrimitives.WriteUInt32LittleEndian(Reserve(4), v);

    public void WriteSequenceNumber(long sn)
    {
        WriteInt32((int)(sn >> 32));
        WriteUInt32((uint)sn);
    }

    public void WriteBytes(ReadOnlySpan<byte> bytes) => bytes.CopyTo(Reserve(bytes.Length));

    public void WriteGuidPrefix(GuidPrefix p) => p.Write(Reserve(12));

    public void WriteEntityId(EntityId e) => e.Write(Reserve(4));

    public void WriteGuid(RtpsGuid g) => g.Write(Reserve(16));

    public void WriteLocator(Locator l) => l.Write(Reserve(Locator.Size));

    public void WriteSet(SequenceNumberSet set)
    {
        WriteSequenceNumber(set.Base);
        WriteUInt32((uint)set.NumBits);
        foreach (var w in set.Bitmap)
        {
            WriteUInt32(w);
        }
    }

    public void Align4()
    {
        while ((Position & 3) != 0)
        {
            WriteByte(0);
        }
    }

    public void PatchUInt16(int offset, ushort value) => BinaryPrimitives.WriteUInt16LittleEndian(_buffer.AsSpan(offset), value);

    private void Ensure(int extra)
    {
        if (Position + extra <= _buffer.Length)
        {
            return;
        }

        var next = ArrayPool<byte>.Shared.Rent(Math.Max(_buffer.Length * 2, Position + extra));
        _buffer.AsSpan(0, Position).CopyTo(next);
        ArrayPool<byte>.Shared.Return(_buffer);
        _buffer = next;
    }

    public void Dispose()
    {
        var b = _buffer;
        _buffer = [];
        if (b.Length > 0)
        {
            ArrayPool<byte>.Shared.Return(b);
        }
    }
}

/// <summary>A cursor over received bytes with a switchable byte order.</summary>
public ref struct SpanReader(ReadOnlySpan<byte> data, bool littleEndian)
{
    private readonly ReadOnlySpan<byte> _data = data;

    public int Position { get; set; }

    public bool LittleEndian { get; set; } = littleEndian;

    public readonly int Remaining => _data.Length - Position;

    public readonly ReadOnlySpan<byte> Data => _data;

    public ReadOnlySpan<byte> Take(int n)
    {
        if (n < 0 || Position + n > _data.Length)
        {
            throw new RtpsFormatException($"Truncated RTPS data: wanted {n} bytes at {Position}, have {_data.Length - Position}.");
        }

        var s = _data.Slice(Position, n);
        Position += n;
        return s;
    }

    public byte ReadByte() => Take(1)[0];

    public ushort ReadUInt16() => LittleEndian ? BinaryPrimitives.ReadUInt16LittleEndian(Take(2)) : BinaryPrimitives.ReadUInt16BigEndian(Take(2));

    public short ReadInt16() => (short)ReadUInt16();

    public uint ReadUInt32() => LittleEndian ? BinaryPrimitives.ReadUInt32LittleEndian(Take(4)) : BinaryPrimitives.ReadUInt32BigEndian(Take(4));

    public int ReadInt32() => (int)ReadUInt32();

    public long ReadSequenceNumber()
    {
        var high = ReadInt32();
        var low = ReadUInt32();
        return ((long)high << 32) | low;
    }

    public GuidPrefix ReadGuidPrefix() => GuidPrefix.Read(Take(12));

    public EntityId ReadEntityId() => EntityId.Read(Take(4));

    public RtpsGuid ReadGuid() => RtpsGuid.Read(Take(16));

    public SequenceNumberSet ReadSet()
    {
        var b = ReadSequenceNumber();
        var n = (int)ReadUInt32();
        if (n is < 0 or > 256)
        {
            throw new RtpsFormatException($"Invalid set size {n}.");
        }

        var set = new SequenceNumberSet(b, n);
        for (var i = 0; i < set.Bitmap.Length; i++)
        {
            set.Bitmap[i] = ReadUInt32();
        }

        return set;
    }
}

/// <summary>Malformed RTPS data.</summary>
public sealed class RtpsFormatException(string message) : Exception(message);
