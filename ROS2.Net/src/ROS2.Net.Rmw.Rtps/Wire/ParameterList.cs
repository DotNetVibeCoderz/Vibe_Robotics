using System.Buffers.Binary;
using System.Text;

namespace ROS2.Net.Rmw.Rtps.Wire;

/// <summary>Parameter ids used by SPDP, SEDP and inline QoS.</summary>
public static class ParameterId
{
    public const ushort Pad = 0x0000;
    public const ushort Sentinel = 0x0001;
    public const ushort ParticipantLeaseDuration = 0x0002;
    public const ushort TopicName = 0x0005;
    public const ushort TypeName = 0x0007;
    public const ushort DomainId = 0x000f;
    public const ushort ProtocolVersion = 0x0015;
    public const ushort VendorId = 0x0016;
    public const ushort Reliability = 0x001a;
    public const ushort Liveliness = 0x001b;
    public const ushort Durability = 0x001d;
    public const ushort Deadline = 0x0023;
    public const ushort Lifespan = 0x002b;
    public const ushort UserData = 0x002c;
    public const ushort UnicastLocator = 0x002f;
    public const ushort MulticastLocator = 0x0030;
    public const ushort DefaultUnicastLocator = 0x0031;
    public const ushort MetatrafficUnicastLocator = 0x0032;
    public const ushort MetatrafficMulticastLocator = 0x0033;
    public const ushort History = 0x0040;
    public const ushort ExpectsInlineQos = 0x0043;
    public const ushort DefaultMulticastLocator = 0x0048;
    public const ushort ParticipantGuid = 0x0050;
    public const ushort BuiltinEndpointSet = 0x0058;
    public const ushort EndpointGuid = 0x005a;
    public const ushort EntityName = 0x0062;
    public const ushort KeyHash = 0x0070;
    public const ushort StatusInfo = 0x0071;
    public const ushort DataRepresentation = 0x0073;

    /// <summary>DDS-RPC standard related sample identity.</summary>
    public const ushort RelatedSampleIdentity = 0x0083;

    /// <summary>eProsima's vendor-specific related sample identity, used by rmw_fastrtps for service replies.</summary>
    public const ushort FastDdsRelatedSampleIdentity = 0x800f;

    /// <summary>Must-understand flag in the pid.</summary>
    public const ushort MustUnderstandFlag = 0x4000;

    /// <summary>Vendor-specific range flag.</summary>
    public const ushort VendorSpecificFlag = 0x8000;
}

/// <summary>
/// Writes a PL_CDR_LE parameter list: (uint16 pid, uint16 length, value padded to 4) entries ending
/// with PID_SENTINEL.
/// </summary>
public sealed class ParameterListWriter
{
    private readonly RtpsBuffer _buffer;
    private int _start = -1;

    public ParameterListWriter(RtpsBuffer buffer) => _buffer = buffer;

    /// <summary>Writes the 4-byte encapsulation header for PL_CDR_LE (used when the list is a serialized payload).</summary>
    public void WriteEncapsulation()
    {
        _buffer.WriteBytes([0x00, 0x03, 0x00, 0x00]);
    }

    public void Begin(ushort pid)
    {
        _buffer.WriteUInt16(pid);
        _buffer.WriteUInt16(0);
        _start = _buffer.Position;
    }

    public void End()
    {
        _buffer.Align4();
        _buffer.PatchUInt16(_start - 2, (ushort)(_buffer.Position - _start));
        _start = -1;
    }

    public RtpsBuffer Buffer => _buffer;

    public void WriteUInt32(ushort pid, uint value)
    {
        Begin(pid);
        _buffer.WriteUInt32(value);
        End();
    }

    public void WriteInt32(ushort pid, int value) => WriteUInt32(pid, (uint)value);

    public void WriteString(ushort pid, string value)
    {
        Begin(pid);
        var bytes = Encoding.UTF8.GetBytes(value);
        _buffer.WriteUInt32((uint)bytes.Length + 1);
        _buffer.WriteBytes(bytes);
        _buffer.WriteByte(0);
        End();
    }

    public void WriteGuid(ushort pid, RtpsGuid guid)
    {
        Begin(pid);
        _buffer.WriteGuid(guid);
        End();
    }

    public void WriteLocator(ushort pid, Locator locator)
    {
        Begin(pid);
        _buffer.WriteLocator(locator);
        End();
    }

    public void WriteDuration(ushort pid, TimeSpan? duration)
    {
        Begin(pid);
        var (s, f) = RtpsTime.FromDuration(duration);
        _buffer.WriteInt32(s);
        _buffer.WriteUInt32(f);
        End();
    }

    public void WriteOctets(ushort pid, ReadOnlySpan<byte> bytes)
    {
        Begin(pid);
        _buffer.WriteUInt32((uint)bytes.Length);
        _buffer.WriteBytes(bytes);
        End();
    }

    public void WriteRaw(ushort pid, ReadOnlySpan<byte> value)
    {
        Begin(pid);
        _buffer.WriteBytes(value);
        End();
    }

    public void WriteSentinel()
    {
        _buffer.WriteUInt16(ParameterId.Sentinel);
        _buffer.WriteUInt16(0);
    }
}

/// <summary>A parsed parameter.</summary>
public readonly record struct RtpsParameter(ushort Id, int Offset, int Length);

/// <summary>Reads a parameter list in either byte order.</summary>
public sealed class ParameterList
{
    private readonly byte[] _data;

    private ParameterList(byte[] data, bool littleEndian, List<RtpsParameter> parameters)
    {
        _data = data;
        LittleEndian = littleEndian;
        Parameters = parameters;
    }

    public bool LittleEndian { get; }

    public IReadOnlyList<RtpsParameter> Parameters { get; }

    /// <summary>Parses a serialized payload that starts with a PL_CDR_BE/PL_CDR_LE encapsulation header.</summary>
    public static ParameterList ParsePayload(ReadOnlySpan<byte> payload)
    {
        if (payload.Length < 4)
        {
            throw new RtpsFormatException("Parameter list payload too short.");
        }

        var little = (payload[1] & 0x01) != 0;
        return Parse(payload[4..], little, out _);
    }

    /// <summary>Parses a raw list (inline QoS). <paramref name="consumed"/> includes the sentinel.</summary>
    public static ParameterList Parse(ReadOnlySpan<byte> data, bool littleEndian, out int consumed)
    {
        var list = new List<RtpsParameter>();
        var pos = 0;
        while (true)
        {
            if (pos + 4 > data.Length)
            {
                // Tolerate a missing sentinel at the end of a payload.
                consumed = pos;
                break;
            }

            var pid = littleEndian ? BinaryPrimitives.ReadUInt16LittleEndian(data[pos..]) : BinaryPrimitives.ReadUInt16BigEndian(data[pos..]);
            var len = littleEndian ? BinaryPrimitives.ReadUInt16LittleEndian(data[(pos + 2)..]) : BinaryPrimitives.ReadUInt16BigEndian(data[(pos + 2)..]);
            pos += 4;
            if (pid == ParameterId.Sentinel)
            {
                consumed = pos;
                break;
            }

            if (pos + len > data.Length)
            {
                throw new RtpsFormatException($"Parameter 0x{pid:x4} overruns the list.");
            }

            list.Add(new RtpsParameter(pid, pos, len));
            pos += len;
        }

        return new ParameterList(data[..consumed].ToArray(), littleEndian, list);
    }

    public bool TryGet(ushort pid, out ReadOnlySpan<byte> value)
    {
        foreach (var p in Parameters)
        {
            if ((p.Id & 0x3fff) == (pid & 0x3fff) && ((p.Id & ParameterId.VendorSpecificFlag) == (pid & ParameterId.VendorSpecificFlag)))
            {
                value = _data.AsSpan(p.Offset, p.Length);
                return true;
            }
        }

        value = default;
        return false;
    }

    public IEnumerable<ReadOnlyMemory<byte>> GetAll(ushort pid)
    {
        foreach (var p in Parameters)
        {
            if ((p.Id & 0x3fff) == (pid & 0x3fff) && ((p.Id & ParameterId.VendorSpecificFlag) == (pid & ParameterId.VendorSpecificFlag)))
            {
                yield return _data.AsMemory(p.Offset, p.Length);
            }
        }
    }

    public uint? GetUInt32(ushort pid) =>
        TryGet(pid, out var v) && v.Length >= 4 ? (LittleEndian ? BinaryPrimitives.ReadUInt32LittleEndian(v) : BinaryPrimitives.ReadUInt32BigEndian(v)) : null;

    public int? GetInt32(ushort pid) => GetUInt32(pid) is { } u ? (int)u : null;

    public string? GetString(ushort pid) => TryGet(pid, out var v) ? DecodeString(v, LittleEndian) : null;

    public RtpsGuid? GetGuid(ushort pid) => TryGet(pid, out var v) && v.Length >= 16 ? RtpsGuid.Read(v) : null;

    public TimeSpan? GetDuration(ushort pid, TimeSpan? fallback)
    {
        if (!TryGet(pid, out var v) || v.Length < 8)
        {
            return fallback;
        }

        var r = new SpanReader(v, LittleEndian);
        return RtpsTime.ToDuration(r.ReadInt32(), r.ReadUInt32());
    }

    public IEnumerable<Locator> GetLocators(ushort pid)
    {
        foreach (var m in GetAll(pid))
        {
            if (m.Length >= Locator.Size)
            {
                yield return Locator.Read(m.Span, LittleEndian);
            }
        }
    }

    public byte[]? GetOctets(ushort pid)
    {
        if (!TryGet(pid, out var v) || v.Length < 4)
        {
            return null;
        }

        var n = (int)(LittleEndian ? BinaryPrimitives.ReadUInt32LittleEndian(v) : BinaryPrimitives.ReadUInt32BigEndian(v));
        return n <= v.Length - 4 ? v.Slice(4, n).ToArray() : null;
    }

    public static string DecodeString(ReadOnlySpan<byte> v, bool littleEndian)
    {
        if (v.Length < 4)
        {
            return string.Empty;
        }

        var n = (int)(littleEndian ? BinaryPrimitives.ReadUInt32LittleEndian(v) : BinaryPrimitives.ReadUInt32BigEndian(v));
        if (n <= 0 || n > v.Length - 4)
        {
            return string.Empty;
        }

        var bytes = v.Slice(4, n);
        if (bytes[^1] == 0)
        {
            bytes = bytes[..^1];
        }

        return Encoding.UTF8.GetString(bytes);
    }
}
