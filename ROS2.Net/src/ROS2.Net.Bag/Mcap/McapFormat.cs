using System.Buffers.Binary;
using System.Text;

namespace ROS2.Net.Bag.Mcap;

/// <summary>Record opcodes of the MCAP format (https://mcap.dev/spec).</summary>
public enum McapOpcode : byte
{
    Header = 0x01,
    Footer = 0x02,
    Schema = 0x03,
    Channel = 0x04,
    Message = 0x05,
    Chunk = 0x06,
    MessageIndex = 0x07,
    ChunkIndex = 0x08,
    Attachment = 0x09,
    AttachmentIndex = 0x0A,
    Statistics = 0x0B,
    Metadata = 0x0C,
    MetadataIndex = 0x0D,
    SummaryOffset = 0x0E,
    DataEnd = 0x0F,
}

/// <summary>Chunk compression.</summary>
public enum McapCompression
{
    None,
    Zstd,
    Lz4,
}

/// <summary>Raised for malformed MCAP data or failed CRC checks.</summary>
public sealed class McapException(string message) : Exception(message);

/// <summary>A schema record.</summary>
public sealed record McapSchema(ushort Id, string Name, string Encoding, byte[] Data)
{
    public string DataText => Encoding is "ros2msg" or "ros2idl" or "jsonschema" ? System.Text.Encoding.UTF8.GetString(Data) : string.Empty;
}

/// <summary>A channel record.</summary>
public sealed record McapChannel(ushort Id, ushort SchemaId, string Topic, string MessageEncoding, IReadOnlyDictionary<string, string> Metadata);

/// <summary>A message record. <see cref="Data"/> references the decoded chunk buffer and stays valid.</summary>
public readonly record struct McapMessage(ushort ChannelId, uint Sequence, ulong LogTime, ulong PublishTime, ReadOnlyMemory<byte> Data);

/// <summary>A chunk index record from the summary section.</summary>
public sealed record McapChunkIndex(
    ulong MessageStartTime,
    ulong MessageEndTime,
    ulong ChunkStartOffset,
    ulong ChunkLength,
    IReadOnlyDictionary<ushort, ulong> MessageIndexOffsets,
    ulong MessageIndexLength,
    string Compression,
    ulong CompressedSize,
    ulong UncompressedSize);

/// <summary>The statistics record.</summary>
public sealed record McapStatistics(
    ulong MessageCount,
    ushort SchemaCount,
    uint ChannelCount,
    uint AttachmentCount,
    uint MetadataCount,
    uint ChunkCount,
    ulong MessageStartTime,
    ulong MessageEndTime,
    IReadOnlyDictionary<ushort, ulong> ChannelMessageCounts);

/// <summary>A metadata record (name + string map).</summary>
public sealed record McapMetadata(string Name, IReadOnlyDictionary<string, string> Values);

internal static class McapConstants
{
    /// <summary>"\x89MCAP0\r\n".</summary>
    public static ReadOnlySpan<byte> Magic => [0x89, (byte)'M', (byte)'C', (byte)'A', (byte)'P', (byte)'0', (byte)'\r', (byte)'\n'];

    public static string CompressionName(McapCompression c) => c switch
    {
        McapCompression.Zstd => "zstd",
        McapCompression.Lz4 => "lz4",
        _ => string.Empty,
    };
}

/// <summary>Growable little-endian byte buffer for building record contents.</summary>
internal sealed class RecordBuffer
{
    private byte[] _buffer;
    private int _length;

    public RecordBuffer(int capacity = 256) => _buffer = new byte[capacity];

    public int Length => _length;

    public ReadOnlySpan<byte> Span => _buffer.AsSpan(0, _length);

    public byte[] Buffer => _buffer;

    public void Clear() => _length = 0;

    private Span<byte> Grow(int n)
    {
        if (_length + n > _buffer.Length)
        {
            Array.Resize(ref _buffer, Math.Max(_buffer.Length * 2, _length + n));
        }

        var s = _buffer.AsSpan(_length, n);
        _length += n;
        return s;
    }

    public void U8(byte v) => Grow(1)[0] = v;

    public void U16(ushort v) => BinaryPrimitives.WriteUInt16LittleEndian(Grow(2), v);

    public void U32(uint v) => BinaryPrimitives.WriteUInt32LittleEndian(Grow(4), v);

    public void U64(ulong v) => BinaryPrimitives.WriteUInt64LittleEndian(Grow(8), v);

    public void Raw(ReadOnlySpan<byte> bytes) => bytes.CopyTo(Grow(bytes.Length));

    public void Str(string s)
    {
        var n = Encoding.UTF8.GetByteCount(s);
        U32((uint)n);
        Encoding.UTF8.GetBytes(s, Grow(n));
    }

    public void Bytes32(ReadOnlySpan<byte> bytes)
    {
        U32((uint)bytes.Length);
        Raw(bytes);
    }

    public void StringMap(IReadOnlyDictionary<string, string> map)
    {
        var lengthAt = _length;
        U32(0);
        foreach (var (k, v) in map.OrderBy(kv => kv.Key, StringComparer.Ordinal))
        {
            Str(k);
            Str(v);
        }

        BinaryPrimitives.WriteUInt32LittleEndian(_buffer.AsSpan(lengthAt), (uint)(_length - lengthAt - 4));
    }

    /// <summary>Reserves a record header (opcode + u64 length) and returns its position for <see cref="EndRecord"/>.</summary>
    public int BeginRecord(McapOpcode op)
    {
        var at = _length;
        U8((byte)op);
        U64(0);
        return at;
    }

    public void EndRecord(int at) =>
        BinaryPrimitives.WriteUInt64LittleEndian(_buffer.AsSpan(at + 1), (ulong)(_length - at - 9));
}

/// <summary>Little-endian cursor over record contents.</summary>
internal ref struct RecordReader(ReadOnlySpan<byte> data)
{
    private readonly ReadOnlySpan<byte> _data = data;
    private int _pos;

    public readonly int Position => _pos;

    public readonly int Remaining => _data.Length - _pos;

    private ReadOnlySpan<byte> Take(int n)
    {
        if (_pos + n > _data.Length)
        {
            throw new McapException($"Record truncated: needed {n} bytes at {_pos}, have {_data.Length - _pos}.");
        }

        var s = _data.Slice(_pos, n);
        _pos += n;
        return s;
    }

    public byte U8() => Take(1)[0];

    public ushort U16() => BinaryPrimitives.ReadUInt16LittleEndian(Take(2));

    public uint U32() => BinaryPrimitives.ReadUInt32LittleEndian(Take(4));

    public ulong U64() => BinaryPrimitives.ReadUInt64LittleEndian(Take(8));

    public string Str() => Encoding.UTF8.GetString(Take((int)U32()));

    public ReadOnlySpan<byte> Bytes32() => Take((int)U32());

    public ReadOnlySpan<byte> Rest() => Take(Remaining);

    public Dictionary<string, string> StringMap()
    {
        var len = (int)U32();
        var end = _pos + len;
        var map = new Dictionary<string, string>(StringComparer.Ordinal);
        while (_pos < end)
        {
            var k = Str();
            map[k] = Str();
        }

        return map;
    }

    public Dictionary<ushort, ulong> U16U64Map()
    {
        var len = (int)U32();
        var end = _pos + len;
        var map = new Dictionary<ushort, ulong>();
        while (_pos < end)
        {
            var k = U16();
            map[k] = U64();
        }

        return map;
    }
}
