using System.Buffers.Binary;
using System.IO.Hashing;

namespace ROS2.Net.Bag.Mcap;

/// <summary>Options for <see cref="McapWriter"/>.</summary>
public sealed class McapWriterOptions
{
    /// <summary>Profile in the Header record; rosbag2 uses <c>ros2</c>.</summary>
    public string Profile { get; init; } = "ros2";

    public string Library { get; init; } = "ROS2.Net.Bag (Gravicode Studios)";

    /// <summary>Uncompressed chunk size that triggers a flush.</summary>
    public int ChunkSize { get; init; } = 1024 * 1024;

    public McapCompression Compression { get; init; } = McapCompression.None;

    /// <summary>Compute the data section and chunk CRCs.</summary>
    public bool ComputeCrc { get; init; } = true;
}

/// <summary>
/// Writes indexed, chunked MCAP files: data section (schemas, channels, chunks + message indexes,
/// metadata), DataEnd, then a summary section (schemas, channels, statistics, chunk and metadata
/// indexes) with summary offsets and a CRC-protected footer. Not thread-safe; wrap calls in a lock.
/// </summary>
public sealed class McapWriter : IDisposable
{
    private readonly Stream _out;
    private readonly bool _leaveOpen;
    private readonly McapWriterOptions _options;
    private readonly Crc32 _dataCrc = new();
    private readonly RecordBuffer _scratch = new();
    private readonly RecordBuffer _chunk = new(64 * 1024);
    private readonly SortedDictionary<ushort, List<(ulong Time, ulong Offset)>> _chunkIndex = new();
    private readonly List<McapSchema> _schemas = [];
    private readonly List<McapChannel> _channels = [];
    private readonly List<McapChunkIndex> _chunkIndexes = [];
    private readonly List<(ulong Offset, ulong Length, string Name)> _metadataIndexes = [];
    private readonly Dictionary<ushort, ulong> _channelCounts = [];
    private ulong _position;
    private ulong _chunkStart = ulong.MaxValue;
    private ulong _chunkEnd;
    private ulong _messageCount;
    private ulong _messageStart = ulong.MaxValue;
    private ulong _messageEnd;
    private bool _closed;

    public McapWriter(Stream output, McapWriterOptions? options = null, bool leaveOpen = false)
    {
        _out = output;
        _leaveOpen = leaveOpen;
        _options = options ?? new McapWriterOptions();
        WriteData(McapConstants.Magic);
        _scratch.Clear();
        var at = _scratch.BeginRecord(McapOpcode.Header);
        _scratch.Str(_options.Profile);
        _scratch.Str(_options.Library);
        _scratch.EndRecord(at);
        WriteData(_scratch.Span);
    }

    /// <summary>Bytes written so far.</summary>
    public long Position => (long)_position;

    public ulong MessageCount => _messageCount;

    public IReadOnlyList<McapChannel> Channels => _channels;

    /// <summary>Registers a schema and returns its id (ids start at 1; 0 means "no schema").</summary>
    public ushort AddSchema(string name, string encoding, ReadOnlySpan<byte> data)
    {
        var id = (ushort)(_schemas.Count + 1);
        var schema = new McapSchema(id, name, encoding, data.ToArray());
        _schemas.Add(schema);
        WriteSchemaRecord(_scratch, schema);
        WriteData(_scratch.Span);
        return id;
    }

    /// <summary>Registers a channel and returns its id.</summary>
    public ushort AddChannel(ushort schemaId, string topic, string messageEncoding, IReadOnlyDictionary<string, string>? metadata = null)
    {
        var id = (ushort)_channels.Count;
        var channel = new McapChannel(id, schemaId, topic, messageEncoding, metadata ?? new Dictionary<string, string>());
        _channels.Add(channel);
        _channelCounts[id] = 0;
        WriteChannelRecord(_scratch, channel);
        WriteData(_scratch.Span);
        return id;
    }

    /// <summary>Appends a message to the current chunk.</summary>
    public void WriteMessage(ushort channelId, uint sequence, ulong logTime, ulong publishTime, ReadOnlySpan<byte> data)
    {
        ObjectDisposedException.ThrowIf(_closed, this);
        if (channelId >= _channels.Count)
        {
            throw new ArgumentOutOfRangeException(nameof(channelId), $"Channel {channelId} has not been added.");
        }

        var offset = (ulong)_chunk.Length;
        var at = _chunk.BeginRecord(McapOpcode.Message);
        _chunk.U16(channelId);
        _chunk.U32(sequence);
        _chunk.U64(logTime);
        _chunk.U64(publishTime);
        _chunk.Raw(data);
        _chunk.EndRecord(at);

        if (!_chunkIndex.TryGetValue(channelId, out var list))
        {
            _chunkIndex[channelId] = list = [];
        }

        list.Add((logTime, offset));
        _chunkStart = Math.Min(_chunkStart, logTime);
        _chunkEnd = Math.Max(_chunkEnd, logTime);
        _messageStart = Math.Min(_messageStart, logTime);
        _messageEnd = Math.Max(_messageEnd, logTime);
        _messageCount++;
        _channelCounts[channelId]++;

        if (_chunk.Length >= _options.ChunkSize)
        {
            FlushChunk();
        }
    }

    /// <summary>Writes a metadata record (outside chunks) and indexes it in the summary.</summary>
    public void AddMetadata(string name, IReadOnlyDictionary<string, string> values)
    {
        FlushChunk();
        _scratch.Clear();
        var at = _scratch.BeginRecord(McapOpcode.Metadata);
        _scratch.Str(name);
        _scratch.StringMap(values);
        _scratch.EndRecord(at);
        _metadataIndexes.Add((_position, (ulong)_scratch.Length, name));
        WriteData(_scratch.Span);
    }

    /// <summary>Writes the buffered chunk, followed by its message index records.</summary>
    public void FlushChunk()
    {
        if (_chunk.Length == 0)
        {
            return;
        }

        var uncompressed = _chunk.Span;
        var crc = _options.ComputeCrc ? Crc32.HashToUInt32(uncompressed) : 0u;
        var compressed = McapCompressionCodec.Compress(_options.Compression, uncompressed);
        var compression = McapConstants.CompressionName(_options.Compression);

        var chunkOffset = _position;
        _scratch.Clear();
        var at = _scratch.BeginRecord(McapOpcode.Chunk);
        _scratch.U64(_chunkStart);
        _scratch.U64(_chunkEnd);
        _scratch.U64((ulong)uncompressed.Length);
        _scratch.U32(crc);
        _scratch.Str(compression);
        _scratch.U64((ulong)compressed.Length);
        _scratch.Raw(compressed);
        _scratch.EndRecord(at);
        WriteData(_scratch.Span);
        var chunkLength = _position - chunkOffset;

        var indexOffsets = new Dictionary<ushort, ulong>();
        var indexStart = _position;
        foreach (var (channel, entries) in _chunkIndex)
        {
            if (entries.Count == 0)
            {
                continue;
            }

            indexOffsets[channel] = _position;
            _scratch.Clear();
            var mi = _scratch.BeginRecord(McapOpcode.MessageIndex);
            _scratch.U16(channel);
            _scratch.U32((uint)(entries.Count * 16));
            foreach (var (time, offset) in entries)
            {
                _scratch.U64(time);
                _scratch.U64(offset);
            }

            _scratch.EndRecord(mi);
            WriteData(_scratch.Span);
            entries.Clear();
        }

        _chunkIndexes.Add(new McapChunkIndex(_chunkStart, _chunkEnd, chunkOffset, chunkLength, indexOffsets,
            _position - indexStart, compression, (ulong)compressed.Length, (ulong)uncompressed.Length));
        _chunk.Clear();
        _chunkStart = ulong.MaxValue;
        _chunkEnd = 0;
    }

    /// <summary>Flushes, writes DataEnd, the summary section and the footer.</summary>
    public void Close()
    {
        if (_closed)
        {
            return;
        }

        FlushChunk();
        _scratch.Clear();
        var de = _scratch.BeginRecord(McapOpcode.DataEnd);
        _scratch.U32(_options.ComputeCrc ? _dataCrc.GetCurrentHashAsUInt32() : 0u);
        _scratch.EndRecord(de);
        WriteRaw(_scratch.Span, crc: null);

        var summaryCrc = new Crc32();
        var summaryStart = _position;
        var groups = new List<(McapOpcode Op, ulong Start, ulong Length)>();

        void Group(McapOpcode op, Action<RecordBuffer> write)
        {
            _scratch.Clear();
            write(_scratch);
            if (_scratch.Length == 0)
            {
                return;
            }

            groups.Add((op, _position, (ulong)_scratch.Length));
            WriteRaw(_scratch.Span, summaryCrc);
        }

        Group(McapOpcode.Schema, b =>
        {
            foreach (var s in _schemas)
            {
                AppendSchema(b, s);
            }
        });
        Group(McapOpcode.Channel, b =>
        {
            foreach (var c in _channels)
            {
                AppendChannel(b, c);
            }
        });
        Group(McapOpcode.Statistics, b =>
        {
            var at = b.BeginRecord(McapOpcode.Statistics);
            b.U64(_messageCount);
            b.U16((ushort)_schemas.Count);
            b.U32((uint)_channels.Count);
            b.U32(0);
            b.U32((uint)_metadataIndexes.Count);
            b.U32((uint)_chunkIndexes.Count);
            b.U64(_messageCount == 0 ? 0 : _messageStart);
            b.U64(_messageEnd);
            b.U32((uint)(_channelCounts.Count * 10));
            foreach (var (ch, count) in _channelCounts.OrderBy(kv => kv.Key))
            {
                b.U16(ch);
                b.U64(count);
            }

            b.EndRecord(at);
        });
        Group(McapOpcode.ChunkIndex, b =>
        {
            foreach (var ci in _chunkIndexes)
            {
                var at = b.BeginRecord(McapOpcode.ChunkIndex);
                b.U64(ci.MessageStartTime);
                b.U64(ci.MessageEndTime);
                b.U64(ci.ChunkStartOffset);
                b.U64(ci.ChunkLength);
                b.U32((uint)(ci.MessageIndexOffsets.Count * 10));
                foreach (var (ch, off) in ci.MessageIndexOffsets.OrderBy(kv => kv.Key))
                {
                    b.U16(ch);
                    b.U64(off);
                }

                b.U64(ci.MessageIndexLength);
                b.Str(ci.Compression);
                b.U64(ci.CompressedSize);
                b.U64(ci.UncompressedSize);
                b.EndRecord(at);
            }
        });
        Group(McapOpcode.MetadataIndex, b =>
        {
            foreach (var (offset, length, name) in _metadataIndexes)
            {
                var at = b.BeginRecord(McapOpcode.MetadataIndex);
                b.U64(offset);
                b.U64(length);
                b.Str(name);
                b.EndRecord(at);
            }
        });

        var summaryOffsetStart = _position;
        _scratch.Clear();
        foreach (var (op, start, length) in groups)
        {
            var at = _scratch.BeginRecord(McapOpcode.SummaryOffset);
            _scratch.U8((byte)op);
            _scratch.U64(start);
            _scratch.U64(length);
            _scratch.EndRecord(at);
        }

        WriteRaw(_scratch.Span, summaryCrc);

        // Footer: the CRC covers the summary section through the footer's summary_offset_start field.
        Span<byte> footer = stackalloc byte[1 + 8 + 8 + 8 + 4];
        footer[0] = (byte)McapOpcode.Footer;
        BinaryPrimitives.WriteUInt64LittleEndian(footer[1..], 20);
        BinaryPrimitives.WriteUInt64LittleEndian(footer[9..], summaryStart);
        BinaryPrimitives.WriteUInt64LittleEndian(footer[17..], summaryOffsetStart);
        summaryCrc.Append(footer[..25]);
        BinaryPrimitives.WriteUInt32LittleEndian(footer[25..], _options.ComputeCrc ? summaryCrc.GetCurrentHashAsUInt32() : 0u);
        WriteRaw(footer, crc: null);
        WriteRaw(McapConstants.Magic, crc: null);
        _out.Flush();
        _closed = true;
    }

    private static void AppendSchema(RecordBuffer b, McapSchema s)
    {
        var at = b.BeginRecord(McapOpcode.Schema);
        b.U16(s.Id);
        b.Str(s.Name);
        b.Str(s.Encoding);
        b.Bytes32(s.Data);
        b.EndRecord(at);
    }

    private static void AppendChannel(RecordBuffer b, McapChannel c)
    {
        var at = b.BeginRecord(McapOpcode.Channel);
        b.U16(c.Id);
        b.U16(c.SchemaId);
        b.Str(c.Topic);
        b.Str(c.MessageEncoding);
        b.StringMap(c.Metadata);
        b.EndRecord(at);
    }

    private static void WriteSchemaRecord(RecordBuffer b, McapSchema s)
    {
        b.Clear();
        AppendSchema(b, s);
    }

    private static void WriteChannelRecord(RecordBuffer b, McapChannel c)
    {
        b.Clear();
        AppendChannel(b, c);
    }

    private void WriteData(ReadOnlySpan<byte> bytes) => WriteRaw(bytes, _options.ComputeCrc ? _dataCrc : null);

    private void WriteRaw(ReadOnlySpan<byte> bytes, Crc32? crc)
    {
        _out.Write(bytes);
        crc?.Append(bytes);
        _position += (ulong)bytes.Length;
    }

    public void Dispose()
    {
        Close();
        if (!_leaveOpen)
        {
            _out.Dispose();
        }
    }
}
