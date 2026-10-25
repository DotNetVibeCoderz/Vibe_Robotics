using System.Buffers.Binary;
using System.IO.Hashing;

namespace ROS2.Net.Bag.Mcap;

/// <summary>
/// Reads MCAP files. When the file has a summary section, schemas, channels, statistics and chunk
/// indexes are loaded up front and time-range reads decompress only the overlapping chunks;
/// otherwise the file is scanned sequentially.
/// </summary>
public sealed class McapReader : IDisposable
{
    private readonly Stream _stream;
    private readonly bool _leaveOpen;
    private readonly Dictionary<ushort, McapSchema> _schemas = [];
    private readonly Dictionary<ushort, McapChannel> _channels = [];
    private readonly List<McapChunkIndex> _chunkIndexes = [];
    private readonly List<(ulong Offset, ulong Length, string Name)> _metadataIndexes = [];

    public McapReader(Stream stream, bool leaveOpen = false, bool validateCrc = true)
    {
        if (!stream.CanSeek)
        {
            throw new ArgumentException("MCAP reading requires a seekable stream.", nameof(stream));
        }

        _stream = stream;
        _leaveOpen = leaveOpen;
        ValidateCrc = validateCrc;

        Span<byte> magic = stackalloc byte[8];
        stream.Position = 0;
        stream.ReadExactly(magic);
        if (!magic.SequenceEqual(McapConstants.Magic))
        {
            throw new McapException("Not an MCAP file (bad leading magic).");
        }

        if (stream.Length >= 8 + 29 + 8)
        {
            stream.Position = stream.Length - 8;
            stream.ReadExactly(magic);
            if (magic.SequenceEqual(McapConstants.Magic))
            {
                ReadSummary();
            }
        }

        if (!HasSummary)
        {
            ScanForDefinitions();
        }
    }

    public static McapReader Open(string path, bool validateCrc = true) =>
        new(new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 16), leaveOpen: false, validateCrc);

    public bool ValidateCrc { get; }

    public bool HasSummary { get; private set; }

    public string Profile { get; private set; } = string.Empty;

    public string Library { get; private set; } = string.Empty;

    public IReadOnlyDictionary<ushort, McapSchema> Schemas => _schemas;

    public IReadOnlyDictionary<ushort, McapChannel> Channels => _channels;

    public IReadOnlyList<McapChunkIndex> ChunkIndexes => _chunkIndexes;

    public McapStatistics? Statistics { get; private set; }

    /// <summary>Number of chunks decompressed so far (useful to verify index-based seeking).</summary>
    public int ChunksRead { get; private set; }

    private void ReadSummary()
    {
        var footerAt = _stream.Length - 8 - 29;
        var footer = ReadAt(footerAt, 29);
        if (footer[0] != (byte)McapOpcode.Footer)
        {
            throw new McapException("Footer record not found.");
        }

        var summaryStart = BinaryPrimitives.ReadUInt64LittleEndian(footer.AsSpan(9));
        var summaryCrc = BinaryPrimitives.ReadUInt32LittleEndian(footer.AsSpan(25));
        ReadHeader();
        if (summaryStart == 0)
        {
            return;
        }

        var summaryLength = (int)(footerAt - (long)summaryStart);
        var summary = ReadAt((long)summaryStart, summaryLength);
        if (ValidateCrc && summaryCrc != 0)
        {
            var crc = new Crc32();
            crc.Append(summary);
            crc.Append(footer.AsSpan(0, 25));
            if (crc.GetCurrentHashAsUInt32() != summaryCrc)
            {
                throw new McapException("Summary CRC mismatch.");
            }
        }

        var pos = 0;
        while (pos < summary.Length)
        {
            var op = (McapOpcode)summary[pos];
            var len = (int)BinaryPrimitives.ReadUInt64LittleEndian(summary.AsSpan(pos + 1));
            var content = summary.AsSpan(pos + 9, len);
            pos += 9 + len;
            switch (op)
            {
                case McapOpcode.Schema: AddSchema(content); break;
                case McapOpcode.Channel: AddChannel(content); break;
                case McapOpcode.Statistics: Statistics = ParseStatistics(content); break;
                case McapOpcode.ChunkIndex: _chunkIndexes.Add(ParseChunkIndex(content)); break;
                case McapOpcode.MetadataIndex:
                {
                    var r = new RecordReader(content);
                    _metadataIndexes.Add((r.U64(), r.U64(), r.Str()));
                    break;
                }
            }
        }

        _chunkIndexes.Sort((a, b) => a.MessageStartTime.CompareTo(b.MessageStartTime));
        HasSummary = true;
    }

    private void ReadHeader()
    {
        _stream.Position = 8;
        var head = new byte[9];
        _stream.ReadExactly(head);
        if (head[0] != (byte)McapOpcode.Header)
        {
            throw new McapException("Header record missing.");
        }

        var content = new byte[BinaryPrimitives.ReadUInt64LittleEndian(head.AsSpan(1))];
        _stream.ReadExactly(content);
        var r = new RecordReader(content);
        Profile = r.Str();
        Library = r.Str();
    }

    private byte[] ReadAt(long offset, int length)
    {
        var buf = new byte[length];
        _stream.Position = offset;
        _stream.ReadExactly(buf);
        return buf;
    }

    private void AddSchema(ReadOnlySpan<byte> content)
    {
        var r = new RecordReader(content);
        var id = r.U16();
        var name = r.Str();
        var enc = r.Str();
        _schemas[id] = new McapSchema(id, name, enc, r.Bytes32().ToArray());
    }

    private void AddChannel(ReadOnlySpan<byte> content)
    {
        var r = new RecordReader(content);
        var id = r.U16();
        var schema = r.U16();
        var topic = r.Str();
        var enc = r.Str();
        _channels[id] = new McapChannel(id, schema, topic, enc, r.StringMap());
    }

    private static McapStatistics ParseStatistics(ReadOnlySpan<byte> content)
    {
        var r = new RecordReader(content);
        return new McapStatistics(r.U64(), r.U16(), r.U32(), r.U32(), r.U32(), r.U32(), r.U64(), r.U64(), r.U16U64Map());
    }

    private static McapChunkIndex ParseChunkIndex(ReadOnlySpan<byte> content)
    {
        var r = new RecordReader(content);
        return new McapChunkIndex(r.U64(), r.U64(), r.U64(), r.U64(), r.U16U64Map(), r.U64(), r.Str(), r.U64(), r.U64());
    }

    /// <summary>Iterates every top-level record of the data section as (opcode, content).</summary>
    private IEnumerable<(McapOpcode Op, byte[] Content)> DataRecords()
    {
        _stream.Position = 8;
        var head = new byte[9];
        var crc = ValidateCrc ? new Crc32() : null;
        crc?.Append(McapConstants.Magic);
        while (true)
        {
            var read = _stream.ReadAtLeast(head, 9, throwOnEndOfStream: false);
            if (read < 9)
            {
                yield break;
            }

            var op = (McapOpcode)head[0];
            var len = BinaryPrimitives.ReadUInt64LittleEndian(head.AsSpan(1));
            if (op == McapOpcode.Footer || (long)len > _stream.Length - _stream.Position)
            {
                yield break;
            }

            var content = new byte[len];
            _stream.ReadExactly(content);
            if (op == McapOpcode.DataEnd)
            {
                var expected = BinaryPrimitives.ReadUInt32LittleEndian(content);
                if (crc is not null && expected != 0 && crc.GetCurrentHashAsUInt32() != expected)
                {
                    throw new McapException("Data section CRC mismatch.");
                }

                yield break;
            }

            crc?.Append(head);
            crc?.Append(content);
            var resume = _stream.Position;
            yield return (op, content);
            _stream.Position = resume;
        }
    }

    private void ScanForDefinitions()
    {
        ReadHeader();
        foreach (var (op, content) in DataRecords())
        {
            switch (op)
            {
                case McapOpcode.Schema: AddSchema(content); break;
                case McapOpcode.Channel: AddChannel(content); break;
                case McapOpcode.Chunk:
                    foreach (var (iop, ic) in ChunkRecords(content))
                    {
                        if (iop == McapOpcode.Schema)
                        {
                            AddSchema(ic.Span);
                        }
                        else if (iop == McapOpcode.Channel)
                        {
                            AddChannel(ic.Span);
                        }
                    }

                    break;
            }
        }
    }

    /// <summary>Verifies the data-section CRC (and every chunk CRC) by reading the whole file.</summary>
    public void Verify()
    {
        foreach (var (op, content) in DataRecords())
        {
            if (op == McapOpcode.Chunk)
            {
                _ = ChunkRecords(content).Count();
            }
        }
    }

    /// <summary>Metadata records.</summary>
    public IEnumerable<McapMetadata> ReadMetadata()
    {
        if (HasSummary)
        {
            foreach (var (offset, length, _) in _metadataIndexes)
            {
                var rec = ReadAt((long)offset, (int)length);
                var r = new RecordReader(rec.AsSpan(9));
                yield return new McapMetadata(r.Str(), r.StringMap());
            }

            yield break;
        }

        foreach (var (op, content) in DataRecords())
        {
            if (op == McapOpcode.Metadata)
            {
                var r = new RecordReader(content);
                yield return new McapMetadata(r.Str(), r.StringMap());
            }
        }
    }

    /// <summary>
    /// Messages with <c>startTime &lt;= log_time &lt;= endTime</c>, optionally restricted to channels,
    /// in log-time order when the file is indexed (file order otherwise).
    /// </summary>
    public IEnumerable<McapMessage> ReadMessages(ulong startTime = 0, ulong endTime = ulong.MaxValue, ISet<ushort>? channels = null)
    {
        bool Want(in McapMessage m) => m.LogTime >= startTime && m.LogTime <= endTime && (channels is null || channels.Contains(m.ChannelId));

        if (!HasSummary || _chunkIndexes.Count == 0)
        {
            foreach (var (op, content) in DataRecords())
            {
                if (op == McapOpcode.Message)
                {
                    var m = ParseMessage(content);
                    if (Want(m))
                    {
                        yield return m;
                    }
                }
                else if (op == McapOpcode.Chunk)
                {
                    var r = new RecordReader(content);
                    var cs = r.U64();
                    var ce = r.U64();
                    if (ce < startTime || cs > endTime)
                    {
                        continue;
                    }

                    foreach (var m in MessagesOf(content))
                    {
                        if (Want(m))
                        {
                            yield return m;
                        }
                    }
                }
            }

            yield break;
        }

        // Indexed path: merge overlapping chunks in log-time order.
        var heap = new PriorityQueue<McapMessage, (ulong, long)>();
        long order = 0;
        var selected = _chunkIndexes
            .Where(c => c.MessageEndTime >= startTime && c.MessageStartTime <= endTime)
            .Where(c => channels is null || c.MessageIndexOffsets.Keys.Any(channels.Contains))
            .ToList();
        for (var i = 0; i < selected.Count; i++)
        {
            var chunk = selected[i];
            var record = ReadAt((long)chunk.ChunkStartOffset, (int)chunk.ChunkLength);
            foreach (var m in MessagesOf(record.AsMemory(9)))
            {
                if (Want(m))
                {
                    heap.Enqueue(m, (m.LogTime, order++));
                }
            }

            var nextStart = i + 1 < selected.Count ? selected[i + 1].MessageStartTime : ulong.MaxValue;
            while (heap.TryPeek(out var top, out var key) && key.Item1 < nextStart)
            {
                heap.Dequeue();
                yield return top;
            }
        }

        while (heap.TryDequeue(out var m, out _))
        {
            yield return m;
        }
    }

    private List<McapMessage> MessagesOf(ReadOnlyMemory<byte> chunkContent)
    {
        var list = new List<McapMessage>();
        foreach (var (op, content) in ChunkRecords(chunkContent))
        {
            if (op == McapOpcode.Message)
            {
                list.Add(ParseMessage(content));
            }
        }

        return list;
    }

    private static McapMessage ParseMessage(ReadOnlyMemory<byte> content)
    {
        var r = new RecordReader(content.Span);
        var channel = r.U16();
        var seq = r.U32();
        var log = r.U64();
        var pub = r.U64();
        return new McapMessage(channel, seq, log, pub, content[r.Position..]);
    }

    private List<(McapOpcode Op, ReadOnlyMemory<byte> Content)> ChunkRecords(ReadOnlyMemory<byte> chunkContent)
    {
        ReadOnlyMemory<byte> records;
        {
            var r = new RecordReader(chunkContent.Span);
            r.U64();
            r.U64();
            var uncompressedSize = r.U64();
            var crc = r.U32();
            var compression = r.Str();
            var size = (int)r.U64();
            var start = r.Position;
            var data = McapCompressionCodec.Decompress(compression, chunkContent.Span.Slice(start, size), uncompressedSize);
            if (ValidateCrc && crc != 0 && Crc32.HashToUInt32(data) != crc)
            {
                throw new McapException("Chunk CRC mismatch.");
            }

            records = data;
        }

        ChunksRead++;
        var list = new List<(McapOpcode, ReadOnlyMemory<byte>)>();
        var span = records.Span;
        var pos = 0;
        while (pos + 9 <= span.Length)
        {
            var op = (McapOpcode)span[pos];
            var len = (int)BinaryPrimitives.ReadUInt64LittleEndian(span[(pos + 1)..]);
            if (pos + 9 + len > span.Length)
            {
                throw new McapException("Chunk record truncated.");
            }

            list.Add((op, records.Slice(pos + 9, len)));
            pos += 9 + len;
        }

        return list;
    }

    public void Dispose()
    {
        if (!_leaveOpen)
        {
            _stream.Dispose();
        }
    }
}
