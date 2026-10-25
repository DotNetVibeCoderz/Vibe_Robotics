using System.Text;
using ROS2.Net.Bag.Mcap;

namespace ROS2.Net.Bag;

/// <summary>Options for <see cref="BagWriter"/> and <see cref="BagRecorder"/>.</summary>
public sealed class BagWriterOptions
{
    public McapCompression Compression { get; init; } = McapCompression.None;

    public int ChunkSize { get; init; } = 1024 * 1024;

    /// <summary>Start a new storage file once the current one exceeds this many bytes (0 = never).</summary>
    public long MaxFileSize { get; init; }

    /// <summary>Start a new storage file once the current one spans this much message time (null = never).</summary>
    public TimeSpan? MaxFileDuration { get; init; }

    /// <summary>Replace an existing bag directory.</summary>
    public bool Overwrite { get; init; }
}

/// <summary>
/// Writes a rosbag2 directory: <c>&lt;name&gt;_0.mcap</c>, <c>&lt;name&gt;_1.mcap</c>, ... plus
/// <c>metadata.yaml</c>. Thread-safe.
/// </summary>
public sealed class BagWriter : IDisposable
{
    private readonly object _gate = new();
    private readonly BagWriterOptions _options;
    private readonly string _baseName;
    private readonly Dictionary<string, TopicState> _topics = new(StringComparer.Ordinal);
    private readonly BagMetadata _metadata = new();
    private McapWriter? _writer;
    private int _fileIndex = -1;
    private long _fileStart = long.MaxValue;
    private long _fileEnd;
    private ulong _fileCount;
    private long _bagStart = long.MaxValue;
    private long _bagEnd;
    private ulong _total;
    private bool _closed;

    private sealed class TopicState
    {
        public required string Name { get; init; }
        public required string Type { get; init; }
        public required IReadOnlyList<QosProfile> Qos { get; init; }
        public ushort Channel { get; set; }
        public uint Sequence { get; set; }
        public ulong Count { get; set; }
    }

    public BagWriter(string directory, BagWriterOptions? options = null)
    {
        _options = options ?? new BagWriterOptions();
        Directory = Path.GetFullPath(directory);
        if (System.IO.Directory.Exists(Directory) && System.IO.Directory.EnumerateFileSystemEntries(Directory).Any())
        {
            if (!_options.Overwrite)
            {
                throw new IOException($"Bag directory '{Directory}' already exists and is not empty.");
            }

            System.IO.Directory.Delete(Directory, recursive: true);
        }

        System.IO.Directory.CreateDirectory(Directory);
        _baseName = Path.GetFileName(Directory.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
        // MCAP chunk compression is transparent to rosbag2, so the bag-level compression fields stay empty
        // (setting them would make `ros2 bag` try to decompress every message a second time).
        OpenNextFile();
    }

    public string Directory { get; }

    public ulong MessageCount
    {
        get
        {
            lock (_gate)
            {
                return _total;
            }
        }
    }

    /// <summary>Declares a topic. The schema comes from <see cref="RosTypeRegistry"/> when the type is known.</summary>
    public void AddTopic(string topic, string type, IReadOnlyList<QosProfile>? offeredQos = null)
    {
        lock (_gate)
        {
            if (_topics.ContainsKey(topic))
            {
                return;
            }

            var state = new TopicState { Name = topic, Type = type, Qos = offeredQos ?? [QosProfile.Default] };
            state.Channel = AddChannel(_writer!, state);
            _topics[topic] = state;
        }
    }

    public bool HasTopic(string topic)
    {
        lock (_gate)
        {
            return _topics.ContainsKey(topic);
        }
    }

    /// <summary>Appends one CDR message (encapsulation header included).</summary>
    public void Write(string topic, ReadOnlySpan<byte> cdr, RosTime receiveTime, RosTime? publishTime = null)
    {
        lock (_gate)
        {
            ObjectDisposedException.ThrowIf(_closed, this);
            if (!_topics.TryGetValue(topic, out var state))
            {
                throw new InvalidOperationException($"Topic '{topic}' was not added to the bag.");
            }

            var t = receiveTime.Nanoseconds;
            if (_fileCount > 0 && NeedsSplit(t))
            {
                OpenNextFile();
            }

            _writer!.WriteMessage(state.Channel, state.Sequence++, (ulong)t, (ulong)(publishTime ?? receiveTime).Nanoseconds, cdr);
            state.Count++;
            _total++;
            _fileCount++;
            _fileStart = Math.Min(_fileStart, t);
            _fileEnd = Math.Max(_fileEnd, t);
            _bagStart = Math.Min(_bagStart, t);
            _bagEnd = Math.Max(_bagEnd, t);
        }
    }

    /// <summary>Serializes and appends a typed message.</summary>
    public void Write<T>(string topic, T message, RosTime receiveTime)
        where T : class, IRosMessage<T>, new()
    {
        if (!HasTopic(topic))
        {
            AddTopic(topic, T.TypeName);
        }

        Write(topic, Serialization.CdrWriter.Serialize(message), receiveTime);
    }

    private bool NeedsSplit(long t) =>
        (_options.MaxFileSize > 0 && _writer!.Position >= _options.MaxFileSize) ||
        (_options.MaxFileDuration is { } d && t - _fileStart >= d.Ticks * 100);

    private void OpenNextFile()
    {
        CloseCurrentFile();
        _fileIndex++;
        var fileName = $"{_baseName}_{_fileIndex}.mcap";
        var stream = new FileStream(Path.Combine(Directory, fileName), FileMode.Create, FileAccess.Write, FileShare.Read, 1 << 16);
        _writer = new McapWriter(stream, new McapWriterOptions
        {
            Compression = _options.Compression,
            ChunkSize = _options.ChunkSize,
            Library = "ROS2.Net.Bag (Gravicode Studios)",
        });
        _metadata.Files.Add(new BagFileInfo(fileName, default, default, 0));
        foreach (var state in _topics.Values)
        {
            state.Channel = AddChannel(_writer, state);
        }
    }

    private static ushort AddChannel(McapWriter writer, TopicState state)
    {
        ushort schemaId = 0;
        var definition = RosTypeRegistry.GetFullDefinition(state.Type);
        var schemaName = RosTypeRegistry.TryGetMessage(state.Type, out var info) ? info.Name : state.Type;
        if (definition.Length > 0 || schemaName.Length > 0)
        {
            schemaId = writer.AddSchema(schemaName, "ros2msg", Encoding.UTF8.GetBytes(definition));
        }

        return writer.AddChannel(schemaId, state.Name, "cdr", new Dictionary<string, string>
        {
            ["offered_qos_profiles"] = QosYaml.Serialize(state.Qos),
            ["topic_type_hash"] = string.Empty,
        });
    }

    private void CloseCurrentFile()
    {
        if (_writer is null)
        {
            return;
        }

        _writer.Dispose();
        var last = _metadata.Files[^1];
        _metadata.Files[^1] = last with
        {
            StartingTime = new RosTime(_fileCount == 0 ? 0 : _fileStart),
            Duration = new RosDuration(_fileCount == 0 ? 0 : _fileEnd - _fileStart),
            MessageCount = _fileCount,
        };
        _writer = null;
        _fileCount = 0;
        _fileStart = long.MaxValue;
        _fileEnd = 0;
    }

    /// <summary>Closes the storage file and writes <c>metadata.yaml</c>.</summary>
    public void Close()
    {
        lock (_gate)
        {
            if (_closed)
            {
                return;
            }

            CloseCurrentFile();
            _metadata.StartingTime = new RosTime(_total == 0 ? 0 : _bagStart);
            _metadata.Duration = new RosDuration(_total == 0 ? 0 : _bagEnd - _bagStart);
            _metadata.MessageCount = _total;
            _metadata.Topics.Clear();
            foreach (var t in _topics.Values.OrderBy(t => t.Name, StringComparer.Ordinal))
            {
                _metadata.Topics.Add(new BagTopicInfo(t.Name, t.Type, "cdr", t.Qos, t.Count));
            }

            _metadata.Save(Directory);
            _closed = true;
        }
    }

    public void Dispose() => Close();
}
