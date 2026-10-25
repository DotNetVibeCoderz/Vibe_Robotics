using System.Collections.Concurrent;
using ROS2.Net.Bag.Mcap;
using ROS2.Net.Serialization;

namespace ROS2.Net.Bag;

/// <summary>A recorded message. <see cref="Data"/> is CDR with its encapsulation header.</summary>
public readonly record struct BagMessage(string Topic, string Type, RosTime LogTime, RosTime PublishTime, uint Sequence, ReadOnlyMemory<byte> Data);

/// <summary>
/// Reads a rosbag2 directory (or a single <c>.mcap</c> file): metadata, topics, and messages in
/// time order, as raw CDR, typed messages or JSON.
/// </summary>
public sealed class BagReader : IDisposable
{
    private readonly List<(string Path, McapReader Reader)> _files = [];
    private readonly ConcurrentDictionary<string, DynamicMessageType?> _dynamic = new(StringComparer.Ordinal);
    private readonly Dictionary<string, string> _schemaText = new(StringComparer.Ordinal);

    private BagReader(string path, BagMetadata metadata, IEnumerable<string> files)
    {
        Path = path;
        Metadata = metadata;
        foreach (var f in files)
        {
            _files.Add((f, McapReader.Open(f)));
        }

        foreach (var (_, r) in _files)
        {
            foreach (var ch in r.Channels.Values)
            {
                if (r.Schemas.TryGetValue(ch.SchemaId, out var schema) && schema.Data.Length > 0)
                {
                    _schemaText.TryAdd(schema.Name, schema.DataText);
                }
            }
        }
    }

    /// <summary>The bag directory or file.</summary>
    public string Path { get; }

    public BagMetadata Metadata { get; }

    public IReadOnlyList<BagTopicInfo> Topics => Metadata.Topics;

    /// <summary>Storage files in playback order.</summary>
    public IReadOnlyList<string> Files => _files.Select(f => f.Path).ToList();

    /// <summary>Opens a bag directory (with or without <c>metadata.yaml</c>) or an <c>.mcap</c> file.</summary>
    public static BagReader Open(string path)
    {
        if (File.Exists(path))
        {
            return new BagReader(path, FromMcap([path]), [path]);
        }

        if (!Directory.Exists(path))
        {
            throw new FileNotFoundException($"No bag at '{path}'.");
        }

        if (File.Exists(System.IO.Path.Combine(path, BagMetadata.FileName)))
        {
            var meta = BagMetadata.Load(path);
            var files = meta.Files.Select(f => System.IO.Path.Combine(path, f.Path)).Where(File.Exists).ToList();
            return new BagReader(path, meta, files);
        }

        var mcaps = Directory.GetFiles(path, "*.mcap").Order(StringComparer.Ordinal).ToList();
        if (mcaps.Count == 0)
        {
            throw new FileNotFoundException($"'{path}' contains neither metadata.yaml nor .mcap files.");
        }

        return new BagReader(path, FromMcap(mcaps), mcaps);
    }

    /// <summary>Rebuilds rosbag2 metadata from MCAP summaries (for bags without <c>metadata.yaml</c>).</summary>
    private static BagMetadata FromMcap(IReadOnlyList<string> files)
    {
        var meta = new BagMetadata();
        var topics = new Dictionary<string, (string Type, IReadOnlyList<QosProfile> Qos, ulong Count)>(StringComparer.Ordinal);
        long start = long.MaxValue, end = 0;
        foreach (var file in files)
        {
            using var r = McapReader.Open(file);
            ulong count = 0;
            long fs = long.MaxValue, fe = 0;
            var counts = r.Statistics?.ChannelMessageCounts;
            if (counts is null)
            {
                // Unindexed file: count by scanning.
                var tally = new Dictionary<ushort, ulong>();
                foreach (var m in r.ReadMessages())
                {
                    tally[m.ChannelId] = tally.GetValueOrDefault(m.ChannelId) + 1;
                    fs = Math.Min(fs, (long)m.LogTime);
                    fe = Math.Max(fe, (long)m.LogTime);
                }

                counts = tally;
            }
            else if (r.Statistics!.MessageCount > 0)
            {
                fs = (long)r.Statistics.MessageStartTime;
                fe = (long)r.Statistics.MessageEndTime;
            }

            foreach (var ch in r.Channels.Values)
            {
                var type = r.Schemas.TryGetValue(ch.SchemaId, out var s) ? s.Name : string.Empty;
                var qos = QosYaml.Parse(ch.Metadata.GetValueOrDefault("offered_qos_profiles"));
                var c = counts.GetValueOrDefault(ch.Id);
                count += c;
                topics[ch.Topic] = topics.TryGetValue(ch.Topic, out var prev) ? (type, qos, prev.Count + c) : (type, qos, c);
            }

            if (count > 0)
            {
                start = Math.Min(start, fs);
                end = Math.Max(end, fe);
            }

            meta.Files.Add(new BagFileInfo(System.IO.Path.GetFileName(file), new RosTime(count > 0 ? fs : 0), new RosDuration(count > 0 ? fe - fs : 0), count));
            meta.MessageCount += count;
        }

        meta.StartingTime = new RosTime(meta.MessageCount > 0 ? start : 0);
        meta.Duration = new RosDuration(meta.MessageCount > 0 ? end - start : 0);
        foreach (var (name, t) in topics.OrderBy(kv => kv.Key, StringComparer.Ordinal))
        {
            meta.Topics.Add(new BagTopicInfo(name, t.Type, "cdr", t.Qos, t.Count));
        }

        return meta;
    }

    /// <summary>Messages in log-time order, optionally limited to a time range and topics.</summary>
    public IEnumerable<BagMessage> ReadMessages(RosTime? start = null, RosTime? end = null, IEnumerable<string>? topics = null)
    {
        var topicSet = topics is null ? null : new HashSet<string>(topics, StringComparer.Ordinal);
        var s = (ulong)Math.Max(0, start?.Nanoseconds ?? 0);
        var e = end is { } en ? (ulong)Math.Max(0, en.Nanoseconds) : ulong.MaxValue;
        foreach (var (_, reader) in _files)
        {
            HashSet<ushort>? channels = null;
            if (topicSet is not null)
            {
                channels = reader.Channels.Values.Where(c => topicSet.Contains(c.Topic)).Select(c => c.Id).ToHashSet();
                if (channels.Count == 0)
                {
                    continue;
                }
            }

            foreach (var m in reader.ReadMessages(s, e, channels))
            {
                var ch = reader.Channels[m.ChannelId];
                var type = reader.Schemas.TryGetValue(ch.SchemaId, out var schema) ? schema.Name : string.Empty;
                yield return new BagMessage(ch.Topic, type, new RosTime((long)m.LogTime), new RosTime((long)m.PublishTime), m.Sequence, m.Data);
            }
        }
    }

    /// <summary>Decodes a message into its generated type.</summary>
    public static T Deserialize<T>(BagMessage message)
        where T : class, IRosMessage, new() => CdrReader.Deserialize<T>(message.Data.Span);

    /// <summary>Decodes a message through <see cref="RosTypeRegistry"/>; null when its type is not loaded.</summary>
    public static IRosMessage? Deserialize(BagMessage message)
    {
        var msg = RosTypeRegistry.CreateMessage(message.Type);
        if (msg is null)
        {
            return null;
        }

        var reader = new CdrReader(message.Data.Span);
        msg.Deserialize(ref reader);
        return msg;
    }

    /// <summary>
    /// Decodes a message to rosbridge-style JSON using the schema stored in the bag, so no generated
    /// code is needed (falls back to the registry's definition).
    /// </summary>
    public string ToJson(BagMessage message, bool indented = false)
    {
        var type = _dynamic.GetOrAdd(message.Type, t =>
        {
            try
            {
                if (_schemaText.TryGetValue(t, out var text) && text.Length > 0)
                {
                    return DynamicMessageType.Parse(t, text);
                }

                return DynamicMessageType.TryFromRegistry(t, out var dyn) ? dyn : null;
            }
            catch (Exception ex) when (ex is Rosidl.RosidlParseException or ArgumentException or InvalidOperationException)
            {
                return null;
            }
        });

        if (type is null)
        {
            throw new InvalidOperationException($"No definition is available for type '{message.Type}'.");
        }

        return type.ToJson(message.Data.Span, indented);
    }

    public void Dispose()
    {
        foreach (var (_, r) in _files)
        {
            r.Dispose();
        }
    }
}
