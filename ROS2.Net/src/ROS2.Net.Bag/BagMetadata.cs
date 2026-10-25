using System.Globalization;
using System.Text;
using YamlDotNet.RepresentationModel;

namespace ROS2.Net.Bag;

/// <summary>A topic recorded in a bag.</summary>
public sealed record BagTopicInfo(string Name, string Type, string SerializationFormat, IReadOnlyList<QosProfile> OfferedQos, ulong MessageCount)
{
    /// <summary>The QoS to use when playing this topic back.</summary>
    public QosProfile PlaybackQos => QosYaml.Adapt(OfferedQos);
}

/// <summary>One storage file of a bag.</summary>
public sealed record BagFileInfo(string Path, RosTime StartingTime, RosDuration Duration, ulong MessageCount);

/// <summary>The contents of a rosbag2 <c>metadata.yaml</c> (format version 5).</summary>
public sealed class BagMetadata
{
    public const string FileName = "metadata.yaml";

    public int Version { get; set; } = 5;

    public string StorageIdentifier { get; set; } = "mcap";

    public RosTime StartingTime { get; set; }

    public RosDuration Duration { get; set; }

    public ulong MessageCount { get; set; }

    public List<BagTopicInfo> Topics { get; } = [];

    public List<BagFileInfo> Files { get; } = [];

    public string CompressionFormat { get; set; } = string.Empty;

    public string CompressionMode { get; set; } = string.Empty;

    public string RosDistro { get; set; } = "ros2net";

    public RosTime EndTime => StartingTime + Duration;

    public string ToYaml()
    {
        var inv = CultureInfo.InvariantCulture;
        var sb = new StringBuilder();
        sb.Append("rosbag2_bagfile_information:\n");
        sb.Append("  version: ").Append(Version.ToString(inv)).Append('\n');
        sb.Append("  storage_identifier: ").Append(StorageIdentifier).Append('\n');
        sb.Append("  duration:\n    nanoseconds: ").Append(Duration.Nanoseconds.ToString(inv)).Append('\n');
        sb.Append("  starting_time:\n    nanoseconds_since_epoch: ").Append(StartingTime.Nanoseconds.ToString(inv)).Append('\n');
        sb.Append("  message_count: ").Append(MessageCount.ToString(inv)).Append('\n');
        sb.Append("  topics_with_message_count:\n");
        foreach (var t in Topics)
        {
            sb.Append("    - topic_metadata:\n");
            sb.Append("        name: ").Append(t.Name).Append('\n');
            sb.Append("        type: ").Append(t.Type).Append('\n');
            sb.Append("        serialization_format: ").Append(t.SerializationFormat).Append('\n');
            sb.Append("        offered_qos_profiles: ").Append(Quote(QosYaml.Serialize(t.OfferedQos))).Append('\n');
            sb.Append("      message_count: ").Append(t.MessageCount.ToString(inv)).Append('\n');
        }

        sb.Append("  compression_format: ").Append(Quote(CompressionFormat)).Append('\n');
        sb.Append("  compression_mode: ").Append(Quote(CompressionMode)).Append('\n');
        sb.Append("  relative_file_paths:\n");
        foreach (var f in Files)
        {
            sb.Append("    - ").Append(f.Path).Append('\n');
        }

        sb.Append("  files:\n");
        foreach (var f in Files)
        {
            sb.Append("    - path: ").Append(f.Path).Append('\n');
            sb.Append("      starting_time:\n        nanoseconds_since_epoch: ").Append(f.StartingTime.Nanoseconds.ToString(inv)).Append('\n');
            sb.Append("      duration:\n        nanoseconds: ").Append(f.Duration.Nanoseconds.ToString(inv)).Append('\n');
            sb.Append("      message_count: ").Append(f.MessageCount.ToString(inv)).Append('\n');
        }

        sb.Append("  custom_data: ~\n");
        sb.Append("  ros_distro: ").Append(RosDistro).Append('\n');
        return sb.ToString();
    }

    private static string Quote(string s) =>
        "\"" + s.Replace("\\", "\\\\").Replace("\"", "\\\"").Replace("\n", "\\n") + "\"";

    public void Save(string bagDirectory) => File.WriteAllText(Path.Combine(bagDirectory, FileName), ToYaml());

    public static BagMetadata Load(string bagDirectory) => Parse(File.ReadAllText(Path.Combine(bagDirectory, FileName)));

    public static BagMetadata Parse(string yaml)
    {
        var stream = new YamlStream();
        stream.Load(new StringReader(yaml));
        var root = (YamlMappingNode)stream.Documents[0].RootNode;
        var info = (YamlMappingNode)root.Children[new YamlScalarNode("rosbag2_bagfile_information")];
        var meta = new BagMetadata();

        string? S(YamlMappingNode m, string key) => m.Children.TryGetValue(new YamlScalarNode(key), out var v) && v is YamlScalarNode s ? s.Value : null;
        long L(YamlMappingNode m, string key) => long.TryParse(S(m, key), NumberStyles.Integer, CultureInfo.InvariantCulture, out var v) ? v : 0;
        YamlMappingNode? M(YamlMappingNode m, string key) => m.Children.TryGetValue(new YamlScalarNode(key), out var v) ? v as YamlMappingNode : null;

        meta.Version = (int)L(info, "version");
        meta.StorageIdentifier = S(info, "storage_identifier") ?? "mcap";
        meta.Duration = new RosDuration(M(info, "duration") is { } d ? L(d, "nanoseconds") : 0);
        meta.StartingTime = new RosTime(M(info, "starting_time") is { } st ? L(st, "nanoseconds_since_epoch") : 0);
        meta.MessageCount = (ulong)L(info, "message_count");
        meta.CompressionFormat = S(info, "compression_format") ?? string.Empty;
        meta.CompressionMode = S(info, "compression_mode") ?? string.Empty;
        meta.RosDistro = S(info, "ros_distro") ?? string.Empty;

        if (info.Children.TryGetValue(new YamlScalarNode("topics_with_message_count"), out var topics) && topics is YamlSequenceNode ts)
        {
            foreach (var t in ts.Children.OfType<YamlMappingNode>())
            {
                var md = M(t, "topic_metadata")!;
                meta.Topics.Add(new BagTopicInfo(
                    S(md, "name") ?? string.Empty,
                    S(md, "type") ?? string.Empty,
                    S(md, "serialization_format") ?? "cdr",
                    QosYaml.Parse(S(md, "offered_qos_profiles")),
                    (ulong)L(t, "message_count")));
            }
        }

        if (info.Children.TryGetValue(new YamlScalarNode("files"), out var files) && files is YamlSequenceNode fs)
        {
            foreach (var f in fs.Children.OfType<YamlMappingNode>())
            {
                meta.Files.Add(new BagFileInfo(
                    S(f, "path") ?? string.Empty,
                    new RosTime(M(f, "starting_time") is { } s ? L(s, "nanoseconds_since_epoch") : 0),
                    new RosDuration(M(f, "duration") is { } du ? L(du, "nanoseconds") : 0),
                    (ulong)L(f, "message_count")));
            }
        }
        else if (info.Children.TryGetValue(new YamlScalarNode("relative_file_paths"), out var rel) && rel is YamlSequenceNode rs)
        {
            foreach (var p in rs.Children.OfType<YamlScalarNode>())
            {
                meta.Files.Add(new BagFileInfo(p.Value ?? string.Empty, default, default, 0));
            }
        }

        return meta;
    }
}
