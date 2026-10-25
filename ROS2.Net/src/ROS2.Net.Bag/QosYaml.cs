using System.Globalization;
using System.Text;
using YamlDotNet.RepresentationModel;

namespace ROS2.Net.Bag;

/// <summary>
/// The <c>offered_qos_profiles</c> YAML that rosbag2 stores per topic. Written in the integer form
/// rosbag2 Humble uses; read in both the integer and the Iron+/Jazzy string form.
/// </summary>
public static class QosYaml
{
    private const int InfiniteSec = int.MaxValue;
    private const uint InfiniteNsec = uint.MaxValue;

    public static string Serialize(IEnumerable<QosProfile> profiles)
    {
        var sb = new StringBuilder();
        foreach (var p in profiles)
        {
            sb.Append("- history: ").Append((int)HistoryCode(p.History)).Append('\n');
            sb.Append("  depth: ").Append(p.Depth.ToString(CultureInfo.InvariantCulture)).Append('\n');
            sb.Append("  reliability: ").Append(ReliabilityCode(p.Reliability)).Append('\n');
            sb.Append("  durability: ").Append(DurabilityCode(p.Durability)).Append('\n');
            AppendDuration(sb, "deadline", p.Deadline);
            AppendDuration(sb, "lifespan", p.Lifespan);
            sb.Append("  liveliness: ").Append(LivelinessCode(p.Liveliness)).Append('\n');
            AppendDuration(sb, "liveliness_lease_duration", p.LivelinessLeaseDuration);
            sb.Append("  avoid_ros_namespace_conventions: ").Append(p.AvoidRosNamespaceConventions ? "true" : "false").Append('\n');
        }

        return sb.ToString();
    }

    public static IReadOnlyList<QosProfile> Parse(string? yaml)
    {
        if (string.IsNullOrWhiteSpace(yaml))
        {
            return [];
        }

        var stream = new YamlStream();
        stream.Load(new StringReader(yaml));
        if (stream.Documents.Count == 0 || stream.Documents[0].RootNode is not YamlSequenceNode seq)
        {
            return [];
        }

        var list = new List<QosProfile>();
        foreach (var item in seq.Children.OfType<YamlMappingNode>())
        {
            string? Get(string key) => item.Children.TryGetValue(new YamlScalarNode(key), out var v) && v is YamlScalarNode s ? s.Value : null;
            TimeSpan? Dur(string key)
            {
                if (!item.Children.TryGetValue(new YamlScalarNode(key), out var v) || v is not YamlMappingNode m)
                {
                    return null;
                }

                long sec = 0, nsec = 0;
                foreach (var (k, val) in m.Children)
                {
                    var name = ((YamlScalarNode)k).Value;
                    var text = ((YamlScalarNode)val).Value ?? "0";
                    if (name == "sec")
                    {
                        sec = long.Parse(text, CultureInfo.InvariantCulture);
                    }
                    else if (name is "nsec" or "nanosec")
                    {
                        nsec = long.Parse(text, CultureInfo.InvariantCulture);
                    }
                }

                if ((sec == InfiniteSec && nsec == InfiniteNsec) || (sec == 0 && nsec == 0))
                {
                    return null;
                }

                return TimeSpan.FromTicks((sec * TimeSpan.TicksPerSecond) + (nsec / 100));
            }

            list.Add(new QosProfile(int.TryParse(Get("depth"), out var d) ? d : 10)
            {
                History = Get("history") switch
                {
                    "1" or "keep_last" => HistoryPolicy.KeepLast,
                    "2" or "keep_all" => HistoryPolicy.KeepAll,
                    _ => HistoryPolicy.SystemDefault,
                },
                Reliability = Get("reliability") switch
                {
                    "1" or "reliable" => ReliabilityPolicy.Reliable,
                    "2" or "best_effort" => ReliabilityPolicy.BestEffort,
                    _ => ReliabilityPolicy.SystemDefault,
                },
                Durability = Get("durability") switch
                {
                    "1" or "transient_local" => DurabilityPolicy.TransientLocal,
                    "2" or "volatile" => DurabilityPolicy.Volatile,
                    _ => DurabilityPolicy.SystemDefault,
                },
                Liveliness = Get("liveliness") switch
                {
                    "1" or "automatic" => LivelinessPolicy.Automatic,
                    "3" or "manual_by_topic" => LivelinessPolicy.ManualByTopic,
                    _ => LivelinessPolicy.SystemDefault,
                },
                Deadline = Dur("deadline"),
                Lifespan = Dur("lifespan"),
                LivelinessLeaseDuration = Dur("liveliness_lease_duration"),
                AvoidRosNamespaceConventions = Get("avoid_ros_namespace_conventions") == "true",
            });
        }

        return list;
    }

    /// <summary>
    /// The QoS a recorder should request to receive from every offered profile, and the QoS a player
    /// should offer: reliable/transient-local only when every publisher offers it (rosbag2's adaptation).
    /// </summary>
    public static QosProfile Adapt(IReadOnlyList<QosProfile> offered, int depth = 100)
    {
        var qos = QosProfile.Default.KeepLast(depth);
        if (offered.Count == 0)
        {
            return qos;
        }

        var allReliable = offered.All(p => p.Resolve().Reliability == ReliabilityPolicy.Reliable);
        var allLatched = offered.All(p => p.Resolve().Durability == DurabilityPolicy.TransientLocal);
        return qos with
        {
            Reliability = allReliable ? ReliabilityPolicy.Reliable : ReliabilityPolicy.BestEffort,
            Durability = allLatched ? DurabilityPolicy.TransientLocal : DurabilityPolicy.Volatile,
        };
    }

    private static int HistoryCode(HistoryPolicy h) => h switch { HistoryPolicy.KeepLast => 1, HistoryPolicy.KeepAll => 2, _ => 3 };

    private static int ReliabilityCode(ReliabilityPolicy r) => r switch { ReliabilityPolicy.Reliable => 1, ReliabilityPolicy.BestEffort => 2, _ => 0 };

    private static int DurabilityCode(DurabilityPolicy d) => d switch { DurabilityPolicy.TransientLocal => 1, DurabilityPolicy.Volatile => 2, _ => 0 };

    private static int LivelinessCode(LivelinessPolicy l) => l switch { LivelinessPolicy.Automatic => 1, LivelinessPolicy.ManualByTopic => 3, _ => 0 };

    private static void AppendDuration(StringBuilder sb, string name, TimeSpan? d)
    {
        long sec = InfiniteSec;
        long nsec = InfiniteNsec;
        if (d is { } v)
        {
            var ns = v.Ticks * 100;
            sec = ns / 1_000_000_000;
            nsec = ns % 1_000_000_000;
        }

        sb.Append("  ").Append(name).Append(":\n");
        sb.Append("    sec: ").Append(sec.ToString(CultureInfo.InvariantCulture)).Append('\n');
        sb.Append("    nsec: ").Append(nsec.ToString(CultureInfo.InvariantCulture)).Append('\n');
    }
}
