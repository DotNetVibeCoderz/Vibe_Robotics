using System.Text.Json;
using ROS2.Net.Serialization;

namespace ROS2.Net.Rmw.Rtps.Discovery;

/// <summary>One node in <c>rmw_dds_common/msg/ParticipantEntitiesInfo</c>.</summary>
public sealed record NodeEntitiesInfo(string Namespace, string Name, IReadOnlyList<Gid> Readers, IReadOnlyList<Gid> Writers);

/// <summary>
/// <c>rmw_dds_common/msg/ParticipantEntitiesInfo</c>, published on <c>ros_discovery_info</c> so tools
/// map DDS endpoints to ROS nodes. <c>rmw_dds_common/msg/Gid</c> is <c>uint8[24]</c> up to Iron
/// and <c>uint8[16]</c> from Jazzy, so the Gid width is a parameter.
/// </summary>
public sealed class ParticipantEntitiesInfo : IRosMessage
{
    public const string RosType = "rmw_dds_common/msg/ParticipantEntitiesInfo";
    public const string TopicName = "ros_discovery_info";

    public ParticipantEntitiesInfo()
        : this(16)
    {
    }

    public ParticipantEntitiesInfo(int gidSize) => GidSize = gidSize;

    public int GidSize { get; set; }

    public Gid ParticipantGid { get; set; }

    public List<NodeEntitiesInfo> Nodes { get; set; } = [];

    public string RosTypeName => RosType;

    public static MessageTypeInfo TypeInfo(int gidSize) => MessageTypeInfo.Create(
        RosType,
        typeof(ParticipantEntitiesInfo),
        () => new ParticipantEntitiesInfo(gidSize),
        "Gid gid\nNodeEntitiesInfo[] node_entities_info_seq\n");

    public void Serialize(CdrWriter writer)
    {
        WriteGid(writer, ParticipantGid);
        writer.WriteUInt32((uint)Nodes.Count);
        foreach (var n in Nodes)
        {
            writer.WriteString(n.Namespace);
            writer.WriteString(n.Name);
            writer.WriteUInt32((uint)n.Readers.Count);
            foreach (var g in n.Readers)
            {
                WriteGid(writer, g);
            }

            writer.WriteUInt32((uint)n.Writers.Count);
            foreach (var g in n.Writers)
            {
                WriteGid(writer, g);
            }
        }
    }

    public void Deserialize(ref CdrReader reader)
    {
        ParticipantGid = ReadGid(ref reader);
        var count = reader.ReadSequenceLength(12);
        Nodes = new List<NodeEntitiesInfo>(count);
        for (var i = 0; i < count; i++)
        {
            var ns = reader.ReadString();
            var name = reader.ReadString();
            var readers = new Gid[reader.ReadSequenceLength(GidSize)];
            for (var j = 0; j < readers.Length; j++)
            {
                readers[j] = ReadGid(ref reader);
            }

            var writers = new Gid[reader.ReadSequenceLength(GidSize)];
            for (var j = 0; j < writers.Length; j++)
            {
                writers[j] = ReadGid(ref reader);
            }

            Nodes.Add(new NodeEntitiesInfo(ns, name, readers, writers));
        }
    }

    private void WriteGid(CdrWriter writer, Gid gid)
    {
        Span<byte> b = stackalloc byte[24];
        b.Clear();
        gid.WriteTo(b);
        writer.WriteRaw(b[..GidSize]);
    }

    private Gid ReadGid(ref CdrReader reader) => Gid.FromBytes(reader.ReadRaw(GidSize)[..16]);

    public void WriteJson(Utf8JsonWriter writer)
    {
        writer.WriteStartObject();
        writer.WriteString("gid", ParticipantGid.ToString());
        writer.WriteStartArray("node_entities_info_seq");
        foreach (var n in Nodes)
        {
            writer.WriteStartObject();
            writer.WriteString("node_namespace", n.Namespace);
            writer.WriteString("node_name", n.Name);
            writer.WriteNumber("readers", n.Readers.Count);
            writer.WriteNumber("writers", n.Writers.Count);
            writer.WriteEndObject();
        }

        writer.WriteEndArray();
        writer.WriteEndObject();
    }

    public void ReadJson(JsonElement element) => throw new NotSupportedException();

    public IRosMessage CloneMessage() => new ParticipantEntitiesInfo(GidSize) { ParticipantGid = ParticipantGid, Nodes = [.. Nodes] };
}
