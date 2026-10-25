using System.Globalization;
using System.Text;

namespace ROS2.Net.Bag;

/// <summary>A summary of a bag, as printed by <c>ros2 bag info</c>.</summary>
public sealed class BagInfo
{
    private BagInfo(string path, BagMetadata metadata, IReadOnlyList<(string File, long Size)> files)
    {
        Path = path;
        Metadata = metadata;
        Files = files;
    }

    public string Path { get; }

    public BagMetadata Metadata { get; }

    public IReadOnlyList<(string File, long Size)> Files { get; }

    public long TotalSize => Files.Sum(f => f.Size);

    public static BagInfo Read(string path)
    {
        using var reader = BagReader.Open(path);
        var files = reader.Files.Select(f => (System.IO.Path.GetFileName(f), new FileInfo(f).Length)).ToList();
        return new BagInfo(path, reader.Metadata, files);
    }

    public override string ToString()
    {
        var inv = CultureInfo.InvariantCulture;
        var m = Metadata;
        var sb = new StringBuilder();
        sb.Append("Files:             ").AppendJoin("\n                   ", Files.Select(f => f.File)).Append('\n');
        sb.Append("Bag size:          ").Append(FormatSize(TotalSize)).Append('\n');
        sb.Append("Storage id:        ").Append(m.StorageIdentifier).Append('\n');
        sb.Append("Duration:          ").Append((m.Duration.Nanoseconds / 1e9).ToString("0.000000000", inv)).Append("s\n");
        sb.Append("Start:             ").Append(FormatTime(m.StartingTime)).Append('\n');
        sb.Append("End:               ").Append(FormatTime(m.EndTime)).Append('\n');
        sb.Append("Messages:          ").Append(m.MessageCount.ToString(inv)).Append('\n');
        sb.Append("Topic information: ");
        var first = true;
        foreach (var t in m.Topics)
        {
            if (!first)
            {
                sb.Append("                   ");
            }

            first = false;
            sb.Append("Topic: ").Append(t.Name).Append(" | Type: ").Append(t.Type)
              .Append(" | Count: ").Append(t.MessageCount.ToString(inv))
              .Append(" | Serialization Format: ").Append(t.SerializationFormat).Append('\n');
        }

        if (first)
        {
            sb.Append('\n');
        }

        return sb.ToString();
    }

    private static string FormatTime(RosTime t) =>
        t.ToDateTime().ToString("MMM dd yyyy HH:mm:ss.fffffff", CultureInfo.InvariantCulture) + $" ({t})";

    public static string FormatSize(long bytes)
    {
        string[] units = ["B", "KiB", "MiB", "GiB", "TiB"];
        double v = bytes;
        var u = 0;
        while (v >= 1024 && u < units.Length - 1)
        {
            v /= 1024;
            u++;
        }

        return v.ToString(u == 0 ? "0" : "0.0", CultureInfo.InvariantCulture) + " " + units[u];
    }
}
