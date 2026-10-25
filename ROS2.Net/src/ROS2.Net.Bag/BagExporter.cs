using System.Globalization;
using System.Text;
using System.Text.Json;

namespace ROS2.Net.Bag;

/// <summary>
/// Turns bags into datasets for machine learning: one JSON object per message (JSONL), or a CSV of
/// a topic's flattened numeric/boolean/string fields with one row per message.
/// </summary>
public static class BagExporter
{
    /// <summary>
    /// Writes <c>{"topic":…,"type":…,"log_time_ns":…,"publish_time_ns":…,"message":{…}}</c> per line.
    /// Returns the number of lines written.
    /// </summary>
    public static long ToJsonLines(BagReader reader, TextWriter output, IEnumerable<string>? topics = null, RosTime? start = null, RosTime? end = null)
    {
        long n = 0;
        foreach (var msg in reader.ReadMessages(start, end, topics))
        {
            output.Write("{\"topic\":");
            output.Write(JsonSerializer.Serialize(msg.Topic));
            output.Write(",\"type\":");
            output.Write(JsonSerializer.Serialize(msg.Type));
            output.Write(",\"log_time_ns\":");
            output.Write(msg.LogTime.Nanoseconds.ToString(CultureInfo.InvariantCulture));
            output.Write(",\"publish_time_ns\":");
            output.Write(msg.PublishTime.Nanoseconds.ToString(CultureInfo.InvariantCulture));
            output.Write(",\"message\":");
            output.Write(reader.ToJson(msg));
            output.Write("}\n");
            n++;
        }

        return n;
    }

    /// <summary>
    /// Writes a CSV for one topic. Columns are <c>log_time_ns</c> plus every leaf field of the first
    /// message (nested fields as <c>a.b</c>, arrays as <c>a[0]</c> up to <paramref name="maxArrayElements"/>);
    /// base64 byte blobs are skipped. Returns the number of rows.
    /// </summary>
    public static long ToCsv(BagReader reader, string topic, TextWriter output, int maxArrayElements = 64, RosTime? start = null, RosTime? end = null)
    {
        List<string>? columns = null;
        long rows = 0;
        var values = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (var msg in reader.ReadMessages(start, end, [topic]))
        {
            values.Clear();
            using (var doc = JsonDocument.Parse(reader.ToJson(msg)))
            {
                Flatten(doc.RootElement, string.Empty, values, maxArrayElements);
            }

            if (columns is null)
            {
                columns = values.Keys.ToList();
                output.Write("log_time_ns");
                foreach (var c in columns)
                {
                    output.Write(',');
                    output.Write(Escape(c));
                }

                output.Write('\n');
            }

            output.Write(msg.LogTime.Nanoseconds.ToString(CultureInfo.InvariantCulture));
            foreach (var c in columns)
            {
                output.Write(',');
                if (values.TryGetValue(c, out var v))
                {
                    output.Write(v);
                }
            }

            output.Write('\n');
            rows++;
        }

        return rows;
    }

    private static void Flatten(JsonElement e, string prefix, Dictionary<string, string> into, int maxArray)
    {
        switch (e.ValueKind)
        {
            case JsonValueKind.Object:
                foreach (var p in e.EnumerateObject())
                {
                    Flatten(p.Value, prefix.Length == 0 ? p.Name : prefix + "." + p.Name, into, maxArray);
                }

                break;
            case JsonValueKind.Array:
                var i = 0;
                foreach (var item in e.EnumerateArray())
                {
                    if (i >= maxArray)
                    {
                        break;
                    }

                    Flatten(item, $"{prefix}[{i}]", into, maxArray);
                    i++;
                }

                break;
            case JsonValueKind.Number:
                into[prefix] = e.GetRawText();
                break;
            case JsonValueKind.True:
                into[prefix] = "1";
                break;
            case JsonValueKind.False:
                into[prefix] = "0";
                break;
            case JsonValueKind.String:
                var s = e.GetString() ?? string.Empty;
                if (s is "NaN" or "Infinity" or "-Infinity")
                {
                    into[prefix] = s == "NaN" ? "nan" : s == "Infinity" ? "inf" : "-inf";
                }
                else if (!LooksLikeBase64Blob(s))
                {
                    into[prefix] = Escape(s);
                }

                break;
        }
    }

    private static bool LooksLikeBase64Blob(string s) => s.Length > 64 && s.All(c => char.IsAsciiLetterOrDigit(c) || c is '+' or '/' or '=');

    private static string Escape(string s) =>
        s.IndexOfAny([',', '"', '\n', '\r']) >= 0 ? "\"" + s.Replace("\"", "\"\"") + "\"" : s;

    /// <summary>JSONL into a file; returns the line count.</summary>
    public static long ToJsonLinesFile(BagReader reader, string path, IEnumerable<string>? topics = null)
    {
        using var w = new StreamWriter(path, false, new UTF8Encoding(false));
        return ToJsonLines(reader, w, topics);
    }

    /// <summary>CSV into a file; returns the row count.</summary>
    public static long ToCsvFile(BagReader reader, string topic, string path, int maxArrayElements = 64)
    {
        using var w = new StreamWriter(path, false, new UTF8Encoding(false));
        return ToCsv(reader, topic, w, maxArrayElements);
    }
}
