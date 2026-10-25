using System.CommandLine;
using System.Globalization;
using System.Text;
using System.Text.Json;
using System.Text.Json.Nodes;
using YamlDotNet.RepresentationModel;

namespace ROS2.Net.Cli;

internal static class Cmd
{
    /// <summary>Attaches an action that turns <see cref="CliException"/> and common failures into clean error output.</summary>
    public static void Handle(this Command command, CliEnvironment env, Func<ParseResult, CancellationToken, Task<int>> action) =>
        command.SetAction(async (parse, ct) =>
        {
            try
            {
                return await action(parse, ct).ConfigureAwait(false);
            }
            catch (OperationCanceledException) when (ct.IsCancellationRequested)
            {
                return 0;
            }
            catch (Exception ex) when (ex is CliException or InvalidRosNameException or TimeoutException or ParameterException or JsonException or YamlDotNet.Core.YamlException or IOException or InvalidOperationException or ArgumentException)
            {
                await env.Error.WriteLineAsync($"error: {ex.Message}").ConfigureAwait(false);
                return 1;
            }
        });

    /// <summary>Cancellation token that fires after <paramref name="seconds"/> (if positive) or when <paramref name="ct"/> fires.</summary>
    public static CancellationTokenSource Limit(double? seconds, CancellationToken ct)
    {
        var cts = CancellationTokenSource.CreateLinkedTokenSource(ct);
        if (seconds is > 0)
        {
            cts.CancelAfter(TimeSpan.FromSeconds(seconds.Value));
        }

        return cts;
    }
}

/// <summary>YAML/JSON conversions for message input and output, in the style of the ros2 CLI.</summary>
internal static class YamlJson
{
    /// <summary>Accepts JSON or YAML (flow or block, e.g. <c>"{data: hello}"</c> / <c>"linear: {x: 0.5}"</c>) and returns JSON.</summary>
    public static string ToJson(string? text)
    {
        if (string.IsNullOrWhiteSpace(text))
        {
            return "{}";
        }

        var t = text.Trim();
        try
        {
            using var doc = JsonDocument.Parse(t);
            return doc.RootElement.GetRawText();
        }
        catch (JsonException)
        {
            // Not JSON: fall through to YAML.
        }

        var stream = new YamlStream();
        stream.Load(new StringReader(t));
        if (stream.Documents.Count == 0)
        {
            return "{}";
        }

        var node = Convert(stream.Documents[0].RootNode);
        return node?.ToJsonString() ?? "{}";
    }

    private static JsonNode? Convert(YamlNode node) => node switch
    {
        YamlMappingNode map => new JsonObject(map.Children.Select(kv =>
            KeyValuePair.Create(((YamlScalarNode)kv.Key).Value ?? string.Empty, Convert(kv.Value)))),
        YamlSequenceNode seq => new JsonArray(seq.Children.Select(Convert).ToArray()),
        YamlScalarNode s => Scalar(s),
        _ => null,
    };

    private static JsonNode? Scalar(YamlScalarNode s)
    {
        var v = s.Value ?? string.Empty;
        if (s.Style is YamlDotNet.Core.ScalarStyle.SingleQuoted or YamlDotNet.Core.ScalarStyle.DoubleQuoted)
        {
            return JsonValue.Create(v);
        }

        if (v is "true" or "True")
        {
            return JsonValue.Create(true);
        }

        if (v is "false" or "False")
        {
            return JsonValue.Create(false);
        }

        if (v is "null" or "~" or "")
        {
            return null;
        }

        if (long.TryParse(v, NumberStyles.AllowLeadingSign, CultureInfo.InvariantCulture, out var l))
        {
            return JsonValue.Create(l);
        }

        if (double.TryParse(v, NumberStyles.Float, CultureInfo.InvariantCulture, out var d))
        {
            return JsonValue.Create(d);
        }

        return JsonValue.Create(v);
    }

    /// <summary>Formats JSON as YAML, like <c>ros2 topic echo</c>.</summary>
    public static string ToYaml(string json)
    {
        using var doc = JsonDocument.Parse(json);
        var sb = new StringBuilder();
        WriteYaml(doc.RootElement, sb, 0, inSequence: false);
        return sb.ToString().TrimEnd('\n');
    }

    private static void WriteYaml(JsonElement e, StringBuilder sb, int indent, bool inSequence)
    {
        var pad = new string(' ', indent);
        switch (e.ValueKind)
        {
            case JsonValueKind.Object:
                var first = true;
                foreach (var p in e.EnumerateObject())
                {
                    var prefix = inSequence && first ? string.Empty : pad;
                    first = false;
                    if (p.Value.ValueKind == JsonValueKind.Object && p.Value.EnumerateObject().Any())
                    {
                        sb.Append(prefix).Append(p.Name).Append(":\n");
                        WriteYaml(p.Value, sb, indent + 2, inSequence: false);
                    }
                    else if (p.Value.ValueKind == JsonValueKind.Array && p.Value.GetArrayLength() > 0 && p.Value.EnumerateArray().Any(x => x.ValueKind == JsonValueKind.Object))
                    {
                        sb.Append(prefix).Append(p.Name).Append(":\n");
                        WriteYaml(p.Value, sb, indent, inSequence: false);
                    }
                    else
                    {
                        sb.Append(prefix).Append(p.Name).Append(": ").Append(Inline(p.Value)).Append('\n');
                    }
                }

                break;
            case JsonValueKind.Array:
                foreach (var item in e.EnumerateArray())
                {
                    sb.Append(pad).Append("- ");
                    if (item.ValueKind == JsonValueKind.Object)
                    {
                        WriteYaml(item, sb, indent + 2, inSequence: true);
                    }
                    else
                    {
                        sb.Append(Inline(item)).Append('\n');
                    }
                }

                break;
            default:
                sb.Append(pad).Append(Inline(e)).Append('\n');
                break;
        }
    }

    private static string Inline(JsonElement e) => e.ValueKind switch
    {
        JsonValueKind.String => QuoteIfNeeded(e.GetString() ?? string.Empty),
        JsonValueKind.Array => "[" + string.Join(", ", e.EnumerateArray().Select(Inline)) + "]",
        JsonValueKind.Object => "{}",
        JsonValueKind.True => "true",
        JsonValueKind.False => "false",
        JsonValueKind.Null => "null",
        _ => e.GetRawText(),
    };

    private static string QuoteIfNeeded(string s) =>
        s.Length == 0 || s.IndexOfAny([':', '#', '\n', '"', '\'', '{', '}', '[', ']', ',']) >= 0 || s.Trim() != s
            ? JsonSerializer.Serialize(s)
            : s;
}

/// <summary>QoS command-line options shared by echo/pub/hz/bw.</summary>
internal sealed class QosOptions
{
    public Option<string?> Profile { get; } = new("--qos-profile") { Description = "default | sensor_data | services_default | parameters | system_default | latched" };

    public Option<string?> Reliability { get; } = new("--qos-reliability") { Description = "reliable | best_effort" };

    public Option<string?> Durability { get; } = new("--qos-durability") { Description = "volatile | transient_local" };

    public Option<int?> Depth { get; } = new("--qos-depth") { Description = "History depth (keep last)." };

    public void AddTo(Command c)
    {
        c.Add(Profile);
        c.Add(Reliability);
        c.Add(Durability);
        c.Add(Depth);
    }

    /// <summary>Explicit QoS from the options, or one adapted to the publishers on the topic (like rosbag2/ros2 topic echo).</summary>
    public QosProfile Resolve(ParseResult parse, Node node, string topic)
    {
        var name = parse.GetValue(Profile);
        QosProfile qos;
        if (name is null)
        {
            var pubs = node.GetPublishersInfo(topic);
            qos = QosProfile.Default;
            if (pubs.Count > 0)
            {
                if (pubs.Any(p => p.Qos.Reliability == ReliabilityPolicy.BestEffort))
                {
                    qos = qos.AsBestEffort();
                }

                if (pubs.All(p => p.Qos.Durability == DurabilityPolicy.TransientLocal))
                {
                    qos = qos.AsTransientLocal();
                }
            }
        }
        else
        {
            qos = name switch
            {
                "default" => QosProfile.Default,
                "sensor_data" => QosProfile.SensorData,
                "services_default" => QosProfile.ServicesDefault,
                "parameters" => QosProfile.Parameters,
                "system_default" => QosProfile.SystemDefault,
                "latched" => QosProfile.Latched,
                _ => throw new CliException($"Unknown QoS profile '{name}'."),
            };
        }

        qos = parse.GetValue(Reliability) switch
        {
            null => qos,
            "reliable" => qos.AsReliable(),
            "best_effort" => qos.AsBestEffort(),
            var r => throw new CliException($"Unknown reliability '{r}'."),
        };
        qos = parse.GetValue(Durability) switch
        {
            null => qos,
            "volatile" => qos.AsVolatile(),
            "transient_local" => qos.AsTransientLocal(),
            var d => throw new CliException($"Unknown durability '{d}'."),
        };
        return parse.GetValue(Depth) is { } depth ? qos.KeepLast(depth) : qos;
    }
}

internal static class Types
{
    /// <summary>Type metadata by name: the registered type, or a placeholder carrying only the name (enough for raw CDR endpoints).</summary>
    public static MessageTypeInfo Resolve(string typeName) =>
        RosTypeRegistry.TryGetMessage(typeName, out var info)
            ? info
            : MessageTypeInfo.Create(Normalize(typeName), typeof(IRosMessage), static () => throw new NotSupportedException("Type not compiled in."));

    /// <summary><c>pkg/Name</c> → <c>pkg/msg/Name</c>.</summary>
    public static string Normalize(string typeName)
    {
        var parts = typeName.Split('/');
        return parts.Length == 2 ? $"{parts[0]}/msg/{parts[1]}" : typeName;
    }
}
