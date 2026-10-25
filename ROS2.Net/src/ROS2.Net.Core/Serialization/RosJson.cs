using System.Buffers;
using System.Globalization;
using System.Text;
using System.Text.Json;

namespace ROS2.Net.Serialization;

/// <summary>
/// JSON mapping of ROS 2 messages, compatible with rosbridge v2: fields keep their snake_case names,
/// <c>uint8[]</c>/<c>char[]</c> are base64 strings, and non-finite floats are written as strings.
/// </summary>
public static class RosJson
{
    private static readonly JsonWriterOptions s_compact = new() { SkipValidation = true };
    private static readonly JsonWriterOptions s_indented = new() { Indented = true, SkipValidation = true };

    /// <summary>Serializes a message to a JSON string.</summary>
    public static string ToJsonString(IRosMessage message, bool indented = false)
    {
        var buffer = new ArrayBufferWriter<byte>(256);
        using (var writer = new Utf8JsonWriter(buffer, indented ? s_indented : s_compact))
        {
            message.WriteJson(writer);
        }

        return Encoding.UTF8.GetString(buffer.WrittenSpan);
    }

    /// <summary>Serializes a message to UTF-8 JSON bytes.</summary>
    public static byte[] ToJsonBytes(IRosMessage message)
    {
        var buffer = new ArrayBufferWriter<byte>(256);
        using (var writer = new Utf8JsonWriter(buffer, s_compact))
        {
            message.WriteJson(writer);
        }

        return buffer.WrittenSpan.ToArray();
    }

    /// <summary>Parses a message from JSON. Missing fields keep their defaults.</summary>
    public static T FromJson<T>(string json)
        where T : class, IRosMessage, new()
    {
        using var doc = JsonDocument.Parse(json);
        var message = new T();
        message.ReadJson(doc.RootElement);
        return message;
    }

    /// <summary>Parses JSON into an existing message.</summary>
    public static void Populate(IRosMessage message, JsonElement element) => message.ReadJson(element);

    // ---- writing ----

    public static void WriteFloat(Utf8JsonWriter writer, string name, double value)
    {
        if (double.IsFinite(value))
        {
            writer.WriteNumber(name, value);
        }
        else
        {
            writer.WriteString(name, NonFinite(value));
        }
    }

    public static void WriteFloat(Utf8JsonWriter writer, string name, float value)
    {
        if (float.IsFinite(value))
        {
            writer.WriteNumber(name, value);
        }
        else
        {
            writer.WriteString(name, NonFinite(value));
        }
    }

    public static void WriteBytes(Utf8JsonWriter writer, string name, byte[]? value) =>
        writer.WriteBase64String(name, value ?? []);

    public static void WriteArray(Utf8JsonWriter writer, string name, bool[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteBooleanValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, sbyte[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteNumberValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, short[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteNumberValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, ushort[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteNumberValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, int[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteNumberValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, uint[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteNumberValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, long[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteNumberValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, ulong[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteNumberValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, float[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            if (float.IsFinite(v))
            {
                writer.WriteNumberValue(v);
            }
            else
            {
                writer.WriteStringValue(NonFinite(v));
            }
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, double[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            if (double.IsFinite(v))
            {
                writer.WriteNumberValue(v);
            }
            else
            {
                writer.WriteStringValue(NonFinite(v));
            }
        }

        writer.WriteEndArray();
    }

    public static void WriteArray(Utf8JsonWriter writer, string name, string[]? values)
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            writer.WriteStringValue(v);
        }

        writer.WriteEndArray();
    }

    public static void WriteMessages<T>(Utf8JsonWriter writer, string name, T[]? values)
        where T : IRosMessage
    {
        writer.WriteStartArray(name);
        foreach (var v in values ?? [])
        {
            v.WriteJson(writer);
        }

        writer.WriteEndArray();
    }

    private static string NonFinite(double v) =>
        double.IsNaN(v) ? "NaN" : v > 0 ? "Infinity" : "-Infinity";

    // ---- reading ----

    public static bool ReadBool(JsonElement e) => e.ValueKind switch
    {
        JsonValueKind.True => true,
        JsonValueKind.False => false,
        JsonValueKind.Number => e.GetDouble() != 0,
        JsonValueKind.String => bool.TryParse(e.GetString(), out var b) && b,
        _ => false,
    };

    public static byte ReadByte(JsonElement e) => (byte)ReadInteger(e);

    public static sbyte ReadSByte(JsonElement e) => (sbyte)ReadInteger(e);

    public static short ReadInt16(JsonElement e) => (short)ReadInteger(e);

    public static ushort ReadUInt16(JsonElement e) => (ushort)ReadInteger(e);

    public static int ReadInt32(JsonElement e) => (int)ReadInteger(e);

    public static uint ReadUInt32(JsonElement e) => (uint)ReadInteger(e);

    public static long ReadInt64(JsonElement e) => ReadInteger(e);

    public static ulong ReadUInt64(JsonElement e) => e.ValueKind == JsonValueKind.Number && e.TryGetUInt64(out var u)
        ? u
        : (ulong)ReadInteger(e);

    public static float ReadSingle(JsonElement e) => (float)ReadDouble(e);

    public static double ReadDouble(JsonElement e) => e.ValueKind switch
    {
        JsonValueKind.Number => e.GetDouble(),
        JsonValueKind.String => e.GetString() switch
        {
            "NaN" or "nan" => double.NaN,
            "Infinity" or "inf" => double.PositiveInfinity,
            "-Infinity" or "-inf" => double.NegativeInfinity,
            var s => double.Parse(s!, CultureInfo.InvariantCulture),
        },
        JsonValueKind.Null => double.NaN,
        _ => 0,
    };

    public static string ReadString(JsonElement e) => e.ValueKind switch
    {
        JsonValueKind.String => e.GetString() ?? string.Empty,
        JsonValueKind.Null => string.Empty,
        _ => e.GetRawText(),
    };

    /// <summary>Reads <c>uint8[]</c>: base64 string (rosbridge default) or a JSON array of numbers.</summary>
    public static byte[] ReadBytes(JsonElement e)
    {
        if (e.ValueKind == JsonValueKind.String)
        {
            return e.GetBytesFromBase64();
        }

        if (e.ValueKind != JsonValueKind.Array)
        {
            return [];
        }

        var result = new byte[e.GetArrayLength()];
        var i = 0;
        foreach (var item in e.EnumerateArray())
        {
            result[i++] = (byte)ReadInteger(item);
        }

        return result;
    }

    public static T[] ReadArray<T>(JsonElement e, Func<JsonElement, T> read)
    {
        if (e.ValueKind != JsonValueKind.Array)
        {
            return [];
        }

        var result = new T[e.GetArrayLength()];
        var i = 0;
        foreach (var item in e.EnumerateArray())
        {
            result[i++] = read(item);
        }

        return result;
    }

    public static T[] ReadMessages<T>(JsonElement e)
        where T : class, IRosMessage, new()
    {
        if (e.ValueKind != JsonValueKind.Array)
        {
            return [];
        }

        var result = new T[e.GetArrayLength()];
        var i = 0;
        foreach (var item in e.EnumerateArray())
        {
            var m = new T();
            m.ReadJson(item);
            result[i++] = m;
        }

        return result;
    }

    private static long ReadInteger(JsonElement e) => e.ValueKind switch
    {
        JsonValueKind.Number => e.TryGetInt64(out var l) ? l : (long)e.GetDouble(),
        JsonValueKind.String => long.Parse(e.GetString()!, CultureInfo.InvariantCulture),
        JsonValueKind.True => 1,
        _ => 0,
    };
}
