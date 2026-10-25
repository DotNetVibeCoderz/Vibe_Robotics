using System.Buffers;
using System.Text.Json;
using ROS2.Net.Rosidl;

namespace ROS2.Net.Serialization;

/// <summary>
/// A message type known only from its definition text, for tools that handle arbitrary types:
/// converts between CDR and JSON without generated code (topic echo/pub, bag inspection, bridges).
/// </summary>
public sealed class DynamicMessageType
{
    private const string Separator = "================================================================================";
    private readonly Dictionary<string, MessageDef> _types;

    private DynamicMessageType(string name, MessageDef root, Dictionary<string, MessageDef> types)
    {
        Name = name;
        Root = root;
        _types = types;
    }

    /// <summary>ROS type name, e.g. <c>sensor_msgs/msg/Imu</c>.</summary>
    public string Name { get; }

    public MessageDef Root { get; }

    /// <summary>
    /// Parses a self-contained <c>ros2msg</c> definition (the format of MCAP schemas and
    /// <see cref="RosTypeRegistry.GetFullDefinition"/>): the root text followed by <c>MSG: pkg/Name</c> sections.
    /// </summary>
    public static DynamicMessageType Parse(string typeName, string fullDefinition)
    {
        var (pkg, name) = Split(typeName);
        var types = new Dictionary<string, MessageDef>(StringComparer.Ordinal);
        var sections = fullDefinition.Replace("\r\n", "\n").Split(Separator);
        var root = RosidlParser.ParseMessage(pkg, name, sections[0]);
        types[pkg + "/" + name] = root;
        for (var i = 1; i < sections.Length; i++)
        {
            var text = sections[i].TrimStart('\n');
            var nl = text.IndexOf('\n');
            var header = (nl < 0 ? text : text[..nl]).Trim();
            if (!header.StartsWith("MSG:", StringComparison.Ordinal))
            {
                continue;
            }

            var (dp, dn) = Split(header[4..].Trim());
            types[dp + "/" + dn] = RosidlParser.ParseMessage(dp, dn, nl < 0 ? string.Empty : text[(nl + 1)..]);
        }

        return new DynamicMessageType(typeName, root, types);
    }

    /// <summary>Builds the type from definitions registered in <see cref="RosTypeRegistry"/>.</summary>
    public static bool TryFromRegistry(string typeName, [System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out DynamicMessageType? type)
    {
        var full = RosTypeRegistry.GetFullDefinition(typeName);
        if (full.Length == 0)
        {
            type = null;
            return false;
        }

        type = Parse(RosTypeRegistry.TryGetMessage(typeName, out var info) ? info.Name : typeName, full);
        return true;
    }

    private static (string Package, string Name) Split(string typeName)
    {
        var parts = typeName.Split('/');
        return parts.Length switch
        {
            3 => (parts[0], parts[2]),
            2 => (parts[0], parts[1]),
            _ => throw new ArgumentException($"'{typeName}' is not a ROS type name.", nameof(typeName)),
        };
    }

    // ------------------------------------------------------------------ CDR → JSON

    /// <summary>Decodes CDR (with encapsulation header) to JSON.</summary>
    public string ToJson(ReadOnlySpan<byte> cdr, bool indented = false)
    {
        var buffer = new ArrayBufferWriter<byte>(512);
        using (var writer = new Utf8JsonWriter(buffer, new JsonWriterOptions { Indented = indented }))
        {
            WriteJson(cdr, writer);
        }

        return System.Text.Encoding.UTF8.GetString(buffer.WrittenSpan);
    }

    public void WriteJson(ReadOnlySpan<byte> cdr, Utf8JsonWriter writer)
    {
        var reader = new CdrReader(cdr);
        WriteStruct(Root, ref reader, writer);
    }

    private void WriteStruct(MessageDef def, ref CdrReader r, Utf8JsonWriter w)
    {
        w.WriteStartObject();
        if (def.Fields.Count == 0)
        {
            r.ReadByte();
        }

        foreach (var f in def.Fields)
        {
            w.WritePropertyName(f.Name);
            var t = f.Type;
            if (!t.IsArray)
            {
                WriteValue(t, ref r, w);
                continue;
            }

            var count = t.Array == ArrayKind.Fixed ? t.ArraySize : r.ReadSequenceLength(Math.Max(1, PrimitiveNames.WireSize(t.Primitive)));
            if (t.Primitive is PrimitiveKind.UInt8 or PrimitiveKind.Byte or PrimitiveKind.Char)
            {
                w.WriteBase64StringValue(r.ReadRaw(count));
                continue;
            }

            w.WriteStartArray();
            for (var i = 0; i < count; i++)
            {
                WriteValue(t, ref r, w);
            }

            w.WriteEndArray();
        }

        w.WriteEndObject();
    }

    private void WriteValue(FieldType t, ref CdrReader r, Utf8JsonWriter w)
    {
        switch (t.Primitive)
        {
            case PrimitiveKind.None: WriteStruct(Resolve(t), ref r, w); break;
            case PrimitiveKind.Bool: w.WriteBooleanValue(r.ReadBool()); break;
            case PrimitiveKind.Byte or PrimitiveKind.Char or PrimitiveKind.UInt8: w.WriteNumberValue(r.ReadByte()); break;
            case PrimitiveKind.Int8: w.WriteNumberValue(r.ReadSByte()); break;
            case PrimitiveKind.Int16: w.WriteNumberValue(r.ReadInt16()); break;
            case PrimitiveKind.UInt16: w.WriteNumberValue(r.ReadUInt16()); break;
            case PrimitiveKind.Int32: w.WriteNumberValue(r.ReadInt32()); break;
            case PrimitiveKind.UInt32: w.WriteNumberValue(r.ReadUInt32()); break;
            case PrimitiveKind.Int64: w.WriteNumberValue(r.ReadInt64()); break;
            case PrimitiveKind.UInt64: w.WriteNumberValue(r.ReadUInt64()); break;
            case PrimitiveKind.Float32: WriteFloat(w, r.ReadSingle()); break;
            case PrimitiveKind.Float64: WriteFloat(w, r.ReadDouble()); break;
            case PrimitiveKind.String: w.WriteStringValue(r.ReadString()); break;
            case PrimitiveKind.WString: w.WriteStringValue(r.ReadWString()); break;
        }
    }

    private static void WriteFloat(Utf8JsonWriter w, double v)
    {
        if (double.IsFinite(v))
        {
            w.WriteNumberValue(v);
        }
        else
        {
            w.WriteStringValue(double.IsNaN(v) ? "NaN" : v > 0 ? "Infinity" : "-Infinity");
        }
    }

    // ------------------------------------------------------------------ JSON → CDR

    /// <summary>Encodes JSON (rosbridge conventions; missing fields take defaults) to CDR with header.</summary>
    public byte[] FromJson(JsonElement element)
    {
        using var writer = new CdrWriter(512);
        WriteStructCdr(Root, element, writer);
        return writer.ToArray();
    }

    public byte[] FromJson(string json)
    {
        using var doc = JsonDocument.Parse(string.IsNullOrWhiteSpace(json) ? "{}" : json);
        return FromJson(doc.RootElement);
    }

    private void WriteStructCdr(MessageDef def, JsonElement? e, CdrWriter w)
    {
        if (def.Fields.Count == 0)
        {
            w.WriteByte(0);
        }

        foreach (var f in def.Fields)
        {
            JsonElement? v = e is { ValueKind: JsonValueKind.Object } obj && obj.TryGetProperty(f.Name, out var p) ? p : null;
            var t = f.Type;
            if (!t.IsArray)
            {
                WriteValueCdr(t, v ?? Default(f), w);
                continue;
            }

            var items = new List<JsonElement?>();
            if (v is { ValueKind: JsonValueKind.String } s && t.Primitive is PrimitiveKind.UInt8 or PrimitiveKind.Byte or PrimitiveKind.Char)
            {
                var bytes = s.GetBytesFromBase64();
                if (t.Array != ArrayKind.Fixed)
                {
                    w.WriteUInt32((uint)bytes.Length);
                }

                w.WriteRaw(bytes);
                continue;
            }

            if (v is { ValueKind: JsonValueKind.Array } arr)
            {
                items.AddRange(arr.EnumerateArray().Select(x => (JsonElement?)x));
            }

            if (t.Array == ArrayKind.Fixed)
            {
                while (items.Count < t.ArraySize)
                {
                    items.Add(null);
                }

                items = items.Take(t.ArraySize).ToList();
            }
            else
            {
                w.WriteUInt32((uint)items.Count);
            }

            foreach (var item in items)
            {
                WriteValueCdr(t.ElementType, item, w);
            }
        }
    }

    private static JsonElement? Default(FieldDef f) =>
        f.DefaultValue is { Length: > 0 } d && !f.Type.IsComplex
            ? JsonDocument.Parse(f.Type.IsString ? JsonSerializer.Serialize(d.Trim('"', '\'')) : d).RootElement
            : null;

    private void WriteValueCdr(FieldType t, JsonElement? v, CdrWriter w)
    {
        switch (t.Primitive)
        {
            case PrimitiveKind.None: WriteStructCdr(Resolve(t), v, w); break;
            case PrimitiveKind.Bool: w.WriteBool(v is { } b && RosJson.ReadBool(b)); break;
            case PrimitiveKind.Byte or PrimitiveKind.Char or PrimitiveKind.UInt8: w.WriteByte(v is { } x1 ? RosJson.ReadByte(x1) : (byte)0); break;
            case PrimitiveKind.Int8: w.WriteSByte(v is { } x2 ? RosJson.ReadSByte(x2) : (sbyte)0); break;
            case PrimitiveKind.Int16: w.WriteInt16(v is { } x3 ? RosJson.ReadInt16(x3) : (short)0); break;
            case PrimitiveKind.UInt16: w.WriteUInt16(v is { } x4 ? RosJson.ReadUInt16(x4) : (ushort)0); break;
            case PrimitiveKind.Int32: w.WriteInt32(v is { } x5 ? RosJson.ReadInt32(x5) : 0); break;
            case PrimitiveKind.UInt32: w.WriteUInt32(v is { } x6 ? RosJson.ReadUInt32(x6) : 0); break;
            case PrimitiveKind.Int64: w.WriteInt64(v is { } x7 ? RosJson.ReadInt64(x7) : 0); break;
            case PrimitiveKind.UInt64: w.WriteUInt64(v is { } x8 ? RosJson.ReadUInt64(x8) : 0); break;
            case PrimitiveKind.Float32: w.WriteSingle(v is { } x9 ? RosJson.ReadSingle(x9) : 0f); break;
            case PrimitiveKind.Float64: w.WriteDouble(v is { } x10 ? RosJson.ReadDouble(x10) : 0d); break;
            case PrimitiveKind.String: w.WriteString(v is { } x11 ? RosJson.ReadString(x11) : string.Empty); break;
            case PrimitiveKind.WString: w.WriteWString(v is { } x12 ? RosJson.ReadString(x12) : string.Empty); break;
        }
    }

    private MessageDef Resolve(FieldType t) =>
        _types.TryGetValue(t.ComplexRosName, out var def)
            ? def
            : throw new InvalidOperationException($"Definition of '{t.ComplexRosName}' is missing from the type description of {Name}.");
}
