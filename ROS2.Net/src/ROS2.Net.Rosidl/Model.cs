using System;
using System.Collections.Generic;
using System.Linq;

namespace ROS2.Net.Rosidl;

/// <summary>Primitive ROS 2 IDL types. <see cref="None"/> marks a complex (nested message) type.</summary>
public enum PrimitiveKind
{
    None,
    Bool,
    Byte,
    Char,
    Float32,
    Float64,
    Int8,
    UInt8,
    Int16,
    UInt16,
    Int32,
    UInt32,
    Int64,
    UInt64,
    String,
    WString,
}

/// <summary>How a field is repeated.</summary>
public enum ArrayKind
{
    None,
    /// <summary><c>T[N]</c>: exactly N elements, no length prefix on the wire.</summary>
    Fixed,
    /// <summary><c>T[]</c>: length-prefixed sequence.</summary>
    Unbounded,
    /// <summary><c>T[&lt;=N]</c>: length-prefixed sequence with an upper bound.</summary>
    Bounded,
}

/// <summary>The type of a field or constant as written in a ROS interface file.</summary>
public sealed class FieldType
{
    public PrimitiveKind Primitive { get; set; }

    /// <summary>Package of a complex type, e.g. <c>geometry_msgs</c>.</summary>
    public string? Package { get; set; }

    /// <summary>Name of a complex type, e.g. <c>Vector3</c>.</summary>
    public string? Name { get; set; }

    /// <summary>Upper bound of a bounded string (<c>string&lt;=N</c>), or null.</summary>
    public int? StringBound { get; set; }

    public ArrayKind Array { get; set; }

    /// <summary>Element count for fixed arrays, or the bound for bounded sequences.</summary>
    public int ArraySize { get; set; }

    public bool IsComplex => Primitive == PrimitiveKind.None;

    public bool IsArray => Array != ArrayKind.None;

    public bool IsSequence => Array is ArrayKind.Unbounded or ArrayKind.Bounded;

    public bool IsString => Primitive is PrimitiveKind.String or PrimitiveKind.WString;

    /// <summary>A primitive that is not a string: it has a fixed wire size.</summary>
    public bool IsFixedSizePrimitive => !IsComplex && !IsString;

    /// <summary>The element type without array decoration.</summary>
    public FieldType ElementType => new()
    {
        Primitive = Primitive,
        Package = Package,
        Name = Name,
        StringBound = StringBound,
    };

    /// <summary>Complex type reference in <c>pkg/Name</c> form.</summary>
    public string ComplexRosName => $"{Package}/{Name}";

    public override string ToString()
    {
        var baseName = IsComplex ? ComplexRosName : PrimitiveNames.ToRosName(Primitive);
        if (StringBound is { } b)
        {
            baseName += "<=" + b;
        }

        return Array switch
        {
            ArrayKind.Fixed => $"{baseName}[{ArraySize}]",
            ArrayKind.Unbounded => baseName + "[]",
            ArrayKind.Bounded => $"{baseName}[<={ArraySize}]",
            _ => baseName,
        };
    }
}

/// <summary>A field of a message.</summary>
public sealed class FieldDef
{
    public FieldDef(FieldType type, string name)
    {
        Type = type;
        Name = name;
    }

    public FieldType Type { get; }

    /// <summary>The ROS (snake_case) field name.</summary>
    public string Name { get; }

    /// <summary>Raw default value text, if the definition gave one.</summary>
    public string? DefaultValue { get; set; }

    /// <summary>Comment lines attached to the field.</summary>
    public string? Comment { get; set; }
}

/// <summary>A constant declared in a message (<c>TYPE NAME=VALUE</c>).</summary>
public sealed class ConstantDef
{
    public ConstantDef(FieldType type, string name, string value)
    {
        Type = type;
        Name = name;
        Value = value;
    }

    public FieldType Type { get; }

    public string Name { get; }

    /// <summary>Raw value text (strings are unquoted).</summary>
    public string Value { get; }
}

/// <summary>Where a message structure came from.</summary>
public enum InterfaceKind
{
    Message,
    Service,
    Action,
}

/// <summary>A message structure: a standalone <c>.msg</c>, or one part of a service or action.</summary>
public sealed class MessageDef
{
    public MessageDef(string package, string name, InterfaceKind kind, string rawText)
    {
        Package = package;
        Name = name;
        Kind = kind;
        RawText = rawText;
    }

    public string Package { get; }

    /// <summary>Structure name. For service and action parts this already carries the suffix, e.g. <c>SetBool_Request</c>.</summary>
    public string Name { get; }

    public InterfaceKind Kind { get; }

    public string RawText { get; }

    public List<FieldDef> Fields { get; } = new();

    public List<ConstantDef> Constants { get; } = new();

    /// <summary>Leading comment of the file, used as the generated type's summary.</summary>
    public string? Comment { get; set; }

    /// <summary>The ROS 2 type name, e.g. <c>std_msgs/msg/String</c> or <c>std_srvs/srv/SetBool_Request</c>.</summary>
    public string FullName => $"{Package}/{KindDirectory(Kind)}/{Name}";

    /// <summary>The DDS type name used on the wire, e.g. <c>std_msgs::msg::dds_::String_</c>.</summary>
    public string DdsName => $"{Package}::{KindDirectory(Kind)}::dds_::{Name}_";

    /// <summary>Complex types this structure references, as <c>pkg/Name</c>.</summary>
    public IEnumerable<string> Dependencies =>
        Fields.Where(f => f.Type.IsComplex).Select(f => f.Type.ComplexRosName).Distinct();

    public static string KindDirectory(InterfaceKind kind) => kind switch
    {
        InterfaceKind.Service => "srv",
        InterfaceKind.Action => "action",
        _ => "msg",
    };
}

/// <summary>A <c>.srv</c> definition.</summary>
public sealed class ServiceDef
{
    public ServiceDef(string package, string name, MessageDef request, MessageDef response, string rawText)
    {
        Package = package;
        Name = name;
        Request = request;
        Response = response;
        RawText = rawText;
    }

    public string Package { get; }

    public string Name { get; }

    public MessageDef Request { get; }

    public MessageDef Response { get; }

    public string RawText { get; }

    public string FullName => $"{Package}/srv/{Name}";
}

/// <summary>A <c>.action</c> definition.</summary>
public sealed class ActionDef
{
    public ActionDef(string package, string name, MessageDef goal, MessageDef result, MessageDef feedback, string rawText)
    {
        Package = package;
        Name = name;
        Goal = goal;
        Result = result;
        Feedback = feedback;
        RawText = rawText;
    }

    public string Package { get; }

    public string Name { get; }

    public MessageDef Goal { get; }

    public MessageDef Result { get; }

    public MessageDef Feedback { get; }

    public string RawText { get; }

    public string FullName => $"{Package}/action/{Name}";
}

/// <summary>Thrown when an interface file cannot be parsed.</summary>
public sealed class RosidlParseException : Exception
{
    public RosidlParseException(string file, int line, string message)
        : base($"{file}({line}): {message}")
    {
        File = file;
        Line = line;
    }

    public string File { get; }

    public int Line { get; }
}

/// <summary>Maps between ROS primitive names and <see cref="PrimitiveKind"/>.</summary>
public static class PrimitiveNames
{
    private static readonly Dictionary<string, PrimitiveKind> s_byName = new(StringComparer.Ordinal)
    {
        ["bool"] = PrimitiveKind.Bool,
        ["byte"] = PrimitiveKind.Byte,
        ["char"] = PrimitiveKind.Char,
        ["float32"] = PrimitiveKind.Float32,
        ["float64"] = PrimitiveKind.Float64,
        ["int8"] = PrimitiveKind.Int8,
        ["uint8"] = PrimitiveKind.UInt8,
        ["int16"] = PrimitiveKind.Int16,
        ["uint16"] = PrimitiveKind.UInt16,
        ["int32"] = PrimitiveKind.Int32,
        ["uint32"] = PrimitiveKind.UInt32,
        ["int64"] = PrimitiveKind.Int64,
        ["uint64"] = PrimitiveKind.UInt64,
        ["string"] = PrimitiveKind.String,
        ["wstring"] = PrimitiveKind.WString,
    };

    public static bool TryParse(string name, out PrimitiveKind kind) => s_byName.TryGetValue(name, out kind);

    public static string ToRosName(PrimitiveKind kind) => kind switch
    {
        PrimitiveKind.Bool => "bool",
        PrimitiveKind.Byte => "byte",
        PrimitiveKind.Char => "char",
        PrimitiveKind.Float32 => "float32",
        PrimitiveKind.Float64 => "float64",
        PrimitiveKind.Int8 => "int8",
        PrimitiveKind.UInt8 => "uint8",
        PrimitiveKind.Int16 => "int16",
        PrimitiveKind.UInt16 => "uint16",
        PrimitiveKind.Int32 => "int32",
        PrimitiveKind.UInt32 => "uint32",
        PrimitiveKind.Int64 => "int64",
        PrimitiveKind.UInt64 => "uint64",
        PrimitiveKind.String => "string",
        PrimitiveKind.WString => "wstring",
        _ => throw new ArgumentOutOfRangeException(nameof(kind)),
    };

    /// <summary>Size of the primitive on the CDR wire, or 0 for strings.</summary>
    public static int WireSize(PrimitiveKind kind) => kind switch
    {
        PrimitiveKind.Bool or PrimitiveKind.Byte or PrimitiveKind.Char or PrimitiveKind.Int8 or PrimitiveKind.UInt8 => 1,
        PrimitiveKind.Int16 or PrimitiveKind.UInt16 => 2,
        PrimitiveKind.Int32 or PrimitiveKind.UInt32 or PrimitiveKind.Float32 => 4,
        PrimitiveKind.Int64 or PrimitiveKind.UInt64 or PrimitiveKind.Float64 => 8,
        _ => 0,
    };
}
