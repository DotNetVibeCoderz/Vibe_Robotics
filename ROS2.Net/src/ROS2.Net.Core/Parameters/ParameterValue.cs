using System.Globalization;
using Msg = ROS2.Net.Msgs.RclInterfaces;

namespace ROS2.Net;

/// <summary>Type of a parameter value (<c>rcl_interfaces/msg/ParameterType</c>).</summary>
public enum ParameterType : byte
{
    NotSet = 0,
    Bool = 1,
    Integer = 2,
    Double = 3,
    String = 4,
    ByteArray = 5,
    BoolArray = 6,
    IntegerArray = 7,
    DoubleArray = 8,
    StringArray = 9,
}

/// <summary>An immutable, typed parameter value.</summary>
public sealed class ParameterValue : IEquatable<ParameterValue>
{
    private readonly object? _value;

    private ParameterValue(ParameterType type, object? value)
    {
        Type = type;
        _value = value;
    }

    public static ParameterValue NotSet { get; } = new(ParameterType.NotSet, null);

    public ParameterType Type { get; }

    public bool IsSet => Type != ParameterType.NotSet;

    public static ParameterValue From(bool v) => new(ParameterType.Bool, v);

    public static ParameterValue From(long v) => new(ParameterType.Integer, v);

    public static ParameterValue From(double v) => new(ParameterType.Double, v);

    public static ParameterValue From(string v) => new(ParameterType.String, v ?? string.Empty);

    public static ParameterValue From(byte[] v) => new(ParameterType.ByteArray, (byte[])v.Clone());

    public static ParameterValue From(bool[] v) => new(ParameterType.BoolArray, (bool[])v.Clone());

    public static ParameterValue From(long[] v) => new(ParameterType.IntegerArray, (long[])v.Clone());

    public static ParameterValue From(double[] v) => new(ParameterType.DoubleArray, (double[])v.Clone());

    public static ParameterValue From(string[] v) => new(ParameterType.StringArray, (string[])v.Clone());

    /// <summary>Converts common CLR values (int, float, lists, ...) to a parameter value.</summary>
    public static ParameterValue FromObject(object? value) => value switch
    {
        null => NotSet,
        ParameterValue p => p,
        bool b => From(b),
        byte or sbyte or short or ushort or int or uint or long => From(Convert.ToInt64(value, CultureInfo.InvariantCulture)),
        ulong u => From(checked((long)u)),
        float or double or decimal => From(Convert.ToDouble(value, CultureInfo.InvariantCulture)),
        string s => From(s),
        byte[] bytes => From(bytes),
        bool[] bools => From(bools),
        long[] longs => From(longs),
        int[] ints => From(ints.Select(i => (long)i).ToArray()),
        double[] doubles => From(doubles),
        float[] floats => From(floats.Select(f => (double)f).ToArray()),
        string[] strings => From(strings),
        IEnumerable<bool> bs => From(bs.ToArray()),
        IEnumerable<long> ls => From(ls.ToArray()),
        IEnumerable<int> iss => From(iss.Select(i => (long)i).ToArray()),
        IEnumerable<double> ds => From(ds.ToArray()),
        IEnumerable<string> ss => From(ss.ToArray()),
        _ => throw new ArgumentException($"Type {value.GetType()} cannot be a parameter value.", nameof(value)),
    };

    public bool AsBool() => Get<bool>(ParameterType.Bool);

    public long AsInteger() => Get<long>(ParameterType.Integer);

    /// <summary>Returns the value as a double; integers are widened.</summary>
    public double AsDouble() => Type == ParameterType.Integer ? (long)_value! : Get<double>(ParameterType.Double);

    public string AsString() => Get<string>(ParameterType.String);

    public byte[] AsByteArray() => Get<byte[]>(ParameterType.ByteArray);

    public bool[] AsBoolArray() => Get<bool[]>(ParameterType.BoolArray);

    public long[] AsIntegerArray() => Get<long[]>(ParameterType.IntegerArray);

    public double[] AsDoubleArray() => Type == ParameterType.IntegerArray
        ? ((long[])_value!).Select(l => (double)l).ToArray()
        : Get<double[]>(ParameterType.DoubleArray);

    public string[] AsStringArray() => Get<string[]>(ParameterType.StringArray);

    /// <summary>The boxed value (null when not set).</summary>
    public object? Value => _value;

    /// <summary>Converts to <typeparamref name="T"/>, accepting int/float/list conveniences.</summary>
    public T As<T>()
    {
        object? result = typeof(T) switch
        {
            var t when t == typeof(bool) => AsBool(),
            var t when t == typeof(long) => AsInteger(),
            var t when t == typeof(int) => checked((int)AsInteger()),
            var t when t == typeof(double) => AsDouble(),
            var t when t == typeof(float) => (float)AsDouble(),
            var t when t == typeof(string) => AsString(),
            var t when t == typeof(byte[]) => AsByteArray(),
            var t when t == typeof(bool[]) => AsBoolArray(),
            var t when t == typeof(long[]) => AsIntegerArray(),
            var t when t == typeof(int[]) => AsIntegerArray().Select(l => checked((int)l)).ToArray(),
            var t when t == typeof(double[]) => AsDoubleArray(),
            var t when t == typeof(string[]) => AsStringArray(),
            var t when t == typeof(ParameterValue) => this,
            _ => throw new InvalidCastException($"Parameters cannot be read as {typeof(T)}."),
        };
        return (T)result!;
    }

    private T Get<T>(ParameterType expected)
    {
        if (Type != expected)
        {
            throw new InvalidParameterTypeException($"Parameter is {Type}, not {expected}.");
        }

        return (T)_value!;
    }

    public Msg.ParameterValue ToMsg()
    {
        var m = new Msg.ParameterValue { Type = (byte)Type };
        switch (Type)
        {
            case ParameterType.Bool: m.BoolValue = (bool)_value!; break;
            case ParameterType.Integer: m.IntegerValue = (long)_value!; break;
            case ParameterType.Double: m.DoubleValue = (double)_value!; break;
            case ParameterType.String: m.StringValue = (string)_value!; break;
            case ParameterType.ByteArray: m.ByteArrayValue = (byte[])_value!; break;
            case ParameterType.BoolArray: m.BoolArrayValue = (bool[])_value!; break;
            case ParameterType.IntegerArray: m.IntegerArrayValue = (long[])_value!; break;
            case ParameterType.DoubleArray: m.DoubleArrayValue = (double[])_value!; break;
            case ParameterType.StringArray: m.StringArrayValue = (string[])_value!; break;
        }

        return m;
    }

    public static ParameterValue FromMsg(Msg.ParameterValue m) => (ParameterType)m.Type switch
    {
        ParameterType.Bool => From(m.BoolValue),
        ParameterType.Integer => From(m.IntegerValue),
        ParameterType.Double => From(m.DoubleValue),
        ParameterType.String => From(m.StringValue),
        ParameterType.ByteArray => From(m.ByteArrayValue),
        ParameterType.BoolArray => From(m.BoolArrayValue),
        ParameterType.IntegerArray => From(m.IntegerArrayValue),
        ParameterType.DoubleArray => From(m.DoubleArrayValue),
        ParameterType.StringArray => From(m.StringArrayValue),
        _ => NotSet,
    };

    /// <summary>Parses a command-line value (<c>-p name:=value</c>) using YAML scalar rules.</summary>
    public static ParameterValue Parse(string text) => ParameterYaml.ParseScalarOrList(text);

    public bool Equals(ParameterValue? other)
    {
        if (other is null || other.Type != Type)
        {
            return false;
        }

        return Type switch
        {
            ParameterType.NotSet => true,
            ParameterType.ByteArray => ((byte[])_value!).AsSpan().SequenceEqual((byte[])other._value!),
            ParameterType.BoolArray => ((bool[])_value!).AsSpan().SequenceEqual((bool[])other._value!),
            ParameterType.IntegerArray => ((long[])_value!).AsSpan().SequenceEqual((long[])other._value!),
            ParameterType.DoubleArray => ((double[])_value!).AsSpan().SequenceEqual((double[])other._value!),
            ParameterType.StringArray => ((string[])_value!).SequenceEqual((string[])other._value!),
            _ => Equals(_value, other._value),
        };
    }

    public override bool Equals(object? obj) => Equals(obj as ParameterValue);

    public override int GetHashCode() => HashCode.Combine(Type, _value is Array ? 0 : _value?.GetHashCode() ?? 0);

    public override string ToString() => Type switch
    {
        ParameterType.NotSet => "<not set>",
        ParameterType.Bool => (bool)_value! ? "true" : "false",
        ParameterType.Double => ((double)_value!).ToString("R", CultureInfo.InvariantCulture),
        ParameterType.Integer => ((long)_value!).ToString(CultureInfo.InvariantCulture),
        ParameterType.String => (string)_value!,
        ParameterType.ByteArray => "[" + string.Join(", ", (byte[])_value!) + "]",
        ParameterType.BoolArray => "[" + string.Join(", ", ((bool[])_value!).Select(b => b ? "true" : "false")) + "]",
        ParameterType.IntegerArray => "[" + string.Join(", ", (long[])_value!) + "]",
        ParameterType.DoubleArray => "[" + string.Join(", ", ((double[])_value!).Select(d => d.ToString("R", CultureInfo.InvariantCulture))) + "]",
        ParameterType.StringArray => "[" + string.Join(", ", (string[])_value!) + "]",
        _ => string.Empty,
    };

    public static implicit operator ParameterValue(bool v) => From(v);

    public static implicit operator ParameterValue(long v) => From(v);

    public static implicit operator ParameterValue(int v) => From(v);

    public static implicit operator ParameterValue(double v) => From(v);

    public static implicit operator ParameterValue(string v) => From(v);
}

/// <summary>A named parameter value.</summary>
public sealed record Parameter(string Name, ParameterValue Value)
{
    public Parameter(string name, object? value)
        : this(name, ParameterValue.FromObject(value))
    {
    }

    public Msg.Parameter ToMsg() => new() { Name = Name, Value = Value.ToMsg() };

    public static Parameter FromMsg(Msg.Parameter m) => new(m.Name, ParameterValue.FromMsg(m.Value));

    public override string ToString() => $"{Name}={Value}";
}

/// <summary>Range constraint for a floating-point parameter. <c>Step</c> of 0 means continuous.</summary>
public sealed record FloatingPointRange(double From, double To, double Step = 0);

/// <summary>Range constraint for an integer parameter. <c>Step</c> of 0 means any integer.</summary>
public sealed record IntegerRange(long From, long To, ulong Step = 0);

/// <summary>Describes a parameter: its type, documentation and constraints.</summary>
public sealed record ParameterDescriptor
{
    public string Name { get; init; } = string.Empty;

    public ParameterType Type { get; init; }

    public string Description { get; init; } = string.Empty;

    public string AdditionalConstraints { get; init; } = string.Empty;

    public bool ReadOnly { get; init; }

    /// <summary>Allow the value's type to change after declaration.</summary>
    public bool DynamicTyping { get; init; }

    public FloatingPointRange? FloatingPointRange { get; init; }

    public IntegerRange? IntegerRange { get; init; }

    public Msg.ParameterDescriptor ToMsg() => new()
    {
        Name = Name,
        Type = (byte)Type,
        Description = Description,
        AdditionalConstraints = AdditionalConstraints,
        ReadOnly = ReadOnly,
        DynamicTyping = DynamicTyping,
        FloatingPointRange = FloatingPointRange is { } f ? [new Msg.FloatingPointRange { FromValue = f.From, ToValue = f.To, Step = f.Step }] : [],
        IntegerRange = IntegerRange is { } i ? [new Msg.IntegerRange { FromValue = i.From, ToValue = i.To, Step = i.Step }] : [],
    };

    public static ParameterDescriptor FromMsg(Msg.ParameterDescriptor m) => new()
    {
        Name = m.Name,
        Type = (ParameterType)m.Type,
        Description = m.Description,
        AdditionalConstraints = m.AdditionalConstraints,
        ReadOnly = m.ReadOnly,
        DynamicTyping = m.DynamicTyping,
        FloatingPointRange = m.FloatingPointRange.Length > 0 ? new FloatingPointRange(m.FloatingPointRange[0].FromValue, m.FloatingPointRange[0].ToValue, m.FloatingPointRange[0].Step) : null,
        IntegerRange = m.IntegerRange.Length > 0 ? new IntegerRange(m.IntegerRange[0].FromValue, m.IntegerRange[0].ToValue, m.IntegerRange[0].Step) : null,
    };
}

/// <summary>Result of setting parameters, returned by validation callbacks and set calls.</summary>
public sealed record SetParametersResult(bool Successful, string Reason = "")
{
    public static SetParametersResult Success { get; } = new(true);

    public static SetParametersResult Failure(string reason) => new(false, reason);
}

/// <summary>Base of parameter errors.</summary>
public class ParameterException(string message) : InvalidOperationException(message);

/// <summary>The parameter was not declared.</summary>
public sealed class ParameterNotDeclaredException(string name) : ParameterException($"Parameter '{name}' has not been declared.");

/// <summary>The parameter was declared twice.</summary>
public sealed class ParameterAlreadyDeclaredException(string name) : ParameterException($"Parameter '{name}' has already been declared.");

/// <summary>A value of the wrong type was used.</summary>
public sealed class InvalidParameterTypeException(string message) : ParameterException(message);

/// <summary>A set was rejected by a constraint or validation callback.</summary>
public sealed class InvalidParameterValueException(string message) : ParameterException(message);
