using System.Buffers.Binary;
using ROS2.Net.Msgs.SensorMsgs;

namespace ROS2.Net.Perception;

/// <summary>Reads one field of a PointCloud2 point, for any datatype and byte order.</summary>
public readonly record struct PointFieldAccessor(string Name, int Offset, byte Datatype, int Count, bool BigEndian)
{
    /// <summary>Byte size of one element of <paramref name="datatype"/>.</summary>
    public static int SizeOf(byte datatype) => datatype switch
    {
        PointField.INT8 or PointField.UINT8 => 1,
        PointField.INT16 or PointField.UINT16 => 2,
        PointField.INT32 or PointField.UINT32 or PointField.FLOAT32 => 4,
        PointField.FLOAT64 => 8,
        _ => throw new NotSupportedException($"Unknown PointField datatype {datatype}."),
    };

    public int ElementSize => SizeOf(Datatype);

    /// <summary>Reads element <paramref name="element"/> of the field from a point's bytes.</summary>
    public double Read(ReadOnlySpan<byte> point, int element = 0)
    {
        var s = point.Slice(Offset + (element * ElementSize));
        return Datatype switch
        {
            PointField.INT8 => (sbyte)s[0],
            PointField.UINT8 => s[0],
            PointField.INT16 => BigEndian ? BinaryPrimitives.ReadInt16BigEndian(s) : BinaryPrimitives.ReadInt16LittleEndian(s),
            PointField.UINT16 => BigEndian ? BinaryPrimitives.ReadUInt16BigEndian(s) : BinaryPrimitives.ReadUInt16LittleEndian(s),
            PointField.INT32 => BigEndian ? BinaryPrimitives.ReadInt32BigEndian(s) : BinaryPrimitives.ReadInt32LittleEndian(s),
            PointField.UINT32 => BigEndian ? BinaryPrimitives.ReadUInt32BigEndian(s) : BinaryPrimitives.ReadUInt32LittleEndian(s),
            PointField.FLOAT32 => BigEndian ? BinaryPrimitives.ReadSingleBigEndian(s) : BinaryPrimitives.ReadSingleLittleEndian(s),
            PointField.FLOAT64 => BigEndian ? BinaryPrimitives.ReadDoubleBigEndian(s) : BinaryPrimitives.ReadDoubleLittleEndian(s),
            _ => double.NaN,
        };
    }

    /// <summary>Reads the field's raw 32 bits (used for PCL's packed float "rgb").</summary>
    public uint ReadBits32(ReadOnlySpan<byte> point)
    {
        var s = point.Slice(Offset, 4);
        return BigEndian ? BinaryPrimitives.ReadUInt32BigEndian(s) : BinaryPrimitives.ReadUInt32LittleEndian(s);
    }

    /// <summary>Writes element <paramref name="element"/> of the field into a point's bytes.</summary>
    public void Write(Span<byte> point, double value, int element = 0)
    {
        var s = point.Slice(Offset + (element * ElementSize));
        switch (Datatype)
        {
            case PointField.INT8: s[0] = unchecked((byte)(sbyte)value); break;
            case PointField.UINT8: s[0] = (byte)value; break;
            case PointField.INT16: if (BigEndian) { BinaryPrimitives.WriteInt16BigEndian(s, (short)value); } else { BinaryPrimitives.WriteInt16LittleEndian(s, (short)value); } break;
            case PointField.UINT16: if (BigEndian) { BinaryPrimitives.WriteUInt16BigEndian(s, (ushort)value); } else { BinaryPrimitives.WriteUInt16LittleEndian(s, (ushort)value); } break;
            case PointField.INT32: if (BigEndian) { BinaryPrimitives.WriteInt32BigEndian(s, (int)value); } else { BinaryPrimitives.WriteInt32LittleEndian(s, (int)value); } break;
            case PointField.UINT32: if (BigEndian) { BinaryPrimitives.WriteUInt32BigEndian(s, (uint)value); } else { BinaryPrimitives.WriteUInt32LittleEndian(s, (uint)value); } break;
            case PointField.FLOAT32: if (BigEndian) { BinaryPrimitives.WriteSingleBigEndian(s, (float)value); } else { BinaryPrimitives.WriteSingleLittleEndian(s, (float)value); } break;
            case PointField.FLOAT64: if (BigEndian) { BinaryPrimitives.WriteDoubleBigEndian(s, value); } else { BinaryPrimitives.WriteDoubleLittleEndian(s, value); } break;
        }
    }

    public void WriteBits32(Span<byte> point, uint bits)
    {
        var s = point.Slice(Offset, 4);
        if (BigEndian)
        {
            BinaryPrimitives.WriteUInt32BigEndian(s, bits);
        }
        else
        {
            BinaryPrimitives.WriteUInt32LittleEndian(s, bits);
        }
    }
}

/// <summary>
/// Zero-copy view of a <c>sensor_msgs/PointCloud2</c>: points are read straight from the message's
/// data array, honoring <c>point_step</c>, <c>row_step</c> padding, organized clouds and byte order.
/// </summary>
public readonly struct PointCloud2Reader
{
    private readonly PointCloud2 _msg;

    public PointCloud2Reader(PointCloud2 message)
    {
        _msg = message ?? throw new ArgumentNullException(nameof(message));
        var needed = (long)message.RowStep * message.Height;
        if (message.Data.Length < needed || message.PointStep * (long)message.Width > message.RowStep)
        {
            throw new ArgumentException($"PointCloud2 is inconsistent: {message.Data.Length} data bytes for {message.Height} rows of {message.RowStep}.", nameof(message));
        }
    }

    public int Count => checked((int)(_msg.Width * _msg.Height));

    public int Width => (int)_msg.Width;

    public int Height => (int)_msg.Height;

    public bool IsBigEndian => _msg.IsBigendian;

    public PointCloud2 Message => _msg;

    /// <summary>The bytes of point <paramref name="index"/> (row-major).</summary>
    public ReadOnlySpan<byte> Point(int index)
    {
        var row = index / (int)_msg.Width;
        var col = index - (row * (int)_msg.Width);
        return _msg.Data.AsSpan((int)((row * _msg.RowStep) + (col * _msg.PointStep)), (int)_msg.PointStep);
    }

    public bool HasField(string name) => _msg.Fields.Any(f => f.Name == name);

    public PointFieldAccessor GetField(string name)
    {
        var f = _msg.Fields.FirstOrDefault(f => f.Name == name)
            ?? throw new KeyNotFoundException($"PointCloud2 has no field '{name}' (fields: {string.Join(", ", _msg.Fields.Select(x => x.Name))}).");
        return new PointFieldAccessor(f.Name, (int)f.Offset, f.Datatype, (int)Math.Max(1, f.Count), _msg.IsBigendian);
    }

    public bool TryGetField(string name, out PointFieldAccessor accessor)
    {
        if (HasField(name))
        {
            accessor = GetField(name);
            return true;
        }

        accessor = default;
        return false;
    }

    /// <summary>Copies positions into <paramref name="destination"/> (3 floats per point). Returns the number of points written.</summary>
    public int ReadXyz(Span<float> destination, bool skipNonFinite = false)
    {
        var fx = GetField("x");
        var fy = GetField("y");
        var fz = GetField("z");
        var fast = !IsBigEndian && BitConverter.IsLittleEndian && fx.Datatype == PointField.FLOAT32 && fy.Datatype == PointField.FLOAT32 && fz.Datatype == PointField.FLOAT32;
        var n = 0;
        for (var i = 0; i < Count; i++)
        {
            var p = Point(i);
            float x, y, z;
            if (fast)
            {
                x = BitConverter.ToSingle(p.Slice(fx.Offset, 4));
                y = BitConverter.ToSingle(p.Slice(fy.Offset, 4));
                z = BitConverter.ToSingle(p.Slice(fz.Offset, 4));
            }
            else
            {
                x = (float)fx.Read(p);
                y = (float)fy.Read(p);
                z = (float)fz.Read(p);
            }

            if (skipNonFinite && !(float.IsFinite(x) && float.IsFinite(y) && float.IsFinite(z)))
            {
                continue;
            }

            destination[3 * n] = x;
            destination[(3 * n) + 1] = y;
            destination[(3 * n) + 2] = z;
            n++;
        }

        return n;
    }

    /// <summary>Converts to a <see cref="PointCloud"/>, picking up <c>intensity</c> and <c>rgb</c>/<c>rgba</c> fields.</summary>
    public static PointCloud ToPointCloud(PointCloud2 message, bool skipNonFinite = false)
    {
        var reader = new PointCloud2Reader(message);
        var fx = reader.GetField("x");
        var fy = reader.GetField("y");
        var fz = reader.GetField("z");
        var hasI = reader.TryGetField("intensity", out var fi);
        var hasRgb = reader.TryGetField("rgb", out var frgb) || reader.TryGetField("rgba", out frgb);
        var cloud = new PointCloud(reader.Count)
        {
            FrameId = message.Header.FrameId,
            Stamp = RosTime.FromMsg(message.Header.Stamp),
        };
        if (hasI)
        {
            cloud.WithIntensity();
        }

        if (hasRgb)
        {
            cloud.WithRgb();
        }

        for (var i = 0; i < reader.Count; i++)
        {
            var p = reader.Point(i);
            var x = (float)fx.Read(p);
            var y = (float)fy.Read(p);
            var z = (float)fz.Read(p);
            if (skipNonFinite && !(float.IsFinite(x) && float.IsFinite(y) && float.IsFinite(z)))
            {
                continue;
            }

            var intensity = hasI ? (float)fi.Read(p) : 0f;
            var rgb = hasRgb ? (frgb.Datatype is PointField.FLOAT32 or PointField.UINT32 or PointField.INT32 ? frgb.ReadBits32(p) : (uint)frgb.Read(p)) & 0x00FFFFFF : 0u;
            cloud.Add(x, y, z, intensity, rgb);
        }

        return cloud;
    }
}

/// <summary>Builds <c>sensor_msgs/PointCloud2</c> messages with arbitrary field layouts.</summary>
public sealed class PointCloud2Writer
{
    private readonly List<PointField> _fields = [];
    private int _pointStep;

    /// <summary>Appends a field at the next naturally aligned offset.</summary>
    public PointCloud2Writer AddField(string name, byte datatype, int count = 1)
    {
        var size = PointFieldAccessor.SizeOf(datatype);
        _pointStep = (_pointStep + size - 1) / size * size;
        _fields.Add(new PointField { Name = name, Offset = (uint)_pointStep, Datatype = datatype, Count = (uint)count });
        _pointStep += size * count;
        return this;
    }

    /// <summary>Inserts padding bytes (e.g. PCL's 16-byte aligned xyz + pad).</summary>
    public PointCloud2Writer Pad(int bytes)
    {
        _pointStep += bytes;
        return this;
    }

    public int PointStep => _pointStep;

    public bool BigEndian { get; set; }

    /// <summary>Accessor of a declared field, for <see cref="Create"/>d data.</summary>
    public PointFieldAccessor Field(string name)
    {
        var f = _fields.First(f => f.Name == name);
        return new PointFieldAccessor(f.Name, (int)f.Offset, f.Datatype, (int)f.Count, BigEndian);
    }

    /// <summary>Allocates an unorganized (height 1) or organized cloud message; fill it with <see cref="Field"/> accessors.</summary>
    public PointCloud2 Create(int width, int height = 1, string frameId = "", RosTime? stamp = null, bool isDense = true)
    {
        var msg = new PointCloud2
        {
            Width = (uint)width,
            Height = (uint)height,
            Fields = _fields.Select(f => f.Clone()).ToArray(),
            IsBigendian = BigEndian,
            PointStep = (uint)_pointStep,
            RowStep = (uint)(_pointStep * width),
            Data = new byte[_pointStep * width * height],
            IsDense = isDense,
        };
        msg.Header.FrameId = frameId;
        (stamp ?? RosTime.Zero).CopyTo(msg.Header.Stamp);
        return msg;
    }

    /// <summary>Converts a <see cref="PointCloud"/> to PointCloud2: float32 x,y,z, plus intensity (float32) and rgb (packed float32) when present.</summary>
    public static PointCloud2 FromPointCloud(PointCloud cloud, string? frameId = null, RosTime? stamp = null)
    {
        var w = new PointCloud2Writer().AddField("x", PointField.FLOAT32).AddField("y", PointField.FLOAT32).AddField("z", PointField.FLOAT32);
        if (cloud.HasIntensity)
        {
            w.AddField("intensity", PointField.FLOAT32);
        }

        if (cloud.HasRgb)
        {
            w.AddField("rgb", PointField.FLOAT32);
        }

        var xyz = cloud.Xyz;
        var dense = true;
        for (var i = 0; i < xyz.Length; i++)
        {
            if (!float.IsFinite(xyz[i]))
            {
                dense = false;
                break;
            }
        }

        var msg = w.Create(cloud.Count, 1, frameId ?? cloud.FrameId, stamp ?? cloud.Stamp, dense);
        var step = w.PointStep;
        var data = msg.Data.AsSpan();
        var intensity = cloud.Intensity;
        var rgb = cloud.Rgb;
        for (var i = 0; i < cloud.Count; i++)
        {
            var p = data.Slice(i * step, step);
            BinaryPrimitives.WriteSingleLittleEndian(p, xyz[3 * i]);
            BinaryPrimitives.WriteSingleLittleEndian(p[4..], xyz[(3 * i) + 1]);
            BinaryPrimitives.WriteSingleLittleEndian(p[8..], xyz[(3 * i) + 2]);
            var off = 12;
            if (cloud.HasIntensity)
            {
                BinaryPrimitives.WriteSingleLittleEndian(p[off..], intensity[i]);
                off += 4;
            }

            if (cloud.HasRgb)
            {
                BinaryPrimitives.WriteUInt32LittleEndian(p[off..], rgb[i]);
            }
        }

        return msg;
    }
}
