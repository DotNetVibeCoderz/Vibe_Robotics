using System.Buffers.Binary;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace ROS2.Net.Serialization;

/// <summary>
/// Reads OMG CDR (XCDR1) in either byte order. A <see langword="ref struct"/> over the received bytes,
/// so decoding never copies the payload; every length is validated against the remaining data.
/// </summary>
public ref struct CdrReader
{
    private readonly ReadOnlySpan<byte> _data;
    private readonly int _origin;
    private readonly bool _bigEndian;
    private int _position;

    /// <param name="data">Encoded message.</param>
    /// <param name="hasEncapsulation">Whether <paramref name="data"/> starts with the 4-byte encapsulation header.</param>
    public CdrReader(ReadOnlySpan<byte> data, bool hasEncapsulation = true)
    {
        _data = data;
        _position = 0;
        _bigEndian = false;
        if (hasEncapsulation)
        {
            if (data.Length < 4)
            {
                throw new CdrException("CDR payload is shorter than its encapsulation header.");
            }

            // Byte 1 carries the representation: 0x00 CDR_BE, 0x01 CDR_LE (also PL_CDR_* 0x02/0x03).
            _bigEndian = (data[1] & 0x01) == 0;
            _position = 4;
        }

        _origin = _position;
    }

    /// <summary>Creates a reader over headerless data with an explicit byte order.</summary>
    public CdrReader(ReadOnlySpan<byte> data, bool bigEndian, int origin)
    {
        _data = data;
        _bigEndian = bigEndian;
        _origin = origin;
        _position = origin;
    }

    public readonly int Position => _position;

    public readonly int Remaining => _data.Length - _position;

    public readonly bool IsBigEndian => _bigEndian;

    public bool ReadBool() => ReadByte() != 0;

    public byte ReadByte()
    {
        Need(1);
        return _data[_position++];
    }

    public sbyte ReadSByte() => unchecked((sbyte)ReadByte());

    public short ReadInt16()
    {
        var s = Take(2);
        return _bigEndian ? BinaryPrimitives.ReadInt16BigEndian(s) : BinaryPrimitives.ReadInt16LittleEndian(s);
    }

    public ushort ReadUInt16()
    {
        var s = Take(2);
        return _bigEndian ? BinaryPrimitives.ReadUInt16BigEndian(s) : BinaryPrimitives.ReadUInt16LittleEndian(s);
    }

    public int ReadInt32()
    {
        var s = Take(4);
        return _bigEndian ? BinaryPrimitives.ReadInt32BigEndian(s) : BinaryPrimitives.ReadInt32LittleEndian(s);
    }

    public uint ReadUInt32()
    {
        var s = Take(4);
        return _bigEndian ? BinaryPrimitives.ReadUInt32BigEndian(s) : BinaryPrimitives.ReadUInt32LittleEndian(s);
    }

    public long ReadInt64()
    {
        var s = Take(8);
        return _bigEndian ? BinaryPrimitives.ReadInt64BigEndian(s) : BinaryPrimitives.ReadInt64LittleEndian(s);
    }

    public ulong ReadUInt64()
    {
        var s = Take(8);
        return _bigEndian ? BinaryPrimitives.ReadUInt64BigEndian(s) : BinaryPrimitives.ReadUInt64LittleEndian(s);
    }

    public float ReadSingle()
    {
        var s = Take(4);
        return _bigEndian ? BinaryPrimitives.ReadSingleBigEndian(s) : BinaryPrimitives.ReadSingleLittleEndian(s);
    }

    public double ReadDouble()
    {
        var s = Take(8);
        return _bigEndian ? BinaryPrimitives.ReadDoubleBigEndian(s) : BinaryPrimitives.ReadDoubleLittleEndian(s);
    }

    /// <summary>Reads a CDR string. Tolerates a missing NUL terminator, which some implementations emit for empty strings.</summary>
    public string ReadString()
    {
        var length = ReadUInt32();
        if (length == 0)
        {
            return string.Empty;
        }

        if (length > Remaining)
        {
            throw new CdrException($"String length {length} exceeds the {Remaining} remaining bytes.");
        }

        var bytes = _data.Slice(_position, (int)length);
        _position += (int)length;
        if (bytes[^1] == 0)
        {
            bytes = bytes[..^1];
        }

        return bytes.IsEmpty ? string.Empty : Encoding.UTF8.GetString(bytes);
    }

    /// <summary>Reads a wide string written by <see cref="CdrWriter.WriteWString"/>.</summary>
    public string ReadWString()
    {
        var length = (int)ReadUInt32();
        if ((long)length * 2 > Remaining)
        {
            throw new CdrException($"Wide string length {length} exceeds the remaining data.");
        }

        var chars = length <= 256 ? stackalloc char[length] : new char[length];
        for (var i = 0; i < length; i++)
        {
            var s = _data.Slice(_position, 2);
            chars[i] = (char)(_bigEndian ? BinaryPrimitives.ReadUInt16BigEndian(s) : BinaryPrimitives.ReadUInt16LittleEndian(s));
            _position += 2;
        }

        return new string(chars);
    }

    /// <summary>Reads a sequence length and checks it against the remaining data.</summary>
    /// <param name="minElementSize">Smallest possible encoded size of one element; guards against huge bogus counts.</param>
    public int ReadSequenceLength(int minElementSize)
    {
        var count = ReadUInt32();
        if ((long)count * Math.Max(1, minElementSize) > Remaining)
        {
            throw new CdrException($"Sequence of {count} elements cannot fit in {Remaining} remaining bytes.");
        }

        return (int)count;
    }

    /// <summary>Reads a fixed-size primitive array, reusing <paramref name="existing"/> when it already has the right length.</summary>
    public T[] ReadArray<T>(T[]? existing, int length)
        where T : unmanaged
    {
        var result = existing is not null && existing.Length == length ? existing : new T[length];
        ReadBlock<T>(result);
        return result;
    }

    /// <summary>Reads a primitive sequence, reusing <paramref name="existing"/> when the length matches.</summary>
    public T[] ReadSequence<T>(T[]? existing)
        where T : unmanaged
    {
        var count = ReadSequenceLength(Unsafe.SizeOf<T>());
        if (count == 0)
        {
            return existing is { Length: 0 } ? existing : [];
        }

        var result = existing is not null && existing.Length == count ? existing : new T[count];
        ReadBlock<T>(result);
        return result;
    }

    /// <summary>Reads primitive elements into <paramref name="destination"/>.</summary>
    public void ReadBlock<T>(Span<T> destination)
        where T : unmanaged
    {
        if (destination.IsEmpty)
        {
            return;
        }

        var size = Unsafe.SizeOf<T>();
        Align(size);
        var bytes = MemoryMarshal.AsBytes(destination);
        Need(bytes.Length);
        _data.Slice(_position, bytes.Length).CopyTo(bytes);
        _position += bytes.Length;
        if (size > 1 && _bigEndian == BitConverter.IsLittleEndian)
        {
            CdrWriter.ReverseEach(bytes, size);
        }
    }

    /// <summary>Returns a slice of raw bytes without alignment.</summary>
    public ReadOnlySpan<byte> ReadRaw(int length)
    {
        Need(length);
        var s = _data.Slice(_position, length);
        _position += length;
        return s;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Align(int alignment)
    {
        var misalignment = (_position - _origin) & (alignment - 1);
        if (misalignment != 0)
        {
            _position += alignment - misalignment;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ReadOnlySpan<byte> Take(int size)
    {
        Align(size);
        Need(size);
        var s = _data.Slice(_position, size);
        _position += size;
        return s;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private readonly void Need(int size)
    {
        if (_position + size > _data.Length)
        {
            throw new CdrException($"Unexpected end of CDR data: needed {size} bytes at offset {_position}, have {_data.Length - _position}.");
        }
    }

    /// <summary>Decodes a message from CDR bytes (with encapsulation header).</summary>
    public static T Deserialize<T>(ReadOnlySpan<byte> data, T? reuse = null)
        where T : class, IRosMessage, new()
    {
        var message = reuse ?? new T();
        var reader = new CdrReader(data);
        message.Deserialize(ref reader);
        return message;
    }
}
