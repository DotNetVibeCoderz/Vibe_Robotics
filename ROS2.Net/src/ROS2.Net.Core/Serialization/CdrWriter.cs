using System.Buffers;
using System.Buffers.Binary;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace ROS2.Net.Serialization;

/// <summary>
/// Writes OMG CDR (XCDR1, little-endian), the encoding every ROS 2 DDS middleware uses for messages.
/// </summary>
/// <remarks>
/// The writer owns a pooled buffer and is meant to be reused: call <see cref="Reset"/> before each
/// message. Publishers keep one writer per thread, so steady-state publishing allocates nothing.
/// Alignment is relative to the end of the 4-byte encapsulation header, as the CDR spec requires.
/// </remarks>
public sealed class CdrWriter : IDisposable
{
    /// <summary>Encapsulation identifier for little-endian plain CDR.</summary>
    public const ushort CdrLittleEndian = 0x0001;

    private byte[] _buffer;
    private int _position;
    private int _origin;

    public CdrWriter(int initialCapacity = 256)
    {
        _buffer = ArrayPool<byte>.Shared.Rent(Math.Max(16, initialCapacity));
        Reset();
    }

    /// <summary>Bytes written so far, including the encapsulation header.</summary>
    public int Length => _position;

    /// <summary>The encoded message.</summary>
    public ReadOnlySpan<byte> WrittenSpan => _buffer.AsSpan(0, _position);

    /// <summary>The encoded message. Valid until the next write or <see cref="Reset"/>.</summary>
    public ReadOnlyMemory<byte> WrittenMemory => _buffer.AsMemory(0, _position);

    /// <summary>Starts a new message.</summary>
    /// <param name="writeEncapsulation">Write the 4-byte CDR_LE encapsulation header. Nested structures inside RTPS submessages omit it.</param>
    public void Reset(bool writeEncapsulation = true)
    {
        _position = 0;
        if (writeEncapsulation)
        {
            _buffer[0] = 0x00;
            _buffer[1] = 0x01;
            _buffer[2] = 0x00;
            _buffer[3] = 0x00;
            _position = 4;
        }

        _origin = _position;
    }

    /// <summary>Copies the encoded message into a new array.</summary>
    public byte[] ToArray() => WrittenSpan.ToArray();

    public void WriteBool(bool value)
    {
        Ensure(1);
        _buffer[_position++] = value ? (byte)1 : (byte)0;
    }

    public void WriteByte(byte value)
    {
        Ensure(1);
        _buffer[_position++] = value;
    }

    public void WriteSByte(sbyte value) => WriteByte(unchecked((byte)value));

    public void WriteInt16(short value)
    {
        Align(2);
        Ensure(2);
        BinaryPrimitives.WriteInt16LittleEndian(_buffer.AsSpan(_position), value);
        _position += 2;
    }

    public void WriteUInt16(ushort value)
    {
        Align(2);
        Ensure(2);
        BinaryPrimitives.WriteUInt16LittleEndian(_buffer.AsSpan(_position), value);
        _position += 2;
    }

    public void WriteInt32(int value)
    {
        Align(4);
        Ensure(4);
        BinaryPrimitives.WriteInt32LittleEndian(_buffer.AsSpan(_position), value);
        _position += 4;
    }

    public void WriteUInt32(uint value)
    {
        Align(4);
        Ensure(4);
        BinaryPrimitives.WriteUInt32LittleEndian(_buffer.AsSpan(_position), value);
        _position += 4;
    }

    public void WriteInt64(long value)
    {
        Align(8);
        Ensure(8);
        BinaryPrimitives.WriteInt64LittleEndian(_buffer.AsSpan(_position), value);
        _position += 8;
    }

    public void WriteUInt64(ulong value)
    {
        Align(8);
        Ensure(8);
        BinaryPrimitives.WriteUInt64LittleEndian(_buffer.AsSpan(_position), value);
        _position += 8;
    }

    public void WriteSingle(float value)
    {
        Align(4);
        Ensure(4);
        BinaryPrimitives.WriteSingleLittleEndian(_buffer.AsSpan(_position), value);
        _position += 4;
    }

    public void WriteDouble(double value)
    {
        Align(8);
        Ensure(8);
        BinaryPrimitives.WriteDoubleLittleEndian(_buffer.AsSpan(_position), value);
        _position += 8;
    }

    /// <summary>Writes a CDR string: UTF-8 bytes with a uint32 length that counts the trailing NUL.</summary>
    public void WriteString(string? value)
    {
        value ??= string.Empty;
        var byteCount = Encoding.UTF8.GetByteCount(value);
        WriteUInt32((uint)(byteCount + 1));
        Ensure(byteCount + 1);
        Encoding.UTF8.GetBytes(value, _buffer.AsSpan(_position));
        _position += byteCount;
        _buffer[_position++] = 0;
    }

    /// <summary>Writes a wide string as a uint32 code-unit count followed by UTF-16LE code units (no terminator).</summary>
    public void WriteWString(string? value)
    {
        value ??= string.Empty;
        WriteUInt32((uint)value.Length);
        Ensure(value.Length * 2);
        foreach (var c in value)
        {
            BinaryPrimitives.WriteUInt16LittleEndian(_buffer.AsSpan(_position), c);
            _position += 2;
        }
    }

    /// <summary>Writes a fixed-size primitive array (no length prefix). Throws if the array has the wrong length.</summary>
    public void WriteArray<T>(T[]? values, int length, string field)
        where T : unmanaged
    {
        var span = values ?? [];
        CheckFixedLength(span.Length, length, field);
        WriteBlock<T>(span);
    }

    /// <summary>Writes a primitive sequence: uint32 count followed by the elements.</summary>
    /// <param name="values">Elements; null is written as an empty sequence.</param>
    /// <param name="bound">Upper bound for bounded sequences, or 0 when unbounded.</param>
    /// <param name="field">Field name, for error messages.</param>
    public void WriteSequence<T>(T[]? values, int bound, string field)
        where T : unmanaged
    {
        var span = values ?? [];
        WriteSequenceLength(span.Length, bound, field);
        WriteBlock<T>(span);
    }

    /// <summary>Writes a primitive sequence from a span: uint32 count followed by the elements.</summary>
    public void WriteSequence<T>(ReadOnlySpan<T> values)
        where T : unmanaged
    {
        WriteUInt32((uint)values.Length);
        WriteBlock(values);
    }

    /// <summary>Writes the uint32 element count of a sequence, enforcing its bound.</summary>
    public void WriteSequenceLength(int count, int bound, string field)
    {
        if (bound > 0 && count > bound)
        {
            throw new CdrException($"Field '{field}' has {count} elements but is bounded to {bound}.");
        }

        WriteUInt32((uint)count);
    }

    /// <summary>Throws when a fixed-size array field does not hold exactly <paramref name="expected"/> elements.</summary>
    public void CheckFixedLength(int actual, int expected, string field)
    {
        if (actual != expected)
        {
            throw new CdrException($"Field '{field}' is a fixed array of {expected} elements but holds {actual}.");
        }
    }

    /// <summary>Appends raw bytes without alignment.</summary>
    public void WriteRaw(ReadOnlySpan<byte> bytes)
    {
        Ensure(bytes.Length);
        bytes.CopyTo(_buffer.AsSpan(_position));
        _position += bytes.Length;
    }

    /// <summary>Pads to an <paramref name="alignment"/>-byte boundary relative to the CDR origin.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Align(int alignment)
    {
        var misalignment = (_position - _origin) & (alignment - 1);
        if (misalignment != 0)
        {
            var pad = alignment - misalignment;
            Ensure(pad);
            _buffer.AsSpan(_position, pad).Clear();
            _position += pad;
        }
    }

    private void WriteBlock<T>(ReadOnlySpan<T> values)
        where T : unmanaged
    {
        if (values.IsEmpty)
        {
            return;
        }

        var size = Unsafe.SizeOf<T>();
        Align(size);
        var bytes = MemoryMarshal.AsBytes(values);
        Ensure(bytes.Length);
        var dest = _buffer.AsSpan(_position, bytes.Length);
        bytes.CopyTo(dest);
        if (!BitConverter.IsLittleEndian && size > 1)
        {
            ReverseEach(dest, size);
        }

        _position += bytes.Length;
    }

    internal static void ReverseEach(Span<byte> data, int size)
    {
        for (var i = 0; i < data.Length; i += size)
        {
            data.Slice(i, size).Reverse();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Ensure(int additional)
    {
        if (_position + additional > _buffer.Length)
        {
            Grow(additional);
        }
    }

    private void Grow(int additional)
    {
        var next = ArrayPool<byte>.Shared.Rent(Math.Max(_buffer.Length * 2, _position + additional));
        _buffer.AsSpan(0, _position).CopyTo(next);
        ArrayPool<byte>.Shared.Return(_buffer);
        _buffer = next;
    }

    public void Dispose()
    {
        var buffer = _buffer;
        _buffer = [];
        if (buffer.Length > 0)
        {
            ArrayPool<byte>.Shared.Return(buffer);
        }
    }

    [ThreadStatic]
    private static CdrWriter? t_cached;

    /// <summary>
    /// Serializes <paramref name="message"/> with a per-thread cached writer and hands the bytes to
    /// <paramref name="consume"/>. The span is only valid during the callback.
    /// </summary>
    public static void SerializeWith<TState>(IRosMessage message, TState state, ReadOnlySpanAction<byte, TState> consume)
    {
        var writer = t_cached ??= new CdrWriter(1024);
        t_cached = null; // Re-entrancy: a nested serialize on the same thread gets its own writer.
        try
        {
            writer.Reset();
            message.Serialize(writer);
            consume(writer.WrittenSpan, state);
        }
        finally
        {
            t_cached = writer;
        }
    }

    /// <summary>Serializes a message into a new array (header included).</summary>
    public static byte[] Serialize(IRosMessage message)
    {
        using var writer = new CdrWriter(512);
        message.Serialize(writer);
        return writer.ToArray();
    }
}

/// <summary>Raised for malformed or out-of-contract CDR data.</summary>
public sealed class CdrException(string message) : Exception(message);
