using System.Buffers.Binary;
using System.Security.Cryptography;

namespace ROS2.Net.Rmw.Rtps.Wire;

/// <summary>The 12-byte RTPS GUID prefix that identifies a participant.</summary>
public readonly record struct GuidPrefix(ulong High, uint Low)
{
    public static GuidPrefix Unknown => default;

    public bool IsUnknown => High == 0 && Low == 0;

    public static GuidPrefix Read(ReadOnlySpan<byte> bytes) =>
        new(BinaryPrimitives.ReadUInt64BigEndian(bytes), BinaryPrimitives.ReadUInt32BigEndian(bytes[8..]));

    public void Write(Span<byte> destination)
    {
        BinaryPrimitives.WriteUInt64BigEndian(destination, High);
        BinaryPrimitives.WriteUInt32BigEndian(destination[8..], Low);
    }

    public byte[] ToArray()
    {
        var b = new byte[12];
        Write(b);
        return b;
    }

    /// <summary>A new prefix: two vendor bytes followed by ten random bytes.</summary>
    public static GuidPrefix Create(VendorId vendor)
    {
        Span<byte> b = stackalloc byte[12];
        RandomNumberGenerator.Fill(b);
        b[0] = vendor.Major;
        b[1] = vendor.Minor;
        return Read(b);
    }

    public override string ToString() => $"{High:x16}{Low:x8}";
}

/// <summary>An RTPS entity id: a 3-byte key and a 1-byte kind, always sent big-endian.</summary>
public readonly record struct EntityId(uint Value)
{
    public static EntityId Unknown => new(0x00000000);

    public static EntityId Participant => new(0x000001c1);

    public static EntityId SedpPublicationsWriter => new(0x000003c2);

    public static EntityId SedpPublicationsReader => new(0x000003c7);

    public static EntityId SedpSubscriptionsWriter => new(0x000004c2);

    public static EntityId SedpSubscriptionsReader => new(0x000004c7);

    public static EntityId SpdpWriter => new(0x000100c2);

    public static EntityId SpdpReader => new(0x000100c7);

    public static EntityId ParticipantMessageWriter => new(0x000200c2);

    public static EntityId ParticipantMessageReader => new(0x000200c7);

    /// <summary>User writer without key.</summary>
    public const byte KindWriterNoKey = 0x03;

    /// <summary>User reader without key.</summary>
    public const byte KindReaderNoKey = 0x04;

    public byte Kind => (byte)(Value & 0xff);

    /// <summary>True for writer kinds (user or built-in).</summary>
    public bool IsWriter => (Kind & 0x0f) is 0x02 or 0x03;

    public bool IsBuiltin => (Kind & 0xc0) == 0xc0;

    public static EntityId Create(uint key, byte kind) => new((key << 8) | kind);

    public static EntityId Read(ReadOnlySpan<byte> bytes) => new(BinaryPrimitives.ReadUInt32BigEndian(bytes));

    public void Write(Span<byte> destination) => BinaryPrimitives.WriteUInt32BigEndian(destination, Value);

    public override string ToString() => Value.ToString("x8");
}

/// <summary>A full RTPS GUID: participant prefix + entity id. Maps 1:1 onto <see cref="Gid"/>.</summary>
public readonly record struct RtpsGuid(GuidPrefix Prefix, EntityId EntityId)
{
    public static RtpsGuid Unknown => default;

    public static RtpsGuid Read(ReadOnlySpan<byte> bytes) => new(GuidPrefix.Read(bytes), EntityId.Read(bytes[12..]));

    public void Write(Span<byte> destination)
    {
        Prefix.Write(destination);
        EntityId.Write(destination[12..]);
    }

    public byte[] ToArray()
    {
        var b = new byte[16];
        Write(b);
        return b;
    }

    public Gid ToGid()
    {
        Span<byte> b = stackalloc byte[16];
        Write(b);
        return Gid.FromBytes(b);
    }

    public static RtpsGuid FromGid(Gid gid)
    {
        Span<byte> b = stackalloc byte[16];
        gid.WriteTo(b);
        return Read(b);
    }

    public override string ToString() => $"{Prefix}|{EntityId}";
}

/// <summary>RTPS vendor id.</summary>
public readonly record struct VendorId(byte Major, byte Minor)
{
    /// <summary>Vendor id this implementation announces. Not registered with the OMG; peers treat it as an unknown vendor.</summary>
    public static VendorId Ros2Net => new(0x01, 0xa2);

    public static VendorId EProsima => new(0x01, 0x0f);

    public static VendorId EclipseCyclone => new(0x01, 0x10);

    public override string ToString() => $"{Major:x2}.{Minor:x2}";
}
