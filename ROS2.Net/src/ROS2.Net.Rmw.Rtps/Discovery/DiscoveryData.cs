using System.Text;
using ROS2.Net.Rmw.Rtps.Wire;

namespace ROS2.Net.Rmw.Rtps.Discovery;

/// <summary>Bits of <c>PID_BUILTIN_ENDPOINT_SET</c>.</summary>
[Flags]
public enum BuiltinEndpoints : uint
{
    None = 0,
    ParticipantAnnouncer = 1u << 0,
    ParticipantDetector = 1u << 1,
    PublicationsAnnouncer = 1u << 2,
    PublicationsDetector = 1u << 3,
    SubscriptionsAnnouncer = 1u << 4,
    SubscriptionsDetector = 1u << 5,
    ParticipantMessageWriter = 1u << 10,
    ParticipantMessageReader = 1u << 11,
}

/// <summary>SPDP participant announcement (<c>SPDPdiscoveredParticipantData</c>).</summary>
public sealed class ParticipantData
{
    public RtpsGuid Guid { get; set; }

    public VendorId Vendor { get; set; } = VendorId.Ros2Net;

    public byte ProtocolMajor { get; set; } = 2;

    public byte ProtocolMinor { get; set; } = 3;

    public int? DomainId { get; set; }

    public List<Locator> MetatrafficUnicast { get; } = [];

    public List<Locator> MetatrafficMulticast { get; } = [];

    public List<Locator> DefaultUnicast { get; } = [];

    public List<Locator> DefaultMulticast { get; } = [];

    public TimeSpan LeaseDuration { get; set; } = TimeSpan.FromSeconds(20);

    public BuiltinEndpoints BuiltinEndpoints { get; set; }

    /// <summary>USER_DATA; ROS 2 puts <c>enclave=/;</c> here.</summary>
    public byte[] UserData { get; set; } = [];

    public string? EntityName { get; set; }

    /// <summary>The enclave parsed from <see cref="UserData"/>.</summary>
    public string Enclave
    {
        get
        {
            var text = Encoding.UTF8.GetString(UserData);
            foreach (var part in text.Split(';', StringSplitOptions.RemoveEmptyEntries))
            {
                if (part.StartsWith("enclave=", StringComparison.Ordinal))
                {
                    return part["enclave=".Length..];
                }
            }

            return "/";
        }
    }

    /// <summary>Serializes as a PL_CDR_LE payload.</summary>
    public void Write(RtpsBuffer buffer)
    {
        var w = new ParameterListWriter(buffer);
        w.WriteEncapsulation();
        w.WriteRaw(ParameterId.ProtocolVersion, [ProtocolMajor, ProtocolMinor, 0, 0]);
        w.WriteRaw(ParameterId.VendorId, [Vendor.Major, Vendor.Minor, 0, 0]);
        w.WriteGuid(ParameterId.ParticipantGuid, Guid);
        if (DomainId is { } d)
        {
            w.WriteUInt32(ParameterId.DomainId, (uint)d);
        }

        foreach (var l in MetatrafficUnicast)
        {
            w.WriteLocator(ParameterId.MetatrafficUnicastLocator, l);
        }

        foreach (var l in MetatrafficMulticast)
        {
            w.WriteLocator(ParameterId.MetatrafficMulticastLocator, l);
        }

        foreach (var l in DefaultUnicast)
        {
            w.WriteLocator(ParameterId.DefaultUnicastLocator, l);
        }

        foreach (var l in DefaultMulticast)
        {
            w.WriteLocator(ParameterId.DefaultMulticastLocator, l);
        }

        w.WriteDuration(ParameterId.ParticipantLeaseDuration, LeaseDuration);
        w.WriteUInt32(ParameterId.BuiltinEndpointSet, (uint)BuiltinEndpoints);
        if (UserData.Length > 0)
        {
            w.WriteOctets(ParameterId.UserData, UserData);
        }

        if (EntityName is not null)
        {
            w.WriteString(ParameterId.EntityName, EntityName);
        }

        w.WriteSentinel();
    }

    public static ParticipantData Read(ParameterList pl)
    {
        var data = new ParticipantData
        {
            Guid = pl.GetGuid(ParameterId.ParticipantGuid) ?? throw new RtpsFormatException("SPDP data without PID_PARTICIPANT_GUID."),
            DomainId = pl.GetInt32(ParameterId.DomainId),
            LeaseDuration = pl.GetDuration(ParameterId.ParticipantLeaseDuration, TimeSpan.FromSeconds(100)) ?? TimeSpan.FromDays(1),
            BuiltinEndpoints = (BuiltinEndpoints)(pl.GetUInt32(ParameterId.BuiltinEndpointSet) ?? 0),
            UserData = pl.GetOctets(ParameterId.UserData) ?? [],
            EntityName = pl.GetString(ParameterId.EntityName),
        };
        if (pl.TryGet(ParameterId.VendorId, out var v) && v.Length >= 2)
        {
            data.Vendor = new VendorId(v[0], v[1]);
        }

        if (pl.TryGet(ParameterId.ProtocolVersion, out var pv) && pv.Length >= 2)
        {
            data.ProtocolMajor = pv[0];
            data.ProtocolMinor = pv[1];
        }

        data.MetatrafficUnicast.AddRange(pl.GetLocators(ParameterId.MetatrafficUnicastLocator));
        data.MetatrafficMulticast.AddRange(pl.GetLocators(ParameterId.MetatrafficMulticastLocator));
        data.DefaultUnicast.AddRange(pl.GetLocators(ParameterId.DefaultUnicastLocator));
        data.DefaultMulticast.AddRange(pl.GetLocators(ParameterId.DefaultMulticastLocator));
        return data;
    }
}

/// <summary>SEDP endpoint announcement (<c>DiscoveredWriterData</c> / <c>DiscoveredReaderData</c>).</summary>
public sealed class EndpointData
{
    public RtpsGuid Guid { get; set; }

    public RtpsGuid ParticipantGuid { get; set; }

    public string TopicName { get; set; } = string.Empty;

    public string TypeName { get; set; } = string.Empty;

    public QosProfile Qos { get; set; } = QosProfile.Default;

    public List<Locator> UnicastLocators { get; } = [];

    public List<Locator> MulticastLocators { get; } = [];

    public bool ExpectsInlineQos { get; set; }

    public byte[] UserData { get; set; } = [];

    public VendorId Vendor { get; set; } = VendorId.Ros2Net;

    public bool IsWriter => Guid.EntityId.IsWriter;

    public void Write(RtpsBuffer buffer)
    {
        var w = new ParameterListWriter(buffer);
        w.WriteEncapsulation();
        w.WriteGuid(ParameterId.EndpointGuid, Guid);
        w.WriteGuid(ParameterId.ParticipantGuid, ParticipantGuid);
        w.WriteString(ParameterId.TopicName, TopicName);
        w.WriteString(ParameterId.TypeName, TypeName);
        w.WriteRaw(ParameterId.ProtocolVersion, [2, 3, 0, 0]);
        w.WriteRaw(ParameterId.VendorId, [Vendor.Major, Vendor.Minor, 0, 0]);

        var q = Qos.Resolve();
        w.Begin(ParameterId.Reliability);
        w.Buffer.WriteUInt32(q.Reliability == ReliabilityPolicy.BestEffort ? 1u : 2u);
        var (bs, bf) = RtpsTime.FromDuration(TimeSpan.FromMilliseconds(100));
        w.Buffer.WriteInt32(bs);
        w.Buffer.WriteUInt32(bf);
        w.End();

        w.WriteUInt32(ParameterId.Durability, q.Durability == DurabilityPolicy.TransientLocal ? 1u : 0u);

        w.Begin(ParameterId.History);
        w.Buffer.WriteUInt32(q.History == HistoryPolicy.KeepAll ? 1u : 0u);
        w.Buffer.WriteInt32(q.History == HistoryPolicy.KeepAll ? -1 : q.Capacity);
        w.End();

        w.WriteDuration(ParameterId.Deadline, q.Deadline);

        w.Begin(ParameterId.Liveliness);
        w.Buffer.WriteUInt32(q.Liveliness == LivelinessPolicy.ManualByTopic ? 2u : 0u);
        var (ls, lf) = RtpsTime.FromDuration(q.LivelinessLeaseDuration);
        w.Buffer.WriteInt32(ls);
        w.Buffer.WriteUInt32(lf);
        w.End();

        if (IsWriter)
        {
            w.WriteDuration(ParameterId.Lifespan, q.Lifespan);
        }

        // XCDR1 only (DataRepresentationQosPolicy: sequence<int16> { XCDR_DATA_REPRESENTATION = 0 }).
        w.Begin(ParameterId.DataRepresentation);
        w.Buffer.WriteUInt32(1);
        w.Buffer.WriteUInt16(0);
        w.End();

        foreach (var l in UnicastLocators)
        {
            w.WriteLocator(ParameterId.UnicastLocator, l);
        }

        foreach (var l in MulticastLocators)
        {
            w.WriteLocator(ParameterId.MulticastLocator, l);
        }

        if (UserData.Length > 0)
        {
            w.WriteOctets(ParameterId.UserData, UserData);
        }

        w.WriteSentinel();
    }

    public static EndpointData Read(ParameterList pl, RtpsGuid? keyFallback = null)
    {
        var guid = pl.GetGuid(ParameterId.EndpointGuid) ?? keyFallback ?? throw new RtpsFormatException("SEDP data without PID_ENDPOINT_GUID.");
        var data = new EndpointData
        {
            Guid = guid,
            ParticipantGuid = pl.GetGuid(ParameterId.ParticipantGuid) ?? new RtpsGuid(guid.Prefix, EntityId.Participant),
            TopicName = pl.GetString(ParameterId.TopicName) ?? string.Empty,
            TypeName = pl.GetString(ParameterId.TypeName) ?? string.Empty,
            ExpectsInlineQos = (pl.GetUInt32(ParameterId.ExpectsInlineQos) ?? 0) != 0,
            UserData = pl.GetOctets(ParameterId.UserData) ?? [],
        };

        // DDS defaults: writers are reliable, readers best effort; both volatile, keep last 1.
        var reliability = data.IsWriter ? ReliabilityPolicy.Reliable : ReliabilityPolicy.BestEffort;
        if (pl.GetUInt32(ParameterId.Reliability) is { } r)
        {
            reliability = r == 1 ? ReliabilityPolicy.BestEffort : ReliabilityPolicy.Reliable;
        }

        var durability = (pl.GetUInt32(ParameterId.Durability) ?? 0) >= 1 ? DurabilityPolicy.TransientLocal : DurabilityPolicy.Volatile;
        var history = HistoryPolicy.KeepLast;
        var depth = 1;
        if (pl.TryGet(ParameterId.History, out var h) && h.Length >= 8)
        {
            var hr = new SpanReader(h, pl.LittleEndian);
            history = hr.ReadUInt32() == 1 ? HistoryPolicy.KeepAll : HistoryPolicy.KeepLast;
            depth = hr.ReadInt32();
        }

        var liveliness = LivelinessPolicy.Automatic;
        TimeSpan? lease = null;
        if (pl.TryGet(ParameterId.Liveliness, out var lv) && lv.Length >= 12)
        {
            var lr = new SpanReader(lv, pl.LittleEndian);
            liveliness = lr.ReadUInt32() == 0 ? LivelinessPolicy.Automatic : LivelinessPolicy.ManualByTopic;
            lease = RtpsTime.ToDuration(lr.ReadInt32(), lr.ReadUInt32());
        }

        data.Qos = new QosProfile(Math.Max(1, depth)) with
        {
            History = history,
            Reliability = reliability,
            Durability = durability,
            Deadline = pl.GetDuration(ParameterId.Deadline, null),
            Lifespan = pl.GetDuration(ParameterId.Lifespan, null),
            Liveliness = liveliness,
            LivelinessLeaseDuration = lease,
        };

        if (pl.TryGet(ParameterId.VendorId, out var v) && v.Length >= 2)
        {
            data.Vendor = new VendorId(v[0], v[1]);
        }

        data.UnicastLocators.AddRange(pl.GetLocators(ParameterId.UnicastLocator));
        data.MulticastLocators.AddRange(pl.GetLocators(ParameterId.MulticastLocator));
        return data;
    }
}
