using System.Globalization;
using System.Net;
using System.Runtime.CompilerServices;

namespace ROS2.Net.Rmw.Rtps;

/// <summary>How service requests are correlated with replies on the wire.</summary>
public enum ServiceMapping
{
    /// <summary>
    /// rmw_fastrtps style: the reply carries the request's sample identity (request writer GUID +
    /// sequence number) in inline QoS <c>PID_RELATED_SAMPLE_IDENTITY</c> (0x800f, plus the standard 0x0083).
    /// </summary>
    FastDds,

    /// <summary>
    /// rmw_cyclonedds style: request and reply payloads are prefixed (inside the CDR stream) with a
    /// 16-byte header <c>{ uint64 client_guid; int64 sequence_number; }</c>.
    /// </summary>
    CycloneDds,
}

/// <summary>Which ROS 2 distribution's <c>rmw_dds_common</c> layout to use for <c>ros_discovery_info</c>.</summary>
public enum RosDistro
{
    /// <summary>Humble and Iron: <c>rmw_dds_common/msg/Gid</c> is <c>uint8[24]</c>.</summary>
    Humble,

    /// <summary>Jazzy and later: <c>rmw_dds_common/msg/Gid</c> is <c>uint8[16]</c>.</summary>
    Jazzy,
}

/// <summary>Settings of the managed RTPS middleware.</summary>
public sealed class RtpsOptions
{
    /// <summary>Only talk to participants on this machine (binds and announces 127.0.0.1, no multicast).</summary>
    public bool LocalhostOnly { get; set; }

    /// <summary>Use SPDP multicast (239.255.0.1). Disabled automatically with <see cref="LocalhostOnly"/>.</summary>
    public bool UseMulticast { get; set; } = true;

    /// <summary>Restrict traffic to this local IPv4 address. Null means every up, non-loopback interface.</summary>
    public IPAddress? InterfaceAddress { get; set; }

    /// <summary>SPDP multicast group.</summary>
    public IPAddress MulticastGroup { get; set; } = IPAddress.Parse("239.255.0.1");

    /// <summary>Participants on this host are also discovered by unicasting SPDP to participant ids 0..N-1 on 127.0.0.1.</summary>
    public int LocalPeerParticipantIds { get; set; } = 10;

    /// <summary>Additional unicast discovery peers (<c>host</c> or <c>host:port</c>), like Fast DDS initial peers.</summary>
    public List<string> InitialPeers { get; private set; } = [];

    public TimeSpan ParticipantLeaseDuration { get; set; } = TimeSpan.FromSeconds(20);

    /// <summary>Steady-state SPDP announcement period (the first few announcements are faster).</summary>
    public TimeSpan AnnouncementPeriod { get; set; } = TimeSpan.FromSeconds(3);

    /// <summary>Heartbeat period of reliable writers while samples are unacknowledged.</summary>
    public TimeSpan HeartbeatPeriod { get; set; } = TimeSpan.FromMilliseconds(100);

    /// <summary>Payloads larger than this are sent as DATA_FRAG fragments of this size.</summary>
    public int MaxFragmentSize { get; set; } = 60_000;

    /// <summary>OS socket buffer size; large values avoid drops on bursts of fragments.</summary>
    public int SocketBufferSize { get; set; } = 4 * 1024 * 1024;

    public ServiceMapping ServiceMapping { get; set; } = ServiceMapping.FastDds;

    public RosDistro Distro { get; set; } = RosDistro.Jazzy;

    /// <summary>Bytes per <c>rmw_dds_common/msg/Gid</c> in <c>ros_discovery_info</c>.</summary>
    public int GidSize => Distro == RosDistro.Humble ? 24 : 16;

    /// <summary>Publish and consume <c>ros_discovery_info</c> so ROS tools map endpoints to nodes.</summary>
    public bool PublishDiscoveryInfo { get; set; } = true;

    /// <summary>Applies <c>key=value</c> settings from <see cref="Ros2ContextOptions.RmwSettings"/>.</summary>
    public void Apply(IReadOnlyDictionary<string, string> settings)
    {
        foreach (var (key, value) in settings)
        {
            switch (key.ToLowerInvariant())
            {
                case "localhost" or "localhost_only":
                    LocalhostOnly = bool.Parse(value);
                    break;
                case "multicast":
                    UseMulticast = bool.Parse(value);
                    break;
                case "interface":
                    InterfaceAddress = IPAddress.Parse(value);
                    break;
                case "service_mapping":
                    ServiceMapping = value.Contains("cyclone", StringComparison.OrdinalIgnoreCase) ? ServiceMapping.CycloneDds : ServiceMapping.FastDds;
                    break;
                case "distro":
                    Distro = value.Equals("humble", StringComparison.OrdinalIgnoreCase) || value.Equals("iron", StringComparison.OrdinalIgnoreCase) ? RosDistro.Humble : RosDistro.Jazzy;
                    break;
                case "gid_size":
                    Distro = int.Parse(value, CultureInfo.InvariantCulture) == 24 ? RosDistro.Humble : RosDistro.Jazzy;
                    break;
                case "initial_peers":
                    InitialPeers.AddRange(value.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries));
                    break;
                case "max_fragment_size":
                    MaxFragmentSize = int.Parse(value, CultureInfo.InvariantCulture);
                    break;
                case "lease_seconds":
                    ParticipantLeaseDuration = TimeSpan.FromSeconds(double.Parse(value, CultureInfo.InvariantCulture));
                    break;
            }
        }
    }

    public RtpsOptions Clone()
    {
        var c = (RtpsOptions)MemberwiseClone();
        c.InitialPeers = [.. InitialPeers];
        return c;
    }
}

/// <summary>Creates <see cref="RtpsRmw"/> instances (identifier <c>rtps</c>).</summary>
public sealed class RtpsRmwFactory(RtpsOptions? options = null) : IRmwFactory
{
    private readonly RtpsOptions _options = options ?? new RtpsOptions();

    public string Identifier => "rtps";

    public async ValueTask<IRmwImplementation> CreateAsync(RmwContextOptions options, CancellationToken cancellationToken)
    {
        var o = _options.Clone();
        o.Apply(options.Settings);
        if (options.LocalhostOnly)
        {
            o.LocalhostOnly = true;
        }

        var rmw = new RtpsRmw(options.DomainId, options.Enclave, o, options.LoggerFactory);
        await rmw.StartAsync(cancellationToken).ConfigureAwait(false);
        return rmw;
    }

#pragma warning disable CA2255 // Registering the middleware when the assembly loads is the intent.
    [ModuleInitializer]
#pragma warning restore CA2255
    internal static void Register() => RmwRegistry.Register(new RtpsRmwFactory());
}

/// <summary>Registration helpers.</summary>
public static class RtpsExtensions
{
    /// <summary>Selects the managed RTPS middleware for a context.</summary>
    public static Ros2ContextOptions UseRtps(this Ros2ContextOptions options, Action<RtpsOptions>? configure = null)
    {
        var o = new RtpsOptions();
        configure?.Invoke(o);
        options.Rmw = new RtpsRmwFactory(o);
        return options;
    }
}
