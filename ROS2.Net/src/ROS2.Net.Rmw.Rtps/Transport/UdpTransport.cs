using System.Buffers;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using Microsoft.Extensions.Logging;
using ROS2.Net.Rmw.Rtps.Wire;

namespace ROS2.Net.Rmw.Rtps.Transport;

/// <summary>Well-known RTPS port mapping (spec 9.6.1.1): PB=7400, DG=250, PG=2, d0=0, d1=10, d2=1, d3=11.</summary>
public static class RtpsPorts
{
    public const int PortBase = 7400;
    public const int DomainGain = 250;
    public const int ParticipantGain = 2;

    public static int SpdpMulticast(int domain) => PortBase + (DomainGain * domain);

    public static int MetatrafficUnicast(int domain, int participantId) => PortBase + (DomainGain * domain) + 10 + (ParticipantGain * participantId);

    public static int UserMulticast(int domain) => PortBase + (DomainGain * domain) + 1;

    public static int UserUnicast(int domain, int participantId) => PortBase + (DomainGain * domain) + 11 + (ParticipantGain * participantId);
}

/// <summary>
/// UDPv4 sockets of one participant: metatraffic and user unicast (ports chosen by participant id),
/// plus the shared SPDP and user multicast ports. One receive loop per socket.
/// </summary>
internal sealed class UdpTransport : IAsyncDisposable
{
    private const int SioUdpConnReset = -1744830452;
    private readonly RtpsOptions _options;
    private readonly ILogger _logger;
    private readonly List<Socket> _receivers = [];
    private readonly CancellationTokenSource _cts = new();
    private readonly List<Task> _loops = [];
    private readonly List<(IPAddress Address, IPAddress Mask)> _localAddresses = [];
    private Socket _sender = null!;

    public UdpTransport(int domainId, RtpsOptions options, ILogger logger)
    {
        DomainId = domainId;
        _options = options;
        _logger = logger;
    }

    public int DomainId { get; }

    public int ParticipantId { get; private set; } = -1;

    public List<Locator> MetatrafficUnicast { get; } = [];

    public List<Locator> MetatrafficMulticast { get; } = [];

    public List<Locator> UserUnicast { get; } = [];

    public List<Locator> UserMulticast { get; } = [];

    /// <summary>SPDP destinations: multicast group plus local and configured unicast peers.</summary>
    public List<IPEndPoint> DiscoveryPeers { get; } = [];

    public bool Multicast => _options.UseMulticast && !_options.LocalhostOnly;

    public void Open()
    {
        DiscoverAddresses();
        var bindAddress = _options.LocalhostOnly ? IPAddress.Loopback : IPAddress.Any;

        Socket? meta = null, user = null;
        for (var pid = 0; pid < 120 && user is null; pid++)
        {
            meta = TryBind(bindAddress, RtpsPorts.MetatrafficUnicast(DomainId, pid), reuse: false);
            if (meta is null)
            {
                continue;
            }

            user = TryBind(bindAddress, RtpsPorts.UserUnicast(DomainId, pid), reuse: false);
            if (user is null)
            {
                meta.Dispose();
                meta = null;
                continue;
            }

            ParticipantId = pid;
        }

        if (meta is null || user is null)
        {
            throw new InvalidOperationException($"No free RTPS participant id for domain {DomainId} (ports {RtpsPorts.MetatrafficUnicast(DomainId, 0)}+ are in use).");
        }

        _receivers.Add(meta);
        _receivers.Add(user);
        _sender = user;

        var announce = _options.LocalhostOnly ? [IPAddress.Loopback] : _localAddresses.Select(a => a.Address).ToList();
        foreach (var a in announce)
        {
            MetatrafficUnicast.Add(Locator.UdpV4(a, RtpsPorts.MetatrafficUnicast(DomainId, ParticipantId)));
            UserUnicast.Add(Locator.UdpV4(a, RtpsPorts.UserUnicast(DomainId, ParticipantId)));
        }

        if (Multicast)
        {
            var spdp = TryBind(IPAddress.Any, RtpsPorts.SpdpMulticast(DomainId), reuse: true);
            var userMc = TryBind(IPAddress.Any, RtpsPorts.UserMulticast(DomainId), reuse: true);
            foreach (var s in new[] { spdp, userMc })
            {
                if (s is null)
                {
                    continue;
                }

                foreach (var (address, _) in _localAddresses)
                {
                    try
                    {
                        s.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.AddMembership, new MulticastOption(_options.MulticastGroup, address));
                    }
                    catch (SocketException ex)
                    {
                        _logger.LogDebug(ex, "Could not join {Group} on {Address}", _options.MulticastGroup, address);
                    }
                }

                _receivers.Add(s);
            }

            MetatrafficMulticast.Add(Locator.UdpV4(_options.MulticastGroup, RtpsPorts.SpdpMulticast(DomainId)));
            UserMulticast.Add(Locator.UdpV4(_options.MulticastGroup, RtpsPorts.UserMulticast(DomainId)));
            DiscoveryPeers.Add(new IPEndPoint(_options.MulticastGroup, RtpsPorts.SpdpMulticast(DomainId)));
            _sender.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastLoopback, true);
            _sender.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastTimeToLive, 1);
            if (_options.InterfaceAddress is { } ifa)
            {
                _sender.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastInterface, ifa.GetAddressBytes());
            }
        }

        for (var pid = 0; pid < _options.LocalPeerParticipantIds; pid++)
        {
            if (pid != ParticipantId)
            {
                DiscoveryPeers.Add(new IPEndPoint(IPAddress.Loopback, RtpsPorts.MetatrafficUnicast(DomainId, pid)));
            }
        }

        foreach (var peer in _options.InitialPeers)
        {
            var parts = peer.Split(':');
            if (!IPAddress.TryParse(parts[0], out var address))
            {
                address = Dns.GetHostAddresses(parts[0]).FirstOrDefault(a => a.AddressFamily == AddressFamily.InterNetwork);
            }

            if (address is null)
            {
                continue;
            }

            if (parts.Length > 1 && int.TryParse(parts[1], out var port))
            {
                DiscoveryPeers.Add(new IPEndPoint(address, port));
            }
            else
            {
                for (var pid = 0; pid < _options.LocalPeerParticipantIds; pid++)
                {
                    DiscoveryPeers.Add(new IPEndPoint(address, RtpsPorts.MetatrafficUnicast(DomainId, pid)));
                }
            }
        }
    }

    /// <summary>Starts the receive loops; <paramref name="onDatagram"/> runs on the loop's thread.</summary>
    public void Start(Action<ReadOnlyMemory<byte>> onDatagram)
    {
        foreach (var s in _receivers)
        {
            _loops.Add(Task.Factory.StartNew(() => ReceiveLoop(s, onDatagram), TaskCreationOptions.LongRunning).Unwrap());
        }
    }

    private async Task ReceiveLoop(Socket socket, Action<ReadOnlyMemory<byte>> onDatagram)
    {
        var buffer = ArrayPool<byte>.Shared.Rent(65536);
        EndPoint any = new IPEndPoint(IPAddress.Any, 0);
        try
        {
            while (!_cts.IsCancellationRequested)
            {
                SocketReceiveFromResult result;
                try
                {
                    result = await socket.ReceiveFromAsync(buffer, SocketFlags.None, any, _cts.Token).ConfigureAwait(false);
                }
                catch (SocketException ex) when (ex.SocketErrorCode is SocketError.ConnectionReset or SocketError.MessageSize)
                {
                    continue;
                }

                try
                {
                    onDatagram(buffer.AsMemory(0, result.ReceivedBytes));
                }
                catch (Exception ex) when (ex is not OutOfMemoryException)
                {
                    _logger.LogWarning(ex, "Failed to process an RTPS datagram");
                }
            }
        }
        catch (OperationCanceledException)
        {
        }
        catch (ObjectDisposedException)
        {
        }
        catch (SocketException ex) when (_cts.IsCancellationRequested || ex.SocketErrorCode == SocketError.OperationAborted)
        {
        }
        finally
        {
            ArrayPool<byte>.Shared.Return(buffer);
        }
    }

    public void Send(ReadOnlySpan<byte> datagram, IPEndPoint destination)
    {
        try
        {
            _sender.SendTo(datagram, SocketFlags.None, destination);
        }
        catch (SocketException ex)
        {
            _logger.LogTrace("Send to {Destination} failed: {Error}", destination, ex.SocketErrorCode);
        }
        catch (ObjectDisposedException)
        {
        }
    }

    /// <summary>Picks the endpoints to use for a remote participant's locators (prefers loopback for same-host peers).</summary>
    public List<IPEndPoint> Select(IEnumerable<Locator> locators)
    {
        var v4 = locators.Where(l => l.Kind == Locator.KindUdpV4 && !IsMulticast(l.Address)).ToList();
        if (v4.Count == 0)
        {
            return [];
        }

        var loop = v4.FirstOrDefault(l => IPAddress.IsLoopback(l.Address));
        if (loop.Address is not null)
        {
            return [loop.ToEndPoint()];
        }

        var local = v4.FirstOrDefault(l => _localAddresses.Any(a => a.Address.Equals(l.Address)));
        if (local.Address is not null)
        {
            return [new IPEndPoint(IPAddress.Loopback, (int)local.Port)];
        }

        if (_options.LocalhostOnly)
        {
            return [];
        }

        var sameSubnet = v4.Where(l => _localAddresses.Any(a => SameSubnet(a.Address, l.Address, a.Mask))).Select(l => l.ToEndPoint()).ToList();
        return sameSubnet.Count > 0 ? [sameSubnet[0]] : v4.Select(l => l.ToEndPoint()).ToList();
    }

    private static bool IsMulticast(IPAddress a) => a.AddressFamily == AddressFamily.InterNetwork && (a.GetAddressBytes()[0] & 0xf0) == 0xe0;

    private static bool SameSubnet(IPAddress a, IPAddress b, IPAddress mask)
    {
        var x = a.GetAddressBytes();
        var y = b.GetAddressBytes();
        var m = mask.GetAddressBytes();
        for (var i = 0; i < 4; i++)
        {
            if ((x[i] & m[i]) != (y[i] & m[i]))
            {
                return false;
            }
        }

        return true;
    }

    private void DiscoverAddresses()
    {
        if (_options.LocalhostOnly)
        {
            _localAddresses.Add((IPAddress.Loopback, IPAddress.Parse("255.0.0.0")));
            return;
        }

        try
        {
            foreach (var nic in NetworkInterface.GetAllNetworkInterfaces())
            {
                if (nic.OperationalStatus != OperationalStatus.Up || nic.NetworkInterfaceType == NetworkInterfaceType.Loopback)
                {
                    continue;
                }

                foreach (var ua in nic.GetIPProperties().UnicastAddresses)
                {
                    if (ua.Address.AddressFamily != AddressFamily.InterNetwork)
                    {
                        continue;
                    }

                    if (_options.InterfaceAddress is { } only && !only.Equals(ua.Address))
                    {
                        continue;
                    }

                    _localAddresses.Add((ua.Address, ua.IPv4Mask ?? IPAddress.Parse("255.255.255.0")));
                }
            }
        }
        catch (NetworkInformationException ex)
        {
            _logger.LogWarning(ex, "Could not enumerate network interfaces");
        }

        if (_localAddresses.Count == 0)
        {
            _localAddresses.Add((IPAddress.Loopback, IPAddress.Parse("255.0.0.0")));
        }
    }

    private Socket? TryBind(IPAddress address, int port, bool reuse)
    {
        var s = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        try
        {
            if (OperatingSystem.IsWindows())
            {
                // Without this, an ICMP "port unreachable" (e.g. SPDP to an unused peer port) faults the next receive.
                s.IOControl(SioUdpConnReset, [0, 0, 0, 0], null);
                if (!reuse)
                {
                    s.ExclusiveAddressUse = true;
                }
            }

            if (reuse)
            {
                s.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            }

            s.ReceiveBufferSize = _options.SocketBufferSize;
            s.SendBufferSize = _options.SocketBufferSize;
            s.Bind(new IPEndPoint(address, port));
            return s;
        }
        catch (SocketException)
        {
            s.Dispose();
            return null;
        }
    }

    public async ValueTask DisposeAsync()
    {
        _cts.Cancel();
        foreach (var s in _receivers)
        {
            s.Dispose();
        }

        try
        {
            await Task.WhenAll(_loops).WaitAsync(TimeSpan.FromSeconds(2)).ConfigureAwait(false);
        }
        catch (Exception ex) when (ex is TimeoutException or OperationCanceledException or ObjectDisposedException)
        {
        }

        _cts.Dispose();
    }
}
