using System.CommandLine;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using ROS2.Net.Native;
using ROS2.Net.Rmw;
using ROS2.Net.Rmw.Rosbridge;
using ROS2.Net.Rmw.Rtps;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net doctor</c>: checks the environment, like <c>ros2 doctor</c>.</summary>
internal static class DoctorCommand
{
    public static Command Build(CliEnvironment env)
    {
        var report = new Option<bool>("--report", "-r") { Description = "Print the full report including network interfaces." };
        var cmd = new Command("doctor", "Check the environment: runtime, middlewares, network, multicast, native library.") { report };
        cmd.Handle(env, async (p, ct) =>
        {
            var o = env.Out;
            var problems = 0;

            // Touch the middleware assemblies so their factories register.
            RosTypeRegistry.EnsureLoaded(typeof(RtpsRmwFactory).Assembly);
            RosTypeRegistry.EnsureLoaded(typeof(RosbridgeRmwFactory).Assembly);

            o.WriteLine("ros2net doctor — ROS2.Net, made by Gravicode Studios, led by Kang Fadhil");
            o.WriteLine();
            o.WriteLine("  PLATFORM INFORMATION");
            o.WriteLine($"  .NET runtime     : {RuntimeInformation.FrameworkDescription}");
            o.WriteLine($"  OS               : {RuntimeInformation.OSDescription} ({RuntimeInformation.OSArchitecture})");
            o.WriteLine($"  ros2net version  : {typeof(Ros2Context).Assembly.GetName().Version}");
            o.WriteLine();
            o.WriteLine("  ROS ENVIRONMENT");
            foreach (var v in new[] { "ROS_DOMAIN_ID", "ROS2NET_RMW", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY", "ROS_AUTOMATIC_DISCOVERY_RANGE", "ROS_SECURITY_ENCLAVE_OVERRIDE", "ROS_DISTRO" })
            {
                o.WriteLine($"  {v,-30}: {Environment.GetEnvironmentVariable(v) ?? "<unset>"}");
            }

            if (int.TryParse(Environment.GetEnvironmentVariable("ROS_DOMAIN_ID"), out var d) && d is < 0 or > 232)
            {
                o.WriteLine("  WARNING: ROS_DOMAIN_ID must be within 0..232.");
                problems++;
            }

            o.WriteLine();
            o.WriteLine("  MIDDLEWARE");
            o.WriteLine($"  available        : {string.Join(", ", RmwRegistry.Identifiers)}");
            o.WriteLine($"  default          : {SafeDefault()}");
            o.WriteLine();

            o.WriteLine("  NETWORK");
            var interfaces = NetworkInterface.GetAllNetworkInterfaces().Where(n => n.OperationalStatus == OperationalStatus.Up).ToList();
            var multicastCapable = interfaces.Count(n => n.SupportsMulticast && n.NetworkInterfaceType != NetworkInterfaceType.Loopback);
            o.WriteLine($"  interfaces up    : {interfaces.Count} ({multicastCapable} multicast-capable, non-loopback)");
            if (p.GetValue(report))
            {
                foreach (var n in interfaces)
                {
                    var addrs = n.GetIPProperties().UnicastAddresses.Where(a => a.Address.AddressFamily == AddressFamily.InterNetwork).Select(a => a.Address);
                    o.WriteLine($"    {n.Name} [{n.NetworkInterfaceType}] {string.Join(", ", addrs)}{(n.SupportsMulticast ? " multicast" : string.Empty)}");
                }
            }

            var multicast = await MulticastLoopbackAsync(ct);
            o.WriteLine($"  multicast test   : {(multicast ? "OK (sent and received on 239.255.0.1)" : "FAILED — DDS discovery needs multicast or --localhost / initial peers")}");
            problems += multicast ? 0 : 1;
            var portFree = PortAvailable(7400);
            o.WriteLine($"  DDS port 7400    : {(portFree ? "bindable" : "in use (another DDS participant is running — that is fine)")}");
            o.WriteLine();

            o.WriteLine("  NATIVE ACCELERATION");
            o.WriteLine(NativeLibraryInfo.IsAvailable
                ? $"  ros2net_native   : available, version {NativeLibraryInfo.Version} (ABI {NativeLibraryInfo.AbiVersion})"
                : $"  ros2net_native   : not available, managed fallback in use ({NativeLibraryInfo.LoadError})");
            o.WriteLine();
            o.WriteLine(problems == 0 ? "All checks passed." : $"{problems} problem(s) found.");
            return problems == 0 ? 0 : 1;
        });
        return cmd;
    }

    private static string SafeDefault()
    {
        try
        {
            return RmwRegistry.GetDefault().Identifier;
        }
        catch (InvalidOperationException ex)
        {
            return ex.Message;
        }
    }

    private static bool PortAvailable(int port)
    {
        try
        {
            using var s = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            s.Bind(new IPEndPoint(IPAddress.Any, port));
            return true;
        }
        catch (SocketException)
        {
            return false;
        }
    }

    /// <summary>Sends one datagram to a multicast group and checks it loops back.</summary>
    internal static async Task<bool> MulticastLoopbackAsync(CancellationToken ct)
    {
        var group = IPAddress.Parse("239.255.0.1");
        try
        {
            using var receiver = new UdpClient(AddressFamily.InterNetwork);
            receiver.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
            receiver.Client.Bind(new IPEndPoint(IPAddress.Any, 0));
            var port = ((IPEndPoint)receiver.Client.LocalEndPoint!).Port;
            receiver.JoinMulticastGroup(group);
            receiver.MulticastLoopback = true;

            using var sender = new UdpClient(AddressFamily.InterNetwork);
            sender.MulticastLoopback = true;
            var payload = "ros2net-doctor"u8.ToArray();
            await sender.SendAsync(payload, new IPEndPoint(group, port), ct);

            using var timeout = CancellationTokenSource.CreateLinkedTokenSource(ct);
            timeout.CancelAfter(TimeSpan.FromSeconds(1));
            var result = await receiver.ReceiveAsync(timeout.Token);
            return result.Buffer.AsSpan().SequenceEqual(payload);
        }
        catch (Exception ex) when (ex is SocketException or OperationCanceledException)
        {
            return false;
        }
    }
}
