using System.Globalization;
using System.Runtime.CompilerServices;
using Microsoft.Extensions.Logging;

namespace ROS2.Net.Rmw.Rosbridge;

/// <summary>Settings of the rosbridge middleware, read from <see cref="RmwContextOptions.Settings"/>.</summary>
public sealed class RosbridgeOptions
{
    /// <summary>Setting <c>url</c>: the rosbridge_server WebSocket endpoint.</summary>
    public Uri Url { get; set; } = new("ws://localhost:9090");

    /// <summary>Setting <c>connect_timeout_ms</c>: how long the first connection may take before context creation fails.</summary>
    public TimeSpan ConnectTimeout { get; set; } = TimeSpan.FromSeconds(5);

    /// <summary>Setting <c>reconnect_ms</c>: delay between reconnection attempts after the link drops.</summary>
    public TimeSpan ReconnectDelay { get; set; } = TimeSpan.FromSeconds(1);

    /// <summary>Setting <c>graph_poll_ms</c>: how often the graph is refreshed from rosapi. 0 disables polling.</summary>
    public TimeSpan GraphPollInterval { get; set; } = TimeSpan.FromSeconds(2);

    /// <summary>Setting <c>throttle_rate</c>: minimum milliseconds between messages rosbridge sends per subscription (0 = unthrottled).</summary>
    public int ThrottleRateMs { get; set; }

    /// <summary>Setting <c>fragment_size</c>: ask rosbridge to fragment outgoing messages larger than this (0 = never).</summary>
    public int FragmentSize { get; set; }

    internal static RosbridgeOptions FromSettings(IReadOnlyDictionary<string, string> settings)
    {
        var o = new RosbridgeOptions();
        if (settings.TryGetValue("url", out var url) && !string.IsNullOrWhiteSpace(url))
        {
            o.Url = new Uri(url);
        }

        o.ConnectTimeout = Ms(settings, "connect_timeout_ms", o.ConnectTimeout);
        o.ReconnectDelay = Ms(settings, "reconnect_ms", o.ReconnectDelay);
        o.GraphPollInterval = Ms(settings, "graph_poll_ms", o.GraphPollInterval);
        if (settings.TryGetValue("throttle_rate", out var t) && int.TryParse(t, NumberStyles.Integer, CultureInfo.InvariantCulture, out var tr))
        {
            o.ThrottleRateMs = tr;
        }

        if (settings.TryGetValue("fragment_size", out var f) && int.TryParse(f, NumberStyles.Integer, CultureInfo.InvariantCulture, out var fs))
        {
            o.FragmentSize = fs;
        }

        return o;
    }

    private static TimeSpan Ms(IReadOnlyDictionary<string, string> s, string key, TimeSpan fallback) =>
        s.TryGetValue(key, out var v) && double.TryParse(v, NumberStyles.Float, CultureInfo.InvariantCulture, out var ms) ? TimeSpan.FromMilliseconds(ms) : fallback;
}

/// <summary>Factory for the rosbridge middleware (identifier <c>rosbridge</c>).</summary>
public sealed class RosbridgeRmwFactory : IRmwFactory
{
    public string Identifier => "rosbridge";

    public async ValueTask<IRmwImplementation> CreateAsync(RmwContextOptions options, CancellationToken cancellationToken)
    {
        var settings = RosbridgeOptions.FromSettings(options.Settings);
        var rmw = new RosbridgeRmw(settings, options.LoggerFactory.CreateLogger("ros2net.rosbridge"));
        try
        {
            await rmw.StartAsync(cancellationToken).ConfigureAwait(false);
        }
        catch
        {
            await rmw.DisposeAsync().ConfigureAwait(false);
            throw;
        }

        return rmw;
    }

    // Registers 'rosbridge' with RmwRegistry as soon as this assembly loads, so ROS2NET_RMW=rosbridge works.
#pragma warning disable CA2255
    [ModuleInitializer]
#pragma warning restore CA2255
    internal static void Register() => RmwRegistry.Register(new RosbridgeRmwFactory());
}

/// <summary>Configuration helpers for the rosbridge middleware.</summary>
public static class RosbridgeContextOptionsExtensions
{
    /// <summary>Uses rosbridge_server at <paramref name="url"/> as the middleware.</summary>
    public static Ros2ContextOptions UseRosbridge(this Ros2ContextOptions options, string url = "ws://localhost:9090", Action<RosbridgeOptions>? configure = null)
    {
        ArgumentNullException.ThrowIfNull(options);
        options.Rmw = new RosbridgeRmwFactory();
        options.RmwSettings["url"] = url;
        if (configure is not null)
        {
            var o = RosbridgeOptions.FromSettings(options.RmwSettings);
            configure(o);
            options.RmwSettings["url"] = o.Url.ToString();
            options.RmwSettings["connect_timeout_ms"] = o.ConnectTimeout.TotalMilliseconds.ToString(CultureInfo.InvariantCulture);
            options.RmwSettings["reconnect_ms"] = o.ReconnectDelay.TotalMilliseconds.ToString(CultureInfo.InvariantCulture);
            options.RmwSettings["graph_poll_ms"] = o.GraphPollInterval.TotalMilliseconds.ToString(CultureInfo.InvariantCulture);
            options.RmwSettings["throttle_rate"] = o.ThrottleRateMs.ToString(CultureInfo.InvariantCulture);
            options.RmwSettings["fragment_size"] = o.FragmentSize.ToString(CultureInfo.InvariantCulture);
        }

        return options;
    }
}
