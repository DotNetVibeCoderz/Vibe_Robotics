using Microsoft.Extensions.Logging;
using ROS2.Net;
using ROS2.Net.Msgs.SensorMsgs;

namespace {{Namespace}}.Services;

/// <summary>Simulated greenhouse sensors (replace with real devices publishing the same topics).</summary>
public sealed class AppRuntime(Ros2Context context) : IHostedService
{
    private Node? _node;

    public Task StartAsync(CancellationToken cancellationToken)
    {
        _node = context.CreateNode("greenhouse");
        var temp = _node.CreatePublisher<Temperature>("/greenhouse/temperature");
        var humidity = _node.CreatePublisher<RelativeHumidity>("/greenhouse/humidity");
        var light = _node.CreatePublisher<Illuminance>("/greenhouse/light");
        var rnd = new Random();
        var t = 0.0;
        _node.CreateTimer(TimeSpan.FromMilliseconds(500), () =>
        {
            t += 0.5;
            var day = Math.Sin(t / 60);
            var c = 24 + (6 * day) + rnd.NextDouble();
            temp.Publish(new Temperature { TemperatureValue = c });
            humidity.Publish(new RelativeHumidity { RelativeHumidityValue = Math.Clamp(0.7 - (0.15 * day) + (rnd.NextDouble() * 0.03), 0, 1) });
            light.Publish(new Illuminance { IlluminanceValue = Math.Max(0, 30000 * day) + (rnd.NextDouble() * 500) });
            if (c > 29)
            {
                _node.Logger.LogWarning("Temperature {Temp:0.0} °C: opening roof vents", c);
            }
        });
        return Task.CompletedTask;
    }

    public Task StopAsync(CancellationToken cancellationToken)
    {
        _node?.Dispose();
        return Task.CompletedTask;
    }
}
