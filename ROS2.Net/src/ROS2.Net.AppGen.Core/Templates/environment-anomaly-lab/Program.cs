// {{ProjectName}} — environmental monitoring with anomaly detection (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.AI.Diagnostics;
using ROS2.Net.Msgs.DiagnosticMsgs;
using ROS2.Net.Msgs.SensorMsgs;

await using var context = await AppTarget.CreateContextAsync(args);

// Sensor node: a slow daily cycle, noise, a drift after 30 s and a spike every 40 s.
var sensors = context.CreateNode("lab_sensors");
var temperature = sensors.CreatePublisher<Temperature>("/lab/temperature");
var humidity = sensors.CreatePublisher<RelativeHumidity>("/lab/humidity");
var random = new Random(42);
var t = 0.0;
sensors.CreateTimer(TimeSpan.FromMilliseconds(200), () =>
{
    t += 0.2;
    var temp = 22 + (0.5 * Math.Sin(t / 20)) + (random.NextDouble() * 0.1) + (t > 30 ? (t - 30) * 0.05 : 0) + ((int)t % 40 == 39 ? 6 : 0);
    temperature.Publish(new Temperature { TemperatureValue = temp, Variance = 0.01 });
    humidity.Publish(new RelativeHumidity { RelativeHumidityValue = 0.45 + (random.NextDouble() * 0.02), Variance = 0.001 });
});

// Monitor node: EWMA z-score per signal, diagnostics on /diagnostics.
var monitor = context.CreateNode("lab_monitor");
var diagnostics = monitor.CreatePublisher<DiagnosticArray>("/diagnostics");
var tempDetector = new EwmaAnomalyDetector(alpha: 0.05, threshold: 4);
monitor.CreateSubscription<Temperature>("/lab/temperature", m =>
{
    var r = tempDetector.Update(m.TemperatureValue);
    var level = r.IsAnomaly ? DiagnosticStatus.WARN : DiagnosticStatus.OK;
    if (r.IsAnomaly)
    {
        Console.WriteLine($"[{t,6:0.0}s] ANOMALY temperature {r.Value:0.00} °C (mean {r.Mean:0.00}, z={r.Score:0.0})");
    }

    var status = new DiagnosticStatus { Name = "lab/temperature", Level = level, Message = r.IsAnomaly ? "anomaly" : "ok", HardwareId = "lab-thermo-1" };
    status.Values = [new KeyValue { Key = "value", Value = r.Value.ToString("0.00") }, new KeyValue { Key = "zscore", Value = r.Score.ToString("0.0") }];
    diagnostics.Publish(new DiagnosticArray { Status = [status] });
});

Console.WriteLine("Monitoring the lab — press Ctrl+C to stop.");
await context.SpinAsync();
