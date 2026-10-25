// {{ProjectName}} — machine condition monitoring (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.AI.Diagnostics;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.GeometryMsgs;

await using var context = await AppTarget.CreateContextAsync(args);

// The machine: motor temperature (sensor_msgs/Temperature) and vibration (Vector3Stamped, g).
var machine = context.CreateNode("press_line_1");
var temp = machine.CreatePublisher<Temperature>("/press/motor_temperature");
var vib = machine.CreatePublisher<Vector3Stamped>("/press/vibration");
var rnd = new Random(1);
var t = 0.0;
machine.CreateTimer(TimeSpan.FromMilliseconds(100), () =>
{
    t += 0.1;
    var bearingFault = t > 25 ? 1 + ((t - 25) * 0.08) : 1;   // a bearing starts to wear after 25 s
    temp.Publish(new Temperature { TemperatureValue = 55 + (2 * Math.Sin(t / 7)) + (rnd.NextDouble() * 0.5) + (t > 25 ? (t - 25) * 0.15 : 0) });
    vib.Publish(new Vector3Stamped { Vector = new Vector3 { X = 0.02 * bearingFault * rnd.NextDouble(), Y = 0.02 * bearingFault * rnd.NextDouble(), Z = 0.05 * bearingFault * rnd.NextDouble() } });
});

// Monitors publish diagnostic_msgs/DiagnosticArray on /diagnostics and raise events.
var monitor = context.CreateNode("condition_monitor");
using var thermal = new TelemetryAnomalyMonitor<Temperature>(monitor, "/press/motor_temperature",
    [("motor_temperature", m => m.TemperatureValue)], () => new EwmaAnomalyDetector(0.03, 4, 40), "press-1-motor");
using var vibration = new TelemetryAnomalyMonitor<Vector3Stamped>(monitor, "/press/vibration",
    [("vibration_rms", m => Math.Sqrt((m.Vector.X * m.Vector.X) + (m.Vector.Y * m.Vector.Y) + (m.Vector.Z * m.Vector.Z)))], () => new RollingZScoreDetector(200, 4), "press-1-bearing");
thermal.AnomalyDetected += (signal, r) => Console.WriteLine($"[{t,5:0.0}s] ⚠ {signal}: {r.Value:0.0} °C (z={r.Score:0.0})");
vibration.AnomalyDetected += (signal, r) => Console.WriteLine($"[{t,5:0.0}s] ⚠ {signal}: {r.Value:0.000} g (z={r.Score:0.0}) — schedule bearing inspection");

Console.WriteLine("Monitoring press line 1 — Ctrl+C to stop.");
await context.SpinAsync();
