using Microsoft.Extensions.Logging;
using ROS2.Net;
using ROS2.Net.AI.Diagnostics;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace {{Namespace}}.Services;

/// <summary>Simulated bedside monitor publishing /patient/heart_rate and /patient/spo2, with anomaly alerts.</summary>
public sealed class AppRuntime(Ros2Context context) : IHostedService
{
    private Node? _monitor;
    private Node? _detector;

    public Task StartAsync(CancellationToken cancellationToken)
    {
        _monitor = context.CreateNode("bedside_monitor");
        var hr = _monitor.CreatePublisher<StdMsgs.Float64>("/patient/heart_rate");
        var spo2 = _monitor.CreatePublisher<StdMsgs.Float64>("/patient/spo2");
        var rnd = new Random();
        var t = 0.0;
        _monitor.CreateTimer(TimeSpan.FromMilliseconds(500), () =>
        {
            t += 0.5;
            var episode = t % 90 > 75;                 // a tachycardia / desaturation episode every 90 s
            hr.Publish(new StdMsgs.Float64 { Data = (episode ? 128 : 72) + (rnd.NextDouble() * 4) });
            spo2.Publish(new StdMsgs.Float64 { Data = (episode ? 89 : 97.5) + (rnd.NextDouble() * 1.5) });
        });

        _detector = context.CreateNode("vitals_watch");
        var hrDetector = new EwmaAnomalyDetector(0.05, 4, 20);
        _detector.CreateSubscription<StdMsgs.Float64>("/patient/heart_rate", m =>
        {
            if (hrDetector.Update(m.Data).IsAnomaly || m.Data > 120)
            {
                _detector.Logger.LogWarning("Heart rate {Hr:0} bpm — check patient", m.Data);
            }
        });
        _detector.CreateSubscription<StdMsgs.Float64>("/patient/spo2", m =>
        {
            if (m.Data < 92)
            {
                _detector.Logger.LogError("SpO2 {Spo2:0.0}% below 92%", m.Data);
            }
        });
        return Task.CompletedTask;
    }

    public Task StopAsync(CancellationToken cancellationToken)
    {
        _detector?.Dispose();
        _monitor?.Dispose();
        return Task.CompletedTask;
    }
}
