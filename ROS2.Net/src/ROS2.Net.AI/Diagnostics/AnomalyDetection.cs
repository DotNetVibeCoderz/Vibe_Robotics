using ROS2.Net.Msgs.DiagnosticMsgs;

namespace ROS2.Net.AI.Diagnostics;

/// <summary>Result of feeding one sample to a detector.</summary>
/// <param name="IsAnomaly">The sample exceeded the threshold (always false during warm-up).</param>
/// <param name="Score">|value − mean| / std.</param>
/// <param name="Mean">Running mean before this sample.</param>
/// <param name="StdDev">Running standard deviation before this sample.</param>
/// <param name="Value">The sample.</param>
public readonly record struct AnomalyResult(bool IsAnomaly, double Score, double Mean, double StdDev, double Value);

/// <summary>A streaming anomaly detector over a scalar signal.</summary>
public interface IAnomalyDetector
{
    AnomalyResult Update(double value);

    void Reset();
}

/// <summary>
/// Exponentially weighted mean/variance with a z-score threshold. O(1), allocation-free; adapts to
/// slow drift, flags sudden jumps (current spikes, IMU shocks, latency bursts).
/// </summary>
public sealed class EwmaAnomalyDetector(double alpha = 0.05, double threshold = 4.0, int warmup = 20) : IAnomalyDetector
{
    private double _mean, _variance;
    private int _count;

    public double Mean => _mean;

    public double StdDev => Math.Sqrt(_variance);

    public AnomalyResult Update(double value)
    {
        if (_count == 0)
        {
            _mean = value;
            _variance = 0;
            _count = 1;
            return new AnomalyResult(false, 0, value, 0, value);
        }

        var std = Math.Sqrt(_variance);
        var score = std > 1e-12 ? Math.Abs(value - _mean) / std : (Math.Abs(value - _mean) > 1e-12 ? double.PositiveInfinity : 0);
        var anomaly = _count >= warmup && score > threshold;
        var result = new AnomalyResult(anomaly, score, _mean, std, value);

        // Anomalies update the statistics with a smaller weight so a single spike does not mask the next one.
        var a = anomaly ? alpha * 0.1 : alpha;
        var diff = value - _mean;
        _mean += a * diff;
        _variance = (1 - a) * (_variance + (a * diff * diff));
        _count++;
        return result;
    }

    public void Reset()
    {
        _count = 0;
        _mean = _variance = 0;
    }
}

/// <summary>Z-score over a fixed sliding window (ring buffer, allocation-free after construction).</summary>
public sealed class RollingZScoreDetector : IAnomalyDetector
{
    private readonly double[] _window;
    private readonly double _threshold;
    private int _count, _head;
    private double _sum, _sumSq;

    public RollingZScoreDetector(int window = 100, double threshold = 3.0)
    {
        if (window < 2)
        {
            throw new ArgumentOutOfRangeException(nameof(window));
        }

        _window = new double[window];
        _threshold = threshold;
    }

    public AnomalyResult Update(double value)
    {
        var mean = _count > 0 ? _sum / _count : value;
        var variance = _count > 1 ? Math.Max(0, (_sumSq / _count) - (mean * mean)) : 0;
        var std = Math.Sqrt(variance);
        var score = std > 1e-12 ? Math.Abs(value - mean) / std : 0;
        var anomaly = _count >= _window.Length / 2 && score > _threshold;

        if (_count == _window.Length)
        {
            var old = _window[_head];
            _sum -= old;
            _sumSq -= old * old;
        }
        else
        {
            _count++;
        }

        _window[_head] = value;
        _head = (_head + 1) % _window.Length;
        _sum += value;
        _sumSq += value * value;
        return new AnomalyResult(anomaly, score, mean, std, value);
    }

    public void Reset()
    {
        _count = _head = 0;
        _sum = _sumSq = 0;
    }
}

/// <summary>
/// Watches numeric signals extracted from a topic and publishes <c>diagnostic_msgs/DiagnosticArray</c>
/// on <c>/diagnostics</c> (WARN on anomaly, OK otherwise), so <c>rqt_robot_monitor</c> and dashboards
/// pick them up. The subscription callback runs on the node's executor.
/// </summary>
public sealed class TelemetryAnomalyMonitor<T> : IDisposable
    where T : class, IRosMessage<T>, new()
{
    private readonly (string Name, Func<T, double> Extract, IAnomalyDetector Detector)[] _signals;
    private readonly Subscription<T> _subscription;
    private readonly Publisher<DiagnosticArray> _diagnostics;
    private readonly string _hardwareId;

    public TelemetryAnomalyMonitor(Node node, string topic, IEnumerable<(string Name, Func<T, double> Extract)> signals, Func<IAnomalyDetector>? detectorFactory = null, string? hardwareId = null, QosProfile? qos = null)
    {
        detectorFactory ??= () => new EwmaAnomalyDetector();
        _signals = signals.Select(s => (s.Name, s.Extract, detectorFactory())).ToArray();
        _hardwareId = hardwareId ?? topic;
        _diagnostics = node.CreatePublisher<DiagnosticArray>("/diagnostics");
        _subscription = node.CreateSubscription<T>(topic, OnMessage, qos ?? QosProfile.SensorData);
        Topic = _subscription.TopicName;
    }

    public string Topic { get; }

    /// <summary>Raised for each anomalous sample with the signal name.</summary>
    public event Action<string, AnomalyResult>? AnomalyDetected;

    /// <summary>Feeds a message directly (also used by the subscription).</summary>
    public void OnMessage(T message)
    {
        var statuses = new DiagnosticStatus[_signals.Length];
        for (var i = 0; i < _signals.Length; i++)
        {
            var (name, extract, detector) = _signals[i];
            var r = detector.Update(extract(message));
            if (r.IsAnomaly)
            {
                AnomalyDetected?.Invoke(name, r);
            }

            statuses[i] = new DiagnosticStatus
            {
                Level = r.IsAnomaly ? DiagnosticStatus.WARN : DiagnosticStatus.OK,
                Name = $"{Topic}: {name}",
                Message = r.IsAnomaly ? $"anomaly (z={r.Score:0.0})" : "ok",
                HardwareId = _hardwareId,
                Values =
                [
                    new KeyValue { Key = "value", Value = r.Value.ToString("G6", System.Globalization.CultureInfo.InvariantCulture) },
                    new KeyValue { Key = "mean", Value = r.Mean.ToString("G6", System.Globalization.CultureInfo.InvariantCulture) },
                    new KeyValue { Key = "stddev", Value = r.StdDev.ToString("G6", System.Globalization.CultureInfo.InvariantCulture) },
                ],
            };
        }

        var array = new DiagnosticArray { Status = statuses };
        RosTime.Now().CopyTo(array.Header.Stamp);
        _diagnostics.Publish(array);
    }

    public void Dispose()
    {
        _subscription.Dispose();
        _diagnostics.Dispose();
    }
}
