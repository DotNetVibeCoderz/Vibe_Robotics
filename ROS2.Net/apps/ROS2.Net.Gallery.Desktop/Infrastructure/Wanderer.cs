using ROS2.Net.Simulation;

namespace Ros2NetGallery;

/// <summary>
/// A tiny reactive controller that keeps simulated robots roaming: drive forward, turn away when the
/// LiDAR sees something close in front. Runs on the world's stepping thread.
/// </summary>
public sealed class Wanderer : IDisposable
{
    private readonly SimWorld _world;
    private readonly Dictionary<string, double> _turn = new(StringComparer.Ordinal);
    private readonly Random _random = new(11);

    public Wanderer(SimWorld world, Func<SimRobot, bool>? filter = null)
    {
        _world = world;
        Filter = filter ?? (_ => true);
        world.Stepped += OnStep;
    }

    public Func<SimRobot, bool> Filter { get; set; }

    public double Speed { get; set; } = 0.22;

    private void OnStep(SimWorld w)
    {
        if (w.StepCount % 5 != 0)
        {
            return;
        }

        foreach (var r in w.Robots)
        {
            if (!Filter(r) || r.Lidar is not { } lidar || lidar.Ranges.Length == 0)
            {
                continue;
            }

            var ranges = lidar.Ranges;
            var n = ranges.Length;
            var front = float.MaxValue;
            var left = 0.0;
            var right = 0.0;
            for (var i = 0; i < n; i++)
            {
                var angle = -Math.PI + ((2 * Math.PI) * i / n);
                var rr = float.IsFinite(ranges[i]) ? ranges[i] : 3.5f;
                if (Math.Abs(angle) < 0.45)
                {
                    front = Math.Min(front, rr);
                }
                else if (angle > 0.45 && angle < 1.6)
                {
                    left += rr;
                }
                else if (angle < -0.45 && angle > -1.6)
                {
                    right += rr;
                }
            }

            _turn.TryGetValue(r.Name, out var turning);
            double lin, ang;
            if (front < 0.45 || turning > 0)
            {
                if (turning <= 0)
                {
                    turning = 0.6 + (_random.NextDouble() * 0.8);
                }

                turning -= 0.05;
                lin = front < 0.25 ? -0.05 : 0.02;
                ang = left >= right ? 1.6 : -1.6;
            }
            else
            {
                lin = Speed * Math.Clamp(front / 1.2, 0.35, 1.0);
                ang = Math.Clamp((left - right) * 0.002, -0.6, 0.6) + ((_random.NextDouble() - 0.5) * 0.3);
            }

            _turn[r.Name] = turning;
            r.SetCommand(lin, 0, ang, w.Time);
        }
    }

    public void Dispose() => _world.Stepped -= OnStep;
}

/// <summary>Runs a <see cref="SimWorld"/> in real time on a background task until disposed.</summary>
public sealed class WorldRunner : IDisposable
{
    private readonly CancellationTokenSource _cts = new();

    public WorldRunner(SimWorld world)
    {
        World = world;
        Task = Task.Run(() => world.RunAsync(_cts.Token));
    }

    public SimWorld World { get; }

    public Task Task { get; }

    public void Dispose()
    {
        _cts.Cancel();
        try
        {
            Task.Wait(TimeSpan.FromSeconds(1));
        }
        catch (AggregateException)
        {
        }
    }
}
