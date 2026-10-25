using ROS2.Net;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Simulation;

namespace {{Namespace}}.Services;

/// <summary>Runs a 3-robot warehouse with simple autonomous wandering.</summary>
public sealed class AppRuntime(Ros2Context context) : IHostedService, IDisposable
{
    private SimulationNode? _sim;
    private Node? _brains;

    public SimWorld? World { get; private set; }

    public Task StartAsync(CancellationToken cancellationToken)
    {
        // Simulator: three AMRs with wandering autonomy. Physical robot: the dashboard only watches it.
        _sim = AppTarget.StartSimulator(context, ScenarioKind.Warehouse, robots: 3);
        World = AppTarget.World;
        if (World is null)
        {
            return Task.CompletedTask;
        }

        _brains = context.CreateNode("fleet_autonomy");
        var turn = 1.0;
        foreach (var r in World.Robots)
        {
            var cmd = _brains.CreatePublisher<Twist>($"/{r.Name}/cmd_vel");
            var sign = turn;
            turn = -turn;
            _brains.CreateSubscription<LaserScan>($"/{r.Name}/scan", scan =>
            {
                var front = float.PositiveInfinity;
                for (var i = 0; i < scan.Ranges.Length; i++)
                {
                    var a = scan.AngleMin + (i * scan.AngleIncrement);
                    if (Math.Abs(Math.Atan2(Math.Sin(a), Math.Cos(a))) < 0.5 && scan.Ranges[i] > scan.RangeMin)
                    {
                        front = Math.Min(front, scan.Ranges[i]);
                    }
                }

                cmd.Publish(front < 0.6
                    ? new Twist { Angular = new Vector3 { Z = 1.2 * sign } }
                    : new Twist { Linear = new Vector3 { X = 0.35 }, Angular = new Vector3 { Z = 0.1 * sign } });
            }, QosProfile.SensorData);
        }

        return Task.CompletedTask;
    }

    public Task StopAsync(CancellationToken cancellationToken) => _sim?.StopAsync() ?? Task.CompletedTask;

    public void Dispose()
    {
        _brains?.Dispose();
        _sim?.Dispose();
    }
}
