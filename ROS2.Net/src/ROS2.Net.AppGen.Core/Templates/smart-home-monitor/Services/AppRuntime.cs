using ROS2.Net;
using ROS2.Net.Simulation;

namespace {{Namespace}}.Services;

/// <summary>Runs the simulated home (office scenario) for the lifetime of the web app.</summary>
public sealed class AppRuntime(Ros2Context context) : IHostedService, IDisposable
{
    private SimulationNode? _sim;

    public SimWorld? World { get; private set; }

    public Task StartAsync(CancellationToken cancellationToken)
    {
        // Simulator mode: a simulated apartment (office scenario). Physical robot: the real robot's topics.
        _sim = AppTarget.StartSimulator(context, ScenarioKind.Office);
        World = AppTarget.World;
        return Task.CompletedTask;
    }

    public Task StopAsync(CancellationToken cancellationToken) => _sim?.StopAsync() ?? Task.CompletedTask;

    public void Dispose() => _sim?.Dispose();
}
