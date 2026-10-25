using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Options;

namespace ROS2.Net.Hosting;

/// <summary>Creates the registered nodes at host start, spins the context, and shuts it down at host stop.</summary>
internal sealed class Ros2HostedService : IHostedService
{
    private readonly IServiceProvider _services;
    private readonly IOptions<Ros2HostOptions> _options;
    private readonly IEnumerable<Ros2NodeRegistration> _registrations;
    private readonly IHostApplicationLifetime? _lifetime;
    private readonly ILogger<Ros2HostedService>? _logger;
    private readonly CancellationTokenSource _stopping = new();
    private Ros2Context? _context;
    private Task? _spin;

    public Ros2HostedService(
        IServiceProvider services,
        IOptions<Ros2HostOptions> options,
        IEnumerable<Ros2NodeRegistration> registrations,
        IHostApplicationLifetime? lifetime = null,
        ILogger<Ros2HostedService>? logger = null)
    {
        _services = services;
        _options = options;
        _registrations = registrations;
        _lifetime = lifetime;
        _logger = logger;
    }

    public Task StartAsync(CancellationToken cancellationToken)
    {
        _context = (Ros2Context)_services.GetService(typeof(Ros2Context))!;
        foreach (var registration in _registrations)
        {
            registration.Factory(_services);
        }

        if (_options.Value.StopApplicationOnShutdown && _lifetime is not null)
        {
            _context.ShuttingDown += _ => _lifetime.StopApplication();
        }

        _logger?.LogInformation("ROS 2 context started on {Rmw} (domain {Domain}) with {Count} node(s)", _context.Rmw.Identifier, _context.DomainId, _context.Nodes.Count);
        var executor = _options.Value.CreateExecutor();
        _spin = Task.Run(() => _context.SpinAsync(_stopping.Token, executor), CancellationToken.None);
        return Task.CompletedTask;
    }

    public async Task StopAsync(CancellationToken cancellationToken)
    {
        await _stopping.CancelAsync().ConfigureAwait(false);
        if (_spin is not null)
        {
            try
            {
                await _spin.WaitAsync(cancellationToken).ConfigureAwait(false);
            }
            catch (OperationCanceledException)
            {
            }
        }

        _context?.Shutdown("host stopping");
        _stopping.Dispose();
    }
}
