namespace ROS2.Net.Control;

/// <summary>Raised when a command interface is already claimed or does not exist.</summary>
public sealed class InterfaceClaimException(string message) : InvalidOperationException(message);

/// <summary>
/// Owns hardware components and their interfaces, like <c>hardware_interface::ResourceManager</c>.
/// Command interfaces can be claimed by one controller at a time.
/// </summary>
public sealed class ResourceManager
{
    private readonly List<IHardwareInterface> _hardware = [];
    private readonly Dictionary<string, StateInterface> _states = new(StringComparer.Ordinal);
    private readonly Dictionary<string, CommandInterface> _commands = new(StringComparer.Ordinal);
    private readonly Dictionary<string, string> _claims = new(StringComparer.Ordinal);
    private IHardwareInterface[] _snapshot = [];

    public IReadOnlyList<IHardwareInterface> Hardware => _hardware;

    public IEnumerable<StateInterface> StateInterfaces => _states.Values;

    public IEnumerable<CommandInterface> CommandInterfaces => _commands.Values;

    /// <summary>Adds a component, then configures and activates it.</summary>
    public void Add(IHardwareInterface hardware, bool activate = true)
    {
        foreach (var s in hardware.StateInterfaces)
        {
            if (!_states.TryAdd(s.Name, s))
            {
                throw new ArgumentException($"State interface '{s.Name}' is exported twice.", nameof(hardware));
            }
        }

        foreach (var c in hardware.CommandInterfaces)
        {
            if (!_commands.TryAdd(c.Name, c))
            {
                throw new ArgumentException($"Command interface '{c.Name}' is exported twice.", nameof(hardware));
            }
        }

        _hardware.Add(hardware);
        _snapshot = _hardware.ToArray();
        if (activate)
        {
            if (hardware.Configure() != CallbackReturn.Success || hardware.Activate() != CallbackReturn.Success)
            {
                throw new InvalidOperationException($"Hardware '{hardware.Name}' failed to start.");
            }
        }
    }

    public bool HasStateInterface(string name) => _states.ContainsKey(name);

    public bool HasCommandInterface(string name) => _commands.ContainsKey(name);

    public StateInterface GetStateInterface(string name) =>
        _states.TryGetValue(name, out var s) ? s : throw new KeyNotFoundException($"No state interface '{name}'.");

    public bool IsClaimed(string commandInterface) => _claims.ContainsKey(commandInterface);

    /// <summary>The controller holding a command interface, if any.</summary>
    public string? ClaimedBy(string commandInterface) => _claims.GetValueOrDefault(commandInterface);

    /// <summary>Claims command interfaces for <paramref name="owner"/>: all or none.</summary>
    public IReadOnlyList<CommandInterface> Claim(string owner, IEnumerable<string> names)
    {
        var list = names.ToList();
        foreach (var n in list)
        {
            if (!_commands.ContainsKey(n))
            {
                throw new InterfaceClaimException($"Command interface '{n}' does not exist.");
            }

            if (_claims.TryGetValue(n, out var holder) && holder != owner)
            {
                throw new InterfaceClaimException($"Command interface '{n}' is already claimed by '{holder}'.");
            }
        }

        foreach (var n in list)
        {
            _claims[n] = owner;
        }

        return list.Select(n => _commands[n]).ToList();
    }

    public void Release(string owner)
    {
        foreach (var key in _claims.Where(kv => kv.Value == owner).Select(kv => kv.Key).ToList())
        {
            _claims.Remove(key);
        }
    }

    /// <summary>Reads every active component. Allocation-free.</summary>
    public bool ReadAll(RosTime time, RosDuration period)
    {
        var ok = true;
        foreach (var h in _snapshot)
        {
            if (h.State == HardwareState.Active)
            {
                ok &= h.Read(time, period) == HardwareReturn.Ok;
            }
        }

        return ok;
    }

    /// <summary>Writes every active component. Allocation-free.</summary>
    public bool WriteAll(RosTime time, RosDuration period)
    {
        var ok = true;
        foreach (var h in _snapshot)
        {
            if (h.State == HardwareState.Active)
            {
                ok &= h.Write(time, period) == HardwareReturn.Ok;
            }
        }

        return ok;
    }
}
