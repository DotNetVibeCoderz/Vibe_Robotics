namespace ROS2.Net.Control;

/// <summary>How a controller selects interfaces.</summary>
public enum InterfaceConfigurationType
{
    /// <summary>Every available interface.</summary>
    All,
    /// <summary>The listed interfaces, in the listed order.</summary>
    Individual,
    /// <summary>No interfaces.</summary>
    None,
}

/// <summary>The command or state interfaces a controller requires.</summary>
public sealed record InterfaceConfiguration(InterfaceConfigurationType Type, IReadOnlyList<string> Names)
{
    public static InterfaceConfiguration None { get; } = new(InterfaceConfigurationType.None, []);

    public static InterfaceConfiguration All { get; } = new(InterfaceConfigurationType.All, []);

    public static InterfaceConfiguration Individual(IEnumerable<string> names) => new(InterfaceConfigurationType.Individual, names.ToList());

    public static InterfaceConfiguration Individual(params string[] names) => new(InterfaceConfigurationType.Individual, names);
}

/// <summary>Result of a controller update.</summary>
public enum ControllerReturn
{
    Ok,
    Error,
}

/// <summary>Controller lifecycle states, labelled like ros2_control (<c>unconfigured</c>, <c>inactive</c>, <c>active</c>, <c>finalized</c>).</summary>
public enum ControllerLifecycle
{
    Unconfigured,
    Inactive,
    Active,
    Finalized,
}

/// <summary>
/// Base of all controllers, like <c>controller_interface::ControllerInterface</c>. Each controller owns a
/// node named after it (parameters come from YAML/overrides under that name). <see cref="Update"/> runs on
/// the controller manager's real-time thread and must not allocate or block.
/// </summary>
public abstract class ControllerBase : IDisposable
{
    private IReadOnlyList<CommandInterface> _commands = [];
    private IReadOnlyList<StateInterface> _states = [];
    private Node? _node;

    /// <summary>Controller name, e.g. <c>diff_drive_controller</c>.</summary>
    public string Name { get; private set; } = string.Empty;

    /// <summary>Plugin type name, e.g. <c>diff_drive_controller/DiffDriveController</c>.</summary>
    public string TypeName { get; internal set; } = string.Empty;

    public ControllerLifecycle Lifecycle { get; internal set; } = ControllerLifecycle.Unconfigured;

    /// <summary>The controller's node (valid after loading).</summary>
    public Node Node => _node ?? throw new InvalidOperationException("The controller has not been loaded.");

    /// <summary>The controller's own update rate in Hz, or 0 to run at the manager's rate.</summary>
    public int UpdateRate { get; protected set; }

    internal long LastUpdateNs { get; set; } = long.MinValue;

    /// <summary>Claimed command interfaces, ordered as requested by <see cref="CommandInterfaceConfiguration"/>.</summary>
    protected IReadOnlyList<CommandInterface> CommandInterfaces => _commands;

    /// <summary>Assigned state interfaces, ordered as requested by <see cref="StateInterfaceConfiguration"/>.</summary>
    protected IReadOnlyList<StateInterface> StateInterfaces => _states;

    /// <summary>Interfaces the controller will claim. Evaluated after <see cref="OnConfigure"/>, so it may depend on parameters.</summary>
    public abstract InterfaceConfiguration CommandInterfaceConfiguration { get; }

    public abstract InterfaceConfiguration StateInterfaceConfiguration { get; }

    internal void Initialize(Node node, string name)
    {
        _node = node;
        Name = name;
        UpdateRate = (int)node.DeclareParameter("update_rate", 0L);
        OnInit();
    }

    internal void Assign(IReadOnlyList<CommandInterface> commands, IReadOnlyList<StateInterface> states)
    {
        _commands = commands;
        _states = states;
    }

    internal void Unassign()
    {
        _commands = [];
        _states = [];
    }

    /// <summary>Declare parameters here.</summary>
    protected virtual void OnInit()
    {
    }

    /// <summary>Read parameters and create publishers/subscriptions.</summary>
    protected internal virtual CallbackReturn OnConfigure() => CallbackReturn.Success;

    /// <summary>Interfaces are assigned; initialize commands (e.g. hold the current position).</summary>
    protected internal virtual CallbackReturn OnActivate() => CallbackReturn.Success;

    protected internal virtual CallbackReturn OnDeactivate() => CallbackReturn.Success;

    protected internal virtual CallbackReturn OnCleanup() => CallbackReturn.Success;

    /// <summary>One control cycle. Real-time: no allocation, no blocking, no logging.</summary>
    public abstract ControllerReturn Update(RosTime time, RosDuration period);

    /// <summary>Index of a command interface by full name, or -1.</summary>
    protected int FindCommand(string name)
    {
        for (var i = 0; i < _commands.Count; i++)
        {
            if (_commands[i].Name == name)
            {
                return i;
            }
        }

        return -1;
    }

    /// <summary>Index of a state interface by full name, or -1.</summary>
    protected int FindState(string name)
    {
        for (var i = 0; i < _states.Count; i++)
        {
            if (_states[i].Name == name)
            {
                return i;
            }
        }

        return -1;
    }

    public void Dispose()
    {
        _node?.Dispose();
        GC.SuppressFinalize(this);
    }
}

/// <summary>
/// Hands data from non-real-time threads (subscription callbacks) to the control loop without locks:
/// the writer publishes a new immutable reference, the loop reads the latest one.
/// </summary>
public sealed class RealtimeBuffer<T>
    where T : class
{
    private T? _value;

    public void Write(T? value) => Volatile.Write(ref _value, value);

    public T? Read() => Volatile.Read(ref _value);

    /// <summary>Takes the value and clears the buffer.</summary>
    public T? Take() => Interlocked.Exchange(ref _value, null);
}
