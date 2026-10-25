namespace ROS2.Net.Control;

/// <summary>Standard interface names, as in <c>hardware_interface/types/hardware_interface_type_values.hpp</c>.</summary>
public static class HardwareInterfaceNames
{
    public const string Position = "position";
    public const string Velocity = "velocity";
    public const string Acceleration = "acceleration";
    public const string Effort = "effort";
}

/// <summary>A shared double value. Hardware owns it; controllers read or write it through handles without copying or allocating.</summary>
public sealed class InterfaceSlot
{
    /// <summary>The value. A plain field so the control loop touches no properties or locks.</summary>
    public double Value = double.NaN;
}

/// <summary>Base of state and command handles: <c>prefix/interface</c>, e.g. <c>left_wheel_joint/velocity</c>.</summary>
public abstract class InterfaceHandle
{
    protected InterfaceHandle(string prefixName, string interfaceName, InterfaceSlot slot)
    {
        PrefixName = prefixName;
        InterfaceName = interfaceName;
        Slot = slot;
        Name = prefixName + "/" + interfaceName;
    }

    /// <summary>Usually the joint or sensor name.</summary>
    public string PrefixName { get; }

    /// <summary>E.g. <c>position</c>.</summary>
    public string InterfaceName { get; }

    /// <summary>Full name <c>prefix/interface</c>.</summary>
    public string Name { get; }

    internal InterfaceSlot Slot { get; }

    public override string ToString() => Name;
}

/// <summary>Read-only access to a hardware state value.</summary>
public sealed class StateInterface(string prefixName, string interfaceName, InterfaceSlot slot) : InterfaceHandle(prefixName, interfaceName, slot)
{
    public double Value => Slot.Value;
}

/// <summary>Read-write access to a hardware command value. Claimed exclusively by one controller.</summary>
public sealed class CommandInterface(string prefixName, string interfaceName, InterfaceSlot slot) : InterfaceHandle(prefixName, interfaceName, slot)
{
    public double Value
    {
        get => Slot.Value;
        set => Slot.Value = value;
    }
}

/// <summary>Kind of hardware component.</summary>
public enum HardwareKind
{
    /// <summary>Multiple joints with commands and states (a robot arm, a mobile base).</summary>
    System,
    /// <summary>One joint with command and state.</summary>
    Actuator,
    /// <summary>State only (IMU, force-torque sensor).</summary>
    Sensor,
}

/// <summary>Lifecycle state of a hardware component.</summary>
public enum HardwareState
{
    Unconfigured,
    Inactive,
    Active,
    Finalized,
}

/// <summary>Result of a hardware read/write.</summary>
public enum HardwareReturn
{
    Ok,
    Error,
}

/// <summary>A piece of hardware managed by the <see cref="ResourceManager"/>, like <c>hardware_interface::SystemInterface</c>.</summary>
public interface IHardwareInterface
{
    string Name { get; }

    HardwareKind Kind { get; }

    HardwareState State { get; }

    IReadOnlyList<StateInterface> StateInterfaces { get; }

    IReadOnlyList<CommandInterface> CommandInterfaces { get; }

    CallbackReturn Configure();

    CallbackReturn Activate();

    CallbackReturn Deactivate();

    /// <summary>Reads sensor values into the state interfaces. Called every control cycle; must not allocate.</summary>
    HardwareReturn Read(RosTime time, RosDuration period);

    /// <summary>Writes command interfaces to the hardware. Called every control cycle; must not allocate.</summary>
    HardwareReturn Write(RosTime time, RosDuration period);
}

/// <summary>
/// Convenience base for hardware: declare interfaces with <see cref="AddState"/> / <see cref="AddCommand"/>
/// in the constructor, then implement <see cref="Read"/> and <see cref="Write"/>.
/// </summary>
public abstract class HardwareComponent : IHardwareInterface
{
    private readonly List<StateInterface> _states = [];
    private readonly List<CommandInterface> _commands = [];

    protected HardwareComponent(string name, HardwareKind kind)
    {
        Name = name;
        Kind = kind;
    }

    public string Name { get; }

    public HardwareKind Kind { get; }

    public HardwareState State { get; private set; } = HardwareState.Unconfigured;

    public IReadOnlyList<StateInterface> StateInterfaces => _states;

    public IReadOnlyList<CommandInterface> CommandInterfaces => _commands;

    /// <summary>Declares a state interface and returns its slot for the component to write.</summary>
    protected InterfaceSlot AddState(string prefix, string interfaceName, double initial = 0)
    {
        var slot = new InterfaceSlot { Value = initial };
        _states.Add(new StateInterface(prefix, interfaceName, slot));
        return slot;
    }

    /// <summary>Declares a command interface and returns its slot for the component to read.</summary>
    protected InterfaceSlot AddCommand(string prefix, string interfaceName, double initial = double.NaN)
    {
        var slot = new InterfaceSlot { Value = initial };
        _commands.Add(new CommandInterface(prefix, interfaceName, slot));
        return slot;
    }

    public CallbackReturn Configure()
    {
        var r = OnConfigure();
        if (r == CallbackReturn.Success)
        {
            State = HardwareState.Inactive;
        }

        return r;
    }

    public CallbackReturn Activate()
    {
        var r = OnActivate();
        if (r == CallbackReturn.Success)
        {
            State = HardwareState.Active;
        }

        return r;
    }

    public CallbackReturn Deactivate()
    {
        var r = OnDeactivate();
        if (r == CallbackReturn.Success)
        {
            State = HardwareState.Inactive;
        }

        return r;
    }

    protected virtual CallbackReturn OnConfigure() => CallbackReturn.Success;

    protected virtual CallbackReturn OnActivate() => CallbackReturn.Success;

    protected virtual CallbackReturn OnDeactivate() => CallbackReturn.Success;

    public abstract HardwareReturn Read(RosTime time, RosDuration period);

    public abstract HardwareReturn Write(RosTime time, RosDuration period);
}
