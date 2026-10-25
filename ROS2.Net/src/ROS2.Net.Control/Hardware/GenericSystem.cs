namespace ROS2.Net.Control;

/// <summary>
/// A mock system like <c>mock_components/GenericSystem</c>: every joint gets position, velocity and effort
/// state interfaces and the requested command interfaces. Commands are mirrored to states, either
/// instantly or through a first-order lag (<see cref="TimeConstant"/>), and velocity commands are integrated.
/// </summary>
public sealed class GenericSystem : HardwareComponent
{
    private readonly Joint[] _joints;

    private sealed class Joint
    {
        public required InterfaceSlot Position;
        public required InterfaceSlot Velocity;
        public required InterfaceSlot Effort;
        public InterfaceSlot? PositionCommand;
        public InterfaceSlot? VelocityCommand;
        public InterfaceSlot? EffortCommand;
    }

    /// <param name="name">Component name.</param>
    /// <param name="joints">Joint names.</param>
    /// <param name="commandInterfaces">Command interfaces per joint (default: position).</param>
    /// <param name="initialPositions">Optional initial positions.</param>
    public GenericSystem(string name, IReadOnlyList<string> joints, IReadOnlyList<string>? commandInterfaces = null, IReadOnlyList<double>? initialPositions = null)
        : base(name, HardwareKind.System)
    {
        commandInterfaces ??= [HardwareInterfaceNames.Position];
        _joints = new Joint[joints.Count];
        for (var i = 0; i < joints.Count; i++)
        {
            var j = joints[i];
            var p0 = initialPositions is not null && i < initialPositions.Count ? initialPositions[i] : 0;
            var joint = new Joint
            {
                Position = AddState(j, HardwareInterfaceNames.Position, p0),
                Velocity = AddState(j, HardwareInterfaceNames.Velocity),
                Effort = AddState(j, HardwareInterfaceNames.Effort),
            };
            foreach (var ci in commandInterfaces)
            {
                switch (ci)
                {
                    case HardwareInterfaceNames.Position: joint.PositionCommand = AddCommand(j, ci, p0); break;
                    case HardwareInterfaceNames.Velocity: joint.VelocityCommand = AddCommand(j, ci, 0); break;
                    case HardwareInterfaceNames.Effort: joint.EffortCommand = AddCommand(j, ci, 0); break;
                    default: throw new ArgumentException($"Unsupported command interface '{ci}'.", nameof(commandInterfaces));
                }
            }

            _joints[i] = joint;
        }
    }

    /// <summary>First-order time constant (s) of the position response. 0 = commands are applied instantly.</summary>
    public double TimeConstant { get; set; }

    /// <summary>Offset added to mirrored positions (simulates a calibration error).</summary>
    public double PositionStateOffset { get; set; }

    public override HardwareReturn Read(RosTime time, RosDuration period)
    {
        var dt = period.TotalSeconds;
        foreach (var j in _joints)
        {
            if (j.PositionCommand is { } pc && !double.IsNaN(pc.Value))
            {
                var previous = j.Position.Value;
                var target = pc.Value + PositionStateOffset;
                var next = TimeConstant > 0 && dt > 0 ? previous + ((target - previous) * Math.Min(1.0, dt / TimeConstant)) : target;
                j.Position.Value = next;
                j.Velocity.Value = dt > 0 ? (next - previous) / dt : 0;
            }
            else if (j.VelocityCommand is { } vc && !double.IsNaN(vc.Value))
            {
                var v = TimeConstant > 0 && dt > 0 ? j.Velocity.Value + ((vc.Value - j.Velocity.Value) * Math.Min(1.0, dt / TimeConstant)) : vc.Value;
                j.Velocity.Value = v;
                j.Position.Value += v * dt;
            }

            if (j.EffortCommand is { } ec && !double.IsNaN(ec.Value))
            {
                j.Effort.Value = ec.Value;
            }
        }

        return HardwareReturn.Ok;
    }

    public override HardwareReturn Write(RosTime time, RosDuration period) => HardwareReturn.Ok;
}

/// <summary>
/// A simulated two-wheel base: velocity commands on <c>left/velocity</c> and <c>right/velocity</c> (rad/s)
/// drive wheel position/velocity states, with optional wheel slip noise.
/// </summary>
public sealed class SimulatedDiffDriveHardware : HardwareComponent
{
    private readonly InterfaceSlot _lp;
    private readonly InterfaceSlot _lv;
    private readonly InterfaceSlot _rp;
    private readonly InterfaceSlot _rv;
    private readonly InterfaceSlot _lc;
    private readonly InterfaceSlot _rc;

    public SimulatedDiffDriveHardware(string name = "DiffBot", string leftJoint = "left_wheel_joint", string rightJoint = "right_wheel_joint")
        : base(name, HardwareKind.System)
    {
        LeftJoint = leftJoint;
        RightJoint = rightJoint;
        _lp = AddState(leftJoint, HardwareInterfaceNames.Position);
        _lv = AddState(leftJoint, HardwareInterfaceNames.Velocity);
        _rp = AddState(rightJoint, HardwareInterfaceNames.Position);
        _rv = AddState(rightJoint, HardwareInterfaceNames.Velocity);
        _lc = AddCommand(leftJoint, HardwareInterfaceNames.Velocity, 0);
        _rc = AddCommand(rightJoint, HardwareInterfaceNames.Velocity, 0);
    }

    public string LeftJoint { get; }

    public string RightJoint { get; }

    /// <summary>Maximum wheel speed (rad/s); commands beyond it saturate.</summary>
    public double MaxWheelSpeed { get; set; } = 50;

    public override HardwareReturn Read(RosTime time, RosDuration period)
    {
        var dt = period.TotalSeconds;
        _lv.Value = double.IsNaN(_lc.Value) ? 0 : Math.Clamp(_lc.Value, -MaxWheelSpeed, MaxWheelSpeed);
        _rv.Value = double.IsNaN(_rc.Value) ? 0 : Math.Clamp(_rc.Value, -MaxWheelSpeed, MaxWheelSpeed);
        _lp.Value += _lv.Value * dt;
        _rp.Value += _rv.Value * dt;
        return HardwareReturn.Ok;
    }

    public override HardwareReturn Write(RosTime time, RosDuration period) => HardwareReturn.Ok;
}
