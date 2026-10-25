using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Control;

/// <summary>
/// Publishes every joint's position/velocity/effort state interfaces as <c>sensor_msgs/JointState</c>
/// on <c>/joint_states</c>, like <c>joint_state_broadcaster</c>. Parameter <c>publish_rate</c> (Hz, 0 = every update).
/// </summary>
public sealed class JointStateBroadcaster : ControllerBase
{
    private Publisher<JointState>? _publisher;
    private string[] _joints = [];
    private int[] _position = [];
    private int[] _velocity = [];
    private int[] _effort = [];
    private double _publishRate;
    private long _lastPublishNs = long.MinValue;

    public override InterfaceConfiguration CommandInterfaceConfiguration => InterfaceConfiguration.None;

    public override InterfaceConfiguration StateInterfaceConfiguration => InterfaceConfiguration.All;

    protected override void OnInit()
    {
        Node.DeclareParameter("publish_rate", 0.0);
        Node.DeclareParameter("joints", Array.Empty<string>());
    }

    protected internal override CallbackReturn OnConfigure()
    {
        _publishRate = Node.GetParameter<double>("publish_rate");
        _publisher ??= Node.CreatePublisher<JointState>("/joint_states");
        return CallbackReturn.Success;
    }

    protected internal override CallbackReturn OnActivate()
    {
        var filter = Node.GetParameter<string[]>("joints");
        var joints = StateInterfaces
            .Where(s => s.InterfaceName is HardwareInterfaceNames.Position or HardwareInterfaceNames.Velocity or HardwareInterfaceNames.Effort)
            .Select(s => s.PrefixName)
            .Distinct()
            .Where(j => filter.Length == 0 || filter.Contains(j))
            .ToArray();
        _joints = joints;
        _position = joints.Select(j => FindState(j + "/" + HardwareInterfaceNames.Position)).ToArray();
        _velocity = joints.Select(j => FindState(j + "/" + HardwareInterfaceNames.Velocity)).ToArray();
        _effort = joints.Select(j => FindState(j + "/" + HardwareInterfaceNames.Effort)).ToArray();
        return CallbackReturn.Success;
    }

    public override ControllerReturn Update(RosTime time, RosDuration period)
    {
        if (_publishRate > 0 && _lastPublishNs != long.MinValue && time.Nanoseconds - _lastPublishNs < 1e9 / _publishRate)
        {
            return ControllerReturn.Ok;
        }

        _lastPublishNs = time.Nanoseconds;
        var n = _joints.Length;
        var msg = new JointState { Name = _joints, Position = new double[n], Velocity = new double[n], Effort = new double[n] };
        msg.Header.Stamp = RosTime.Now().ToMsg();
        var states = StateInterfaces;
        for (var i = 0; i < n; i++)
        {
            msg.Position[i] = _position[i] >= 0 ? states[_position[i]].Value : double.NaN;
            msg.Velocity[i] = _velocity[i] >= 0 ? states[_velocity[i]].Value : double.NaN;
            msg.Effort[i] = _effort[i] >= 0 ? states[_effort[i]].Value : double.NaN;
        }

        _publisher!.Publish(msg);
        return ControllerReturn.Ok;
    }
}

/// <summary>
/// Forwards <c>std_msgs/Float64MultiArray</c> commands from <c>~/commands</c> to one interface per joint,
/// like <c>forward_command_controller</c> and the <c>JointGroup*Controller</c> family.
/// Parameters: <c>joints</c> (string[]), <c>interface_name</c>.
/// </summary>
public sealed class ForwardCommandController : ControllerBase
{
    private readonly string? _fixedInterface;
    private readonly RealtimeBuffer<double[]> _command = new();
    private string[] _joints = [];
    private string _interface = HardwareInterfaceNames.Position;

    public ForwardCommandController(string? interfaceName = null) => _fixedInterface = interfaceName;

    public override InterfaceConfiguration CommandInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_joints.Select(j => j + "/" + _interface));

    public override InterfaceConfiguration StateInterfaceConfiguration => InterfaceConfiguration.None;

    protected override void OnInit()
    {
        Node.DeclareParameter("joints", Array.Empty<string>());
        Node.DeclareParameter("interface_name", _fixedInterface ?? HardwareInterfaceNames.Position);
    }

    protected internal override CallbackReturn OnConfigure()
    {
        _joints = Node.GetParameter<string[]>("joints");
        _interface = _fixedInterface ?? Node.GetParameter<string>("interface_name");
        if (_joints.Length == 0)
        {
            return CallbackReturn.Failure;
        }

        Node.CreateSubscription<Float64MultiArray>("~/commands", m =>
        {
            if (m.Data.Length == _joints.Length)
            {
                _command.Write(m.Data);
            }
        });
        return CallbackReturn.Success;
    }

    protected internal override CallbackReturn OnActivate()
    {
        _command.Write(null);
        return CallbackReturn.Success;
    }

    /// <summary>Sets the command directly (bypassing the topic).</summary>
    public void SetCommand(double[] values) => _command.Write(values);

    public override ControllerReturn Update(RosTime time, RosDuration period)
    {
        var cmd = _command.Read();
        if (cmd is null)
        {
            return ControllerReturn.Ok;
        }

        var commands = CommandInterfaces;
        for (var i = 0; i < commands.Count && i < cmd.Length; i++)
        {
            commands[i].Value = cmd[i];
        }

        return ControllerReturn.Ok;
    }
}
