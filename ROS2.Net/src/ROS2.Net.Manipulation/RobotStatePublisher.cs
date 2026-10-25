using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.Tf2Msgs;
using StdString = ROS2.Net.Msgs.StdMsgs.String;

namespace ROS2.Net.Manipulation;

/// <summary>
/// The equivalent of <c>robot_state_publisher</c>: publishes the URDF on <c>/robot_description</c> (latched) and the
/// <c>robot_description</c> parameter, fixed joints on <c>/tf_static</c>, and movable joints on <c>/tf</c> from
/// <c>/joint_states</c>. Mimic joints follow their master.
/// </summary>
public sealed class RobotStatePublisher : Node
{
    private readonly Publisher<TFMessage> _tf;
    private readonly Publisher<TFMessage> _tfStatic;
    private readonly Dictionary<string, double> _positions = new(StringComparer.Ordinal);
    private readonly RobotKinematics _kinematics;

    public RobotStatePublisher(Ros2Context context, UrdfModel model, string name = "robot_state_publisher", NodeOptions? options = null)
        : base(context, name, options)
    {
        Model = model;
        _kinematics = new RobotKinematics(model);
        FramePrefix = DeclareParameter("frame_prefix", string.Empty);
        DeclareParameter("robot_description", model.Xml, "The URDF of the robot.", readOnly: true);

        CreatePublisher<StdString>("/robot_description", QosProfile.Latched).Publish(new StdString { Data = model.Xml });
        _tf = CreatePublisher<TFMessage>("/tf", QosProfile.Default.KeepLast(100));
        _tfStatic = CreatePublisher<TFMessage>("/tf_static", QosProfile.Latched);
        PublishFixed();
        CreateSubscription<JointState>("/joint_states", OnJointState);
    }

    public UrdfModel Model { get; }

    public string FramePrefix { get; }

    /// <summary>Publishes transforms for the given joint positions immediately.</summary>
    public void PublishJointPositions(IReadOnlyDictionary<string, double> positions, RosTime? stamp = null)
    {
        lock (_positions)
        {
            foreach (var (k, v) in positions)
            {
                _positions[k] = v;
            }
        }

        PublishMoving(stamp ?? Clock.Now());
    }

    /// <summary>World pose of every link at the last known joint positions.</summary>
    public Dictionary<string, Pose3> LinkPoses()
    {
        lock (_positions)
        {
            return _kinematics.LinkPoses(new Dictionary<string, double>(_positions));
        }
    }

    private void OnJointState(JointState msg)
    {
        lock (_positions)
        {
            for (var i = 0; i < msg.Name.Length && i < msg.Position.Length; i++)
            {
                _positions[msg.Name[i]] = msg.Position[i];
            }
        }

        var stamp = msg.Header.Stamp.Sec == 0 && msg.Header.Stamp.Nanosec == 0 ? Clock.Now() : RosTime.FromMsg(msg.Header.Stamp);
        PublishMoving(stamp);
    }

    private void PublishFixed()
    {
        var stamp = Clock.Now().ToMsg();
        var transforms = Model.Joints.Where(j => j.Type == JointType.Fixed)
            .Select(j => Stamped(j, j.Origin, stamp))
            .ToArray();
        if (transforms.Length > 0)
        {
            _tfStatic.Publish(new TFMessage { Transforms = transforms });
        }
    }

    private void PublishMoving(RosTime stamp)
    {
        var msgStamp = stamp.ToMsg();
        var list = new List<TransformStamped>();
        lock (_positions)
        {
            foreach (var j in Model.Joints)
            {
                if (!j.IsMovable)
                {
                    continue;
                }

                var master = j.Mimic?.Joint ?? j.Name;
                if (!_positions.ContainsKey(master))
                {
                    continue;
                }

                var q = _kinematics.JointPosition(j, n => _positions.TryGetValue(n, out var v) ? v : null);
                list.Add(Stamped(j, j.Transform(q), msgStamp));
            }
        }

        if (list.Count > 0)
        {
            _tf.Publish(new TFMessage { Transforms = list.ToArray() });
        }
    }

    private TransformStamped Stamped(UrdfJoint j, Pose3 pose, Msgs.BuiltinInterfaces.Time stamp)
    {
        var t = new TransformStamped { ChildFrameId = FramePrefix + j.Child, Transform = pose.ToTransformMsg() };
        t.Header.FrameId = FramePrefix + j.Parent;
        t.Header.Stamp = stamp;
        return t;
    }
}
