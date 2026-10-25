using ROS2.Net;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace Ros2NetBlazor.Services;

/// <summary>The app's ROS node: listens to /chatter and /odom and drives /cmd_vel. Components subscribe to <see cref="Changed"/>.</summary>
public sealed class RobotLink : Node
{
    private readonly Publisher<Twist> _cmdVel;
    private readonly Queue<string> _chatter = new();

    public RobotLink(Ros2Context context)
        : base(context, "blazor_dashboard")
    {
        _cmdVel = CreatePublisher<Twist>("cmd_vel");
        CreateSubscription<StdMsgs.String>("chatter", m =>
        {
            lock (_chatter)
            {
                _chatter.Enqueue($"{DateTime.Now:HH:mm:ss}  {m.Data}");
                while (_chatter.Count > 20)
                {
                    _chatter.Dequeue();
                }
            }

            Changed?.Invoke();
        });
        CreateSubscription<Odometry>("odom", m =>
        {
            Pose = (m.Pose.Pose.Position.X, m.Pose.Pose.Position.Y, m.Pose.Pose.Orientation.Yaw());
            Changed?.Invoke();
        }, QosProfile.SensorData);
    }

    /// <summary>Raised on a ROS thread whenever new data arrived.</summary>
    public event Action? Changed;

    public (double X, double Y, double Yaw) Pose { get; private set; }

    public IReadOnlyList<string> Chatter
    {
        get
        {
            lock (_chatter)
            {
                return _chatter.Reverse().ToList();
            }
        }
    }

    public void Drive(double linear, double angular) => _cmdVel.Publish(MessageExtensions.Twist2D(linear, angular));

    public IReadOnlyList<ROS2.Net.Rmw.TopicInfo> Topics => GetTopicNamesAndTypes();
}
