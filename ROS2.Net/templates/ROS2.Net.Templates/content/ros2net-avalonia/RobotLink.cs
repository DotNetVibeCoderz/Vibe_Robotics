using ROS2.Net;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace Ros2NetAvalonia;

/// <summary>The app's ROS node: listens to /chatter and /odom, drives /cmd_vel.</summary>
public sealed class RobotLink : Node
{
    private readonly Publisher<Twist> _cmdVel;

    public RobotLink(Ros2Context context)
        : base(context, "avalonia_dashboard")
    {
        _cmdVel = CreatePublisher<Twist>("cmd_vel");
        CreateSubscription<StdMsgs.String>("chatter", m => ChatterReceived?.Invoke(m.Data));
        CreateSubscription<Odometry>("odom", m =>
            PoseReceived?.Invoke((m.Pose.Pose.Position.X, m.Pose.Pose.Position.Y, m.Pose.Pose.Orientation.Yaw())), QosProfile.SensorData);
    }

    /// <summary>Raised on a ROS thread; marshal to the UI thread before touching controls.</summary>
    public event Action<string>? ChatterReceived;

    public event Action<(double X, double Y, double Yaw)>? PoseReceived;

    public void Drive(double linear, double angular) => _cmdVel.Publish(MessageExtensions.Twist2D(linear, angular));
}
