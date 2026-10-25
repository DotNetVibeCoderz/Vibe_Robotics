using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Msgs.Tf2Msgs;
using ROS2.Net.Navigation.Control;

namespace ROS2.Net.Navigation.Nodes;

/// <summary>Supplies the robot pose in the navigation (map) frame. Replaces a TF lookup of <c>map → base_link</c>.</summary>
public interface IRobotPoseProvider
{
    /// <summary>Returns false until a pose is known.</summary>
    bool TryGetPose(out Pose2D pose, out Velocity2D velocity);
}

/// <summary>A pose provider backed by a delegate (simulators, motion capture, tests, a TF buffer).</summary>
public sealed class DelegatePoseProvider(Func<(Pose2D Pose, Velocity2D Velocity)?> source) : IRobotPoseProvider
{
    public bool TryGetPose(out Pose2D pose, out Velocity2D velocity)
    {
        if (source() is { } v)
        {
            (pose, velocity) = v;
            return true;
        }

        pose = default;
        velocity = default;
        return false;
    }
}

/// <summary>
/// Tracks <c>nav_msgs/Odometry</c> and reports <c>Correction ⊕ odom_pose</c>, where
/// <see cref="Correction"/> is the map→odom transform maintained by AMCL or SLAM (identity when the
/// odometry frame is used as the map frame). Reads the topic without needing an executor.
/// </summary>
public sealed class OdometryPoseProvider : IRobotPoseProvider, IDisposable
{
    private readonly Subscription<Odometry> _subscription;
    private readonly CancellationTokenSource _cts = new();
    private readonly object _gate = new();
    private Pose2D _odomPose;
    private Velocity2D _velocity;
    private bool _hasPose;

    public OdometryPoseProvider(Node node, string topic = "/odom")
    {
        _subscription = node.CreateSubscription<Odometry>(topic, QosProfile.SensorData.KeepLast(10).AsReliable());
        _ = Task.Run(async () =>
        {
            try
            {
                await foreach (var msg in _subscription.ReadAllAsync(_cts.Token).ConfigureAwait(false))
                {
                    lock (_gate)
                    {
                        _odomPose = Pose2D.FromMsg(msg.Pose.Pose);
                        _velocity = new Velocity2D(msg.Twist.Twist.Linear.X, msg.Twist.Twist.Angular.Z);
                        _hasPose = true;
                    }

                    OdometryReceived?.Invoke(_odomPose);
                }
            }
            catch (OperationCanceledException)
            {
            }
        });
    }

    /// <summary>The map→odom correction applied to odometry poses.</summary>
    public Pose2D Correction { get; set; }

    /// <summary>The latest raw odometry pose.</summary>
    public Pose2D OdometryPose
    {
        get
        {
            lock (_gate)
            {
                return _odomPose;
            }
        }
    }

    public bool HasOdometry => _hasPose;

    /// <summary>Raised on the reader task for each odometry message.</summary>
    public event Action<Pose2D>? OdometryReceived;

    public bool TryGetPose(out Pose2D pose, out Velocity2D velocity)
    {
        lock (_gate)
        {
            pose = Correction.Compose(_odomPose);
            velocity = _velocity;
            return _hasPose;
        }
    }

    public void Dispose()
    {
        _cts.Cancel();
        _subscription.Dispose();
        _cts.Dispose();
    }
}

/// <summary>Helpers to publish planar transforms on <c>/tf</c>.</summary>
public static class TfHelpers
{
    public static TransformStamped ToTransform(Pose2D pose, string parent, string child, RosTime stamp)
    {
        var t = new TransformStamped { ChildFrameId = child };
        stamp.CopyTo(t.Header.Stamp);
        t.Header.FrameId = parent;
        t.Transform.Translation.X = pose.X;
        t.Transform.Translation.Y = pose.Y;
        t.Transform.Rotation = Msgs.MessageExtensions.QuaternionFromYaw(pose.Theta);
        return t;
    }

    public static TFMessage ToTfMessage(Pose2D pose, string parent, string child, RosTime stamp) =>
        new() { Transforms = [ToTransform(pose, parent, child, stamp)] };
}
