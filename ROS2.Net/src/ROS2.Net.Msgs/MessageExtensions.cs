using ROS2.Net.Msgs.BuiltinInterfaces;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Msgs;

/// <summary>Conveniences for the most used standard messages.</summary>
public static class MessageExtensions
{
    /// <summary>Forces registration of every standard interface type (for tools that resolve types by name).</summary>
    public static void EnsureRegistered() => RosTypeRegistry.EnsureLoaded(typeof(Header).Assembly);

    /// <summary>Sets the header stamp and frame.</summary>
    public static Header WithStamp(this Header header, RosTime time, string? frameId = null)
    {
        time.CopyTo(header.Stamp);
        if (frameId is not null)
        {
            header.FrameId = frameId;
        }

        return header;
    }

    public static RosTime ToRosTime(this Time time) => RosTime.FromMsg(time);

    public static Vector3 ToVector3(this (double X, double Y, double Z) v) => new() { X = v.X, Y = v.Y, Z = v.Z };

    public static Point ToPoint(this (double X, double Y, double Z) v) => new() { X = v.X, Y = v.Y, Z = v.Z };

    /// <summary>A quaternion for a rotation of <paramref name="yaw"/> radians about Z.</summary>
    public static Quaternion QuaternionFromYaw(double yaw) => QuaternionFromRpy(0, 0, yaw);

    /// <summary>A quaternion from roll/pitch/yaw (radians, extrinsic X-Y-Z, as tf2 uses).</summary>
    public static Quaternion QuaternionFromRpy(double roll, double pitch, double yaw)
    {
        double cr = Math.Cos(roll * 0.5), sr = Math.Sin(roll * 0.5);
        double cp = Math.Cos(pitch * 0.5), sp = Math.Sin(pitch * 0.5);
        double cy = Math.Cos(yaw * 0.5), sy = Math.Sin(yaw * 0.5);
        return new Quaternion
        {
            W = (cr * cp * cy) + (sr * sp * sy),
            X = (sr * cp * cy) - (cr * sp * sy),
            Y = (cr * sp * cy) + (sr * cp * sy),
            Z = (cr * cp * sy) - (sr * sp * cy),
        };
    }

    /// <summary>Yaw (rotation about Z, radians) of a quaternion.</summary>
    public static double Yaw(this Quaternion q) =>
        Math.Atan2(2 * ((q.W * q.Z) + (q.X * q.Y)), 1 - (2 * ((q.Y * q.Y) + (q.Z * q.Z))));

    /// <summary>Roll, pitch and yaw (radians) of a quaternion.</summary>
    public static (double Roll, double Pitch, double Yaw) ToRpy(this Quaternion q)
    {
        var sinr = 2 * ((q.W * q.X) + (q.Y * q.Z));
        var cosr = 1 - (2 * ((q.X * q.X) + (q.Y * q.Y)));
        var sinp = 2 * ((q.W * q.Y) - (q.Z * q.X));
        var pitch = Math.Abs(sinp) >= 1 ? Math.CopySign(Math.PI / 2, sinp) : Math.Asin(sinp);
        return (Math.Atan2(sinr, cosr), pitch, q.Yaw());
    }

    /// <summary>A <c>geometry_msgs/Twist</c> for planar motion.</summary>
    public static Twist Twist2D(double linearX, double angularZ) => new()
    {
        Linear = new Vector3 { X = linearX },
        Angular = new Vector3 { Z = angularZ },
    };
}
