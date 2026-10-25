using System.Globalization;
using Microsoft.Extensions.DependencyInjection;
using ROS2.Net.Hosting;

namespace ROS2.Net.Blazor;

/// <summary>Registration of ROS2.Net for Blazor apps.</summary>
public static class Ros2BlazorExtensions
{
    /// <summary>
    /// Registers a singleton <see cref="Ros2Context"/> spun by a hosted service (see ROS2.Net.Hosting).
    /// Components derive from <see cref="RosComponentBase"/> and inject it. Add
    /// <c>&lt;link href="_content/ROS2.Net.Blazor/ros2net.css" rel="stylesheet" /&gt;</c> for the theme tokens.
    /// </summary>
    public static IServiceCollection AddRos2Blazor(this IServiceCollection services, Action<Ros2HostOptions>? configure = null)
    {
        services.AddRos2(o =>
        {
            o.Executor = Ros2ExecutorKind.MultiThreaded;
            configure?.Invoke(o);
        });
        return services;
    }
}

/// <summary>A planar pose for overlays (meters, radians).</summary>
public readonly record struct PlanarPose(double X, double Y, double Theta = 0)
{
    public static PlanarPose FromMsg(Msgs.GeometryMsgs.Pose pose)
    {
        var q = pose.Orientation;
        var yaw = Math.Atan2(2 * ((q.W * q.Z) + (q.X * q.Y)), 1 - (2 * ((q.Y * q.Y) + (q.Z * q.Z))));
        return new PlanarPose(pose.Position.X, pose.Position.Y, yaw);
    }
}

internal static class Svg
{
    public static string F(double v) => double.IsFinite(v) ? v.ToString("0.###", CultureInfo.InvariantCulture) : "0";
}
