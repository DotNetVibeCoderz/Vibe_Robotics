using ROS2.Net.AppGen;

namespace Ros2AppGen;

/// <summary>Creates the backend host from App.config next to the executable (created with defaults on first run).</summary>
internal static class HostBootstrap
{
    public static AppGenHost Create(string? configPath = null) => AppGenHost.Create(configPath);
}
