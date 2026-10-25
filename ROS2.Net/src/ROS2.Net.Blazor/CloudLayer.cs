using ROS2.Net.Perception;

namespace ROS2.Net.Blazor;

/// <summary>One colored cloud in a <c>PointCloudView</c>.</summary>
public sealed record CloudLayer(PointCloud Cloud, string Color, double Opacity = 0.95);

/// <summary>2D projection used by <c>PointCloudView</c>.</summary>
public enum CloudProjection
{
    /// <summary>Looking down: x forward (up), y left.</summary>
    Top,
    /// <summary>x right, z up.</summary>
    Front,
    /// <summary>y right, z up.</summary>
    Side,
}
