namespace ROS2.Net.Simulation;

/// <summary>Immutable state of a world at one instant, for renderers and UIs.</summary>
/// <param name="Name">World name.</param>
/// <param name="Time">Simulation time (s).</param>
/// <param name="Step">Step counter.</param>
/// <param name="Bounds">World bounds.</param>
/// <param name="FloorColor">Floor color.</param>
/// <param name="StaticObstacles">Static obstacles (the same array instance until static geometry changes).</param>
/// <param name="DynamicObstacles">Pushable obstacles at their current poses.</param>
/// <param name="Robots">Robots.</param>
/// <param name="Paused">Whether the world is paused.</param>
/// <param name="RealTimeFactor">Configured real-time factor.</param>
public sealed record WorldSnapshot(
    string Name,
    double Time,
    long Step,
    Bounds2D Bounds,
    SimColor FloorColor,
    IReadOnlyList<ObstacleSnapshot> StaticObstacles,
    IReadOnlyList<ObstacleSnapshot> DynamicObstacles,
    IReadOnlyList<RobotSnapshot> Robots,
    bool Paused,
    double RealTimeFactor)
{
    /// <summary>Static and dynamic obstacles together.</summary>
    public IEnumerable<ObstacleSnapshot> Obstacles => StaticObstacles.Concat(DynamicObstacles);
}

/// <summary>Render-facing description of an obstacle.</summary>
/// <param name="Name">Name.</param>
/// <param name="Kind">Shape kind.</param>
/// <param name="Center">Center in world coordinates.</param>
/// <param name="Rotation">Rotation (rad).</param>
/// <param name="Size">Full width/depth of boxes and walls (zero otherwise).</param>
/// <param name="Radius">Radius of cylinders (zero otherwise).</param>
/// <param name="Vertices">World-space outline (empty for cylinders).</param>
/// <param name="Height">Height (m).</param>
/// <param name="Color">Color.</param>
/// <param name="IsDynamic">Pushable.</param>
/// <param name="Tag">Tag such as <c>shelf</c> or <c>wall</c>.</param>
public sealed record ObstacleSnapshot(
    string Name,
    ShapeKind Kind,
    Vec2 Center,
    double Rotation,
    Vec2 Size,
    double Radius,
    IReadOnlyList<Vec2> Vertices,
    double Height,
    SimColor Color,
    bool IsDynamic,
    string Tag)
{
    internal static ObstacleSnapshot From(SimObstacle o) => new(
        o.Name, o.Kind, o.Center, o.Rotation, o.HalfExtents * 2, o.Radius, o.Vertices.ToArray(), o.Height, o.Color, o.IsDynamic, o.Tag);
}

/// <summary>Render-facing description of a robot.</summary>
/// <param name="Name">Robot name.</param>
/// <param name="Namespace">ROS namespace.</param>
/// <param name="Kind">Drive kind.</param>
/// <param name="Pose">Ground-truth pose.</param>
/// <param name="OdomPose">Odometry estimate in the odom frame.</param>
/// <param name="LinearVelocity">Forward speed (m/s).</param>
/// <param name="LateralVelocity">Sideways speed (m/s).</param>
/// <param name="AngularVelocity">Yaw rate (rad/s).</param>
/// <param name="Radius">Collision radius (m).</param>
/// <param name="Length">Footprint length (m).</param>
/// <param name="Width">Footprint width (m).</param>
/// <param name="Height">Height (m).</param>
/// <param name="Color">Body color.</param>
/// <param name="Bumper">In contact.</param>
/// <param name="BatteryPercentage">State of charge in [0,1], or NaN without a battery.</param>
/// <param name="SteeringAngle">Ackermann steering angle (rad).</param>
/// <param name="LaserOrigin">LiDAR world pose at its last update, if any.</param>
/// <param name="LaserHits">World-space beam end points of the last scan (empty without LiDAR).</param>
/// <param name="Trail">Recent positions, oldest first.</param>
/// <param name="JointNames">Arm joint names (empty without an arm).</param>
/// <param name="JointPositions">Arm joint positions (rad).</param>
/// <param name="ArmLinkLengths">Arm link lengths for drawing (m).</param>
public sealed record RobotSnapshot(
    string Name,
    string Namespace,
    RobotKind Kind,
    Pose2D Pose,
    Pose2D OdomPose,
    double LinearVelocity,
    double LateralVelocity,
    double AngularVelocity,
    double Radius,
    double Length,
    double Width,
    double Height,
    SimColor Color,
    bool Bumper,
    double BatteryPercentage,
    double SteeringAngle,
    Pose2D? LaserOrigin,
    IReadOnlyList<Vec2> LaserHits,
    IReadOnlyList<Vec2> Trail,
    IReadOnlyList<string> JointNames,
    IReadOnlyList<double> JointPositions,
    IReadOnlyList<double> ArmLinkLengths)
{
    internal static RobotSnapshot From(SimRobot r)
    {
        var p = r.Parameters;
        var lidar = r.Lidar;
        var battery = r.Battery;
        return new RobotSnapshot(
            r.Name, r.Namespace, r.Kind, r.Pose, r.OdomPose, r.LinearVelocity, r.LateralVelocity, r.AngularVelocity,
            p.Radius, p.Length, p.Width, p.Height, p.Color, r.Bumper, battery?.Percentage ?? double.NaN, r.SteeringAngle,
            lidar is { Sequence: > 0 } ? lidar.LastOrigin : null,
            lidar is { Sequence: > 0 } ? (Vec2[])lidar.HitPoints.Clone() : [],
            r.CopyTrail(),
            r.Arm?.JointNames ?? [],
            r.Arm is { } arm ? (double[])arm.Positions.Clone() : [],
            r.Arm?.LinkLengths ?? []);
    }
}
