using ROS2.Net.Simulation;

namespace ROS2.Net.AI.Gym;

/// <summary>
/// The observation vector shared by <see cref="RobotGymEnv"/> (training) and
/// <see cref="PolicyRunnerNode"/> (deployment), so a policy trained in simulation sees exactly the
/// same features on a real robot: <c>[sector_0 … sector_{n-1}, distance, cos(bearing), sin(bearing)]</c>.
/// Sectors are the minimum LiDAR range in equal angular slices, normalized to [0, 1].
/// </summary>
public static class NavigationObservation
{
    /// <summary>Length of the vector for <paramref name="sectors"/> LiDAR sectors.</summary>
    public static int Size(int sectors) => sectors + 3;

    /// <summary>Fills <paramref name="destination"/>. Allocation-free.</summary>
    /// <param name="destination">Output, at least <see cref="Size"/> long.</param>
    /// <param name="ranges">LiDAR ranges (empty when there is no LiDAR; sectors then read 1).</param>
    /// <param name="angleMin">Angle of the first beam relative to the robot heading.</param>
    /// <param name="angleIncrement">Angle between beams.</param>
    /// <param name="rangeMax">Range used for normalization.</param>
    /// <param name="sectors">Number of sectors spanning the full circle, starting behind the robot.</param>
    /// <param name="pose">Robot pose in the goal's frame.</param>
    /// <param name="goal">Goal position.</param>
    /// <param name="maxGoalDistance">Distance used for normalization.</param>
    public static void Fill(Span<float> destination, ReadOnlySpan<float> ranges, double angleMin, double angleIncrement, double rangeMax,
        int sectors, Pose2D pose, Vec2 goal, double maxGoalDistance)
    {
        for (var s = 0; s < sectors; s++)
        {
            destination[s] = 1f;
        }

        if (sectors > 0 && rangeMax > 0)
        {
            for (var i = 0; i < ranges.Length; i++)
            {
                var r = ranges[i];
                var value = float.IsNaN(r) || float.IsPositiveInfinity(r) ? 1f : float.IsNegativeInfinity(r) ? 0f : (float)Math.Clamp(r / rangeMax, 0, 1);
                var a = Angles.Normalize(angleMin + (i * angleIncrement)) + Math.PI; // [0, 2π)
                var sector = Math.Clamp((int)(a / (2 * Math.PI) * sectors), 0, sectors - 1);
                if (value < destination[sector])
                {
                    destination[sector] = value;
                }
            }
        }

        var toGoal = goal - pose.Position;
        var distance = toGoal.Length;
        var bearing = Angles.Normalize(Math.Atan2(toGoal.Y, toGoal.X) - pose.Theta);
        destination[sectors] = (float)Math.Clamp(distance / Math.Max(1e-6, maxGoalDistance), 0, 1);
        destination[sectors + 1] = (float)Math.Cos(bearing);
        destination[sectors + 2] = (float)Math.Sin(bearing);
    }

    /// <summary>Default discrete action table: forward, forward-left, forward-right, rotate left, rotate right.</summary>
    public static (double Linear, double Angular) DefaultAction(int action, double forwardSpeed, double turnSpeed) => action switch
    {
        0 => (forwardSpeed, 0),
        1 => (forwardSpeed * 0.5, turnSpeed),
        2 => (forwardSpeed * 0.5, -turnSpeed),
        3 => (0, turnSpeed),
        4 => (0, -turnSpeed),
        _ => (0, 0),
    };

    public static IReadOnlyList<string> DefaultActionLabels { get; } = ["forward", "forward_left", "forward_right", "rotate_left", "rotate_right"];
}
