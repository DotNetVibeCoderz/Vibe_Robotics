using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Navigation.Costmaps;

namespace ROS2.Net.Navigation.Control;

/// <summary>A planar velocity command: forward speed (m/s) and yaw rate (rad/s).</summary>
public readonly record struct Velocity2D(double Linear, double Angular)
{
    public static Velocity2D Zero => default;

    public Twist ToTwist() => new() { Linear = new Vector3 { X = Linear }, Angular = new Vector3 { Z = Angular } };

    public static Velocity2D FromTwist(Twist t) => new(t.Linear.X, t.Angular.Z);
}

/// <summary>A local trajectory controller (Nav2 controller plugin equivalent).</summary>
public interface IController
{
    string Name { get; }

    /// <summary>Sets the global path to follow.</summary>
    void SetPlan(IReadOnlyList<Pose2D> path);

    /// <summary>Computes the next command for the robot at <paramref name="pose"/> moving at <paramref name="current"/>.</summary>
    Velocity2D ComputeVelocity(Pose2D pose, Velocity2D current, Costmap2D? costmap);
}

/// <summary>Decides when the goal is reached (Nav2 <c>SimpleGoalChecker</c>).</summary>
public sealed class GoalChecker(double xyTolerance = 0.25, double yawTolerance = 0.25, bool stateful = true)
{
    private bool _xyReached;

    public double XyTolerance { get; set; } = xyTolerance;

    public double YawTolerance { get; set; } = yawTolerance;

    public void Reset() => _xyReached = false;

    public bool IsGoalReached(Pose2D pose, Pose2D goal)
    {
        var xyOk = _xyReached || pose.DistanceSquaredTo(goal) <= XyTolerance * XyTolerance;
        if (stateful && xyOk)
        {
            _xyReached = true;
        }

        return xyOk && Math.Abs(Angles.Difference(pose.Theta, goal.Theta)) <= YawTolerance;
    }
}

/// <summary>Fails navigation when the robot does not move <see cref="RequiredMovementRadius"/> within <see cref="MovementTimeAllowance"/> (Nav2 <c>SimpleProgressChecker</c>).</summary>
public sealed class ProgressChecker(double requiredMovementRadius = 0.5, double movementTimeAllowanceSeconds = 10)
{
    private Pose2D _baseline;
    private double _baselineTime = double.NaN;

    public double RequiredMovementRadius { get; set; } = requiredMovementRadius;

    public TimeSpan MovementTimeAllowance { get; set; } = TimeSpan.FromSeconds(movementTimeAllowanceSeconds);

    public void Reset() => _baselineTime = double.NaN;

    /// <summary>Returns false when the robot is stuck.</summary>
    public bool Check(Pose2D pose, double nowSeconds)
    {
        if (double.IsNaN(_baselineTime) || pose.DistanceTo(_baseline) > RequiredMovementRadius)
        {
            _baseline = pose;
            _baselineTime = nowSeconds;
            return true;
        }

        return nowSeconds - _baselineTime <= MovementTimeAllowance.TotalSeconds;
    }
}

/// <summary>Common path bookkeeping for path-tracking controllers.</summary>
public abstract class PathTrackingController : IController
{
    protected List<Pose2D> Path { get; private set; } = [];

    protected int ClosestIndex { get; set; }

    public abstract string Name { get; }

    public double MaxLinearSpeed { get; set; } = 0.5;

    public double MaxAngularSpeed { get; set; } = 1.5;

    /// <summary>Within this distance of the end the controller turns in place to the final heading.</summary>
    public double GoalXyTolerance { get; set; } = 0.2;

    public virtual void SetPlan(IReadOnlyList<Pose2D> path)
    {
        Path = path.ToList();
        ClosestIndex = 0;
    }

    public abstract Velocity2D ComputeVelocity(Pose2D pose, Velocity2D current, Costmap2D? costmap);

    /// <summary>Advances <see cref="ClosestIndex"/> to the nearest path pose (monotonic, searching a window ahead).</summary>
    protected void UpdateClosest(Pose2D pose, int window = 50)
    {
        var best = double.MaxValue;
        var end = Math.Min(Path.Count, ClosestIndex + window);
        var bestIndex = ClosestIndex;
        for (var i = ClosestIndex; i < end; i++)
        {
            var d = pose.DistanceSquaredTo(Path[i]);
            if (d < best)
            {
                best = d;
                bestIndex = i;
            }
        }

        ClosestIndex = bestIndex;
    }

    /// <summary>First path pose at least <paramref name="distance"/> from the robot, else the last pose.</summary>
    protected Pose2D LookaheadPoint(Pose2D pose, double distance)
    {
        var d2 = distance * distance;
        for (var i = ClosestIndex; i < Path.Count; i++)
        {
            if (pose.DistanceSquaredTo(Path[i]) >= d2)
            {
                return Path[i];
            }
        }

        return Path[^1];
    }

    protected double RemainingDistance(Pose2D pose)
    {
        if (Path.Count == 0)
        {
            return 0;
        }

        var d = pose.DistanceTo(Path[ClosestIndex]);
        for (var i = ClosestIndex + 1; i < Path.Count; i++)
        {
            d += Path[i - 1].DistanceTo(Path[i]);
        }

        return d;
    }

    protected Velocity2D RotateTo(double headingError) =>
        new(0, Math.Clamp(2.0 * headingError, -MaxAngularSpeed, MaxAngularSpeed));
}

/// <summary>
/// Pure Pursuit: steers along the arc through a lookahead point on the path
/// (curvature κ = 2·y / L²). Rotates in place when the path is behind the robot.
/// </summary>
public class PurePursuitController : PathTrackingController
{
    public override string Name => "PurePursuit";

    public double LookaheadDistance { get; set; } = 0.6;

    /// <summary>Rotate in place first when the lookahead point is more than this angle off the heading.</summary>
    public double RotateToHeadingAngle { get; set; } = 0.785;

    public bool UseRotateToHeading { get; set; } = true;

    public override Velocity2D ComputeVelocity(Pose2D pose, Velocity2D current, Costmap2D? costmap)
    {
        if (Path.Count == 0)
        {
            return Velocity2D.Zero;
        }

        UpdateClosest(pose);
        var goal = Path[^1];
        if (pose.DistanceTo(goal) <= GoalXyTolerance)
        {
            return RotateTo(Angles.Difference(goal.Theta, pose.Theta));
        }

        var lookahead = ComputeLookaheadDistance(current);
        var target = LookaheadPoint(pose, lookahead);
        var local = pose.Between(target);
        var angleToTarget = Math.Atan2(local.Y, local.X);
        if (UseRotateToHeading && Math.Abs(angleToTarget) > RotateToHeadingAngle)
        {
            return RotateTo(angleToTarget);
        }

        var l2 = (local.X * local.X) + (local.Y * local.Y);
        var curvature = l2 > 1e-9 ? 2.0 * local.Y / l2 : 0.0;
        var v = RegulateSpeed(MaxLinearSpeed, curvature, pose, costmap);
        var w = Math.Clamp(v * curvature, -MaxAngularSpeed, MaxAngularSpeed);

        // Keep the arc when the angular limit saturates.
        if (Math.Abs(curvature) > 1e-9 && Math.Abs(v * curvature) > MaxAngularSpeed)
        {
            v = MaxAngularSpeed / Math.Abs(curvature);
        }

        return new Velocity2D(v, w);
    }

    protected virtual double ComputeLookaheadDistance(Velocity2D current) => LookaheadDistance;

    protected virtual double RegulateSpeed(double desired, double curvature, Pose2D pose, Costmap2D? costmap) => desired;
}

/// <summary>
/// Regulated Pure Pursuit (Macenski et al., 2023): adds velocity-scaled lookahead and slows down on
/// tight curvature, near obstacles and when approaching the goal.
/// </summary>
public sealed class RegulatedPurePursuitController : PurePursuitController
{
    public override string Name => "RegulatedPurePursuit";

    public bool UseVelocityScaledLookahead { get; set; } = true;

    public double LookaheadTime { get; set; } = 1.5;

    public double MinLookahead { get; set; } = 0.3;

    public double MaxLookahead { get; set; } = 0.9;

    /// <summary>Below this turning radius speed scales down proportionally.</summary>
    public double RegulatedMinRadius { get; set; } = 0.9;

    public bool UseCostRegulatedScaling { get; set; } = true;

    /// <summary>Gain applied to the normalized inflation cost under the robot.</summary>
    public double CostScalingGain { get; set; } = 1.0;

    public double ApproachVelocityScalingDistance { get; set; } = 0.6;

    public double MinApproachLinearVelocity { get; set; } = 0.05;

    public double RegulatedMinSpeed { get; set; } = 0.1;

    private double _remaining;

    public override Velocity2D ComputeVelocity(Pose2D pose, Velocity2D current, Costmap2D? costmap)
    {
        if (Path.Count > 0)
        {
            UpdateClosest(pose);
            _remaining = RemainingDistance(pose);
        }

        return base.ComputeVelocity(pose, current, costmap);
    }

    protected override double ComputeLookaheadDistance(Velocity2D current) =>
        UseVelocityScaledLookahead ? Math.Clamp(Math.Abs(current.Linear) * LookaheadTime, MinLookahead, MaxLookahead) : LookaheadDistance;

    protected override double RegulateSpeed(double desired, double curvature, Pose2D pose, Costmap2D? costmap)
    {
        var v = desired;
        var radius = Math.Abs(curvature) > 1e-9 ? 1.0 / Math.Abs(curvature) : double.MaxValue;
        if (radius < RegulatedMinRadius)
        {
            v = Math.Max(desired * (radius / RegulatedMinRadius), RegulatedMinSpeed);
        }

        if (UseCostRegulatedScaling && costmap is not null)
        {
            var cost = costmap.GetCostAt(pose.X, pose.Y);
            if (cost is > Costs.Free and < Costs.Inscribed)
            {
                var scale = 1.0 - (CostScalingGain * cost / Costs.MaxNonObstacle);
                v = Math.Max(Math.Min(v, desired * scale), RegulatedMinSpeed);
            }
        }

        if (_remaining < ApproachVelocityScalingDistance)
        {
            v = Math.Max(Math.Min(v, desired * (_remaining / ApproachVelocityScalingDistance)), MinApproachLinearVelocity);
        }

        return v;
    }
}

/// <summary>Weights of the DWB critics.</summary>
public sealed record DwbCritics
{
    public double PathDistance { get; init; } = 32;

    public double GoalDistance { get; init; } = 24;

    public double PathAlign { get; init; } = 12;

    public double Obstacle { get; init; } = 0.02;

    public double PreferForward { get; init; } = 2;
}

/// <summary>
/// A DWB / DWA-style controller: samples velocities in the dynamic window, forward-simulates each
/// over <see cref="SimTime"/>, scores the trajectories with critics and picks the cheapest.
/// </summary>
public sealed class DwbController : PathTrackingController
{
    public override string Name => "DWB";

    public double MinLinearSpeed { get; set; }

    public double LinearAcceleration { get; set; } = 2.5;

    public double AngularAcceleration { get; set; } = 3.2;

    public double SimTime { get; set; } = 1.5;

    public double SimStep { get; set; } = 0.1;

    public double ControlPeriod { get; set; } = 0.05;

    public int LinearSamples { get; set; } = 12;

    public int AngularSamples { get; set; } = 21;

    public double CarrotDistance { get; set; } = 1.0;

    public DwbCritics Critics { get; set; } = new();

    public override Velocity2D ComputeVelocity(Pose2D pose, Velocity2D current, Costmap2D? costmap)
    {
        if (Path.Count == 0)
        {
            return Velocity2D.Zero;
        }

        UpdateClosest(pose);
        var goal = Path[^1];
        if (pose.DistanceTo(goal) <= GoalXyTolerance)
        {
            return RotateTo(Angles.Difference(goal.Theta, pose.Theta));
        }

        var carrot = LookaheadPoint(pose, CarrotDistance);
        var vLo = Math.Max(MinLinearSpeed, current.Linear - (LinearAcceleration * ControlPeriod * 4));
        var vHi = Math.Min(MaxLinearSpeed, current.Linear + (LinearAcceleration * ControlPeriod * 4));
        var wLo = Math.Max(-MaxAngularSpeed, current.Angular - (AngularAcceleration * ControlPeriod * 4));
        var wHi = Math.Min(MaxAngularSpeed, current.Angular + (AngularAcceleration * ControlPeriod * 4));
        var best = double.MaxValue;
        var bestCmd = Velocity2D.Zero;
        var found = false;
        for (var i = 0; i < LinearSamples; i++)
        {
            var v = LinearSamples == 1 ? vHi : vLo + ((vHi - vLo) * i / (LinearSamples - 1));
            for (var j = 0; j < AngularSamples; j++)
            {
                var w = AngularSamples == 1 ? 0 : wLo + ((wHi - wLo) * j / (AngularSamples - 1));
                var score = Score(pose, v, w, carrot, costmap);
                if (score < best)
                {
                    best = score;
                    bestCmd = new Velocity2D(v, w);
                    found = true;
                }
            }
        }

        if (!found)
        {
            // Every trajectory collides: turn toward the carrot in place.
            var local = pose.Between(carrot);
            return RotateTo(Math.Atan2(local.Y, local.X));
        }

        return bestCmd;
    }

    private double Score(Pose2D start, double v, double w, Pose2D carrot, Costmap2D? costmap)
    {
        var p = start;
        var obstacle = 0.0;
        for (var t = 0.0; t < SimTime; t += SimStep)
        {
            p = new Pose2D(p.X + (v * Math.Cos(p.Theta) * SimStep), p.Y + (v * Math.Sin(p.Theta) * SimStep), Angles.Normalize(p.Theta + (w * SimStep)));
            if (costmap is not null)
            {
                var c = costmap.GetCostAt(p.X, p.Y);
                if (c >= Costs.Inscribed && c != Costs.NoInformation)
                {
                    return double.MaxValue;
                }

                obstacle += c == Costs.NoInformation ? 0 : c;
            }
        }

        // Path distance: from the trajectory end to the closest path pose.
        var pathDist = double.MaxValue;
        var nearest = Path[ClosestIndex];
        var end = Math.Min(Path.Count, ClosestIndex + 100);
        for (var i = ClosestIndex; i < end; i++)
        {
            var d = p.DistanceSquaredTo(Path[i]);
            if (d < pathDist)
            {
                pathDist = d;
                nearest = Path[i];
            }
        }

        var goalDist = p.DistanceTo(carrot);
        var align = Math.Abs(Angles.Difference(p.Theta, nearest.Theta));
        return (Critics.PathDistance * Math.Sqrt(pathDist)) + (Critics.GoalDistance * goalDist) + (Critics.PathAlign * align * 0.1) +
               (Critics.Obstacle * obstacle) + (Critics.PreferForward * (MaxLinearSpeed - v));
    }
}
