using ROS2.Net.Navigation.Control;
using ROS2.Net.Navigation.Costmaps;

namespace ROS2.Net.Navigation.Behaviors;

/// <summary>Status of a running behavior.</summary>
public enum BehaviorStatus
{
    Running,
    Succeeded,
    Failed,
}

/// <summary>A recovery behavior (Nav2 behavior server plugin equivalent) driven by periodic updates.</summary>
public interface IRecoveryBehavior
{
    string Name { get; }

    void Start(Pose2D pose);

    /// <summary>Advances the behavior by <paramref name="dt"/> seconds and returns the command to send.</summary>
    BehaviorStatus Update(Pose2D pose, double dt, out Velocity2D command);
}

/// <summary>Rotates in place by a relative angle.</summary>
public sealed class SpinBehavior(double targetYaw = Math.PI / 2, double angularSpeed = 1.0, double timeout = 15) : IRecoveryBehavior
{
    private double _turned;
    private double _lastYaw;
    private double _elapsed;

    public string Name => "Spin";

    public double TargetYaw { get; set; } = targetYaw;

    /// <summary>Optional collision check of the rotated pose.</summary>
    public Func<Pose2D, bool>? IsColliding { get; set; }

    public void Start(Pose2D pose)
    {
        _turned = 0;
        _lastYaw = pose.Theta;
        _elapsed = 0;
    }

    public BehaviorStatus Update(Pose2D pose, double dt, out Velocity2D command)
    {
        _turned += Angles.Difference(pose.Theta, _lastYaw);
        _lastYaw = pose.Theta;
        _elapsed += dt;
        var remaining = TargetYaw - _turned;
        if (Math.Abs(remaining) < 0.05)
        {
            command = Velocity2D.Zero;
            return BehaviorStatus.Succeeded;
        }

        if (_elapsed > timeout || (IsColliding?.Invoke(pose with { Theta = pose.Theta + (Math.Sign(remaining) * 0.1) }) ?? false))
        {
            command = Velocity2D.Zero;
            return BehaviorStatus.Failed;
        }

        var w = Math.Sign(remaining) * Math.Min(angularSpeed, Math.Max(0.2, Math.Abs(remaining) * 2));
        command = new Velocity2D(0, w);
        return BehaviorStatus.Running;
    }
}

/// <summary>Drives straight backwards a fixed distance.</summary>
public sealed class BackUpBehavior(double distance = 0.3, double speed = 0.1, double timeout = 10) : IRecoveryBehavior
{
    private Pose2D _start;
    private double _elapsed;

    public string Name => "BackUp";

    public Func<Pose2D, bool>? IsColliding { get; set; }

    public void Start(Pose2D pose)
    {
        _start = pose;
        _elapsed = 0;
    }

    public BehaviorStatus Update(Pose2D pose, double dt, out Velocity2D command)
    {
        _elapsed += dt;
        if (pose.DistanceTo(_start) >= distance)
        {
            command = Velocity2D.Zero;
            return BehaviorStatus.Succeeded;
        }

        var next = new Pose2D(pose.X - (0.1 * Math.Cos(pose.Theta)), pose.Y - (0.1 * Math.Sin(pose.Theta)), pose.Theta);
        if (_elapsed > timeout || (IsColliding?.Invoke(next) ?? false))
        {
            command = Velocity2D.Zero;
            return BehaviorStatus.Failed;
        }

        command = new Velocity2D(-Math.Abs(speed), 0);
        return BehaviorStatus.Running;
    }
}

/// <summary>Stays still for a duration.</summary>
public sealed class WaitBehavior(double seconds = 5) : IRecoveryBehavior
{
    private double _elapsed;

    public string Name => "Wait";

    public void Start(Pose2D pose) => _elapsed = 0;

    public BehaviorStatus Update(Pose2D pose, double dt, out Velocity2D command)
    {
        command = Velocity2D.Zero;
        _elapsed += dt;
        return _elapsed >= seconds ? BehaviorStatus.Succeeded : BehaviorStatus.Running;
    }
}

/// <summary>Clears the dynamic layers of one or more costmaps (Nav2's <c>ClearEntireCostmap</c>).</summary>
public sealed class ClearCostmapBehavior(params Costmap2D[] costmaps) : IRecoveryBehavior
{
    public string Name => "ClearCostmap";

    public void Start(Pose2D pose)
    {
        foreach (var c in costmaps)
        {
            c.ClearDynamic();
        }
    }

    public BehaviorStatus Update(Pose2D pose, double dt, out Velocity2D command)
    {
        command = Velocity2D.Zero;
        return BehaviorStatus.Succeeded;
    }
}
