using ROS2.Net.Msgs.TrajectoryMsgs;

namespace ROS2.Net.Manipulation;

/// <summary>Velocity profile shape of each segment.</summary>
public enum VelocityProfile
{
    /// <summary>Trapezoidal (bang-coast-bang) velocity, time-optimal under velocity and acceleration limits.</summary>
    Trapezoidal,
    /// <summary>Quintic polynomial (smooth acceleration, zero jerk discontinuity at the ends).</summary>
    Quintic,
}

/// <summary>
/// Turns joint-space waypoints into a timed <c>trajectory_msgs/JointTrajectory</c>. Each segment starts and ends
/// at rest; all joints share one normalized profile per segment, scaled so that no joint exceeds its velocity or
/// acceleration limit (the slowest joint sets the segment duration). Simpler than MoveIt's TOTG, which blends
/// through waypoints without stopping.
/// </summary>
public static class TimeParameterization
{
    /// <param name="jointNames">Joint names of the trajectory.</param>
    /// <param name="waypoints">Joint positions, at least one.</param>
    /// <param name="maxVelocity">Per-joint velocity limits (must be positive).</param>
    /// <param name="maxAcceleration">Per-joint acceleration limits (must be positive).</param>
    /// <param name="profile">Segment profile.</param>
    /// <param name="sampleTime">Spacing of the emitted points (s); 0 emits only the waypoints.</param>
    /// <param name="velocityScaling">Fraction (0, 1] of the velocity limits to use.</param>
    /// <param name="accelerationScaling">Fraction (0, 1] of the acceleration limits to use.</param>
    public static JointTrajectory Compute(
        IReadOnlyList<string> jointNames,
        IReadOnlyList<double[]> waypoints,
        IReadOnlyList<double> maxVelocity,
        IReadOnlyList<double> maxAcceleration,
        VelocityProfile profile = VelocityProfile.Trapezoidal,
        double sampleTime = 0.02,
        double velocityScaling = 1.0,
        double accelerationScaling = 1.0)
    {
        if (waypoints.Count == 0)
        {
            throw new ArgumentException("At least one waypoint is required.", nameof(waypoints));
        }

        var n = jointNames.Count;
        var vmax = Enumerable.Range(0, n).Select(i => maxVelocity[i] * Math.Clamp(velocityScaling, 1e-6, 1)).ToArray();
        var amax = Enumerable.Range(0, n).Select(i => maxAcceleration[i] * Math.Clamp(accelerationScaling, 1e-6, 1)).ToArray();
        if (vmax.Any(v => v <= 0) || amax.Any(a => a <= 0))
        {
            throw new ArgumentException("Velocity and acceleration limits must be positive.");
        }

        var traj = new JointTrajectory { JointNames = jointNames.ToArray() };
        var points = new List<JointTrajectoryPoint> { Point(waypoints[0], new double[n], new double[n], 0) };
        var t0 = 0.0;
        for (var s = 1; s < waypoints.Count; s++)
        {
            var from = waypoints[s - 1];
            var to = waypoints[s];
            var d = Enumerable.Range(0, n).Select(i => to[i] - from[i]).ToArray();
            var (duration, accelTime) = SegmentTiming(d, vmax, amax, profile);
            if (duration <= 0)
            {
                continue;
            }

            var steps = sampleTime > 0 ? Math.Max(1, (int)Math.Ceiling(duration / sampleTime)) : 1;
            for (var k = 1; k <= steps; k++)
            {
                var t = duration * k / steps;
                var (sp, sv, sa) = profile == VelocityProfile.Quintic ? QuinticS(t, duration) : TrapezoidS(t, duration, accelTime);
                var pos = new double[n];
                var vel = new double[n];
                var acc = new double[n];
                for (var i = 0; i < n; i++)
                {
                    pos[i] = from[i] + (d[i] * sp);
                    vel[i] = d[i] * sv;
                    acc[i] = d[i] * sa;
                }

                points.Add(Point(pos, vel, acc, t0 + t));
            }

            t0 += duration;
        }

        traj.Points = points.ToArray();
        return traj;
    }

    /// <summary>Total duration of a parameterized trajectory.</summary>
    public static double Duration(JointTrajectory trajectory) =>
        trajectory.Points.Length == 0 ? 0 : RosDuration.FromMsg(trajectory.Points[^1].TimeFromStart).TotalSeconds;

    private static (double Duration, double AccelTime) SegmentTiming(double[] d, double[] vmax, double[] amax, VelocityProfile profile)
    {
        if (profile == VelocityProfile.Quintic)
        {
            // s' peaks at 1.875/T, s'' at 5.7735/T² (10/√3).
            var T = 0.0;
            for (var i = 0; i < d.Length; i++)
            {
                var a = Math.Abs(d[i]);
                T = Math.Max(T, Math.Max(1.875 * a / vmax[i], Math.Sqrt(5.773502691896258 * a / amax[i])));
            }

            return (T, 0);
        }

        // Time-optimal trapezoid of the limiting joint, then scale time until every joint fits.
        double best = 0, ta = 0;
        for (var i = 0; i < d.Length; i++)
        {
            var a = Math.Abs(d[i]);
            if (a < 1e-12)
            {
                continue;
            }

            double Ti, tai;
            if (a >= vmax[i] * vmax[i] / amax[i])
            {
                tai = vmax[i] / amax[i];
                Ti = (a / vmax[i]) + tai;
            }
            else
            {
                tai = Math.Sqrt(a / amax[i]);
                Ti = 2 * tai;
            }

            if (Ti > best)
            {
                best = Ti;
                ta = tai;
            }
        }

        if (best <= 0)
        {
            return (0, 0);
        }

        // With the shared shape, joint i needs |d|/(T−ta) ≤ v and |d|/((T−ta)·ta) ≤ a. Scaling time by k
        // divides velocity by k and acceleration by k².
        var k = 1.0;
        for (var i = 0; i < d.Length; i++)
        {
            var a = Math.Abs(d[i]);
            var v = a / (best - ta);
            var acc = v / ta;
            k = Math.Max(k, Math.Max(v / vmax[i], Math.Sqrt(acc / amax[i])));
        }

        return (best * k, ta * k);
    }

    // Normalized trapezoid: s(0)=0, s(T)=1, accelerate for ta, coast, decelerate for ta.
    private static (double S, double Sv, double Sa) TrapezoidS(double t, double T, double ta)
    {
        var vPeak = 1.0 / (T - ta);
        var a = vPeak / ta;
        if (t <= ta)
        {
            return (0.5 * a * t * t, a * t, a);
        }

        if (t <= T - ta)
        {
            return ((0.5 * a * ta * ta) + (vPeak * (t - ta)), vPeak, 0);
        }

        var r = T - t;
        return (1 - (0.5 * a * r * r), a * r, -a);
    }

    private static (double S, double Sv, double Sa) QuinticS(double t, double T)
    {
        var x = Math.Clamp(t / T, 0, 1);
        var x2 = x * x;
        var x3 = x2 * x;
        return (
            (10 * x3) - (15 * x3 * x) + (6 * x3 * x2),
            ((30 * x2) - (60 * x3) + (30 * x2 * x2)) / T,
            ((60 * x) - (180 * x2) + (120 * x3)) / (T * T));
    }

    private static JointTrajectoryPoint Point(double[] p, double[] v, double[] a, double t) => new()
    {
        Positions = (double[])p.Clone(),
        Velocities = v,
        Accelerations = a,
        TimeFromStart = RosDuration.FromSeconds(t).ToMsg(),
    };
}
