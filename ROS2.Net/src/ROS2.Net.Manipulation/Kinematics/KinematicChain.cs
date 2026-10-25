namespace ROS2.Net.Manipulation;

/// <summary>
/// A serial chain from a base link to a tip link of a <see cref="UrdfModel"/>, like a KDL chain or a
/// MoveIt planning group. Exposes forward kinematics and the geometric Jacobian over its movable joints;
/// mimic joints inside the chain follow their master joint.
/// </summary>
public sealed class KinematicChain
{
    private readonly UrdfJoint[] _segments;
    private readonly int[] _variableIndex;
    private readonly double[] _mimicMultiplier;
    private readonly double[] _mimicOffset;

    public KinematicChain(UrdfModel model, string baseLink, string tipLink)
    {
        Model = model;
        BaseLink = baseLink;
        TipLink = tipLink;

        var path = new List<UrdfJoint>();
        var link = tipLink;
        while (link != baseLink)
        {
            var joint = model.ParentJointOf(link) ?? throw new ArgumentException($"'{tipLink}' is not below '{baseLink}'.");
            path.Add(joint);
            link = joint.Parent;
        }

        path.Reverse();
        _segments = path.ToArray();

        var variables = _segments.Where(j => j.IsMovable && j.Mimic is null).ToList();
        Joints = variables;
        JointNames = variables.Select(j => j.Name).ToArray();
        LowerLimits = variables.Select(j => j.Lower).ToArray();
        UpperLimits = variables.Select(j => j.Upper).ToArray();
        VelocityLimits = variables.Select(j => j.Limits?.Velocity is > 0 ? j.Limits.Velocity : 1.0).ToArray();

        _variableIndex = new int[_segments.Length];
        _mimicMultiplier = new double[_segments.Length];
        _mimicOffset = new double[_segments.Length];
        for (var i = 0; i < _segments.Length; i++)
        {
            var s = _segments[i];
            _mimicMultiplier[i] = 1;
            if (!s.IsMovable)
            {
                _variableIndex[i] = -1;
            }
            else if (s.Mimic is { } m)
            {
                _variableIndex[i] = Array.IndexOf(JointNames, m.Joint);
                _mimicMultiplier[i] = m.Multiplier;
                _mimicOffset[i] = m.Offset;
            }
            else
            {
                _variableIndex[i] = variables.IndexOf(s);
            }
        }
    }

    public UrdfModel Model { get; }

    public string BaseLink { get; }

    public string TipLink { get; }

    /// <summary>Movable, non-mimic joints in base→tip order.</summary>
    public IReadOnlyList<UrdfJoint> Joints { get; }

    public string[] JointNames { get; }

    public int Dof => JointNames.Length;

    public double[] LowerLimits { get; }

    public double[] UpperLimits { get; }

    public double[] VelocityLimits { get; }

    private double ValueOf(int segment, ReadOnlySpan<double> q) =>
        _variableIndex[segment] < 0 ? 0 : (q[_variableIndex[segment]] * _mimicMultiplier[segment]) + _mimicOffset[segment];

    /// <summary>Pose of the tip in the base frame.</summary>
    public Pose3 ForwardKinematics(ReadOnlySpan<double> q)
    {
        if (q.Length != Dof)
        {
            throw new ArgumentException($"Expected {Dof} joint values, got {q.Length}.", nameof(q));
        }

        var pose = Pose3.Identity;
        for (var i = 0; i < _segments.Length; i++)
        {
            pose *= _segments[i].Transform(ValueOf(i, q));
        }

        return pose;
    }

    /// <summary>Pose of every link along the chain (index 0 = first child link).</summary>
    public Pose3[] LinkPoses(ReadOnlySpan<double> q)
    {
        var poses = new Pose3[_segments.Length];
        var pose = Pose3.Identity;
        for (var i = 0; i < _segments.Length; i++)
        {
            pose *= _segments[i].Transform(ValueOf(i, q));
            poses[i] = pose;
        }

        return poses;
    }

    /// <summary>
    /// Geometric Jacobian (6 × Dof, row-major: linear rows then angular rows) of the tip, expressed in the
    /// base frame, written into <paramref name="jacobian"/>.
    /// </summary>
    public Pose3 Jacobian(ReadOnlySpan<double> q, Span<double> jacobian)
    {
        var n = Dof;
        if (jacobian.Length < 6 * n)
        {
            throw new ArgumentException("Jacobian buffer must hold 6 × Dof values.", nameof(jacobian));
        }

        jacobian[..(6 * n)].Clear();
        Span<Vec3> axes = _segments.Length <= 64 ? stackalloc Vec3[_segments.Length] : new Vec3[_segments.Length];
        Span<Vec3> origins = _segments.Length <= 64 ? stackalloc Vec3[_segments.Length] : new Vec3[_segments.Length];

        var pose = Pose3.Identity;
        for (var i = 0; i < _segments.Length; i++)
        {
            var s = _segments[i];

            // The joint axis lives in the frame after the joint origin, before the joint motion.
            var jointFrame = pose * s.Origin;
            axes[i] = jointFrame.Rotation.Rotate(s.Axis.Normalized());
            origins[i] = jointFrame.Position;
            pose *= s.Transform(ValueOf(i, q));
        }

        var tip = pose.Position;
        for (var i = 0; i < _segments.Length; i++)
        {
            var col = _variableIndex[i];
            if (col < 0)
            {
                continue;
            }

            var s = _segments[i];
            var m = _mimicMultiplier[i];
            Vec3 linear, angular;
            if (s.Type == JointType.Prismatic)
            {
                linear = axes[i];
                angular = Vec3.Zero;
            }
            else
            {
                linear = axes[i].Cross(tip - origins[i]);
                angular = axes[i];
            }

            jacobian[col] += linear.X * m;
            jacobian[n + col] += linear.Y * m;
            jacobian[(2 * n) + col] += linear.Z * m;
            jacobian[(3 * n) + col] += angular.X * m;
            jacobian[(4 * n) + col] += angular.Y * m;
            jacobian[(5 * n) + col] += angular.Z * m;
        }

        return pose;
    }

    /// <summary>Clamps positions into the joint limits in place.</summary>
    public void ClampToLimits(Span<double> q)
    {
        for (var i = 0; i < q.Length; i++)
        {
            q[i] = Math.Clamp(q[i], LowerLimits[i], UpperLimits[i]);
        }
    }

    public bool WithinLimits(ReadOnlySpan<double> q)
    {
        for (var i = 0; i < q.Length; i++)
        {
            if (q[i] < LowerLimits[i] - 1e-9 || q[i] > UpperLimits[i] + 1e-9)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>A uniformly random configuration within limits (continuous joints in [-π, π]; very wide limits capped to [-π, π]).</summary>
    public double[] RandomConfiguration(Random random)
    {
        var q = new double[Dof];
        for (var i = 0; i < Dof; i++)
        {
            var lo = Math.Max(LowerLimits[i], -Math.PI);
            var hi = Math.Min(UpperLimits[i], Math.PI);
            q[i] = lo + (random.NextDouble() * (hi - lo));
        }

        return q;
    }
}

/// <summary>Forward kinematics of a whole URDF tree (every link), used for collision checking and robot_state_publisher.</summary>
public sealed class RobotKinematics(UrdfModel model)
{
    public UrdfModel Model { get; } = model;

    /// <summary>World poses (relative to the root link) of every link. Missing joints default to 0; mimic joints follow their master.</summary>
    public Dictionary<string, Pose3> LinkPoses(Func<string, double?> jointValue)
    {
        var result = new Dictionary<string, Pose3>(StringComparer.Ordinal) { [Model.RootLink] = Pose3.Identity };
        foreach (var j in Model.TreeOrder())
        {
            result[j.Child] = result[j.Parent] * j.Transform(JointPosition(j, jointValue));
        }

        return result;
    }

    public Dictionary<string, Pose3> LinkPoses(IReadOnlyDictionary<string, double> values) =>
        LinkPoses(n => values.TryGetValue(n, out var v) ? v : null);

    /// <summary>Position of a joint, resolving mimic joints.</summary>
    public double JointPosition(UrdfJoint joint, Func<string, double?> jointValue)
    {
        if (!joint.IsMovable)
        {
            return 0;
        }

        if (joint.Mimic is { } m)
        {
            return ((jointValue(m.Joint) ?? 0) * m.Multiplier) + m.Offset;
        }

        return jointValue(joint.Name) ?? 0;
    }
}
