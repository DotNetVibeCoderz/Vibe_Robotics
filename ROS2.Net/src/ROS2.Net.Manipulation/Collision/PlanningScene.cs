namespace ROS2.Net.Manipulation;

/// <summary>An object in the world: primitives in the object's frame at a pose.</summary>
public sealed record WorldObject(string Id, Pose3 Pose, IReadOnlyList<CollisionShape> Shapes);

/// <summary>An object attached to a robot link (it moves with the link). Collisions with <see cref="TouchLinks"/> are allowed.</summary>
public sealed record AttachedObject(string Id, string Link, Pose3 PoseInLink, IReadOnlyList<CollisionShape> Shapes, IReadOnlyCollection<string> TouchLinks);

/// <summary>
/// The robot and its environment for collision checking, like MoveIt's planning scene: world objects,
/// attached objects and an allowed collision matrix. Adjacent links (joined by a joint) are allowed to
/// touch by default.
/// </summary>
public sealed class PlanningScene
{
    private readonly Dictionary<string, List<CollisionShape>> _linkShapes = new(StringComparer.Ordinal);
    private readonly Dictionary<string, WorldObject> _world = new(StringComparer.Ordinal);
    private readonly Dictionary<string, AttachedObject> _attached = new(StringComparer.Ordinal);
    private readonly HashSet<(string, string)> _allowed = [];
    private readonly RobotKinematics _kinematics;

    public PlanningScene(UrdfModel model)
    {
        Model = model;
        _kinematics = new RobotKinematics(model);
        foreach (var link in model.Links)
        {
            var shapes = link.Collisions.Select(Geometry3.FromGeometry).OfType<CollisionShape>().ToList();
            if (shapes.Count > 0)
            {
                _linkShapes[link.Name] = shapes;
            }
        }

        foreach (var j in model.Joints)
        {
            Allow(j.Parent, j.Child);
        }

        // Links connected through fixed joints only (e.g. base_footprint–base_link–caster) are rigidly adjacent too.
        foreach (var j in model.Joints.Where(j => j.Type == JointType.Fixed))
        {
            foreach (var k in model.Joints.Where(k => k.Parent == j.Child || k.Child == j.Parent))
            {
                Allow(j.Parent, k.Child);
                Allow(j.Child, k.Parent);
            }
        }
    }

    public UrdfModel Model { get; }

    /// <summary>Positions used for joints that a query does not specify.</summary>
    public Dictionary<string, double> DefaultJointValues { get; } = new(StringComparer.Ordinal);

    /// <summary>Extra clearance (m) required between bodies.</summary>
    public double Padding { get; set; }

    public IReadOnlyCollection<WorldObject> WorldObjects => _world.Values;

    public IReadOnlyCollection<AttachedObject> AttachedObjects => _attached.Values;

    /// <summary>Allows two bodies (links or object ids) to touch.</summary>
    public void Allow(string a, string b) => _allowed.Add(Key(a, b));

    public void Disallow(string a, string b) => _allowed.Remove(Key(a, b));

    public bool IsAllowed(string a, string b) => _allowed.Contains(Key(a, b));

    /// <summary>Allows every link pair that collides at <paramref name="state"/> (like MoveIt's "default" disabled pairs).</summary>
    public int AllowCollisionsAt(IReadOnlyDictionary<string, double> state)
    {
        var added = 0;
        foreach (var (a, b) in SelfCollisions(state))
        {
            Allow(a, b);
            added++;
        }

        return added;
    }

    public void AddWorldObject(WorldObject obj) => _world[obj.Id] = obj;

    /// <summary>Adds an axis-aligned box obstacle centered at <paramref name="center"/>.</summary>
    public void AddBox(string id, Vec3 center, Vec3 size, Quat? rotation = null) =>
        AddWorldObject(new WorldObject(id, new Pose3(center, rotation ?? Quat.Identity), [new BoxShape(Pose3.Identity, size / 2)]));

    public void AddSphere(string id, Vec3 center, double radius) =>
        AddWorldObject(new WorldObject(id, new Pose3(center, Quat.Identity), [new SphereShape(Vec3.Zero, radius)]));

    public bool RemoveWorldObject(string id) => _world.Remove(id);

    /// <summary>Attaches a world object (or new shapes) to a link, e.g. after grasping.</summary>
    public void Attach(string objectId, string link, IReadOnlyDictionary<string, double> state, IEnumerable<string>? touchLinks = null)
    {
        if (!_world.Remove(objectId, out var obj))
        {
            throw new KeyNotFoundException($"No world object '{objectId}'.");
        }

        var linkPose = _kinematics.LinkPoses(state)[link];
        var inLink = linkPose.Inverse() * obj.Pose;
        _attached[objectId] = new AttachedObject(objectId, link, inLink, obj.Shapes, (touchLinks ?? [link]).ToHashSet(StringComparer.Ordinal));
    }

    /// <summary>Detaches an object, leaving it in the world where it is.</summary>
    public void Detach(string objectId, IReadOnlyDictionary<string, double> state)
    {
        if (!_attached.Remove(objectId, out var att))
        {
            return;
        }

        var linkPose = _kinematics.LinkPoses(state)[att.Link];
        _world[objectId] = new WorldObject(objectId, linkPose * att.PoseInLink, att.Shapes);
    }

    /// <summary>True when the state is collision-free.</summary>
    public bool IsStateValid(IReadOnlyDictionary<string, double> state) => FirstCollision(state) is null;

    /// <summary>True when the chain configuration is collision-free (other joints use <see cref="DefaultJointValues"/>).</summary>
    public bool IsStateValid(KinematicChain chain, ReadOnlySpan<double> q) => IsStateValid(StateFor(chain, q));

    /// <summary>Merges a chain configuration into the default state.</summary>
    public Dictionary<string, double> StateFor(KinematicChain chain, ReadOnlySpan<double> q)
    {
        var state = new Dictionary<string, double>(DefaultJointValues, StringComparer.Ordinal);
        for (var i = 0; i < chain.Dof; i++)
        {
            state[chain.JointNames[i]] = q[i];
        }

        return state;
    }

    /// <summary>The first colliding pair, or null.</summary>
    public (string A, string B)? FirstCollision(IReadOnlyDictionary<string, double> state)
    {
        var bodies = Bodies(state);
        for (var i = 0; i < bodies.Count; i++)
        {
            for (var j = i + 1; j < bodies.Count; j++)
            {
                var (na, sa, ta) = bodies[i];
                var (nb, sb, tb) = bodies[j];
                if (na == nb || IsAllowed(na, nb) || ta.Contains(nb) || tb.Contains(na) || (_world.ContainsKey(na) && _world.ContainsKey(nb)))
                {
                    continue;
                }

                foreach (var x in sa)
                {
                    foreach (var y in sb)
                    {
                        if (x.Intersects(y, Padding))
                        {
                            return (na, nb);
                        }
                    }
                }
            }
        }

        return null;
    }

    private IEnumerable<(string, string)> SelfCollisions(IReadOnlyDictionary<string, double> state)
    {
        var poses = _kinematics.LinkPoses(state);
        var links = _linkShapes.Select(kv => (kv.Key, kv.Value.Select(s => s.Transformed(poses[kv.Key])).ToList())).ToList();
        for (var i = 0; i < links.Count; i++)
        {
            for (var j = i + 1; j < links.Count; j++)
            {
                if (!IsAllowed(links[i].Key, links[j].Key) && links[i].Item2.Any(a => links[j].Item2.Any(b => a.Intersects(b))))
                {
                    yield return (links[i].Key, links[j].Key);
                }
            }
        }
    }

    private List<(string Name, List<CollisionShape> Shapes, IReadOnlyCollection<string> Touch)> Bodies(IReadOnlyDictionary<string, double> state)
    {
        var poses = _kinematics.LinkPoses(state);
        var bodies = new List<(string, List<CollisionShape>, IReadOnlyCollection<string>)>();
        foreach (var (link, shapes) in _linkShapes)
        {
            bodies.Add((link, shapes.Select(s => s.Transformed(poses[link])).ToList(), Array.Empty<string>()));
        }

        foreach (var att in _attached.Values)
        {
            var pose = poses[att.Link] * att.PoseInLink;
            bodies.Add((att.Id, att.Shapes.Select(s => s.Transformed(pose)).ToList(), att.TouchLinks));
        }

        foreach (var obj in _world.Values)
        {
            bodies.Add((obj.Id, obj.Shapes.Select(s => s.Transformed(obj.Pose)).ToList(), Array.Empty<string>()));
        }

        return bodies;
    }

    private static (string, string) Key(string a, string b) => string.CompareOrdinal(a, b) <= 0 ? (a, b) : (b, a);
}
