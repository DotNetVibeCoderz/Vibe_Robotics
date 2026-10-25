using System.Globalization;
using System.Reflection;
using System.Xml.Linq;

namespace ROS2.Net.Manipulation;

/// <summary>URDF joint types.</summary>
public enum JointType
{
    Fixed,
    Revolute,
    Continuous,
    Prismatic,
    Floating,
    Planar,
}

/// <summary>Collision/visual geometry.</summary>
public abstract record Geometry;

public sealed record BoxGeometry(Vec3 Size) : Geometry;

public sealed record CylinderGeometry(double Radius, double Length) : Geometry;

public sealed record SphereGeometry(double Radius) : Geometry;

public sealed record MeshGeometry(string Filename, Vec3 Scale) : Geometry;

/// <summary>A visual or collision element: geometry at an origin in the link frame.</summary>
public sealed record GeometryElement(Pose3 Origin, Geometry Geometry, string? Name = null, string? Material = null);

public sealed record Inertial(Pose3 Origin, double Mass, double Ixx, double Ixy, double Ixz, double Iyy, double Iyz, double Izz);

public sealed record JointLimits(double Lower, double Upper, double Effort, double Velocity);

public sealed record JointMimic(string Joint, double Multiplier, double Offset);

/// <summary>A URDF link.</summary>
public sealed class UrdfLink
{
    public required string Name { get; init; }

    public Inertial? Inertial { get; init; }

    public List<GeometryElement> Visuals { get; } = [];

    public List<GeometryElement> Collisions { get; } = [];
}

/// <summary>A URDF joint.</summary>
public sealed class UrdfJoint
{
    public required string Name { get; init; }

    public required JointType Type { get; init; }

    public required string Parent { get; init; }

    public required string Child { get; init; }

    public Pose3 Origin { get; init; } = Pose3.Identity;

    public Vec3 Axis { get; init; } = Vec3.UnitX;

    public JointLimits? Limits { get; init; }

    public JointMimic? Mimic { get; init; }

    /// <summary>True for joints with one degree of freedom that a chain can actuate.</summary>
    public bool IsMovable => Type is JointType.Revolute or JointType.Continuous or JointType.Prismatic;

    /// <summary>Lower position bound (−∞ for continuous joints).</summary>
    public double Lower => Type == JointType.Continuous ? double.NegativeInfinity : Limits?.Lower ?? double.NegativeInfinity;

    public double Upper => Type == JointType.Continuous ? double.PositiveInfinity : Limits?.Upper ?? double.PositiveInfinity;

    /// <summary>The transform from the parent link frame to the child link frame at <paramref name="position"/>.</summary>
    public Pose3 Transform(double position) => Type switch
    {
        JointType.Revolute or JointType.Continuous => Origin * new Pose3(Vec3.Zero, Quat.FromAxisAngle(Axis, position)),
        JointType.Prismatic => Origin * new Pose3(Axis.Normalized() * position, Quat.Identity),
        _ => Origin,
    };
}

/// <summary>Thrown for malformed URDF.</summary>
public sealed class UrdfException(string message) : FormatException(message);

/// <summary>A parsed URDF robot model.</summary>
public sealed class UrdfModel
{
    private readonly Dictionary<string, UrdfLink> _links = new(StringComparer.Ordinal);
    private readonly Dictionary<string, UrdfJoint> _joints = new(StringComparer.Ordinal);
    private readonly Dictionary<string, UrdfJoint> _parentJoint = new(StringComparer.Ordinal);

    private UrdfModel(string name, string xml)
    {
        Name = name;
        Xml = xml;
    }

    public string Name { get; }

    /// <summary>The source XML (what <c>robot_description</c> carries).</summary>
    public string Xml { get; }

    public IReadOnlyCollection<UrdfLink> Links => _links.Values;

    public IReadOnlyCollection<UrdfJoint> Joints => _joints.Values;

    /// <summary>The link without a parent joint.</summary>
    public string RootLink { get; private set; } = string.Empty;

    public UrdfLink GetLink(string name) => _links.TryGetValue(name, out var l) ? l : throw new KeyNotFoundException($"No link '{name}'.");

    public UrdfJoint GetJoint(string name) => _joints.TryGetValue(name, out var j) ? j : throw new KeyNotFoundException($"No joint '{name}'.");

    public bool TryGetJoint(string name, out UrdfJoint joint) => _joints.TryGetValue(name, out joint!);

    /// <summary>The joint whose child is <paramref name="link"/>, or null for the root.</summary>
    public UrdfJoint? ParentJointOf(string link) => _parentJoint.GetValueOrDefault(link);

    /// <summary>Joints whose parent is <paramref name="link"/>.</summary>
    public IEnumerable<UrdfJoint> ChildJointsOf(string link) => _joints.Values.Where(j => j.Parent == link);

    /// <summary>Movable (single-DOF, non-mimic) joints in tree order.</summary>
    public IReadOnlyList<UrdfJoint> ActuatedJoints => TreeOrder().Where(j => j.IsMovable && j.Mimic is null).ToList();

    /// <summary>Joints in depth-first order from the root.</summary>
    public IEnumerable<UrdfJoint> TreeOrder()
    {
        var stack = new Stack<string>();
        stack.Push(RootLink);
        while (stack.Count > 0)
        {
            var link = stack.Pop();
            foreach (var j in ChildJointsOf(link).OrderByDescending(j => j.Name, StringComparer.Ordinal))
            {
                yield return j;
                stack.Push(j.Child);
            }
        }
    }

    public static UrdfModel LoadFile(string path) => Parse(File.ReadAllText(path));

    /// <summary>Loads one of the bundled robots: <c>ur5</c>, <c>diffbot</c>, <c>planar2</c>.</summary>
    public static UrdfModel LoadBuiltIn(string name)
    {
        using var stream = Assembly.GetExecutingAssembly().GetManifestResourceStream($"ROS2.Net.Manipulation.Robots.{name}.urdf")
            ?? throw new KeyNotFoundException($"No bundled robot '{name}'.");
        using var reader = new StreamReader(stream);
        return Parse(reader.ReadToEnd());
    }

    /// <summary>Parses URDF XML.</summary>
    public static UrdfModel Parse(string xml)
    {
        var doc = XDocument.Parse(xml);
        var robot = doc.Root is { Name.LocalName: "robot" } r ? r : throw new UrdfException("URDF root element must be <robot>.");
        var model = new UrdfModel((string?)robot.Attribute("name") ?? "robot", xml);
        var materials = robot.Elements("material").ToDictionary(m => (string?)m.Attribute("name") ?? string.Empty, m => m);

        foreach (var l in robot.Elements("link"))
        {
            var link = new UrdfLink { Name = Req(l, "name"), Inertial = ParseInertial(l.Element("inertial")) };
            foreach (var v in l.Elements("visual"))
            {
                if (ParseGeometry(v) is { } g)
                {
                    link.Visuals.Add(new GeometryElement(ParseOrigin(v.Element("origin")), g, (string?)v.Attribute("name"), (string?)v.Element("material")?.Attribute("name")));
                }
            }

            foreach (var c in l.Elements("collision"))
            {
                if (ParseGeometry(c) is { } g)
                {
                    link.Collisions.Add(new GeometryElement(ParseOrigin(c.Element("origin")), g, (string?)c.Attribute("name")));
                }
            }

            if (!model._links.TryAdd(link.Name, link))
            {
                throw new UrdfException($"Duplicate link '{link.Name}'.");
            }
        }

        foreach (var j in robot.Elements("joint"))
        {
            var type = Req(j, "type") switch
            {
                "fixed" => JointType.Fixed,
                "revolute" => JointType.Revolute,
                "continuous" => JointType.Continuous,
                "prismatic" => JointType.Prismatic,
                "floating" => JointType.Floating,
                "planar" => JointType.Planar,
                var t => throw new UrdfException($"Unknown joint type '{t}'."),
            };
            var limit = j.Element("limit");
            var mimic = j.Element("mimic");
            var joint = new UrdfJoint
            {
                Name = Req(j, "name"),
                Type = type,
                Parent = Req(j.Element("parent") ?? throw new UrdfException("Joint without <parent>."), "link"),
                Child = Req(j.Element("child") ?? throw new UrdfException("Joint without <child>."), "link"),
                Origin = ParseOrigin(j.Element("origin")),
                Axis = j.Element("axis") is { } a ? ParseVec((string?)a.Attribute("xyz"), Vec3.UnitX) : Vec3.UnitX,
                Limits = limit is null ? null : new JointLimits(Num(limit, "lower", 0), Num(limit, "upper", 0), Num(limit, "effort", 0), Num(limit, "velocity", 0)),
                Mimic = mimic is null ? null : new JointMimic(Req(mimic, "joint"), Num(mimic, "multiplier", 1), Num(mimic, "offset", 0)),
            };
            if (type is JointType.Revolute or JointType.Prismatic && joint.Limits is null)
            {
                throw new UrdfException($"Joint '{joint.Name}' of type {type} needs <limit>.");
            }

            if (!model._links.ContainsKey(joint.Parent) || !model._links.ContainsKey(joint.Child))
            {
                throw new UrdfException($"Joint '{joint.Name}' references an unknown link.");
            }

            if (!model._joints.TryAdd(joint.Name, joint) || !model._parentJoint.TryAdd(joint.Child, joint))
            {
                throw new UrdfException($"Duplicate joint '{joint.Name}' or link '{joint.Child}' has two parents.");
            }
        }

        var roots = model._links.Keys.Where(l => !model._parentJoint.ContainsKey(l)).ToList();
        if (roots.Count != 1)
        {
            throw new UrdfException($"URDF must have exactly one root link, found {roots.Count}.");
        }

        model.RootLink = roots[0];
        _ = materials;
        return model;
    }

    private static string Req(XElement e, string attribute) =>
        (string?)e.Attribute(attribute) ?? throw new UrdfException($"<{e.Name.LocalName}> is missing '{attribute}'.");

    private static double Num(XElement e, string attribute, double fallback) =>
        e.Attribute(attribute) is { } a ? double.Parse(a.Value, CultureInfo.InvariantCulture) : fallback;

    private static Vec3 ParseVec(string? text, Vec3 fallback)
    {
        if (string.IsNullOrWhiteSpace(text))
        {
            return fallback;
        }

        var p = text.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
        return p.Length == 3 ? new Vec3(p[0], p[1], p[2]) : throw new UrdfException($"Expected three numbers, got '{text}'.");
    }

    private static Pose3 ParseOrigin(XElement? origin)
    {
        if (origin is null)
        {
            return Pose3.Identity;
        }

        var xyz = ParseVec((string?)origin.Attribute("xyz"), Vec3.Zero);
        var rpy = ParseVec((string?)origin.Attribute("rpy"), Vec3.Zero);
        return new Pose3(xyz, Quat.FromRpy(rpy.X, rpy.Y, rpy.Z));
    }

    private static Inertial? ParseInertial(XElement? e)
    {
        if (e is null)
        {
            return null;
        }

        var mass = e.Element("mass") is { } m ? Num(m, "value", 0) : 0;
        var i = e.Element("inertia");
        return new Inertial(ParseOrigin(e.Element("origin")), mass,
            i is null ? 0 : Num(i, "ixx", 0), i is null ? 0 : Num(i, "ixy", 0), i is null ? 0 : Num(i, "ixz", 0),
            i is null ? 0 : Num(i, "iyy", 0), i is null ? 0 : Num(i, "iyz", 0), i is null ? 0 : Num(i, "izz", 0));
    }

    private static Geometry? ParseGeometry(XElement parent)
    {
        var g = parent.Element("geometry")?.Elements().FirstOrDefault();
        return g?.Name.LocalName switch
        {
            "box" => new BoxGeometry(ParseVec((string?)g.Attribute("size"), new Vec3(1, 1, 1))),
            "cylinder" => new CylinderGeometry(Num(g, "radius", 0), Num(g, "length", 0)),
            "sphere" => new SphereGeometry(Num(g, "radius", 0)),
            "mesh" => new MeshGeometry((string?)g.Attribute("filename") ?? string.Empty, ParseVec((string?)g.Attribute("scale"), new Vec3(1, 1, 1))),
            _ => null,
        };
    }
}
