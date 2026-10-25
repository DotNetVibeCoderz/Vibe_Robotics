using System.Text.Json;
using System.Text.Json.Serialization;
using YamlDotNet.Serialization;
using YamlDotNet.Serialization.NamingConventions;

namespace ROS2.Net.Simulation;

/// <summary>
/// Serializable description of a world (JSON or YAML). Angles in files are in degrees.
/// <code>
/// name: lab
/// bounds: [0, 0, 10, 8]
/// obstacles:
///   - { type: wall, x1: 0, y1: 0, x2: 10, y2: 0 }
///   - { type: box, x: 4, y: 3, width: 2, depth: 0.6, yaw: 90 }
///   - { type: cylinder, x: 7, y: 5, radius: 0.2 }
/// robots:
///   - { name: robot1, type: diffdrive, x: 1, y: 1, yaw: 0 }
/// </code>
/// </summary>
public sealed class WorldDefinition
{
    private static readonly JsonSerializerOptions s_json = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
        WriteIndented = true,
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true,
    };

    public string Name { get; set; } = "world";

    /// <summary><c>[minX, minY, maxX, maxY]</c>.</summary>
    public double[] Bounds { get; set; } = [-10, -10, 10, 10];

    public ulong Seed { get; set; } = 42;

    public double Friction { get; set; } = 1.0;

    public string? FloorColor { get; set; }

    /// <summary><c>[latitude, longitude, altitude]</c> of the world origin.</summary>
    public double[]? GeoOrigin { get; set; }

    public List<ObstacleDefinition> Obstacles { get; set; } = [];

    public List<RobotDefinition> Robots { get; set; } = [];

    public List<PlaceDefinition> SpawnPoints { get; set; } = [];

    public List<PlaceDefinition> Waypoints { get; set; } = [];

    public static WorldDefinition FromJson(string json) =>
        JsonSerializer.Deserialize<WorldDefinition>(json, s_json) ?? throw new FormatException("Empty world file.");

    public static WorldDefinition FromYaml(string yaml) =>
        new DeserializerBuilder()
            .WithNamingConvention(CamelCaseNamingConvention.Instance)
            .IgnoreUnmatchedProperties()
            .Build()
            .Deserialize<WorldDefinition>(yaml) ?? throw new FormatException("Empty world file.");

    /// <summary>Loads a <c>.json</c>, <c>.yaml</c> or <c>.yml</c> world file.</summary>
    public static WorldDefinition FromFile(string path)
    {
        var text = File.ReadAllText(path);
        return Path.GetExtension(path).ToLowerInvariant() is ".yaml" or ".yml" ? FromYaml(text) : FromJson(text);
    }

    public string ToJson() => JsonSerializer.Serialize(this, s_json);

    public string ToYaml() =>
        new SerializerBuilder()
            .WithNamingConvention(CamelCaseNamingConvention.Instance)
            .ConfigureDefaultValuesHandling(DefaultValuesHandling.OmitNull)
            .Build()
            .Serialize(this);

    /// <summary>Builds a live world (obstacles, robots, spawn points and waypoints).</summary>
    public SimWorld Build(SimWorldOptions? options = null)
    {
        options ??= new SimWorldOptions();
        options.Name = Name;
        if (Bounds is { Length: 4 })
        {
            options.Bounds = new Bounds2D(Bounds[0], Bounds[1], Bounds[2], Bounds[3]);
        }

        options.Seed = Seed;
        options.Friction = Friction;
        if (FloorColor is not null)
        {
            options.FloorColor = SimColor.Parse(FloorColor);
        }

        if (GeoOrigin is { Length: >= 2 } g)
        {
            options.GeoOrigin = new GeoOrigin(g[0], g[1], g.Length > 2 ? g[2] : 0);
        }

        var world = new SimWorld(options);
        var i = 0;
        foreach (var o in Obstacles)
        {
            world.AddObstacle(o.Create($"{o.Type}_{i++}"));
        }

        foreach (var s in SpawnPoints)
        {
            world.SpawnPoints.Add((s.Name, s.ToPose()));
        }

        foreach (var w in Waypoints)
        {
            world.Waypoints[w.Name] = w.ToPose();
        }

        foreach (var r in Robots)
        {
            world.AddRobot(r.Create());
        }

        return world;
    }
}

/// <summary>An obstacle entry of a world file.</summary>
public sealed class ObstacleDefinition
{
    /// <summary><c>box</c>, <c>cylinder</c>, <c>wall</c> or <c>polygon</c>.</summary>
    public string Type { get; set; } = "box";

    public string? Name { get; set; }

    public double X { get; set; }

    public double Y { get; set; }

    public double Width { get; set; } = 1;

    public double Depth { get; set; } = 1;

    public double Radius { get; set; } = 0.25;

    public double Height { get; set; } = 1;

    /// <summary>Rotation in degrees.</summary>
    public double Yaw { get; set; }

    public double X1 { get; set; }

    public double Y1 { get; set; }

    public double X2 { get; set; }

    public double Y2 { get; set; }

    public double Thickness { get; set; } = 0.1;

    /// <summary>Polygon points as <c>[x, y]</c> pairs.</summary>
    public List<double[]>? Points { get; set; }

    public string? Color { get; set; }

    public bool Dynamic { get; set; }

    public double Mass { get; set; } = 5;

    public string? Tag { get; set; }

    public SimObstacle Create(string fallbackName)
    {
        var name = Name ?? fallbackName;
        SimColor? color = Color is null ? null : SimColor.Parse(Color);
        return Type.ToLowerInvariant() switch
        {
            "cylinder" or "circle" => SimObstacle.Cylinder(name, X, Y, Radius, Height, color, Dynamic, Tag ?? string.Empty),
            "wall" => SimObstacle.Wall(name, new Vec2(X1, Y1), new Vec2(X2, Y2), Thickness, Height == 1 ? 1.2 : Height, color),
            "polygon" => SimObstacle.Polygon(name, (Points ?? []).Select(p => new Vec2(p[0], p[1])).ToList(), Height, color, Tag ?? string.Empty),
            _ => SimObstacle.Box(name, X, Y, Width, Depth, Angles.ToRadians(Yaw), Height, color, Dynamic, Mass, Tag ?? string.Empty),
        };
    }
}

/// <summary>A robot entry of a world file.</summary>
public sealed class RobotDefinition
{
    public string Name { get; set; } = "robot1";

    /// <summary><c>diffdrive</c>, <c>ackermann</c> or <c>omni</c>.</summary>
    public string Type { get; set; } = "diffdrive";

    /// <summary>Parameter preset: <c>burger</c>, <c>waffle</c>, <c>car</c>, <c>omni</c>.</summary>
    public string? Preset { get; set; }

    /// <summary>ROS namespace; defaults to <c>/name</c>.</summary>
    public string? Namespace { get; set; }

    public double X { get; set; }

    public double Y { get; set; }

    /// <summary>Heading in degrees.</summary>
    public double Yaw { get; set; }

    public RobotSensorOptions Sensors { get; set; } = new();

    public SimRobot Create()
    {
        var robot = RobotFactory.Create(RobotFactory.ParseKind(Type), Name, new Pose2D(X, Y, Angles.ToRadians(Yaw)), RobotFactory.Preset(Preset), Sensors);
        if (Namespace is not null)
        {
            robot.Namespace = Namespace;
        }

        return robot;
    }
}

/// <summary>A named pose (spawn point or waypoint); yaw in degrees.</summary>
public sealed class PlaceDefinition
{
    public string Name { get; set; } = string.Empty;

    public double X { get; set; }

    public double Y { get; set; }

    public double Yaw { get; set; }

    public Pose2D ToPose() => new(X, Y, Angles.ToRadians(Yaw));
}
