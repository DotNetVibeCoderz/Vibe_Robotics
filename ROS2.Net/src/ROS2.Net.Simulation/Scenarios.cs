namespace ROS2.Net.Simulation;

/// <summary>Built-in worlds.</summary>
public enum ScenarioKind
{
    Empty,
    Warehouse,
    Office,
    Maze,
    CorridorLoop,
    AgricultureRows,
}

/// <summary>
/// Ready-made worlds. Each returns a <see cref="WorldDefinition"/> (editable, saveable data);
/// call <see cref="WorldDefinition.Build"/> or use <see cref="Create"/> for a live world.
/// </summary>
public static class Scenarios
{
    public static IReadOnlyList<ScenarioKind> All { get; } = Enum.GetValues<ScenarioKind>();

    /// <summary>Builds a live world, optionally with robots at the scenario's spawn points.</summary>
    public static SimWorld Create(ScenarioKind kind, int robots = 0, RobotKind robotKind = RobotKind.DiffDrive, ulong seed = 42, RobotSensorOptions? sensors = null)
    {
        var def = Definition(kind, seed);
        var world = def.Build();
        for (var i = 0; i < robots; i++)
        {
            var spawn = world.SpawnPoints.Count > 0 ? world.SpawnPoints[i % world.SpawnPoints.Count].Pose : new Pose2D(i, 0, 0);
            if (i >= world.SpawnPoints.Count && world.SpawnPoints.Count > 0)
            {
                spawn = spawn with { X = spawn.X + (0.6 * (i / world.SpawnPoints.Count)) };
            }

            world.AddRobot(RobotFactory.Create(robotKind, $"robot{i + 1}", spawn, null, sensors));
        }

        return world;
    }

    /// <summary>Parses a scenario name (case-insensitive, <c>-</c>/<c>_</c>/spaces ignored).</summary>
    public static ScenarioKind Parse(string name)
    {
        var key = name.Replace("-", string.Empty).Replace("_", string.Empty).Replace(" ", string.Empty);
        return Enum.TryParse<ScenarioKind>(key, ignoreCase: true, out var k)
            ? k
            : key.ToLowerInvariant() switch
            {
                "corridor" or "loop" => ScenarioKind.CorridorLoop,
                "agriculture" or "farm" or "rows" => ScenarioKind.AgricultureRows,
                _ => throw new ArgumentException($"Unknown scenario '{name}'. Known: {string.Join(", ", All)}."),
            };
    }

    public static WorldDefinition Definition(ScenarioKind kind, ulong seed = 42) => kind switch
    {
        ScenarioKind.Warehouse => Warehouse(seed),
        ScenarioKind.Office => Office(seed),
        ScenarioKind.Maze => Maze(seed),
        ScenarioKind.CorridorLoop => CorridorLoop(seed),
        ScenarioKind.AgricultureRows => AgricultureRows(seed),
        _ => Empty(20, seed),
    };

    /// <summary>An open square of <paramref name="size"/> meters centered on the origin, without walls.</summary>
    public static WorldDefinition Empty(double size = 20, ulong seed = 42) => new()
    {
        Name = "empty",
        Bounds = [-size / 2, -size / 2, size / 2, size / 2],
        Seed = seed,
        SpawnPoints = [Place("origin", 0, 0), Place("east", 1, 0), Place("west", -1, 0), Place("north", 0, 1)],
    };

    /// <summary>A 24×16 m warehouse: shelf aisles, pillars, pushable crates, a loading dock and a charger.</summary>
    public static WorldDefinition Warehouse(ulong seed = 42)
    {
        var w = Walled("warehouse", 24, 16, seed);
        for (var row = 0; row < 3; row++)
        {
            for (var col = 0; col < 4; col++)
            {
                var x = 6 + (col * 4.2);
                var y = 4 + (row * 3.6);
                w.Obstacles.Add(new ObstacleDefinition { Type = "box", Name = $"shelf_{(char)('A' + row)}{col + 1}", X = x, Y = y, Width = 3.2, Depth = 0.8, Height = 2.2, Color = "#C48C48", Tag = "shelf" });
                w.Waypoints.Add(Place($"shelf_{(char)('A' + row)}{col + 1}", x, y - 1.0, 90));
            }
        }

        foreach (var (x, y) in new[] { (4.0, 2.0), (4.0, 14.0), (20.5, 2.0), (20.5, 14.0) })
        {
            w.Obstacles.Add(new ObstacleDefinition { Type = "cylinder", X = x, Y = y, Radius = 0.25, Height = 3, Color = "#6E7682", Tag = "pillar" });
        }

        var rng = new SimRandom(seed);
        for (var i = 0; i < 6; i++)
        {
            w.Obstacles.Add(new ObstacleDefinition
            {
                Type = "box", Name = $"crate_{i + 1}", X = 22 - rng.Uniform(0, 1.5), Y = 5 + (i * 1.1), Width = 0.5, Depth = 0.5,
                Height = 0.5, Yaw = rng.Uniform(-20, 20), Dynamic = true, Mass = 4, Color = "#AA6E3C", Tag = "crate",
            });
        }

        w.Obstacles.Add(new ObstacleDefinition { Type = "box", Name = "charger", X = 1.0, Y = 8, Width = 0.3, Depth = 0.6, Height = 0.4, Color = "#2EB872", Tag = "charger" });
        w.SpawnPoints.AddRange([Place("dock_1", 2, 2), Place("dock_2", 2, 3.2), Place("dock_3", 2, 12.8), Place("dock_4", 2, 14)]);
        w.Waypoints.AddRange([Place("loading_dock", 22, 1.5, 180), Place("charger", 1.6, 8, 180), Place("entrance", 2, 8)]);
        return w;
    }

    /// <summary>A 16×12 m office with four rooms, doorways, desks and a meeting table.</summary>
    public static WorldDefinition Office(ulong seed = 42)
    {
        var w = Walled("office", 16, 12, seed);
        // Central corridor along y=6, rooms north and south with 1 m doors.
        AddWallWithDoor(w, 0, 4.5, 16, 4.5, [3, 11]);
        AddWallWithDoor(w, 0, 7.5, 16, 7.5, [5, 13]);
        w.Obstacles.Add(Wall(8, 0, 8, 4.5));
        w.Obstacles.Add(Wall(8, 7.5, 8, 12));
        foreach (var (x, y) in new[] { (2.0, 2.0), (5.5, 1.2), (10.5, 2.0), (13.5, 1.2), (2.5, 10.5), (13.5, 10.5) })
        {
            w.Obstacles.Add(new ObstacleDefinition { Type = "box", X = x, Y = y, Width = 1.4, Depth = 0.7, Height = 0.75, Color = "#8D6E63", Tag = "desk" });
        }

        w.Obstacles.Add(new ObstacleDefinition { Type = "box", Name = "meeting_table", X = 4, Y = 9.8, Width = 2.4, Depth = 1.2, Height = 0.75, Color = "#5D4037", Tag = "table" });
        w.Obstacles.Add(new ObstacleDefinition { Type = "cylinder", Name = "plant", X = 11, Y = 9.5, Radius = 0.3, Height = 1.2, Color = "#43A047", Tag = "plant" });
        w.SpawnPoints.AddRange([Place("lobby", 1.2, 6), Place("lobby_2", 2.2, 6)]);
        w.Waypoints.AddRange([Place("room_sw", 4, 2.5), Place("room_se", 12, 2.5), Place("room_nw", 4, 9.5, 90), Place("room_ne", 12, 9.5, 90), Place("corridor_end", 15, 6)]);
        return w;
    }

    /// <summary>A seeded 8×6 maze with 1.5 m cells; the exit waypoint is the far corner.</summary>
    public static WorldDefinition Maze(ulong seed = 42, int cols = 8, int rows = 6, double cell = 1.5)
    {
        var w = Walled("maze", cols * cell, rows * cell, seed);
        var rng = new SimRandom(seed);

        // Recursive backtracker on a cell grid; walls[c, r, 0] = east wall, [.., 1] = north wall.
        var east = new bool[cols, rows];
        var north = new bool[cols, rows];
        for (var c = 0; c < cols; c++)
        {
            for (var r = 0; r < rows; r++)
            {
                east[c, r] = north[c, r] = true;
            }
        }

        var visited = new bool[cols, rows];
        var stack = new Stack<(int C, int R)>();
        stack.Push((0, 0));
        visited[0, 0] = true;
        Span<(int dc, int dr)> dirs = [(1, 0), (-1, 0), (0, 1), (0, -1)];
        while (stack.Count > 0)
        {
            var (c, r) = stack.Peek();
            var options = new List<(int, int)>();
            foreach (var (dc, dr) in dirs)
            {
                var nc = c + dc;
                var nr = r + dr;
                if (nc >= 0 && nc < cols && nr >= 0 && nr < rows && !visited[nc, nr])
                {
                    options.Add((dc, dr));
                }
            }

            if (options.Count == 0)
            {
                stack.Pop();
                continue;
            }

            var (odc, odr) = options[(int)(rng.NextDouble() * options.Count)];
            if (odc == 1)
            {
                east[c, r] = false;
            }
            else if (odc == -1)
            {
                east[c - 1, r] = false;
            }
            else if (odr == 1)
            {
                north[c, r] = false;
            }
            else
            {
                north[c, r - 1] = false;
            }

            visited[c + odc, r + odr] = true;
            stack.Push((c + odc, r + odr));
        }

        for (var c = 0; c < cols; c++)
        {
            for (var r = 0; r < rows; r++)
            {
                if (east[c, r] && c < cols - 1)
                {
                    w.Obstacles.Add(Wall((c + 1) * cell, r * cell, (c + 1) * cell, (r + 1) * cell));
                }

                if (north[c, r] && r < rows - 1)
                {
                    w.Obstacles.Add(Wall(c * cell, (r + 1) * cell, (c + 1) * cell, (r + 1) * cell));
                }
            }
        }

        w.SpawnPoints.Add(Place("start", cell / 2, cell / 2));
        w.Waypoints.Add(Place("exit", (cols - 0.5) * cell, (rows - 0.5) * cell));
        return w;
    }

    /// <summary>A rectangular loop corridor, 2 m wide, around a 10×6 m block — ideal for SLAM loop closure.</summary>
    public static WorldDefinition CorridorLoop(ulong seed = 42)
    {
        var w = Walled("corridor_loop", 14, 10, seed);
        w.Obstacles.Add(new ObstacleDefinition { Type = "box", Name = "core", X = 7, Y = 5, Width = 10, Depth = 6, Height = 1.2, Color = "#5A6270", Tag = "wall" });
        foreach (var (x, y) in new[] { (3.5, 0.6), (10.5, 9.4), (13.4, 3.0), (0.6, 7.0) })
        {
            w.Obstacles.Add(new ObstacleDefinition { Type = "cylinder", X = x, Y = y, Radius = 0.15, Height = 1.0, Color = "#E0A030", Tag = "landmark" });
        }

        w.SpawnPoints.AddRange([Place("start", 1, 1), Place("start_2", 2, 1)]);
        w.Waypoints.AddRange([Place("corner_se", 13, 1, 90), Place("corner_ne", 13, 9, 180), Place("corner_nw", 1, 9, 270), Place("corner_sw", 1, 1)]);
        return w;
    }

    /// <summary>A 24×16 m field with crop rows, headlands, trees and lower traction.</summary>
    public static WorldDefinition AgricultureRows(ulong seed = 42)
    {
        var w = new WorldDefinition
        {
            Name = "agriculture_rows",
            Bounds = [0, 0, 24, 16],
            Seed = seed,
            Friction = 0.85,
            FloorColor = "#B8A07A",
            GeoOrigin = [-7.25, 107.9, 1150],
        };
        for (var i = 0; i < 9; i++)
        {
            var y = 3 + (i * 1.2);
            w.Obstacles.Add(new ObstacleDefinition { Type = "box", Name = $"row_{i + 1}", X = 12, Y = y, Width = 16, Depth = 0.35, Height = 0.6, Color = "#4CA046", Tag = "crop" });
            w.Waypoints.Add(Place($"row_{i + 1}_start", 3.2, y + 0.6));
            w.Waypoints.Add(Place($"row_{i + 1}_end", 20.8, y + 0.6, 180));
        }

        var rng = new SimRandom(seed);
        for (var i = 0; i < 10; i++)
        {
            w.Obstacles.Add(new ObstacleDefinition { Type = "cylinder", Name = $"tree_{i + 1}", X = 1 + (i * 2.4), Y = 15.2 + rng.Uniform(-0.2, 0.2), Radius = 0.35, Height = 4, Color = "#2E7D32", Tag = "tree" });
        }

        w.Obstacles.Add(new ObstacleDefinition { Type = "box", Name = "barn", X = 22, Y = 1.2, Width = 3.5, Depth = 2, Height = 3, Color = "#B03A2E", Tag = "building" });
        w.SpawnPoints.AddRange([Place("gate", 1.5, 1.5), Place("gate_2", 1.5, 2.3)]);
        return w;
    }

    private static WorldDefinition Walled(string name, double width, double height, ulong seed)
    {
        var w = new WorldDefinition { Name = name, Bounds = [0, 0, width, height], Seed = seed };
        w.Obstacles.Add(Wall(0, 0, width, 0));
        w.Obstacles.Add(Wall(width, 0, width, height));
        w.Obstacles.Add(Wall(width, height, 0, height));
        w.Obstacles.Add(Wall(0, height, 0, 0));
        return w;
    }

    private static void AddWallWithDoor(WorldDefinition w, double x1, double y1, double x2, double y2, double[] doorCenters, double doorWidth = 1.0)
    {
        // Horizontal walls only (y1 == y2), which is all the office needs.
        var cursor = x1;
        foreach (var d in doorCenters.Order())
        {
            w.Obstacles.Add(Wall(cursor, y1, d - (doorWidth / 2), y2));
            cursor = d + (doorWidth / 2);
        }

        w.Obstacles.Add(Wall(cursor, y1, x2, y2));
    }

    private static ObstacleDefinition Wall(double x1, double y1, double x2, double y2) =>
        new() { Type = "wall", X1 = x1, Y1 = y1, X2 = x2, Y2 = y2, Thickness = 0.1, Height = 1.2, Color = "#5A6270" };

    private static PlaceDefinition Place(string name, double x, double y, double yaw = 0) => new() { Name = name, X = x, Y = y, Yaw = yaw };
}
