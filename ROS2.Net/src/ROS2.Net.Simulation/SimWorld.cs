using System.Diagnostics;
using ROS2.Net.Msgs.NavMsgs;

namespace ROS2.Net.Simulation;

/// <summary>Construction options of a <see cref="SimWorld"/>.</summary>
public sealed class SimWorldOptions
{
    public string Name { get; set; } = "world";

    /// <summary>Area covered by the spatial index and exported maps.</summary>
    public Bounds2D Bounds { get; set; } = new(-10, -10, 10, 10);

    /// <summary>Seed of every noise source; equal seeds give identical runs.</summary>
    public ulong Seed { get; set; } = 42;

    /// <summary>Physics step (s). Default 0.01 (100 Hz).</summary>
    public double StepSize { get; set; } = 0.01;

    /// <summary>Traction factor of the floor in (0, 1]; lower values mean wheels achieve less motion.</summary>
    public double Friction { get; set; } = 1.0;

    public GeoOrigin GeoOrigin { get; set; } = GeoOrigin.Default;

    public SimColor FloorColor { get; set; } = SimColor.Floor;

    /// <summary>Cell size of the static spatial index (m).</summary>
    public double IndexCellSize { get; set; } = 1.0;
}

/// <summary>
/// A deterministic 2.5D robot world: planar physics and sensing with heights for rendering.
/// Stepping is fixed-rate and allocation-free; all mutation happens under <see cref="SyncRoot"/>,
/// so a renderer can call <see cref="Snapshot"/> from its own thread while the world runs.
/// </summary>
public sealed class SimWorld
{
    private readonly object _gate = new();
    private readonly List<SimObstacle> _static = [];
    private readonly List<SimObstacle> _dynamic = [];
    private readonly List<SimRobot> _robots = [];
    private SimRobot[] _robotArray = [];
    private SimObstacle[] _staticArray = [];
    private SimObstacle[] _dynamicArray = [];
    private int[][] _cells = [];
    private int _cols, _rows;
    private double _cellSize;
    private Bounds2D _gridBounds;
    private int[] _stamps = [];
    private int _stamp;
    private bool _indexDirty = true;
    private int _staticVersion;
    private ObstacleSnapshot[] _staticSnapshot = [];
    private int _staticSnapshotVersion = -1;

    public SimWorld(SimWorldOptions? options = null)
    {
        options ??= new SimWorldOptions();
        Name = options.Name;
        Bounds = options.Bounds;
        Seed = options.Seed;
        StepSize = options.StepSize;
        Friction = options.Friction;
        GeoOrigin = options.GeoOrigin;
        FloorColor = options.FloorColor;
        _cellSize = options.IndexCellSize;
        Random = new SimRandom(Seed);
    }

    public string Name { get; set; }

    public Bounds2D Bounds { get; set; }

    public ulong Seed { get; set; }

    public double StepSize { get; }

    public double Friction { get; set; }

    public GeoOrigin GeoOrigin { get; set; }

    public SimColor FloorColor { get; set; }

    /// <summary>Simulation time (s) since start or reset.</summary>
    public double Time { get; private set; }

    public long StepCount { get; private set; }

    /// <summary>Shared deterministic noise source.</summary>
    public SimRandom Random { get; }

    /// <summary>When set, <see cref="RunAsync"/> stops stepping (manual <see cref="Step()"/> still works).</summary>
    public bool Paused { get; set; }

    /// <summary>Simulated seconds per wall-clock second for <see cref="RunAsync"/>; 0 or less runs as fast as possible.</summary>
    public double RealTimeFactor { get; set; } = 1.0;

    /// <summary>Lock guarding world state.</summary>
    public object SyncRoot => _gate;

    /// <summary>Named spawn points supplied by the scenario.</summary>
    public List<(string Name, Pose2D Pose)> SpawnPoints { get; } = [];

    /// <summary>Named waypoints (e.g. shelves, docks) supplied by the scenario.</summary>
    public Dictionary<string, Pose2D> Waypoints { get; } = new(StringComparer.OrdinalIgnoreCase);

    public IReadOnlyList<SimRobot> Robots => Volatile.Read(ref _robotArray);

    public IReadOnlyList<SimObstacle> StaticObstacles => Volatile.Read(ref _staticArray);

    public IReadOnlyList<SimObstacle> DynamicObstacles => Volatile.Read(ref _dynamicArray);

    /// <summary>Raised on the stepping thread after every step, outside the lock.</summary>
    public event Action<SimWorld>? Stepped;

    /// <summary>Raised after <see cref="Reset"/>.</summary>
    public event Action<SimWorld>? WasReset;

    /// <summary>Raised when robots are added or removed.</summary>
    public event Action<SimRobot, bool>? RobotsChanged;

    // ------------------------------------------------------------------ content

    public SimObstacle AddObstacle(SimObstacle obstacle)
    {
        lock (_gate)
        {
            if (obstacle.IsDynamic)
            {
                _dynamic.Add(obstacle);
                _dynamicArray = [.. _dynamic];
            }
            else
            {
                _static.Add(obstacle);
                _staticArray = [.. _static];
                _indexDirty = true;
                _staticVersion++;
            }
        }

        return obstacle;
    }

    public bool RemoveObstacle(SimObstacle obstacle)
    {
        lock (_gate)
        {
            if (_dynamic.Remove(obstacle))
            {
                _dynamicArray = [.. _dynamic];
                return true;
            }

            if (_static.Remove(obstacle))
            {
                _staticArray = [.. _static];
                _indexDirty = true;
                _staticVersion++;
                return true;
            }

            return false;
        }
    }

    public T AddRobot<T>(T robot)
        where T : SimRobot
    {
        lock (_gate)
        {
            if (_robots.Exists(r => r.Name == robot.Name))
            {
                throw new InvalidOperationException($"A robot named '{robot.Name}' already exists.");
            }

            _robots.Add(robot);
            _robotArray = [.. _robots];
        }

        RobotsChanged?.Invoke(robot, true);
        return robot;
    }

    public bool RemoveRobot(string name)
    {
        SimRobot? removed;
        lock (_gate)
        {
            removed = _robots.Find(r => r.Name == name);
            if (removed is null)
            {
                return false;
            }

            _robots.Remove(removed);
            _robotArray = [.. _robots];
        }

        RobotsChanged?.Invoke(removed, false);
        return true;
    }

    public SimRobot? GetRobot(string name) => Robots.FirstOrDefault(r => r.Name == name);

    // ------------------------------------------------------------------ stepping

    /// <summary>Advances the world by one fixed step.</summary>
    public void Step()
    {
        lock (_gate)
        {
            StepLocked();
        }

        Stepped?.Invoke(this);
    }

    /// <summary>Advances by <paramref name="steps"/> steps.</summary>
    public void Step(int steps)
    {
        for (var i = 0; i < steps; i++)
        {
            Step();
        }
    }

    /// <summary>Advances by at least <paramref name="seconds"/> of simulated time.</summary>
    public void StepFor(double seconds) => Step((int)Math.Ceiling(seconds / StepSize - 1e-9));

    private void StepLocked()
    {
        EnsureIndex();
        var dt = StepSize;
        var robots = _robotArray;
        for (var i = 0; i < robots.Length; i++)
        {
            robots[i].Step(this, dt);
        }

        Time += dt;
        StepCount++;
        for (var i = 0; i < robots.Length; i++)
        {
            var sensors = robots[i].Sensors;
            for (var s = 0; s < sensors.Count; s++)
            {
                sensors[s].Tick(this, robots[i], dt);
            }
        }
    }

    /// <summary>Restores robots and dynamic obstacles to their start, resets time and reseeds noise.</summary>
    public void Reset()
    {
        lock (_gate)
        {
            foreach (var r in _robots)
            {
                r.Reset();
            }

            foreach (var o in _dynamic)
            {
                o.ResetToInitial();
            }

            Time = 0;
            StepCount = 0;
            Random.Reseed(Seed);
        }

        WasReset?.Invoke(this);
    }

    /// <summary>
    /// Steps in (scaled) real time until cancelled. Missed steps are caught up in bursts of at most
    /// 50 so a slow machine degrades the real-time factor rather than freezing.
    /// </summary>
    public async Task RunAsync(CancellationToken cancellationToken)
    {
        var clock = Stopwatch.StartNew();
        var simAtStart = Time;
        var wallAtStart = 0.0;
        var lastFactor = RealTimeFactor;
        while (!cancellationToken.IsCancellationRequested)
        {
            if (Paused)
            {
                await Task.Delay(10, cancellationToken).ConfigureAwait(false);
                simAtStart = Time;
                wallAtStart = clock.Elapsed.TotalSeconds;
                continue;
            }

            if (RealTimeFactor != lastFactor || Time < simAtStart)
            {
                // Factor changed or the world was reset: re-anchor.
                lastFactor = RealTimeFactor;
                simAtStart = Time;
                wallAtStart = clock.Elapsed.TotalSeconds;
            }

            if (RealTimeFactor <= 0)
            {
                Step(100);
                await Task.Yield();
                continue;
            }

            var target = simAtStart + ((clock.Elapsed.TotalSeconds - wallAtStart) * RealTimeFactor);
            var steps = 0;
            while (Time + (StepSize / 2) < target && steps < 50)
            {
                Step();
                steps++;
            }

            if (steps == 50)
            {
                simAtStart = Time;
                wallAtStart = clock.Elapsed.TotalSeconds;
            }

            await Task.Delay(1, cancellationToken).ConfigureAwait(false);
        }
    }

    // ------------------------------------------------------------------ spatial index

    private void EnsureIndex()
    {
        if (!_indexDirty)
        {
            return;
        }

        var bounds = Bounds;
        foreach (var o in _static)
        {
            bounds = bounds.Union(o.Bounds);
        }

        bounds = bounds.Inflate(_cellSize);
        _gridBounds = bounds;
        _cols = Math.Max(1, (int)Math.Ceiling(bounds.Width / _cellSize));
        _rows = Math.Max(1, (int)Math.Ceiling(bounds.Height / _cellSize));
        var lists = new List<int>[_cols * _rows];
        for (var i = 0; i < _static.Count; i++)
        {
            var b = _static[i].Bounds;
            var (c0, r0) = CellOf(new Vec2(b.MinX, b.MinY));
            var (c1, r1) = CellOf(new Vec2(b.MaxX, b.MaxY));
            for (var r = r0; r <= r1; r++)
            {
                for (var c = c0; c <= c1; c++)
                {
                    (lists[(r * _cols) + c] ??= []).Add(i);
                }
            }
        }

        _cells = lists.Select(l => l?.ToArray() ?? []).ToArray();
        _stamps = new int[_static.Count];
        _stamp = 0;
        _indexDirty = false;
    }

    private (int Col, int Row) CellOf(Vec2 p) =>
        (Math.Clamp((int)Math.Floor((p.X - _gridBounds.MinX) / _cellSize), 0, _cols - 1),
         Math.Clamp((int)Math.Floor((p.Y - _gridBounds.MinY) / _cellSize), 0, _rows - 1));

    private int NextStamp()
    {
        if (++_stamp == int.MaxValue)
        {
            Array.Clear(_stamps);
            _stamp = 1;
        }

        return _stamp;
    }

    // ------------------------------------------------------------------ queries

    /// <summary>
    /// Casts a ray and returns the distance to the first hit (obstacles and robots), or
    /// <paramref name="maxRange"/> when nothing is hit. <paramref name="ignore"/> is excluded (the sensing robot).
    /// </summary>
    public double RayCast(Vec2 origin, double angle, double maxRange, SimRobot? ignore = null)
    {
        lock (_gate)
        {
            EnsureIndex();
            var (s, c) = Math.SinCos(angle);
            var dir = new Vec2(c, s);
            var best = RayCastStatic(origin, dir, maxRange);

            foreach (var o in _dynamicArray)
            {
                var t = o.RayIntersect(origin, dir, best);
                if (t < best)
                {
                    best = t;
                }
            }

            foreach (var r in _robotArray)
            {
                if (r == ignore)
                {
                    continue;
                }

                var t = SimObstacle.RayCircle(origin, dir, r.Pose.Position, r.Parameters.Radius, best);
                if (t < best)
                {
                    best = t;
                }
            }

            return best;
        }
    }

    private double RayCastStatic(Vec2 o, Vec2 d, double maxRange)
    {
        var best = maxRange;
        var g = _gridBounds;

        // Clip the ray to the grid (slab test).
        double tEnter = 0, tExit = maxRange;
        if (!ClipAxis(o.X, d.X, g.MinX, g.MaxX, ref tEnter, ref tExit) || !ClipAxis(o.Y, d.Y, g.MinY, g.MaxY, ref tEnter, ref tExit))
        {
            return best;
        }

        var start = o + (d * (tEnter + 1e-9));
        var (col, row) = CellOf(start);
        var stepX = d.X > 0 ? 1 : -1;
        var stepY = d.Y > 0 ? 1 : -1;
        var nextX = _gridBounds.MinX + ((col + (stepX > 0 ? 1 : 0)) * _cellSize);
        var nextY = _gridBounds.MinY + ((row + (stepY > 0 ? 1 : 0)) * _cellSize);
        var tMaxX = Math.Abs(d.X) < 1e-12 ? double.PositiveInfinity : (nextX - o.X) / d.X;
        var tMaxY = Math.Abs(d.Y) < 1e-12 ? double.PositiveInfinity : (nextY - o.Y) / d.Y;
        var tDeltaX = Math.Abs(d.X) < 1e-12 ? double.PositiveInfinity : _cellSize / Math.Abs(d.X);
        var tDeltaY = Math.Abs(d.Y) < 1e-12 ? double.PositiveInfinity : _cellSize / Math.Abs(d.Y);
        var stamp = NextStamp();

        while (true)
        {
            var cell = _cells[(row * _cols) + col];
            for (var i = 0; i < cell.Length; i++)
            {
                var idx = cell[i];
                if (_stamps[idx] == stamp)
                {
                    continue;
                }

                _stamps[idx] = stamp;
                var t = _staticArray[idx].RayIntersect(o, d, best);
                if (t < best)
                {
                    best = t;
                }
            }

            var cellExit = Math.Min(tMaxX, tMaxY);
            if (best <= cellExit || cellExit > tExit)
            {
                return best;
            }

            if (tMaxX < tMaxY)
            {
                col += stepX;
                tMaxX += tDeltaX;
            }
            else
            {
                row += stepY;
                tMaxY += tDeltaY;
            }

            if (col < 0 || col >= _cols || row < 0 || row >= _rows)
            {
                return best;
            }
        }
    }

    private static bool ClipAxis(double o, double d, double min, double max, ref double tEnter, ref double tExit)
    {
        if (Math.Abs(d) < 1e-12)
        {
            return o >= min && o <= max;
        }

        var t0 = (min - o) / d;
        var t1 = (max - o) / d;
        if (t0 > t1)
        {
            (t0, t1) = (t1, t0);
        }

        tEnter = Math.Max(tEnter, t0);
        tExit = Math.Min(tExit, t1);
        return tEnter <= tExit;
    }

    /// <summary>True when <paramref name="p"/> lies inside an obstacle (inflated by <paramref name="inflate"/>).</summary>
    public bool IsOccupied(Vec2 p, double inflate = 0, bool includeDynamic = true)
    {
        lock (_gate)
        {
            EnsureIndex();
            return FindObstacleAt(p, inflate, includeDynamic) is not null;
        }
    }

    private SimObstacle? FindObstacleAt(Vec2 p, double inflate, bool includeDynamic)
    {
        if (includeDynamic)
        {
            foreach (var o in _dynamicArray)
            {
                if (o.Contains(p, inflate))
                {
                    return o;
                }
            }
        }

        if (!_gridBounds.Contains(p))
        {
            return null;
        }

        var (c0, r0) = CellOf(new Vec2(p.X - inflate, p.Y - inflate));
        var (c1, r1) = CellOf(new Vec2(p.X + inflate, p.Y + inflate));
        for (var r = r0; r <= r1; r++)
        {
            for (var c = c0; c <= c1; c++)
            {
                foreach (var idx in _cells[(r * _cols) + c])
                {
                    if (_staticArray[idx].Contains(p, inflate))
                    {
                        return _staticArray[idx];
                    }
                }
            }
        }

        return null;
    }

    /// <summary>Color visible from above at <paramref name="p"/> (used by the camera sensor).</summary>
    public SimColor SampleColor(Vec2 p, SimRobot? ignore = null)
    {
        lock (_gate)
        {
            foreach (var r in _robotArray)
            {
                if (r != ignore && (p - r.Pose.Position).LengthSquared <= r.Parameters.Radius * r.Parameters.Radius)
                {
                    return r.Parameters.Color;
                }
            }

            EnsureIndex();
            if (FindObstacleAt(p, 0, true) is { } o)
            {
                return o.Color;
            }

            // Floor with faint 1 m tiles, which helps visual odometry demos.
            var fx = p.X - Math.Floor(p.X);
            var fy = p.Y - Math.Floor(p.Y);
            if (fx < 0.02 || fy < 0.02)
            {
                return new SimColor((byte)(FloorColor.R * 0.85), (byte)(FloorColor.G * 0.85), (byte)(FloorColor.B * 0.85));
            }

            return FloorColor;
        }
    }

    /// <summary>
    /// Pushes a robot out of obstacles and other robots, pushing dynamic obstacles along.
    /// Returns true when a contact occurred. Called under the lock, allocation-free.
    /// </summary>
    internal bool ResolveCollisions(SimRobot robot, Pose2D before)
    {
        var radius = robot.Parameters.Radius;
        var contact = false;
        for (var pass = 0; pass < 3; pass++)
        {
            var center = robot.Pose.Position;
            var resolved = true;

            // Static geometry via the index.
            var (c0, r0) = CellOf(new Vec2(center.X - radius, center.Y - radius));
            var (c1, r1) = CellOf(new Vec2(center.X + radius, center.Y + radius));
            var stamp = NextStamp();
            for (var r = r0; r <= r1; r++)
            {
                for (var c = c0; c <= c1; c++)
                {
                    foreach (var idx in _cells[(r * _cols) + c])
                    {
                        if (_stamps[idx] == stamp)
                        {
                            continue;
                        }

                        _stamps[idx] = stamp;
                        var depth = _staticArray[idx].Penetration(center, radius, out var push);
                        if (depth > 1e-9)
                        {
                            center += push * (depth + 1e-6);
                            contact = true;
                            resolved = false;
                        }
                    }
                }
            }

            foreach (var o in _dynamicArray)
            {
                var depth = o.Penetration(center, radius, out var push);
                if (depth <= 1e-9)
                {
                    continue;
                }

                contact = true;
                resolved = false;

                // Light objects move most of the way; heavy ones mostly stop the robot.
                var share = 1.0 / (1.0 + (o.Mass / 10.0));
                var previous = o.Center;
                o.Place(o.Center - (push * (depth * share)), o.Rotation);
                if (BlockedByStatic(o))
                {
                    o.Place(previous, o.Rotation);
                    share = 0;
                }

                center += push * ((depth * (1 - share)) + 1e-6);
            }

            foreach (var other in _robotArray)
            {
                if (other == robot)
                {
                    continue;
                }

                var d = center - other.Pose.Position;
                var min = radius + other.Parameters.Radius;
                var dist = d.Length;
                if (dist < min)
                {
                    var dir = dist > 1e-9 ? d * (1 / dist) : new Vec2(1, 0);
                    center += dir * (min - dist + 1e-6);
                    contact = true;
                    resolved = false;
                }
            }

            robot.Pose = robot.Pose with { X = center.X, Y = center.Y };
            if (resolved)
            {
                break;
            }
        }

        _ = before;
        return contact;
    }

    private bool BlockedByStatic(SimObstacle o)
    {
        var b = o.Bounds;
        var (c0, r0) = CellOf(new Vec2(b.MinX, b.MinY));
        var (c1, r1) = CellOf(new Vec2(b.MaxX, b.MaxY));
        for (var r = r0; r <= r1; r++)
        {
            for (var c = c0; c <= c1; c++)
            {
                foreach (var idx in _cells[(r * _cols) + c])
                {
                    var s = _staticArray[idx];
                    foreach (var v in o.Vertices)
                    {
                        if (s.Contains(v))
                        {
                            return true;
                        }
                    }

                    if (o.IsCircle && s.Contains(o.Center, o.Radius))
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    // ------------------------------------------------------------------ maps & snapshots

    /// <summary>
    /// Rasterizes the world into a ground-truth <c>nav_msgs/OccupancyGrid</c> (0 free, 100 occupied),
    /// origin at the bottom-left of <see cref="Bounds"/>.
    /// </summary>
    public OccupancyGrid ToOccupancyGrid(double resolution = 0.05, string frameId = "map", bool includeDynamic = false)
    {
        lock (_gate)
        {
            EnsureIndex();
            var b = Bounds;
            var width = (int)Math.Ceiling(b.Width / resolution);
            var height = (int)Math.Ceiling(b.Height / resolution);
            var data = new sbyte[width * height];
            var half = resolution * 0.5;
            for (var y = 0; y < height; y++)
            {
                for (var x = 0; x < width; x++)
                {
                    var p = new Vec2(b.MinX + ((x + 0.5) * resolution), b.MinY + ((y + 0.5) * resolution));
                    data[(y * width) + x] = FindObstacleAt(p, half * 0.99, includeDynamic) is null ? (sbyte)0 : (sbyte)100;
                }
            }

            var grid = new OccupancyGrid
            {
                Data = data,
                Info = new MapMetaData
                {
                    Resolution = (float)resolution,
                    Width = (uint)width,
                    Height = (uint)height,
                },
            };
            grid.Header.FrameId = frameId;
            new RosTime((long)(Time * 1e9)).CopyTo(grid.Header.Stamp);
            grid.Info.Origin.Position.X = b.MinX;
            grid.Info.Origin.Position.Y = b.MinY;
            grid.Info.Origin.Orientation.W = 1;
            return grid;
        }
    }

    /// <summary>
    /// An immutable view of the world for renderers. Static obstacles are cached and shared between
    /// snapshots until the static geometry changes, so taking a snapshot per frame is cheap.
    /// </summary>
    public WorldSnapshot Snapshot()
    {
        lock (_gate)
        {
            if (_staticSnapshotVersion != _staticVersion)
            {
                _staticSnapshot = _static.Select(ObstacleSnapshot.From).ToArray();
                _staticSnapshotVersion = _staticVersion;
            }

            var dynamicSnap = _dynamic.Select(ObstacleSnapshot.From).ToArray();
            var robots = _robots.Select(RobotSnapshot.From).ToArray();
            return new WorldSnapshot(Name, Time, StepCount, Bounds, FloorColor, _staticSnapshot, dynamicSnap, robots, Paused, RealTimeFactor);
        }
    }
}
