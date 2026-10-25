using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Navigation.Maps;

namespace ROS2.Net.Navigation.Slam;

/// <summary>SLAM modes, mirroring slam_toolbox.</summary>
public enum SlamMode
{
    /// <summary>Build a new map while driving.</summary>
    Mapping,
    /// <summary>Localize against the existing graph without adding keyframes (pure localization).</summary>
    Localization,
}

/// <summary>Options of <see cref="OnlineSlam"/> (names follow slam_toolbox where one exists).</summary>
public sealed record SlamOptions
{
    public double Resolution { get; init; } = 0.05;

    public double MaxLaserRange { get; init; } = 12.0;

    /// <summary>Add a keyframe after this much travel (<c>minimum_travel_distance</c>).</summary>
    public double MinimumTravelDistance { get; init; } = 0.3;

    /// <summary>…or this much rotation (<c>minimum_travel_heading</c>).</summary>
    public double MinimumTravelHeading { get; init; } = 0.35;

    /// <summary>Keyframes in the rolling local map used for scan-to-map matching.</summary>
    public int ScanBufferSize { get; init; } = 12;

    public double CorrelationSearchSpaceDimension { get; init; } = 0.4;

    public double CorrelationAngularWindow { get; init; } = 0.3;

    public double SmearDeviation { get; init; } = 0.06;

    /// <summary>Enable loop-closure detection.</summary>
    public bool DoLoopClosing { get; init; } = true;

    /// <summary>Only keyframes within this distance of the current estimate are loop candidates.</summary>
    public double LoopSearchMaximumDistance { get; init; } = 3.0;

    /// <summary>Candidates must be at least this many keyframes old.</summary>
    public int LoopMatchMinimumChainSize { get; init; } = 15;

    public double LoopSearchSpaceDimension { get; init; } = 1.5;

    public double LoopAngularWindow { get; init; } = 0.5;

    /// <summary>Minimum match score to accept a loop closure (<c>loop_match_minimum_response_fine</c>).</summary>
    public double LoopMatchMinimumResponse { get; init; } = 0.55;

    /// <summary>Beam stride used for matching (every n-th beam).</summary>
    public int BeamStride { get; init; } = 2;

    public Pose2D LaserOffset { get; init; }

    public SlamMode Mode { get; init; } = SlamMode.Mapping;
}

/// <summary>A stored keyframe: its graph node index, local scan points and the odometry pose when it was taken.</summary>
public sealed record SlamKeyframe(int Node, (double X, double Y)[] Points, Pose2D Odometry);

/// <summary>
/// Online graph SLAM in the spirit of slam_toolbox / Karto: odometry-predicted poses are refined by
/// correlative scan-to-local-map matching, keyframes become pose-graph nodes, loop closures are
/// detected by matching against older submaps and the graph is re-optimized, after which the
/// occupancy map is rebuilt from the corrected keyframes.
/// </summary>
public sealed class OnlineSlam
{
    private readonly SlamOptions _options;
    private readonly PoseGraph _graph = new();
    private readonly List<SlamKeyframe> _keyframes = [];
    private readonly CorrelativeScanMatcher _matcher = new();
    private Pose2D _lastKeyframeOdom;
    private bool _mapDirty = true;
    private OccupancyGridMap? _map;

    public SlamOptions Options => _options;

    public OnlineSlam(SlamOptions? options = null) => _options = options ?? new SlamOptions();

    public PoseGraph Graph => _graph;

    public IReadOnlyList<SlamKeyframe> Keyframes => _keyframes;

    public int LoopClosureCount { get; private set; }

    /// <summary>Best estimate of the current robot pose in the map frame.</summary>
    public Pose2D Pose { get; private set; }

    /// <summary>The map→odom correction: <c>map_pose ⊕ odom_pose⁻¹</c>.</summary>
    public Pose2D MapToOdom { get; private set; }

    /// <summary>Raised when a loop closure was added and the graph re-optimized.</summary>
    public event Action<int>? LoopClosed;

    /// <summary>
    /// Processes one scan with the odometry pose at the same time. Returns true when a keyframe was added.
    /// </summary>
    public bool Process(Pose2D odom, LaserScan scan)
    {
        var points = ScanPoints.FromScan(scan, _options.LaserOffset, _options.MaxLaserRange, _options.BeamStride);
        return Process(odom, points);
    }

    /// <summary>Processes robot-frame scan points with the odometry pose at the same time.</summary>
    public bool Process(Pose2D odom, (double X, double Y)[] points)
    {
        if (_keyframes.Count == 0)
        {
            if (_options.Mode == SlamMode.Localization)
            {
                return false;
            }

            var node = _graph.AddNode(odom);
            _keyframes.Add(new SlamKeyframe(node, points, odom));
            _lastKeyframeOdom = odom;
            Pose = odom;
            MapToOdom = Pose.Compose(odom.Inverse());
            _mapDirty = true;
            return true;
        }

        var lastKf = _keyframes[^1];
        var lastPose = _graph.Nodes[lastKf.Node];
        var odomDelta = _lastKeyframeOdom.Between(odom);
        var predicted = lastPose.Compose(odomDelta);
        var moved = Math.Sqrt((odomDelta.X * odomDelta.X) + (odomDelta.Y * odomDelta.Y)) >= _options.MinimumTravelDistance ||
                    Math.Abs(odomDelta.Theta) >= _options.MinimumTravelHeading;
        if (!moved || points.Length < 10)
        {
            Pose = predicted;
            return false;
        }

        // Scan-to-local-map matching.
        var local = BuildWorldPoints(Math.Max(0, _keyframes.Count - _options.ScanBufferSize), _keyframes.Count);
        var grid = MatchGrid.Build(local, _options.Resolution, _options.SmearDeviation);
        var step = AngularStep(points);
        var match = _matcher.Match(grid, points, predicted, _options.CorrelationSearchSpaceDimension, _options.CorrelationAngularWindow, step);
        var corrected = match.Score > 0.3 ? match.Pose : predicted;
        Pose = corrected;
        MapToOdom = corrected.Compose(odom.Inverse());

        if (_options.Mode == SlamMode.Localization)
        {
            return false;
        }

        var nodeIndex = _graph.AddNode(corrected);
        var infoXy = match.Score > 0.3 ? 400 * match.Score : 25;
        _graph.AddEdge(new PoseGraphEdge(lastKf.Node, nodeIndex, lastPose.Between(corrected), infoXy, infoXy * 4, IsLoopClosure: false));
        _keyframes.Add(new SlamKeyframe(nodeIndex, points, odom));
        _lastKeyframeOdom = odom;
        _mapDirty = true;

        if (_options.DoLoopClosing && TryCloseLoop(points, step))
        {
            Pose = _graph.Nodes[_keyframes[^1].Node];
            MapToOdom = Pose.Compose(odom.Inverse());
        }

        return true;
    }

    private bool TryCloseLoop((double X, double Y)[] points, double step)
    {
        var current = _keyframes.Count - 1;
        var pose = _graph.Nodes[_keyframes[current].Node];
        var bestIndex = -1;
        var bestDistance = double.MaxValue;
        for (var i = 0; i < current - _options.LoopMatchMinimumChainSize; i++)
        {
            var d = _graph.Nodes[_keyframes[i].Node].DistanceTo(pose);
            if (d < _options.LoopSearchMaximumDistance && d < bestDistance)
            {
                bestDistance = d;
                bestIndex = i;
            }
        }

        if (bestIndex < 0)
        {
            return false;
        }

        var from = Math.Max(0, bestIndex - 5);
        var to = Math.Min(current - _options.LoopMatchMinimumChainSize, bestIndex + 6);
        var submap = MatchGrid.Build(BuildWorldPoints(from, to), _options.Resolution, _options.SmearDeviation);
        var match = _matcher.Match(submap, points, pose, _options.LoopSearchSpaceDimension, _options.LoopAngularWindow, step);
        if (match.Score < _options.LoopMatchMinimumResponse)
        {
            return false;
        }

        var candidate = _graph.Nodes[_keyframes[bestIndex].Node];
        _graph.AddEdge(new PoseGraphEdge(_keyframes[bestIndex].Node, _keyframes[current].Node, candidate.Between(match.Pose), 800 * match.Score, 3200 * match.Score, IsLoopClosure: true));
        _graph.Optimize();
        LoopClosureCount++;
        _mapDirty = true;
        LoopClosed?.Invoke(LoopClosureCount);
        return true;
    }

    private double AngularStep((double X, double Y)[] points)
    {
        var maxRange = 0.0;
        foreach (var (x, y) in points)
        {
            maxRange = Math.Max(maxRange, (x * x) + (y * y));
        }

        maxRange = Math.Max(1.0, Math.Sqrt(maxRange));

        // Rotation that moves the farthest point by about one cell.
        return Math.Clamp(Math.Acos(1 - ((_options.Resolution * _options.Resolution) / (2 * maxRange * maxRange))), 0.002, 0.05);
    }

    private List<(double X, double Y)> BuildWorldPoints(int from, int to)
    {
        var list = new List<(double, double)>();
        for (var k = from; k < to; k++)
        {
            var kf = _keyframes[k];
            var p = _graph.Nodes[kf.Node];
            foreach (var (x, y) in kf.Points)
            {
                list.Add(p.TransformPoint(x, y));
            }
        }

        return list;
    }

    /// <summary>
    /// The occupancy map built from all keyframes at their optimized poses (log-odds ray casting).
    /// Rebuilt lazily after keyframes are added or the graph changes.
    /// </summary>
    public OccupancyGridMap GetMap()
    {
        if (!_mapDirty && _map is not null)
        {
            return _map;
        }

        _map = BuildMap();
        _mapDirty = false;
        return _map;
    }

    private OccupancyGridMap BuildMap()
    {
        var res = _options.Resolution;
        double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
        foreach (var kf in _keyframes)
        {
            var p = _graph.Nodes[kf.Node];
            minX = Math.Min(minX, p.X);
            minY = Math.Min(minY, p.Y);
            maxX = Math.Max(maxX, p.X);
            maxY = Math.Max(maxY, p.Y);
            foreach (var (x, y) in kf.Points)
            {
                var (wx, wy) = p.TransformPoint(x, y);
                minX = Math.Min(minX, wx);
                minY = Math.Min(minY, wy);
                maxX = Math.Max(maxX, wx);
                maxY = Math.Max(maxY, wy);
            }
        }

        if (_keyframes.Count == 0)
        {
            return new OccupancyGridMap(1, 1, res);
        }

        minX -= 1;
        minY -= 1;
        var w = (int)Math.Ceiling((maxX + 1 - minX) / res);
        var h = (int)Math.Ceiling((maxY + 1 - minY) / res);
        var map = new OccupancyGridMap(w, h, res, new Pose2D(minX, minY, 0));
        var logOdds = new float[w * h];
        var seen = new bool[w * h];
        const float hit = 0.85f;
        const float miss = -0.4f;
        foreach (var kf in _keyframes)
        {
            var p = _graph.Nodes[kf.Node];
            var sensor = p.Compose(_options.LaserOffset);
            map.WorldToCell(sensor.X, sensor.Y, out var sx, out var sy);
            foreach (var (x, y) in kf.Points)
            {
                var (wx, wy) = p.TransformPoint(x, y);
                if (!map.WorldToCell(wx, wy, out var ex, out var ey))
                {
                    continue;
                }

                OccupancyGridMap.RayTrace(sx, sy, ex, ey, (cx, cy) =>
                {
                    if (cx == ex && cy == ey)
                    {
                        return false;
                    }

                    if (map.InBounds(cx, cy))
                    {
                        var i = (cy * w) + cx;
                        logOdds[i] = Math.Max(-4f, logOdds[i] + miss);
                        seen[i] = true;
                    }

                    return true;
                });
                var hi = (ey * w) + ex;
                logOdds[hi] = Math.Min(6f, logOdds[hi] + hit);
                seen[hi] = true;
            }
        }

        for (var i = 0; i < logOdds.Length; i++)
        {
            if (!seen[i])
            {
                continue;
            }

            var prob = 1.0 - (1.0 / (1.0 + Math.Exp(logOdds[i])));
            map.Data[i] = prob > 0.65 ? OccupancyGridMap.Occupied : prob < 0.35 ? OccupancyGridMap.Free : (sbyte)Math.Round(prob * 100);
        }

        return map;
    }

    /// <summary>Saves the current map in map_server format.</summary>
    public void SaveMap(string basePath) => GetMap().SaveMapServer(basePath);
}
