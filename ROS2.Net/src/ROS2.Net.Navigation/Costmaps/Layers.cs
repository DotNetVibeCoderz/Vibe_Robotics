using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Navigation.Maps;

namespace ROS2.Net.Navigation.Costmaps;

/// <summary>Copies a static map into the costmap: occupied → lethal, unknown → no-information (or free).</summary>
public sealed class StaticLayer(OccupancyGridMap map, bool trackUnknown = false) : ICostmapLayer
{
    private OccupancyGridMap _map = map;

    public string Name => "static_layer";

    public bool Enabled { get; set; } = true;

    public OccupancyGridMap Map => _map;

    /// <summary>Replaces the map (e.g. a new <c>/map</c> from SLAM). The new map must have the costmap's size.</summary>
    public void SetMap(OccupancyGridMap map) => _map = map;

    public void UpdateCosts(Costmap2D master)
    {
        var data = _map.Data;
        var occ = _map.OccupiedThreshold;
        var sameGeometry = _map.Width == master.Width && _map.Height == master.Height;
        for (var cy = 0; cy < master.Height; cy++)
        {
            for (var cx = 0; cx < master.Width; cx++)
            {
                sbyte v;
                if (sameGeometry)
                {
                    v = data[(cy * master.Width) + cx];
                }
                else
                {
                    var (wx, wy) = master.CellToWorld(cx, cy);
                    v = _map.WorldToCell(wx, wy, out var mx, out var my) ? _map[mx, my] : OccupancyGridMap.Unknown;
                }

                var i = (cy * master.Width) + cx;
                if (v >= occ)
                {
                    master.Data[i] = Costs.Lethal;
                }
                else if (v < 0)
                {
                    if (trackUnknown)
                    {
                        master.Data[i] = Costs.NoInformation;
                    }
                }
                else if (master.Data[i] == Costs.NoInformation)
                {
                    master.Data[i] = Costs.Free;
                }
            }
        }
    }

    public void Reset()
    {
    }
}

/// <summary>
/// Marks obstacles seen by laser scans and clears the free space between the sensor and each hit,
/// like Nav2's obstacle layer. Marks persist until cleared by a later ray or <see cref="Reset"/>.
/// </summary>
public sealed class ObstacleLayer : ICostmapLayer
{
    private byte[] _marks = [];
    private int _width;
    private int _height;

    public string Name => "obstacle_layer";

    public bool Enabled { get; set; } = true;

    /// <summary>Hits farther than this are ignored for marking.</summary>
    public double ObstacleMaxRange { get; set; } = 2.5;

    /// <summary>Rays clear up to this distance.</summary>
    public double RaytraceMaxRange { get; set; } = 3.0;

    private Costmap2D? _geometry;

    /// <summary>Adds a scan taken at <paramref name="sensorPose"/> (world frame).</summary>
    public void AddScan(Costmap2D costmap, LaserScan scan, Pose2D sensorPose)
    {
        Ensure(costmap);
        if (!costmap.WorldToCell(sensorPose.X, sensorPose.Y, out var sx, out var sy))
        {
            return;
        }

        var angle = (double)scan.AngleMin;
        for (var i = 0; i < scan.Ranges.Length; i++, angle += scan.AngleIncrement)
        {
            var r = (double)scan.Ranges[i];
            var hit = float.IsFinite(scan.Ranges[i]) && r >= scan.RangeMin && r < scan.RangeMax;
            var clearRange = hit ? Math.Min(r, RaytraceMaxRange) : Math.Min(scan.RangeMax, RaytraceMaxRange);
            var a = sensorPose.Theta + angle;
            var ex = sensorPose.X + (clearRange * Math.Cos(a));
            var ey = sensorPose.Y + (clearRange * Math.Sin(a));
            costmap.WorldToCell(ex, ey, out var cx, out var cy);
            cx = Math.Clamp(cx, 0, _width - 1);
            cy = Math.Clamp(cy, 0, _height - 1);
            OccupancyGridMap.RayTrace(sx, sy, cx, cy, (x, y) =>
            {
                _marks[(y * _width) + x] = 0;
                return true;
            });

            if (hit && r <= ObstacleMaxRange)
            {
                var hx = sensorPose.X + (r * Math.Cos(a));
                var hy = sensorPose.Y + (r * Math.Sin(a));
                if (costmap.WorldToCell(hx, hy, out var mx, out var my))
                {
                    _marks[(my * _width) + mx] = 1;
                }
            }
        }
    }

    /// <summary>Marks a single world point as an obstacle (e.g. from a point cloud).</summary>
    public void Mark(Costmap2D costmap, double wx, double wy)
    {
        Ensure(costmap);
        if (costmap.WorldToCell(wx, wy, out var cx, out var cy))
        {
            _marks[(cy * _width) + cx] = 1;
        }
    }

    private void Ensure(Costmap2D costmap)
    {
        if (_geometry != costmap || _marks.Length != costmap.Width * costmap.Height)
        {
            _geometry = costmap;
            _width = costmap.Width;
            _height = costmap.Height;
            _marks = new byte[_width * _height];
        }
    }

    public void UpdateCosts(Costmap2D master)
    {
        if (_marks.Length != master.Data.Length)
        {
            return;
        }

        for (var i = 0; i < _marks.Length; i++)
        {
            if (_marks[i] != 0)
            {
                master.Data[i] = Costs.Lethal;
            }
        }
    }

    public void Reset() => Array.Clear(_marks);
}

/// <summary>
/// Inflates lethal cells: cells within <see cref="InscribedRadius"/> become 253, farther cells decay
/// as <c>252·exp(−cost_scaling_factor·(d − inscribed_radius))</c> out to <see cref="InflationRadius"/>,
/// exactly like Nav2's inflation layer.
/// </summary>
public sealed class InflationLayer(double inflationRadius = 0.55, double costScalingFactor = 3.0, double inscribedRadius = 0.2) : ICostmapLayer
{
    private byte[] _kernel = [];
    private int _kernelRadius = -1;
    private double _kernelResolution;
    private int[] _queue = [];
    private float[] _dist = [];

    public string Name => "inflation_layer";

    public bool Enabled { get; set; } = true;

    public double InflationRadius { get; set; } = inflationRadius;

    public double CostScalingFactor { get; set; } = costScalingFactor;

    public double InscribedRadius { get; set; } = inscribedRadius;

    /// <summary>Cost for a cell <paramref name="distance"/> meters from the nearest obstacle.</summary>
    public byte ComputeCost(double distance)
    {
        if (distance <= 0)
        {
            return Costs.Lethal;
        }

        if (distance <= InscribedRadius)
        {
            return Costs.Inscribed;
        }

        if (distance > InflationRadius)
        {
            return Costs.Free;
        }

        return (byte)((Costs.Inscribed - 1) * Math.Exp(-CostScalingFactor * (distance - InscribedRadius)));
    }

    public void UpdateCosts(Costmap2D master)
    {
        var radiusCells = (int)Math.Ceiling(InflationRadius / master.Resolution);
        BuildKernel(radiusCells, master.Resolution);

        // Brushfire over lethal cells: only cells next to free space need to spread their kernel.
        var w = master.Width;
        var h = master.Height;
        var data = master.Data;
        var n = data.Length;
        if (_dist.Length != n)
        {
            _dist = new float[n];
            _queue = new int[n];
        }

        var count = 0;
        for (var i = 0; i < n; i++)
        {
            if (data[i] != Costs.Lethal)
            {
                continue;
            }

            var x = i % w;
            var y = i / w;
            if ((x > 0 && data[i - 1] != Costs.Lethal) || (x < w - 1 && data[i + 1] != Costs.Lethal) ||
                (y > 0 && data[i - w] != Costs.Lethal) || (y < h - 1 && data[i + w] != Costs.Lethal))
            {
                _queue[count++] = i;
            }
        }

        var k = _kernelRadius;
        var kw = (2 * k) + 1;
        for (var q = 0; q < count; q++)
        {
            var i = _queue[q];
            var ox = i % w;
            var oy = i / w;
            var y0 = Math.Max(0, oy - k);
            var y1 = Math.Min(h - 1, oy + k);
            var x0 = Math.Max(0, ox - k);
            var x1 = Math.Min(w - 1, ox + k);
            for (var y = y0; y <= y1; y++)
            {
                var row = y * w;
                var krow = (y - oy + k) * kw;
                for (var x = x0; x <= x1; x++)
                {
                    var cost = _kernel[krow + (x - ox + k)];
                    var j = row + x;
                    var current = data[j];
                    if (cost > current && current != Costs.Lethal && (current != Costs.NoInformation || cost >= Costs.Inscribed))
                    {
                        data[j] = cost;
                    }
                }
            }
        }
    }

    private void BuildKernel(int radiusCells, double resolution)
    {
        if (radiusCells == _kernelRadius && resolution == _kernelResolution)
        {
            return;
        }

        _kernelRadius = radiusCells;
        _kernelResolution = resolution;
        var kw = (2 * radiusCells) + 1;
        _kernel = new byte[kw * kw];
        for (var dy = -radiusCells; dy <= radiusCells; dy++)
        {
            for (var dx = -radiusCells; dx <= radiusCells; dx++)
            {
                var d = Math.Sqrt((dx * dx) + (dy * dy)) * resolution;
                _kernel[((dy + radiusCells) * kw) + dx + radiusCells] = dx == 0 && dy == 0 ? Costs.Lethal : ComputeCost(d);
            }
        }
    }

    public void Reset()
    {
    }
}
