using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Navigation.Maps;

namespace ROS2.Net.Navigation.Costmaps;

/// <summary>Cost values, as in <c>nav2_costmap_2d</c>.</summary>
public static class Costs
{
    public const byte Free = 0;

    /// <summary>The robot's center here means its footprint touches an obstacle.</summary>
    public const byte Inscribed = 253;

    public const byte Lethal = 254;

    public const byte NoInformation = 255;

    /// <summary>Highest cost of an inflated, still traversable cell.</summary>
    public const byte MaxNonObstacle = 252;
}

/// <summary>A layer that writes into a master costmap.</summary>
public interface ICostmapLayer
{
    string Name { get; }

    bool Enabled { get; set; }

    /// <summary>Writes this layer's costs into <paramref name="master"/>.</summary>
    void UpdateCosts(Costmap2D master);

    /// <summary>Forgets dynamic information (obstacle marks).</summary>
    void Reset();
}

/// <summary>
/// A layered 2-D costmap: a byte grid of traversal costs (0 free … 252 inflated, 253 inscribed,
/// 254 lethal, 255 unknown) rebuilt from its layers on <see cref="Update"/>.
/// </summary>
public sealed class Costmap2D
{
    private readonly List<ICostmapLayer> _layers = [];
    private readonly object _gate = new();

    public Costmap2D(int width, int height, double resolution, Pose2D origin = default, byte defaultCost = Costs.Free)
    {
        Width = width;
        Height = height;
        Resolution = resolution;
        Origin = origin;
        DefaultCost = defaultCost;
        Data = new byte[width * height];
        Data.AsSpan().Fill(defaultCost);
    }

    /// <summary>A costmap the size of <paramref name="map"/>.</summary>
    public static Costmap2D FromMap(OccupancyGridMap map, bool trackUnknown = false) =>
        new(map.Width, map.Height, map.Resolution, map.Origin, trackUnknown ? Costs.NoInformation : Costs.Free);

    public int Width { get; }

    public int Height { get; }

    public double Resolution { get; }

    public Pose2D Origin { get; }

    public byte DefaultCost { get; }

    public byte[] Data { get; }

    public IReadOnlyList<ICostmapLayer> Layers => _layers;

    /// <summary>Synchronizes readers (planners/controllers) against <see cref="Update"/>.</summary>
    public object SyncRoot => _gate;

    public Costmap2D AddLayer(ICostmapLayer layer)
    {
        _layers.Add(layer);
        return this;
    }

    public T? GetLayer<T>()
        where T : class, ICostmapLayer => _layers.OfType<T>().FirstOrDefault();

    /// <summary>Rebuilds the master grid from every enabled layer, in order.</summary>
    public void Update()
    {
        lock (_gate)
        {
            Data.AsSpan().Fill(DefaultCost);
            foreach (var layer in _layers)
            {
                if (layer.Enabled)
                {
                    layer.UpdateCosts(this);
                }
            }
        }
    }

    /// <summary>Clears dynamic layers (Nav2's <c>clear_entirely</c>) and rebuilds.</summary>
    public void ClearDynamic()
    {
        foreach (var l in _layers)
        {
            l.Reset();
        }

        Update();
    }

    public int Index(int cx, int cy) => (cy * Width) + cx;

    public bool InBounds(int cx, int cy) => (uint)cx < (uint)Width && (uint)cy < (uint)Height;

    public byte GetCost(int cx, int cy) => Data[(cy * Width) + cx];

    public void SetCost(int cx, int cy, byte cost) => Data[(cy * Width) + cx] = cost;

    public bool WorldToCell(double wx, double wy, out int cx, out int cy)
    {
        double lx = wx - Origin.X, ly = wy - Origin.Y;
        if (Origin.Theta != 0)
        {
            var c = Math.Cos(Origin.Theta);
            var s = Math.Sin(Origin.Theta);
            (lx, ly) = ((c * lx) + (s * ly), (-s * lx) + (c * ly));
        }

        cx = (int)Math.Floor(lx / Resolution);
        cy = (int)Math.Floor(ly / Resolution);
        return InBounds(cx, cy);
    }

    public (double X, double Y) CellToWorld(int cx, int cy) =>
        Origin.TransformPoint((cx + 0.5) * Resolution, (cy + 0.5) * Resolution);

    /// <summary>Cost at a world point; <see cref="Costs.Lethal"/> outside the map.</summary>
    public byte GetCostAt(double wx, double wy) =>
        WorldToCell(wx, wy, out var cx, out var cy) ? Data[(cy * Width) + cx] : Costs.Lethal;

    /// <summary>Converts to <c>nav_msgs/OccupancyGrid</c> with Nav2's cost translation (254→100, 255→-1, 1..252→1..98).</summary>
    public OccupancyGrid ToMsg(string frameId = "map", RosTime stamp = default)
    {
        var msg = new OccupancyGrid
        {
            Info = new MapMetaData { Resolution = (float)Resolution, Width = (uint)Width, Height = (uint)Height, Origin = Origin.ToMsg() },
            Data = new sbyte[Data.Length],
        };
        stamp.CopyTo(msg.Header.Stamp);
        msg.Header.FrameId = frameId;
        for (var i = 0; i < Data.Length; i++)
        {
            msg.Data[i] = Data[i] switch
            {
                Costs.Free => 0,
                Costs.NoInformation => -1,
                Costs.Lethal => 100,
                Costs.Inscribed => 99,
                var c => (sbyte)(1 + (97 * (c - 1) / 251)),
            };
        }

        return msg;
    }
}
