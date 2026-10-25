using System.Globalization;
using System.Text;
using ROS2.Net.Msgs.NavMsgs;
using Path = System.IO.Path;
using YamlDotNet.RepresentationModel;

namespace ROS2.Net.Navigation.Maps;

/// <summary>
/// A 2-D occupancy grid with ROS semantics: row-major cells starting at the origin, values -1
/// (unknown) or 0..100 (occupancy probability in percent).
/// </summary>
public sealed class OccupancyGridMap
{
    public const sbyte Unknown = -1;
    public const sbyte Free = 0;
    public const sbyte Occupied = 100;

    public OccupancyGridMap(int width, int height, double resolution, Pose2D origin = default, sbyte fill = Unknown)
    {
        if (width <= 0 || height <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(width), "Map dimensions must be positive.");
        }

        if (resolution <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(resolution), "Resolution must be positive.");
        }

        Width = width;
        Height = height;
        Resolution = resolution;
        Origin = origin;
        Data = new sbyte[width * height];
        if (fill != 0)
        {
            Data.AsSpan().Fill(fill);
        }
    }

    public int Width { get; }

    public int Height { get; }

    /// <summary>Meters per cell.</summary>
    public double Resolution { get; }

    /// <summary>World pose of cell (0, 0)'s lower-left corner.</summary>
    public Pose2D Origin { get; }

    /// <summary>Row-major cell values.</summary>
    public sbyte[] Data { get; }

    /// <summary>Cells at or above this value count as occupied (map_server's occupied_thresh 0.65).</summary>
    public sbyte OccupiedThreshold { get; set; } = 65;

    /// <summary>Cells at or below this value count as free (free_thresh 0.196).</summary>
    public sbyte FreeThreshold { get; set; } = 19;

    public double WorldWidth => Width * Resolution;

    public double WorldHeight => Height * Resolution;

    public int Index(int cx, int cy) => (cy * Width) + cx;

    public bool InBounds(int cx, int cy) => (uint)cx < (uint)Width && (uint)cy < (uint)Height;

    public sbyte this[int cx, int cy]
    {
        get => Data[(cy * Width) + cx];
        set => Data[(cy * Width) + cx] = value;
    }

    /// <summary>Converts world coordinates to a cell. Returns false outside the map.</summary>
    public bool WorldToCell(double wx, double wy, out int cx, out int cy)
    {
        var (lx, ly) = ToLocal(wx, wy);
        cx = (int)Math.Floor(lx / Resolution);
        cy = (int)Math.Floor(ly / Resolution);
        return InBounds(cx, cy);
    }

    /// <summary>World coordinates of a cell's center.</summary>
    public (double X, double Y) CellToWorld(int cx, int cy) =>
        Origin.TransformPoint((cx + 0.5) * Resolution, (cy + 0.5) * Resolution);

    private (double X, double Y) ToLocal(double wx, double wy)
    {
        if (Origin.Theta == 0)
        {
            return (wx - Origin.X, wy - Origin.Y);
        }

        var c = Math.Cos(Origin.Theta);
        var s = Math.Sin(Origin.Theta);
        var dx = wx - Origin.X;
        var dy = wy - Origin.Y;
        return ((c * dx) + (s * dy), (-s * dx) + (c * dy));
    }

    public bool IsOccupied(int cx, int cy) => Data[Index(cx, cy)] >= OccupiedThreshold;

    public bool IsFree(int cx, int cy)
    {
        var v = Data[Index(cx, cy)];
        return v >= 0 && v <= FreeThreshold;
    }

    /// <summary>Marks a world-space axis-aligned rectangle with a value (handy for building test maps).</summary>
    public void FillRect(double x0, double y0, double x1, double y1, sbyte value)
    {
        WorldToCellClamped(Math.Min(x0, x1), Math.Min(y0, y1), out var ax, out var ay);
        WorldToCellClamped(Math.Max(x0, x1), Math.Max(y0, y1), out var bx, out var by);
        for (var y = ay; y <= by; y++)
        {
            Data.AsSpan(Index(ax, y), bx - ax + 1).Fill(value);
        }
    }

    private void WorldToCellClamped(double wx, double wy, out int cx, out int cy)
    {
        WorldToCell(wx, wy, out cx, out cy);
        cx = Math.Clamp(cx, 0, Width - 1);
        cy = Math.Clamp(cy, 0, Height - 1);
    }

    /// <summary>Visits the cells on the Bresenham line between two cells (inclusive). Stops early when <paramref name="visit"/> returns false.</summary>
    public static void RayTrace(int x0, int y0, int x1, int y1, Func<int, int, bool> visit)
    {
        int dx = Math.Abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -Math.Abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        var err = dx + dy;
        while (true)
        {
            if (!visit(x0, y0))
            {
                return;
            }

            if (x0 == x1 && y0 == y1)
            {
                return;
            }

            var e2 = 2 * err;
            if (e2 >= dy)
            {
                err += dy;
                x0 += sx;
            }

            if (e2 <= dx)
            {
                err += dx;
                y0 += sy;
            }
        }
    }

    /// <summary>
    /// Casts a ray from a world point and returns the distance to the first occupied (or unknown,
    /// when <paramref name="unknownIsObstacle"/>) cell, or <paramref name="maxRange"/>.
    /// </summary>
    public double Raycast(double wx, double wy, double angle, double maxRange, bool unknownIsObstacle = false)
    {
        // DDA traversal: exact cell boundaries, so ranges are accurate to the cell edge.
        var (lx, ly) = ToLocal(wx, wy);
        var a = angle - Origin.Theta;
        var dirX = Math.Cos(a);
        var dirY = Math.Sin(a);
        var px = lx / Resolution;
        var py = ly / Resolution;
        var cx = (int)Math.Floor(px);
        var cy = (int)Math.Floor(py);
        var stepX = dirX >= 0 ? 1 : -1;
        var stepY = dirY >= 0 ? 1 : -1;
        var tDeltaX = dirX == 0 ? double.PositiveInfinity : Math.Abs(1 / dirX);
        var tDeltaY = dirY == 0 ? double.PositiveInfinity : Math.Abs(1 / dirY);
        var tMaxX = dirX == 0 ? double.PositiveInfinity : ((stepX > 0 ? cx + 1 - px : px - cx) * tDeltaX);
        var tMaxY = dirY == 0 ? double.PositiveInfinity : ((stepY > 0 ? cy + 1 - py : py - cy) * tDeltaY);
        var maxCells = maxRange / Resolution;
        var t = 0.0;
        while (t <= maxCells)
        {
            if (!InBounds(cx, cy))
            {
                return maxRange;
            }

            var v = Data[(cy * Width) + cx];
            if (v >= OccupiedThreshold || (unknownIsObstacle && v < 0))
            {
                return Math.Min(maxRange, t * Resolution);
            }

            if (tMaxX < tMaxY)
            {
                t = tMaxX;
                tMaxX += tDeltaX;
                cx += stepX;
            }
            else
            {
                t = tMaxY;
                tMaxY += tDeltaY;
                cy += stepY;
            }
        }

        return maxRange;
    }

    public OccupancyGridMap Clone()
    {
        var copy = new OccupancyGridMap(Width, Height, Resolution, Origin, 0)
        {
            OccupiedThreshold = OccupiedThreshold,
            FreeThreshold = FreeThreshold,
        };
        Data.CopyTo(copy.Data, 0);
        return copy;
    }

    // ------------------------------------------------------------------ messages

    public static OccupancyGridMap FromMsg(OccupancyGrid msg)
    {
        var map = new OccupancyGridMap((int)msg.Info.Width, (int)msg.Info.Height, msg.Info.Resolution, Pose2D.FromMsg(msg.Info.Origin), 0);
        msg.Data.AsSpan(0, Math.Min(msg.Data.Length, map.Data.Length)).CopyTo(map.Data);
        return map;
    }

    public OccupancyGrid ToMsg(string frameId = "map", RosTime stamp = default)
    {
        var msg = new OccupancyGrid
        {
            Info = new MapMetaData
            {
                Resolution = (float)Resolution,
                Width = (uint)Width,
                Height = (uint)Height,
                Origin = Origin.ToMsg(),
            },
            Data = (sbyte[])Data.Clone(),
        };
        stamp.CopyTo(msg.Header.Stamp);
        stamp.CopyTo(msg.Info.MapLoadTime);
        msg.Header.FrameId = frameId;
        return msg;
    }

    // ------------------------------------------------------------------ map_server files

    /// <summary>Loads a map_server map: a YAML file referencing a PGM (P5/P2) image.</summary>
    public static OccupancyGridMap LoadMapServer(string yamlPath)
    {
        var yaml = new YamlStream();
        yaml.Load(new StringReader(File.ReadAllText(yamlPath)));
        var root = (YamlMappingNode)yaml.Documents[0].RootNode;
        string Scalar(string key, string fallback = "") =>
            root.Children.TryGetValue(new YamlScalarNode(key), out var n) && n is YamlScalarNode s ? s.Value ?? fallback : fallback;

        var image = Scalar("image");
        var resolution = double.Parse(Scalar("resolution", "0.05"), CultureInfo.InvariantCulture);
        var negate = Scalar("negate", "0") is "1" or "true";
        var occThresh = double.Parse(Scalar("occupied_thresh", "0.65"), CultureInfo.InvariantCulture);
        var freeThresh = double.Parse(Scalar("free_thresh", "0.196"), CultureInfo.InvariantCulture);
        var mode = Scalar("mode", "trinary");
        var origin = Pose2D.Identity;
        if (root.Children.TryGetValue(new YamlScalarNode("origin"), out var o) && o is YamlSequenceNode seq && seq.Children.Count >= 2)
        {
            double V(int i) => i < seq.Children.Count ? double.Parse(((YamlScalarNode)seq.Children[i]).Value!, CultureInfo.InvariantCulture) : 0;
            origin = new Pose2D(V(0), V(1), V(2));
        }

        var imagePath = Path.IsPathRooted(image) ? image : Path.Combine(Path.GetDirectoryName(Path.GetFullPath(yamlPath))!, image);
        var (w, h, maxVal, pixels) = ReadPgm(imagePath);
        var map = new OccupancyGridMap(w, h, resolution, origin, 0)
        {
            OccupiedThreshold = (sbyte)Math.Round(occThresh * 100),
            FreeThreshold = (sbyte)Math.Round(freeThresh * 100),
        };
        for (var row = 0; row < h; row++)
        {
            var my = h - 1 - row; // image row 0 is the top of the map
            for (var col = 0; col < w; col++)
            {
                var pixel = pixels[(row * w) + col] / (double)maxVal;
                var occ = negate ? pixel : 1.0 - pixel;
                sbyte value;
                if (occ > occThresh)
                {
                    value = Occupied;
                }
                else if (occ < freeThresh)
                {
                    value = Free;
                }
                else
                {
                    value = mode == "scale" ? (sbyte)Math.Round(occ * 100) : Unknown;
                }

                map.Data[(my * w) + col] = value;
            }
        }

        return map;
    }

    /// <summary>Saves the map like <c>map_saver_cli</c> (trinary): <c>&lt;basePath&gt;.yaml</c> + <c>&lt;basePath&gt;.pgm</c>.</summary>
    public void SaveMapServer(string basePath)
    {
        var dir = Path.GetDirectoryName(Path.GetFullPath(basePath))!;
        Directory.CreateDirectory(dir);
        var pgm = basePath + ".pgm";
        using (var fs = File.Create(pgm))
        {
            var header = Encoding.ASCII.GetBytes($"P5\n# CREATOR: ROS2.Net {Resolution:0.000} m/pix\n{Width} {Height}\n255\n");
            fs.Write(header);
            var row = new byte[Width];
            for (var r = 0; r < Height; r++)
            {
                var my = Height - 1 - r;
                for (var x = 0; x < Width; x++)
                {
                    var v = Data[(my * Width) + x];
                    row[x] = v < 0 ? (byte)205 : v >= OccupiedThreshold ? (byte)0 : v <= FreeThreshold ? (byte)254 : (byte)205;
                }

                fs.Write(row);
            }
        }

        var inv = CultureInfo.InvariantCulture;
        File.WriteAllText(basePath + ".yaml",
            $"image: {Path.GetFileName(pgm)}\n" +
            "mode: trinary\n" +
            $"resolution: {Resolution.ToString("0.###", inv)}\n" +
            $"origin: [{Origin.X.ToString("R", inv)}, {Origin.Y.ToString("R", inv)}, {Origin.Theta.ToString("R", inv)}]\n" +
            "negate: 0\n" +
            $"occupied_thresh: {(OccupiedThreshold / 100.0).ToString("0.###", inv)}\n" +
            $"free_thresh: {(Math.Max(FreeThreshold, (sbyte)1) / 100.0).ToString("0.###", inv)}\n");
    }

    private static (int Width, int Height, int MaxValue, int[] Pixels) ReadPgm(string path)
    {
        var bytes = File.ReadAllBytes(path);
        var pos = 0;
        string Token()
        {
            while (pos < bytes.Length)
            {
                if (bytes[pos] == '#')
                {
                    while (pos < bytes.Length && bytes[pos] != '\n')
                    {
                        pos++;
                    }
                }
                else if (char.IsWhiteSpace((char)bytes[pos]))
                {
                    pos++;
                }
                else
                {
                    break;
                }
            }

            var start = pos;
            while (pos < bytes.Length && !char.IsWhiteSpace((char)bytes[pos]))
            {
                pos++;
            }

            return Encoding.ASCII.GetString(bytes, start, pos - start);
        }

        var magic = Token();
        var w = int.Parse(Token(), CultureInfo.InvariantCulture);
        var h = int.Parse(Token(), CultureInfo.InvariantCulture);
        var max = int.Parse(Token(), CultureInfo.InvariantCulture);
        var pixels = new int[w * h];
        if (magic == "P5")
        {
            pos++; // single whitespace after maxval
            var wide = max > 255;
            for (var i = 0; i < pixels.Length; i++)
            {
                pixels[i] = wide ? (bytes[pos + (2 * i)] << 8) | bytes[pos + (2 * i) + 1] : bytes[pos + i];
            }
        }
        else if (magic == "P2")
        {
            for (var i = 0; i < pixels.Length; i++)
            {
                pixels[i] = int.Parse(Token(), CultureInfo.InvariantCulture);
            }
        }
        else
        {
            throw new InvalidDataException($"'{path}' is not a PGM image (magic '{magic}').");
        }

        return (w, h, max, pixels);
    }
}
