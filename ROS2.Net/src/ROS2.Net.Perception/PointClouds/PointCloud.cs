using System.Numerics;
using ROS2.Net.Msgs.SensorMsgs;

namespace ROS2.Net.Perception;

/// <summary>
/// An in-memory point cloud. Positions are one interleaved xyz buffer (the layout the native
/// kernels consume directly); optional channels — intensity, packed RGB, normals — are separate
/// arrays, so each channel is contiguous.
/// </summary>
public sealed class PointCloud
{
    private float[] _xyz;
    private float[]? _intensity;
    private uint[]? _rgb;
    private float[]? _normals;

    public PointCloud(int capacity = 0)
    {
        _xyz = capacity > 0 ? new float[capacity * 3] : [];
    }

    /// <summary>Wraps an xyz buffer (3·N floats) without copying.</summary>
    public static PointCloud FromXyz(float[] xyz, int? count = null)
    {
        var c = count ?? xyz.Length / 3;
        if (c * 3 > xyz.Length)
        {
            throw new ArgumentException("Buffer is shorter than 3 × count.", nameof(xyz));
        }

        return new PointCloud { _xyz = xyz, Count = c };
    }

    public static PointCloud FromPoints(IEnumerable<Vector3> points)
    {
        var cloud = new PointCloud();
        foreach (var p in points)
        {
            cloud.Add(p.X, p.Y, p.Z);
        }

        return cloud;
    }

    public int Count { get; private set; }

    public string FrameId { get; set; } = string.Empty;

    public RosTime Stamp { get; set; }

    /// <summary>Interleaved positions of the <see cref="Count"/> points.</summary>
    public Span<float> Xyz => _xyz.AsSpan(0, Count * 3);

    /// <summary>The backing xyz array (may be longer than 3·Count).</summary>
    public float[] XyzBuffer => _xyz;

    public bool HasIntensity => _intensity is not null;

    public bool HasRgb => _rgb is not null;

    public bool HasNormals => _normals is not null;

    /// <summary>Per-point intensity, or empty when absent.</summary>
    public Span<float> Intensity => _intensity is null ? [] : _intensity.AsSpan(0, Count);

    /// <summary>Per-point color packed as 0x00RRGGBB, or empty when absent.</summary>
    public Span<uint> Rgb => _rgb is null ? [] : _rgb.AsSpan(0, Count);

    /// <summary>Per-point unit normals (interleaved xyz), or empty when absent.</summary>
    public Span<float> Normals => _normals is null ? [] : _normals.AsSpan(0, Count * 3);

    public Vector3 this[int index]
    {
        get
        {
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)index, (uint)Count, nameof(index));
            return new Vector3(_xyz[3 * index], _xyz[(3 * index) + 1], _xyz[(3 * index) + 2]);
        }
        set
        {
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)index, (uint)Count, nameof(index));
            _xyz[3 * index] = value.X;
            _xyz[(3 * index) + 1] = value.Y;
            _xyz[(3 * index) + 2] = value.Z;
        }
    }

    /// <summary>Allocates the intensity channel (zeros).</summary>
    public PointCloud WithIntensity()
    {
        _intensity ??= new float[Math.Max(Count, _xyz.Length / 3)];
        return this;
    }

    /// <summary>Allocates the RGB channel (black).</summary>
    public PointCloud WithRgb()
    {
        _rgb ??= new uint[Math.Max(Count, _xyz.Length / 3)];
        return this;
    }

    /// <summary>Sets the normals channel (3·Count floats).</summary>
    public void SetNormals(float[] normals)
    {
        if (normals.Length < Count * 3)
        {
            throw new ArgumentException("Normals must hold 3 floats per point.", nameof(normals));
        }

        _normals = normals;
    }

    public void Add(float x, float y, float z, float intensity = 0, uint rgb = 0)
    {
        EnsureCapacity(Count + 1);
        var i = Count++;
        _xyz[3 * i] = x;
        _xyz[(3 * i) + 1] = y;
        _xyz[(3 * i) + 2] = z;
        if (_intensity is not null)
        {
            _intensity[i] = intensity;
        }

        if (_rgb is not null)
        {
            _rgb[i] = rgb;
        }
    }

    public void Add(Vector3 p) => Add(p.X, p.Y, p.Z);

    public void Clear() => Count = 0;

    private void EnsureCapacity(int count)
    {
        if (count * 3 <= _xyz.Length)
        {
            return;
        }

        var cap = Math.Max(count, Math.Max(16, _xyz.Length / 3 * 2));
        Array.Resize(ref _xyz, cap * 3);
        if (_intensity is not null)
        {
            Array.Resize(ref _intensity, cap);
        }

        if (_rgb is not null)
        {
            Array.Resize(ref _rgb, cap);
        }

        if (_normals is not null)
        {
            Array.Resize(ref _normals, cap * 3);
        }
    }

    /// <summary>The points whose mask entry is non-zero, with all channels.</summary>
    public PointCloud Select(ReadOnlySpan<byte> mask)
    {
        var indices = new List<int>(Count);
        for (var i = 0; i < Count && i < mask.Length; i++)
        {
            if (mask[i] != 0)
            {
                indices.Add(i);
            }
        }

        return Select(indices);
    }

    /// <summary>The points at <paramref name="indices"/>, with all channels.</summary>
    public PointCloud Select(IReadOnlyList<int> indices)
    {
        var result = new PointCloud(indices.Count) { FrameId = FrameId, Stamp = Stamp };
        if (_intensity is not null)
        {
            result.WithIntensity();
        }

        if (_rgb is not null)
        {
            result.WithRgb();
        }

        float[]? normals = _normals is null ? null : new float[indices.Count * 3];
        for (var k = 0; k < indices.Count; k++)
        {
            var i = indices[k];
            result.Add(_xyz[3 * i], _xyz[(3 * i) + 1], _xyz[(3 * i) + 2], _intensity?[i] ?? 0, _rgb?[i] ?? 0);
            if (normals is not null)
            {
                normals[3 * k] = _normals![3 * i];
                normals[(3 * k) + 1] = _normals[(3 * i) + 1];
                normals[(3 * k) + 2] = _normals[(3 * i) + 2];
            }
        }

        if (normals is not null)
        {
            result._normals = normals;
        }

        return result;
    }

    /// <summary>A deep copy.</summary>
    public PointCloud Clone() => Select(Enumerable.Range(0, Count).ToArray());

    /// <summary>Applies a row-major 4×4 rigid transform in place (normals are rotated).</summary>
    public PointCloud Transform(ReadOnlySpan<double> m)
    {
        if (m.Length < 12)
        {
            throw new ArgumentException("Expected a row-major 4×4 matrix.", nameof(m));
        }

        for (var i = 0; i < Count; i++)
        {
            double x = _xyz[3 * i], y = _xyz[(3 * i) + 1], z = _xyz[(3 * i) + 2];
            _xyz[3 * i] = (float)((m[0] * x) + (m[1] * y) + (m[2] * z) + m[3]);
            _xyz[(3 * i) + 1] = (float)((m[4] * x) + (m[5] * y) + (m[6] * z) + m[7]);
            _xyz[(3 * i) + 2] = (float)((m[8] * x) + (m[9] * y) + (m[10] * z) + m[11]);
            if (_normals is not null)
            {
                double nx = _normals[3 * i], ny = _normals[(3 * i) + 1], nz = _normals[(3 * i) + 2];
                _normals[3 * i] = (float)((m[0] * nx) + (m[1] * ny) + (m[2] * nz));
                _normals[(3 * i) + 1] = (float)((m[4] * nx) + (m[5] * ny) + (m[6] * nz));
                _normals[(3 * i) + 2] = (float)((m[8] * nx) + (m[9] * ny) + (m[10] * nz));
            }
        }

        return this;
    }

    /// <summary>Applies a System.Numerics transform in place (row-vector convention: p' = p · M).</summary>
    public PointCloud Transform(Matrix4x4 m)
    {
        for (var i = 0; i < Count; i++)
        {
            var p = Vector3.Transform(this[i], m);
            this[i] = p;
        }

        return this;
    }

    /// <summary>Appends another cloud (channels this cloud lacks are dropped).</summary>
    public void AddRange(PointCloud other)
    {
        for (var i = 0; i < other.Count; i++)
        {
            Add(other._xyz[3 * i], other._xyz[(3 * i) + 1], other._xyz[(3 * i) + 2], other._intensity?[i] ?? 0, other._rgb?[i] ?? 0);
        }
    }

    public Vector3 Centroid()
    {
        double x = 0, y = 0, z = 0;
        var n = 0;
        for (var i = 0; i < Count; i++)
        {
            if (float.IsFinite(_xyz[3 * i]) && float.IsFinite(_xyz[(3 * i) + 1]) && float.IsFinite(_xyz[(3 * i) + 2]))
            {
                x += _xyz[3 * i];
                y += _xyz[(3 * i) + 1];
                z += _xyz[(3 * i) + 2];
                n++;
            }
        }

        return n == 0 ? Vector3.Zero : new Vector3((float)(x / n), (float)(y / n), (float)(z / n));
    }

    /// <summary>Reads a <c>sensor_msgs/PointCloud2</c> (x/y/z required; intensity and rgb/rgba picked up when present).</summary>
    public static PointCloud FromMessage(PointCloud2 message, bool skipNonFinite = false) => PointCloud2Reader.ToPointCloud(message, skipNonFinite);

    /// <summary>Writes a <c>sensor_msgs/PointCloud2</c> with float32 x/y/z and the channels this cloud has.</summary>
    public PointCloud2 ToMessage(string? frameId = null, RosTime? stamp = null) => PointCloud2Writer.FromPointCloud(this, frameId, stamp);
}
