using System.Buffers.Binary;
using ROS2.Net.Msgs.SensorMsgs;

namespace ROS2.Net.Perception.Images;

/// <summary>
/// Typed, zero-copy access to the pixels of a <c>sensor_msgs/Image</c>: honors <c>step</c> padding
/// and <c>is_bigendian</c>. Element values are read as floats in their native range (0–255, 0–65535,
/// or the float value).
/// </summary>
public readonly struct ImageView
{
    public ImageView(Image image)
    {
        Image = image ?? throw new ArgumentNullException(nameof(image));
        if (!ImageEncodings.TryGetLayout(image.Encoding, out var c, out var d))
        {
            throw new NotSupportedException($"Image encoding '{image.Encoding}' is not supported.");
        }

        Channels = c;
        Depth = d;
        ElementSize = ImageEncodings.BytesPerElement(d);
        if (image.Step < image.Width * c * ElementSize || image.Data.Length < (long)image.Step * image.Height)
        {
            throw new ArgumentException($"Image data ({image.Data.Length} bytes, step {image.Step}) is too small for {image.Width}×{image.Height} {image.Encoding}.", nameof(image));
        }
    }

    public Image Image { get; }

    public int Width => (int)Image.Width;

    public int Height => (int)Image.Height;

    public int Step => (int)Image.Step;

    public string Encoding => Image.Encoding;

    public int Channels { get; }

    public ChannelDepth Depth { get; }

    public int ElementSize { get; }

    private bool BigEndian => Image.IsBigendian != 0;

    /// <summary>Raw bytes of row <paramref name="y"/> (without padding).</summary>
    public ReadOnlySpan<byte> Row(int y) => Image.Data.AsSpan(y * Step, Width * Channels * ElementSize);

    /// <summary>Channel <paramref name="c"/> of pixel (x, y).</summary>
    public float Get(int x, int y, int c = 0)
    {
        var offset = (y * Step) + (((x * Channels) + c) * ElementSize);
        var s = Image.Data.AsSpan(offset);
        return Depth switch
        {
            ChannelDepth.U8 => s[0],
            ChannelDepth.U16 => BigEndian ? BinaryPrimitives.ReadUInt16BigEndian(s) : BinaryPrimitives.ReadUInt16LittleEndian(s),
            _ => BigEndian ? BinaryPrimitives.ReadSingleBigEndian(s) : BinaryPrimitives.ReadSingleLittleEndian(s),
        };
    }

    /// <summary>Writes channel <paramref name="c"/> of pixel (x, y), saturating integer types.</summary>
    public void Set(int x, int y, int c, float value)
    {
        var offset = (y * Step) + (((x * Channels) + c) * ElementSize);
        var s = Image.Data.AsSpan(offset);
        switch (Depth)
        {
            case ChannelDepth.U8:
                s[0] = (byte)Math.Clamp(MathF.Round(value), 0, 255);
                break;
            case ChannelDepth.U16:
                var u = (ushort)Math.Clamp(MathF.Round(value), 0, 65535);
                if (BigEndian)
                {
                    BinaryPrimitives.WriteUInt16BigEndian(s, u);
                }
                else
                {
                    BinaryPrimitives.WriteUInt16LittleEndian(s, u);
                }

                break;
            default:
                if (BigEndian)
                {
                    BinaryPrimitives.WriteSingleBigEndian(s, value);
                }
                else
                {
                    BinaryPrimitives.WriteSingleLittleEndian(s, value);
                }

                break;
        }
    }

    /// <summary>The pixel as RGB bytes, whatever the encoding (mono is replicated, 16-bit is scaled).</summary>
    public (byte R, byte G, byte B) GetRgb(int x, int y)
    {
        switch (Encoding)
        {
            case ImageEncodings.Rgb8 or ImageEncodings.Rgba8 or ImageEncodings.Type8UC3:
                return ((byte)Get(x, y, 0), (byte)Get(x, y, 1), (byte)Get(x, y, 2));
            case ImageEncodings.Bgr8 or ImageEncodings.Bgra8:
                return ((byte)Get(x, y, 2), (byte)Get(x, y, 1), (byte)Get(x, y, 0));
            case ImageEncodings.Yuv422 or ImageEncodings.Yuv422Yuy2:
                return YuvPixel(x, y);
            default:
                var g = Gray(x, y);
                var b = (byte)Math.Clamp(MathF.Round(g), 0, 255);
                return (b, b, b);
        }
    }

    /// <summary>Luma in 0–255 (16-bit and float images are scaled from their full range / 0–1).</summary>
    public float Gray(int x, int y)
    {
        switch (Encoding)
        {
            case ImageEncodings.Mono8 or ImageEncodings.Type8UC1:
                return Get(x, y);
            case ImageEncodings.Mono16 or ImageEncodings.Type16UC1:
                return Get(x, y) / 257f;
            case ImageEncodings.Type32FC1:
                return Get(x, y) * 255f;
            default:
                var (r, g, b) = GetRgb(x, y);
                return (0.299f * r) + (0.587f * g) + (0.114f * b);
        }
    }

    private (byte, byte, byte) YuvPixel(int x, int y)
    {
        var pair = x & ~1;
        var row = Image.Data.AsSpan(y * Step);
        var q = row.Slice(pair * 2, 4);
        float yv, u, v;
        if (Encoding == ImageEncodings.Yuv422)
        {
            u = q[0];
            v = q[2];
            yv = (x & 1) == 0 ? q[1] : q[3];
        }
        else
        {
            u = q[1];
            v = q[3];
            yv = (x & 1) == 0 ? q[0] : q[2];
        }

        var c = yv - 16;
        var d = u - 128;
        var e = v - 128;
        static byte Clamp(float f) => (byte)Math.Clamp(MathF.Round(f), 0, 255);
        return (Clamp((1.164f * c) + (1.596f * e)), Clamp((1.164f * c) - (0.392f * d) - (0.813f * e)), Clamp((1.164f * c) + (2.017f * d)));
    }
}
