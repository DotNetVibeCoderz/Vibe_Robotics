namespace ROS2.Net.Perception.Images;

/// <summary>Element type of an image channel.</summary>
public enum ChannelDepth
{
    U8,
    U16,
    F32,
}

/// <summary>The <c>sensor_msgs/image_encodings</c> names supported by ROS2.Net, and their layout.</summary>
public static class ImageEncodings
{
    public const string Rgb8 = "rgb8";
    public const string Bgr8 = "bgr8";
    public const string Rgba8 = "rgba8";
    public const string Bgra8 = "bgra8";
    public const string Mono8 = "mono8";
    public const string Mono16 = "mono16";
    public const string Type32FC1 = "32FC1";
    public const string Type16UC1 = "16UC1";
    public const string Type8UC1 = "8UC1";
    public const string Type8UC3 = "8UC3";

    /// <summary>UYVY 4:2:2 (ROS "yuv422"): two pixels in four bytes U0 Y0 V0 Y1.</summary>
    public const string Yuv422 = "yuv422";

    /// <summary>YUYV 4:2:2: Y0 U0 Y1 V0.</summary>
    public const string Yuv422Yuy2 = "yuv422_yuy2";

    public static bool IsSupported(string encoding) => TryGetLayout(encoding, out _, out _);

    /// <summary>Channels and element depth of an encoding. YUV formats report 2 U8 channels per pixel.</summary>
    public static bool TryGetLayout(string encoding, out int channels, out ChannelDepth depth)
    {
        (channels, depth) = encoding switch
        {
            Rgb8 or Bgr8 or Type8UC3 => (3, ChannelDepth.U8),
            Rgba8 or Bgra8 => (4, ChannelDepth.U8),
            Mono8 or Type8UC1 => (1, ChannelDepth.U8),
            Mono16 or Type16UC1 => (1, ChannelDepth.U16),
            Type32FC1 => (1, ChannelDepth.F32),
            Yuv422 or Yuv422Yuy2 => (2, ChannelDepth.U8),
            _ => (0, ChannelDepth.U8),
        };
        return channels > 0;
    }

    public static int Channels(string encoding) =>
        TryGetLayout(encoding, out var c, out _) ? c : throw new NotSupportedException($"Image encoding '{encoding}' is not supported.");

    public static int BytesPerElement(ChannelDepth depth) => depth switch
    {
        ChannelDepth.U8 => 1,
        ChannelDepth.U16 => 2,
        _ => 4,
    };

    /// <summary>Bytes per pixel.</summary>
    public static int BytesPerPixel(string encoding) =>
        TryGetLayout(encoding, out var c, out var d) ? c * BytesPerElement(d) : throw new NotSupportedException($"Image encoding '{encoding}' is not supported.");

    public static bool IsColor(string encoding) => encoding is Rgb8 or Bgr8 or Rgba8 or Bgra8 or Yuv422 or Yuv422Yuy2;

    public static bool HasAlpha(string encoding) => encoding is Rgba8 or Bgra8;

    public static bool IsDepth(string encoding) => encoding is Type16UC1 or Type32FC1;
}
