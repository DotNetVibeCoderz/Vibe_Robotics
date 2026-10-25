using Avalonia;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using Image = ROS2.Net.Msgs.SensorMsgs.Image;

namespace ROS2.Net.Avalonia.Controls;

/// <summary>
/// Shows a <c>sensor_msgs/Image</c>: rgb8, bgr8, rgba8, bgra8, mono8, mono16, 16UC1 and 32FC1 (depth
/// normalised to its range). The bitmap is reused between frames of the same size.
/// </summary>
public sealed class ImageView : RosDrawnControl
{
    private WriteableBitmap? _bitmap;
    private string _info = string.Empty;
    private IDisposable? _sub;

    public ImageView() => Title = "Camera";

    public Stretch Stretch { get; set; } = Stretch.Uniform;

    /// <summary>Displays an image (call on the UI thread).</summary>
    public void SetImage(Image image)
    {
        Convert(image);
        InvalidateVisual();
    }

    public void Attach(Node node, string topic)
    {
        _sub?.Dispose();
        _sub = RosUi.SubscribeLatest<Image>(node, topic, SetImage, QosProfile.SensorData);
    }

    private unsafe void Convert(Image img)
    {
        int w = (int)img.Width, h = (int)img.Height;
        if (w == 0 || h == 0)
        {
            return;
        }

        if (_bitmap is null || _bitmap.PixelSize.Width != w || _bitmap.PixelSize.Height != h)
        {
            _bitmap = new WriteableBitmap(new PixelSize(w, h), new Vector(96, 96), PixelFormat.Bgra8888, AlphaFormat.Premul);
        }

        _info = $"{w}×{h} {img.Encoding}";
        var step = (int)img.Step;
        var data = img.Data;
        var enc = img.Encoding;

        // Depth range for float/16-bit images.
        float dMin = float.MaxValue, dMax = float.MinValue;
        if (enc is "32FC1" or "16UC1" or "mono16")
        {
            for (var y = 0; y < h; y++)
            {
                for (var x = 0; x < w; x++)
                {
                    var v = Sample(data, step, x, y, enc, img.IsBigendian != 0);
                    if (float.IsFinite(v) && v > 0)
                    {
                        dMin = Math.Min(dMin, v);
                        dMax = Math.Max(dMax, v);
                    }
                }
            }
        }

        using var fb = _bitmap.Lock();
        for (var y = 0; y < h; y++)
        {
            var row = new Span<uint>((byte*)fb.Address + (y * fb.RowBytes), w);
            var src = y * step;
            for (var x = 0; x < w; x++)
            {
                byte r, g, b, a = 255;
                switch (enc)
                {
                    case "rgb8" or "8UC3":
                        r = At(data, src + (x * 3));
                        g = At(data, src + (x * 3) + 1);
                        b = At(data, src + (x * 3) + 2);
                        break;
                    case "bgr8":
                        b = At(data, src + (x * 3));
                        g = At(data, src + (x * 3) + 1);
                        r = At(data, src + (x * 3) + 2);
                        break;
                    case "rgba8":
                        r = At(data, src + (x * 4));
                        g = At(data, src + (x * 4) + 1);
                        b = At(data, src + (x * 4) + 2);
                        a = At(data, src + (x * 4) + 3);
                        break;
                    case "bgra8":
                        b = At(data, src + (x * 4));
                        g = At(data, src + (x * 4) + 1);
                        r = At(data, src + (x * 4) + 2);
                        a = At(data, src + (x * 4) + 3);
                        break;
                    case "mono8" or "8UC1":
                        r = g = b = At(data, src + x);
                        break;
                    case "32FC1" or "16UC1" or "mono16":
                    {
                        var v = Sample(data, step, x, y, enc, img.IsBigendian != 0);
                        if (!float.IsFinite(v) || v <= 0 || dMax <= dMin)
                        {
                            r = g = b = 0;
                        }
                        else
                        {
                            var c = RosTheme.Current.Ramp((v - dMin) / (dMax - dMin));
                            (r, g, b) = (c.R, c.G, c.B);
                        }

                        break;
                    }

                    default:
                        r = g = b = 128;
                        break;
                }

                row[x] = (uint)(a << 24 | (r * a / 255) << 16 | (g * a / 255) << 8 | (b * a / 255));
            }
        }
    }

    private static byte At(byte[] data, int i) => (uint)i < (uint)data.Length ? data[i] : (byte)0;

    private static float Sample(byte[] data, int step, int x, int y, string enc, bool bigEndian)
    {
        if (enc == "32FC1")
        {
            var i = (y * step) + (x * 4);
            if (i + 4 > data.Length)
            {
                return float.NaN;
            }

            var span = data.AsSpan(i, 4);
            return bigEndian ? System.Buffers.Binary.BinaryPrimitives.ReadSingleBigEndian(span) : System.Buffers.Binary.BinaryPrimitives.ReadSingleLittleEndian(span);
        }

        var j = (y * step) + (x * 2);
        if (j + 2 > data.Length)
        {
            return float.NaN;
        }

        var s = data.AsSpan(j, 2);
        return bigEndian ? System.Buffers.Binary.BinaryPrimitives.ReadUInt16BigEndian(s) : System.Buffers.Binary.BinaryPrimitives.ReadUInt16LittleEndian(s);
    }

    protected override void RenderContent(DrawingContext context, Rect bounds)
    {
        if (_bitmap is null)
        {
            DrawEmpty(context, bounds, "Waiting for image…");
            return;
        }

        var area = new Rect(bounds.X + 8, bounds.Y + 30, bounds.Width - 16, bounds.Height - 38);
        var src = new Size(_bitmap.PixelSize.Width, _bitmap.PixelSize.Height);
        var scale = Stretch == Stretch.Fill ? 1 : Math.Min(area.Width / src.Width, area.Height / src.Height);
        var dest = Stretch == Stretch.Fill
            ? area
            : new Rect(area.Center.X - (src.Width * scale / 2), area.Center.Y - (src.Height * scale / 2), src.Width * scale, src.Height * scale);
        context.DrawImage(_bitmap, new Rect(src), dest);
        DrawPill(context, _info, new Point(bounds.Right - 10, 6), P.Telemetry);
    }

    protected override void OnDetachedFromVisualTree(VisualTreeAttachmentEventArgs e)
    {
        _sub?.Dispose();
        _sub = null;
        base.OnDetachedFromVisualTree(e);
    }
}
