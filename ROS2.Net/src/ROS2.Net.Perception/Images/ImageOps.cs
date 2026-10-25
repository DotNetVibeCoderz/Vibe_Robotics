using ROS2.Net.Msgs.SensorMsgs;

namespace ROS2.Net.Perception.Images;

/// <summary>
/// Image operations on <c>sensor_msgs/Image</c>. Each returns a new, tightly packed little-endian
/// image and copies the source header.
/// </summary>
public static class ImageOps
{
    /// <summary>Allocates a zeroed image.</summary>
    public static Image Create(int width, int height, string encoding, string frameId = "")
    {
        var bpp = ImageEncodings.BytesPerPixel(encoding);
        var img = new Image
        {
            Width = (uint)width,
            Height = (uint)height,
            Encoding = encoding,
            IsBigendian = 0,
            Step = (uint)(width * bpp),
            Data = new byte[width * height * bpp],
        };
        img.Header.FrameId = frameId;
        return img;
    }

    private static Image Like(Image src, int width, int height, string encoding)
    {
        var img = Create(width, height, encoding);
        img.Header = src.Header.Clone();
        return img;
    }

    /// <summary>
    /// Converts between encodings: any color/mono/16-bit/float/YUV source to rgb8, bgr8, rgba8,
    /// bgra8, mono8, mono16; and 16UC1 (millimeters) ↔ 32FC1 (meters) for depth.
    /// </summary>
    public static Image ConvertTo(Image image, string encoding)
    {
        if (image.Encoding == encoding && image.IsBigendian == 0)
        {
            return image.Clone();
        }

        var src = new ImageView(image);
        var dst = Like(image, src.Width, src.Height, encoding);
        var dv = new ImageView(dst);

        if (image.Encoding == ImageEncodings.Type16UC1 && encoding == ImageEncodings.Type32FC1)
        {
            ForEach(src, (x, y) =>
            {
                var mm = src.Get(x, y);
                dv.Set(x, y, 0, mm == 0 ? float.NaN : mm / 1000f);
            });
            return dst;
        }

        if (image.Encoding == ImageEncodings.Type32FC1 && encoding == ImageEncodings.Type16UC1)
        {
            ForEach(src, (x, y) =>
            {
                var m = src.Get(x, y);
                dv.Set(x, y, 0, float.IsFinite(m) ? m * 1000f : 0);
            });
            return dst;
        }

        switch (encoding)
        {
            case ImageEncodings.Rgb8 or ImageEncodings.Bgr8 or ImageEncodings.Rgba8 or ImageEncodings.Bgra8:
                var bgr = encoding is ImageEncodings.Bgr8 or ImageEncodings.Bgra8;
                var alpha = ImageEncodings.HasAlpha(encoding);
                var srcAlpha = ImageEncodings.HasAlpha(image.Encoding);
                ForEach(src, (x, y) =>
                {
                    var (r, g, b) = src.GetRgb(x, y);
                    dv.Set(x, y, 0, bgr ? b : r);
                    dv.Set(x, y, 1, g);
                    dv.Set(x, y, 2, bgr ? r : b);
                    if (alpha)
                    {
                        dv.Set(x, y, 3, srcAlpha ? src.Get(x, y, 3) : 255);
                    }
                });
                return dst;
            case ImageEncodings.Mono8 or ImageEncodings.Type8UC1:
                ForEach(src, (x, y) => dv.Set(x, y, 0, src.Gray(x, y)));
                return dst;
            case ImageEncodings.Mono16 or ImageEncodings.Type16UC1:
                ForEach(src, (x, y) => dv.Set(x, y, 0, src.Gray(x, y) * 257f));
                return dst;
            case ImageEncodings.Type32FC1:
                ForEach(src, (x, y) => dv.Set(x, y, 0, src.Gray(x, y) / 255f));
                return dst;
            default:
                throw new NotSupportedException($"Conversion from '{image.Encoding}' to '{encoding}' is not supported.");
        }
    }

    /// <summary>Bilinear resize (YUV images are converted to rgb8 first).</summary>
    public static Image Resize(Image image, int width, int height)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(width);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(height);
        if (image.Encoding is ImageEncodings.Yuv422 or ImageEncodings.Yuv422Yuy2)
        {
            image = ConvertTo(image, ImageEncodings.Rgb8);
        }

        var src = new ImageView(image);
        var dst = Like(image, width, height, image.Encoding);
        var dv = new ImageView(dst);
        var sx = (float)src.Width / width;
        var sy = (float)src.Height / height;
        for (var y = 0; y < height; y++)
        {
            var fy = Math.Clamp(((y + 0.5f) * sy) - 0.5f, 0, src.Height - 1);
            var y0 = (int)fy;
            var y1 = Math.Min(y0 + 1, src.Height - 1);
            var wy = fy - y0;
            for (var x = 0; x < width; x++)
            {
                var fx = Math.Clamp(((x + 0.5f) * sx) - 0.5f, 0, src.Width - 1);
                var x0 = (int)fx;
                var x1 = Math.Min(x0 + 1, src.Width - 1);
                var wx = fx - x0;
                for (var c = 0; c < src.Channels; c++)
                {
                    var top = (src.Get(x0, y0, c) * (1 - wx)) + (src.Get(x1, y0, c) * wx);
                    var bottom = (src.Get(x0, y1, c) * (1 - wx)) + (src.Get(x1, y1, c) * wx);
                    dv.Set(x, y, c, (top * (1 - wy)) + (bottom * wy));
                }
            }
        }

        return dst;
    }

    /// <summary>Copies the rectangle (x, y, width, height), clipped to the image.</summary>
    public static Image Crop(Image image, int x, int y, int width, int height)
    {
        var src = new ImageView(image);
        x = Math.Clamp(x, 0, src.Width);
        y = Math.Clamp(y, 0, src.Height);
        width = Math.Clamp(width, 0, src.Width - x);
        height = Math.Clamp(height, 0, src.Height - y);
        if (image.Encoding is ImageEncodings.Yuv422 or ImageEncodings.Yuv422Yuy2)
        {
            // Keep pixel pairs intact.
            x &= ~1;
            width &= ~1;
        }

        var dst = Like(image, width, height, image.Encoding);
        var bpp = src.Channels * src.ElementSize;
        for (var r = 0; r < height; r++)
        {
            image.Data.AsSpan(((y + r) * src.Step) + (x * bpp), width * bpp).CopyTo(dst.Data.AsSpan(r * (int)dst.Step));
        }

        dst.IsBigendian = image.IsBigendian;
        return dst;
    }

    /// <summary>Mirrors left-right.</summary>
    public static Image FlipHorizontal(Image image)
    {
        if (image.Encoding is ImageEncodings.Yuv422 or ImageEncodings.Yuv422Yuy2)
        {
            image = ConvertTo(image, ImageEncodings.Rgb8);
        }

        var src = new ImageView(image);
        var dst = Like(image, src.Width, src.Height, image.Encoding);
        dst.IsBigendian = image.IsBigendian;
        var bpp = src.Channels * src.ElementSize;
        for (var y = 0; y < src.Height; y++)
        {
            for (var x = 0; x < src.Width; x++)
            {
                image.Data.AsSpan((y * src.Step) + (x * bpp), bpp).CopyTo(dst.Data.AsSpan((y * (int)dst.Step) + ((src.Width - 1 - x) * bpp)));
            }
        }

        return dst;
    }

    /// <summary>Mirrors top-bottom.</summary>
    public static Image FlipVertical(Image image)
    {
        var src = new ImageView(image);
        var dst = Like(image, src.Width, src.Height, image.Encoding);
        dst.IsBigendian = image.IsBigendian;
        var rowBytes = (int)dst.Step;
        for (var y = 0; y < src.Height; y++)
        {
            image.Data.AsSpan(y * src.Step, rowBytes).CopyTo(dst.Data.AsSpan((src.Height - 1 - y) * rowBytes));
        }

        return dst;
    }

    /// <summary>Separable Gaussian blur (radius ≈ 3σ, clamped borders).</summary>
    public static Image GaussianBlur(Image image, float sigma)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(sigma);
        if (image.Encoding is ImageEncodings.Yuv422 or ImageEncodings.Yuv422Yuy2)
        {
            image = ConvertTo(image, ImageEncodings.Rgb8);
        }

        var radius = Math.Max(1, (int)MathF.Ceiling(sigma * 3));
        var kernel = new float[(2 * radius) + 1];
        float sum = 0;
        for (var i = -radius; i <= radius; i++)
        {
            sum += kernel[i + radius] = MathF.Exp(-(i * i) / (2 * sigma * sigma));
        }

        for (var i = 0; i < kernel.Length; i++)
        {
            kernel[i] /= sum;
        }

        var src = new ImageView(image);
        int w = src.Width, h = src.Height, ch = src.Channels;
        var tmp = new float[w * h * ch];
        for (var y = 0; y < h; y++)
        {
            for (var x = 0; x < w; x++)
            {
                for (var c = 0; c < ch; c++)
                {
                    float acc = 0;
                    for (var k = -radius; k <= radius; k++)
                    {
                        acc += kernel[k + radius] * src.Get(Math.Clamp(x + k, 0, w - 1), y, c);
                    }

                    tmp[(((y * w) + x) * ch) + c] = acc;
                }
            }
        }

        var dst = Like(image, w, h, image.Encoding);
        var dv = new ImageView(dst);
        for (var y = 0; y < h; y++)
        {
            for (var x = 0; x < w; x++)
            {
                for (var c = 0; c < ch; c++)
                {
                    float acc = 0;
                    for (var k = -radius; k <= radius; k++)
                    {
                        acc += kernel[k + radius] * tmp[(((Math.Clamp(y + k, 0, h - 1) * w) + x) * ch) + c];
                    }

                    dv.Set(x, y, c, acc);
                }
            }
        }

        return dst;
    }

    /// <summary>Sobel gradient magnitude of the luma, as mono8 (saturated).</summary>
    public static Image Sobel(Image image)
    {
        var src = new ImageView(image);
        int w = src.Width, h = src.Height;
        var gray = new float[w * h];
        ForEach(src, (x, y) => gray[(y * w) + x] = src.Gray(x, y));
        var dst = Like(image, w, h, ImageEncodings.Mono8);
        float G(int x, int y) => gray[(Math.Clamp(y, 0, h - 1) * w) + Math.Clamp(x, 0, w - 1)];
        for (var y = 0; y < h; y++)
        {
            for (var x = 0; x < w; x++)
            {
                var gx = -G(x - 1, y - 1) - (2 * G(x - 1, y)) - G(x - 1, y + 1) + G(x + 1, y - 1) + (2 * G(x + 1, y)) + G(x + 1, y + 1);
                var gy = -G(x - 1, y - 1) - (2 * G(x, y - 1)) - G(x + 1, y - 1) + G(x - 1, y + 1) + (2 * G(x, y + 1)) + G(x + 1, y + 1);
                dst.Data[(y * w) + x] = (byte)Math.Clamp(MathF.Sqrt((gx * gx) + (gy * gy)), 0, 255);
            }
        }

        return dst;
    }

    /// <summary>Binary threshold of the luma: 255 where luma &gt; <paramref name="threshold"/> (or ≤, with <paramref name="invert"/>).</summary>
    public static Image Threshold(Image image, float threshold, bool invert = false)
    {
        var src = new ImageView(image);
        var dst = Like(image, src.Width, src.Height, ImageEncodings.Mono8);
        ForEach(src, (x, y) => dst.Data[(y * src.Width) + x] = (byte)((src.Gray(x, y) > threshold) != invert ? 255 : 0));
        return dst;
    }

    private static void ForEach(ImageView view, Action<int, int> action)
    {
        for (var y = 0; y < view.Height; y++)
        {
            for (var x = 0; x < view.Width; x++)
            {
                action(x, y);
            }
        }
    }
}
