using System.Buffers.Binary;
using System.IO.Compression;
using ROS2.Net.Msgs.SensorMsgs;

namespace ROS2.Net.Perception.Images;

/// <summary>
/// A small, dependency-free PNG codec (zlib via System.IO.Compression). Encodes mono8, mono16,
/// rgb8/bgr8 and rgba8/bgra8; decodes non-interlaced grayscale, gray+alpha, RGB, RGBA and palette
/// images at 8 or 16 bits per channel.
/// </summary>
public static class Png
{
    private static ReadOnlySpan<byte> Signature => [0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A];

    /// <summary>Encodes an image. BGR(A) is reordered to RGB(A); 16UC1 is stored as 16-bit gray.</summary>
    public static byte[] Encode(Image image, CompressionLevel level = CompressionLevel.Fastest)
    {
        var src = image.Encoding switch
        {
            ImageEncodings.Bgr8 => ImageOps.ConvertTo(image, ImageEncodings.Rgb8),
            ImageEncodings.Bgra8 => ImageOps.ConvertTo(image, ImageEncodings.Rgba8),
            ImageEncodings.Yuv422 or ImageEncodings.Yuv422Yuy2 => ImageOps.ConvertTo(image, ImageEncodings.Rgb8),
            _ => image,
        };
        var view = new ImageView(src);
        var (colorType, bitDepth) = src.Encoding switch
        {
            ImageEncodings.Mono8 or ImageEncodings.Type8UC1 => ((byte)0, (byte)8),
            ImageEncodings.Mono16 or ImageEncodings.Type16UC1 => ((byte)0, (byte)16),
            ImageEncodings.Rgb8 or ImageEncodings.Type8UC3 => ((byte)2, (byte)8),
            ImageEncodings.Rgba8 => ((byte)6, (byte)8),
            _ => throw new NotSupportedException($"PNG cannot store '{src.Encoding}'."),
        };

        var rowBytes = view.Width * view.Channels * view.ElementSize;
        using var raw = new MemoryStream();
        using (var z = new ZLibStream(raw, level, leaveOpen: true))
        {
            var line = new byte[rowBytes + 1];
            for (var y = 0; y < view.Height; y++)
            {
                line[0] = 0; // filter: none
                view.Row(y).CopyTo(line.AsSpan(1));
                if (bitDepth == 16 && src.IsBigendian == 0)
                {
                    for (var i = 1; i + 1 < line.Length; i += 2)
                    {
                        (line[i], line[i + 1]) = (line[i + 1], line[i]);
                    }
                }

                z.Write(line);
            }
        }

        using var output = new MemoryStream();
        output.Write(Signature);
        Span<byte> ihdr = stackalloc byte[13];
        BinaryPrimitives.WriteUInt32BigEndian(ihdr, (uint)view.Width);
        BinaryPrimitives.WriteUInt32BigEndian(ihdr[4..], (uint)view.Height);
        ihdr[8] = bitDepth;
        ihdr[9] = colorType;
        WriteChunk(output, "IHDR", ihdr);
        WriteChunk(output, "IDAT", raw.GetBuffer().AsSpan(0, (int)raw.Length));
        WriteChunk(output, "IEND", []);
        return output.ToArray();
    }

    private static void WriteChunk(Stream s, string type, ReadOnlySpan<byte> data)
    {
        Span<byte> head = stackalloc byte[8];
        BinaryPrimitives.WriteUInt32BigEndian(head, (uint)data.Length);
        for (var i = 0; i < 4; i++)
        {
            head[4 + i] = (byte)type[i];
        }

        s.Write(head);
        s.Write(data);
        var crc = PngCrc.Update(PngCrc.Update(0xFFFFFFFFu, head[4..]), data) ^ 0xFFFFFFFFu;
        Span<byte> c = stackalloc byte[4];
        BinaryPrimitives.WriteUInt32BigEndian(c, crc);
        s.Write(c);
    }

    /// <summary>Decodes a PNG to mono8, mono16, rgb8 or rgba8 (gray+alpha → rgba8, palette → rgb8/rgba8).</summary>
    public static Image Decode(ReadOnlySpan<byte> png)
    {
        if (png.Length < 8 || !png[..8].SequenceEqual(Signature))
        {
            throw new InvalidDataException("Not a PNG file.");
        }

        int width = 0, height = 0, bitDepth = 0, colorType = 0;
        byte[]? palette = null;
        byte[]? trns = null;
        using var idat = new MemoryStream();
        var pos = 8;
        while (pos + 12 <= png.Length)
        {
            var len = (int)BinaryPrimitives.ReadUInt32BigEndian(png[pos..]);
            if (len < 0 || pos + 12 + len > png.Length)
            {
                throw new InvalidDataException("Truncated PNG chunk.");
            }

            var type = System.Text.Encoding.ASCII.GetString(png.Slice(pos + 4, 4));
            var data = png.Slice(pos + 8, len);
            var crc = PngCrc.Update(0xFFFFFFFFu, png.Slice(pos + 4, 4 + len)) ^ 0xFFFFFFFFu;
            if (crc != BinaryPrimitives.ReadUInt32BigEndian(png[(pos + 8 + len)..]))
            {
                throw new InvalidDataException($"PNG chunk '{type}' has a bad CRC.");
            }

            switch (type)
            {
                case "IHDR":
                    width = (int)BinaryPrimitives.ReadUInt32BigEndian(data);
                    height = (int)BinaryPrimitives.ReadUInt32BigEndian(data[4..]);
                    bitDepth = data[8];
                    colorType = data[9];
                    if (data[12] != 0)
                    {
                        throw new NotSupportedException("Interlaced PNGs are not supported.");
                    }

                    break;
                case "PLTE":
                    palette = data.ToArray();
                    break;
                case "tRNS":
                    trns = data.ToArray();
                    break;
                case "IDAT":
                    idat.Write(data);
                    break;
            }

            pos += 12 + len;
            if (type == "IEND")
            {
                break;
            }
        }

        var channels = colorType switch
        {
            0 => 1,
            2 => 3,
            3 => 1,
            4 => 2,
            6 => 4,
            _ => throw new NotSupportedException($"PNG color type {colorType} is not supported."),
        };
        if (bitDepth != 8 && bitDepth != 16)
        {
            throw new NotSupportedException($"PNG bit depth {bitDepth} is not supported (use 8 or 16).");
        }

        var bpp = channels * bitDepth / 8;
        var stride = width * bpp;
        var pixels = new byte[stride * height];
        idat.Position = 0;
        using (var z = new ZLibStream(idat, CompressionMode.Decompress))
        {
            var prev = new byte[stride];
            var cur = new byte[stride];
            Span<byte> filter = stackalloc byte[1];
            for (var y = 0; y < height; y++)
            {
                z.ReadExactly(filter);
                z.ReadExactly(cur);
                Unfilter(filter[0], cur, prev, bpp);
                cur.CopyTo(pixels.AsSpan(y * stride));
                (prev, cur) = (cur, prev);
            }
        }

        return ToImage(width, height, colorType, bitDepth, pixels, palette, trns);
    }

    private static void Unfilter(byte filter, Span<byte> cur, ReadOnlySpan<byte> prev, int bpp)
    {
        for (var i = 0; i < cur.Length; i++)
        {
            int a = i >= bpp ? cur[i - bpp] : 0, b = prev[i], c = i >= bpp ? prev[i - bpp] : 0;
            cur[i] = filter switch
            {
                0 => cur[i],
                1 => (byte)(cur[i] + a),
                2 => (byte)(cur[i] + b),
                3 => (byte)(cur[i] + ((a + b) >> 1)),
                4 => (byte)(cur[i] + Paeth(a, b, c)),
                _ => throw new InvalidDataException($"Unknown PNG filter {filter}."),
            };
        }
    }

    private static int Paeth(int a, int b, int c)
    {
        var p = a + b - c;
        int pa = Math.Abs(p - a), pb = Math.Abs(p - b), pc = Math.Abs(p - c);
        return pa <= pb && pa <= pc ? a : pb <= pc ? b : c;
    }

    private static Image ToImage(int width, int height, int colorType, int bitDepth, byte[] px, byte[]? palette, byte[]? trns)
    {
        if (bitDepth == 16 && colorType == 0)
        {
            var img16 = ImageOps.Create(width, height, ImageEncodings.Mono16);
            img16.IsBigendian = 1; // PNG stores samples big-endian; keep them as-is.
            px.CopyTo(img16.Data, 0);
            return ImageOps.ConvertTo(img16, ImageEncodings.Mono16);
        }

        // Reduce 16-bit color to 8-bit (high byte).
        if (bitDepth == 16)
        {
            var reduced = new byte[px.Length / 2];
            for (var i = 0; i < reduced.Length; i++)
            {
                reduced[i] = px[2 * i];
            }

            px = reduced;
        }

        switch (colorType)
        {
            case 0:
            {
                var img = ImageOps.Create(width, height, ImageEncodings.Mono8);
                px.CopyTo(img.Data, 0);
                return img;
            }

            case 2:
            {
                var img = ImageOps.Create(width, height, ImageEncodings.Rgb8);
                px.CopyTo(img.Data, 0);
                return img;
            }

            case 6:
            {
                var img = ImageOps.Create(width, height, ImageEncodings.Rgba8);
                px.CopyTo(img.Data, 0);
                return img;
            }

            case 4:
            {
                var img = ImageOps.Create(width, height, ImageEncodings.Rgba8);
                for (var i = 0; i < width * height; i++)
                {
                    img.Data[4 * i] = img.Data[(4 * i) + 1] = img.Data[(4 * i) + 2] = px[2 * i];
                    img.Data[(4 * i) + 3] = px[(2 * i) + 1];
                }

                return img;
            }

            default:
            {
                var pal = palette ?? throw new InvalidDataException("Palette PNG without PLTE.");
                var alpha = trns is { Length: > 0 };
                var img = ImageOps.Create(width, height, alpha ? ImageEncodings.Rgba8 : ImageEncodings.Rgb8);
                var ch = alpha ? 4 : 3;
                for (var i = 0; i < width * height; i++)
                {
                    var k = px[i];
                    img.Data[ch * i] = pal[3 * k];
                    img.Data[(ch * i) + 1] = pal[(3 * k) + 1];
                    img.Data[(ch * i) + 2] = pal[(3 * k) + 2];
                    if (alpha)
                    {
                        img.Data[(ch * i) + 3] = k < trns!.Length ? trns[k] : (byte)255;
                    }
                }

                return img;
            }
        }
    }
}

/// <summary>CRC-32 (ISO-HDLC) as used by PNG chunks.</summary>
internal static class PngCrc
{
    private static readonly uint[] s_table = BuildTable();

    private static uint[] BuildTable()
    {
        var t = new uint[256];
        for (uint n = 0; n < 256; n++)
        {
            var c = n;
            for (var k = 0; k < 8; k++)
            {
                c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            }

            t[n] = c;
        }

        return t;
    }

    public static uint Update(uint crc, ReadOnlySpan<byte> data)
    {
        foreach (var b in data)
        {
            crc = s_table[(crc ^ b) & 0xFF] ^ (crc >> 8);
        }

        return crc;
    }
}

/// <summary>
/// <c>sensor_msgs/CompressedImage</c> codec compatible with image_transport's <c>compressed</c> plugin
/// naming (<c>"bgr8; png compressed bgr8"</c>). PNG only: JPEG needs a native codec and is reported as
/// unsupported.
/// </summary>
public static class CompressedImageCodec
{
    public static CompressedImage Encode(Image image, string format = "png")
    {
        if (!format.Equals("png", StringComparison.OrdinalIgnoreCase))
        {
            throw new NotSupportedException($"Compression format '{format}' is not supported; only 'png' is built in.");
        }

        var msg = new CompressedImage
        {
            Header = image.Header.Clone(),
            Format = $"{image.Encoding}; png compressed {image.Encoding}",
            Data = Png.Encode(image),
        };
        return msg;
    }

    /// <summary>Decodes a PNG CompressedImage. When the format names a target encoding (e.g. bgr8), the result is converted to it.</summary>
    public static Image Decode(CompressedImage message)
    {
        var data = message.Data.AsSpan();
        var isPng = data.Length >= 8 && data[0] == 0x89 && data[1] == (byte)'P';
        if (!isPng)
        {
            throw new NotSupportedException($"Only PNG compressed images can be decoded (format '{message.Format}').");
        }

        var image = Png.Decode(data);
        image.Header = message.Header.Clone();
        var encoding = message.Format.Split(';')[0].Trim();
        if (ImageEncodings.IsSupported(encoding) && encoding != image.Encoding && !encoding.Contains("png", StringComparison.OrdinalIgnoreCase))
        {
            image = ImageOps.ConvertTo(image, encoding);
        }

        return image;
    }
}
