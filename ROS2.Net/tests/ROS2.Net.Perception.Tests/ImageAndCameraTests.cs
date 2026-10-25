using System.Numerics;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Perception.Images;
using Shouldly;

namespace ROS2.Net.Perception.Tests;

public class ImageAndCameraTests
{
    private static Image Gradient(int w, int h, string encoding = ImageEncodings.Rgb8)
    {
        var img = ImageOps.Create(w, h, ImageEncodings.Rgb8);
        for (var y = 0; y < h; y++)
        {
            for (var x = 0; x < w; x++)
            {
                var o = (y * w * 3) + (x * 3);
                img.Data[o] = (byte)(x * 10);
                img.Data[o + 1] = (byte)(y * 10);
                img.Data[o + 2] = 200;
            }
        }

        return encoding == ImageEncodings.Rgb8 ? img : ImageOps.ConvertTo(img, encoding);
    }

    [Fact]
    public void Color_conversions_preserve_pixels()
    {
        var rgb = Gradient(8, 6);
        var bgr = ImageOps.ConvertTo(rgb, ImageEncodings.Bgr8);
        bgr.Data[0..3].ShouldBe(new byte[] { 200, 0, 0 });
        var rgba = ImageOps.ConvertTo(bgr, ImageEncodings.Rgba8);
        new ImageView(rgba).GetRgb(3, 2).ShouldBe(((byte)30, (byte)20, (byte)200));
        rgba.Data[3].ShouldBe((byte)255);
        ImageOps.ConvertTo(rgba, ImageEncodings.Rgb8).Data.ShouldBe(rgb.Data);

        var mono = ImageOps.ConvertTo(rgb, ImageEncodings.Mono8);
        mono.Data[0].ShouldBe((byte)Math.Round(0.114 * 200));
        var mono16 = ImageOps.ConvertTo(mono, ImageEncodings.Mono16);
        new ImageView(mono16).Get(0, 0).ShouldBe(mono.Data[0] * 257f);
    }

    [Fact]
    public void Yuv422_decodes_to_rgb()
    {
        // UYVY gray pair: U=V=128, Y=16 (black) and Y=235 (white).
        var yuv = new Image { Width = 2, Height = 1, Encoding = ImageEncodings.Yuv422, Step = 4, Data = [128, 16, 128, 235] };
        var rgb = ImageOps.ConvertTo(yuv, ImageEncodings.Rgb8);
        rgb.Data[0..3].ShouldBe(new byte[] { 0, 0, 0 });
        rgb.Data[3..6].ShouldBe(new byte[] { 255, 255, 255 });
    }

    [Fact]
    public void Depth_unit_conversion()
    {
        var depth = ImageOps.Create(2, 1, ImageEncodings.Type16UC1);
        BitConverter.TryWriteBytes(depth.Data.AsSpan(0), (ushort)1500);
        var meters = ImageOps.ConvertTo(depth, ImageEncodings.Type32FC1);
        new ImageView(meters).Get(0, 0).ShouldBe(1.5f);
        float.IsNaN(new ImageView(meters).Get(1, 0)).ShouldBeTrue();
        new ImageView(ImageOps.ConvertTo(meters, ImageEncodings.Type16UC1)).Get(0, 0).ShouldBe(1500f);
    }

    [Fact]
    public void Geometric_ops()
    {
        var img = Gradient(10, 8);
        var resized = ImageOps.Resize(img, 5, 4);
        (resized.Width, resized.Height, resized.Step).ShouldBe((5u, 4u, 15u));
        var crop = ImageOps.Crop(img, 2, 3, 4, 2);
        new ImageView(crop).GetRgb(0, 0).ShouldBe(((byte)20, (byte)30, (byte)200));
        new ImageView(ImageOps.FlipHorizontal(img)).GetRgb(0, 0).ShouldBe(((byte)90, (byte)0, (byte)200));
        new ImageView(ImageOps.FlipVertical(img)).GetRgb(0, 0).ShouldBe(((byte)0, (byte)70, (byte)200));
    }

    [Fact]
    public void Filters_blur_edges_threshold()
    {
        var flat = ImageOps.Create(16, 16, ImageEncodings.Mono8);
        flat.Data.AsSpan().Fill(77);
        ImageOps.GaussianBlur(flat, 1.5f).Data.ShouldAllBe(b => b == 77);

        var step = ImageOps.Create(16, 16, ImageEncodings.Mono8);
        for (var y = 0; y < 16; y++)
        {
            step.Data.AsSpan((y * 16) + 8, 8).Fill(255);
        }

        var edges = ImageOps.Sobel(step);
        edges.Data[(5 * 16) + 8].ShouldBe((byte)255);
        edges.Data[(5 * 16) + 2].ShouldBe((byte)0);
        ImageOps.Threshold(step, 128).Data.Count(b => b == 255).ShouldBe(128);
    }

    [Theory]
    [InlineData(ImageEncodings.Rgb8)]
    [InlineData(ImageEncodings.Bgr8)]
    [InlineData(ImageEncodings.Rgba8)]
    [InlineData(ImageEncodings.Mono8)]
    [InlineData(ImageEncodings.Mono16)]
    public void Png_round_trips(string encoding)
    {
        var img = Gradient(13, 7, encoding);
        var png = Png.Encode(img);
        var back = Png.Decode(png);
        var expected = encoding == ImageEncodings.Bgr8 ? ImageOps.ConvertTo(img, ImageEncodings.Rgb8) : img;
        back.Encoding.ShouldBe(expected.Encoding);
        back.Data.ShouldBe(expected.Data);
    }

    [Fact]
    public void Png_rejects_corruption()
    {
        var png = Png.Encode(Gradient(4, 4));
        png[40] ^= 0xFF;
        Should.Throw<InvalidDataException>(() => Png.Decode(png));
    }

    [Fact]
    public void Compressed_image_codec_restores_the_original_encoding()
    {
        var bgr = Gradient(6, 5, ImageEncodings.Bgr8);
        bgr.Header.FrameId = "cam";
        var compressed = CompressedImageCodec.Encode(bgr);
        compressed.Format.ShouldBe("bgr8; png compressed bgr8");
        var back = CompressedImageCodec.Decode(compressed);
        back.Encoding.ShouldBe(ImageEncodings.Bgr8);
        back.Data.ShouldBe(bgr.Data);
        back.Header.FrameId.ShouldBe("cam");
        Should.Throw<NotSupportedException>(() => CompressedImageCodec.Encode(bgr, "jpeg"));
    }

    [Fact]
    public void Camera_project_unproject_round_trip_with_distortion()
    {
        var cam = new CameraModel(640, 480, 525, 520, 319.5, 239.5, [-0.28, 0.07, 0.001, -0.0005, 0]);
        foreach (var p in new[] { new Vector3(0.3f, -0.2f, 2f), new Vector3(-0.5f, 0.4f, 1.2f), new Vector3(0, 0, 5) })
        {
            var px = cam.Project(p);
            var back = cam.Unproject(px.X, px.Y, p.Z);
            back.X.ShouldBe(p.X, 1e-4f);
            back.Y.ShouldBe(p.Y, 1e-4f);
        }

        cam.TryProject(new Vector3(0, 0, -1), out _).ShouldBeFalse();

        var info = cam.ToCameraInfo("cam");
        var again = CameraModel.FromCameraInfo(info);
        again.Fx.ShouldBe(525);
        again.Distortion.Count.ShouldBe(5);
    }

    [Fact]
    public void Rectify_map_is_identity_without_distortion()
    {
        var cam = new CameraModel(8, 6, 100, 100, 4, 3);
        var (mx, my) = cam.BuildRectifyMap();
        mx[(2 * 8) + 5].ShouldBe(5f, 1e-4f);
        my[(2 * 8) + 5].ShouldBe(2f, 1e-4f);
        var img = Gradient(8, 6);
        cam.Rectify(img).Data.ShouldBe(img.Data);
    }

    [Fact]
    public void Depth_image_becomes_points()
    {
        var cam = new CameraModel(4, 4, 2, 2, 2, 2);
        var depth = ImageOps.Create(4, 4, ImageEncodings.Type32FC1);
        var view = new ImageView(depth);
        view.Set(2, 2, 0, 3f);
        view.Set(0, 0, 0, 1f);
        var cloud = DepthProjection.ToPointCloud(depth, cam);
        cloud.Count.ShouldBe(2);
        cloud[0].ShouldBe(new Vector3(-1, -1, 1));
        cloud[1].ShouldBe(new Vector3(0, 0, 3));
        DepthProjection.ToPointCloud(depth, cam, organized: true).Count.ShouldBe(16);
    }
}
