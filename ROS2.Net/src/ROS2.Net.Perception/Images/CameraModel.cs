using System.Numerics;
using ROS2.Net.Msgs.SensorMsgs;

namespace ROS2.Net.Perception.Images;

/// <summary>Lens distortion model names used in <c>CameraInfo.distortion_model</c>.</summary>
public static class DistortionModels
{
    public const string PlumbBob = "plumb_bob";
    public const string RationalPolynomial = "rational_polynomial";
    public const string Equidistant = "equidistant";
}

/// <summary>
/// A pinhole camera with lens distortion, built from <c>sensor_msgs/CameraInfo</c> (like
/// <c>image_geometry::PinholeCameraModel</c>). Points are in the optical frame: x right, y down, z forward.
/// </summary>
public sealed class CameraModel
{
    private readonly double[] _d;

    public CameraModel(int width, int height, double fx, double fy, double cx, double cy, double[]? distortion = null, string distortionModel = DistortionModels.PlumbBob)
    {
        Width = width;
        Height = height;
        Fx = fx;
        Fy = fy;
        Cx = cx;
        Cy = cy;
        _d = distortion ?? [];
        DistortionModel = distortionModel;
        R = [1, 0, 0, 0, 1, 0, 0, 0, 1];
        P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0];
    }

    public static CameraModel FromCameraInfo(CameraInfo info)
    {
        var model = new CameraModel((int)info.Width, (int)info.Height, info.K[0], info.K[4], info.K[2], info.K[5], info.D, string.IsNullOrEmpty(info.DistortionModel) ? DistortionModels.PlumbBob : info.DistortionModel);
        if (info.R.Length == 9 && info.R.Any(v => v != 0))
        {
            model.R = (double[])info.R.Clone();
        }

        if (info.P.Length == 12 && info.P[0] != 0)
        {
            model.P = (double[])info.P.Clone();
        }

        return model;
    }

    public int Width { get; }

    public int Height { get; }

    public double Fx { get; }

    public double Fy { get; }

    public double Cx { get; }

    public double Cy { get; }

    public string DistortionModel { get; }

    public IReadOnlyList<double> Distortion => _d;

    /// <summary>Rectification rotation (row-major 3×3).</summary>
    public double[] R { get; private set; }

    /// <summary>Rectified projection (row-major 3×4).</summary>
    public double[] P { get; private set; }

    public bool HasDistortion => _d.Any(v => v != 0);

    /// <summary>Builds a <c>CameraInfo</c> message for this model.</summary>
    public CameraInfo ToCameraInfo(string frameId = "")
    {
        var info = new CameraInfo
        {
            Width = (uint)Width,
            Height = (uint)Height,
            DistortionModel = DistortionModel,
            D = (double[])_d.Clone(),
            K = [Fx, 0, Cx, 0, Fy, Cy, 0, 0, 1],
            R = (double[])R.Clone(),
            P = (double[])P.Clone(),
        };
        info.Header.FrameId = frameId;
        return info;
    }

    private double D(int i) => i < _d.Length ? _d[i] : 0;

    /// <summary>Applies the distortion model to normalized image coordinates.</summary>
    public (double X, double Y) Distort(double x, double y)
    {
        if (DistortionModel == DistortionModels.Equidistant)
        {
            var r = Math.Sqrt((x * x) + (y * y));
            if (r < 1e-12)
            {
                return (x, y);
            }

            var th = Math.Atan(r);
            double th2 = th * th, th4 = th2 * th2, th6 = th4 * th2, th8 = th4 * th4;
            var thd = th * (1 + (D(0) * th2) + (D(1) * th4) + (D(2) * th6) + (D(3) * th8));
            return (x * thd / r, y * thd / r);
        }

        double k1 = D(0), k2 = D(1), p1 = D(2), p2 = D(3), k3 = D(4);
        var r2 = (x * x) + (y * y);
        var radial = 1 + (k1 * r2) + (k2 * r2 * r2) + (k3 * r2 * r2 * r2);
        if (DistortionModel == DistortionModels.RationalPolynomial)
        {
            radial /= 1 + (D(5) * r2) + (D(6) * r2 * r2) + (D(7) * r2 * r2 * r2);
        }

        var xd = (x * radial) + (2 * p1 * x * y) + (p2 * (r2 + (2 * x * x)));
        var yd = (y * radial) + (p1 * (r2 + (2 * y * y))) + (2 * p2 * x * y);
        return (xd, yd);
    }

    /// <summary>Inverts <see cref="Distort"/> by fixed-point iteration (converges for realistic lenses).</summary>
    public (double X, double Y) Undistort(double xd, double yd, int iterations = 20)
    {
        if (!HasDistortion)
        {
            return (xd, yd);
        }

        double x = xd, y = yd;
        for (var i = 0; i < iterations; i++)
        {
            var (dx, dy) = Distort(x, y);
            x += xd - dx;
            y += yd - dy;
        }

        return (x, y);
    }

    /// <summary>Projects a 3D point (optical frame) to raw (distorted) pixel coordinates. Returns false behind the camera.</summary>
    public bool TryProject(Vector3 point, out Vector2 pixel)
    {
        if (point.Z <= 0)
        {
            pixel = default;
            return false;
        }

        var (xd, yd) = Distort(point.X / (double)point.Z, point.Y / (double)point.Z);
        pixel = new Vector2((float)((Fx * xd) + Cx), (float)((Fy * yd) + Cy));
        return true;
    }

    public Vector2 Project(Vector3 point) =>
        TryProject(point, out var px) ? px : throw new ArgumentException("Point is behind the camera.", nameof(point));

    /// <summary>The unit-depth ray (z = 1) through a raw pixel.</summary>
    public Vector3 Unproject(double u, double v)
    {
        var (x, y) = Undistort((u - Cx) / Fx, (v - Cy) / Fy);
        return new Vector3((float)x, (float)y, 1f);
    }

    /// <summary>The 3D point at <paramref name="depth"/> along the ray through a raw pixel.</summary>
    public Vector3 Unproject(double u, double v, float depth) => Unproject(u, v) * depth;

    /// <summary>
    /// Builds a rectification map: for each rectified pixel, the raw pixel to sample (like
    /// <c>cv::initUndistortRectifyMap</c>). Maps are row-major, <see cref="Width"/>×<see cref="Height"/>.
    /// </summary>
    public (float[] MapX, float[] MapY) BuildRectifyMap()
    {
        var mapX = new float[Width * Height];
        var mapY = new float[Width * Height];
        double fxp = P[0], fyp = P[5], cxp = P[2], cyp = P[6];
        var r = R;
        for (var v = 0; v < Height; v++)
        {
            for (var u = 0; u < Width; u++)
            {
                // Rectified ray, rotated back into the raw camera frame (Rᵀ).
                double xr = (u - cxp) / fxp, yr = (v - cyp) / fyp;
                var x = (r[0] * xr) + (r[3] * yr) + r[6];
                var y = (r[1] * xr) + (r[4] * yr) + r[7];
                var w = (r[2] * xr) + (r[5] * yr) + r[8];
                var (xd, yd) = Distort(x / w, y / w);
                mapX[(v * Width) + u] = (float)((Fx * xd) + Cx);
                mapY[(v * Width) + u] = (float)((Fy * yd) + Cy);
            }
        }

        return (mapX, mapY);
    }

    /// <summary>Rectifies a raw image with bilinear sampling (pixels mapping outside the image become 0).</summary>
    public Image Rectify(Image raw, (float[] MapX, float[] MapY)? map = null)
    {
        var (mx, my) = map ?? BuildRectifyMap();
        var src = new ImageView(raw.Encoding is ImageEncodings.Yuv422 or ImageEncodings.Yuv422Yuy2 ? ImageOps.ConvertTo(raw, ImageEncodings.Rgb8) : raw);
        var dst = ImageOps.Create(Width, Height, src.Encoding);
        dst.Header = raw.Header.Clone();
        var dv = new ImageView(dst);
        for (var v = 0; v < Height; v++)
        {
            for (var u = 0; u < Width; u++)
            {
                var sx = mx[(v * Width) + u];
                var sy = my[(v * Width) + u];
                if (sx < 0 || sy < 0 || sx > src.Width - 1 || sy > src.Height - 1)
                {
                    continue;
                }

                var x0 = (int)sx;
                var y0 = (int)sy;
                var x1 = Math.Min(x0 + 1, src.Width - 1);
                var y1 = Math.Min(y0 + 1, src.Height - 1);
                float wx = sx - x0, wy = sy - y0;
                for (var c = 0; c < src.Channels; c++)
                {
                    var top = (src.Get(x0, y0, c) * (1 - wx)) + (src.Get(x1, y0, c) * wx);
                    var bottom = (src.Get(x0, y1, c) * (1 - wx)) + (src.Get(x1, y1, c) * wx);
                    dv.Set(u, v, c, (top * (1 - wy)) + (bottom * wy));
                }
            }
        }

        return dst;
    }
}

/// <summary>Depth image → point cloud, like <c>depth_image_proc/point_cloud_xyz</c>.</summary>
public static class DepthProjection
{
    /// <summary>
    /// Converts a depth image (16UC1 millimeters or 32FC1 meters) into points in the optical frame.
    /// Invalid depths (0, NaN, beyond <paramref name="maxDepth"/>) are skipped unless
    /// <paramref name="organized"/>, in which case they become NaN and the cloud keeps one point per pixel.
    /// </summary>
    public static PointCloud ToPointCloud(Image depth, CameraModel camera, int stride = 1, float maxDepth = float.PositiveInfinity, bool organized = false)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(stride);
        var view = new ImageView(depth);
        var scale = depth.Encoding switch
        {
            ImageEncodings.Type16UC1 or ImageEncodings.Mono16 => 0.001f,
            ImageEncodings.Type32FC1 => 1f,
            _ => throw new NotSupportedException($"Depth encoding '{depth.Encoding}' is not supported (use 16UC1 or 32FC1)."),
        };
        var cloud = new PointCloud((view.Width / stride) * (view.Height / stride))
        {
            FrameId = depth.Header.FrameId,
            Stamp = RosTime.FromMsg(depth.Header.Stamp),
        };

        // Rays depend only on the pixel, so undistortion cost is paid once per pixel.
        for (var v = 0; v < view.Height; v += stride)
        {
            for (var u = 0; u < view.Width; u += stride)
            {
                var z = view.Get(u, v) * scale;
                if (!(z > 0) || z > maxDepth || !float.IsFinite(z))
                {
                    if (organized)
                    {
                        cloud.Add(float.NaN, float.NaN, float.NaN);
                    }

                    continue;
                }

                var p = camera.Unproject(u, v, z);
                cloud.Add(p.X, p.Y, p.Z);
            }
        }

        return cloud;
    }
}
