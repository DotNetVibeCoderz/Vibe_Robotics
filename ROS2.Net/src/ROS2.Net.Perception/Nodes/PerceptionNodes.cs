using Microsoft.Extensions.Logging;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.VisionMsgs;
using ROS2.Net.Perception.Images;
using ROS2.Net.Perception.Vision;

namespace ROS2.Net.Perception.Nodes;

/// <summary>
/// Filters a point cloud stream: optional pass-through on one axis, voxel downsampling and
/// statistical outlier removal. Parameters: <c>input_topic</c>, <c>output_topic</c>,
/// <c>voxel_leaf_size</c> (0 = off), <c>sor_mean_k</c> (0 = off), <c>sor_std_mul</c>,
/// <c>passthrough_axis</c> ("" / x / y / z), <c>passthrough_min</c>, <c>passthrough_max</c>.
/// </summary>
[RosComponent("ros2net_perception::PointCloudFilter")]
public sealed class PointCloudFilterNode : Node
{
    private readonly Publisher<PointCloud2> _output;

    public PointCloudFilterNode(Ros2Context context, NodeOptions options)
        : this(context, "pointcloud_filter", options)
    {
    }

    public PointCloudFilterNode(Ros2Context context, string name = "pointcloud_filter", NodeOptions? options = null)
        : base(context, name, options)
    {
        var input = DeclareParameter("input_topic", "points");
        var output = DeclareParameter("output_topic", "points_filtered");
        DeclareParameter("voxel_leaf_size", 0.05, "Voxel size in meters; 0 disables downsampling.");
        DeclareParameter("sor_mean_k", 0L, "Neighbours for statistical outlier removal; 0 disables it.");
        DeclareParameter("sor_std_mul", 1.0);
        DeclareParameter("passthrough_axis", string.Empty, "x, y or z; empty disables the pass-through filter.");
        DeclareParameter("passthrough_min", -1e9);
        DeclareParameter("passthrough_max", 1e9);
        _output = CreatePublisher<PointCloud2>(output, QosProfile.SensorData);
        CreateSubscription<PointCloud2>(input, OnCloud, QosProfile.SensorData);
    }

    /// <summary>Messages processed so far.</summary>
    public long Processed { get; private set; }

    /// <summary>Applies the configured pipeline (exposed for testing and reuse).</summary>
    public PointCloud Process(PointCloud cloud)
    {
        var axis = GetParameter<string>("passthrough_axis");
        if (axis.Length > 0)
        {
            var a = axis.ToLowerInvariant() switch
            {
                "x" => Axis.X,
                "y" => Axis.Y,
                _ => Axis.Z,
            };
            cloud = Filters.PassThrough(cloud, a, (float)GetParameter<double>("passthrough_min"), (float)GetParameter<double>("passthrough_max"));
        }

        var leaf = GetParameter<double>("voxel_leaf_size");
        if (leaf > 0)
        {
            cloud = Filters.VoxelGrid(cloud, (float)leaf);
        }

        var k = GetParameter<long>("sor_mean_k");
        if (k > 0 && cloud.Count > k)
        {
            cloud = Filters.StatisticalOutlierRemoval(cloud, (int)k, GetParameter<double>("sor_std_mul"));
        }

        return cloud;
    }

    private void OnCloud(PointCloud2 msg)
    {
        var cloud = Process(PointCloud.FromMessage(msg, skipNonFinite: true));
        _output.Publish(cloud.ToMessage(msg.Header.FrameId, RosTime.FromMsg(msg.Header.Stamp)));
        Processed++;
    }
}

/// <summary>Converts <c>sensor_msgs/LaserScan</c> to <c>sensor_msgs/PointCloud2</c>. Parameters: <c>scan_topic</c>, <c>cloud_topic</c>.</summary>
[RosComponent("ros2net_perception::ScanToCloud")]
public sealed class ScanToCloudNode : Node
{
    private readonly Publisher<PointCloud2> _output;

    public ScanToCloudNode(Ros2Context context, NodeOptions options)
        : this(context, "scan_to_cloud", options)
    {
    }

    public ScanToCloudNode(Ros2Context context, string name = "scan_to_cloud", NodeOptions? options = null)
        : base(context, name, options)
    {
        var scan = DeclareParameter("scan_topic", "scan");
        var cloud = DeclareParameter("cloud_topic", "scan_cloud");
        _output = CreatePublisher<PointCloud2>(cloud, QosProfile.SensorData);
        CreateSubscription<LaserScan>(scan, s => _output.Publish(LaserScanProjector.ToPointCloud2(s)), QosProfile.SensorData);
    }
}

/// <summary>
/// Converts depth images into point clouds using the matching camera info, like
/// <c>depth_image_proc/point_cloud_xyz</c>. Parameters: <c>depth_topic</c>, <c>camera_info_topic</c>,
/// <c>cloud_topic</c>, <c>stride</c>, <c>max_depth</c>.
/// </summary>
[RosComponent("ros2net_perception::DepthToCloud")]
public sealed class DepthToCloudNode : Node
{
    private readonly Publisher<PointCloud2> _output;
    private CameraModel? _camera;

    public DepthToCloudNode(Ros2Context context, NodeOptions options)
        : this(context, "depth_to_cloud", options)
    {
    }

    public DepthToCloudNode(Ros2Context context, string name = "depth_to_cloud", NodeOptions? options = null)
        : base(context, name, options)
    {
        var depth = DeclareParameter("depth_topic", "camera/depth/image_raw");
        var info = DeclareParameter("camera_info_topic", "camera/depth/camera_info");
        var cloud = DeclareParameter("cloud_topic", "camera/depth/points");
        DeclareParameter("stride", 1L);
        DeclareParameter("max_depth", 10.0);
        _output = CreatePublisher<PointCloud2>(cloud, QosProfile.SensorData);
        CreateSubscription<CameraInfo>(info, i => _camera = CameraModel.FromCameraInfo(i), QosProfile.SensorData);
        CreateSubscription<Image>(depth, OnDepth, QosProfile.SensorData);
    }

    private void OnDepth(Image image)
    {
        if (_camera is null)
        {
            return; // Wait for camera_info.
        }

        var cloud = DepthProjection.ToPointCloud(image, _camera, (int)Math.Max(1, GetParameter<long>("stride")), (float)GetParameter<double>("max_depth"));
        _output.Publish(cloud.ToMessage());
    }
}

/// <summary>
/// Runs an <see cref="IObjectDetector"/> on an image stream and publishes
/// <c>vision_msgs/Detection2DArray</c>. Frames that arrive while a detection is running are dropped,
/// so latency stays bounded. Parameters: <c>image_topic</c>, <c>detections_topic</c>.
/// </summary>
public sealed class DetectionNode : Node
{
    private readonly IObjectDetector _detector;
    private readonly Publisher<Detection2DArray> _output;
    private int _busy;

    public DetectionNode(Ros2Context context, IObjectDetector detector, string name = "object_detector", NodeOptions? options = null)
        : base(context, name, options)
    {
        _detector = detector ?? throw new ArgumentNullException(nameof(detector));
        var image = DeclareParameter("image_topic", "camera/image_raw");
        var output = DeclareParameter("detections_topic", "detections");
        _output = CreatePublisher<Detection2DArray>(output);
        CreateSubscription<Image>(image, OnImageAsync, QosProfile.SensorData.KeepLast(1));
    }

    /// <summary>Raised after each processed frame.</summary>
    public event Action<IReadOnlyList<Detection>>? Detected;

    public long DroppedFrames { get; private set; }

    private async ValueTask OnImageAsync(Image image)
    {
        if (Interlocked.Exchange(ref _busy, 1) == 1)
        {
            DroppedFrames++;
            return;
        }

        try
        {
            var detections = await _detector.DetectAsync(image, Context.ShutdownToken).ConfigureAwait(false);
            _output.Publish(DetectionPostprocessing.ToMessage(detections, image.Header));
            Detected?.Invoke(detections);
        }
        catch (Exception ex) when (ex is not OperationCanceledException)
        {
            Logger.LogError(ex, "Object detection failed");
        }
        finally
        {
            Volatile.Write(ref _busy, 0);
        }
    }
}
