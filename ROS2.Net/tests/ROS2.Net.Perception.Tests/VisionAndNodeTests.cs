using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.VisionMsgs;
using ROS2.Net.Perception.Images;
using ROS2.Net.Perception.Nodes;
using ROS2.Net.Perception.Vision;
using ROS2.Net.Rmw.InProcess;
using Shouldly;

namespace ROS2.Net.Perception.Tests;

public class VisionAndNodeTests
{
    private static int s_domain = 150;

    private static Ros2Context CreateContext() => Ros2Context.Create(new Ros2ContextOptions
    {
        DomainId = Interlocked.Increment(ref s_domain) % 230,
        Rmw = new InProcessRmwFactory(),
    });

    [Fact]
    public void Nms_keeps_the_best_box_per_class()
    {
        var dets = new[]
        {
            new Detection(0, "person", 0.9f, new BoxF(0, 0, 10, 10)),
            new Detection(0, "person", 0.8f, new BoxF(1, 1, 10, 10)),
            new Detection(1, "dog", 0.7f, new BoxF(1, 1, 10, 10)),
            new Detection(0, "person", 0.6f, new BoxF(50, 50, 10, 10)),
        };
        var kept = DetectionPostprocessing.NonMaxSuppression(dets, 0.5f);
        kept.Select(d => d.Score).ShouldBe(new[] { 0.9f, 0.7f, 0.6f });
        DetectionPostprocessing.NonMaxSuppression(dets, 0.5f, classAgnostic: true).Count.ShouldBe(2);
    }

    [Fact]
    public void Yolo_outputs_decode_in_both_layouts()
    {
        // Two classes, three boxes; layout [4 + C, N].
        float[] v8 =
        [
            100, 200, 300, // cx
            100, 200, 300, // cy
            20, 40, 60, // w
            20, 40, 60, // h
            0.9f, 0.1f, 0.2f, // class 0
            0.05f, 0.8f, 0.1f, // class 1
        ];
        var d = DetectionPostprocessing.DecodeYoloV8(v8, 2, 3, 0.25f, ["a", "b"]);
        d.Count.ShouldBe(2);
        d[0].Label.ShouldBe("a");
        d[0].Box.ShouldBe(new BoxF(90, 90, 20, 20));
        d[1].ClassId.ShouldBe(1);

        float[] v5 = [50, 50, 10, 10, 0.9f, 0.2f, 0.8f, 10, 10, 4, 4, 0.1f, 0.9f, 0.1f];
        var e = DetectionPostprocessing.DecodeYoloV5(v5, 2, 2, 0.25f);
        e.Count.ShouldBe(1);
        e[0].ClassId.ShouldBe(1);
        e[0].Score.ShouldBe(0.72f, 1e-5f);
    }

    [Fact]
    public void Letterbox_maps_back_to_image_pixels()
    {
        var lb = Letterbox.Compute(1280, 720, 640);
        lb.Scale.ShouldBe(0.5f);
        lb.PadY.ShouldBe(140f);
        lb.ToImage(new BoxF(10, 150, 20, 20)).ShouldBe(new BoxF(20, 20, 40, 40));
    }

    private sealed class FakeDetector : IObjectDetector
    {
        public IReadOnlyList<string> Labels => ["thing"];

        public ValueTask<IReadOnlyList<Detection>> DetectAsync(Image image, CancellationToken cancellationToken = default) =>
            ValueTask.FromResult<IReadOnlyList<Detection>>([new Detection(0, "thing", 0.5f, new BoxF(1, 2, (int)image.Width, 4))]);

        public void Dispose()
        {
        }
    }

    [Fact]
    public async Task Detection_node_publishes_vision_msgs()
    {
        await using var ctx = CreateContext();
        var node = new DetectionNode(ctx, new FakeDetector());
        var observer = ctx.CreateNode("observer");
        var output = observer.CreateSubscription<Detection2DArray>("/detections");
        var input = observer.CreatePublisher<Image>("/camera/image_raw", QosProfile.SensorData);
        using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        var spin = ctx.SpinAsync(cts.Token);

        var img = ImageOps.Create(32, 16, ImageEncodings.Rgb8, "camera");
        input.Publish(img);
        var msg = await output.TakeAsync(cts.Token);
        msg.Detections.Length.ShouldBe(1);
        msg.Detections[0].Bbox.SizeX.ShouldBe(32);
        msg.Detections[0].Results[0].Hypothesis.ClassId.ShouldBe("thing");
        msg.Header.FrameId.ShouldBe("camera");

        cts.Cancel();
        await spin;
        node.Dispose();
    }

    [Fact]
    public async Task Point_cloud_filter_node_downsamples_the_stream()
    {
        await using var ctx = CreateContext();
        var filter = new PointCloudFilterNode(ctx, options: new NodeOptions().WithParameter("voxel_leaf_size", 0.5));
        var observer = ctx.CreateNode("observer");
        var output = observer.CreateSubscription<PointCloud2>("/points_filtered", QosProfile.SensorData);
        var input = observer.CreatePublisher<PointCloud2>("/points", QosProfile.SensorData);
        using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        var spin = ctx.SpinAsync(cts.Token);

        input.Publish(Synthetic.Plane(10, 0.1f, 0).ToMessage("base"));
        var msg = await output.TakeAsync(cts.Token);
        msg.Width.ShouldBe(4u);
        msg.Header.FrameId.ShouldBe("base");
        filter.Processed.ShouldBe(1);

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Scan_to_cloud_node_projects_scans()
    {
        await using var ctx = CreateContext();
        using var node = new ScanToCloudNode(ctx);
        var observer = ctx.CreateNode("observer");
        var output = observer.CreateSubscription<PointCloud2>("/scan_cloud", QosProfile.SensorData);
        var input = observer.CreatePublisher<LaserScan>("/scan", QosProfile.SensorData);
        using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        var spin = ctx.SpinAsync(cts.Token);

        input.Publish(new LaserScan { AngleIncrement = 0.1f, RangeMin = 0.1f, RangeMax = 10, Ranges = [1, 2, 3] });
        (await output.TakeAsync(cts.Token)).Width.ShouldBe(3u);

        cts.Cancel();
        await spin;
    }
}
