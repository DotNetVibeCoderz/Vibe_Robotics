# Perception

`ROS2.Net.Perception` covers point clouds, laser scans, images, cameras and object-detection hooks. Heavy point-cloud kernels run in the Rust library `ros2net_native` when it is present, and fall back to bit-compatible managed code otherwise (see [native.md](native.md)).

```bash
dotnet add package ROS2.Net.Perception
dotnet add package ROS2.Net.Perception.Onnx   # optional: YOLO on ONNX Runtime
```

## Point clouds

`PointCloud` stores positions as one interleaved `xyz` float buffer, which the native kernels consume without copying. Optional channels are separate arrays: intensity, packed RGB (`0x00RRGGBB`) and normals.

```csharp
using ROS2.Net.Perception;

var cloud = PointCloud.FromMessage(msg, skipNonFinite: true);   // sensor_msgs/PointCloud2 → cloud
cloud = Filters.PassThrough(cloud, Axis.Z, -0.5f, 2.0f);
cloud = Filters.VoxelGrid(cloud, 0.05f);
cloud = Filters.StatisticalOutlierRemoval(cloud, meanK: 16, stdDevMultiplier: 1.0);
publisher.Publish(cloud.ToMessage("base_link"));
```

### Reading arbitrary layouts

`PointCloud2Reader` is a zero-copy view over a message. It honors `point_step`, `row_step` padding, organized clouds, big-endian data and every `PointField` datatype:

```csharp
var reader = new PointCloud2Reader(msg);
var ring = reader.GetField("ring");               // e.g. a UINT16 Velodyne field
for (var i = 0; i < reader.Count; i++)
{
    var r = (int)ring.Read(reader.Point(i));
}
```

`PointCloud2Writer` builds custom layouts with natural alignment and explicit padding. For example, the PCL `PointXYZI` 32-byte layout:

```csharp
var w = new PointCloud2Writer()
    .AddField("x", PointField.FLOAT32).AddField("y", PointField.FLOAT32).AddField("z", PointField.FLOAT32).Pad(4)
    .AddField("intensity", PointField.FLOAT32).Pad(12);
var msg = w.Create(width: n);
w.Field("intensity").Write(msg.Data.AsSpan(i * w.PointStep), 42);
```

### Algorithms

| API | What it does | Native |
|---|---|---|
| `Filters.PassThrough`, `CropBox`, `RemoveNonFinite` | Range filters | – |
| `Filters.VoxelGrid` | Voxel centroids (intensity/RGB averaged when present) | ✔ (xyz-only clouds) |
| `Filters.StatisticalOutlierRemoval` | PCL-style SOR | ✔ |
| `Filters.RadiusOutlierRemoval` | Minimum neighbours within a radius | ✔ |
| `Normals.Estimate`, `Normals.Pca` | PCA normals, curvature, viewpoint orientation | k-d tree ✔ |
| `Segmentation.SegmentPlane` | RANSAC plane with a reproducible seed | ✔ |
| `Segmentation.EuclideanClusters` | Euclidean cluster extraction | ✔ |
| `Registration.Icp` | ICP point-to-point (Horn closed form) or point-to-plane | point-to-point ✔ |
| `BoundingBoxes.ComputeAabb` / `ComputeObb` | Axis-aligned or PCA-oriented boxes | – |
| `LaserScanProjector` | `LaserScan` → points / `PointCloud2` / xy pairs | ✔ |
| `KdTree` | kNN and radius search, ordered by (distance², index) | ✔ |

RANSAC and clustering are deterministic. The same seed gives the same result on both implementations and across runs.

## Images

`ImageView` gives typed, zero-copy pixel access to `sensor_msgs/Image`, honoring `step` padding and `is_bigendian`. Supported encodings: `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `mono16`, `8UC1`, `8UC3`, `16UC1`, `32FC1`, `yuv422` (UYVY) and `yuv422_yuy2`.

`ImageOps` returns new, tightly packed images:

- `ConvertTo` handles every color and mono conversion, plus depth in `16UC1` mm ↔ `32FC1` m (0 mm becomes NaN).
- `Resize` is bilinear. `Crop`, `FlipHorizontal` and `FlipVertical` are also available.
- `GaussianBlur`, `Sobel` (edge magnitude) and `Threshold` are the built-in filters.

`Png` is a dependency-free codec. It encodes 8/16-bit gray, RGB and RGBA, and decodes gray, gray+alpha, RGB, RGBA and palette images at 8 or 16 bits, non-interlaced only, with CRC verification. `CompressedImageCodec` produces image_transport-style `sensor_msgs/CompressedImage` (`"bgr8; png compressed bgr8"`). **JPEG is not supported**: it would need a native codec. Use OpenCvSharp if you need it.

## Cameras

```csharp
var cam = CameraModel.FromCameraInfo(info);   // plumb_bob, rational_polynomial, equidistant
Vector2 pixel = cam.Project(new Vector3(0.2f, -0.1f, 1.5f));   // optical frame: x right, y down, z forward
Vector3 point = cam.Unproject(pixel.X, pixel.Y, depth: 1.5f);
var rectified = cam.Rectify(rawImage);       // or reuse cam.BuildRectifyMap()
var cloud = DepthProjection.ToPointCloud(depthImage, cam, stride: 2, maxDepth: 5f);
```

## Object detection

`IObjectDetector` is the hook for AI models. `DetectionNode` subscribes to images and publishes `vision_msgs/Detection2DArray`. The `vision_msgs` 4.x types (`Detection2D`, `BoundingBox2D/3D`, `ObjectHypothesisWithPose`, …) are generated into `ROS2.Net.Msgs.VisionMsgs`.

```csharp
using var detector = new YoloDetector("yolov8n.onnx", cocoLabels);   // ROS2.Net.Perception.Onnx
var node = new DetectionNode(context, detector);
```

`DetectionPostprocessing` contains the reusable pieces: YOLOv8/v11 and YOLOv5 output decoding, letterbox mapping and NMS.

## Ready-made nodes

| Node | Subscribes | Publishes | Parameters |
|---|---|---|---|
| `PointCloudFilterNode` | `points` | `points_filtered` | `voxel_leaf_size`, `sor_mean_k`, `sor_std_mul`, `passthrough_axis/min/max` |
| `ScanToCloudNode` | `scan` | `scan_cloud` | `scan_topic`, `cloud_topic` |
| `DepthToCloudNode` | `camera/depth/image_raw` and `camera_info` | `camera/depth/points` | `stride`, `max_depth` |
| `DetectionNode` | `camera/image_raw` | `detections` | `image_topic`, `detections_topic` |

The first three carry `[RosComponent]`, so they can be loaded into a `ComponentContainer`.

## Limitations

- JPEG is not supported, and PNG decoding does not handle interlaced images.
- `YoloDetector` was not run against a real model on the build machine. Only its decoding, NMS and letterbox logic are unit-tested.
- Laser projection does not correct per-beam timing (no `transformLaserScanToPointCloud` equivalent yet).

---
Made by Gravicode Studios, led by Kang Fadhil.
