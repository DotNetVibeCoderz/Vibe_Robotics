# Persepsi

`ROS2.Net.Perception` menangani point cloud, laser scan, gambar, kamera, dan hook untuk deteksi objek. Kernel point cloud yang berat dijalankan oleh library Rust `ros2net_native` kalau tersedia. Kalau tidak, ada cadangan kode managed yang hasilnya identik (lihat [native.md](native.md)).

```bash
dotnet add package ROS2.Net.Perception
dotnet add package ROS2.Net.Perception.Onnx   # opsional: YOLO di ONNX Runtime
```

## Point cloud

`PointCloud` menyimpan posisi titik dalam satu buffer float `xyz` yang interleaved, sehingga kernel native bisa langsung memakainya tanpa menyalin. Kanal tambahan disimpan sebagai array terpisah: intensitas, RGB terpaket (`0x00RRGGBB`), dan normal.

```csharp
using ROS2.Net.Perception;

var cloud = PointCloud.FromMessage(msg, skipNonFinite: true);   // sensor_msgs/PointCloud2 → cloud
cloud = Filters.PassThrough(cloud, Axis.Z, -0.5f, 2.0f);
cloud = Filters.VoxelGrid(cloud, 0.05f);
cloud = Filters.StatisticalOutlierRemoval(cloud, meanK: 16, stdDevMultiplier: 1.0);
publisher.Publish(cloud.ToMessage("base_link"));
```

### Membaca layout apa pun

`PointCloud2Reader` adalah tampilan zero-copy atas sebuah pesan. Reader ini mengikuti `point_step`, padding `row_step`, cloud terorganisasi, data big-endian, dan semua tipe data `PointField`:

```csharp
var reader = new PointCloud2Reader(msg);
var ring = reader.GetField("ring");               // misalnya field UINT16 dari Velodyne
for (var i = 0; i < reader.Count; i++)
{
    var r = (int)ring.Read(reader.Point(i));
}
```

`PointCloud2Writer` membuat layout kustom dengan perataan alami dan padding eksplisit. Contohnya layout 32-byte `PointXYZI` milik PCL:

```csharp
var w = new PointCloud2Writer()
    .AddField("x", PointField.FLOAT32).AddField("y", PointField.FLOAT32).AddField("z", PointField.FLOAT32).Pad(4)
    .AddField("intensity", PointField.FLOAT32).Pad(12);
var msg = w.Create(width: n);
```

### Algoritma

| API | Fungsi | Native |
|---|---|---|
| `Filters.PassThrough`, `CropBox`, `RemoveNonFinite` | Filter rentang | – |
| `Filters.VoxelGrid` | Centroid voxel (intensitas/RGB ikut dirata-rata) | ✔ (cloud xyz saja) |
| `Filters.StatisticalOutlierRemoval` | SOR ala PCL | ✔ |
| `Filters.RadiusOutlierRemoval` | Minimal tetangga dalam radius | ✔ |
| `Normals.Estimate`, `Normals.Pca` | Normal via PCA, kelengkungan, orientasi ke viewpoint | k-d tree ✔ |
| `Segmentation.SegmentPlane` | Bidang RANSAC dengan seed yang bisa diulang | ✔ |
| `Segmentation.EuclideanClusters` | Ekstraksi klaster Euclidean | ✔ |
| `Registration.Icp` | ICP titik-ke-titik (bentuk tertutup Horn) atau titik-ke-bidang | titik-ke-titik ✔ |
| `BoundingBoxes.ComputeAabb` / `ComputeObb` | Kotak sejajar sumbu atau berorientasi PCA | – |
| `LaserScanProjector` | `LaserScan` → titik / `PointCloud2` / pasangan xy | ✔ |
| `KdTree` | Pencarian kNN dan radius, diurutkan menurut (jarak², indeks) | ✔ |

RANSAC dan clustering bersifat deterministik. Seed yang sama memberi hasil yang sama di kedua implementasi dan di setiap run.

## Gambar

`ImageView` memberi akses piksel bertipe dan zero-copy ke `sensor_msgs/Image`, mengikuti padding `step` dan `is_bigendian`. Encoding yang didukung: `rgb8`, `bgr8`, `rgba8`, `bgra8`, `mono8`, `mono16`, `8UC1`, `8UC3`, `16UC1`, `32FC1`, `yuv422` (UYVY), dan `yuv422_yuy2`.

`ImageOps` selalu menghasilkan gambar baru yang padat:

- `ConvertTo` menangani semua konversi warna dan mono, serta kedalaman `16UC1` mm ↔ `32FC1` m (0 mm menjadi NaN).
- `Resize` memakai interpolasi bilinear. Tersedia juga `Crop`, `FlipHorizontal`, dan `FlipVertical`.
- Filter bawaannya adalah `GaussianBlur`, `Sobel` (magnitudo tepi), dan `Threshold`.

`Png` adalah codec tanpa dependensi. Codec ini mengenkode gray, RGB, dan RGBA 8/16-bit, serta mendekode gray, gray+alpha, RGB, RGBA, dan palet 8/16-bit (hanya non-interlaced, dengan verifikasi CRC). `CompressedImageCodec` menghasilkan `sensor_msgs/CompressedImage` bergaya image_transport (`"bgr8; png compressed bgr8"`). **JPEG belum didukung** karena butuh codec native. Pakai OpenCvSharp kalau perlu.

## Kamera

```csharp
var cam = CameraModel.FromCameraInfo(info);   // plumb_bob, rational_polynomial, equidistant
Vector2 pixel = cam.Project(new Vector3(0.2f, -0.1f, 1.5f));   // frame optik: x kanan, y bawah, z depan
Vector3 point = cam.Unproject(pixel.X, pixel.Y, depth: 1.5f);
var rectified = cam.Rectify(rawImage);
var cloud = DepthProjection.ToPointCloud(depthImage, cam, stride: 2, maxDepth: 5f);
```

## Deteksi objek

`IObjectDetector` adalah titik sambung untuk model AI. `DetectionNode` berlangganan gambar lalu menerbitkan `vision_msgs/Detection2DArray`. Tipe `vision_msgs` 4.x di-generate ke `ROS2.Net.Msgs.VisionMsgs`.

```csharp
using var detector = new YoloDetector("yolov8n.onnx", cocoLabels);   // ROS2.Net.Perception.Onnx
var node = new DetectionNode(context, detector);
```

`DetectionPostprocessing` berisi komponen yang bisa dipakai ulang: dekode output YOLOv8/v11 dan YOLOv5, pemetaan letterbox, dan NMS.

## Node siap pakai

| Node | Berlangganan | Menerbitkan | Parameter |
|---|---|---|---|
| `PointCloudFilterNode` | `points` | `points_filtered` | `voxel_leaf_size`, `sor_mean_k`, `sor_std_mul`, `passthrough_axis/min/max` |
| `ScanToCloudNode` | `scan` | `scan_cloud` | `scan_topic`, `cloud_topic` |
| `DepthToCloudNode` | `camera/depth/image_raw` dan `camera_info` | `camera/depth/points` | `stride`, `max_depth` |
| `DetectionNode` | `camera/image_raw` | `detections` | `image_topic`, `detections_topic` |

Tiga node pertama memakai `[RosComponent]`, jadi bisa dimuat ke `ComponentContainer`.

## Batasan

- JPEG belum didukung, dan dekode PNG belum menangani gambar interlaced.
- `YoloDetector` belum diuji dengan model sungguhan di mesin build. Yang sudah diuji unit hanya logika dekode, NMS, dan letterbox-nya.
- Proyeksi laser belum mengoreksi waktu per berkas.

---
Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.
