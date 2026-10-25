# Galeri ROS2.Net (desktop)

> Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.

Galeri adalah aplikasi etalase yang setiap halamannya menjalankan graf ROS 2 sungguhan di dalam jendela, di atas middleware in-process. Setiap halaman berisi demo langsung, kode yang menggerakkannya, dan interface ROS yang dipakainya, yang ditampilkan sebagai chip di header. Tidak perlu memasang ROS.

```bash
dotnet run --project apps/ROS2.Net.Gallery.Desktop
```

![Beranda galeri](../images/desktop/gallery-home.png)

Sidebar dapat beralih antara Bahasa Inggris dan Bahasa Indonesia, serta antara tema gelap dan terang. **Sembunyikan kode** menutup panel kode.

| | |
|---|---|
| ![Bahasa Indonesia](../images/desktop/gallery-home-id.png) | ![Tema terang](../images/desktop/gallery-home-light.png) |

## Halaman

Halaman dikelompokkan menurut lapisan stack ROS.

### Komunikasi

| Halaman | Menunjukkan |
|---|---|
| Talker & listener | Publisher, subscription, timer, aliran async (`ReadAllAsync`), `PlotView` langsung |
| Service | Server `AddTwoInts`, client dengan await, latensi tiap panggilan |
| Action | Goal Fibonacci, aliran feedback, pembatalan, status hasil |
| Parameter | Deskriptor dan rentang nilai, callback validasi, parameter read-only, `ParameterEditor` melalui service parameter, `/parameter_events` |
| Lifecycle node | Mesin status yang dikendalikan dengan tombol. Lifecycle publisher hanya mengirim saat Active. |

![Talker](../images/desktop/gallery-talker.png)
![Action](../images/desktop/gallery-actions.png)
![Parameter](../images/desktop/gallery-parameters.png)
![Lifecycle](../images/desktop/gallery-lifecycle.png)
![Service](../images/desktop/gallery-services.png)

### Robotika

| Halaman | Menunjukkan |
|---|---|
| Teleop TurtleBot | Simulator + `TeleopJoystick` di `/robot1/cmd_vel`, plot odometri, baterai |
| LiDAR & peta | `LaserScanView` dan `/map` yang di-latch, dengan pose ground-truth dan scan yang diproyeksikan |
| Perencanaan jalur | Costmap dengan inflation; A*, Theta*, Dijkstra, dan Hybrid-A* (`ROS2.Net.Navigation`). Klik untuk memilih tujuan. |
| Frame TF2 | Dua robot di `/tf` dan `/tf_static`, `TfTreeView`, `lookupTransform` langsung |
| ros2_control | Controller manager, diff-drive controller, joint state broadcaster, lintasan odometri |
| Kinematika lengan | UR5 dari URDF, IK Levenberg–Marquardt dalam batas sendi, tampilan orbit |

![Teleop](../images/desktop/gallery-teleop.png)
![LiDAR](../images/desktop/gallery-lidar.png)
![Navigasi](../images/desktop/gallery-navigation.png)
![TF](../images/desktop/gallery-tf.png)
![Kontrol](../images/desktop/gallery-control.png)
![Lengan](../images/desktop/gallery-arm.png)

### Kecerdasan

| Halaman | Menunjukkan |
|---|---|
| Point cloud | Voxel grid, SOR, penghapusan lantai RANSAC, dan klaster Euclidean. Memakai kernel Rust bila tersedia. |
| Reinforcement learning | Melatih agen Q-table di `RobotGymEnv` (RLNet), menampilkan kurva return, lalu policy greedy yang mengemudi |
| Perintah bahasa alami | `NaturalLanguageCommanderNode` dengan model rule-based offline, dalam Bahasa Inggris dan Indonesia. Robot bergerak di bawah pembatas keselamatan. |

![Persepsi](../images/desktop/gallery-perception.png)
![RL](../images/desktop/gallery-rl.png)
![Commander](../images/desktop/gallery-commander.png)

### Perkakas

| Halaman | Menunjukkan |
|---|---|
| Perekaman rosbag2 | Merekam CDR mentah ke MCAP beserta `metadata.yaml`, `BagInfo`, pemutaran berulang ke `/replay/*` |
| Konektivitas & graf | Beralih antara middleware in-process, RTPS, dan rosbridge; `NodeGraphView` dan `TopicMonitor` |

![Bag](../images/desktop/gallery-bag.png)
![Konektivitas](../images/desktop/gallery-connectivity.png)

## Cara kerjanya

- `GalleryRuntime` memiliki satu `Ros2Context`: middleware in-process di domain tersendiri, di-spin oleh `MultiThreadedExecutor`. Halaman membuat node-nya di sana dan membuangnya saat pengguna pindah halaman.
- Setiap halaman diturunkan dari `GalleryPage`. Halaman menyediakan `Id`, `Group`, `Title`, dan `Summary` dalam dua bahasa, `Interfaces` (chip), `Code`, dan `Build(runtime)`.
- UI dibangun dengan C# tanpa XAML, memakai helper `Ui` yang mengambil semua warna dari `RosTheme`.

## Screenshot

Gambar di dokumen ini dirender secara headless dari jendela aslinya, memakai Avalonia.Headless dan Skia:

```bash
dotnet test tests/ROS2.Net.Avalonia.Tests --filter Category=Screenshots
```

Perintah ini menulis `docs/images/desktop/*.png`.
