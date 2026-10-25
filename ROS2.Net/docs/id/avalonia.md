# Kontrol Avalonia UI — `ROS2.Net.Avalonia`

> Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.

Kontrol desktop untuk UI robot di Avalonia 12. Kontrol digambar sendiri (custom-drawn), sehingga tetap cepat pada laju sensor dan tidak bergantung pada template. Semuanya mengikuti satu sistem desain: aksen sinyal ungu → cyan di atas biru dongker gelap, diambil dari ikon paket. Tersedia palet terang dan gelap.

```bash
dotnet add package ROS2.Net.Avalonia
```

![Halaman teleop di galeri](../images/desktop/gallery-teleop.png)

## Kontrol

| Kontrol | Menampilkan / melakukan | Cara menyambungkan |
|---|---|---|
| `TeleopJoystick` | Seret knob atau pakai tombol panah/WASD untuk menghasilkan `geometry_msgs/Twist`. Menerbitkan 10 Hz selama digeser, lalu satu pesan nol terakhir. | `Attach(node, "/cmd_vel")` |
| `LaserScanView` | `sensor_msgs/LaserScan` dalam bentuk polar dengan cincin jarak, diwarnai berdasarkan jarak | `Attach(node, "/scan")` |
| `OccupancyGridView` | Peta atau costmap dengan geser/zoom. Overlay: robot, jalur, tujuan, titik. `WorldClicked` memberi koordinat peta. | `Attach(node, "/map", "/plan")` |
| `PointCloudView` | Point cloud xyz dalam proyeksi atas, samping, depan, atau orbit, berlapis. Warna menurut ketinggian atau warna tetap. | `SetPoints(xyz)` / `AddLayer(...)` |
| `ImageView` | `sensor_msgs/Image`: rgb8, bgr8, rgba8, bgra8, mono8, mono16, 16UC1, 32FC1. Gambar kedalaman dinormalisasi. | `Attach(node, "/camera/image_raw")` |
| `PlotView` | Plot multi-seri real-time (ala rqt_plot), skala otomatis, jendela waktu bergeser | `AddSample(seri, t, v)` dari thread mana pun |
| `LogConsole` | `/rosout` berwarna sesuai level, dengan filter dan gulir mundur | `Attach(node)` |
| `TopicMonitor` | Semua topic beserta tipe, jumlah pub/sub, Hz, dan bandwidth, diukur dengan subscription mentah (serialized) | `Attach(node)` |
| `NodeGraphView` | Graf node → topic → node ala rqt_graph | `Attach(node)` |
| `TfTreeView` | Pohon frame dari sebuah `TfBuffer`, lengkap dengan translasi | `Buffer = tfBuffer` |
| `ParameterEditor` | Ubah parameter node mana pun lewat service parameternya. Menampilkan tipe, deskripsi, dan penolakan. | `await AttachAsync(node, "/robot/controller")` |
| `BatteryGauge` | Gauge busur untuk `sensor_msgs/BatteryState` | `Attach(node, "/battery_state")` |
| `JointStatePanel` | Satu bar per sendi di dalam batasnya | `Attach(node, "/joint_states")` |
| `SimWorldView` | Dunia `ROS2.Net.Simulation` dari atas: rintangan, robot, sinar LiDAR, jejak | `Attach(simWorld)` |

## Threading

Callback ROS tidak pernah menyentuh kontrol secara langsung. `RosUi` menjembatani topic ke thread UI:

```csharp
// Hanya pesan terbaru; pesan di antaranya dibuang, jadi topic 1 kHz tidak membanjiri UI.
using var sub = RosUi.SubscribeLatest<Odometry>(node, "/odom", o => speed.Text = $"{o.Twist.Twist.Linear.X:0.00}");

// Semua pesan, dikumpulkan per frame UI (log, event).
using var logs = RosUi.SubscribeAll<Log>(node, "/rosout", batch => { foreach (var m in batch) console.Append(DateTime.Now, m.Level, m.Name, m.Msg); });
```

Keduanya memakai subscription tanpa callback, jadi tetap berfungsi baik node sedang spin maupun tidak.

## Tema

```csharp
RosTheme.IsDark = false;          // mengganti palet dan varian tema Fluent; kontrol digambar ulang
var p = RosTheme.Current;         // Background, Surface, Seam, Ink, Haze, Signal, Telemetry, Alert, Fault, Ok
border.Background = p.SignalGradient();
```

Panggil `.WithInterFont()` pada `AppBuilder` agar tipografinya sesuai rancangan. Data dan nama topic memakai font monospace (Cascadia Mono / Consolas / Menlo).

## Aplikasi minimal

```csharp
var context = await Ros2Context.CreateAsync(new Ros2ContextOptions().UseRtps());
_ = context.SpinAsync();
var node = context.CreateNode("dashboard");

var joystick = new TeleopJoystick();
joystick.Attach(node, "/cmd_vel");
var scan = new LaserScanView();
scan.Attach(node, "/scan");
window.Content = new Grid { ColumnDefinitions = new("*,*"), Children = { joystick, scan } };
Grid.SetColumn(scan, 1);
```

Lihat [galeri](gallery.md) untuk melihat setiap kontrol dalam konteksnya, dan [simulator](simulator.md) untuk contoh aplikasi lengkap.
