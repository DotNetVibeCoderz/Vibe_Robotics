# TF2 — kerangka koordinat

`ROS2.Net.Tf2` melacak hubungan antar-kerangka koordinat (`map`, `odom`, `base_link`, `laser_link`, ...) dari waktu ke waktu, seperti `tf2` / `tf2_ros`. Library ini memakai topik standar `/tf` dan `/tf_static` (`tf2_msgs/msg/TFMessage`), sehingga bisa langsung bekerja bersama RViz, `robot_state_publisher` dan node ROS 2 lainnya.

```bash
dotnet add package ROS2.Net.Tf2
```

## Tipe matematika

| Tipe | Fungsi |
|---|---|
| `Vector3d` | vektor 3D: `+ - *`, `Dot`, `Cross`, `Lerp`, `Normalized()` |
| `Quaterniond` | rotasi: `*`, `Inverse()`, `Rotate(v)`, `Slerp`, `FromRpy`/`ToRpy`, `FromYaw`, `FromAxisAngle`/`ToAxisAngle` |
| `Transform3d` | transformasi kaku `p' = R·p + t`: `a * b` (b diterapkan dulu), `Inverse()`, `TransformPoint`, `TransformVector`, `Interpolate`, `From2D(x, y, yaw)` |

Semuanya punya `ToMsg()` / `FromMsg()` untuk tipe `geometry_msgs`.

## Buffer

```csharp
var buffer = new TfBuffer();                     // menyimpan riwayat 10 detik per kerangka
buffer.SetTransform("map", "odom", Transform3d.Identity, RosTime.Now(), isStatic: true);
buffer.SetTransform("odom", "base_link", Transform3d.From2D(1.0, 2.0, Math.PI / 2), RosTime.Now());

// Di mana posisi laser jika dilihat dari kerangka map?
TransformStamped tf = buffer.LookupTransform("map", "laser_link", RosTime.Zero);
```

- `LookupTransform(target, source, time)` mengembalikan transformasi yang memetakan data dari `source` ke `target`. `RosTime.Zero` berarti "waktu terbaru yang datanya tersedia di semua mata rantai", sama seperti perilaku tf2.
- `LookupTransform(target, targetTime, source, sourceTime, fixedFrame)` adalah *time travel*. Contohnya: "di mana robot satu detik lalu, dilihat dari posisinya sekarang", dengan `odom` sebagai kerangka tetap.
- `CanTransform(...)` dan `TryLookupTransform(...)` memeriksa tanpa melempar exception. `WaitForTransformAsync(..., timeout, ct)` menunggu sampai datanya datang.
- `TransformPose`, `TransformPoint` dan `TransformVector` mengubah data `geometry_msgs` bertimestamp sesuai timestamp datanya sendiri. Berikan timeout opsional jika ingin menunggu.
- Kegagalan melempar tipe exception tf2: `LookupException` (kerangka tidak dikenal), `ConnectivityException` (pohon terpisah), `ExtrapolationException` (waktu di luar cache) dan `TransformTimeoutException`.

Lookup di antara dua sampel menginterpolasi translasi secara linear dan rotasi dengan slerp. Kerangka statis berlaku untuk semua waktu. Setiap kerangka punya lock sendiri, sehingga listener yang menulis dengan laju tinggi jarang berebut dengan control loop yang melakukan lookup.

### Introspeksi

| Method | Keluaran |
|---|---|
| `AllFramesAsString()` | `Frame base_link exists with parent odom.` |
| `AllFramesAsYaml()` | YAML seperti keluaran `view_frames` / `tf2_monitor` (parent, broadcaster, rate, panjang buffer) |
| `ToMermaid()` / `ToDot()` | pohon kerangka dalam bentuk flowchart Mermaid atau Graphviz DOT |
| `FrameNames`, `GetParent(frame)`, `FrameExists(frame)` | |

## Mendengarkan dan menyiarkan

```csharp
await using var context = await Ros2Context.CreateAsync();
var node = context.CreateNode("tf_demo");

var buffer = new TfBuffer();
using var listener = new TransformListener(buffer, node);       // /tf + /tf_static

var broadcaster = new TransformBroadcaster(node);
broadcaster.SendTransform("odom", "base_link", Transform3d.From2D(x, y, yaw), node.Clock.Now());

var statics = new StaticTransformBroadcaster(node);
statics.SendTransform("base_link", "laser_link", new Transform3d(new Vector3d(0.2, 0, 0.15), Quaterniond.Identity));
```

- `TransformListener` memakai subscription tanpa callback yang dikuras oleh task latar belakang tersendiri. Karena itu buffer tetap mutakhir meskipun node tidak di-*spin*, dan lalu lintas TF tidak pernah bersaing dengan callback node. `/tf` memakai keep-last 100, reliable. `/tf_static` memakai keep-last 100, transient local, sehingga kerangka statis yang sudah dipublikasikan sebelumnya tetap diterima.
- `StaticTransformBroadcaster` mempublikasikan ke topik *latched* (depth 1, transient local). Seperti tf2_ros, setiap pemanggilan mempublikasikan ulang seluruh kumpulan kerangka statis, sehingga subscriber yang terlambat bergabung menerima semuanya.

## Status verifikasi

Unit test dan integration test mencakup hal-hal berikut, melalui middleware in-process:
- lookup berantai lewat leluhur bersama, cabang bersaudara dan *latest common time*
- interpolasi, galat ekstrapolasi, pohon yang tidak terhubung dan *time travel*
- helper data bertimestamp, keluaran introspeksi dan menunggu transformasi
- broadcaster → listener secara utuh

Interoperabilitas dengan graph ROS 2 sungguhan bergantung pada middleware yang dipilih (lihat dokumentasi RTPS dan rosbridge).
