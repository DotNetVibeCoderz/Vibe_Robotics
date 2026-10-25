# Simulasi (`ROS2.Net.Simulation`)

Simulator robot 2.5D yang deterministik dan langsung "berbicara" ROS 2. Dunianya planar sehingga fisika, LiDAR, dan peta tetap cepat. Rintangan dan robot juga punya tinggi, jadi renderer bisa menggambarnya dalam 3D. Semuanya berjalan di dalam proses, tanpa perlu Gazebo.

> Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.

## Mulai cepat

```csharp
using ROS2.Net;
using ROS2.Net.Simulation;

await using var context = await Ros2Context.CreateAsync();
var world = Scenarios.Create(ScenarioKind.Warehouse, robots: 2);   // robot1, robot2
using var sim = new SimulationNode(context, world);
await sim.RunAsync(cancellationToken);   // stepping real-time sekaligus melayani service
```

Sekarang node mana pun bisa mengendalikan `/robot1/cmd_vel` serta membaca `/robot1/odom`, `/robot1/scan`, dan topik lainnya. Ini berlaku juga untuk node di mesin lain bila memakai middleware RTPS.

## Dunia simulasi

| Konsep | API |
|---|---|
| Dunia | `SimWorld`: langkah tetap (default 100 Hz), `Step()`, `StepFor(detik)`, `RunAsync(ct)`, `Paused`, `RealTimeFactor` (0 = secepat mungkin), `Reset()` |
| Determinisme | `SimWorldOptions.Seed`. Semua noise berasal dari satu `SimRandom` ber-seed, jadi seed yang sama menghasilkan jalannya simulasi yang identik. |
| Rintangan | `SimObstacle.Box / Cylinder / Wall / Polygon`. Dengan `dynamic: true`, rintangan bisa didorong robot; benda yang lebih berat bergeser lebih sedikit. |
| Medan | `Friction` (faktor traksi) dan `FloorColor` |
| Kueri | `RayCast`, `IsOccupied`, `SampleColor`, dan `ToOccupancyGrid(resolusi)`, yang menghasilkan `nav_msgs/OccupancyGrid` ground-truth |

Rintangan statis diindeks dalam grid seragam, dan sinar menelusurinya dengan traversal DDA. Langkah fisika sama sekali tidak melakukan alokasi memori. Hal ini dibuktikan oleh sebuah test dengan tiga robot bersensor lengkap di gudang.

## Robot

| Tipe | Perintah | Catatan |
|---|---|---|
| `DiffDriveRobot` | `linear.x`, `angular.z` | Dimensi default TurtleBot3 Burger (`RobotParameters.TurtleBot3Waffle()` juga tersedia); ada slip roda dan noise encoder |
| `AckermannRobot` | `linear.x`, `angular.z` (laju yaw → sudut kemudi) | Model sepeda; set `AngularIsSteeringAngle` untuk memerintahkan sudut kemudi secara langsung |
| `OmniRobot` | `linear.x`, `linear.y`, `angular.z` | Basis holonomik |

Perintah yang lebih tua dari `CommandTimeout` (0,5 detik) dianggap nol. Perubahan kecepatan dibatasi oleh batas akselerasi. Saat tabrakan, robot didorong keluar dari rintangan dan `Bumper` menjadi true. Odometri mengintegrasikan bacaan encoder, sehingga lama-kelamaan menyimpang dari ground truth, sama seperti robot sungguhan.

`SimArm.TwoDof()` dan `SimArm.SixDof()` memasang lengan kinematik sederhana. Sendinya bergerak ke target dengan kecepatan terbatas, dan nilainya muncul di `joint_states` serta di snapshot.

`RobotFactory.Create(kind, name, pose, parameters, sensors)` memasang sensor standar:

| Sensor | Kelas | Default |
|---|---|---|
| LiDAR 2D | `LidarSensor` | 360 sinar, 0,12–3,5 m, 10 Hz, σ = 1 cm, dropout opsional |
| IMU | `ImuSensor` | 100 Hz, noise giroskop dan bias random-walk, akselerometer termasuk gravitasi |
| GPS | `GpsSensor` | 5 Hz, σ = 0,5 m di sekitar `GeoOrigin` (default: Bandung) |
| Sonar | `RangeSensor` | depan dan belakang, kerucut 0,5 rad |
| Baterai | `BatterySensor` | Berkurang saat robot bergerak. Gunakan `IsCharging` untuk mengisi daya, dan `DrainFactor` untuk mempercepat pengurasan saat demo. |
| Kamera | `CameraSensor` | Citra rgb8 tampak atas dari area di depan robot (default mati) |

## Skenario

`Scenarios.Create(kind, robots)` membangun dunia siap pakai. Untuk menyuntingnya dulu, panggil `Scenarios.Definition(kind)`, ubah hasilnya, lalu simpan sebagai JSON atau YAML.

| Skenario | Ukuran | Isi |
|---|---|---|
| `Empty` | 20×20 m | Ruang terbuka |
| `Warehouse` | 24×16 m | 12 rak beserta waypoint, pilar, 6 peti yang bisa didorong, dok, stasiun pengisian daya |
| `Office` | 16×12 m | Empat ruangan dengan pintu, meja kerja, meja rapat |
| `Maze` | 12×9 m | Dibangkitkan dari seed, dengan titik `start` dan `exit` |
| `CorridorLoop` | 14×10 m | Koridor melingkar selebar 2 m, cocok untuk menguji loop closure SLAM |
| `AgricultureRows` | 24×16 m | Barisan tanaman, pohon, lumbung; traksi lebih rendah dan geo-origin lahan pertanian |

Contoh berkas dunia (YAML atau JSON; sudut dalam derajat):

```yaml
name: lab
bounds: [0, 0, 6, 4]
obstacles:
  - { type: wall, x1: 0, y1: 0, x2: 6, y2: 0 }
  - { type: box, x: 3, y: 2, width: 1, depth: 0.5, yaw: 30, dynamic: true }
  - { type: cylinder, x: 5, y: 3, radius: 0.2 }
robots:
  - { name: robot1, type: diffdrive, preset: waffle, x: 1, y: 1, sensors: { camera: true } }
waypoints:
  - { name: dock, x: 0.5, y: 3.5, yaw: 180 }
```

Muat dengan `WorldDefinition.FromFile("lab.yaml").Build()`.

## Antarmuka ROS (`SimulationNode`)

Untuk setiap robot, di namespace-nya (`/robot1`, atau `/` bila `Namespace = "/"`):

| Arah | Topik | Tipe |
|---|---|---|
| sub | `cmd_vel` | `geometry_msgs/Twist` |
| pub | `odom` | `nav_msgs/Odometry` (`robot1/odom` → `robot1/base_link`) |
| pub | `scan` | `sensor_msgs/LaserScan` (QoS sensor data) |
| pub | `imu` | `sensor_msgs/Imu` |
| pub | `gps/fix` | `sensor_msgs/NavSatFix` |
| pub | `battery_state` | `sensor_msgs/BatteryState` |
| pub | `sonar_front`, `sonar_rear` | `sensor_msgs/Range` |
| pub | `camera/image_raw` | `sensor_msgs/Image` (rgb8) |
| pub | `joint_states` | `sensor_msgs/JointState` (roda, kemudi, lengan) |
| pub | `ground_truth/pose` | `geometry_msgs/PoseStamped` dalam frame `map` |
| srv | `teleport` | `turtlesim/srv/TeleportAbsolute` |

Topik dan service global:

- `/tf`: odom → base_link untuk setiap robot. Dengan `PublishMapToOdom`, topik ini juga membawa transform map → odom dari ground truth.
- `/tf_static`: posisi pemasangan sensor, latched.
- `/clock`: menggerakkan node yang dijalankan dengan `use_sim_time:=true`.
- `/map`: occupancy grid ground-truth, latched.
- Service: `/sim/reset` (`std_srvs/Empty`), `/sim/pause` (`std_srvs/SetBool`), `/sim/spawn` (semantik `turtlesim/Spawn`) dan `/sim/kill`.

Frame TF diberi awalan nama robot (`robot1/base_link`), sesuai yang diharapkan konfigurasi Nav2 multi-robot.

Data sensor dipublikasikan dari thread stepping. `cmd_vel` dibaca tanpa executor, jadi robot tetap bisa dikendalikan meski node tidak di-spin. Service tetap membutuhkan spin; `RunAsync` sudah menangani keduanya. Untuk mengatur waktu sendiri, misalnya di test atau RL lock-step, set `RunWorld = false` lalu panggil `world.Step()`.

## Mode turtlesim

`TurtlesimNode` adalah pengganti langsung `turtlesim_node`. Isinya:

- Topik: `/turtle1/cmd_vel`, `/turtle1/pose` dan `/turtle1/color_sensor`.
- Service: `/spawn`, `/kill`, `/clear` dan `/reset`.
- Per kura-kura: `set_pen`, `teleport_absolute`, `teleport_relative` dan action `rotate_absolute`.

Karena itu, tutorial pemula ROS 2 standar bisa dijalankan tanpa perubahan. `TurtleWorld.Snapshot()` menyediakan data kura-kura dan goresan pena untuk ditampilkan di UI.

## API rendering (untuk UI)

`SimWorld.Snapshot()` mengembalikan `WorldSnapshot` yang immutable dan murah dipanggil setiap frame. Array `StaticObstacles` dipakai bersama antar-snapshot sampai geometri statis berubah.

- `ObstacleSnapshot`: `Kind`, `Center`, `Rotation`, `Size`, `Radius`, `Vertices` dalam koordinat dunia, `Height`, `Color`, `IsDynamic`, `Tag`.
- `RobotSnapshot`: `Pose`, `OdomPose`, kecepatan, `Radius`, `Length`, `Width`, `Height`, `Color`, `Bumper`, `BatteryPercentage`, `SteeringAngle`, `LaserOrigin` dan `LaserHits` dalam koordinat dunia (untuk menggambar sinar), `Trail`, serta `JointNames`, `JointPositions` dan `ArmLinkLengths` milik lengan.
- Event: `Stepped` (di thread stepping), `WasReset` dan `RobotsChanged`.

## Keterbatasan

- Fisikanya kinematik dengan resolusi kontak sederhana, bukan dinamika benda tegar: robot tidak bisa terguling, dan peti tidak berputar saat didorong.
- Kameranya berupa tampilan sintetis dari atas, bukan render perspektif.
