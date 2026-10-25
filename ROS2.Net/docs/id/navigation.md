# Navigasi — Nav2 untuk .NET

`ROS2.Net.Navigation` adalah tumpukan navigasi bergaya Nav2: peta dan costmap berlapis, global planner, local controller, perilaku pemulihan (*recovery*), mesin behavior tree, lokalisasi AMCL dan EKF, serta node server dengan nama topik, aksi dan pesan yang sama dengan Nav2. Klien Nav2 standar seperti alat "Nav2 Goal" di RViz, `ros2 action send_goal` atau `BasicNavigator` di Python dapat mengendalikannya. BT navigator-nya juga dapat mengendalikan planner server dan controller server Nav2 standar.

```bash
dotnet add package ROS2.Net.Navigation
```

Paket ini hanya bergantung pada `ROS2.Net.Core` dan `ROS2.Net.Msgs`. Paket ini memakai pose planar (`Pose2D`, SE(2)), bukan TF. Pose robot diambil dari sebuah `IRobotPoseProvider`, yang dijelaskan di bawah.

## Peta dan costmap

```csharp
var map = OccupancyGridMap.LoadMapServer("maps/office.yaml");   // YAML map_server + PGM (P5/P2)
map.SaveMapServer("maps/office_copy");                          // menulis .yaml + .pgm (trinary)
double range = map.Raycast(x, y, angle, maxRange: 10);          // ray cast DDA yang presisi

var costmap = Costmap2D.FromMap(map, trackUnknown: true)
    .AddLayer(new StaticLayer(map, trackUnknown: true))
    .AddLayer(new ObstacleLayer())                              // menandai/membersihkan dari LaserScan
    .AddLayer(new InflationLayer(inflationRadius: 0.55, costScalingFactor: 3.0, inscribedRadius: 0.22));
costmap.GetLayer<ObstacleLayer>()!.AddScan(costmap, scan, sensorPose);
costmap.Update();
```

Nilai biaya mengikuti `nav2_costmap_2d`:

| Biaya | Arti |
|---|---|
| 0 | bebas |
| 1–252 | inflasi |
| 253 | inscribed (footprint menyentuh rintangan) |
| 254 | lethal |
| 255 | tidak diketahui |

Inflasi memakai rumus Nav2 `252 · exp(−cost_scaling_factor · (d − inscribed_radius))`. `Costmap2D.ToMsg()` menerapkan konversi Nav2 ke `nav_msgs/OccupancyGrid`, sehingga costmap bisa ditampilkan di RViz.

`Footprint.Circle(r)`, `Footprint.Rectangle(l, w)` dan `Footprint.FromPolygon(...)` menyediakan pengecekan `Cost()` dan `InCollision()`.

## Global planner (`IGlobalPlanner`)

| Planner | Keterangan |
|---|---|
| `AStarPlanner` | 8-arah, heuristik oktil, tanpa memotong sudut, memperhitungkan biaya (`CostWeight`) |
| `DijkstraPlanner` | pencarian seragam ala NavFn |
| `ThetaStarPlanner` | jalur bersudut bebas (*any-angle*) lewat pintasan parent yang saling terlihat (*line of sight*) |
| `HybridAStarPlanner` | ala Smac: pencarian (x, y, θ) dengan primitif busur pada `MinTurningRadius`, opsi mundur (`AllowReverse`), heuristik 2-D yang sadar rintangan dan ekspansi analitik Dubins |
| `PathSmoother` | penghalus *gradient descent* yang menjaga titik ujung dan menghindari rintangan |

```csharp
PlanResult plan = new AStarPlanner().CreatePlan(costmap, start, goal);
if (plan.Success) path.Publish(PathUtils.ToMsg(plan.Poses, "map", RosTime.Now()));
```

Kegagalan perencanaan tidak melempar exception, melainkan mengembalikan `Success = false` beserta `Error`: "no path found", "goal is in an obstacle", "start is outside the costmap", dan sebagainya. Buffer pencarian dipakai ulang, sehingga perencanaan ulang pada costmap yang sama tidak melakukan alokasi per sel.

## Controller (`IController`)

| Controller | Keterangan |
|---|---|
| `PurePursuitController` | busur melalui titik lookahead (κ = 2y/L²), berputar di tempat jika jalur ada di belakang robot |
| `RegulatedPurePursuitController` | menambahkan lookahead yang diskalakan dengan kecepatan, dan melambat saat kelengkungan tajam, saat ada biaya inflasi di dekat robot dan saat mendekati tujuan |
| `DwbController` | mencuplik (v, ω) dalam jendela dinamis, mensimulasikan masing-masing ke depan, lalu menilai lintasannya dengan *critic* (jarak ke jalur, jarak ke tujuan, keselarasan dengan jalur, rintangan, preferensi maju) |

Gunakan bersama `GoalChecker` (toleransi xy/yaw, stateful seperti `SimpleGoalChecker` di Nav2) dan `ProgressChecker` (`SimpleProgressChecker` di Nav2).

## Perilaku pemulihan

`SpinBehavior`, `BackUpBehavior`, `WaitBehavior` dan `ClearCostmapBehavior` mengimplementasikan `IRecoveryBehavior`. Mulai perilaku dengan `Start(pose)`, lalu panggil `Update(pose, dt, out cmd)` sampai hasilnya `Succeeded` atau `Failed`. Spin dan back-up menerima pengecekan `IsColliding` opsional.

## Behavior tree

`BtFactory` memuat XML BehaviorTree.CPP dalam bentuk v3 (`<Action ID="X"/>`) maupun v4 (`<X/>`):

- **Node kontrol bawaan:** `Sequence`, `ReactiveSequence`, `Fallback`, `ReactiveFallback`, `Parallel` (`success_count`/`failure_count`), serta `PipelineSequence`, `RoundRobin` dan `RecoveryNode` (`number_of_retries`) dari Nav2.
- **Dekorator bawaan:** `Inverter`, `ForceSuccess`, `ForceFailure`, `RetryUntilSuccessful`/`Retry` (`num_attempts`), `KeepRunningUntilFailure`, `RateController` (`hz`) dan `Repeat`.
- **Lainnya:** `SubTree`.

Port memakai `{key}` untuk blackboard dan teks biasa untuk nilai literal.

```csharp
var factory = new BtFactory()
    .RegisterCondition("BatteryOk", n => battery > n.GetInput("min", 0.2))
    .RegisterAsyncAction("Dock", async (n, ct) => await DockAsync(ct) ? NodeStatus.Success : NodeStatus.Failure);
var tree = factory.CreateTreeFromText(xml);
tree.Blackboard.Set("goal", pose);
var status = await tree.TickWhileRunningAsync(TimeSpan.FromMilliseconds(100), ct);
```

Aksi async melaporkan `Running` sampai task-nya selesai. Menghentikannya (*halt*), misalnya oleh parent yang reaktif, membatalkan token-nya. `NavTrees.NavigateToPoseWithReplanningAndRecovery` setara dengan tree bawaan Nav2: tree ini merencanakan ulang setiap 1 Hz sambil mengikuti jalur, dan memulihkan diri dengan membersihkan costmap, berputar, menunggu dan mundur.

## Lokalisasi

- **`Amcl`** adalah particle filter dengan model gerak odometri (`alpha1`–`alpha4`) dan model laser likelihood-field. Filter ini memakai resampling adaptif KLD (`MinParticles`/`MaxParticles`, `PfErr`, `PfZ`) dan pemulihan partikel acak yang opsional (`RecoveryAlphaSlow`/`Fast`).
  - Odometri diakumulasikan dan diterapkan sekali per pembaruan, setelah robot bergerak sejauh `UpdateMinD` / `UpdateMinA`, sama seperti `nav2_amcl`.
  - `LaserModel.LikelihoodFieldProduct` (bawaan) adalah perkalian probabilitas beam yang diredam (*tempered*) dan cepat konvergen. `LaserModel.LikelihoodField` meniru kombinasi `1 + Σpz³` milik nav2.
- **`Ekf2D`** adalah EKF bergaya robot_localization untuk (x, y, θ, v, ω). Filter ini menggabungkan subset pengukuran apa pun: `UpdateTwist` (odometri roda), `UpdateYawRate` (giroskop), `UpdateYaw` (kompas/orientasi IMU) dan `UpdatePosition` (GPS/AMCL).

## Node ROS

| Node | Nama bawaan | Antarmuka |
|---|---|---|
| `MapServerNode` | `map_server` | `/map` dan `/map_metadata` (latched); layanan `/map_server/map` (`nav_msgs/srv/GetMap`) |
| `PlannerServerNode` | `planner_server` | aksi `compute_path_to_pose` (`nav2_msgs/action/ComputePathToPose`), menerbitkan `/plan`; costmap global dibangun dari `/map`; `AddPlanner(id, planner)` (id bawaan `GridBased` = A*) |
| `ControllerServerNode` | `controller_server` | aksi `follow_path` (`nav2_msgs/action/FollowPath`) → `/cmd_vel` pada `controller_frequency`; goal baru menggantikan goal lama; `AddController(id, controller)` (bawaan `FollowPath` = RPP) |
| `BtNavigatorNode` | `bt_navigator` | aksi `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) dengan feedback; BT-nya memanggil `compute_path_to_pose` dan `follow_path` |
| `AmclNode` | `amcl` | `/map`, `/scan`, `/initialpose`, odometri → `/amcl_pose`, `/particle_cloud`, `map→odom` di `/tf` |
| `EkfNode` | `ekf_filter_node` | `/odom` + `/imu` → `/odometry/filtered` |
| `SlamNode` | `slam_toolbox` | `/scan` + odometri → `/map`, `map→odom` di `/tf` (lihat [slam.md](slam.md)) |

### Pose robot: `IRobotPoseProvider`

Server-server membaca pose robot dari sebuah `IRobotPoseProvider`, bukan dari TF:
- `OdometryPoseProvider` mengikuti `/odom` tanpa perlu executor. Provider ini melaporkan `Correction ⊕ odom`, sehingga koreksi map→odom dari AMCL atau SLAM bisa dimasukkan: `amcl.CorrectionUpdated += c => pose.Correction = c;`.
- `DelegatePoseProvider` membungkus fungsi apa saja, misalnya simulator, buffer TF atau motion capture.

### Tumpukan lengkap dalam satu proses

```csharp
await using var ctx = await Ros2Context.CreateAsync();
var tools = ctx.CreateNode("nav_app");
using var pose = new OdometryPoseProvider(tools);

new MapServerNode(ctx, OccupancyGridMap.LoadMapServer("map.yaml"));
new PlannerServerNode(ctx, pose);
new ControllerServerNode(ctx, pose);
new BtNavigatorNode(ctx, pose);
await ctx.SpinAsync(ct, new MultiThreadedExecutor());
```

Kirim goal dari C# dengan `CreateActionClient<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback>("/navigate_to_pose")`. Goal juga bisa dikirim dari alat ROS 2 mana pun, asalkan memakai middleware yang sama.

## Perbedaan dengan Nav2

- Perilaku pemulihan berjalan di dalam BT navigator lewat `/cmd_vel`. Tidak ada behavior server terpisah, dan tidak ada aksi Spin/BackUp/Wait dari `nav2_msgs`.
- `ClearEntireCostmap` memanggil `BtNavigatorNode.ClearCostmaps`, bukan layanan costmap.
- `GoalUpdated` selalu bernilai false. Untuk mengganti tujuan, kirim goal baru.
- Belum ada local costmap dengan jendela bergulir (*rolling window*), filter keepout/speed, maupun collision monitor. Costmap planner server memakai layer static, obstacle dan inflation.
- `nav2_msgs` didefinisikan mengikuti tata letak Humble (hasil `NavigateToPose` berupa `std_msgs/Empty`). Distro yang lebih baru menambahkan kode error pada hasilnya.
- Server-server ini adalah `Node` biasa, bukan lifecycle node, sehingga tidak ada jabat tangan dengan `lifecycle_manager`.

## Pengujian

`tests/ROS2.Net.Navigation.Tests` mencakup:
- round trip peta dan PGM, nilai inflasi, serta penandaan/pembersihan rintangan
- setiap planner pada peta dinding bercelah, termasuk kegagalan saat celah tertutup, dan batas kelengkungan jalur Hybrid-A*
- ketiga controller mencapai tujuan pada jalur berbentuk L
- perilaku pemulihan dan semantik mesin BT
- konvergensi AMCL, EKF dibandingkan dead reckoning, dan pose graph
- uji ujung-ke-ujung: map server, planner, controller dan BT navigator mengemudikan robot kinematik mengitari rintangan lewat aksi `NavigateToPose`

Interoperabilitas dengan graph Nav2/ROS 2 sungguhan bergantung pada middleware (RTPS atau rosbridge), dan belum diverifikasi pada instalasi ROS 2 sungguhan.
