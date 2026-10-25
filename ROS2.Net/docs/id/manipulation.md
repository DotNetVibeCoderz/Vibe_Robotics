# Manipulasi (ala MoveIt) — `ROS2.Net.Manipulation`

`ROS2.Net.Manipulation` mencakup inti alur kerja MoveIt 2 dalam kode managed:

- parsing **URDF** dan padanan `robot_state_publisher`;
- **kinematika**: forward kinematics, Jacobian geometris, dan inverse kinematics numerik yang menghormati batas joint;
- **pengecekan tabrakan** terhadap planning scene;
- **perencanaan**: RRT-Connect, jalur Kartesius lurus, dan parameterisasi waktu;
- fasad **MoveGroup** yang mengeksekusi rencana lewat `control_msgs/FollowJointTrajectory`;
- pipeline **pick-and-place**.

```
dotnet add package ROS2.Net.Manipulation
```

## Mulai cepat

```csharp
using ROS2.Net.Manipulation;

var model = UrdfModel.LoadBuiltIn("ur5");                 // bawaan: ur5, diffbot, planar2 — atau UrdfModel.LoadFile(path)
var arm = new KinematicChain(model, "base_link", "tool0");

var pose = arm.ForwardKinematics([0, -1.2, 1.5, -1.9, -1.57, 0]);
var ik = new IkSolver(arm).Solve(pose with { Position = pose.Position + new Vec3(0, 0, -0.1) });
Console.WriteLine(ik.Success ? string.Join(", ", ik.Solution) : "tidak terjangkau");
```

Contoh berikut membutuhkan ROS yang sedang berjalan, misalnya `JointTrajectoryController` milik ROS2.Net di atas `GenericSystem`:

```csharp
var node = context.CreateNode("mover");
var group = new MoveGroup(node, arm);                      // state saat ini dari /joint_states
group.Scene.AddBox("table", new Vec3(0.5, 0, -0.05), new Vec3(1.0, 1.0, 0.1));
await group.MoveToPoseAsync(Pose3.FromXyzRpy(0.4, 0.2, 0.3, Math.PI, 0, 0));
```

## URDF dan robot_state_publisher

`UrdfModel.Parse(xml)` membaca:

- link beserta inersia dan geometri visual/collision (box, cylinder, sphere, mesh);
- semua tipe joint (fixed, revolute, continuous, prismatic, floating, planar), lengkap dengan origin `xyz`/`rpy`, sumbu, batas, dan mimic.

Parser juga memastikan model hanya memiliki satu root.

`new RobotStatePublisher(context, model)` berperilaku seperti `robot_state_publisher`:

- menerbitkan URDF di `/robot_description` (latched) dan sebagai parameter read-only;
- menerbitkan joint fixed sekali di `/tf_static`;
- menerbitkan joint yang bergerak di `/tf` berdasarkan `/joint_states`. Joint mimic ikut dihitung, dan `frame_prefix` didukung.

## Kinematika

- `KinematicChain(model, base, tip)` adalah rantai serial atas joint-joint yang bisa bergerak, berurutan dari base ke tip.
  - `ForwardKinematics(q)` mengembalikan pose ujung (tip).
  - `LinkPoses(q)` mengembalikan pose setiap link di sepanjang rantai.
  - `Jacobian(q, span)` mengisi Jacobian geometris 6×n (baris linear dulu, lalu angular) dalam frame base.
  - Joint mimic di dalam rantai mengikuti joint master-nya.
- `RobotKinematics(model)` menghitung forward kinematics untuk seluruh pohon (setiap link).
- `IkSolver` memakai **Levenberg–Marquardt** (bawaan, dengan damping adaptif) atau **damped least squares**.
  - Hasilnya di-clamp ke batas joint.
  - Solver mencoba ulang dari konfigurasi acak bila seed gagal.
  - Opsinya meliputi `PositionOnly`, toleransi, dan `Validator`, misalnya untuk mewajibkan solusi bebas tabrakan.
- `IkSolver.TwoLinkPlanar(x, y, l1, l2, elbowUp)` memberi solusi tertutup (closed-form) untuk lengan planar 2-link.

Tipe matematika yang tersedia:

- `Vec3`;
- `Quat`, dengan konversi RPY, slerp, dan rotation vector;
- `Pose3`, dengan komposisi, invers, matriks 4×4, serta konversi ke dan dari geometry_msgs.

## Pengecekan tabrakan

- **Primitif**: `SphereShape`, `CapsuleShape`, dan `BoxShape` (berorientasi, dengan uji separating axis).
  - Jarak antarpasangan sphere/capsule dihitung eksak.
  - Box–capsule memakai pencarian garis (line search) konveks.
- **Geometri URDF**: cylinder diubah menjadi capsule, yang konservatif di bagian ujungnya. **Mesh diabaikan**, jadi dekati dengan primitif pada tag `<collision>`.
- **`PlanningScene`** menyimpan:
  - objek dunia (`AddBox`, `AddSphere`, `AddWorldObject`);
  - objek yang menempel di robot (`Attach`/`Detach`, dengan touch link);
  - allowed collision matrix. Link yang bersebelahan diizinkan secara bawaan.

  Fungsi pendukungnya:
  - `AllowCollisionsAt(state)` menonaktifkan pasangan link yang sudah bersentuhan pada pose acuan, seperti pasangan "default" di MoveIt setup assistant.
  - `Padding` menambahkan margin keamanan.
  - `IsStateValid(chain, q)` memeriksa sebuah state.

## Perencanaan

| Kelas | Fungsi |
|---|---|
| `RrtConnectPlanner` | RRT-Connect dua arah di ruang joint. Setiap edge dicek tabrakannya pada `EdgeResolution`, lalu jalurnya dihaluskan dengan shortcut. Hasilnya deterministik karena memakai seed. Opsi: ukuran langkah, iterasi, dan timeout. |
| `CartesianPlanner` | Gerak ujung yang lurus: posisi linear dan orientasi slerp. IK setiap langkah memakai solusi sebelumnya sebagai seed. Planner menjaga agar joint tidak melompat dan melaporkan `Fraction`, yaitu bagian jalur yang bisa dicapai. |
| `TimeParameterization` | `trajectory_msgs/JointTrajectory` bertimestamp dengan segmen **trapesium** (time-optimal) atau **quintic**. Semua joint disinkronkan per segmen dan tetap dalam batas kecepatan/percepatan, dengan faktor skala. |
| `MoveGroup` | `PlanToJointTarget`, `PlanToPose` (IK bebas tabrakan lalu RRT), `PlanCartesian`, `ExecuteAsync` (lewat action FollowJointTrajectory), dan `MoveTo…Async`. |
| `PickAndPlace` | Buka → pre-grasp → approach → grasp (attach) → retreat → pre-place → place → release (detach) → retreat. Gripper diberikan sebagai delegate, misalnya pembungkus `GripperActionController`. |

## Perbedaan dengan MoveIt

MoveIt sangat besar. Paket ini adalah bagian yang ringkas dan bebas dependensi:

- **IK** hanya numerik. Tidak ada IKFast atau TRAC-IK, dan SRDF tidak di-parse, sehingga tidak ada konfigurasi solver per grup.
- **Pengecekan tabrakan** hanya memakai primitif. Tidak ada FCL, geometri mesh, maupun octomap, dan tidak ada continuous collision checking.
- **Perencanaan** hanya tersedia RRT-Connect dan jalur Kartesius. Tidak ada kumpulan planner OMPL, perencanaan dengan constraint, maupun CHOMP/STOMP/Pilz.
- **Parameterisasi waktu** berhenti di setiap waypoint (rest-to-rest). Belum ada blending antar-waypoint seperti TOTG atau Ruckig.
- **MoveGroup** adalah API di dalam proses. Tidak ada node `move_group` maupun antarmuka action MoveIt.

---

Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.
