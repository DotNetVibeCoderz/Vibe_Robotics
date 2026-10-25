# Kontrol (ros2_control) — `ROS2.Net.Control`

`ROS2.Net.Control` menghadirkan arsitektur ros2_control ke .NET:

- Komponen hardware mengekspor **state interface** dan **command interface**.
- **Controller manager** menjalankan loop real-time `read → update → write`.
- Setiap controller mengklaim command interface secara eksklusif.

Controller manager menyediakan service standar `controller_manager_msgs`, sehingga `ros2 control list_controllers` bisa dipakai terhadap proses ROS2.Net.

```
dotnet add package ROS2.Net.Control
```

## Mulai cepat: robot differential drive

```csharp
using ROS2.Net;
using ROS2.Net.Control;

await using var context = await Ros2Context.CreateAsync();

var resources = new ResourceManager();
resources.Add(new SimulatedDiffDriveHardware());          // atau HardwareComponent buatan Anda

var cm = new ControllerManager(context, resources);        // node "controller_manager", update_rate 100 Hz
cm.Spawn(new JointStateBroadcaster(), "joint_state_broadcaster");
cm.Spawn(new DiffDriveController(), "diff_drive_controller",
[
    new Parameter("left_wheel_names", new[] { "left_wheel_joint" }),
    new Parameter("right_wheel_names", new[] { "right_wheel_joint" }),
    new Parameter("wheel_separation", 0.34),
    new Parameter("wheel_radius", 0.05),
]);

cm.Start();                       // thread kontrol khusus berprioritas tinggi
await context.SpinAsync();        // service, subscription cmd_vel, publisher
```

Gerakkan robot dengan `ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`.

## Konsep

| Bagian | Peran |
|---|---|
| `InterfaceSlot` | Sebuah field `double` bersama. Hardware menulisnya, controller membaca/menulis lewat handle tanpa salinan dan tanpa lock. |
| `StateInterface` / `CommandInterface` | Handle bernama `joint/position`, `joint/velocity`, `joint/effort`, dan seterusnya. |
| `IHardwareInterface` / `HardwareComponent` | Hardware dengan lifecycle (`Configure`, `Activate`, `Deactivate`) serta `Read`/`Write` di setiap siklus. |
| `GenericSystem` | Hardware tiruan seperti `mock_components/GenericSystem`: menyalin command ke state, dengan opsi lag orde satu (`TimeConstant`), dan mengintegrasikan command kecepatan. |
| `SimulatedDiffDriveHardware` | Dua roda dengan command kecepatan dan state posisi/kecepatan. |
| `ResourceManager` | Pemilik hardware dan interface. Klaim bersifat semua-atau-tidak-sama-sekali dan eksklusif. |
| `ControllerManager` | `Node` yang memuat, mengonfigurasi, mengganti (switch), dan menjalankan controller. |
| `ControllerBase` | Kelas dasar controller: satu node per controller, `CommandInterfaceConfiguration`, `StateInterfaceConfiguration`, `Update(time, period)`. |
| `RealtimeBuffer<T>` | Serah-terima tanpa lock dari callback subscription ke loop kontrol. |

### Lifecycle controller

Urutannya `unconfigured → (configure) → inactive → (activate lewat switch) → active`.

`SwitchController(activate, deactivate, strictness)` diterapkan di antara dua siklus:

- Mode **strict** memeriksa semua klaim lebih dulu dan tidak mengubah apa pun bila ada yang gagal.
- Mode **best effort** menjalankan apa yang bisa dijalankan.

Menonaktifkan dan mengaktifkan dalam satu panggilan memungkinkan sebuah controller mengambil alih interface yang dilepas controller lain.

### Parameter

Setiap controller memiliki node dengan nama controller itu, jadi parameter YAML ditulis di bawah nama controller:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200
    diff_drive_controller.type: diff_drive_controller/DiffDriveController
diff_drive_controller:
  ros__parameters:
    left_wheel_names: [left_wheel_joint]
    right_wheel_names: [right_wheel_joint]
    wheel_separation: 0.34
    wheel_radius: 0.05
```

Berikan file ini dengan `--ros-args --params-file controllers.yaml`, lalu panggil `cm.LoadController("diff_drive_controller")` (tipenya diambil dari `<nama>.type`) atau service `~/load_controller`. Parameter yang ditulis pada manager sebagai `<controller>.<param>` juga diteruskan ke controller.

## Service

| Service | Tipe |
|---|---|
| `~/list_controllers` | `controller_manager_msgs/srv/ListControllers` |
| `~/list_hardware_interfaces` | `controller_manager_msgs/srv/ListHardwareInterfaces` |
| `~/load_controller`, `~/configure_controller`, `~/unload_controller` | `controller_manager_msgs/srv/*Controller` |
| `~/switch_controller` | `controller_manager_msgs/srv/SwitchController` |

Layout pesan mengikuti **Humble**. `SwitchController` masih membawa `start_controllers`/`stop_controllers`, yang digabung dengan `activate_`/`deactivate_controllers`. Jazzy menghapus field tersebut, sehingga klien `ros2 control` versi Jazzy tidak akan cocok dengan service ini. Ini adalah kekurangan yang sudah diketahui.

## Controller bawaan

| Nama tipe | Kelas | Fitur utama |
|---|---|---|
| `joint_state_broadcaster/JointStateBroadcaster` | `JointStateBroadcaster` | `/joint_states` dari semua state posisi/kecepatan/effort; `publish_rate` (0 = setiap siklus), filter `joints`. |
| `forward_command_controller/ForwardCommandController`, `position_controllers/JointGroupPositionController`, `velocity_controllers/JointGroupVelocityController`, `effort_controllers/JointGroupEffortController` | `ForwardCommandController` | `~/commands` (`std_msgs/Float64MultiArray`) → satu interface per joint. |
| `diff_drive_controller/DiffDriveController` | `DiffDriveController` | `~/cmd_vel` (Twist, atau TwistStamped dengan `use_stamped_vel`), odometri RK2 dari posisi roda (atau `open_loop`), `~/odom` + odom→base_link di `/tf`, batas kecepatan & percepatan `linear.x.*`/`angular.z.*`, `cmd_vel_timeout`. |
| `ackermann_steering_controller/AckermannSteeringController` | `AckermannSteeringController` | Model sepeda (bicycle model): `traction_joints` (kecepatan), `steering_joints` (posisi), `wheelbase`, `wheel_radius`, `max_steering_angle`. |
| `joint_trajectory_controller/JointTrajectoryController` | `JointTrajectoryController` | `control_msgs/action/FollowJointTrajectory` di `~/follow_joint_trajectory` dan topic `~/joint_trajectory`; spline linear/kubik/quintic sesuai isi titik; toleransi jalur & tujuan (`constraints.<joint>.trajectory/goal`, `constraints.goal_time`, `stopped_velocity_tolerance`); goal baru menggantikan goal lama; command interface posisi atau kecepatan (feed-forward + `gains.<joint>.p`). |
| `position_controllers/GripperActionController` | `GripperActionController` | `~/gripper_cmd` (`ros2net_control_msgs/action/GripperCommand`: `position`, `max_effort` → `position`, `effort`, `stalled`, `reached_goal`), deteksi macet (stall) dari kecepatan atau effort. |

`PidController` adalah PID diskret yang bisa dipakai ulang. Fiturnya: clamp integral, anti-windup dengan integrasi bersyarat, derivative-on-measurement, filter derivatif, dan batas output.

## Menulis controller sendiri

```csharp
public sealed class HoldController : ControllerBase
{
    private string[] _joints = [];

    public override InterfaceConfiguration CommandInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_joints.Select(j => j + "/position"));
    public override InterfaceConfiguration StateInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_joints.Select(j => j + "/position"));

    protected override void OnInit() => Node.DeclareParameter("joints", Array.Empty<string>());

    protected override CallbackReturn OnConfigure()
    {
        _joints = Node.GetParameter<string[]>("joints");
        return CallbackReturn.Success;
    }

    public override ControllerReturn Update(RosTime time, RosDuration period)
    {
        for (var i = 0; i < CommandInterfaces.Count; i++)
            CommandInterfaces[i].Value = StateInterfaces[i].Value;   // tanpa alokasi di sini
        return ControllerReturn.Ok;
    }
}

cm.RegisterControllerType("my_pkg/HoldController", () => new HoldController());
```

## Catatan real-time

- `ControllerManager.Start()` menjalankan loop di thread khusus berprioritas `ThreadPriority.Highest`. Thread tidur secara kasar lalu melakukan spin pada ~2 ms terakhir agar periodenya akurat (resolusi timer Windows dinaikkan ke 1 ms). `OverrunCount` menghitung siklus yang terlambat.
- Untuk simulasi dan pengujian yang deterministik, panggil `cm.Update(time, period)` sendiri.
- Perhitungan di jalur update (akses interface, PID, odometri, sampling spline) tidak melakukan alokasi.
- Publikasi pesan (`/joint_states`, `/odom`, `/tf`) mengalokasikan satu pesan per publish. Batasi frekuensinya dengan `publish_rate`.
- Command dari topic dan action sampai ke loop melalui `RealtimeBuffer<T>`, yaitu pertukaran referensi secara atomik.

## Perbedaan dengan ros2_control

Ini adalah implementasi ulang dalam kode managed, bukan binding:

- Hardware dan controller berupa kelas .NET yang didaftarkan lewat kode. Tidak ada pluginlib dan tag `<ros2_control>` di URDF tidak di-parse.
- Belum ada chained controller maupun reference interface. Nilai `is_chainable` selalu false.
- Belum ada service `list_controller_types`, `reload_controller_libraries`, maupun `set_hardware_component_state`.
- Throttle `update_rate` per controller disederhanakan.

---

Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.
