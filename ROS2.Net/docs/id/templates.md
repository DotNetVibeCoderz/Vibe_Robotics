# Template proyek

> ROS2.Net — dibuat oleh **Gravicode Studios** dipimpin oleh **Kang Fadhil**.

```bash
dotnet new install ROS2.Net.Templates
dotnet new list ros2net
```

| Template | Nama pendek | Isinya |
|---|---|---|
| ROS2.Net Node | `ros2net-node` | Aplikasi Generic Host dengan `TalkerNode` + `ListenerNode` (`AddRos2`, `AddRos2Node<T>`), parameter `rate_hz` yang bisa diubah saat berjalan, dan middleware yang dipilih lewat `appsettings.json`. Opsi `--rmw rtps\|inprocess\|rosbridge`. |
| ROS2.Net Lifecycle Node | `ros2net-lifecycle` | Node terkelola `sensor_driver` (configure/activate/deactivate/cleanup/shutdown) dengan lifecycle publisher; `--autostart` untuk langsung aktif saat dijalankan. |
| ROS2.Net Interfaces Library | `ros2net-interfaces` | Class library berisi contoh file `.msg/.srv/.action` yang dikompilasi menjadi C# oleh `ROS2.Net.Rosidl.Generator`. Opsi `--rosPackage my_robot_msgs` menentukan nama paket ROS (folder dan namespace). |
| ROS2.Net Blazor App | `ros2net-blazor` | Dashboard Blazor Server: tombol teleop (`/cmd_vel`), pose (`/odom`), log `/chatter` langsung, dan daftar topic, dengan satu context ROS yang di-host. |
| ROS2.Net Avalonia App | `ros2net-avalonia` | Dashboard desktop Avalonia 12 dengan panel teleop/pose/chatter yang sama. |

Semua template menerima `--rosVersion` (bawaan `0.1.0`) untuk versi paket ROS2.Net.

## Mulai cepat

```bash
dotnet new ros2net-node -n RobotSaya --rmw inprocess
cd RobotSaya && dotnet run
# terminal lain (dengan --rmw rtps):  ros2net topic echo /chatter
```

```bash
dotnet new ros2net-interfaces -n FleetInterfaces --rosPackage fleet_msgs
dotnet build FleetInterfaces     # → ROS2.Net.Msgs.FleetMsgs.BatteryPack, DockRequest, PatrolGoal, …
```

## Cara verifikasinya

Paket template di-pack, dipasang ke hive template terpisah, lalu setiap template dibuat dan
di-build terhadap paket ROS2.Net yang di-pack secara lokal (0 warning). `ros2net-node` dan
`ros2net-lifecycle` dijalankan di middleware in-process; library interface diperiksa berisi tipe
hasil generate (juga dengan `--rosPackage` kustom). Aplikasi Blazor dan Avalonia sudah di-build
tetapi belum dicoba secara interaktif.
