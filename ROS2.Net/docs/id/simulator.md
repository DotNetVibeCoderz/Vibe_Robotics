# Robot Simulator (aplikasi desktop)

> Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.

`apps/ROS2.Net.Simulator` adalah antarmuka desktop untuk `ROS2.Net.Simulation`. Pilih skenario, kemudikan robot, dan amati sensornya. Setiap robot diterbitkan sebagai topic ROS 2 standar di middleware yang Anda pilih, sehingga node eksternal dapat memakai simulator ini seolah-olah perangkat keras sungguhan: Nav2, RViz, `ros2 topic`, atau kode ROS2.Net Anda sendiri.

```bash
dotnet run --project apps/ROS2.Net.Simulator                                    # in-process
dotnet run --project apps/ROS2.Net.Simulator -- --rmw rtps --scenario Office    # terlihat oleh ROS 2 di LAN
dotnet run --project apps/ROS2.Net.Simulator -- --rmw rosbridge                 # lewat rosbridge_server
```

![Simulator](../images/desktop/simulator.png)

## Fitur

| Bagian | Yang Anda dapatkan |
|---|---|
| Skenario | Warehouse (dua robot), Office, Maze, Corridor loop, Agriculture rows, Empty |
| Tampilan | **2D**: tampilan dari atas dengan rintangan, sinar LiDAR, jejak, dan arah hadap; bisa geser dan zoom; klik robot untuk memilihnya. **3D**: tampilan GPU Three.Net dengan rintangan setinggi aslinya; seret untuk memutar, roda mouse untuk zoom. Tampilan ini baru dibuat saat tab dibuka. **Topics**: tabel topic langsung (Hz dan bandwidth) plus contoh perintah `ros2`. |
| Kontrol | Putar/jeda, step, reset, faktor waktu nyata 0,25–4×, **+ Robot** memunculkan robot baru di ruang kosong |
| Panel robot | Joystick ke `/<robot>/cmd_vel`, gauge baterai, gambar kamera, LiDAR, plot IMU (gyro z, accel x) |
| Middleware | `inprocess`, `rtps` (domain dari `ROS_DOMAIN_ID`), atau `rosbridge` (isian URL). **Apply** menjalankan ulang simulasi di middleware baru. |
| Status bar | Middleware, domain, waktu simulasi, jumlah robot, jumlah topic, faktor waktu nyata |

## Topic per robot

Topic-nya sama dengan `SimulationNode` (lihat [simulation.md](simulation.md)):

- `cmd_vel`
- `odom`
- `scan`
- `imu`
- `gps/fix`
- `battery_state`
- `sonar_front`, `sonar_rear`
- `camera/image_raw`
- `joint_states`
- `ground_truth/pose`

Topic global: `/tf`, `/tf_static`, `/clock`, dan `/map` yang di-latch. Simulator mengaktifkan `map → odom` agar pohon TF multi-robot saling terhubung.

```bash
# dari mesin ROS 2 di jaringan yang sama, dengan --rmw rtps
ros2 topic echo /robot1/odom
ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.4}}"
```

## Catatan

- Tampilan 3D membutuhkan renderer GPU (runtime native Three.Net). Bila tidak bisa dijalankan, tab menampilkan pesan dan tampilan 2D tetap berfungsi penuh. Screenshot headless hanya mencakup tampilan 2D.
- Interoperabilitas protokol middleware `rtps` dengan Fast DDS dan Cyclone DDS belum diverifikasi pada instalasi ROS 2 sungguhan. Lihat [rtps.md](rtps.md).
