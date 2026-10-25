# SLAM — membangun peta

`ROS2.Net.Navigation.Slam` adalah graph SLAM 2-D online yang terinspirasi dari slam_toolbox dan Karto. Modul ini bagian dari paket `ROS2.Net.Navigation`.

## Cara kerja

1. **Prediksi.** Untuk setiap scan, pose diprediksi dari keyframe terakhir ditambah perubahan odometri.
2. **Pencocokan scan ke peta.** Setelah robot bergerak sejauh `MinimumTravelDistance` atau berputar sebesar `MinimumTravelHeading`, scan dicocokkan dengan peta lokal bergulir yang dibangun dari `ScanBufferSize` keyframe terakhir. Pencocokan memakai scan matcher korelatif multi-resolusi (Olson 2009):
   - pencarian menyeluruh pada grid kasar hasil *max-pooling*
   - pencarian halus pada grid resolusi penuh
   - penyempurnaan sub-sel secara bilinear
3. **Keyframe.** Pose yang sudah dikoreksi menjadi node pose graph, yang terhubung ke node sebelumnya lewat edge odometri/scan-match.
4. **Penutupan loop (*loop closure*).** Keyframe yang umurnya minimal `LoopMatchMinimumChainSize` dan berada dalam `LoopSearchMaximumDistance` menjadi kandidat. Scan dicocokkan dengan submap kandidat itu pada jendela lebar (`LoopSearchSpaceDimension`, `LoopAngularWindow`). Jika skornya melampaui `LoopMatchMinimumResponse`, edge loop closure ditambahkan.
5. **Optimisasi.** Setelah setiap loop closure, `PoseGraph` menjalankan Levenberg-Marquardt di SE(2), dengan node pertama dibuat tetap.
6. **Peta.** Grid okupansi dibangun ulang secara malas (*lazy*) dari semua keyframe pada pose hasil optimisasi, dengan ray casting log-odds.

```csharp
var slam = new OnlineSlam(new SlamOptions { Resolution = 0.05, MaxLaserRange = 12 });
foreach (var (odomPose, scan) in stream)
{
    slam.Process(odomPose, scan);
}

OccupancyGridMap map = slam.GetMap();
slam.SaveMap("maps/lab");          // lab.yaml + lab.pgm (format map_server)
Pose2D correction = slam.MapToOdom; // terbitkan sebagai map → odom
```

`SlamMode.Localization` mencocokkan scan dengan graph yang sudah ada tanpa menambah keyframe. Komponen penyusunnya bersifat publik, sehingga bisa dipakai sendiri:
- `CorrelativeScanMatcher` dan `MatchGrid`
- `PoseGraph`, termasuk `Chi2()` dan `Optimize()`
- `ScanPoints.FromScan`

## Node ROS

`SlamNode` (nama bawaan `slam_toolbox`):
- berlangganan `/scan` (QoS sensor-data) dan `/odom`
- menerbitkan `/map` (latched) setiap `map_update_interval` detik (parameter, bawaan 2)
- menerbitkan `map → odom` di `/tf` setelah setiap scan
- `SaveMap(basePath)` menulis peta format map_server

Untuk bernavigasi sambil memetakan, masukkan `SlamNode.Slam.MapToOdom` ke `OdometryPoseProvider.Correction`.

## Batasan dibanding slam_toolbox

- Pose graph diselesaikan dengan Cholesky padat (*dense*). Ini cukup untuk ratusan keyframe. Peta yang sangat besar membutuhkan solver *sparse*, yang belum disertakan.
- Kandidat loop closure hanya diambil dari keyframe lama yang terdekat, tanpa pemungutan suara (*voting*) antarkandidat.
- Belum ada pemangkasan peta untuk lifelong mapping, serialisasi pose graph, mode interaktif, maupun penggabungan peta multi-robot.
- Peta dibangun ulang secara penuh setiap kali berubah, bukan secara inkremental.

## Pengujian

Rangkaian uji mengemudikan robot simulasi satu putaran mengelilingi koridor berbentuk loop berukuran 12 × 12 m, dengan odometri yang melenceng (galat skala sekitar 2–3 % dan bias yaw). Uji ini memeriksa bahwa:
- setidaknya satu loop closure terjadi
- galat pose akhir SLAM di bawah 0,3 m, sementara odometri saja melenceng lebih dari 1 m
- lebih dari 85 % sel terisi pada peta berada di dinding yang sebenarnya

Uji terpisah memeriksa kemampuan scan matcher memulihkan pose yang bergeser dan konvergensi optimizer pose graph.
