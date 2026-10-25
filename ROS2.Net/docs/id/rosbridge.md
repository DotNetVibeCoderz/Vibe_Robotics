# Middleware rosbridge

`ROS2.Net.Rmw.Rosbridge` menghubungkan ROS2.Net ke sistem ROS 2 yang sudah ada melalui
[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite). Middleware ini memakai protokol
JSON rosbridge v2 lewat WebSocket, jadi di sisi .NET tidak perlu stack DDS maupun multicast. Inilah cara
termudah untuk menjangkau robot dari laptop, server, container, atau aplikasi Blazor.

```bash
# Di robot
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

```csharp
using ROS2.Net;
using ROS2.Net.Rmw.Rosbridge;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var ctx = await Ros2Context.CreateAsync(new Ros2ContextOptions().UseRosbridge("ws://robot.local:9090"));
var node = ctx.CreateNode("dashboard");
var sub = node.CreateSubscription<StdMsgs.String>("/chatter");
await foreach (var msg in sub.ReadAllAsync())
{
    Console.WriteLine(msg.Data);
}
```

Cara lain: set `ROS2NET_RMW=rosbridge` lalu isi setting `url`
(`options.RmwSettings["url"] = "ws://…"`). Cukup mereferensikan paketnya, middleware akan mendaftarkan
dirinya sendiri.

## Fitur yang didukung

| Fitur | Operasi rosbridge |
|---|---|
| Publisher (typed dan serialized) | `advertise`, `publish`, `unadvertise` (`latch` = transient local, `queue_size` = depth QoS) |
| Subscription (typed dan serialized) | `subscribe`, `unsubscribe` (`queue_length` = depth QoS, opsional `throttle_rate`, `fragment_size`) |
| Klien service | `call_service`, dikorelasikan lewat `id` per panggilan |
| Server service | `advertise_service`, `call_service` yang masuk, `service_response` |
| Action | implementasi action dari Core, lewat service dan topic tersembunyi `<action>/_action/*` |
| Pesan besar | operasi `fragment` yang masuk disatukan kembali |
| Graph | diambil berkala dari `/rosapi/nodes`, `/rosapi/node_details`, `/rosapi/topics` dan `/rosapi/services`, ditambah endpoint milik context sendiri |
| Ketahanan | koneksi otomatis tersambung ulang, lalu semua advertisement, subscription, dan service dibuat ulang |

Pesan dikirim sebagai JSON hasil `WriteJson`/`ReadJson` yang di-generate: nama field snake_case,
`uint8[]` sebagai base64, dan NaN/Infinity sebagai string. Endpoint serialized (perekaman bag, relay)
mengonversi JSON ⇄ CDR melalui factory tipe pesannya, jadi tipenya harus dikenal (hasil generate atau
sudah terdaftar).

## Pengaturan

| Kunci | Default | Arti |
|---|---|---|
| `url` | `ws://localhost:9090` | URL WebSocket rosbridge |
| `connect_timeout_ms` | 5000 | koneksi pertama harus berhasil dalam waktu ini; kalau tidak, `CreateAsync` melempar exception |
| `reconnect_ms` | 1000 | jeda antar percobaan sambung ulang |
| `graph_poll_ms` | 2000 | interval polling rosapi (0 = mati) |
| `throttle_rate` | 0 | jeda minimum (ms) antar pesan per subscription |
| `fragment_size` | 0 | minta rosbridge memecah pesan yang lebih besar dari nilai ini |

`UseRosbridge(url, o => …)` mengatur nilai yang sama lewat `RosbridgeOptions` yang bertipe.

## Perbedaan dengan middleware DDS

- **Satu koneksi per context.** Setiap topic di-advertise dan di-subscribe sekali, lalu disebarkan secara lokal.
- **Tidak ada zero copy intra-process.** rosbridge mengirim balik publikasi klien ke klien itu sendiri,
  jadi `SupportsIgnoreLocalPublications` bernilai false dan `UseIntraProcessComms` diabaikan.
- **QoS hanya mendekati.** Hanya depth dan latching yang diteruskan ke rosbridge. Deadline, lifespan, dan
  liveliness tidak ditegakkan di sisi server, dan tidak ada event QoS.
- **Ketersediaan service diambil dari rosapi.** Tanpa rosapi, `IsServiceReady` selalu bernilai true dan
  panggilan tetap dicoba. Panggilan yang gagal (`result: false`) dicatat di log, dan pemanggil akan timeout.
- **Satu server per service per klien.** rosbridge hanya menerima satu penyedia per WebSocket.

## Status verifikasi

Middleware ini sudah diuji terhadap server rosbridge tiruan yang berjalan in-process: pub/sub, payload
biner, satu siklus panggilan service, graph, action Fibonacci, dan sambung ulang. Middleware ini **belum
diuji terhadap `rosbridge_server` sungguhan**. Khususnya, apakah rosbridge menerima tipe wire action
seperti `example_interfaces/action/Fibonacci_SendGoal` untuk `advertise_service` bergantung pada versi
rosbridge. Versi terbaru juga punya operasi action khusus, yang tidak dipakai di sini.
