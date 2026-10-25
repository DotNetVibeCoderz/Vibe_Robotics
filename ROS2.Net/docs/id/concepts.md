# Konsep inti

ROS2.Net mengikuti konsep `rclcpp` dengan API yang terasa alami di .NET (async/await,
`IAsyncEnumerable`, `ILogger`, `IDisposable`).

## Context dan node

`Ros2Context` memiliki middleware, daftar node, dan sinyal shutdown (`ShutdownToken`). Sebuah `Node`
memiliki entitas-entitasnya; turunkan kelas `Node` untuk membuat komponen yang dapat dipakai ulang.

```csharp
public sealed class Odometer : Node
{
    public Odometer(Ros2Context ctx, NodeOptions? o = null) : base(ctx, "odometer", o)
    {
        var rate = DeclareParameter("rate_hz", 20.0);
        var pub = CreatePublisher<Msgs.NavMsgs.Odometry>("odom", QosProfile.SensorData);
        CreateTimer(TimeSpan.FromSeconds(1 / rate), () => pub.Publish(new()));
    }
}
```

Penamaan mengikuti aturan ROS: relatif (`odom` → `/ns/odom`), absolut, privat
(`~/state` → `/ns/node/state`), dan dapat dipetakan ulang dengan `-r from:=to`.

## Executor dan callback group

Pesan diantrekan per entitas (dibatasi oleh depth QoS) lalu dijalankan oleh executor:

| Executor | Perilaku |
|---|---|
| `SingleThreadedExecutor` | Satu callback dalam satu waktu (bawaan `context.SpinAsync()`). |
| `MultiThreadedExecutor(n)` | `n` worker; grup `MutuallyExclusive` tidak pernah tumpang tindih, grup `Reentrant` berjalan paralel. |
| `RealtimeExecutor(busyWait)` | Thread khusus berprioritas tinggi, opsi busy-wait untuk latensi di bawah 1 ms. |

Callback boleh `async` — executor akan menunggunya selesai. Timer memakai satu penjadwal resolusi
tinggi bersama (satu thread untuk semua timer, akurasi ~0,1 ms) dan mengikuti `/clock` bila
`use_sim_time` bernilai true.

**Perbedaan dengan rclcpp:** respons service dan hasil action langsung menyelesaikan `Task` yang
dikembalikan, sehingga `await client.CallAsync(...)` bekerja tanpa harus melakukan spin pada node
pemanggil.

## Topic

```csharp
var sub = node.CreateSubscription<Imu>("imu", OnImu, QosProfile.SensorData);  // callback lewat executor
var stream = node.CreateSubscription<Imu>("imu");                              // tanpa callback:
await foreach (var imu in stream.ReadAllAsync(ct)) { ... }                    // tidak butuh executor
```

Opsi: `ContentFilter` (pesan dibuang sebelum masuk antrean), `IgnoreLocalPublications`, callback
event QoS. Data CDR mentah: `CreateSerializedPublisher/Subscription` (dipakai oleh bag dan bridge).

## QoS

Preset `QosProfile`: `Default`, `SensorData`, `ServicesDefault`, `Parameters`, `Latched` (transient
local, depth 1), `Rosout`, `SystemDefault`. Gaya fluent: `.KeepLast(5).AsBestEffort().WithDeadline(...)`.
Pencocokan mengikuti aturan DDS (`QosMatching.Check`): publisher best-effort tidak pernah melayani
subscription reliable, publisher volatile tidak pernah melayani subscription transient-local.
Event yang tersedia: deadline terlewat, pesan hilang, matched, QoS tidak kompatibel.

## Service dan action

```csharp
node.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("add", r => new() { Sum = r.A + r.B });
var client = node.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("add");
await client.WaitForServiceAsync(TimeSpan.FromSeconds(5));
var sum = (await client.CallAsync(new() { A = 1, B = 2 }, TimeSpan.FromSeconds(1))).Sum;
```

Action memakai service dan topic standar `_action/*` (kompatibel dengan `ros2 action`):
`CreateActionServer<TGoal,TResult,TFeedback>(nama, async (goal, ct) => hasil)` — lempar
`OperationCanceledException` setelah menanggapi permintaan pembatalan. Di sisi client:
`SendGoalAsync`, `GetResultAsync`, `CancelAsync`, `SendGoalAndWaitAsync`.

## Parameter

`DeclareParameter` dengan deskriptor (read-only, rentang nilai, tipe dinamis), nilai override dari
`-p`, `--params-file` (format YAML ROS termasuk wildcard `/**`) dan `NodeOptions.ParameterOverrides`;
callback validasi (`AddOnSetParametersCallback`), event `ParametersChanged`, topic
`/parameter_events`, serta enam service parameter sehingga `ros2 param` dapat dipakai.
`ParameterClient` mengelola parameter node lain dari jarak jauh.

## Lifecycle node

`LifecycleNode` mengimplementasikan Unconfigured → Inactive → Active → Finalized dengan override
`On…` atau properti handler, pemrosesan error, service `~/change_state` dan kawan-kawannya, serta
topic `~/transition_event`. `CreateLifecyclePublisher` hanya menerbitkan pesan saat node aktif.

## Komposisi dan intra-process

`ComponentContainer` memuat node bertanda `[RosComponent]` dari assembly (juga lewat service
`composition_interfaces`). Dengan `UseIntraProcessComms = true`, publisher mengirim ke subscription
dalam context yang sama secara by-reference, sementara middleware tetap melayani peer jarak jauh.

## Logging dan waktu

`node.Logger` adalah `ILogger` yang juga menerbitkan `rcl_interfaces/msg/Log` ke `/rosout`.
`RosTime`/`RosDuration` beresolusi nanodetik; tersedia `RosClock` (ROS/system/steady) dan `Rate`.

## Abstraksi middleware

`IRmwImplementation` adalah padanan bertipe dari API C `rmw`. Implementasikan `IRmwFactory` dan
daftarkan ke `RmwRegistry` untuk menambahkan transport baru.
