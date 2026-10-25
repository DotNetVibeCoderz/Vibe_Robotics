# Memulai

> ROS2.Net — dibuat oleh **Gravicode Studios** dipimpin oleh **Kang Fadhil**.

## Instalasi

```bash
dotnet new console -n RobotSaya && cd RobotSaya
dotnet add package ROS2.Net            # Core + pesan standar + middleware
```

Atau mulai dari template:

```bash
dotnet new install ROS2.Net.Templates
dotnet new ros2net-node -n RobotSaya
```

Kebutuhan: .NET 10 SDK. **Tidak perlu memasang ROS 2** — ROS2.Net adalah implementasi client
library ROS 2 murni .NET, lengkap dengan middleware-nya sendiri:

| Middleware | Identifier | Dipakai untuk |
|---|---|---|
| In-process | `inprocess` (bawaan) | Sistem terkomposisi, simulator, pengujian. Tanpa salinan data (zero copy). |
| RTPS (DDS terkelola) | `rtps` | Berkomunikasi dengan node ROS 2 di jaringan (protokol kabel Fast DDS / Cyclone DDS). |
| rosbridge | `rosbridge` | Menjangkau sistem ROS 2 lewat `rosbridge_server` (WebSocket), misalnya dari browser atau melewati NAT. |

Pilih dengan variabel lingkungan `ROS2NET_RMW=rtps` atau lewat kode (`options.UseRtps()`,
`options.UseRosbridge("ws://robot:9090")`).

## Halo, ROS 2

```csharp
using ROS2.Net;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await Ros2Context.CreateAsync(new() { Arguments = args, HandleShutdownSignals = true });

var node = context.CreateNode("talker");
var pub = node.CreatePublisher<StdMsgs.String>("chatter");
node.CreateTimer(TimeSpan.FromSeconds(0.5), () => pub.Publish(new() { Data = "Hello World" }));

node.CreateSubscription<StdMsgs.String>("chatter", m => Console.WriteLine($"Saya dengar: {m.Data}"));

await context.SpinAsync();   // menjalankan callback sampai Ctrl+C
```

`StdMsgs.String` adalah tipe hasil generate untuk `std_msgs/msg/String`. Tipe pesan berada di
namespace `ROS2.Net.Msgs.<NamaPaketPascalCase>`; alias seperti
`using StdMsgs = ROS2.Net.Msgs.StdMsgs;` menghindari bentrok dengan `System.String`/`System.Int32`.

## Argumen baris perintah

Setiap aplikasi menerima argumen ROS standar:

```bash
dotnet run -- --ros-args -r __ns:=/robot1 -r chatter:=news -p max_speed:=1.5 --params-file params.yaml --log-level debug
```

## Contoh

| Contoh | Menunjukkan |
|---|---|
| `samples/ROS2.Net.Samples.TalkerListener` | Publisher, subscription, timer, `/rosout` |
| `samples/ROS2.Net.Samples.ServiceClient` | Server `AddTwoInts` dan client async |
| `samples/ROS2.Net.Samples.FibonacciAction` | Action server/client, feedback, pembatalan |
| `samples/ROS2.Net.Samples.LifecycleNode` | Mesin status node terkelola, lifecycle publisher |
| `samples/ROS2.Net.Samples.Parameters` | Deskriptor, rentang nilai, validasi, client parameter jarak jauh, dump YAML |
| `samples/ROS2.Net.Samples.Composition` | Component container, `[RosComponent]` |

Selanjutnya: [Konsep inti](concepts.md) · [Interface & pembuatan kode](interfaces.md).
