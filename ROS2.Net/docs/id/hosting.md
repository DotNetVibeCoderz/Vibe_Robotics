# Generic Host dan dependency injection

`ROS2.Net.Hosting` menjalankan node ROS 2 di dalam .NET Generic Host (`Host.CreateApplicationBuilder`,
ASP.NET Core, Blazor Server, atau Worker Service), lengkap dengan konfigurasi, logging, dan DI seperti
biasa.

```csharp
using ROS2.Net;
using ROS2.Net.Hosting;

var builder = Host.CreateApplicationBuilder(args);
builder.Services.AddSingleton<BatteryMonitor>();
builder.Services.AddRos2(builder.Configuration.GetSection("Ros2"), o =>
{
    o.Executor = Ros2ExecutorKind.MultiThreaded;
});
builder.Services.AddRos2Node<PatrolNode>();
builder.Services.AddRos2Node("heartbeat", (sp, node) =>
{
    var pub = node.CreatePublisher<ROS2.Net.Msgs.StdMsgs.String>("/heartbeat");
    node.CreateTimer(TimeSpan.FromSeconds(1), () => pub.Publish(new() { Data = "alive" }));
});
await builder.Build().RunAsync();

public sealed class PatrolNode : Node
{
    // Ros2Context, NodeOptions, dan service apa pun yang terdaftar akan di-inject.
    public PatrolNode(Ros2Context context, NodeOptions options, BatteryMonitor battery)
        : base(context, "patrol", options) { /* ... */ }
}
```

```json
{
  "Ros2": {
    "DomainId": 7,
    "Rmw": "rtps",
    "RmwSettings": { },
    "UseIntraProcessComms": true,
    "Arguments": [ "-p", "use_sim_time:=true", "-r", "cmd_vel:=/robot1/cmd_vel" ],
    "Executor": "MultiThreaded",
    "ThreadCount": 4
  }
}
```

## Cara kerjanya

- `AddRos2` mendaftarkan `Ros2Context` sebagai singleton, yang dibuat dari `Ros2HostOptions` dengan
  `ILoggerFactory` milik host. Dengan begitu, log node masuk ke pipeline host (console, Serilog, atau
  OpenTelemetry). `AddRos2` juga mendaftarkan sebuah hosted service.
- Saat host **mulai**, hosted service membuat semua node yang didaftarkan lewat `AddRos2Node`. Setelah
  itu ia menjalankan spin context pada executor yang dipilih (`SingleThreaded`, `MultiThreaded`, atau
  `Realtime`). Node yang dibuat belakangan juga ikut di-spin.
- Saat host **berhenti**, spin dibatalkan dan context di-shutdown. Context beserta middleware-nya
  di-dispose bersama service provider.
- Jika kode memanggil `context.Shutdown()`, aplikasi juga ikut berhenti (`StopApplicationOnShutdown`,
  aktif secara default).

## Opsi

| Opsi | Arti |
|---|---|
| `DomainId` | domain DDS; null berarti memakai `ROS_DOMAIN_ID` |
| `Rmw` | identifier middleware: `inprocess`, `rtps`, atau `rosbridge`. Assembly middleware-nya harus direferensikan (ia mendaftar saat dimuat) |
| `RmwFactory` | instance factory; diutamakan di atas `Rmw` |
| `RmwSettings` | pengaturan middleware (misalnya `url` untuk rosbridge) |
| `UseIntraProcessComms` | pengiriman zero-copy antar node dalam satu context |
| `Arguments` | isi `--ros-args`: remap, parameter, level log |
| `LocalhostOnly`, `Enclave` | jangkauan discovery dan enclave SROS2 |
| `Executor`, `ThreadCount` | cara callback dijalankan |
| `ConfigureContext` | kait kode terakhir untuk `Ros2ContextOptions` (misalnya `o => o.UseRosbridge(url)`) |

`AddRos2Node<T>(sp => new NodeOptions()…)` hanya meneruskan `NodeOptions` ke constructor yang memang
punya parameter `NodeOptions`. Node yang terdaftar juga bisa di-resolve sebagai `T`, jadi controller
dan komponen Blazor bisa meng-inject-nya.
