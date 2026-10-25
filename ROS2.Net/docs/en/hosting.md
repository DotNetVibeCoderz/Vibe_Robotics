# Generic Host and dependency injection

`ROS2.Net.Hosting` runs ROS 2 nodes inside a .NET Generic Host (`Host.CreateApplicationBuilder`,
ASP.NET Core, Blazor Server or a Worker Service), with the usual configuration, logging and DI.

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
    // Ros2Context, NodeOptions and any registered service are injected.
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

## What happens

- `AddRos2` registers a singleton `Ros2Context`, created from `Ros2HostOptions` with the host's
  `ILoggerFactory`, so node logs go to the host's console/Serilog/OpenTelemetry pipeline. It also
  registers a hosted service.
- On **start**, the hosted service creates every node registered with `AddRos2Node`. It then spins the
  context on the chosen executor (`SingleThreaded`, `MultiThreaded` or `Realtime`), which also picks up
  nodes created later.
- On **stop**, spinning is cancelled and the context shuts down. The context and its middleware are
  disposed with the service provider.
- If code calls `context.Shutdown()`, the application stops as well (`StopApplicationOnShutdown`,
  on by default).

## Options

| Option | Meaning |
|---|---|
| `DomainId` | DDS domain; null uses `ROS_DOMAIN_ID` |
| `Rmw` | middleware identifier: `inprocess`, `rtps` or `rosbridge`. The middleware's assembly must be referenced (it registers on load) |
| `RmwFactory` | a factory instance; takes precedence over `Rmw` |
| `RmwSettings` | middleware settings (e.g. `url` for rosbridge) |
| `UseIntraProcessComms` | zero-copy delivery between nodes of the context |
| `Arguments` | `--ros-args` content: remaps, parameters, log levels |
| `LocalhostOnly`, `Enclave` | discovery range and SROS2 enclave |
| `Executor`, `ThreadCount` | how callbacks are run |
| `ConfigureContext` | final code hook on the `Ros2ContextOptions` (e.g. `o => o.UseRosbridge(url)`) |

`AddRos2Node<T>(sp => new NodeOptions()…)` passes `NodeOptions` only to constructors that declare a
`NodeOptions` parameter. Registered nodes are also resolvable as `T`, so controllers and Blazor
components can inject them.
