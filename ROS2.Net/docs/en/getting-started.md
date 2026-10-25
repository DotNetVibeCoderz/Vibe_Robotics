# Getting started

> ROS2.Net — made by **Gravicode Studios**, led by **Kang Fadhil**.

## Install

```bash
dotnet new console -n MyRobot && cd MyRobot
dotnet add package ROS2.Net            # Core + standard messages + middlewares
```

Or start from a template:

```bash
dotnet new install ROS2.Net.Templates
dotnet new ros2net-node -n MyRobot
```

Requirements: .NET 10 SDK. **No ROS 2 installation is required** — ROS2.Net is a pure .NET
implementation of the ROS 2 client library, with its own middlewares:

| Middleware | Identifier | Use it for |
|---|---|---|
| In-process | `inprocess` (default) | Composed systems, simulators, tests. Zero copy. |
| RTPS (managed DDS) | `rtps` | Talking to ROS 2 nodes on the network (Fast DDS / Cyclone DDS wire protocol). |
| rosbridge | `rosbridge` | Reaching a ROS 2 system through `rosbridge_server` (WebSocket), e.g. from a browser or across NAT. |

Pick one with `ROS2NET_RMW=rtps` or in code (`options.UseRtps()`, `options.UseRosbridge("ws://robot:9090")`).

## Hello, ROS 2

```csharp
using ROS2.Net;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await Ros2Context.CreateAsync(new() { Arguments = args, HandleShutdownSignals = true });

var node = context.CreateNode("talker");
var pub = node.CreatePublisher<StdMsgs.String>("chatter");
node.CreateTimer(TimeSpan.FromSeconds(0.5), () => pub.Publish(new() { Data = "Hello World" }));

node.CreateSubscription<StdMsgs.String>("chatter", m => Console.WriteLine($"I heard: {m.Data}"));

await context.SpinAsync();   // runs callbacks until Ctrl+C
```

`StdMsgs.String` is the generated `std_msgs/msg/String`. Message types live in
`ROS2.Net.Msgs.<PascalCasePackage>`; an alias such as `using StdMsgs = ROS2.Net.Msgs.StdMsgs;`
avoids clashes with `System.String`/`System.Int32`.

## Command-line arguments

Every application accepts the standard ROS arguments:

```bash
dotnet run -- --ros-args -r __ns:=/robot1 -r chatter:=news -p max_speed:=1.5 --params-file params.yaml --log-level debug
```

## Samples

| Sample | Shows |
|---|---|
| `samples/ROS2.Net.Samples.TalkerListener` | Publisher, subscription, timer, `/rosout` |
| `samples/ROS2.Net.Samples.ServiceClient` | `AddTwoInts` server and async client |
| `samples/ROS2.Net.Samples.FibonacciAction` | Action server/client, feedback, cancellation |
| `samples/ROS2.Net.Samples.LifecycleNode` | Managed node state machine, lifecycle publisher |
| `samples/ROS2.Net.Samples.Parameters` | Descriptors, ranges, validation, remote parameter client, YAML dump |
| `samples/ROS2.Net.Samples.Composition` | Component container, `[RosComponent]` |

Next: [Core concepts](concepts.md) · [Interfaces & code generation](interfaces.md).
