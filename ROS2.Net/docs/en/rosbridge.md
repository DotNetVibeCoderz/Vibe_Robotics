# rosbridge middleware

`ROS2.Net.Rmw.Rosbridge` connects ROS2.Net to an existing ROS 2 system through
[rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite). It speaks the rosbridge v2
JSON protocol over a WebSocket, so no DDS stack or multicast is needed on the .NET side. That makes it the
easiest way to reach a robot from a laptop, a server, a container or a Blazor app.

```bash
# On the robot
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

Alternatively, set `ROS2NET_RMW=rosbridge` and the `url` setting
(`options.RmwSettings["url"] = "ws://…"`). Referencing the package is enough for it to register itself.

## What is supported

| Feature | rosbridge ops used |
|---|---|
| Publishers (typed and serialized) | `advertise`, `publish`, `unadvertise` (`latch` = transient local, `queue_size` = QoS depth) |
| Subscriptions (typed and serialized) | `subscribe`, `unsubscribe` (`queue_length` = QoS depth, optional `throttle_rate`, `fragment_size`) |
| Service clients | `call_service` with per-call `id` correlation |
| Service servers | `advertise_service`, incoming `call_service`, `service_response` |
| Actions | Core's action implementation over the hidden `<action>/_action/*` services and topics |
| Large messages | incoming `fragment` ops are reassembled |
| Graph | polled from `/rosapi/nodes`, `/rosapi/node_details`, `/rosapi/topics` and `/rosapi/services`, plus this context's own endpoints |
| Robustness | automatic reconnect, then every advertisement, subscription and service is re-created |

Messages travel as JSON produced by the generated `WriteJson`/`ReadJson`: snake_case field names,
`uint8[]` as base64, and NaN/Infinity as strings. Serialized endpoints (bag recording, relays) convert
JSON ⇄ CDR through the type's factory, so the type must be known (generated or registered).

## Settings

| Key | Default | Meaning |
|---|---|---|
| `url` | `ws://localhost:9090` | rosbridge WebSocket URL |
| `connect_timeout_ms` | 5000 | the first connection must succeed within this time, otherwise `CreateAsync` throws |
| `reconnect_ms` | 1000 | delay between reconnection attempts |
| `graph_poll_ms` | 2000 | rosapi polling interval (0 disables polling) |
| `throttle_rate` | 0 | minimum ms between messages per subscription |
| `fragment_size` | 0 | ask rosbridge to fragment messages larger than this |

`UseRosbridge(url, o => …)` sets the same values with a typed `RosbridgeOptions`.

## Differences from DDS middlewares

- **One link per context.** Each topic is advertised and subscribed once and fanned out locally.
- **No intra-process zero copy.** rosbridge echoes a client's own publications back to it, so
  `SupportsIgnoreLocalPublications` is false and `UseIntraProcessComms` is ignored.
- **QoS is approximate.** Only depth and latching map onto rosbridge; deadline, lifespan and liveliness
  are not enforced on the server side, and no QoS events are raised.
- **Service availability comes from rosapi.** Without rosapi, `IsServiceReady` reports true and calls
  are attempted anyway. A failed call (`result: false`) is logged and the caller times out.
- **One server per service per client.** rosbridge accepts a single provider per WebSocket.

## Verification status

The middleware is tested against an in-process fake rosbridge server: pub/sub, binary payloads, a
service round trip, the graph, a Fibonacci action and reconnection. It has **not yet been tested against
a real `rosbridge_server`**. In particular, whether rosbridge accepts action wire types such as
`example_interfaces/action/Fibonacci_SendGoal` for `advertise_service` depends on the rosbridge version.
Recent versions also provide dedicated action ops, which are not used here.
