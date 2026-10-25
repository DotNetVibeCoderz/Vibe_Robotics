# Core concepts

ROS2.Net mirrors `rclcpp` concepts with idiomatic .NET APIs (async/await, `IAsyncEnumerable`,
`ILogger`, `IDisposable`).

## Context and nodes

`Ros2Context` owns the middleware, the nodes and the shutdown signal (`ShutdownToken`). A `Node`
owns entities; derive from it to build reusable components.

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

Names follow ROS rules: relative (`odom` → `/ns/odom`), absolute, private (`~/state` → `/ns/node/state`),
remapped by `-r from:=to`.

## Executors and callback groups

Messages are queued per entity (bounded by QoS depth) and run by an executor:

| Executor | Behavior |
|---|---|
| `SingleThreadedExecutor` | One callback at a time (default for `context.SpinAsync()`). |
| `MultiThreadedExecutor(n)` | `n` workers; `MutuallyExclusive` groups never overlap, `Reentrant` groups run in parallel. |
| `RealtimeExecutor(busyWait)` | Dedicated high-priority thread, optional busy-wait for sub-ms latency. |

Callbacks may be `async` — the executor awaits them. Timers use a shared high-resolution scheduler
(1 thread for all timers, ~0.1 ms accuracy) and follow `/clock` when `use_sim_time` is true.

**Difference from rclcpp:** service responses and action results complete the returned `Task`
directly, so `await client.CallAsync(...)` works without spinning the calling node.

## Topics

```csharp
var sub = node.CreateSubscription<Imu>("imu", OnImu, QosProfile.SensorData);  // executor callback
var stream = node.CreateSubscription<Imu>("imu");                              // no callback:
await foreach (var imu in stream.ReadAllAsync(ct)) { ... }                    // no executor needed
```

Options: `ContentFilter` (drop before queueing), `IgnoreLocalPublications`, QoS event callback.
Raw CDR: `CreateSerializedPublisher/Subscription` (used by bags and bridges).

## QoS

`QosProfile` presets: `Default`, `SensorData`, `ServicesDefault`, `Parameters`, `Latched` (transient
local, depth 1), `Rosout`, `SystemDefault`. Fluent: `.KeepLast(5).AsBestEffort().WithDeadline(...)`.
Matching follows DDS rules (`QosMatching.Check`): best-effort publishers never serve reliable
subscriptions, volatile publishers never serve transient-local subscriptions. Events: deadline
missed, message lost, matched, incompatible QoS.

## Services and actions

```csharp
node.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("add", r => new() { Sum = r.A + r.B });
var client = node.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("add");
await client.WaitForServiceAsync(TimeSpan.FromSeconds(5));
var sum = (await client.CallAsync(new() { A = 1, B = 2 }, TimeSpan.FromSeconds(1))).Sum;
```

Actions use the standard `_action/*` services and topics (interoperable with `ros2 action`):
`CreateActionServer<TGoal,TResult,TFeedback>(name, async (goal, ct) => result)` — throw
`OperationCanceledException` after honoring a cancel. Clients: `SendGoalAsync`, `GetResultAsync`,
`CancelAsync`, `SendGoalAndWaitAsync`.

## Parameters

`DeclareParameter` with descriptors (read-only, ranges, dynamic typing), overrides from
`-p`, `--params-file` (ROS YAML layout incl. `/**` wildcards) and `NodeOptions.ParameterOverrides`;
validation callbacks (`AddOnSetParametersCallback`), `ParametersChanged` event, `/parameter_events`,
and the six parameter services so `ros2 param` works. `ParameterClient` manages remote nodes.

## Lifecycle nodes

`LifecycleNode` implements Unconfigured → Inactive → Active → Finalized with `On…` overrides or
handler properties, error processing, `~/change_state` etc. services and `~/transition_event`.
`CreateLifecyclePublisher` only publishes while active.

## Composition and intra-process

`ComponentContainer` loads `[RosComponent]` nodes from assemblies (also via
`composition_interfaces` services). With `UseIntraProcessComms = true` publishers deliver to
subscriptions of the same context by reference while the middleware still serves remote peers.

## Logging and time

`node.Logger` is an `ILogger` that also publishes `rcl_interfaces/msg/Log` on `/rosout`.
`RosTime`/`RosDuration` have nanosecond resolution; `RosClock` (ROS/system/steady) and `Rate`.

## Middleware abstraction

`IRmwImplementation` is a typed equivalent of the C `rmw` API. Implement `IRmwFactory` and
register it with `RmwRegistry` to add a transport.
