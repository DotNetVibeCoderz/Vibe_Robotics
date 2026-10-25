# TF2 — coordinate frames

`ROS2.Net.Tf2` tracks how coordinate frames (`map`, `odom`, `base_link`, `laser_link`, ...) relate over time, like `tf2` / `tf2_ros`. It speaks the standard `/tf` and `/tf_static` topics (`tf2_msgs/msg/TFMessage`), so it interoperates with RViz, `robot_state_publisher` and other ROS 2 nodes.

```bash
dotnet add package ROS2.Net.Tf2
```

## Math types

| Type | Purpose |
|---|---|
| `Vector3d` | 3D vector: `+ - *`, `Dot`, `Cross`, `Lerp`, `Normalized()` |
| `Quaterniond` | rotation: `*`, `Inverse()`, `Rotate(v)`, `Slerp`, `FromRpy`/`ToRpy`, `FromYaw`, `FromAxisAngle`/`ToAxisAngle` |
| `Transform3d` | rigid transform `p' = R·p + t`: `a * b` (b first), `Inverse()`, `TransformPoint`, `TransformVector`, `Interpolate`, `From2D(x, y, yaw)` |

All have `ToMsg()` / `FromMsg()` for the `geometry_msgs` types.

## The buffer

```csharp
var buffer = new TfBuffer();                     // keeps 10 s of history per frame
buffer.SetTransform("map", "odom", Transform3d.Identity, RosTime.Now(), isStatic: true);
buffer.SetTransform("odom", "base_link", Transform3d.From2D(1.0, 2.0, Math.PI / 2), RosTime.Now());

// Where is the laser, expressed in the map frame?
TransformStamped tf = buffer.LookupTransform("map", "laser_link", RosTime.Zero);
```

- `LookupTransform(target, source, time)` returns the transform that maps data in `source` into `target`. `RosTime.Zero` means "the latest time every link in the chain has data", which is what tf2 does.
- `LookupTransform(target, targetTime, source, sourceTime, fixedFrame)` is time travel. For example: "where was the robot one second ago, seen from where it is now", with `odom` as the fixed frame.
- `CanTransform(...)` and `TryLookupTransform(...)` check without throwing. `WaitForTransformAsync(..., timeout, ct)` waits until data arrives.
- `TransformPose`, `TransformPoint` and `TransformVector` convert stamped `geometry_msgs` data at the data's own stamp. Pass an optional timeout to wait.
- Failures throw the tf2 exception types: `LookupException` (unknown frame), `ConnectivityException` (separate trees), `ExtrapolationException` (time outside the cache) and `TransformTimeoutException`.

Lookups between two samples interpolate the translation linearly and the rotation with slerp. Static frames are valid at all times. Every frame has its own lock, so a listener inserting at high rate and control loops looking up transforms rarely contend.

### Introspection

| Method | Output |
|---|---|
| `AllFramesAsString()` | `Frame base_link exists with parent odom.` |
| `AllFramesAsYaml()` | the YAML `view_frames` / `tf2_monitor` print (parent, broadcaster, rate, buffer length) |
| `ToMermaid()` / `ToDot()` | the frame tree as a Mermaid flowchart or Graphviz DOT |
| `FrameNames`, `GetParent(frame)`, `FrameExists(frame)` | |

## Listening and broadcasting

```csharp
await using var context = await Ros2Context.CreateAsync();
var node = context.CreateNode("tf_demo");

var buffer = new TfBuffer();
using var listener = new TransformListener(buffer, node);       // /tf + /tf_static

var broadcaster = new TransformBroadcaster(node);
broadcaster.SendTransform("odom", "base_link", Transform3d.From2D(x, y, yaw), node.Clock.Now());

var statics = new StaticTransformBroadcaster(node);
statics.SendTransform("base_link", "laser_link", new Transform3d(new Vector3d(0.2, 0, 0.15), Quaterniond.Identity));
```

- `TransformListener` uses callback-less subscriptions that a dedicated background task drains. The buffer therefore stays current whether or not the node is spinning, and TF traffic never competes with the node's callbacks. `/tf` uses keep-last 100, reliable. `/tf_static` uses keep-last 100, transient local, so static frames published earlier are received.
- `StaticTransformBroadcaster` publishes on a latched topic (depth 1, transient local). Like tf2_ros, every call republishes the accumulated set of static frames, so late joiners receive all of them.

## Verification status

Unit and integration tests cover the following, over the in-process middleware:
- chained lookups through a common ancestor, sibling branches and latest-common-time
- interpolation, extrapolation errors, disconnected trees and time travel
- stamped-data helpers, introspection output and wait-for-transform
- a broadcaster → listener round trip

Interoperability with a real ROS 2 graph depends on the middleware you choose (see the RTPS and rosbridge documentation).
