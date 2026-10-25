# Blazor components (`ROS2.Net.Blazor`)

Live ROS 2 widgets for Blazor Server and Blazor Web Apps (interactive server render mode). Each
component either follows a topic itself (`Topic="/scan"`) or renders data you pass in
(`Scan="@scan"`). It draws with SVG and PNG data URLs, so it needs no external chart or canvas library.

> Made by Gravicode Studios, led by Kang Fadhil.

![Gallery LiDAR page](../images/web/gallery-lidar.png)

## Setup

```bash
dotnet add package ROS2.Net.Blazor
```

```csharp
// Program.cs
builder.Services.AddRazorComponents().AddInteractiveServerComponents();
builder.Services.AddRos2Blazor(o =>
{
    o.Rmw = "rtps";        // inprocess (default) | rtps | rosbridge
    o.DomainId = 0;
});
```

```razor
@* App.razor <head> *@
<link rel="stylesheet" href="_content/ROS2.Net.Blazor/ros2net.css" />
<link rel="stylesheet" href="YourApp.styles.css" />

@* _Imports.razor *@
@using ROS2.Net.Blazor
@using ROS2.Net.Blazor.Components
```

`AddRos2Blazor` builds on ROS2.Net.Hosting. It registers a singleton `Ros2Context`, spun by a hosted
service on a multi-threaded executor. Components inject this context.

Use an interactive render mode **without prerendering**, for example
`new InteractiveServerRenderMode(prerender: false)`. Otherwise each component subscribes twice: once
during prerender and once when the circuit starts.

## Components

| Component | Shows | Key parameters |
|---|---|---|
| `TeleopJoystick` | drag pad plus W A S D / arrow keys → `geometry_msgs/Twist` | `Topic` (`/cmd_vel`), `MaxLinear`, `MaxAngular`, `RateHz`, `OnCommand` |
| `LaserScanView` | `sensor_msgs/LaserScan` as a polar plot with range rings | `Topic` or `Scan`, `MaxRange` |
| `OccupancyGridView` | `nav_msgs/OccupancyGrid`, pan and zoom; robot, path and goal overlays; click to pick a point | `Topic` (`/map`) or `Map`, `PathTopic`/`Path`, `RobotPose`, `Goal`, `OnMapClick`, `Overlay` |
| `PointCloudView` | `sensor_msgs/PointCloud2` or `PointCloud`, colored by height or by layer | `Topic`, `Cloud`, `Layers`, `Projection` (Top/Front/Side), `MaxPoints` |
| `TopicMonitor` | every topic with type, publishers and subscribers, rate (Hz) and bandwidth | `IncludeHidden`, `Exclude`, `Measure` |
| `NodeGraph` | node ↔ topic graph (publishes / subscribes) | `IncludeHidden`, `Exclude`, `HideUiNodes` |
| `TfTree` | TF frame tree with each frame's offset from its parent | `Buffer` (optional shared `TfBuffer`) |
| `ParameterEditor` | a remote node's parameters, with typed inputs, ranges and read-only state | `TargetNode` |
| `ImageView` | `sensor_msgs/Image` (any encoding ROS2.Net.Perception can convert) | `Topic` or `Image`, `MaxFps` |
| `LogConsole` | `/rosout`, filterable by level, node or text | `Topic`, `MaxLines` |
| `LivePlot` | real-time line chart of numeric fields of **any** registered message type | `Topic`, `Fields` (`twist.twist.linear.x`, `ranges[0]`), `WindowSeconds`, or call `Push(series, value)` |
| `BatteryGauge` | `sensor_msgs/BatteryState` charge ring | `Topic` or `Percentage` |
| `JointStatePanel` | `sensor_msgs/JointState` positions (in degrees) and velocities | `Topic` or `State` |
| `LifecycleStateView` | a lifecycle node's state machine, with transition buttons | `TargetNode` |

```razor
<OccupancyGridView Topic="/map" PathTopic="/plan" RobotPose="@pose" OnMapClick="SendGoal" />
<LaserScanView Topic="/scan" MaxRange="6" />
<TeleopJoystick Topic="/robot1/cmd_vel" MaxLinear="0.5" />
<LivePlot Topic="/odom" Fields="@(["twist.twist.linear.x", "twist.twist.angular.z"])" />
<ParameterEditor TargetNode="/controller" />
```

## Writing your own: `RosComponentBase`

```razor
@inherits RosComponentBase
<p>@_battery</p>
@code {
    private string _battery = "—";
    protected override void OnInitialized() =>
        Subscribe<BatteryState>("/battery_state", b => _battery = $"{b.Percentage:P0}");
}
```

`RosComponentBase` handles the plumbing:

- **Node:** each component gets its own private node (`Node`), created lazily and disposed with the component.
- **Subscriptions:** `Subscribe<T>` reads the topic on a background task, so no executor is involved.
- **Rendering:** it then asks for a re-render, coalesced to `RefreshHz` (30 per second by default), so a 100 Hz topic never floods the SignalR circuit.
- **Helpers:**
  - `SubscribeSerialized` gives raw CDR for types known only at runtime.
  - `Every(period, work)` runs periodic jobs.
  - `Disposing` is a cancellation token.

## Theming

Styles are scoped per component. Colors and fonts come from CSS variables in `ros2net.css`, such as
`--rn-bg`, `--rn-surface`, `--rn-ink`, `--rn-violet`, `--rn-cyan`, `--rn-font` and `--rn-mono`.
Dark is the default. Set `data-theme="light"` on `<html>`, or on any container, for the light palette,
or override the variables yourself. Map raster colors are component parameters (`FreeColor`,
`OccupiedColor`, `UnknownColor`), because PNG pixels cannot read CSS.

## Notes

- The components are rendered on the server. Under Blazor WebAssembly, only the rosbridge middleware
  can work in the browser sandbox; UDP is not available there.
- `OccupancyGridView` and `TeleopJoystick` use a tiny JS module (`_content/ROS2.Net.Blazor/ros2net.js`)
  for element size and wheel capture. All other components are JS-free.
- Tests: `tests/ROS2.Net.Blazor.Tests` (bUnit).
