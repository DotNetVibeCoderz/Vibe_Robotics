# ROS2.Net Gallery (desktop)

> Made by Gravicode Studios, led by Kang Fadhil.

The gallery is a showcase app in which every page runs a real ROS 2 graph inside the window, on the in-process middleware. Each page has a live demo, the code that drives it, and the ROS interfaces it uses, shown as chips in the header. No ROS installation is needed.

```bash
dotnet run --project apps/ROS2.Net.Gallery.Desktop
```

![Gallery home](../images/desktop/gallery-home.png)

The sidebar toggles between English and Bahasa Indonesia, and between dark and light themes. **Hide code** collapses the code panel.

| | |
|---|---|
| ![Indonesian](../images/desktop/gallery-home-id.png) | ![Light](../images/desktop/gallery-home-light.png) |

## Pages

Pages are grouped by layer of the ROS stack.

### Communication

| Page | Demonstrates |
|---|---|
| Talker & listener | Publisher, subscription, timer, async streams (`ReadAllAsync`), live `PlotView` |
| Services | `AddTwoInts` server, awaited client, latency per call |
| Actions | Fibonacci goal, feedback stream, cancel, result status |
| Parameters | Descriptors and ranges, validation callback, read-only parameters, `ParameterEditor` over the parameter services, `/parameter_events` |
| Lifecycle nodes | The state machine, driven by buttons. The lifecycle publisher only delivers while Active. |

![Talker](../images/desktop/gallery-talker.png)
![Actions](../images/desktop/gallery-actions.png)
![Parameters](../images/desktop/gallery-parameters.png)
![Lifecycle](../images/desktop/gallery-lifecycle.png)
![Services](../images/desktop/gallery-services.png)

### Robotics

| Page | Demonstrates |
|---|---|
| Teleop a TurtleBot | Simulator + `TeleopJoystick` on `/robot1/cmd_vel`, odometry plot, battery |
| LiDAR & maps | `LaserScanView` and latched `/map` with the ground-truth pose and the projected scan |
| Path planning | Costmap with inflation; A*, Theta*, Dijkstra and Hybrid-A* (`ROS2.Net.Navigation`). Click to set the goal. |
| TF2 frames | Two robots on `/tf` and `/tf_static`, `TfTreeView`, live `lookupTransform` |
| ros2_control | Controller manager, diff-drive controller, joint state broadcaster, odometry track |
| Arm kinematics | A UR5 from URDF, Levenberg–Marquardt IK under joint limits, orbit view |

![Teleop](../images/desktop/gallery-teleop.png)
![LiDAR](../images/desktop/gallery-lidar.png)
![Navigation](../images/desktop/gallery-navigation.png)
![TF](../images/desktop/gallery-tf.png)
![Control](../images/desktop/gallery-control.png)
![Arm](../images/desktop/gallery-arm.png)

### Intelligence

| Page | Demonstrates |
|---|---|
| Point clouds | Voxel grid, SOR, RANSAC floor removal and Euclidean clusters. Uses the Rust kernels when present. |
| Reinforcement learning | Trains a Q-table agent on `RobotGymEnv` (RLNet), shows the return curve, then the greedy policy drives |
| Language commander | `NaturalLanguageCommanderNode` with the offline rule-based model, in English and Indonesian. The robot moves under a safety limiter. |

![Perception](../images/desktop/gallery-perception.png)
![RL](../images/desktop/gallery-rl.png)
![Commander](../images/desktop/gallery-commander.png)

### Tools

| Page | Demonstrates |
|---|---|
| rosbag2 recording | Record raw CDR to MCAP plus `metadata.yaml`, `BagInfo`, loop playback onto `/replay/*` |
| Connectivity & graph | Switch between the in-process, RTPS and rosbridge middlewares; `NodeGraphView` and `TopicMonitor` |

![Bag](../images/desktop/gallery-bag.png)
![Connectivity](../images/desktop/gallery-connectivity.png)

## How it is built

- `GalleryRuntime` owns one `Ros2Context`: the in-process middleware on a private domain, spun by a `MultiThreadedExecutor`. Pages create their nodes on it and dispose them when the user leaves.
- Each page derives from `GalleryPage`. It supplies `Id`, `Group`, `Title` and `Summary` in both languages, `Interfaces` (the chips), `Code`, and `Build(runtime)`.
- The UI is built in C# with no XAML, using `Ui` helpers that draw every colour from `RosTheme`.

## Screenshots

The images in this document are rendered headlessly from the real windows with Avalonia.Headless and Skia:

```bash
dotnet test tests/ROS2.Net.Avalonia.Tests --filter Category=Screenshots
```

This writes `docs/images/desktop/*.png`.
