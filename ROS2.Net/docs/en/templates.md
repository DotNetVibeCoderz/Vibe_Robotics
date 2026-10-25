# Project templates

> ROS2.Net — made by **Gravicode Studios**, led by **Kang Fadhil**.

```bash
dotnet new install ROS2.Net.Templates
dotnet new list ros2net
```

| Template | Short name | What you get |
|---|---|---|
| ROS2.Net Node | `ros2net-node` | Generic Host app with `TalkerNode` + `ListenerNode` (`AddRos2`, `AddRos2Node<T>`), a live-tunable `rate_hz` parameter, middleware chosen in `appsettings.json`. Option `--rmw rtps\|inprocess\|rosbridge`. |
| ROS2.Net Lifecycle Node | `ros2net-lifecycle` | A managed `sensor_driver` node (configure/activate/deactivate/cleanup/shutdown) with a lifecycle publisher; `--autostart` to activate on launch. |
| ROS2.Net Interfaces Library | `ros2net-interfaces` | Class library with sample `.msg/.srv/.action` files compiled to C# by `ROS2.Net.Rosidl.Generator`. Option `--rosPackage my_robot_msgs` sets the ROS package (folder and namespace). |
| ROS2.Net Blazor App | `ros2net-blazor` | Blazor Server dashboard: teleop pad (`/cmd_vel`), pose (`/odom`), live `/chatter` log and topic list, backed by one hosted ROS context. |
| ROS2.Net Avalonia App | `ros2net-avalonia` | Avalonia 12 desktop dashboard with the same teleop/pose/chatter panels. |

Every template accepts `--rosVersion` (default `0.1.0`) for the ROS2.Net package version.

## Quick start

```bash
dotnet new ros2net-node -n MyRobot --rmw inprocess
cd MyRobot && dotnet run
# another terminal (with --rmw rtps):  ros2net topic echo /chatter
```

```bash
dotnet new ros2net-interfaces -n FleetInterfaces --rosPackage fleet_msgs
dotnet build FleetInterfaces     # → ROS2.Net.Msgs.FleetMsgs.BatteryPack, DockRequest, PatrolGoal, …
```

## How they were verified

The template package was packed, installed into an isolated template hive, and each template was
instantiated and built against locally packed ROS2.Net packages (0 warnings). `ros2net-node` and
`ros2net-lifecycle` were run on the in-process middleware; the interfaces library was checked to
contain the generated types (also with a custom `--rosPackage`). The Blazor and Avalonia apps were
built but not exercised interactively.
