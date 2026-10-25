# Ros2NetInterfaces

Custom ROS 2 interfaces for the `my_robot_msgs` package, generated into C# at compile time by
`ROS2.Net.Rosidl.Generator` (ROS2.Net — made by Gravicode Studios, led by Kang Fadhil).

| File | Generated C# |
|---|---|
| `interfaces/my_robot_msgs/msg/BatteryCell.msg` | `ROS2.Net.Msgs.<PascalCasePackage>.BatteryCell` |
| `interfaces/my_robot_msgs/msg/BatteryPack.msg` | `...BatteryPack` |
| `interfaces/my_robot_msgs/srv/Dock.srv` | `...DockRequest`, `...DockResponse` |
| `interfaces/my_robot_msgs/action/Patrol.action` | `...PatrolGoal`, `...PatrolResult`, `...PatrolFeedback` |

The namespace is the package name in PascalCase (`my_robot_msgs` → `ROS2.Net.Msgs.MyRobotMsgs`).
Add more files under `interfaces/my_robot_msgs/{msg,srv,action}/` and rebuild; syntax errors are reported
as `ROSIDL001` compiler errors. Reference this project from your nodes:

```csharp
var pub = node.CreatePublisher<ROS2.Net.Msgs.MyRobotMsgs.BatteryPack>("battery_pack");
```

Other ROS 2 systems need the same `.msg` files built in their workspace (colcon) to decode these types.
