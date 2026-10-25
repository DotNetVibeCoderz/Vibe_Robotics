# Simulation (`ROS2.Net.Simulation`)

A deterministic 2.5D robot simulator that speaks ROS 2. Worlds are planar, so physics, LiDAR and maps stay fast. Obstacles and robots also carry heights, so renderers can draw them in 3D. Everything runs in-process: no Gazebo is needed.

> Made by Gravicode Studios, led by Kang Fadhil.

## Quick start

```csharp
using ROS2.Net;
using ROS2.Net.Simulation;

await using var context = await Ros2Context.CreateAsync();
var world = Scenarios.Create(ScenarioKind.Warehouse, robots: 2);   // robot1, robot2
using var sim = new SimulationNode(context, world);
await sim.RunAsync(cancellationToken);   // steps in real time and spins the services
```

Any node can now drive `/robot1/cmd_vel` and read `/robot1/odom`, `/robot1/scan` and the other topics. That includes nodes on other machines when the RTPS middleware is used.

## The world

| Concept | API |
|---|---|
| World | `SimWorld`: fixed step (default 100 Hz), `Step()`, `StepFor(seconds)`, `RunAsync(ct)`, `Paused`, `RealTimeFactor` (0 = as fast as possible), `Reset()` |
| Determinism | `SimWorldOptions.Seed`. All noise comes from one seeded `SimRandom`, so equal seeds produce identical runs. |
| Obstacles | `SimObstacle.Box / Cylinder / Wall / Polygon`. With `dynamic: true`, robots can push the obstacle; heavier objects move less. |
| Terrain | `Friction` (traction factor) and `FloorColor` |
| Queries | `RayCast`, `IsOccupied`, `SampleColor`, and `ToOccupancyGrid(resolution)`, which returns the ground-truth `nav_msgs/OccupancyGrid` |

Static obstacles are indexed in a uniform grid, and rays walk it with a DDA traversal. The physics step allocates nothing: a test asserts this for three fully equipped robots in the warehouse.

## Robots

| Type | Commands | Notes |
|---|---|---|
| `DiffDriveRobot` | `linear.x`, `angular.z` | TurtleBot3 Burger dimensions by default (`RobotParameters.TurtleBot3Waffle()` is also available); wheel slip and encoder noise |
| `AckermannRobot` | `linear.x`, `angular.z` (yaw rate → steering angle) | Bicycle model; set `AngularIsSteeringAngle` to command the steering angle directly |
| `OmniRobot` | `linear.x`, `linear.y`, `angular.z` | Holonomic base |

Commands older than `CommandTimeout` (0.5 s) are treated as zero. Velocity changes are limited by the acceleration limits. Collisions push the robot out of the obstacle and set `Bumper`. Odometry integrates the encoder readings, so it drifts away from the ground truth over time, as on a real robot.

`SimArm.TwoDof()` and `SimArm.SixDof()` attach a kinematic arm placeholder. Its joints move toward their targets at a limited speed, and the joint values appear in `joint_states` and in snapshots.

`RobotFactory.Create(kind, name, pose, parameters, sensors)` mounts the standard sensors:

| Sensor | Class | Default |
|---|---|---|
| 2D LiDAR | `LidarSensor` | 360 beams, 0.12–3.5 m, 10 Hz, σ = 1 cm, optional dropout |
| IMU | `ImuSensor` | 100 Hz, gyro noise and a random-walk bias, accelerometer including gravity |
| GPS | `GpsSensor` | 5 Hz, σ = 0.5 m around `GeoOrigin` (Bandung by default) |
| Sonar | `RangeSensor` | front and rear, 0.5 rad cone |
| Battery | `BatterySensor` | Drains with motion. Use `IsCharging` to charge it, and `DrainFactor` to speed up the drain for demos. |
| Camera | `CameraSensor` | Top-down rgb8 image of the area ahead (off by default) |

## Scenarios

`Scenarios.Create(kind, robots)` builds a ready-made world. To edit one first, call `Scenarios.Definition(kind)`, change the result, and save it as JSON or YAML.

| Scenario | Size | Contents |
|---|---|---|
| `Empty` | 20×20 m | Open space |
| `Warehouse` | 24×16 m | 12 shelves with waypoints, pillars, 6 pushable crates, docks, a charger |
| `Office` | 16×12 m | Four rooms with doors, desks, a meeting table |
| `Maze` | 12×9 m | Generated from the seed, with `start` and `exit` points |
| `CorridorLoop` | 14×10 m | A 2 m wide loop, good for testing SLAM loop closure |
| `AgricultureRows` | 24×16 m | Crop rows, trees, a barn; lower traction and a farm geo-origin |

Example world file (YAML or JSON; angles are in degrees):

```yaml
name: lab
bounds: [0, 0, 6, 4]
obstacles:
  - { type: wall, x1: 0, y1: 0, x2: 6, y2: 0 }
  - { type: box, x: 3, y: 2, width: 1, depth: 0.5, yaw: 30, dynamic: true }
  - { type: cylinder, x: 5, y: 3, radius: 0.2 }
robots:
  - { name: robot1, type: diffdrive, preset: waffle, x: 1, y: 1, sensors: { camera: true } }
waypoints:
  - { name: dock, x: 0.5, y: 3.5, yaw: 180 }
```

Load it with `WorldDefinition.FromFile("lab.yaml").Build()`.

## ROS interface (`SimulationNode`)

For each robot, in its namespace (`/robot1`, or `/` when `Namespace = "/"`):

| Direction | Topic | Type |
|---|---|---|
| sub | `cmd_vel` | `geometry_msgs/Twist` |
| pub | `odom` | `nav_msgs/Odometry` (`robot1/odom` → `robot1/base_link`) |
| pub | `scan` | `sensor_msgs/LaserScan` (sensor-data QoS) |
| pub | `imu` | `sensor_msgs/Imu` |
| pub | `gps/fix` | `sensor_msgs/NavSatFix` |
| pub | `battery_state` | `sensor_msgs/BatteryState` |
| pub | `sonar_front`, `sonar_rear` | `sensor_msgs/Range` |
| pub | `camera/image_raw` | `sensor_msgs/Image` (rgb8) |
| pub | `joint_states` | `sensor_msgs/JointState` (wheels, steering, arm) |
| pub | `ground_truth/pose` | `geometry_msgs/PoseStamped` in `map` |
| srv | `teleport` | `turtlesim/srv/TeleportAbsolute` |

Global topics and services:

- `/tf`: odom → base_link for every robot. With `PublishMapToOdom`, it also carries a ground-truth map → odom transform.
- `/tf_static`: sensor mounts, latched.
- `/clock`: drives nodes started with `use_sim_time:=true`.
- `/map`: the ground-truth occupancy grid, latched.
- Services: `/sim/reset` (`std_srvs/Empty`), `/sim/pause` (`std_srvs/SetBool`), `/sim/spawn` (`turtlesim/Spawn` semantics) and `/sim/kill`.

TF frames carry a robot prefix (`robot1/base_link`), as multi-robot Nav2 setups expect.

Sensor data is published from the stepping thread. `cmd_vel` is read without the executor, so driving works even when the node is not spun. The services do need spinning; `RunAsync` handles both. To control time yourself, for example in tests or lock-step RL, set `RunWorld = false` and call `world.Step()`.

## turtlesim mode

`TurtlesimNode` is a drop-in replacement for `turtlesim_node`. It provides:

- Topics: `/turtle1/cmd_vel`, `/turtle1/pose` and `/turtle1/color_sensor`.
- Services: `/spawn`, `/kill`, `/clear` and `/reset`.
- Per turtle: `set_pen`, `teleport_absolute`, `teleport_relative` and the `rotate_absolute` action.

The standard ROS 2 beginner tutorials therefore run unchanged. `TurtleWorld.Snapshot()` exposes the turtles and the pen drawing for a UI.

## Rendering API (for UIs)

`SimWorld.Snapshot()` returns an immutable `WorldSnapshot` and is cheap to call every frame. The `StaticObstacles` array is shared between snapshots until the static geometry changes.

- `ObstacleSnapshot`: `Kind`, `Center`, `Rotation`, `Size`, `Radius`, world-space `Vertices`, `Height`, `Color`, `IsDynamic`, `Tag`.
- `RobotSnapshot`: `Pose`, `OdomPose`, velocities, `Radius`, `Length`, `Width`, `Height`, `Color`, `Bumper`, `BatteryPercentage`, `SteeringAngle`, `LaserOrigin` and world-space `LaserHits` (for drawing rays), `Trail`, and the arm's `JointNames`, `JointPositions` and `ArmLinkLengths`.
- Events: `Stepped` (on the stepping thread), `WasReset` and `RobotsChanged`.

## Limitations

- The physics is kinematic with simple contact resolution, not rigid-body dynamics: robots do not tip, and crates do not rotate when pushed.
- The camera is a synthetic top-down view, not a perspective render.
