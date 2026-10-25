# Navigation — Nav2 for .NET

`ROS2.Net.Navigation` is a Nav2-style navigation stack: maps and layered costmaps, global planners, local controllers, recovery behaviors, a behavior tree engine, AMCL and EKF localization, and server nodes with Nav2's topic, action and message names. A stock Nav2 client such as RViz's "Nav2 Goal" tool, `ros2 action send_goal` or a Python `BasicNavigator` can drive it. Its BT navigator can also drive stock Nav2 planner and controller servers.

```bash
dotnet add package ROS2.Net.Navigation
```

The package depends only on `ROS2.Net.Core` and `ROS2.Net.Msgs`. It uses planar poses (`Pose2D`, SE(2)) instead of TF. The robot pose comes from an `IRobotPoseProvider`, described below.

## Maps and costmaps

```csharp
var map = OccupancyGridMap.LoadMapServer("maps/office.yaml");   // map_server YAML + PGM (P5/P2)
map.SaveMapServer("maps/office_copy");                          // writes .yaml + .pgm (trinary)
double range = map.Raycast(x, y, angle, maxRange: 10);          // exact DDA ray cast

var costmap = Costmap2D.FromMap(map, trackUnknown: true)
    .AddLayer(new StaticLayer(map, trackUnknown: true))
    .AddLayer(new ObstacleLayer())                              // marks/clears from LaserScan
    .AddLayer(new InflationLayer(inflationRadius: 0.55, costScalingFactor: 3.0, inscribedRadius: 0.22));
costmap.GetLayer<ObstacleLayer>()!.AddScan(costmap, scan, sensorPose);
costmap.Update();
```

Costs follow `nav2_costmap_2d`:

| Cost | Meaning |
|---|---|
| 0 | free |
| 1–252 | inflated |
| 253 | inscribed (the footprint touches an obstacle) |
| 254 | lethal |
| 255 | unknown |

Inflation uses Nav2's formula `252 · exp(−cost_scaling_factor · (d − inscribed_radius))`. `Costmap2D.ToMsg()` applies Nav2's translation to `nav_msgs/OccupancyGrid`, so the costmap can be shown in RViz.

`Footprint.Circle(r)`, `Footprint.Rectangle(l, w)` and `Footprint.FromPolygon(...)` provide `Cost()` and `InCollision()` checks.

## Global planners (`IGlobalPlanner`)

| Planner | Notes |
|---|---|
| `AStarPlanner` | 8-connected, octile heuristic, no corner cutting, cost-aware (`CostWeight`) |
| `DijkstraPlanner` | NavFn-style uniform search |
| `ThetaStarPlanner` | any-angle paths via line-of-sight parent shortcuts |
| `HybridAStarPlanner` | Smac-like (x, y, θ) search with arc primitives at `MinTurningRadius`, optional reverse (`AllowReverse`), an obstacle-aware 2-D heuristic and analytic Dubins expansion |
| `PathSmoother` | gradient-descent smoother that keeps endpoints fixed and avoids obstacles |

```csharp
PlanResult plan = new AStarPlanner().CreatePlan(costmap, start, goal);
if (plan.Success) path.Publish(PathUtils.ToMsg(plan.Poses, "map", RosTime.Now()));
```

Planning failures return `Success = false` with an `Error` instead of throwing: "no path found", "goal is in an obstacle", "start is outside the costmap", and so on. Search buffers are reused, so replanning on the same costmap does not allocate per cell.

## Controllers (`IController`)

| Controller | Notes |
|---|---|
| `PurePursuitController` | arc through a lookahead point (κ = 2y/L²), rotates in place when the path is behind the robot |
| `RegulatedPurePursuitController` | adds velocity-scaled lookahead, and slows down for tight curvature, for inflation cost near the robot and on approach to the goal |
| `DwbController` | samples (v, ω) in the dynamic window, forward-simulates each, and scores the trajectories with critics (path distance, goal distance, path align, obstacle, prefer forward) |

Use `GoalChecker` (xy/yaw tolerance, stateful like Nav2's `SimpleGoalChecker`) and `ProgressChecker` (Nav2's `SimpleProgressChecker`) alongside them.

## Recovery behaviors

`SpinBehavior`, `BackUpBehavior`, `WaitBehavior` and `ClearCostmapBehavior` implement `IRecoveryBehavior`. Start a behavior with `Start(pose)`, then call `Update(pose, dt, out cmd)` until it returns `Succeeded` or `Failed`. Spin and back-up accept an optional `IsColliding` check.

## Behavior trees

`BtFactory` loads BehaviorTree.CPP XML in both the v3 (`<Action ID="X"/>`) and v4 (`<X/>`) forms:

- **Built-in control nodes:** `Sequence`, `ReactiveSequence`, `Fallback`, `ReactiveFallback`, `Parallel` (`success_count`/`failure_count`), and Nav2's `PipelineSequence`, `RoundRobin` and `RecoveryNode` (`number_of_retries`).
- **Built-in decorators:** `Inverter`, `ForceSuccess`, `ForceFailure`, `RetryUntilSuccessful`/`Retry` (`num_attempts`), `KeepRunningUntilFailure`, `RateController` (`hz`) and `Repeat`.
- **Other:** `SubTree`.

Ports use `{key}` for the blackboard and plain text for literals.

```csharp
var factory = new BtFactory()
    .RegisterCondition("BatteryOk", n => battery > n.GetInput("min", 0.2))
    .RegisterAsyncAction("Dock", async (n, ct) => await DockAsync(ct) ? NodeStatus.Success : NodeStatus.Failure);
var tree = factory.CreateTreeFromText(xml);
tree.Blackboard.Set("goal", pose);
var status = await tree.TickWhileRunningAsync(TimeSpan.FromMilliseconds(100), ct);
```

Async actions report `Running` until their task finishes. Halting them, for example from a reactive parent, cancels their token. `NavTrees.NavigateToPoseWithReplanningAndRecovery` is the equivalent of Nav2's default tree: it replans at 1 Hz while following the path, and recovers by clearing costmaps, spinning, waiting and backing up.

## Localization

- **`Amcl`** is a particle filter with the odometry motion model (`alpha1`–`alpha4`) and a likelihood-field laser model. It uses KLD-adaptive resampling (`MinParticles`/`MaxParticles`, `PfErr`, `PfZ`) and optional random-particle recovery (`RecoveryAlphaSlow`/`Fast`).
  - Odometry accumulates and is applied once per update, after `UpdateMinD` / `UpdateMinA` of motion, like `nav2_amcl`.
  - `LaserModel.LikelihoodFieldProduct` (the default) is a tempered product of beam probabilities and converges quickly. `LaserModel.LikelihoodField` reproduces nav2's `1 + Σpz³` combination.
- **`Ekf2D`** is a robot_localization-style EKF over (x, y, θ, v, ω). It fuses any subset of the measurements: `UpdateTwist` (wheel odometry), `UpdateYawRate` (gyro), `UpdateYaw` (compass/IMU orientation) and `UpdatePosition` (GPS/AMCL).

## ROS nodes

| Node | Default name | Interface |
|---|---|---|
| `MapServerNode` | `map_server` | latched `/map`, `/map_metadata`; service `/map_server/map` (`nav_msgs/srv/GetMap`) |
| `PlannerServerNode` | `planner_server` | action `compute_path_to_pose` (`nav2_msgs/action/ComputePathToPose`), publishes `/plan`; global costmap built from `/map`; `AddPlanner(id, planner)` (default id `GridBased` = A*) |
| `ControllerServerNode` | `controller_server` | action `follow_path` (`nav2_msgs/action/FollowPath`) → `/cmd_vel` at `controller_frequency`; new goals preempt; `AddController(id, controller)` (default `FollowPath` = RPP) |
| `BtNavigatorNode` | `bt_navigator` | action `navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) with feedback; its BT calls `compute_path_to_pose` and `follow_path` |
| `AmclNode` | `amcl` | `/map`, `/scan`, `/initialpose`, odometry → `/amcl_pose`, `/particle_cloud`, `map→odom` on `/tf` |
| `EkfNode` | `ekf_filter_node` | `/odom` + `/imu` → `/odometry/filtered` |
| `SlamNode` | `slam_toolbox` | `/scan` + odometry → `/map`, `map→odom` on `/tf` (see [slam.md](slam.md)) |

### Robot pose: `IRobotPoseProvider`

The servers read the robot pose from an `IRobotPoseProvider` instead of TF:
- `OdometryPoseProvider` follows `/odom` without needing an executor. It reports `Correction ⊕ odom`, so you can feed it AMCL's or SLAM's map→odom correction: `amcl.CorrectionUpdated += c => pose.Correction = c;`.
- `DelegatePoseProvider` wraps any function, such as a simulator, a TF buffer or motion capture.

### A complete stack in one process

```csharp
await using var ctx = await Ros2Context.CreateAsync();
var tools = ctx.CreateNode("nav_app");
using var pose = new OdometryPoseProvider(tools);

new MapServerNode(ctx, OccupancyGridMap.LoadMapServer("map.yaml"));
new PlannerServerNode(ctx, pose);
new ControllerServerNode(ctx, pose);
new BtNavigatorNode(ctx, pose);
await ctx.SpinAsync(ct, new MultiThreadedExecutor());
```

Send a goal from C# with `CreateActionClient<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback>("/navigate_to_pose")`. You can also send one from any ROS 2 tool, provided it shares the middleware.

## Differences from Nav2

- Recoveries run inside the BT navigator on `/cmd_vel`. There is no separate behavior server, and no `nav2_msgs` Spin/BackUp/Wait actions.
- `ClearEntireCostmap` calls `BtNavigatorNode.ClearCostmaps` rather than a costmap service.
- `GoalUpdated` is always false. To change the goal, send a new one.
- There are no rolling-window local costmaps, keepout/speed filters, or collision monitor. The planner server's costmap uses static, obstacle and inflation layers.
- `nav2_msgs` are defined at the Humble layout (`NavigateToPose` result is `std_msgs/Empty`). Later distros added error codes to the results.
- The servers are plain `Node`s, not lifecycle nodes, so there is no `lifecycle_manager` handshake.

## Tests

`tests/ROS2.Net.Navigation.Tests` covers:
- map and PGM round trips, inflation values, and obstacle mark/clear
- every planner on a wall-with-gap map, including failure when the gap is blocked, and the curvature bound of the Hybrid-A* paths
- all three controllers reaching the goal of an L-shaped path
- the recovery behaviors and the BT engine semantics
- AMCL convergence, EKF versus dead reckoning, and the pose graph
- an end-to-end test where map server, planner, controller and BT navigator drive a kinematic robot around an obstacle through the `NavigateToPose` action

Interoperability with a real Nav2/ROS 2 graph depends on the middleware (RTPS or rosbridge), and has not been verified against real ROS 2 installs.
