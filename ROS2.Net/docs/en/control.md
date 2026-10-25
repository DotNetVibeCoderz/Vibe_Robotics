# Control (ros2_control) — `ROS2.Net.Control`

`ROS2.Net.Control` brings the ros2_control architecture to .NET: hardware components export **state** and
**command interfaces**, a **controller manager** runs a real-time `read → update → write` loop, and
controllers claim command interfaces exclusively. The controller manager exposes the standard
`controller_manager_msgs` services, so `ros2 control list_controllers` works against a ROS2.Net process.

```
dotnet add package ROS2.Net.Control
```

## Quick start: a differential drive robot

```csharp
using ROS2.Net;
using ROS2.Net.Control;

await using var context = await Ros2Context.CreateAsync();

var resources = new ResourceManager();
resources.Add(new SimulatedDiffDriveHardware());          // or your own HardwareComponent

var cm = new ControllerManager(context, resources);        // node "controller_manager", update_rate 100 Hz
cm.Spawn(new JointStateBroadcaster(), "joint_state_broadcaster");
cm.Spawn(new DiffDriveController(), "diff_drive_controller",
[
    new Parameter("left_wheel_names", new[] { "left_wheel_joint" }),
    new Parameter("right_wheel_names", new[] { "right_wheel_joint" }),
    new Parameter("wheel_separation", 0.34),
    new Parameter("wheel_radius", 0.05),
]);

cm.Start();                       // dedicated high-priority control thread
await context.SpinAsync();        // services, cmd_vel subscriptions, publishers
```

Drive it with `ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`.

## Concepts

| Piece | Role |
|---|---|
| `InterfaceSlot` | A shared `double` field. Hardware writes it, controllers read/write it through handles — no copies, no locks. |
| `StateInterface` / `CommandInterface` | Named handles `joint/position`, `joint/velocity`, `joint/effort`, … |
| `IHardwareInterface` / `HardwareComponent` | Hardware with a lifecycle (`Configure`, `Activate`, `Deactivate`) and `Read`/`Write` per cycle. |
| `GenericSystem` | Mock hardware like `mock_components/GenericSystem`: mirrors commands to states, optional first-order lag (`TimeConstant`), integrates velocity commands. |
| `SimulatedDiffDriveHardware` | Two wheels with velocity commands and position/velocity states. |
| `ResourceManager` | Owns hardware and interfaces; claims are all-or-nothing and exclusive. |
| `ControllerManager` | A `Node` that loads, configures, switches and runs controllers. |
| `ControllerBase` | Base class of controllers: one node per controller, `CommandInterfaceConfiguration`, `StateInterfaceConfiguration`, `Update(time, period)`. |
| `RealtimeBuffer<T>` | Lock-free hand-off from subscription callbacks to the control loop. |

### Controller lifecycle

`unconfigured → (configure) → inactive → (activate via switch) → active`. `SwitchController(activate, deactivate, strictness)`
applies between two cycles. **Strict** checks every claim first and changes nothing on failure; **best effort**
does what it can. Deactivating and activating in the same call lets a controller take over interfaces another one
releases.

### Parameters

Each controller gets a node named after it, so YAML parameters go under the controller name:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 200
    diff_drive_controller.type: diff_drive_controller/DiffDriveController
diff_drive_controller:
  ros__parameters:
    left_wheel_names: [left_wheel_joint]
    right_wheel_names: [right_wheel_joint]
    wheel_separation: 0.34
    wheel_radius: 0.05
```

Pass it with `--ros-args --params-file controllers.yaml` and call `cm.LoadController("diff_drive_controller")`
(type from `<name>.type`), or the `~/load_controller` service. Parameters written on the manager as
`<controller>.<param>` are forwarded too.

## Services

| Service | Type |
|---|---|
| `~/list_controllers` | `controller_manager_msgs/srv/ListControllers` |
| `~/list_hardware_interfaces` | `controller_manager_msgs/srv/ListHardwareInterfaces` |
| `~/load_controller`, `~/configure_controller`, `~/unload_controller` | `controller_manager_msgs/srv/*Controller` |
| `~/switch_controller` | `controller_manager_msgs/srv/SwitchController` |

The message layouts follow **Humble** (`SwitchController` still carries `start_controllers`/`stop_controllers`,
which are merged with `activate_`/`deactivate_controllers`). Jazzy removed those fields; a Jazzy `ros2 control`
client will not match these services — this is a known gap.

## Built-in controllers

| Type name | Class | Highlights |
|---|---|---|
| `joint_state_broadcaster/JointStateBroadcaster` | `JointStateBroadcaster` | `/joint_states` from every position/velocity/effort state; `publish_rate` (0 = every cycle), `joints` filter. |
| `forward_command_controller/ForwardCommandController`, `position_controllers/JointGroupPositionController`, `velocity_controllers/JointGroupVelocityController`, `effort_controllers/JointGroupEffortController` | `ForwardCommandController` | `~/commands` (`std_msgs/Float64MultiArray`) → one interface per joint. |
| `diff_drive_controller/DiffDriveController` | `DiffDriveController` | `~/cmd_vel` (Twist, or TwistStamped with `use_stamped_vel`), RK2 odometry from wheel positions (or `open_loop`), `~/odom` + odom→base_link on `/tf`, `linear.x.*`/`angular.z.*` velocity & acceleration limits, `cmd_vel_timeout`. |
| `ackermann_steering_controller/AckermannSteeringController` | `AckermannSteeringController` | Bicycle model: `traction_joints` (velocity), `steering_joints` (position), `wheelbase`, `wheel_radius`, `max_steering_angle`. |
| `joint_trajectory_controller/JointTrajectoryController` | `JointTrajectoryController` | `control_msgs/action/FollowJointTrajectory` on `~/follow_joint_trajectory` and `~/joint_trajectory`; linear / cubic / quintic splines depending on what the points carry; path & goal tolerances (`constraints.<joint>.trajectory/goal`, `constraints.goal_time`, `stopped_velocity_tolerance`); new goals preempt; position or velocity (feed-forward + `gains.<joint>.p`) command interfaces. |
| `position_controllers/GripperActionController` | `GripperActionController` | `~/gripper_cmd` (`ros2net_control_msgs/action/GripperCommand`: `position`, `max_effort` → `position`, `effort`, `stalled`, `reached_goal`), stall detection by velocity or effort. |

`PidController` is a reusable discrete PID (integral clamp, conditional integration anti-windup,
derivative-on-measurement, derivative filter, output limits).

## Writing a controller

```csharp
public sealed class HoldController : ControllerBase
{
    private string[] _joints = [];

    public override InterfaceConfiguration CommandInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_joints.Select(j => j + "/position"));
    public override InterfaceConfiguration StateInterfaceConfiguration =>
        InterfaceConfiguration.Individual(_joints.Select(j => j + "/position"));

    protected override void OnInit() => Node.DeclareParameter("joints", Array.Empty<string>());

    protected override CallbackReturn OnConfigure()
    {
        _joints = Node.GetParameter<string[]>("joints");
        return CallbackReturn.Success;
    }

    public override ControllerReturn Update(RosTime time, RosDuration period)
    {
        for (var i = 0; i < CommandInterfaces.Count; i++)
            CommandInterfaces[i].Value = StateInterfaces[i].Value;   // no allocations here
        return ControllerReturn.Ok;
    }
}

cm.RegisterControllerType("my_pkg/HoldController", () => new HoldController());
```

## Real-time notes

- `ControllerManager.Start()` runs the loop on a dedicated `ThreadPriority.Highest` thread, sleeping coarsely and
  spinning the last ~2 ms for accurate periods (Windows timer resolution is raised to 1 ms). `OverrunCount` counts
  late cycles. For deterministic simulation and tests call `cm.Update(time, period)` yourself instead.
- The math in the update path (interface access, PID, odometry, spline sampling) does not allocate. Publishing
  messages (`/joint_states`, `/odom`, `/tf`) allocates a message per publish — throttle it with `publish_rate`.
- Commands from topics and actions reach the loop through `RealtimeBuffer<T>` (an atomic reference swap).

## Differences from ros2_control

ROS2.Net's ros2_control is a managed re-implementation, not a binding:

- Hardware and controllers are .NET classes registered in code. There is no pluginlib or `<ros2_control>` URDF tag parsing.
- There are no chained controllers and no reference interfaces. `is_chainable` is always false.
- There is no `list_controller_types`, `reload_controller_libraries` or `set_hardware_component_state` service.
- Controllers use a simplified per-controller `update_rate` throttle.

---

Made by Gravicode Studios, led by Kang Fadhil.
