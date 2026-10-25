# Manipulation (MoveIt-style) — `ROS2.Net.Manipulation`

`ROS2.Net.Manipulation` covers the core of a MoveIt 2 workflow in managed code:

- **URDF** parsing and a `robot_state_publisher` equivalent.
- **Kinematics**: forward kinematics, geometric Jacobian, and numerical inverse kinematics with joint limits.
- **Collision checking** against a planning scene.
- **Planning**: RRT-Connect, Cartesian straight-line paths, and time parameterization.
- A **MoveGroup** facade that executes plans through `control_msgs/FollowJointTrajectory`.
- A **pick-and-place** pipeline.

```
dotnet add package ROS2.Net.Manipulation
```

## Quick start

```csharp
using ROS2.Net.Manipulation;

var model = UrdfModel.LoadBuiltIn("ur5");                 // bundled: ur5, diffbot, planar2 — or UrdfModel.LoadFile(path)
var arm = new KinematicChain(model, "base_link", "tool0");

var pose = arm.ForwardKinematics([0, -1.2, 1.5, -1.9, -1.57, 0]);
var ik = new IkSolver(arm).Solve(pose with { Position = pose.Position + new Vec3(0, 0, -0.1) });
Console.WriteLine(ik.Success ? string.Join(", ", ik.Solution) : "unreachable");
```

With ROS running (for example ROS2.Net's `JointTrajectoryController` on a `GenericSystem`):

```csharp
var node = context.CreateNode("mover");
var group = new MoveGroup(node, arm);                      // current state from /joint_states
group.Scene.AddBox("table", new Vec3(0.5, 0, -0.05), new Vec3(1.0, 1.0, 0.1));
await group.MoveToPoseAsync(Pose3.FromXyzRpy(0.4, 0.2, 0.3, Math.PI, 0, 0));
```

## URDF and robot_state_publisher

`UrdfModel.Parse(xml)` reads:

- links with inertials and visual/collision geometry (box, cylinder, sphere, mesh);
- every joint type (fixed, revolute, continuous, prismatic, floating, planar), with origins (`xyz`/`rpy`), axes, limits and mimic.

It validates that the model has a single root.

`new RobotStatePublisher(context, model)` behaves like `robot_state_publisher`:

- publishes the URDF on latched `/robot_description` and as a read-only parameter;
- publishes fixed joints once on `/tf_static`;
- publishes movable joints on `/tf` from `/joint_states`, with mimic joints resolved. `frame_prefix` is supported.

## Kinematics

- `KinematicChain(model, base, tip)` is a serial chain over the movable joints, ordered from base to tip.
  - `ForwardKinematics(q)` returns the tip pose.
  - `LinkPoses(q)` returns every link along the chain.
  - `Jacobian(q, span)` fills the 6×n geometric Jacobian (linear rows, then angular), expressed in the base frame.
  - Mimic joints inside the chain follow their master joint.
- `RobotKinematics(model)` computes forward kinematics for the whole tree, for every link.
- `IkSolver` runs **Levenberg–Marquardt** (the default, with adaptive damping) or **damped least squares**.
  - Results are clamped to joint limits.
  - It restarts from random configurations when the seed fails.
  - Options: `PositionOnly`, tolerances, and a `Validator`, for example to require collision-free solutions.
- `IkSolver.TwoLinkPlanar(x, y, l1, l2, elbowUp)` is a closed-form solution for planar 2-link arms.

Math types: `Vec3`, `Quat` (RPY conversions, slerp, rotation vectors), and `Pose3` (composition, inverse, 4×4 matrix, and conversion to and from geometry_msgs).

## Collision checking

- **Primitives**: `SphereShape`, `CapsuleShape` and `BoxShape` (oriented, separating-axis test). Distances are exact for sphere and capsule pairs; box–capsule uses a convex line search.
- **URDF geometry**: cylinders become capsules, which is conservative at the caps. **Meshes are ignored**, so approximate them with primitives in the `<collision>` tags.
- **`PlanningScene`** holds world objects (`AddBox`, `AddSphere`, `AddWorldObject`), attached objects (`Attach`/`Detach` with touch links) and an allowed collision matrix.
  - Adjacent links are allowed by default.
  - `AllowCollisionsAt(state)` disables pairs that already touch in a reference pose, like the MoveIt setup assistant's "default" pairs.
  - `Padding` adds a safety margin.
  - `IsStateValid(chain, q)` checks one state.

## Planning

| Class | What it does |
|---|---|
| `RrtConnectPlanner` | Bidirectional RRT-Connect in joint space. Edges are collision-checked at `EdgeResolution`, then shortcut-smoothed. Seeded, so results are deterministic. Options cover step size, iterations and timeout. |
| `CartesianPlanner` | Straight tip motion: linear position and slerped orientation. Each step's IK is seeded from the previous one. It guards against joint jumps and reports the feasible `Fraction`. |
| `TimeParameterization` | Timed `trajectory_msgs/JointTrajectory` with **trapezoidal** (time-optimal) or **quintic** segments. All joints are synchronized per segment and respect velocity and acceleration limits, with scaling factors. |
| `MoveGroup` | `PlanToJointTarget`, `PlanToPose` (collision-free IK, then RRT), `PlanCartesian`, `ExecuteAsync` (via the FollowJointTrajectory action), and `MoveTo…Async`. |
| `PickAndPlace` | Open → pre-grasp → approach → grasp (attach) → retreat → pre-place → place → release (detach) → retreat. You pass the gripper as a delegate, for example one wrapping `GripperActionController`. |

## Differences from MoveIt

MoveIt is large. This package is a compact, dependency-free subset:

- **IK** is numerical only: no IKFast, no TRAC-IK, and no per-group solver configuration (SRDF is not parsed).
- **Collision checking** uses primitives only. There is no FCL, no mesh or octomap geometry, and no continuous collision checking.
- **Planning** offers RRT-Connect and Cartesian paths only. There is no OMPL planner zoo, no constraint-aware planning, and no CHOMP/STOMP/Pilz.
- **Time parameterization** stops at every waypoint (rest-to-rest). It does not blend through waypoints like TOTG or Ruckig.
- **MoveGroup** is an in-process API. There is no `move_group` node or MoveIt action interface.

---

Made by Gravicode Studios, led by Kang Fadhil.
