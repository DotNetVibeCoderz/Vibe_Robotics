# Interfaces and code generation

ROS2.Net generates C# types from standard ROS 2 interface files (`.msg`, `.srv`, `.action`) at
compile time with a Roslyn source generator — the equivalent of `rosidl_generator_*`.

## Standard interfaces

`ROS2.Net.Core` carries the runtime's own interfaces (builtin_interfaces, unique_identifier_msgs,
action_msgs, rcl_interfaces, lifecycle_msgs, rosgraph_msgs, composition_interfaces).
`ROS2.Net.Msgs` carries std_msgs, std_srvs, geometry_msgs, sensor_msgs, nav_msgs, tf2_msgs,
visualization_msgs, trajectory_msgs, diagnostic_msgs, control_msgs (FollowJointTrajectory),
nav2_msgs (NavigateToPose, ComputePathToPose, FollowPath), example_interfaces and turtlesim.

## Your own interfaces

```xml
<ItemGroup>
  <PackageReference Include="ROS2.Net.Rosidl.Generator" PrivateAssets="all" />
  <RosInterface Include="interfaces/**/*.msg;interfaces/**/*.srv;interfaces/**/*.action" />
</ItemGroup>
```

Files must follow `<package>/<msg|srv|action>/<Name>.<ext>`, e.g.
`interfaces/my_robot_msgs/msg/BatteryCell.msg` → `ROS2.Net.Msgs.MyRobotMsgs.BatteryCell`.
Parse errors are reported as compiler errors (`ROSIDL001`) at the offending line.

## Mapping

| ROS | C# |
|---|---|
| `bool`, `byte`/`char`/`uint8`, `int8` … `uint64`, `float32`, `float64` | `bool`, `byte`, `sbyte` … `ulong`, `float`, `double` |
| `string`, `wstring`, `string<=N` | `string` |
| `T[N]` | `T[]` with exactly N elements (checked on serialize) |
| `T[]`, `T[<=N]` | `T[]` (bound checked) |
| `pkg/Type` | `ROS2.Net.Msgs.<Pkg>.Type` |
| field `linear_acceleration` | property `LinearAcceleration` (`[JsonPropertyName("linear_acceleration")]`) |
| constant `uint8 FOO=1` | `public const byte FOO = 1;` |
| service `SetBool` | `SetBoolRequest`, `SetBoolResponse`, static descriptor `SetBool` |
| action `Fibonacci` | `FibonacciGoal`, `FibonacciResult`, `FibonacciFeedback`, descriptor `Fibonacci` |

Field names that would clash with the class or generated members get a `Value` suffix
(`sensor_msgs/Range.range` → `RangeValue`).

## Generated members

- `Serialize(CdrWriter)` / `Deserialize(ref CdrReader)` — XCDR1 little-endian, identical to Fast DDS /
  Cyclone DDS; primitive arrays are bulk-copied; decoding reuses arrays of matching length.
- `WriteJson` / `ReadJson` — rosbridge-compatible JSON (`uint8[]` as base64).
- `Clone()` deep copy, `ToString()` as JSON, static `TypeName`, `DdsTypeName`, `Definition`,
  `Dependencies`.
- A module initializer registers every type in `RosTypeRegistry`, so tools can resolve types by
  name (`RosTypeRegistry.CreateMessage("sensor_msgs/msg/Imu")`) and build MCAP schemas
  (`GetFullDefinition`).

## Dynamic types

`DynamicMessageType` converts CDR ⇄ JSON for any type given its definition text — used by the CLI
(`topic echo` of unknown types) and bag tooling.

## Without the generator

`ROS2.Net.Rosidl` exposes the parser and emitter: `RosidlParser.ParseMessage(...)` +
`CSharpEmitter.EmitMessage(...)`; the CLI wraps it as `ros2net interface generate`.
