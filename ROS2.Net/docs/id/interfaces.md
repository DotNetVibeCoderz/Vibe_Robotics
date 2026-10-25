# Interface dan pembuatan kode

ROS2.Net membuat tipe C# dari file interface ROS 2 standar (`.msg`, `.srv`, `.action`) saat
kompilasi menggunakan Roslyn source generator — padanan `rosidl_generator_*`.

## Interface standar

`ROS2.Net.Core` membawa interface yang dibutuhkan runtime (builtin_interfaces,
unique_identifier_msgs, action_msgs, rcl_interfaces, lifecycle_msgs, rosgraph_msgs,
composition_interfaces). `ROS2.Net.Msgs` membawa std_msgs, std_srvs, geometry_msgs, sensor_msgs,
nav_msgs, tf2_msgs, visualization_msgs, trajectory_msgs, diagnostic_msgs, control_msgs
(FollowJointTrajectory), nav2_msgs (NavigateToPose, ComputePathToPose, FollowPath),
example_interfaces, dan turtlesim.

## Interface milik Anda sendiri

```xml
<ItemGroup>
  <PackageReference Include="ROS2.Net.Rosidl.Generator" PrivateAssets="all" />
  <RosInterface Include="interfaces/**/*.msg;interfaces/**/*.srv;interfaces/**/*.action" />
</ItemGroup>
```

File harus mengikuti pola `<paket>/<msg|srv|action>/<Nama>.<ext>`, contoh
`interfaces/my_robot_msgs/msg/BatteryCell.msg` → `ROS2.Net.Msgs.MyRobotMsgs.BatteryCell`.
Kesalahan parsing dilaporkan sebagai error kompilasi (`ROSIDL001`) tepat pada baris yang salah.

## Pemetaan tipe

| ROS | C# |
|---|---|
| `bool`, `byte`/`char`/`uint8`, `int8` … `uint64`, `float32`, `float64` | `bool`, `byte`, `sbyte` … `ulong`, `float`, `double` |
| `string`, `wstring`, `string<=N` | `string` |
| `T[N]` | `T[]` dengan tepat N elemen (dicek saat serialisasi) |
| `T[]`, `T[<=N]` | `T[]` (batas atas dicek) |
| `pkg/Type` | `ROS2.Net.Msgs.<Pkg>.Type` |
| field `linear_acceleration` | properti `LinearAcceleration` (`[JsonPropertyName("linear_acceleration")]`) |
| konstanta `uint8 FOO=1` | `public const byte FOO = 1;` |
| service `SetBool` | `SetBoolRequest`, `SetBoolResponse`, deskriptor statis `SetBool` |
| action `Fibonacci` | `FibonacciGoal`, `FibonacciResult`, `FibonacciFeedback`, deskriptor `Fibonacci` |

Nama field yang bentrok dengan nama kelas atau member hasil generate diberi akhiran `Value`
(`sensor_msgs/Range.range` → `RangeValue`).

## Member hasil generate

- `Serialize(CdrWriter)` / `Deserialize(ref CdrReader)` — XCDR1 little-endian, identik dengan
  Fast DDS / Cyclone DDS; array primitif disalin sekaligus (bulk copy); decoding memakai ulang array
  yang panjangnya sama.
- `WriteJson` / `ReadJson` — JSON yang kompatibel dengan rosbridge (`uint8[]` sebagai base64).
- `Clone()` salinan mendalam, `ToString()` berupa JSON, properti statis `TypeName`, `DdsTypeName`,
  `Definition`, `Dependencies`.
- Module initializer mendaftarkan setiap tipe ke `RosTypeRegistry`, sehingga tool dapat mencari
  tipe berdasarkan nama (`RosTypeRegistry.CreateMessage("sensor_msgs/msg/Imu")`) dan menyusun
  schema MCAP (`GetFullDefinition`).

## Tipe dinamis

`DynamicMessageType` mengonversi CDR ⇄ JSON untuk tipe apa pun berdasarkan teks definisinya —
dipakai oleh CLI (`topic echo` untuk tipe yang tidak dikenal) dan tool bag.

## Tanpa generator

`ROS2.Net.Rosidl` menyediakan parser dan emitter secara langsung:
`RosidlParser.ParseMessage(...)` + `CSharpEmitter.EmitMessage(...)`; CLI membungkusnya sebagai
`ros2net interface generate`.
