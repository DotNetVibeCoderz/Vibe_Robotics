# ROS2.Net — Progress / Kemajuan

> Made by **Gravicode Studios**, led by **Kang Fadhil** · Dibuat oleh **Gravicode Studios** dipimpin oleh **Kang Fadhil**

Roadmap: [PLAN.md](PLAN.md). Status legend: ✅ done & tested · 🟡 done, not verified against upstream ROS 2 · ⏳ planned.

_Last update: 2026-10-25 — version 0.1.0 (preview). Full solution: 43 projects, Release build 0 warnings; 309 tests passing (1 opt-in screenshot test skipped)._

## Libraries

| Area | Project | Status | Tests | Notes |
|---|---|---|---|---|
| Codegen | ROS2.Net.Rosidl, .Rosidl.Generator | ✅ | in Core tests | `.msg/.srv/.action` parser, Roslyn generator, diagnostics ROSIDL001/002 |
| Core | ROS2.Net.Core | ✅ | 64 | Node, pub/sub, services, actions, parameters (+YAML, services, events), lifecycle, composition, executors (single/multi/realtime), QoS + events, sim time, CDR/JSON, dynamic types, in-process RMW |
| Messages | ROS2.Net.Msgs | ✅ | in Core tests | 127 standard interfaces + 36 core interfaces |
| RTPS middleware | ROS2.Net.Rmw.Rtps | 🟡 | 29 | SPDP/SEDP, reliable/best-effort, DATA_FRAG, transient local, services (Fast DDS & Cyclone mappings), `ros_discovery_info`. Tested ROS2.Net↔ROS2.Net over UDP loopback; **interop with Fast DDS/Cyclone not yet verified**. No DDS-Security, SHM, IPv6, type hashes |
| rosbridge middleware | ROS2.Net.Rmw.Rosbridge | 🟡 | 14 (shared) | Tested against an in-test fake rosbridge server; not yet against rosbridge_suite |
| Hosting | ROS2.Net.Hosting | ✅ | (shared) | `AddRos2`, DI nodes, config binding |
| Security | ROS2.Net.Security | 🟡 | (shared) | SROS2 keystore/enclaves/permissions with S/MIME signing; not yet loaded by a real DDS |
| TF2 | ROS2.Net.Tf2 | ✅ | 20 (shared) | Buffer, listener, broadcasters, time travel, frame graph export |
| rosbag2 | ROS2.Net.Bag | 🟡 | (shared) | MCAP (none/zstd/lz4), metadata.yaml v5, recorder/player/reader/exporter; not opened with `ros2 bag`/Foxglove yet |
| Navigation & SLAM | ROS2.Net.Navigation | ✅ | 34 | Costmaps, A*/Dijkstra/Theta*/Hybrid-A*, Pure Pursuit/RPP/DWB, BT engine (BT.CPP v3/v4 XML), AMCL, EKF, graph SLAM, Nav2-named servers (Humble message layout) |
| Perception | ROS2.Net.Perception (+.Onnx) | ✅ | 44 | PointCloud2 I/O, filters, segmentation, ICP, images, PNG, camera models, YOLO post-processing (no real model run) |
| Native | ROS2.Net.Native + Rust `ros2net_native` | ✅ | 8 Rust + parity tests | ABI v1, managed fallback; only win-x64 binary built locally (CI builds Linux) |
| Control | ROS2.Net.Control | ✅ | 30 (shared) | Controller manager + services (Humble layout), PID, diff drive, Ackermann, JTC, gripper, broadcasters |
| Manipulation | ROS2.Net.Manipulation | ✅ | (shared) | URDF, FK/Jacobian/IK, collision primitives, RRT-Connect, Cartesian, time parameterization, MoveGroup facade, robot_state_publisher |
| Simulation | ROS2.Net.Simulation | ✅ | 43 (shared) | Deterministic 2.5D world, 6 scenarios, diff/Ackermann/omni robots, LiDAR/IMU/GPS/camera/battery, ROS bridge node, turtlesim mode |
| AI | ROS2.Net.AI | ✅ | (shared) | RLNet gym env, Q-learning/DQN factory, ONNX runner, LLM commander (OpenAI-compatible, offline rule model EN/ID), anomaly detection |
| Blazor UI | ROS2.Net.Blazor | ✅ | 12 (+1 screenshot) | 14 components |
| Avalonia UI | ROS2.Net.Avalonia | ✅ | 8 (headless) | 14 controls |

## Apps & tools

| Item | Status | Notes |
|---|---|---|
| Gallery Web (Blazor) | ✅ | 16 live demo pages + code tabs, EN/ID, dark/light — screenshots in `docs/images/web` |
| Gallery Desktop (Avalonia) | ✅ | 17 pages, EN/ID, dark/light — screenshots in `docs/images/desktop` |
| Robot Simulator (Avalonia) | ✅ / 🟡 3D | 2D view, scenarios, multi-robot, sensors, middleware switch; ThreeNet 3D tab compiled but not rendered headless |
| `ros2net` CLI | ✅ | topic/node/service/action/param/interface/bag/security/doctor/run-demo; verified cross-process over RTPS |
| `dotnet new` templates | ✅ | node, lifecycle, interfaces, blazor, avalonia — all build against locally packed packages |
| ROS2AppGen (App Generator) | ✅ | Avalonia code-editor app with Jack - The Code Bender (Semantic Kernel: OpenAI, Azure OpenAI, Claude, Gemini, Ollama, DeepSeek), VS Code-style explorer, AvaloniaEdit editor, chat panel, build/run/deploy/format, 38 ROS2.Net templates in 13 categories, run targets (simulator vs physical robot). Live-tested with Azure OpenAI, DeepSeek and Claude; Gemini/Ollama not run live |
| Samples | ✅ | 6 console samples, run successfully |
| NuGet icon | ✅ | `assets/icon.png` |
| Docs EN/ID | ✅ | `docs/en`, `docs/id` (25 pages each) |
| CI / release workflow | ✅ | `.github/workflows/ros2net.yml` in the monorepo (tag `ros2net-v*`) |

## Known gaps / Kekurangan yang diketahui
- No real ROS 2 install was available during development: wire interop (RTPS, rosbridge, bag files, SROS2) is spec-based and self-tested only.
- Service/action calls on types unknown at compile time are not supported by the CLI (Core has no dynamic service API).
- RTPS: no DDS-Security, shared memory, IPv6, liveliness (WLP), XCDR2, keyed topics, RIHS01 type hashes.
- Nav2/ros2_control message layouts follow Humble.
- JPEG codec not implemented; ONNX detector not run on a real model.

## Log
- **2026-10-25** — ROS2AppGen (backend + UI), run targets for simulator vs physical robot; moved into the `Vibe_Robotics` monorepo.
- **2026-10-24** — Core, codegen, messages, in-process middleware, 64 tests; samples; icon; roadmap.
- **2026-10-24/25** — RTPS, rosbridge, hosting, security, TF2, rosbag2, navigation/SLAM, perception + Rust native, control/manipulation, simulation/AI, CLI, templates, Blazor + Avalonia components, galleries, simulator, bilingual docs, screenshots.
