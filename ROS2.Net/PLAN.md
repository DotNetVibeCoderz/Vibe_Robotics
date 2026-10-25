# ROS2.Net — Development Plan / Rencana Pengembangan

> Made by **Gravicode Studios**, led by **Kang Fadhil** · Dibuat oleh **Gravicode Studios** dipimpin oleh **Kang Fadhil**

Progress against this plan is tracked in [Progress.md](Progress.md).

## Vision / Visi

A complete, pure .NET 10 implementation of the ROS 2 client stack — usable **without a ROS 2
installation**, wire-compatible with ROS 2 networks, and extended with the ecosystem
(TF2, rosbag2, Nav2-style navigation, SLAM, ros2_control, MoveIt-style manipulation, perception,
simulation, AI) plus Blazor/Avalonia UI components, a gallery, CLI, simulator and templates.

Implementasi lengkap stack client ROS 2 dalam .NET 10 murni — dapat dipakai **tanpa instalasi
ROS 2**, kompatibel di level protokol jaringan, dan dilengkapi ekosistemnya, komponen UI
Blazor/Avalonia, galeri, CLI, simulator, dan template proyek.

## Architecture / Arsitektur

```
 Apps:      Gallery.Web (Blazor) · Gallery.Desktop (Avalonia) · Simulator · CLI (ros2net) · Templates
 UI:        ROS2.Net.Blazor · ROS2.Net.Avalonia
 Ecosystem: Tf2 · Bag (MCAP) · Navigation (+SLAM, AMCL, EKF, BT) · Perception · Control · Manipulation
            Simulation · AI (RL, ONNX, LLM) · Security (SROS2) · Hosting (DI)
 Client:    ROS2.Net.Core  — Node, pub/sub, services, actions, parameters, lifecycle, executors, QoS
            ROS2.Net.Msgs  — standard interfaces (generated)   ROS2.Net.Rosidl(+Generator) — codegen
 RMW:       InProcess (zero copy) · Rtps (managed DDS-RTPS) · Rosbridge (WebSocket)
 Native:    ROS2.Net.Native → ros2net_native (Rust) for point-cloud / ray-tracing hot paths
```

## Milestones / Tahapan

### M1 — Core client library ✅ target 0.1
- CDR (XCDR1) serializer, JSON (rosbridge) mapping, type registry, dynamic types.
- rosidl parser + Roslyn source generator; standard interfaces.
- Node, publishers, subscriptions (callback & `IAsyncEnumerable`), timers, guard conditions.
- Services, actions, parameters (+YAML, services, events), lifecycle, composition, intra-process.
- Executors: single, multi-threaded, realtime; callback groups; QoS model & events; sim time.
- In-process middleware.

### M2 — Connectivity ✅ target 0.1
- Managed RTPS middleware (SPDP/SEDP, reliable/best-effort, fragmentation, ROS graph discovery).
- rosbridge middleware. Hosting/DI. SROS2 keystore tooling.

### M3 — Ecosystem ✅ target 0.1
- TF2 buffer/listener/broadcasters; rosbag2 MCAP record/play/info/export.
- Navigation: costmaps, A*/Theta*/Hybrid-A*, Pure Pursuit/RPP/DWA, recoveries, BT engine,
  AMCL, EKF, graph SLAM, Nav2-compatible servers.
- Perception (point clouds, images, camera models) with Rust acceleration.
- Control (ros2_control-style) and Manipulation (URDF, FK/IK, RRT-Connect, time parameterization).
- Simulation engine and AI integration (RL gym, ONNX, LLM commander).

### M4 — Tools & UX ✅ target 0.1
- Blazor & Avalonia component libraries; ROS2.Net Gallery (web + desktop).
- Robot Simulator desktop app; `ros2net` CLI; `dotnet new` templates; NuGet packages with icon.
- Bilingual documentation (EN/ID) with screenshots.

### M5 — Hardening (0.2)
- Interop test matrix against real ROS 2 Humble/Jazzy (Fast DDS, Cyclone DDS) in CI containers.
- DDS-Security (authentication/encryption plugins) in the RTPS middleware.
- Shared-memory transport between processes on one host; loaned messages.
- Type description / RIHS01 type hashes (Iron+) in discovery.
- MCAP zstd/lz4 compression; sqlite3 bag storage reader.

### M6 — Scale & ecosystem (0.3+)
- Zenoh middleware (rmw_zenoh interop).
- 3D navigation (voxel layers), MPPI controller, full Smac planners.
- MoveIt-grade planning (OMPL-style planners, collision meshes), grasp synthesis.
- GPU perception (CUDA/DirectML via ONNX Runtime), MediaPipe integration (MediaPipeNet),
  Hugging Face models (HF.Net), IoT bridges (IoTComNet), 3D visualization with ThreeNet.
- Fleet management (Open-RMF-style task allocation), cloud deployment recipes (Docker/Kubernetes).

## Principles / Prinsip
- Wire compatibility with upstream ROS 2 wherever a standard exists.
- Allocation-aware hot paths (pooled buffers, span-based CDR, array reuse).
- Every feature ships with tests, a sample or gallery page, and EN/ID docs.
- Honest status: unverified interop is labeled as such in Progress.md.
