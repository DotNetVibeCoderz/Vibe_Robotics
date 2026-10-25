# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS2.Net — a pure .NET 10 implementation of the ROS 2 client stack plus ecosystem (no ROS 2 install
needed), by Gravicode Studios (led by Kang Fadhil). The brief is `ROS2.Net.txt` (Indonesian) with the
feature catalogue in `features.md`; roadmap in `PLAN.md`, honest status in `Progress.md` (keep it
updated as work lands). It lives as the `ROS2.Net/` folder of the `DotNetVibeCoderz/Vibe_Robotics`
monorepo; CI/release is `../.github/workflows/ros2net.yml` (tag `ros2net-vX.Y.Z` publishes to NuGet
using the `NUGET_API_KEY` repo secret).

## Commands

```bash
cargo build --release --manifest-path native/ros2net_native/Cargo.toml   # Rust kernels (optional; managed fallback exists)
dotnet build ROS2.Net.slnx
dotnet test ROS2.Net.slnx
dotnet test tests/ROS2.Net.Tests --filter "FullyQualifiedName~NodeTests.Talker_reaches_listener"   # single test
dotnet test tests/ROS2.Net.Avalonia.Tests --filter Category=Screenshots      # regenerate docs/images/desktop
ROS2NET_GALLERY_URL=http://localhost:5317 dotnet test tests/ROS2.Net.Blazor.Tests --filter Screenshots  # docs/images/web (gallery must be running)
dotnet pack ROS2.Net.slnx -c Release          # → artifacts/packages (needs README.md + assets/icon.png)
dotnet run --project apps/ROS2.Net.Cli -- doctor
```

Shared config: `Directory.Build.props` (net10.0, nullable, package metadata, icon/README packing,
`IsPackable=false` by default — libraries opt in), `Directory.Packages.props` (central package versions;
add versions there, not in csproj), `.editorconfig`. `ROS2.Net.slnx` is generated from `*/*/*.csproj`.

## Architecture (big picture)

- **Codegen:** `src/ROS2.Net.Rosidl` (parser + C# emitter, netstandard2.0 + net10.0) is compiled into the
  Roslyn generator `src/ROS2.Net.Rosidl.Generator`. Projects add `.msg/.srv/.action` files as
  `AdditionalFiles` laid out `<package>/<msg|srv|action>/Name.ext`. `interfaces/core/**` is generated into
  **Core** (runtime needs: builtin_interfaces, action_msgs, rcl_interfaces, lifecycle_msgs, …);
  `interfaces/standard/**` into **ROS2.Net.Msgs**. Other libraries generate private interfaces from their
  own `interfaces/` folder (Control: controller_manager_msgs; Perception: vision_msgs; RTPS:
  rmw_dds_common). Generated types live in `ROS2.Net.Msgs.<PascalPackage>`; a module initializer registers
  them in `RosTypeRegistry` (by `pkg/msg/Name`, `pkg/Name` and DDS name).
- **Core (`src/ROS2.Net.Core`):** `Ros2Context` → `Node` → entities. The middleware boundary is the typed
  `IRmwImplementation`/`IRmwNode` in `Rmw/RmwAbstractions.cs`; `RmwRegistry` picks one via
  `ROS2NET_RMW` (default `inprocess`). Incoming data goes into per-entity queues (`Waitable`), and an
  `Executor` runs one unit per dequeue honoring `CallbackGroup`s. Service responses / action results
  complete Tasks directly (no spin needed on the caller). Timers share one high-resolution thread
  (`TimerScheduler`) unless sim time is active. Actions are implemented generically over services/topics
  (`Actions/ActionWire.cs`), so they work on every middleware. Intra-process delivery
  (`IntraProcessManager`) is only enabled when the middleware supports ignoring local publications.
- **Middlewares:** in-process (in Core, zero-copy by reference), `ROS2.Net.Rmw.Rtps` (managed DDS-RTPS:
  SPDP/SEDP, reliable writers/readers, fragmentation, `ros_discovery_info` graph, Fast DDS/Cyclone service
  mappings), `ROS2.Net.Rmw.Rosbridge` (WebSocket JSON; uses generated `WriteJson/ReadJson`).
  CDR is XCDR1 LE via `CdrWriter` (pooled, reusable) / `CdrReader` (ref struct, validates lengths).
- **Ecosystem libraries** each reference only Core + Msgs (plus Simulation for AI): Tf2, Bag (MCAP +
  metadata.yaml), Navigation (+SLAM/AMCL/EKF/BT, Nav2-named servers), Perception (+Onnx) over
  `ROS2.Net.Native` (P/Invoke to Rust `native/ros2net_native` with managed twins; `ROS2NET_NATIVE=0`
  forces managed), Control, Manipulation, Simulation (`SimWorld.Snapshot()` is the render API), AI,
  Hosting (DI), Security (SROS2 keystores).
- **UI/apps:** `ROS2.Net.Blazor` + `apps/ROS2.Net.Gallery.Web`, `ROS2.Net.Avalonia` (code-only controls,
  no XAML) + `apps/ROS2.Net.Gallery.Desktop` + `apps/ROS2.Net.Simulator`, `apps/ROS2.Net.Cli` (dotnet tool
  `ros2net`), `templates/ROS2.Net.Templates`. Galleries run everything on the in-process middleware with
  the simulator.

## Gotchas

- Use aliases for message types that clash with BCL names: `using StdMsgs = ROS2.Net.Msgs.StdMsgs;`
  (`String`, `Int32`, …). `Simulation.Pose2D`/`Navigation.Pose2D` clash with `geometry_msgs/Pose2D`.
- Inside `ROS2.Net.*` namespaces, `Control` resolves to the `ROS2.Net.Control` namespace — the Avalonia
  apps use `Ros2NetGallery`/`Ros2NetSimulator` namespaces for that reason.
- Generated field names that clash with the class or generated members get a `Value` suffix
  (`Range.range` → `RangeValue`, `Clock.clock` → `ClockValue`).
- Fixed-size array fields must keep their exact length or `Serialize` throws `CdrException`.
- Interop with real ROS 2 (RTPS, rosbridge, bags, SROS2) is spec-based and not yet validated against an
  upstream install — say so rather than claiming compatibility.
- Docs are bilingual: every page exists in `docs/en` and `docs/id` (natural Bahasa Indonesia) and must
  carry the credit "Made by Gravicode Studios, led by Kang Fadhil" / "Dibuat oleh Gravicode Studios
  dipimpin oleh Kang Fadhil". The NuGet API key file lives outside the repo; never copy it in.
- MCP servers in `.mcp.json` (Blender, Rodin) are for 3D/2D assets (icon, robot models).
