# Vibe_Robotics
Collection of Vibed Robotics Solution

| Project | Description | Packages |
|---|---|---|
| [ROS2.Net](ROS2.Net/README.md) | Pure .NET 10 implementation of ROS 2 — nodes, topics, services, actions, parameters, lifecycle, DDS-RTPS & rosbridge middlewares, TF2, rosbag2, navigation/SLAM, control, manipulation, perception (Rust-accelerated), simulation and AI, with Blazor & Avalonia components, a gallery, a robot simulator, the `ros2net` CLI and `dotnet new` templates. No ROS 2 installation required. | `ROS2.Net`, `ROS2.Net.*` |

Each project lives in its own folder with its own solution; CI and NuGet publishing run from
`.github/workflows/<project>.yml` and are triggered by a `<project>-v<version>` tag (for example
`ros2net-v0.1.0`) using the `NUGET_API_KEY` repository secret.

Made by Gravicode Studios, led by Kang Fadhil · Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil
