# Native code

| Crate | Purpose |
|---|---|
| [`ros2net_native`](ros2net_native) | Rust kernels for point clouds, k-d trees, ICP, laser projection and occupancy ray tracing, exposed through a C ABI and consumed by `src/ROS2.Net.Native` |

```bash
cd ros2net_native
cargo test --release
cargo build --release
```

The build output (`target/release/ros2net_native.{dll,so,dylib}`) is copied to .NET build outputs and packed into the `ROS2.Net.Native` NuGet package automatically. The library is optional: without it, ROS2.Net uses the managed implementations. See [docs/en/native.md](../docs/en/native.md) for the full ABI.

Made by Gravicode Studios, led by Kang Fadhil.
