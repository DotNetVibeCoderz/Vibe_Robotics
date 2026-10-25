# Native acceleration (`ros2net_native`)

`native/ros2net_native` is a Rust `cdylib` with a C ABI. `ROS2.Net.Native` wraps it with `LibraryImport` P/Invoke. Every kernel also has a managed twin, written to follow the Rust code step for step, so applications behave identically with or without the native library. The only differences are last-ulp ones in trigonometry (the laser projection).

## Kernels (ABI version 1)

| Function | Purpose |
|---|---|
| `ros2net_abi_version`, `ros2net_version` | ABI and crate version |
| `ros2net_voxel_downsample` | Voxel-grid centroids, ordered by voxel key |
| `ros2net_statistical_outlier` | Statistical outlier removal mask |
| `ros2net_radius_outlier` | Radius outlier removal mask |
| `ros2net_kdtree_build` / `_knn` / `_radius` / `_free` | Exact k-d tree; results ordered by (distance², index) |
| `ros2net_euclidean_cluster` | Euclidean clustering labels |
| `ros2net_ransac_plane` | RANSAC plane fit (xorshift64* RNG shared with managed code) |
| `ros2net_icp` | Point-to-point ICP (Horn's closed-form quaternion solution, equivalent to SVD/Kabsch) |
| `ros2net_scan_project` | LaserScan → xyz |
| `ros2net_occupancy_update` | Log-odds occupancy grid update by Bresenham ray tracing |

All functions:

- take flat `f32` xyz buffers;
- return status codes (`0` ok, `-1` null pointer, `-2` invalid argument, `-3` buffer too small, `-99` caught panic);
- never unwind across the FFI boundary, because every entry point is wrapped in `catch_unwind`.

## Using it from .NET

```csharp
using ROS2.Net.Native;

Console.WriteLine(NativeLibraryInfo.IsAvailable ? $"native {NativeLibraryInfo.Version}" : NativeLibraryInfo.LoadError);
NativeLibraryInfo.Mode = KernelMode.Auto;   // Auto | Managed | Native (also ROS2NET_NATIVE=0/1)

float[] centroids = PointKernels.VoxelDownsample(xyz, 0.05f);
using var tree = new KdTree(xyz);
var nearest = tree.Nearest(1, 2, 0, k: 8);
```

`ROS2.Net.Perception` uses `PointKernels` internally, so most applications never call this package directly.

## Building

```bash
cd native/ros2net_native
cargo test --release
cargo build --release        # → target/release/ros2net_native.dll | libros2net_native.so | .dylib
```

`ROS2.Net.Native.csproj` picks the library up from `target/release` automatically:

- it copies the library next to the build output of any project that references it;
- it packs the library into the NuGet package under `runtimes/<rid>/native/`.

If the library has not been built, the managed fallback is used and nothing breaks.

## Performance

`NativeEquivalenceTests` reports timings. On the development machine (Windows x64), voxel downsampling plus SOR (k = 8) over 100 000 points took about 7.8 s managed and 1.2 s native, roughly 6× faster.

---
Made by Gravicode Studios, led by Kang Fadhil.
