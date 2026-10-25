# Akselerasi native (`ros2net_native`)

`native/ros2net_native` adalah `cdylib` Rust dengan ABI C. `ROS2.Net.Native` membungkusnya dengan P/Invoke `LibraryImport`. Setiap kernel juga punya kembaran managed yang ditulis langkah demi langkah mengikuti kode Rust. Jadi aplikasi berperilaku sama, dengan atau tanpa library native. Perbedaannya hanya di digit terakhir hasil trigonometri (proyeksi laser).

## Kernel (ABI versi 1)

| Fungsi | Kegunaan |
|---|---|
| `ros2net_abi_version`, `ros2net_version` | Versi ABI dan crate |
| `ros2net_voxel_downsample` | Centroid voxel grid, diurutkan menurut kunci voxel |
| `ros2net_statistical_outlier` | Mask penghapusan outlier statistik |
| `ros2net_radius_outlier` | Mask penghapusan outlier berbasis radius |
| `ros2net_kdtree_build` / `_knn` / `_radius` / `_free` | k-d tree eksak; hasil diurutkan menurut (jarak², indeks) |
| `ros2net_euclidean_cluster` | Label klaster Euclidean |
| `ros2net_ransac_plane` | Fitting bidang RANSAC (RNG xorshift64* sama dengan kode managed) |
| `ros2net_icp` | ICP titik-ke-titik (solusi kuaternion bentuk tertutup Horn, setara SVD/Kabsch) |
| `ros2net_scan_project` | LaserScan → xyz |
| `ros2net_occupancy_update` | Pembaruan grid okupansi log-odds dengan ray tracing Bresenham |

Semua fungsi:

- menerima buffer xyz `f32` datar;
- mengembalikan kode status (`0` ok, `-1` pointer null, `-2` argumen tidak valid, `-3` buffer terlalu kecil, `-99` panic yang tertangkap);
- tidak pernah melempar unwind melewati batas FFI, karena setiap entry point dibungkus `catch_unwind`.

## Memakainya dari .NET

```csharp
using ROS2.Net.Native;

Console.WriteLine(NativeLibraryInfo.IsAvailable ? $"native {NativeLibraryInfo.Version}" : NativeLibraryInfo.LoadError);
NativeLibraryInfo.Mode = KernelMode.Auto;   // Auto | Managed | Native (atau env ROS2NET_NATIVE=0/1)

float[] centroids = PointKernels.VoxelDownsample(xyz, 0.05f);
using var tree = new KdTree(xyz);
var nearest = tree.Nearest(1, 2, 0, k: 8);
```

`ROS2.Net.Perception` memakai `PointKernels` di dalamnya, jadi kebanyakan aplikasi tidak perlu memanggil paket ini secara langsung.

## Build

```bash
cd native/ros2net_native
cargo test --release
cargo build --release        # → target/release/ros2net_native.dll | libros2net_native.so | .dylib
```

`ROS2.Net.Native.csproj` otomatis mengambil library dari `target/release`:

- menyalinnya ke folder output setiap proyek yang mereferensikannya;
- memaketkannya ke paket NuGet di `runtimes/<rid>/native/`.

Kalau library belum di-build, cadangan managed yang dipakai dan tidak ada yang rusak.

## Performa

`NativeEquivalenceTests` melaporkan waktu eksekusi. Di mesin pengembangan (Windows x64), voxel downsampling ditambah SOR (k = 8) untuk 100.000 titik butuh sekitar 7,8 detik dengan managed dan 1,2 detik dengan native, kira-kira 6× lebih cepat.

---
Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil.
