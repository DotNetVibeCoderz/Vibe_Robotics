using System.Runtime.InteropServices;

namespace ROS2.Net.Native;

/// <summary>P/Invoke surface of <c>ros2net_native</c> (ABI version 1). See native/ros2net_native/src/lib.rs.</summary>
internal static unsafe partial class NativeMethods
{
    public const string Library = "ros2net_native";

    public const int StatusOk = 0;
    public const int StatusNull = -1;
    public const int StatusInvalid = -2;
    public const int StatusBufferTooSmall = -3;
    public const int StatusPanic = -99;

    [LibraryImport(Library, EntryPoint = "ros2net_abi_version")]
    public static partial uint AbiVersion();

    [LibraryImport(Library, EntryPoint = "ros2net_version")]
    public static partial byte* Version();

    [LibraryImport(Library, EntryPoint = "ros2net_voxel_downsample")]
    public static partial int VoxelDownsample(float* xyz, nuint n, float leaf, float* output, nuint outCap, out nuint outN);

    [LibraryImport(Library, EntryPoint = "ros2net_statistical_outlier")]
    public static partial int StatisticalOutlier(float* xyz, nuint n, nuint k, double stdMul, byte* keep);

    [LibraryImport(Library, EntryPoint = "ros2net_radius_outlier")]
    public static partial int RadiusOutlier(float* xyz, nuint n, float radius, nuint minNeighbors, byte* keep);

    [LibraryImport(Library, EntryPoint = "ros2net_kdtree_build")]
    public static partial nint KdTreeBuild(float* xyz, nuint n);

    [LibraryImport(Library, EntryPoint = "ros2net_kdtree_free")]
    public static partial void KdTreeFree(nint tree);

    [LibraryImport(Library, EntryPoint = "ros2net_kdtree_knn")]
    public static partial int KdTreeKnn(nint tree, float* query, nuint k, uint* outIdx, float* outD2, out nuint found);

    [LibraryImport(Library, EntryPoint = "ros2net_kdtree_radius")]
    public static partial int KdTreeRadius(nint tree, float* query, float radius, uint* outIdx, float* outD2, nuint cap, out nuint outN);

    [LibraryImport(Library, EntryPoint = "ros2net_euclidean_cluster")]
    public static partial int EuclideanCluster(float* xyz, nuint n, float tolerance, nuint minSize, nuint maxSize, int* labels, out nuint clusters);

    [LibraryImport(Library, EntryPoint = "ros2net_ransac_plane")]
    public static partial int RansacPlane(float* xyz, nuint n, double threshold, nuint iterations, ulong seed, double* coeffs, byte* inliers, out nuint count);

    [LibraryImport(Library, EntryPoint = "ros2net_icp")]
    public static partial int Icp(float* src, nuint ns, float* dst, nuint nd, uint maxIter, float maxCorr, double tolerance, double* transform, out double rmse, out uint iterations, out byte converged);

    [LibraryImport(Library, EntryPoint = "ros2net_scan_project")]
    public static partial int ScanProject(float* ranges, nuint n, float angleMin, float angleInc, float rangeMin, float rangeMax, float* output, out nuint outN);

    [LibraryImport(Library, EntryPoint = "ros2net_occupancy_update")]
    public static partial int OccupancyUpdate(float* grid, int width, int height, double resolution, double originX, double originY, double sensorX, double sensorY, float* pointsXy, nuint n, float lFree, float lOcc, float lMin, float lMax);

    public static void Check(int status, string function)
    {
        if (status != StatusOk)
        {
            throw new NativeKernelException(function, status);
        }
    }
}

/// <summary>Raised when a native kernel reports an error status.</summary>
public sealed class NativeKernelException(string function, int status)
    : InvalidOperationException($"{function} failed with status {status} ({Describe(status)}).")
{
    public int Status { get; } = status;

    private static string Describe(int status) => status switch
    {
        NativeMethods.StatusNull => "null pointer",
        NativeMethods.StatusInvalid => "invalid argument",
        NativeMethods.StatusBufferTooSmall => "buffer too small",
        NativeMethods.StatusPanic => "panic inside native code",
        _ => "unknown",
    };
}
