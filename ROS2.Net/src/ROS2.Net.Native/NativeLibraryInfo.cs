using System.Reflection;
using System.Runtime.InteropServices;

namespace ROS2.Net.Native;

/// <summary>Which implementation the kernels use.</summary>
public enum KernelMode
{
    /// <summary>Native when the library loads and its ABI matches, otherwise managed.</summary>
    Auto,
    /// <summary>Always managed (portable, no native dependency).</summary>
    Managed,
    /// <summary>Always native; throws <see cref="DllNotFoundException"/> when unavailable.</summary>
    Native,
}

/// <summary>Discovers and describes the <c>ros2net_native</c> library.</summary>
public static class NativeLibraryInfo
{
    /// <summary>The ABI version this assembly was built against.</summary>
    public const uint ExpectedAbiVersion = 1;

    private static readonly Lazy<(bool Ok, uint Abi, string Version, string? Error)> s_probe = new(Probe);

    static NativeLibraryInfo()
    {
        try
        {
            NativeLibrary.SetDllImportResolver(typeof(NativeLibraryInfo).Assembly, Resolve);
        }
        catch (InvalidOperationException)
        {
            // A resolver is already registered.
        }
    }

    /// <summary>Kernel selection; can also be forced with <c>ROS2NET_NATIVE=0|1</c>.</summary>
    public static KernelMode Mode { get; set; } = Environment.GetEnvironmentVariable("ROS2NET_NATIVE") switch
    {
        "0" or "false" => KernelMode.Managed,
        "1" or "true" => KernelMode.Native,
        _ => KernelMode.Auto,
    };

    /// <summary>True when the native library loaded and its ABI matches.</summary>
    public static bool IsAvailable => s_probe.Value.Ok;

    public static uint AbiVersion => s_probe.Value.Abi;

    /// <summary>Native library version, or null when unavailable.</summary>
    public static string? Version => s_probe.Value.Ok ? s_probe.Value.Version : null;

    /// <summary>Why the native library is unavailable, if it is.</summary>
    public static string? LoadError => s_probe.Value.Error;

    /// <summary>Whether calls right now go to native code.</summary>
    public static bool UseNative => Mode switch
    {
        KernelMode.Managed => false,
        KernelMode.Native => IsAvailable ? true : throw new DllNotFoundException($"ros2net_native is not available: {LoadError}"),
        _ => IsAvailable,
    };

    private static unsafe (bool, uint, string, string?) Probe()
    {
        try
        {
            var abi = NativeMethods.AbiVersion();
            var version = Marshal.PtrToStringUTF8((nint)NativeMethods.Version()) ?? "?";
            return abi == ExpectedAbiVersion
                ? (true, abi, version, null)
                : (false, abi, version, $"ABI {abi} does not match expected {ExpectedAbiVersion}");
        }
        catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
        {
            return (false, 0, string.Empty, ex.Message);
        }
    }

    private static nint Resolve(string name, Assembly assembly, DllImportSearchPath? path)
    {
        if (name != NativeMethods.Library)
        {
            return 0;
        }

        if (NativeLibrary.TryLoad(name, assembly, path, out var handle))
        {
            return handle;
        }

        var file = OperatingSystem.IsWindows() ? "ros2net_native.dll" : OperatingSystem.IsMacOS() ? "libros2net_native.dylib" : "libros2net_native.so";
        var baseDir = AppContext.BaseDirectory;
        var candidates = new[]
        {
            Path.Combine(baseDir, file),
            Path.Combine(baseDir, "runtimes", RuntimeInformation.RuntimeIdentifier, "native", file),
            Path.Combine(Path.GetDirectoryName(assembly.Location) ?? baseDir, file),
        };
        foreach (var c in candidates)
        {
            if (File.Exists(c) && NativeLibrary.TryLoad(c, out handle))
            {
                return handle;
            }
        }

        return 0;
    }
}
