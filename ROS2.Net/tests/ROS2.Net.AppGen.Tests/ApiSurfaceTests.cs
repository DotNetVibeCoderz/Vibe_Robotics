using System.Reflection;
using System.Runtime.Loader;
using System.Text;
using Shouldly;

namespace ROS2.Net.AppGen.Tests;

/// <summary>
/// Jack's <c>ros2net.get_type_api</c> reads reflected API dumps embedded from
/// <c>src/ROS2.Net.AppGen.Core/Knowledge/api/*.txt</c>. Refresh them after changing a library's public API:
/// <c>dotnet build ROS2.Net.slnx -c Release</c>, then run this test with <c>ROS2NET_APPGEN_REFRESH_API=1</c>.
/// </summary>
public sealed class ApiSurfaceTests
{
    private static readonly string[] s_assemblies =
    [
        "ROS2.Net.Core", "ROS2.Net.Simulation", "ROS2.Net.Navigation", "ROS2.Net.AI", "ROS2.Net.Control", "ROS2.Net.Manipulation",
        "ROS2.Net.Tf2", "ROS2.Net.Bag", "ROS2.Net.Perception", "ROS2.Net.Hosting", "ROS2.Net.Avalonia", "ROS2.Net.Blazor",
    ];

    [Fact]
    public void Embedded_api_surface_covers_every_library()
    {
        var names = typeof(AppGenHost).Assembly.GetManifestResourceNames().Where(n => n.StartsWith("api/", StringComparison.Ordinal)).ToList();
        foreach (var a in s_assemblies)
        {
            names.ShouldContain($"api/{a}.txt");
        }
    }

    [Fact]
    public void Refresh_api_surface()
    {
        Assert.SkipUnless(Environment.GetEnvironmentVariable("ROS2NET_APPGEN_REFRESH_API") == "1", "Set ROS2NET_APPGEN_REFRESH_API=1 after a Release build to regenerate the API dumps.");
        var root = TestHost.RepoRoot();
        var outDir = Path.Combine(root, "src", "ROS2.Net.AppGen.Core", "Knowledge", "api");
        var dirs = Directory.GetDirectories(Path.Combine(root, "src")).Select(d => Path.Combine(d, "bin", "Release", "net10.0")).Where(Directory.Exists).ToList();
        var nuget = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), ".nuget", "packages");
        var index = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (var f in Directory.EnumerateFiles(nuget, "*.dll", SearchOption.AllDirectories).Where(f => f.Replace('\\', '/').Contains("/lib/net", StringComparison.Ordinal)))
        {
            index.TryAdd(Path.GetFileNameWithoutExtension(f), f);
        }

        var alc = new AssemblyLoadContext("api-dump", isCollectible: true);
        alc.Resolving += (ctx, name) =>
        {
            foreach (var d in dirs)
            {
                var p = Path.Combine(d, name.Name + ".dll");
                if (File.Exists(p))
                {
                    return ctx.LoadFromAssemblyPath(p);
                }
            }

            return index.TryGetValue(name.Name!, out var hit) ? ctx.LoadFromAssemblyPath(hit) : null;
        };

        foreach (var name in s_assemblies)
        {
            var dll = dirs.Select(d => Path.Combine(d, name + ".dll")).First(File.Exists);
            File.WriteAllText(Path.Combine(outDir, name + ".txt"), Dump(alc.LoadFromAssemblyPath(dll)));
        }

        alc.Unload();
    }

    private static string Dump(Assembly asm)
    {
        static string N(Type? t) => t is null ? "void" : t.IsGenericType ? t.Name.Split('`')[0] + "<" + string.Join(",", t.GetGenericArguments().Select(N)) + ">" : t.IsArray ? N(t.GetElementType()) + "[]" : t.Name;
        static string P(ParameterInfo p) => N(p.ParameterType) + " " + p.Name + (p.HasDefaultValue ? "=" + (p.DefaultValue ?? "null") : string.Empty);
        const BindingFlags Flags = BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly;
        Type[] types;
        try
        {
            types = asm.GetExportedTypes();
        }
        catch (ReflectionTypeLoadException e)
        {
            types = e.Types.Where(t => t is not null).ToArray()!;
        }

        var sb = new StringBuilder();
        foreach (var t in types.Where(t => t.Namespace?.StartsWith("ROS2.Net.Msgs", StringComparison.Ordinal) != true).OrderBy(t => t.FullName, StringComparer.Ordinal))
        {
            try
            {
                var kind = t.IsEnum ? "enum" : t.IsInterface ? "interface" : t.IsValueType ? "struct" : "class";
                sb.AppendLine($"{kind} {t.FullName}" + (t.BaseType is { } b && b != typeof(object) && !t.IsValueType ? " : " + N(b) : string.Empty));
                if (t.IsEnum)
                {
                    sb.AppendLine("  " + string.Join(", ", Enum.GetNames(t)));
                    continue;
                }

                foreach (var c in t.GetConstructors())
                {
                    sb.AppendLine($"  ctor({string.Join(", ", c.GetParameters().Select(P))})");
                }

                foreach (var p in t.GetProperties(Flags))
                {
                    sb.AppendLine($"  {(p.GetMethod?.IsStatic == true ? "static " : string.Empty)}{N(p.PropertyType)} {p.Name} {{{(p.CanRead ? " get;" : string.Empty)}{(p.SetMethod?.IsPublic == true ? " set;" : string.Empty)} }}");
                }

                foreach (var f in t.GetFields(Flags))
                {
                    sb.AppendLine($"  field {(f.IsStatic ? "static " : string.Empty)}{N(f.FieldType)} {f.Name}");
                }

                foreach (var m in t.GetMethods(Flags).Where(m => !m.IsSpecialName))
                {
                    sb.AppendLine($"  {(m.IsStatic ? "static " : string.Empty)}{N(m.ReturnType)} {m.Name}{(m.IsGenericMethod ? "<" + string.Join(",", m.GetGenericArguments().Select(a => a.Name)) + ">" : string.Empty)}({string.Join(", ", m.GetParameters().Select(P))})");
                }

                foreach (var e in t.GetEvents(BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly))
                {
                    sb.AppendLine($"  event {N(e.EventHandlerType)} {e.Name}");
                }
            }
            catch (Exception ex) when (ex is TypeLoadException or FileNotFoundException or NotSupportedException)
            {
                sb.AppendLine("  !" + ex.Message.Split('\n')[0]);
            }
        }

        return sb.ToString();
    }
}
