using System.Reflection;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;

namespace ROS2.Net.AppGen;

/// <summary>
/// Creates ROS2.Net projects: blank console/Avalonia/Blazor apps, or one of the embedded templates.
/// A template is a folder of embedded files (<c>Templates/&lt;id&gt;/…</c>) with a <c>template.json</c>
/// manifest; the project skeleton (csproj, Avalonia App/Program, Blazor host files) is generated
/// around it. Placeholders: <c>{{ProjectName}}</c>, <c>{{Namespace}}</c>, <c>{{Ros2NetVersion}}</c>.
/// </summary>
public sealed partial class ProjectService : IProjectService
{
    /// <summary>Avalonia version the generated desktop apps use (matches ROS2.Net.Avalonia).</summary>
    public const string AvaloniaVersion = "12.1.4";

    private static readonly JsonSerializerOptions s_json = new() { PropertyNameCaseInsensitive = true };
    private static readonly Lazy<IReadOnlyList<TemplateEntry>> s_templates = new(LoadTemplates);

    private readonly Func<AppGenSettings> _settings;

    public ProjectService(Func<AppGenSettings> settings) => _settings = settings;

    private sealed record TemplateManifest(string Name, string Category, string Description, string AppKind, string[]? Tags, string[]? Packages);

    private sealed record TemplateEntry(ProjectTemplate Template, string[] Packages, IReadOnlyDictionary<string, string> Files);

    public IReadOnlyList<ProjectTemplate> Templates => s_templates.Value.Select(t => t.Template).ToList();

    public IReadOnlyList<string> Categories => s_templates.Value.Select(t => t.Template.Category).Distinct().ToList();

    public string CreateBlank(string parentFolder, string projectName, string appKind = "console")
    {
        var kind = NormalizeKind(appKind);
        var files = kind switch
        {
            "avalonia" => new Dictionary<string, string> { ["MainWindow.cs"] = BlankAvaloniaWindow },
            "blazor" => new Dictionary<string, string> { ["Components/Pages/Home.razor"] = BlankBlazorHome },
            _ => new Dictionary<string, string> { ["Program.cs"] = BlankConsoleProgram },
        };
        return Write(parentFolder, projectName, Render(kind, ["ROS2.Net.Simulation"], files, projectName));
    }

    public string CreateFromTemplate(string templateId, string parentFolder, string projectName) =>
        Write(parentFolder, projectName, RenderTemplate(templateId, projectName));

    /// <summary>Renders a template's complete file set (relative path → content) without writing it.</summary>
    public IReadOnlyDictionary<string, string> RenderTemplate(string templateId, string projectName)
    {
        var entry = s_templates.Value.FirstOrDefault(t => t.Template.Id.Equals(templateId, StringComparison.OrdinalIgnoreCase))
            ?? throw new KeyNotFoundException($"Unknown template '{templateId}'.");
        return Render(entry.Template.AppKind, entry.Packages, entry.Files, projectName);
    }

    private string Write(string parentFolder, string projectName, IReadOnlyDictionary<string, string> files)
    {
        ValidateName(projectName);
        var folder = Path.Combine(Path.GetFullPath(parentFolder), projectName);
        if (Directory.Exists(folder) && Directory.EnumerateFileSystemEntries(folder).Any())
        {
            throw new IOException($"'{folder}' already exists and is not empty.");
        }

        foreach (var (rel, content) in files)
        {
            var path = Path.Combine(folder, rel.Replace('/', Path.DirectorySeparatorChar));
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            File.WriteAllText(path, content);
        }

        return folder;
    }

    private Dictionary<string, string> Render(string appKind, IEnumerable<string> templatePackages, IReadOnlyDictionary<string, string> templateFiles, string projectName)
    {
        ValidateName(projectName);
        var settings = _settings();
        var ns = ToNamespace(projectName);
        var version = string.IsNullOrWhiteSpace(settings.Ros2NetVersion) ? "0.1.0" : settings.Ros2NetVersion;
        var kind = NormalizeKind(appKind);
        string Subst(string s) => s.Replace("{{ProjectName}}", projectName, StringComparison.Ordinal)
            .Replace("{{Namespace}}", ns, StringComparison.Ordinal)
            .Replace("{{Ros2NetVersion}}", version, StringComparison.Ordinal);

        // AppTarget (simulator vs physical robot) needs the simulator and both network middlewares.
        var packages = new SortedSet<string>(StringComparer.Ordinal) { "ROS2.Net.Core", "ROS2.Net.Msgs", "ROS2.Net.Simulation", "ROS2.Net.Rmw.Rtps", "ROS2.Net.Rmw.Rosbridge" };
        foreach (var p in templatePackages)
        {
            packages.Add(p);
        }

        if (kind == "avalonia")
        {
            packages.Add("ROS2.Net.Avalonia");
        }
        else if (kind == "blazor")
        {
            packages.Add("ROS2.Net.Blazor");
            packages.Add("ROS2.Net.Hosting");
        }

        var files = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (var (rel, content) in templateFiles)
        {
            files[Subst(rel)] = Subst(content);
        }

        files[$"{projectName}.csproj"] = Csproj(kind, packages, version);
        files.TryAdd("AppTarget.cs", AppTargetSource);
        switch (kind)
        {
            case "avalonia":
                files.TryAdd("Program.cs", Subst(AvaloniaProgram));
                files.TryAdd("App.cs", Subst(AvaloniaApp));
                files.TryAdd("app.manifest", Subst(AppManifest));
                break;
            case "blazor":
                var hasRuntime = files.ContainsKey("Services/AppRuntime.cs");
                files.TryAdd("AppTarget.Hosting.cs", AppTargetBlazorSource);
                files.TryAdd("Program.cs", Subst(BlazorProgram(hasRuntime)));
                files.TryAdd("Components/App.razor", Subst(BlazorApp));
                files.TryAdd("Components/Routes.razor", BlazorRoutes);
                files.TryAdd("Components/_Imports.razor", Subst(BlazorImports));
                files.TryAdd("Components/Layout/MainLayout.razor", Subst(BlazorLayout));
                files.TryAdd("wwwroot/app.css", BlazorCss);
                files.TryAdd("Properties/launchSettings.json", LaunchSettings);
                break;
        }

        files.TryAdd(".gitignore", "bin/\nobj/\n*.user\n.vs/\n");
        files.TryAdd("README.md", Subst($"# {{{{ProjectName}}}}\n\nA ROS2.Net {kind} app generated by ROS2.Net App Generator (Jack - The Code Bender).\n\n```bash\ndotnet run\n```\n\nMade with ROS2.Net by Gravicode Studios, led by Kang Fadhil.\n"));

        var source = settings.PackageSource?.Trim();
        if (!string.IsNullOrEmpty(source) && Directory.Exists(source))
        {
            files["nuget.config"] = NuGetConfig(Path.GetFullPath(source));
        }

        return files;
    }

    private static string Csproj(string kind, IEnumerable<string> packages, string version)
    {
        var sb = new StringBuilder();
        sb.AppendLine(kind == "blazor" ? "<Project Sdk=\"Microsoft.NET.Sdk.Web\">" : "<Project Sdk=\"Microsoft.NET.Sdk\">");
        sb.AppendLine();
        sb.AppendLine("  <PropertyGroup>");
        if (kind != "blazor")
        {
            sb.AppendLine(kind == "avalonia" ? "    <OutputType>WinExe</OutputType>" : "    <OutputType>Exe</OutputType>");
        }

        sb.AppendLine("    <TargetFramework>net10.0</TargetFramework>");
        sb.AppendLine("    <Nullable>enable</Nullable>");
        sb.AppendLine("    <ImplicitUsings>enable</ImplicitUsings>");
        if (kind == "avalonia")
        {
            sb.AppendLine("    <BuiltInComInteropSupport>true</BuiltInComInteropSupport>");
            sb.AppendLine("    <ApplicationManifest>app.manifest</ApplicationManifest>");
        }

        sb.AppendLine("  </PropertyGroup>");
        sb.AppendLine();
        sb.AppendLine("  <ItemGroup>");
        if (kind == "avalonia")
        {
            foreach (var a in new[] { "Avalonia", "Avalonia.Desktop", "Avalonia.Themes.Fluent", "Avalonia.Fonts.Inter" })
            {
                sb.AppendLine($"    <PackageReference Include=\"{a}\" Version=\"{AvaloniaVersion}\" />");
            }
        }

        foreach (var p in packages)
        {
            sb.AppendLine($"    <PackageReference Include=\"{p}\" Version=\"{version}\" />");
        }

        sb.AppendLine("  </ItemGroup>");
        sb.AppendLine();
        sb.AppendLine("</Project>");
        return sb.ToString();
    }

    private static string NuGetConfig(string folder) => $"""
        <?xml version="1.0" encoding="utf-8"?>
        <configuration>
          <packageSources>
            <!-- Local ROS2.Net packages (produced by: dotnet pack ROS2.Net.slnx -c Release -o artifacts/packages). -->
            <add key="ros2net-local" value="{System.Security.SecurityElement.Escape(folder)}" />
            <add key="nuget.org" value="https://api.nuget.org/v3/index.json" />
          </packageSources>
        </configuration>
        """;

    private static string NormalizeKind(string kind) => kind.Trim().ToLowerInvariant() switch
    {
        "avalonia" or "desktop" => "avalonia",
        "blazor" or "web" => "blazor",
        _ => "console",
    };

    /// <summary>Project names become folder and assembly names: letters, digits, '.', '_' and '-'.</summary>
    public static void ValidateName(string name)
    {
        if (string.IsNullOrWhiteSpace(name) || !ProjectNameRegex().IsMatch(name))
        {
            throw new ArgumentException($"'{name}' is not a valid project name (use letters, digits, '.', '_' or '-', starting with a letter).", nameof(name));
        }
    }

    /// <summary>A C# namespace from a project name: <c>my-robot.app</c> → <c>My_Robot.App</c>.</summary>
    public static string ToNamespace(string name)
    {
        var parts = name.Split('.', StringSplitOptions.RemoveEmptyEntries)
            .Select(p => Regex.Replace(p, "[^A-Za-z0-9_]", "_"))
            .Select(p => char.IsDigit(p[0]) ? "_" + p : char.ToUpperInvariant(p[0]) + p[1..])
            .Select(p => p.StartsWith("ROS2", StringComparison.OrdinalIgnoreCase) ? "App" + p : p);
        return string.Join('.', parts);
    }

    [GeneratedRegex("^[A-Za-z][A-Za-z0-9._-]{0,63}$")]
    private static partial Regex ProjectNameRegex();

    private static IReadOnlyList<TemplateEntry> LoadTemplates()
    {
        var asm = typeof(ProjectService).Assembly;
        const string prefix = "templates/";
        var byId = new Dictionary<string, Dictionary<string, string>>(StringComparer.OrdinalIgnoreCase);
        foreach (var name in asm.GetManifestResourceNames().Where(n => n.StartsWith(prefix, StringComparison.Ordinal)))
        {
            var rest = name[prefix.Length..].Replace((char)92, '/'); // %(RecursiveDir) uses backslashes on Windows
            var slash = rest.IndexOf('/');
            if (slash <= 0)
            {
                continue;
            }

            var id = rest[..slash];
            var rel = rest[(slash + 1)..];
            using var stream = asm.GetManifestResourceStream(name)!;
            using var reader = new StreamReader(stream);
            if (!byId.TryGetValue(id, out var files))
            {
                byId[id] = files = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            }

            files[rel] = reader.ReadToEnd();
        }

        var list = new List<TemplateEntry>();
        foreach (var (id, files) in byId)
        {
            if (!files.Remove("template.json", out var manifestJson))
            {
                continue;
            }

            var m = JsonSerializer.Deserialize<TemplateManifest>(manifestJson, s_json)!;
            list.Add(new TemplateEntry(new ProjectTemplate(id, m.Name, m.Category, m.Description, NormalizeKind(m.AppKind), m.Tags ?? []), m.Packages ?? [], files));
        }

        return list.OrderBy(t => t.Template.Category, StringComparer.Ordinal).ThenBy(t => t.Template.Name, StringComparer.Ordinal).ToList();
    }
}
