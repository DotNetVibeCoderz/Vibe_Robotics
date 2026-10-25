using System.ComponentModel;
using System.Text;
using Microsoft.SemanticKernel;

namespace ROS2.Net.AppGen.Plugins;

/// <summary>Build, run and package management for the open project, plus project creation from templates.</summary>
public sealed class BuildPlugin(ToolContext ctx)
{
    [KernelFunction("build_project")]
    [Description("Runs 'dotnet build' on the open project and returns success/failure with the compiler errors and warnings (file, line, code, message). Always build after writing code and fix every error.")]
    public async Task<string> BuildProjectAsync(CancellationToken cancellationToken = default)
    {
        var root = ctx.RequireRoot();
        var result = await ctx.Build.BuildAsync(root, line => ctx.Log(line.Line), cancellationToken).ConfigureAwait(false);
        var diagnostics = BuildService.ParseDiagnostics(result.Output);
        var errors = diagnostics.Where(d => d.Severity == "error").ToList();
        var warnings = diagnostics.Where(d => d.Severity == "warning").ToList();
        var sb = new StringBuilder();
        sb.AppendLine(result.Success ? $"Build succeeded in {result.Duration.TotalSeconds:0.0}s." : $"Build FAILED (exit {result.ExitCode}) in {result.Duration.TotalSeconds:0.0}s.");
        sb.AppendLine($"{errors.Count} error(s), {warnings.Count} warning(s).");
        foreach (var d in errors.Take(40))
        {
            sb.AppendLine(Relative(root, d));
        }

        foreach (var d in warnings.Take(10))
        {
            sb.AppendLine(Relative(root, d));
        }

        if (!result.Success && errors.Count == 0)
        {
            // Restore or MSBuild failures without file diagnostics: show the tail of the output.
            sb.AppendLine("Output tail:");
            sb.AppendLine(Tail(result.Output, 40));
        }

        return ctx.Truncate(sb.ToString());
    }

    [KernelFunction("run_project_briefly")]
    [Description("Runs the open console project for a few seconds and returns its console output (it is stopped after the time budget). Use it to check behaviour of console apps; GUI apps only show startup output.")]
    public async Task<string> RunProjectBrieflyAsync([Description("Seconds to let it run (1-60)")] int seconds = 8, [Description("Command-line arguments for the app")] string arguments = "", CancellationToken cancellationToken = default)
    {
        var root = ctx.RequireRoot();
        var args = string.IsNullOrWhiteSpace(arguments) ? null : arguments.Split(' ', StringSplitOptions.RemoveEmptyEntries);
        var result = await ctx.Build.RunBrieflyAsync(root, TimeSpan.FromSeconds(Math.Clamp(seconds, 1, 60)), line => ctx.Log(line.Line), cancellationToken, args).ConfigureAwait(false);
        return ctx.Truncate($"exit {result.ExitCode} after {result.Duration.TotalSeconds:0.0}s\n{Tail(result.Output, 120)}");
    }

    [KernelFunction("get_run_targets")]
    [Description("Lists the user's run targets (simulator and physical robots) with the environment variables each passes to the app, and which one Run uses. Generated apps read them through AppTarget.cs.")]
    public string GetRunTargets()
    {
        var settings = ctx.Settings();
        var sb = new StringBuilder();
        sb.AppendLine($"Active: {(string.IsNullOrEmpty(settings.ActiveRunTarget) ? RunTargetEnvironment.DefaultSimulator.Name : settings.ActiveRunTarget)}");
        foreach (var t in settings.RunTargets.Count > 0 ? settings.RunTargets : RunTargetEnvironment.Defaults())
        {
            var env = RunTargetEnvironment.ToEnvironment(t);
            sb.AppendLine($"- {t.Name} [{t.Kind}]: {string.Join(" ", env.Select(kv => $"{kv.Key}={kv.Value}"))} {string.Join(' ', RunTargetEnvironment.ToArguments(t))}".TrimEnd());
        }

        sb.AppendLine("run_project_briefly always uses the in-process simulator target.");
        return sb.ToString();
    }

    [KernelFunction("add_package")]
    [Description("Adds a NuGet package reference to the open project with 'dotnet add package'.")]
    public async Task<string> AddPackageAsync(string packageId, [Description("Version, or empty for the latest")] string version = "", CancellationToken cancellationToken = default)
    {
        var root = ctx.RequireRoot();
        if (packageId.StartsWith("ROS2.Net", StringComparison.OrdinalIgnoreCase) && string.IsNullOrWhiteSpace(version))
        {
            version = ctx.Settings().Ros2NetVersion;
        }

        var result = await ctx.Build.AddPackageAsync(root, packageId, version, line => ctx.Log(line.Line), cancellationToken).ConfigureAwait(false);
        return result.Success ? $"Added {packageId} {version}".Trim() + "." : "Failed to add package:\n" + Tail(result.Output, 30);
    }

    [KernelFunction("list_templates")]
    [Description("Lists the available ROS2.Net project templates (id, name, category, app kind, description). Optionally filter by category or keyword.")]
    public string ListTemplates([Description("Category or keyword, empty for all")] string filter = "")
    {
        var templates = ctx.Projects.Templates.Where(t => string.IsNullOrWhiteSpace(filter)
            || t.Category.Contains(filter, StringComparison.OrdinalIgnoreCase)
            || t.Name.Contains(filter, StringComparison.OrdinalIgnoreCase)
            || t.Description.Contains(filter, StringComparison.OrdinalIgnoreCase)
            || t.Tags.Any(x => x.Contains(filter, StringComparison.OrdinalIgnoreCase)));
        return string.Join('\n', templates.Select(t => $"{t.Id} | {t.Name} | {t.Category} | {t.AppKind} | {t.Description}"));
    }

    [KernelFunction("get_template_source")]
    [Description("Returns the source files of a template (useful as a known-good starting point for similar code).")]
    public string GetTemplateSource(string templateId)
    {
        try
        {
            var files = ctx.Projects.RenderTemplate(templateId, "Sample");
            var sb = new StringBuilder();
            foreach (var (path, content) in files.Where(f => !f.Key.EndsWith(".csproj", StringComparison.OrdinalIgnoreCase) || files.Count < 4))
            {
                sb.AppendLine($"=== {path} ===").AppendLine(content);
            }

            return ctx.Truncate(sb.ToString());
        }
        catch (KeyNotFoundException)
        {
            return $"Unknown template '{templateId}'. Use list_templates.";
        }
    }

    [KernelFunction("create_project")]
    [Description("Creates a new blank ROS2.Net project (appKind: console, avalonia or blazor) in the projects folder and opens it. Only use when no project is open or the user asks for a new one.")]
    public string CreateProject(string projectName, [Description("console, avalonia or blazor")] string appKind = "console")
    {
        var folder = ctx.Projects.CreateBlank(ProjectsFolder(), projectName, appKind);
        ctx.Workspace.Open(folder);
        return $"Created and opened {folder}.";
    }

    [KernelFunction("create_project_from_template")]
    [Description("Creates a new project from a template in the projects folder and opens it.")]
    public string CreateProjectFromTemplate(string templateId, string projectName)
    {
        var folder = ctx.Projects.CreateFromTemplate(templateId, ProjectsFolder(), projectName);
        ctx.Workspace.Open(folder);
        return $"Created {projectName} from template {templateId} and opened {folder}.";
    }

    private string ProjectsFolder()
    {
        var f = ctx.Settings().ProjectsFolder;
        return string.IsNullOrWhiteSpace(f) ? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "ROS2AppGen") : f;
    }

    private static string Relative(string root, BuildDiagnostic d)
    {
        var file = d.File;
        try
        {
            if (Path.IsPathRooted(file))
            {
                file = Path.GetRelativePath(root, file).Replace('\\', '/');
            }
        }
        catch (ArgumentException)
        {
        }

        return $"{file}({d.Line},{d.Column}): {d.Severity} {d.Code}: {d.Message}";
    }

    internal static string Tail(string text, int lines)
    {
        var all = text.Replace("\r\n", "\n").TrimEnd().Split('\n');
        return string.Join('\n', all.Skip(Math.Max(0, all.Length - lines)));
    }
}
