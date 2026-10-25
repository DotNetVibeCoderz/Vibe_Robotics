using System.ComponentModel;
using System.Text;
using System.Text.RegularExpressions;
using System.Xml.Linq;
using Microsoft.SemanticKernel;

namespace ROS2.Net.AppGen.Plugins;

/// <summary>File operations on the open project. Every path is relative to the project root and cannot escape it.</summary>
public sealed class ProjectPlugin(ToolContext ctx)
{
    [KernelFunction("list_files")]
    [Description("Lists the project's files (relative paths), skipping bin/obj. Optionally only under a sub-folder.")]
    public string ListFiles([Description("Sub-folder, or empty for the whole project")] string folder = "")
    {
        ctx.RequireRoot();
        var files = ctx.Workspace.ListFiles(string.IsNullOrWhiteSpace(folder) ? null : folder);
        if (files.Count == 0)
        {
            return "(no files)";
        }

        var shown = files.Take(500).ToList();
        return string.Join('\n', shown) + (files.Count > shown.Count ? $"\n… and {files.Count - shown.Count} more" : string.Empty);
    }

    [KernelFunction("read_file")]
    [Description("Reads a text file from the project. Optionally a 1-based inclusive line range. Lines are prefixed with their numbers when a range is given.")]
    public string ReadFile(string path, [Description("First line (1-based), 0 = whole file")] int startLine = 0, [Description("Last line, 0 = to the end")] int endLine = 0)
    {
        ctx.RequireRoot();
        var text = ctx.Workspace.ReadFile(path);
        if (startLine <= 0 && endLine <= 0)
        {
            return ctx.Truncate(text);
        }

        var lines = text.Replace("\r\n", "\n").Split('\n');
        var from = Math.Max(1, startLine);
        var to = endLine <= 0 ? lines.Length : Math.Min(lines.Length, endLine);
        var sb = new StringBuilder();
        for (var i = from; i <= to; i++)
        {
            sb.Append(i).Append(": ").AppendLine(lines[i - 1]);
        }

        return ctx.Truncate(sb.ToString());
    }

    [KernelFunction("write_file")]
    [Description("Creates or overwrites a file with the COMPLETE new content (creates folders as needed). Use replace_in_file for small edits to large files.")]
    public string WriteFile(string path, [Description("Full file content")] string content)
    {
        ctx.RequireRoot();
        string? old = null;
        try
        {
            old = ctx.Workspace.ReadFile(path);
        }
        catch (FileNotFoundException)
        {
        }

        ctx.Workspace.WriteFile(path, content);
        var newLines = CountLines(content);
        if (old is null)
        {
            ctx.Log($"Jack created {path}");
            return $"Created {path} ({newLines} lines).";
        }

        var (added, removed) = LineDiff(old, content);
        ctx.Log($"Jack updated {path} (+{added} -{removed})");
        return $"Updated {path} ({newLines} lines, +{added} -{removed}).";
    }

    [KernelFunction("replace_in_file")]
    [Description("Replaces an exact text snippet in a file. Fails if the snippet is not found, or is found more than once while replaceAll is false. Include enough context to be unique.")]
    public string ReplaceInFile(string path, [Description("Exact existing text")] string oldText, [Description("Replacement text")] string newText, bool replaceAll = false)
    {
        ctx.RequireRoot();
        var text = ctx.Workspace.ReadFile(path);
        if (string.IsNullOrEmpty(oldText))
        {
            return "Error: oldText must not be empty.";
        }

        // Tolerate CRLF/LF differences between the model's snippet and the file.
        var normalized = text.Replace("\r\n", "\n");
        var needle = oldText.Replace("\r\n", "\n");
        var count = CountOccurrences(normalized, needle);
        if (count == 0)
        {
            return $"Error: the text was not found in {path}. Read the file again and copy the snippet exactly.";
        }

        if (count > 1 && !replaceAll)
        {
            return $"Error: the text occurs {count} times in {path}. Add more surrounding context, or set replaceAll=true.";
        }

        var result = replaceAll ? normalized.Replace(needle, newText.Replace("\r\n", "\n"), StringComparison.Ordinal) : ReplaceFirst(normalized, needle, newText.Replace("\r\n", "\n"));
        if (text.Contains("\r\n", StringComparison.Ordinal))
        {
            result = result.Replace("\n", "\r\n", StringComparison.Ordinal);
        }

        ctx.Workspace.WriteFile(path, result);
        ctx.Log($"Jack edited {path} ({(replaceAll ? count : 1)} replacement(s))");
        return $"Replaced {(replaceAll ? count : 1)} occurrence(s) in {path}.";
    }

    [KernelFunction("append_to_file")]
    [Description("Appends text to the end of a file (creates it if missing).")]
    public string AppendToFile(string path, string text)
    {
        ctx.RequireRoot();
        string existing;
        try
        {
            existing = ctx.Workspace.ReadFile(path);
        }
        catch (FileNotFoundException)
        {
            existing = string.Empty;
        }

        ctx.Workspace.WriteFile(path, existing + text);
        return $"Appended {CountLines(text)} line(s) to {path}.";
    }

    [KernelFunction("delete_path")]
    [Description("Deletes a file or folder from the project.")]
    public string DeletePath(string path)
    {
        ctx.RequireRoot();
        ctx.Workspace.DeleteFile(path);
        ctx.Log($"Jack deleted {path}");
        return $"Deleted {path}.";
    }

    [KernelFunction("rename_path")]
    [Description("Renames or moves a file or folder within the project.")]
    public string RenamePath(string from, string to)
    {
        ctx.RequireRoot();
        ctx.Workspace.RenameFile(from, to);
        return $"Renamed {from} → {to}.";
    }

    [KernelFunction("create_folder")]
    [Description("Creates a folder in the project.")]
    public string CreateFolder(string path)
    {
        ctx.RequireRoot();
        ctx.Workspace.CreateFolder(path);
        return $"Created folder {path}.";
    }

    [KernelFunction("search_in_files")]
    [Description("Searches the project's text files with a .NET regular expression. Returns file:line: text matches (max 200).")]
    public string SearchInFiles([Description("Regex pattern")] string pattern, [Description("File glob such as *.cs, or empty for all text files")] string fileGlob = "")
    {
        ctx.RequireRoot();
        Regex regex;
        try
        {
            regex = new Regex(pattern, RegexOptions.IgnoreCase | RegexOptions.CultureInvariant, TimeSpan.FromSeconds(2));
        }
        catch (ArgumentException ex)
        {
            return "Error: invalid regex: " + ex.Message;
        }

        var globRegex = string.IsNullOrWhiteSpace(fileGlob) ? null : new Regex("^" + Regex.Escape(fileGlob).Replace("\\*", ".*").Replace("\\?", ".") + "$", RegexOptions.IgnoreCase);
        var sb = new StringBuilder();
        var hits = 0;
        foreach (var file in ctx.Workspace.ListFiles())
        {
            if (globRegex is not null && !globRegex.IsMatch(Path.GetFileName(file)))
            {
                continue;
            }

            if (!IsTextFile(file))
            {
                continue;
            }

            var lines = ctx.Workspace.ReadFile(file).Replace("\r\n", "\n").Split('\n');
            for (var i = 0; i < lines.Length; i++)
            {
                if (regex.IsMatch(lines[i]))
                {
                    sb.Append(file).Append(':').Append(i + 1).Append(": ").AppendLine(lines[i].Trim());
                    if (++hits >= 200)
                    {
                        return sb + "… (more matches omitted)";
                    }
                }
            }
        }

        return hits == 0 ? "No matches." : sb.ToString();
    }

    [KernelFunction("get_active_file")]
    [Description("Returns the path and content of the file currently open in the user's editor.")]
    public string GetActiveFile()
    {
        var path = ctx.Workspace.ActiveFile;
        if (string.IsNullOrEmpty(path))
        {
            return "No file is open in the editor.";
        }

        try
        {
            return $"{path}\n---\n{ctx.Truncate(ctx.Workspace.ReadFile(path))}";
        }
        catch (Exception ex) when (ex is FileNotFoundException or InvalidOperationException or UnauthorizedAccessException)
        {
            return $"{path} (could not be read: {ex.Message})";
        }
    }

    [KernelFunction("get_selection")]
    [Description("Returns the text the user has selected in the editor (empty if none).")]
    public string GetSelection() =>
        string.IsNullOrEmpty(ctx.Workspace.ActiveSelection) ? "(no selection)" : ctx.Workspace.ActiveSelection!;

    [KernelFunction("get_project_info")]
    [Description("Summarizes the open project: root folder, project files with target framework, output type and package/project references, and file counts.")]
    public string GetProjectInfo()
    {
        var root = ctx.RequireRoot();
        var sb = new StringBuilder();
        sb.AppendLine($"Root: {root}");
        var files = ctx.Workspace.ListFiles();
        sb.AppendLine($"Files: {files.Count} ({files.Count(f => f.EndsWith(".cs", StringComparison.OrdinalIgnoreCase))} .cs, {files.Count(f => f.EndsWith(".razor", StringComparison.OrdinalIgnoreCase))} .razor, {files.Count(f => f.EndsWith(".axaml", StringComparison.OrdinalIgnoreCase))} .axaml)");
        foreach (var proj in files.Where(f => f.EndsWith(".csproj", StringComparison.OrdinalIgnoreCase)))
        {
            sb.AppendLine($"Project: {proj}");
            try
            {
                var doc = XDocument.Parse(ctx.Workspace.ReadFile(proj));
                var sdk = doc.Root?.Attribute("Sdk")?.Value;
                string? Prop(string n) => doc.Descendants(n).FirstOrDefault()?.Value;
                sb.AppendLine($"  Sdk: {sdk}, TargetFramework: {Prop("TargetFramework") ?? Prop("TargetFrameworks")}, OutputType: {Prop("OutputType") ?? "Library"}");
                foreach (var r in doc.Descendants("PackageReference"))
                {
                    sb.AppendLine($"  package {r.Attribute("Include")?.Value} {r.Attribute("Version")?.Value}");
                }

                foreach (var r in doc.Descendants("ProjectReference"))
                {
                    sb.AppendLine($"  project {r.Attribute("Include")?.Value}");
                }
            }
            catch (System.Xml.XmlException ex)
            {
                sb.AppendLine($"  (invalid XML: {ex.Message})");
            }
        }

        if (files.Contains("nuget.config", StringComparer.OrdinalIgnoreCase))
        {
            sb.AppendLine("nuget.config present (local package feed).");
        }

        return sb.ToString();
    }

    internal static bool IsTextFile(string path) => Path.GetExtension(path).ToLowerInvariant() is
        ".cs" or ".csproj" or ".props" or ".targets" or ".razor" or ".cshtml" or ".axaml" or ".xaml" or ".json" or ".xml" or ".md" or ".txt"
        or ".yaml" or ".yml" or ".msg" or ".srv" or ".action" or ".urdf" or ".css" or ".js" or ".html" or ".config" or ".slnx" or ".sln" or ".editorconfig" or ".gitignore";

    private static int CountLines(string s) => s.Length == 0 ? 0 : s.Count(c => c == '\n') + (s.EndsWith('\n') ? 0 : 1);

    private static int CountOccurrences(string haystack, string needle)
    {
        var count = 0;
        var i = 0;
        while ((i = haystack.IndexOf(needle, i, StringComparison.Ordinal)) >= 0)
        {
            count++;
            i += needle.Length;
        }

        return count;
    }

    private static string ReplaceFirst(string text, string oldValue, string newValue)
    {
        var i = text.IndexOf(oldValue, StringComparison.Ordinal);
        return text[..i] + newValue + text[(i + oldValue.Length)..];
    }

    /// <summary>Approximate added/removed line counts (multiset difference of lines).</summary>
    internal static (int Added, int Removed) LineDiff(string before, string after)
    {
        var a = before.Replace("\r\n", "\n").Split('\n').GroupBy(l => l).ToDictionary(g => g.Key, g => g.Count());
        var b = after.Replace("\r\n", "\n").Split('\n').GroupBy(l => l).ToDictionary(g => g.Key, g => g.Count());
        var added = b.Sum(kv => Math.Max(0, kv.Value - a.GetValueOrDefault(kv.Key)));
        var removed = a.Sum(kv => Math.Max(0, kv.Value - b.GetValueOrDefault(kv.Key)));
        return (added, removed);
    }
}
