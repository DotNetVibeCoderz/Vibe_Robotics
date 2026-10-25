namespace ROS2.Net.AppGen;

/// <summary>
/// A project folder on disk. Every path is relative to <see cref="RootPath"/> and is rejected if it
/// escapes it (<c>..</c>, rooted paths, symlinks are not followed out). Build output (<c>bin</c>,
/// <c>obj</c>) and tool folders are hidden from listings and change notifications.
/// </summary>
public sealed class FileSystemWorkspace : IWorkspace, IDisposable
{
    private static readonly HashSet<string> s_ignoredFolders = new(StringComparer.OrdinalIgnoreCase)
    {
        "bin", "obj", ".git", ".vs", ".vscode", ".idea", "node_modules", "TestResults", "publish",
    };

    private FileSystemWatcher? _watcher;

    public string? RootPath { get; private set; }

    public string? ActiveFile { get; set; }

    public string? ActiveSelection { get; set; }

    public event Action<string>? FileChanged;

    public event Action? ProjectChanged;

    public void Open(string folderOrProjectFile)
    {
        var full = Path.GetFullPath(folderOrProjectFile);
        var folder = File.Exists(full) ? Path.GetDirectoryName(full)! : full;
        if (!Directory.Exists(folder))
        {
            throw new DirectoryNotFoundException($"Project folder '{folder}' does not exist.");
        }

        CloseCore();
        RootPath = folder;
        _watcher = new FileSystemWatcher(folder)
        {
            IncludeSubdirectories = true,
            NotifyFilter = NotifyFilters.FileName | NotifyFilters.DirectoryName | NotifyFilters.LastWrite | NotifyFilters.Size,
            EnableRaisingEvents = true,
        };
        _watcher.Changed += (_, e) => Raise(e.FullPath);
        _watcher.Created += (_, e) => Raise(e.FullPath);
        _watcher.Deleted += (_, e) => Raise(e.FullPath);
        _watcher.Renamed += (_, e) =>
        {
            Raise(e.OldFullPath);
            Raise(e.FullPath);
        };
        ProjectChanged?.Invoke();
    }

    public void Close()
    {
        CloseCore();
        ProjectChanged?.Invoke();
    }

    private void CloseCore()
    {
        _watcher?.Dispose();
        _watcher = null;
        RootPath = null;
        ActiveFile = null;
        ActiveSelection = null;
    }

    public IReadOnlyList<string> ListFiles(string? relativeFolder = null, bool recursive = true)
    {
        var folder = string.IsNullOrWhiteSpace(relativeFolder) ? Root : Resolve(relativeFolder);
        if (!Directory.Exists(folder))
        {
            return [];
        }

        var result = new List<string>();
        Walk(folder, recursive, result);
        result.Sort(StringComparer.OrdinalIgnoreCase);
        return result;
    }

    private void Walk(string folder, bool recursive, List<string> result)
    {
        foreach (var file in Directory.EnumerateFiles(folder))
        {
            result.Add(ToRelative(file));
        }

        if (!recursive)
        {
            return;
        }

        foreach (var dir in Directory.EnumerateDirectories(folder))
        {
            if (!s_ignoredFolders.Contains(Path.GetFileName(dir)) && (File.GetAttributes(dir) & FileAttributes.ReparsePoint) == 0)
            {
                Walk(dir, recursive, result);
            }
        }
    }

    public string ReadFile(string relativePath)
    {
        var path = Resolve(relativePath);
        if (!File.Exists(path))
        {
            throw new FileNotFoundException($"File '{relativePath}' does not exist in the project.");
        }

        return File.ReadAllText(path);
    }

    public void WriteFile(string relativePath, string content)
    {
        var path = Resolve(relativePath);
        Directory.CreateDirectory(Path.GetDirectoryName(path)!);
        File.WriteAllText(path, content);
        Raise(path);
    }

    public void DeleteFile(string relativePath)
    {
        var path = Resolve(relativePath);
        if (File.Exists(path))
        {
            File.Delete(path);
        }
        else if (Directory.Exists(path))
        {
            Directory.Delete(path, recursive: true);
        }
        else
        {
            throw new FileNotFoundException($"'{relativePath}' does not exist in the project.");
        }

        Raise(path);
    }

    public void RenameFile(string from, string to)
    {
        var source = Resolve(from);
        var target = Resolve(to);
        Directory.CreateDirectory(Path.GetDirectoryName(target)!);
        if (Directory.Exists(source))
        {
            Directory.Move(source, target);
        }
        else
        {
            File.Move(source, target);
        }

        Raise(source);
        Raise(target);
    }

    public void CreateFolder(string relativePath) => Directory.CreateDirectory(Resolve(relativePath));

    /// <summary>Resolves a project-relative path, rejecting anything outside the project root.</summary>
    public string Resolve(string relativePath)
    {
        if (string.IsNullOrWhiteSpace(relativePath))
        {
            throw new ArgumentException("Path must not be empty.", nameof(relativePath));
        }

        var root = Root;
        var cleaned = relativePath.Replace('\\', '/').TrimStart('/');
        if (Path.IsPathRooted(relativePath) && !Path.GetFullPath(relativePath).StartsWith(root, StringComparison.OrdinalIgnoreCase))
        {
            throw new UnauthorizedAccessException($"'{relativePath}' is outside the project folder.");
        }

        var full = Path.GetFullPath(Path.IsPathRooted(relativePath) ? relativePath : Path.Combine(root, cleaned));
        var rootWithSep = root.EndsWith(Path.DirectorySeparatorChar) ? root : root + Path.DirectorySeparatorChar;
        if (!full.Equals(root, StringComparison.OrdinalIgnoreCase) && !full.StartsWith(rootWithSep, StringComparison.OrdinalIgnoreCase))
        {
            throw new UnauthorizedAccessException($"'{relativePath}' is outside the project folder.");
        }

        return full;
    }

    /// <summary>Converts an absolute path inside the project to a forward-slash relative path.</summary>
    public string ToRelative(string fullPath) => Path.GetRelativePath(Root, fullPath).Replace('\\', '/');

    private string Root => RootPath ?? throw new InvalidOperationException("No project is open.");

    private void Raise(string fullPath)
    {
        if (RootPath is null)
        {
            return;
        }

        var rel = Path.GetRelativePath(RootPath, fullPath).Replace('\\', '/');
        var first = rel.Split('/')[0];
        if (s_ignoredFolders.Contains(first) || rel.Split('/').Any(s_ignoredFolders.Contains))
        {
            return;
        }

        FileChanged?.Invoke(fullPath);
    }

    public void Dispose() => CloseCore();
}
