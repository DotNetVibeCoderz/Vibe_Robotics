using System.Diagnostics;
using System.Text;
using System.Text.RegularExpressions;

namespace ROS2.Net.AppGen;

/// <summary>A compiler or MSBuild diagnostic parsed from <c>dotnet build</c> output.</summary>
public sealed record BuildDiagnostic(string File, int Line, int Column, string Severity, string Code, string Message)
{
    public override string ToString() => $"{File}({Line},{Column}): {Severity} {Code}: {Message}";
}

/// <summary>Runs the dotnet CLI for a project folder: build, run and publish, with streamed output.</summary>
public sealed partial class BuildService : IBuildService
{
    private readonly object _gate = new();
    private Process? _running;

    /// <summary>Optional log sink (the host forwards it to the logs panel).</summary>
    public Action<string>? Log { get; set; }

    public bool IsRunning
    {
        get
        {
            lock (_gate)
            {
                return _running is { HasExited: false };
            }
        }
    }

    public Task<ProcessResult> BuildAsync(string projectFolder, Action<ProcessOutput>? output, CancellationToken cancellationToken) =>
        RunDotnetAsync(["build", FindProject(projectFolder), "-nologo", "-v", "minimal", "-clp:NoSummary"], projectFolder, output, null, cancellationToken);

    public Task<ProcessResult> RunAsync(string projectFolder, Action<ProcessOutput>? output, CancellationToken cancellationToken) =>
        RunAsync(projectFolder, RunTargetEnvironment.DefaultSimulator, output, cancellationToken);

    public async Task<ProcessResult> RunAsync(string projectFolder, RunTarget target, Action<ProcessOutput>? output, CancellationToken cancellationToken)
    {
        ArgumentNullException.ThrowIfNull(target);
        if (IsRunning)
        {
            throw new InvalidOperationException("An app is already running. Stop it first.");
        }

        var args = new List<string> { "run", "--project", FindProject(projectFolder) };
        var appArgs = RunTargetEnvironment.ToArguments(target);
        if (appArgs.Count > 0)
        {
            args.Add("--");
            args.AddRange(appArgs);
        }

        Log?.Invoke($"Run target: {target.Name} ({target.Kind}, {RunTargetEnvironment.ToEnvironment(target)[RunTargetEnvironment.Rmw]})");
        return await RunDotnetAsync(args, projectFolder, output, null, cancellationToken, track: true, environment: RunTargetEnvironment.ToEnvironment(target)).ConfigureAwait(false);
    }

    /// <summary>Runs a console project for at most <paramref name="timeout"/>, then stops it; used by Jack to check behaviour.</summary>
    public Task<ProcessResult> RunBrieflyAsync(string projectFolder, TimeSpan timeout, Action<ProcessOutput>? output, CancellationToken cancellationToken, IEnumerable<string>? appArgs = null, RunTarget? target = null)
    {
        target ??= RunTargetEnvironment.DefaultSimulator;
        var args = new List<string> { "run", "--project", FindProject(projectFolder) };
        var all = new List<string>(appArgs ?? []);
        all.AddRange(RunTargetEnvironment.ToArguments(target));
        if (all.Count > 0)
        {
            args.Add("--");
            args.AddRange(all);
        }

        return RunDotnetAsync(args, projectFolder, output, timeout, cancellationToken, environment: RunTargetEnvironment.ToEnvironment(target));
    }

    public void StopRun()
    {
        lock (_gate)
        {
            if (_running is { HasExited: false } p)
            {
                try
                {
                    p.Kill(entireProcessTree: true);
                }
                catch (InvalidOperationException)
                {
                }
            }
        }
    }

    public Task<ProcessResult> DeployAsync(string projectFolder, string outputFolder, string runtime, bool selfContained, Action<ProcessOutput>? output, CancellationToken cancellationToken)
    {
        var args = new List<string> { "publish", FindProject(projectFolder), "-c", "Release", "-o", outputFolder, "-nologo" };
        if (!string.IsNullOrWhiteSpace(runtime))
        {
            args.AddRange(["-r", runtime]);
        }

        args.Add(selfContained ? "--self-contained" : "--no-self-contained");
        if (selfContained && !string.IsNullOrWhiteSpace(runtime))
        {
            args.Add("-p:PublishSingleFile=true");
        }

        return RunDotnetAsync(args, projectFolder, output, null, cancellationToken);
    }

    /// <summary>Runs <c>dotnet add package</c>.</summary>
    public Task<ProcessResult> AddPackageAsync(string projectFolder, string package, string? version, Action<ProcessOutput>? output, CancellationToken cancellationToken)
    {
        var args = new List<string> { "add", FindProject(projectFolder), "package", package };
        if (!string.IsNullOrWhiteSpace(version))
        {
            args.AddRange(["--version", version]);
        }

        return RunDotnetAsync(args, projectFolder, output, TimeSpan.FromMinutes(3), cancellationToken);
    }

    /// <summary>Finds the project to build in a folder: a .slnx/.sln at the root, else the first .csproj.</summary>
    public static string FindProject(string folder)
    {
        if (File.Exists(folder))
        {
            return folder;
        }

        var solution = Directory.EnumerateFiles(folder, "*.slnx").Concat(Directory.EnumerateFiles(folder, "*.sln")).FirstOrDefault();
        if (solution is not null)
        {
            return solution;
        }

        var project = Directory.EnumerateFiles(folder, "*.csproj", SearchOption.TopDirectoryOnly).FirstOrDefault()
            ?? Directory.EnumerateFiles(folder, "*.csproj", SearchOption.AllDirectories)
                .Where(p => !p.Contains($"{Path.DirectorySeparatorChar}obj{Path.DirectorySeparatorChar}") && !p.Contains($"{Path.DirectorySeparatorChar}bin{Path.DirectorySeparatorChar}"))
                .OrderBy(p => p.Length)
                .FirstOrDefault();
        return project ?? throw new FileNotFoundException($"No .csproj, .slnx or .sln found in '{folder}'.");
    }

    /// <summary>Extracts errors and warnings from build output (deduplicated, in order).</summary>
    public static IReadOnlyList<BuildDiagnostic> ParseDiagnostics(string output)
    {
        var seen = new HashSet<string>(StringComparer.Ordinal);
        var list = new List<BuildDiagnostic>();
        foreach (Match m in DiagnosticRegex().Matches(output))
        {
            var d = new BuildDiagnostic(
                m.Groups["file"].Value.Trim(),
                int.TryParse(m.Groups["line"].Value, out var l) ? l : 0,
                int.TryParse(m.Groups["col"].Value, out var c) ? c : 0,
                m.Groups["sev"].Value,
                m.Groups["code"].Value,
                m.Groups["msg"].Value.Trim());
            if (seen.Add(d.ToString()))
            {
                list.Add(d);
            }
        }

        return list;
    }

    [GeneratedRegex(@"^\s*(?<file>[^\r\n(]+?)\((?<line>\d+),(?<col>\d+)(?:,\d+,\d+)?\):\s*(?<sev>error|warning)\s+(?<code>[A-Z]+\d+)\s*:\s*(?<msg>[^\r\n\[]+)", RegexOptions.Multiline)]
    private static partial Regex DiagnosticRegex();

    private async Task<ProcessResult> RunDotnetAsync(IReadOnlyList<string> args, string workingDirectory, Action<ProcessOutput>? output, TimeSpan? timeout, CancellationToken cancellationToken, bool track = false, IReadOnlyDictionary<string, string>? environment = null)
    {
        var psi = new ProcessStartInfo("dotnet")
        {
            WorkingDirectory = Directory.Exists(workingDirectory) ? workingDirectory : Path.GetDirectoryName(workingDirectory)!,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            CreateNoWindow = true,
            StandardOutputEncoding = Encoding.UTF8,
            StandardErrorEncoding = Encoding.UTF8,
        };
        foreach (var a in args)
        {
            psi.ArgumentList.Add(a);
        }

        psi.Environment["DOTNET_CLI_TELEMETRY_OPTOUT"] = "1";
        psi.Environment["DOTNET_NOLOGO"] = "1";
        psi.Environment["MSBUILDTERMINALLOGGER"] = "off";
        foreach (var (k, v) in environment ?? new Dictionary<string, string>())
        {
            psi.Environment[k] = v;
        }

        Log?.Invoke("> dotnet " + string.Join(' ', args.Select(a => a.Contains(' ') ? $"\"{a}\"" : a)));
        var sb = new StringBuilder();
        var stopwatch = Stopwatch.StartNew();
        using var process = new Process { StartInfo = psi, EnableRaisingEvents = true };
        void OnLine(string? line, bool error)
        {
            if (line is null)
            {
                return;
            }

            lock (sb)
            {
                sb.AppendLine(line);
            }

            output?.Invoke(new ProcessOutput(line, error));
        }

        process.OutputDataReceived += (_, e) => OnLine(e.Data, false);
        process.ErrorDataReceived += (_, e) => OnLine(e.Data, true);
        process.Start();
        process.BeginOutputReadLine();
        process.BeginErrorReadLine();
        if (track)
        {
            lock (_gate)
            {
                _running = process;
            }
        }

        using var timeoutCts = timeout is { } t ? new CancellationTokenSource(t) : new CancellationTokenSource();
        using var linked = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken, timeoutCts.Token);
        var timedOut = false;
        try
        {
            await process.WaitForExitAsync(linked.Token).ConfigureAwait(false);
        }
        catch (OperationCanceledException)
        {
            timedOut = timeoutCts.IsCancellationRequested && !cancellationToken.IsCancellationRequested;
            try
            {
                process.Kill(entireProcessTree: true);
            }
            catch (InvalidOperationException)
            {
            }

            await process.WaitForExitAsync(CancellationToken.None).ConfigureAwait(false);
            if (!timedOut)
            {
                throw;
            }
        }
        finally
        {
            if (track)
            {
                lock (_gate)
                {
                    _running = null;
                }
            }
        }

        // Let the async readers flush.
        process.WaitForExit();
        string text;
        lock (sb)
        {
            text = sb.ToString();
        }

        // A run stopped by its time budget is a success for "run briefly" purposes.
        var exit = timedOut ? 0 : process.ExitCode;
        Log?.Invoke($"< exit {exit} in {stopwatch.Elapsed.TotalSeconds:0.0}s{(timedOut ? " (stopped after time budget)" : string.Empty)}");
        return new ProcessResult(exit, text, stopwatch.Elapsed);
    }
}
