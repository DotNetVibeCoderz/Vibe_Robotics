using Microsoft.Extensions.Logging;

namespace ROS2.Net;

/// <summary>
/// Parsed <c>--ros-args</c> command-line arguments, as understood by rcl:
/// <c>-r from:=to</c>, <c>-r node:from:=to</c>, <c>-r __ns:=/ns</c>, <c>-r __node:=name</c>,
/// <c>-p name:=value</c>, <c>-p node:name:=value</c>, <c>--params-file file.yaml</c>,
/// <c>--log-level [logger:=]level</c>, <c>-e/--enclave name</c>, terminated by <c>--</c>.
/// </summary>
public sealed class RosArguments
{
    public static RosArguments Empty { get; } = new();

    public List<RemapRule> Remappings { get; } = [];

    /// <summary>Parameter overrides from <c>-p</c>; key is <c>name</c> or <c>node:name</c>.</summary>
    public List<(string? Node, Parameter Parameter)> Parameters { get; } = [];

    public List<string> ParameterFiles { get; } = [];

    public LogLevel? LogLevel { get; set; }

    public Dictionary<string, LogLevel> LoggerLevels { get; } = new(StringComparer.Ordinal);

    public string? Enclave { get; set; }

    /// <summary>Arguments that were not ROS arguments.</summary>
    public List<string> UnparsedArguments { get; } = [];

    /// <summary>Parses a full command line, extracting every <c>--ros-args ... [--]</c> block.</summary>
    public static RosArguments Parse(IEnumerable<string>? args)
    {
        var result = new RosArguments();
        if (args is null)
        {
            return result;
        }

        var list = args.ToList();
        var inRos = false;
        for (var i = 0; i < list.Count; i++)
        {
            var a = list[i];
            if (a == "--ros-args")
            {
                inRos = true;
                continue;
            }

            if (!inRos)
            {
                result.UnparsedArguments.Add(a);
                continue;
            }

            if (a == "--")
            {
                inRos = false;
                continue;
            }

            string Next() => i + 1 < list.Count ? list[++i] : throw new ArgumentException($"ROS argument '{a}' requires a value.");

            switch (a)
            {
                case "-r" or "--remap":
                    result.Remappings.Add(RemapRule.Parse(Next()));
                    break;
                case "-p" or "--param":
                    result.Parameters.Add(ParseParam(Next()));
                    break;
                case "--params-file":
                    result.ParameterFiles.Add(Next());
                    break;
                case "--log-level":
                    var level = Next();
                    var sep = level.IndexOf(":=", StringComparison.Ordinal);
                    if (sep > 0)
                    {
                        result.LoggerLevels[level[..sep]] = ParseLevel(level[(sep + 2)..]);
                    }
                    else
                    {
                        result.LogLevel = ParseLevel(level);
                    }

                    break;
                case "-e" or "--enclave":
                    result.Enclave = Next();
                    break;
                case "--disable-rosout-logs" or "--enable-rosout-logs" or "--disable-stdout-logs" or "--enable-stdout-logs" or "--disable-external-lib-logs":
                    break;
                default:
                    result.UnparsedArguments.Add(a);
                    break;
            }
        }

        return result;
    }

    private static (string? Node, Parameter Parameter) ParseParam(string text)
    {
        var sep = text.IndexOf(":=", StringComparison.Ordinal);
        if (sep <= 0)
        {
            throw new ArgumentException($"Parameter argument '{text}' must have the form name:=value.");
        }

        var name = text[..sep];
        var value = ParameterValue.Parse(text[(sep + 2)..]);
        var colon = name.IndexOf(':');
        return colon > 0
            ? (name[..colon], new Parameter(name[(colon + 1)..], value))
            : (null, new Parameter(name, value));
    }

    private static LogLevel ParseLevel(string s) => s.ToLowerInvariant() switch
    {
        "debug" => Microsoft.Extensions.Logging.LogLevel.Debug,
        "info" => Microsoft.Extensions.Logging.LogLevel.Information,
        "warn" or "warning" => Microsoft.Extensions.Logging.LogLevel.Warning,
        "error" => Microsoft.Extensions.Logging.LogLevel.Error,
        "fatal" => Microsoft.Extensions.Logging.LogLevel.Critical,
        "unset" => Microsoft.Extensions.Logging.LogLevel.Trace,
        _ => throw new ArgumentException($"Unknown log level '{s}'."),
    };

    /// <summary>Parameter overrides from files and <c>-p</c> that apply to a node.</summary>
    public Dictionary<string, ParameterValue> ParametersFor(string nodeName, string fullyQualifiedName)
    {
        var result = new Dictionary<string, ParameterValue>(StringComparer.Ordinal);
        foreach (var file in ParameterFiles)
        {
            foreach (var p in ParameterYaml.LoadFile(file, fullyQualifiedName))
            {
                result[p.Name] = p.Value;
            }
        }

        foreach (var (node, p) in Parameters)
        {
            if (node is null || node == nodeName || node == fullyQualifiedName)
            {
                result[p.Name] = p.Value;
            }
        }

        return result;
    }
}
