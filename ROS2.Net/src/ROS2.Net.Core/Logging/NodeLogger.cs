using Microsoft.Extensions.Logging;
using LogMsg = ROS2.Net.Msgs.RclInterfaces.Log;

namespace ROS2.Net;

/// <summary>
/// A node's logger: forwards to the application's <see cref="ILoggerFactory"/> and, when rosout is
/// enabled, publishes each entry as <c>rcl_interfaces/msg/Log</c> on <c>/rosout</c> so tools like
/// <c>rqt_console</c> and <c>ros2net topic echo /rosout</c> see it.
/// </summary>
public sealed class NodeLogger : ILogger
{
    private readonly ILogger _inner;
    private readonly string _name;
    private Publisher<LogMsg>? _rosout;
    private LogLevel _minimum;

    [ThreadStatic]
    private static bool t_publishing;

    internal NodeLogger(ILogger inner, string name, LogLevel minimum)
    {
        _inner = inner;
        _name = name;
        _minimum = minimum;
    }

    /// <summary>Logger name in ROS form, e.g. <c>robot1.talker</c>.</summary>
    public string Name => _name;

    /// <summary>Minimum level for this logger (<c>--log-level</c>).</summary>
    public LogLevel MinimumLevel
    {
        get => _minimum;
        set => _minimum = value;
    }

    internal void AttachRosout(Publisher<LogMsg> publisher) => _rosout = publisher;

    internal void DetachRosout() => _rosout = null;

    public IDisposable? BeginScope<TState>(TState state)
        where TState : notnull => _inner.BeginScope(state);

    public bool IsEnabled(LogLevel logLevel) => logLevel >= _minimum && (logLevel != LogLevel.None) && (_inner.IsEnabled(logLevel) || _rosout is not null);

    public void Log<TState>(LogLevel logLevel, EventId eventId, TState state, Exception? exception, Func<TState, Exception?, string> formatter)
    {
        if (logLevel < _minimum || logLevel == LogLevel.None)
        {
            return;
        }

        _inner.Log(logLevel, eventId, state, exception, formatter);

        var rosout = _rosout;
        if (rosout is null || t_publishing)
        {
            return;
        }

        t_publishing = true;
        try
        {
            var text = formatter(state, exception);
            if (exception is not null)
            {
                text += Environment.NewLine + exception;
            }

            var msg = new LogMsg
            {
                Stamp = RosTime.Now().ToMsg(),
                Level = ToRosLevel(logLevel),
                Name = _name,
                Msg = text,
                File = string.Empty,
                Function = eventId.Name ?? string.Empty,
                Line = (uint)Math.Max(0, eventId.Id),
            };
            rosout.Publish(msg);
        }
        catch (ObjectDisposedException)
        {
        }
        finally
        {
            t_publishing = false;
        }
    }

    public static byte ToRosLevel(LogLevel level) => level switch
    {
        LogLevel.Trace or LogLevel.Debug => LogMsg.DEBUG,
        LogLevel.Information => LogMsg.INFO,
        LogLevel.Warning => LogMsg.WARN,
        LogLevel.Error => LogMsg.ERROR,
        _ => LogMsg.FATAL,
    };

    public static LogLevel FromRosLevel(byte level) => level switch
    {
        <= LogMsg.DEBUG => LogLevel.Debug,
        <= LogMsg.INFO => LogLevel.Information,
        <= LogMsg.WARN => LogLevel.Warning,
        <= LogMsg.ERROR => LogLevel.Error,
        _ => LogLevel.Critical,
    };
}
