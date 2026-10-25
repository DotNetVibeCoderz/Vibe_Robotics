using System.ComponentModel;
using System.Globalization;
using Microsoft.SemanticKernel;

namespace ROS2.Net.AppGen.Plugins;

/// <summary>Exact arithmetic for the assistant (LLMs are unreliable calculators).</summary>
public sealed class MathPlugin
{
    [KernelFunction("calculate")]
    [Description("Evaluates a math expression exactly. Supports + - * / % ^, parentheses, pi, e, and functions sin cos tan asin acos atan atan2 sqrt abs log ln exp pow min max round floor ceiling deg rad hypot clamp (radians). Example: 'sqrt(2)*sin(rad(30))'.")]
    public string Calculate([Description("The expression, e.g. '2*pi*0.033'")] string expression)
    {
        try
        {
            var v = ExpressionEvaluator.Evaluate(expression);
            return double.IsFinite(v) ? v.ToString("G15", CultureInfo.InvariantCulture) : v.ToString(CultureInfo.InvariantCulture);
        }
        catch (FormatException ex)
        {
            return "Error: " + ex.Message;
        }
    }

    [KernelFunction("convert_units")]
    [Description("Converts between common robotics units: m, cm, mm, km, in, ft; rad, deg; m/s, km/h, mph; s, ms, min, h; kg, g, lb; N, kN.")]
    public string ConvertUnits(double value, [Description("Source unit")] string from, [Description("Target unit")] string to)
    {
        static (string Dim, double Factor)? Unit(string u) => u.Trim().ToLowerInvariant() switch
        {
            "m" => ("len", 1), "cm" => ("len", 0.01), "mm" => ("len", 0.001), "km" => ("len", 1000), "in" => ("len", 0.0254), "ft" => ("len", 0.3048),
            "rad" => ("ang", 1), "deg" => ("ang", Math.PI / 180),
            "m/s" => ("vel", 1), "km/h" or "kmh" => ("vel", 1 / 3.6), "mph" => ("vel", 0.44704),
            "s" => ("time", 1), "ms" => ("time", 0.001), "min" => ("time", 60), "h" => ("time", 3600),
            "kg" => ("mass", 1), "g" => ("mass", 0.001), "lb" => ("mass", 0.45359237),
            "n" => ("force", 1), "kn" => ("force", 1000),
            _ => null,
        };

        if (Unit(from) is not { } f || Unit(to) is not { } t)
        {
            return $"Error: unknown unit '{(Unit(from) is null ? from : to)}'.";
        }

        if (f.Dim != t.Dim)
        {
            return $"Error: cannot convert {from} to {to}.";
        }

        return (value * f.Factor / t.Factor).ToString("G12", CultureInfo.InvariantCulture) + " " + to;
    }
}

/// <summary>Clock and calendar helpers.</summary>
public sealed class DateTimePlugin
{
    private readonly TimeProvider _time;

    public DateTimePlugin(TimeProvider? time = null) => _time = time ?? TimeProvider.System;

    [KernelFunction("now")]
    [Description("Current date and time: local time with its time zone and UTC, plus day of week and ISO week.")]
    public string Now()
    {
        var utc = _time.GetUtcNow();
        var tz = _time.LocalTimeZone;
        var local = TimeZoneInfo.ConvertTime(utc, tz);
        return $"local: {local:yyyy-MM-dd HH:mm:ss zzz} ({tz.Id}), utc: {utc:yyyy-MM-dd HH:mm:ss}Z, day: {local.DayOfWeek}, iso week: {ISOWeek.GetWeekOfYear(local.DateTime)}";
    }

    [KernelFunction("convert_timezone")]
    [Description("Converts a date/time (ISO 8601, or 'now') from one IANA/Windows time zone to another, e.g. 'Asia/Jakarta' to 'Europe/Berlin'.")]
    public string ConvertTimeZone(string dateTime, string fromZone, string toZone)
    {
        try
        {
            var from = TimeZoneInfo.FindSystemTimeZoneById(fromZone);
            var to = TimeZoneInfo.FindSystemTimeZoneById(toZone);
            var source = dateTime.Trim().Equals("now", StringComparison.OrdinalIgnoreCase)
                ? TimeZoneInfo.ConvertTime(_time.GetUtcNow(), from).DateTime
                : DateTime.Parse(dateTime, CultureInfo.InvariantCulture, DateTimeStyles.None);
            var utc = TimeZoneInfo.ConvertTimeToUtc(DateTime.SpecifyKind(source, DateTimeKind.Unspecified), from);
            var target = TimeZoneInfo.ConvertTimeFromUtc(utc, to);
            return $"{target:yyyy-MM-dd HH:mm:ss} {to.Id} (UTC{to.GetUtcOffset(utc):hh\\:mm})";
        }
        catch (Exception ex) when (ex is TimeZoneNotFoundException or InvalidTimeZoneException or FormatException or ArgumentException)
        {
            return "Error: " + ex.Message;
        }
    }

    [KernelFunction("date_diff")]
    [Description("Difference between two ISO 8601 dates/times: days, hours, minutes and seconds.")]
    public string DateDiff(string start, string end)
    {
        if (!DateTimeOffset.TryParse(start, CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal, out var a) ||
            !DateTimeOffset.TryParse(end, CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal, out var b))
        {
            return "Error: dates must be ISO 8601, e.g. 2026-10-25 or 2026-10-25T08:30:00+07:00.";
        }

        var d = b - a;
        return $"{d.TotalDays:0.####} days ({(int)d.TotalDays}d {d.Hours}h {d.Minutes}m {d.Seconds}s), {d.TotalSeconds:0} seconds";
    }

    [KernelFunction("add_duration")]
    [Description("Adds a duration to a date/time. Duration examples: '90m', '2h30m', '3d', '-1d', '45s'.")]
    public string AddDuration(string dateTime, string duration)
    {
        var baseTime = dateTime.Trim().Equals("now", StringComparison.OrdinalIgnoreCase)
            ? _time.GetUtcNow()
            : DateTimeOffset.TryParse(dateTime, CultureInfo.InvariantCulture, DateTimeStyles.AssumeUniversal, out var p) ? p : (DateTimeOffset?)null;
        if (baseTime is null)
        {
            return "Error: date must be ISO 8601 or 'now'.";
        }

        if (!TryParseDuration(duration, out var span))
        {
            return "Error: duration must look like '2h30m', '3d' or '-45s'.";
        }

        return (baseTime.Value + span).ToString("yyyy-MM-dd HH:mm:ss zzz", CultureInfo.InvariantCulture);
    }

    /// <summary>Parses compact durations such as <c>1d2h3m4s</c> (optional leading sign).</summary>
    public static bool TryParseDuration(string text, out TimeSpan span)
    {
        span = TimeSpan.Zero;
        var s = text.Trim().ToLowerInvariant().Replace(" ", string.Empty, StringComparison.Ordinal);
        var sign = 1;
        if (s.StartsWith('-'))
        {
            sign = -1;
            s = s[1..];
        }
        else if (s.StartsWith('+'))
        {
            s = s[1..];
        }

        if (s.Length == 0)
        {
            return false;
        }

        var i = 0;
        while (i < s.Length)
        {
            var start = i;
            while (i < s.Length && (char.IsDigit(s[i]) || s[i] == '.'))
            {
                i++;
            }

            if (start == i || i >= s.Length || !double.TryParse(s[start..i], NumberStyles.Float, CultureInfo.InvariantCulture, out var n))
            {
                return false;
            }

            var unitStart = i;
            while (i < s.Length && char.IsLetter(s[i]))
            {
                i++;
            }

            TimeSpan? part = s[unitStart..i] switch
            {
                "w" => TimeSpan.FromDays(7 * n),
                "d" => TimeSpan.FromDays(n),
                "h" => TimeSpan.FromHours(n),
                "m" or "min" => TimeSpan.FromMinutes(n),
                "s" => TimeSpan.FromSeconds(n),
                "ms" => TimeSpan.FromMilliseconds(n),
                _ => null,
            };
            if (part is null)
            {
                return false;
            }

            span += part.Value;
        }

        span = sign * span;
        return true;
    }
}
