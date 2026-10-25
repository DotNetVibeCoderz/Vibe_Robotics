using System.Configuration;
using System.Globalization;
using System.Reflection;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Xml.Linq;

namespace ROS2.Net.AppGen;

/// <summary>
/// Persists <see cref="AppGenSettings"/> in an <c>App.config</c> file, one <c>appSettings</c> entry per
/// property (<c>AppGen.&lt;Property&gt;</c>). Scalars are stored as invariant text, lists (profiles,
/// recent projects) as JSON. New properties are picked up automatically.
/// </summary>
/// <remarks>
/// API keys are stored as written. A key of the form <c>env:NAME</c> is resolved from the environment
/// variable <c>NAME</c> at use time instead (see <see cref="ResolveSecret"/>), so the file can stay free of secrets.
/// </remarks>
public sealed class AppConfigSettingsStore : ISettingsStore
{
    private const string Prefix = "AppGen.";

    private static readonly JsonSerializerOptions s_json = new()
    {
        WriteIndented = false,
        Converters = { new JsonStringEnumConverter() },
    };

    private static readonly PropertyInfo[] s_properties =
        typeof(AppGenSettings).GetProperties(BindingFlags.Public | BindingFlags.Instance).Where(p => p.CanRead && p.CanWrite).ToArray();

    private readonly object _gate = new();

    /// <param name="configPath">Config file; defaults to <c>App.config</c> next to the executable.</param>
    public AppConfigSettingsStore(string? configPath = null)
    {
        ConfigPath = Path.GetFullPath(configPath ?? Path.Combine(AppContext.BaseDirectory, "App.config"));
    }

    public string ConfigPath { get; }

    public AppGenSettings Load()
    {
        lock (_gate)
        {
            if (!File.Exists(ConfigPath))
            {
                var defaults = DefaultSettings.Create();
                Save(defaults);
                return defaults;
            }

            var config = Open();
            var settings = DefaultSettings.Create();
            var app = config.AppSettings.Settings;
            foreach (var p in s_properties)
            {
                var entry = app[Prefix + p.Name];
                if (entry is null)
                {
                    continue;
                }

                try
                {
                    p.SetValue(settings, Parse(p.PropertyType, entry.Value));
                }
                catch (Exception ex) when (ex is FormatException or JsonException or InvalidCastException or OverflowException)
                {
                    // Keep the default for a malformed entry rather than failing the whole load.
                }
            }

            settings.Profiles ??= [];
            settings.RecentProjects ??= [];
            return settings;
        }
    }

    public void Save(AppGenSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        lock (_gate)
        {
            if (!File.Exists(ConfigPath))
            {
                Directory.CreateDirectory(Path.GetDirectoryName(ConfigPath)!);
                File.WriteAllText(ConfigPath, "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<configuration>\n  <appSettings />\n</configuration>\n");
            }

            var config = Open();
            var app = config.AppSettings.Settings;
            foreach (var p in s_properties)
            {
                var key = Prefix + p.Name;
                var value = Format(p.GetValue(settings));
                if (app[key] is null)
                {
                    app.Add(key, value);
                }
                else
                {
                    app[key].Value = value;
                }
            }

            config.Save(ConfigurationSaveMode.Modified);
        }
    }

    /// <summary>Serializes settings as a complete App.config document.</summary>
    public static string ToXml(AppGenSettings settings)
    {
        var appSettings = new XElement("appSettings",
            s_properties.Select(p => new XElement("add", new XAttribute("key", Prefix + p.Name), new XAttribute("value", Format(p.GetValue(settings))))));
        var doc = new XDocument(new XDeclaration("1.0", "utf-8", null),
            new XComment(" ROS2.Net App Generator settings. Edit them from Settings in the app. API keys may be written as env:VARIABLE_NAME. "),
            new XElement("configuration", appSettings));
        return doc.Declaration + Environment.NewLine + doc.ToString();
    }

    /// <summary>Resolves <c>env:NAME</c> references; other values are returned unchanged.</summary>
    public static string ResolveSecret(string? value)
    {
        if (string.IsNullOrEmpty(value))
        {
            return string.Empty;
        }

        return value.StartsWith("env:", StringComparison.OrdinalIgnoreCase)
            ? Environment.GetEnvironmentVariable(value[4..].Trim()) ?? string.Empty
            : value;
    }

    /// <summary>Masks a secret for display/logging: first 4 characters, then asterisks.</summary>
    public static string Mask(string? secret) =>
        string.IsNullOrEmpty(secret) ? "(empty)" :
        secret.StartsWith("env:", StringComparison.OrdinalIgnoreCase) ? secret :
        secret.Length <= 6 ? "******" : secret[..4] + new string('*', 8);

    private Configuration Open() =>
        ConfigurationManager.OpenMappedExeConfiguration(new ExeConfigurationFileMap { ExeConfigFilename = ConfigPath }, ConfigurationUserLevel.None);

    private static string Format(object? value) => value switch
    {
        null => string.Empty,
        string s => s,
        bool b => b ? "true" : "false",
        double d => d.ToString("R", CultureInfo.InvariantCulture),
        int i => i.ToString(CultureInfo.InvariantCulture),
        Enum e => e.ToString(),
        _ => JsonSerializer.Serialize(value, value.GetType(), s_json),
    };

    private static object? Parse(Type type, string text)
    {
        if (type == typeof(string))
        {
            return text;
        }

        if (type == typeof(bool))
        {
            return bool.Parse(text);
        }

        if (type == typeof(double))
        {
            return double.Parse(text, CultureInfo.InvariantCulture);
        }

        if (type == typeof(int))
        {
            return int.Parse(text, CultureInfo.InvariantCulture);
        }

        if (type.IsEnum)
        {
            return Enum.Parse(type, text, ignoreCase: true);
        }

        return string.IsNullOrWhiteSpace(text) ? Activator.CreateInstance(type) : JsonSerializer.Deserialize(text, type, s_json);
    }
}
