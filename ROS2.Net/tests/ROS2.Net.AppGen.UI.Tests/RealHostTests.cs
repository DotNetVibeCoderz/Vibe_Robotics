using Ros2AppGen;
using Ros2AppGen.Ui;
using Shouldly;

namespace ROS2.Net.AppGen.UI.Tests;

/// <summary>The window on the real backend (App.config in a temp folder), plus an opt-in live chat with a real LLM.</summary>
[Collection("headless")]
public class RealHostTests
{
    private static string TempConfig() => Path.Combine(Path.GetTempPath(), "ros2appgen-real-" + Guid.NewGuid().ToString("N")[..8], "App.config");

    [Fact]
    public void Real_host_starts_creates_config_and_edits_a_project()
    {
        var config = TempConfig();
        Directory.CreateDirectory(Path.GetDirectoryName(config)!);
        Headless.Run(() =>
        {
            Look.IsDark = true;
            var host = AppGenHost.Create(config);
            File.Exists(config).ShouldBeTrue();
            host.Settings.Profiles.ShouldNotBeEmpty();

            var w = new MainWindow(host) { Width = 1400, Height = 880 };
            w.Show();
            try
            {
                var root = FakeHost.CreateSampleProject("RealHostProbe");
                w.OpenProject(root);
                Headless.Pump(200);
                w.Editor.Active!.FileName.ShouldBe("Program.cs");
                w.Editor.Editor.Document.Insert(0, "class   Probe{int x;}\n");
                w.FormatCode();
                w.Editor.Editor.Document.Text.ShouldNotStartWith("class   Probe{int x;}");
                w.Editor.Save();
                w.PersistOnExit();
                File.ReadAllText(config).ShouldContain(root.Replace("\\", "\\\\"), Case.Insensitive, customMessage: "recent projects are saved to App.config");
            }
            finally
            {
                w.Close();
            }
        });
    }

    /// <summary>
    /// Talks to a real model through the UI view-model. Opt-in: set <c>ROS2NET_APPGEN_LIVE=1</c>. Keys are read at runtime
    /// from the developer key file and only ever written to a temp App.config that is deleted afterwards.
    /// </summary>
    [Fact]
    public void Live_chat_with_a_real_model_uses_tools_on_the_project()
    {
        if (Environment.GetEnvironmentVariable("ROS2NET_APPGEN_LIVE") != "1" || LiveKeys.Load() is not { } keys)
        {
            Assert.Skip("Set ROS2NET_APPGEN_LIVE=1 (and provide the developer key file) to run against a real LLM.");
            return;
        }

        var config = TempConfig();
        Directory.CreateDirectory(Path.GetDirectoryName(config)!);
        try
        {
            Headless.RunAsync(async () =>
            {
                var host = AppGenHost.Create(config);
                host.Settings.Profiles = keys.Profiles;
                host.Settings.ActiveProfile = keys.Profiles[0].Name;
                host.Assistant.UseProfile(host.Settings.ActiveProfile);

                var w = new MainWindow(host) { Width = 1600, Height = 960 };
                w.Show();
                try
                {
                    var root = FakeHost.CreateSampleProject("LiveChat");
                    w.OpenProject(root);
                    await w.ChatViewModel.SendAsync("How many .cs files does the open project contain? Use your tools to check, then answer with the number and the file names.");
                    Headless.Pump(400);

                    var reply = w.ChatViewModel.Messages.Last();
                    reply.IsError.ShouldBeFalse(reply.Text);
                    reply.Tools.ShouldNotBeEmpty();
                    reply.Text.ShouldContain("4");
                    Headless.Capture(w, Path.Combine(Path.GetTempPath(), "ros2appgen-live-chat.png"));
                }
                finally
                {
                    w.Close();
                }
            });
        }
        finally
        {
            try
            {
                Directory.Delete(Path.GetDirectoryName(config)!, recursive: true);
            }
            catch (IOException)
            {
            }
        }
    }
}

/// <summary>Reads model credentials from the developer key file (never committed).</summary>
internal sealed record LiveKeys(List<LlmProfile> Profiles)
{
    private const string KeyFile = "C:/Users/mifma/Documents/CodeSandbox/testkey.txt";

    public static LiveKeys? Load()
    {
        if (!File.Exists(KeyFile))
        {
            return null;
        }

        var sections = new Dictionary<string, Dictionary<string, string>>(StringComparer.OrdinalIgnoreCase);
        Dictionary<string, string>? current = null;
        foreach (var raw in File.ReadAllLines(KeyFile))
        {
            var line = raw.Trim();
            if (line.Length == 0 || line.StartsWith('#'))
            {
                continue;
            }

            var colon = line.IndexOf(':');
            if (colon < 0 || line.StartsWith("http", StringComparison.OrdinalIgnoreCase))
            {
                current = sections[line] = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                continue;
            }

            current?.TryAdd(line[..colon].Trim(), line[(colon + 1)..].Trim());
        }

        var profiles = new List<LlmProfile>();
        if (sections.TryGetValue("Anthropic", out var a) && a.TryGetValue("ApiKey", out var ak))
        {
            var headers = a.TryGetValue("workspaceId", out var ws) ? $"anthropic-workspace-id: {ws}" : string.Empty;
            profiles.Add(new LlmProfile { Name = "Claude (live)", Provider = LlmProviderKind.Claude, Model = First(a, "model", "claude-sonnet-5-5"), ApiKey = ak, ExtraHeaders = headers, Temperature = 0.2, MaxTokens = 4000 });
        }

        if (sections.TryGetValue("Azure OpenAI", out var z) && z.TryGetValue("apikey", out var zk) && z.TryGetValue("endpoint", out var ze))
        {
            profiles.Add(new LlmProfile { Name = "Azure (live)", Provider = LlmProviderKind.AzureOpenAI, Model = Last(z, "model", "gpt-5-mini"), ApiKey = zk, Endpoint = ze, Temperature = 1 });
        }

        if (sections.TryGetValue("DeepSeek", out var d) && d.TryGetValue("Deep Seek Api Key", out var dk))
        {
            profiles.Add(new LlmProfile { Name = "DeepSeek (live)", Provider = LlmProviderKind.DeepSeek, Model = First(d, "Model Id", "deepseek-v4-flash"), ApiKey = dk, Endpoint = d.GetValueOrDefault("Deep Seek Endpoint", "https://api.deepseek.com"), SupportsVision = false });
        }

        var preferred = Environment.GetEnvironmentVariable("ROS2NET_APPGEN_LIVE_PROFILE");
        if (preferred is not null)
        {
            profiles = profiles.OrderByDescending(p => p.Name.Contains(preferred, StringComparison.OrdinalIgnoreCase)).ToList();
        }

        return profiles.Count == 0 ? null : new LiveKeys(profiles);
    }

    private static string First(Dictionary<string, string> s, string key, string fallback) =>
        s.TryGetValue(key, out var v) ? v.Split(',')[0].Trim() : fallback;

    private static string Last(Dictionary<string, string> s, string key, string fallback) =>
        s.TryGetValue(key, out var v) ? v.Split(',')[^1].Trim() : fallback;
}
