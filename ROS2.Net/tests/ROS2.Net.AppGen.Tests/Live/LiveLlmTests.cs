using ROS2.Net.AppGen.Ai;
using ROS2.Net.AppGen.Plugins;
using Shouldly;

namespace ROS2.Net.AppGen.Tests.Live;

/// <summary>
/// Real-provider tests. Opt-in: set <c>ROS2NET_APPGEN_LIVE=1</c> (keys come from the local key file, see <see cref="TestKeys"/>).
/// </summary>
[Trait("Category", "RealLlm")]
public sealed class LiveLlmTests
{
    private static void SkipUnlessLive(object? credential)
    {
        Assert.SkipUnless(TestKeys.LiveEnabled, "Set ROS2NET_APPGEN_LIVE=1 to run live LLM tests.");
        Assert.SkipWhen(credential is null, "No credentials for this provider in the key file.");
    }

    private static async Task<string> Ping(LlmProfile? profile)
    {
        SkipUnlessLive(profile);
        var host = TestHost.Create();
        return await host.Assistant.PingAsync(profile!, TestContext.Current.CancellationToken);
    }

    [Fact]
    public async Task Azure_openai_answers() => (await Ping(TestKeys.Azure())).ToLowerInvariant().ShouldContain("pong");

    [Fact]
    public async Task DeepSeek_answers() => (await Ping(TestKeys.DeepSeek())).ToLowerInvariant().ShouldContain("pong");

    [Fact]
    public async Task Claude_answers() => (await Ping(TestKeys.Claude())).ToLowerInvariant().ShouldContain("pong");

    [Fact]
    public async Task Tavily_search_returns_results()
    {
        var key = TestKeys.Tavily();
        SkipUnlessLive(key);
        var host = TestHost.Create();
        host.Settings.TavilyApiKey = key!;
        var web = new WebPlugin(TestHost.Tools(host));
        var result = await web.SearchInternetAsync("ROS 2 Jazzy release date", 3, TestContext.Current.CancellationToken);
        result.ShouldNotStartWith("Error");
        result.ShouldContain("http");
    }

    [Theory]
    [InlineData("claude")]
    [InlineData("azure")]
    [InlineData("deepseek")]
    public async Task Jack_uses_tools_and_answers(string provider)
    {
        var profile = provider switch { "claude" => TestKeys.Claude(), "azure" => TestKeys.Azure(), _ => TestKeys.DeepSeek() };
        SkipUnlessLive(profile);
        var host = TestHost.Create();
        host.Settings.Profiles = [profile!];
        host.Settings.ActiveProfile = profile!.Name;
        var tools = new List<string>();
        var text = new System.Text.StringBuilder();
        string? error = null;
        await foreach (var e in host.Assistant.SendAsync("Use the math tool to compute sqrt(2)*sin(rad(30)) and tell me the result with 6 decimals.", [], TestContext.Current.CancellationToken))
        {
            switch (e.Kind)
            {
                case AssistantEventKind.ToolStarted: tools.Add(e.ToolName!); break;
                case AssistantEventKind.TextDelta: text.Append(e.Text); break;
                case AssistantEventKind.Error: error = e.Text; break;
            }
        }

        error.ShouldBeNull();
        tools.ShouldContain("math.calculate");
        text.ToString().ShouldContain("0.707107");
    }

    /// <summary>End to end: Jack writes a talker/listener + AddTwoInts service into a blank project and it builds.</summary>
    [Theory]
    [InlineData("claude")]
    [InlineData("azure")]
    public async Task Jack_builds_a_ros2net_app(string provider)
    {
        var profile = provider == "claude" ? TestKeys.Claude() : TestKeys.Azure();
        SkipUnlessLive(profile);
        Assert.SkipUnless(Directory.Exists(TestHost.LocalFeed), $"Local package feed missing: run dotnet pack ROS2.Net.slnx -c Release -o {TestHost.LocalFeed}");

        var root = Path.Combine(Path.GetTempPath(), "ros2appgen-live-" + Guid.NewGuid().ToString("N")[..8]);
        Directory.CreateDirectory(root);
        var host = TestHost.Create();
        host.Settings.Profiles = [profile!];
        host.Settings.ActiveProfile = profile!.Name;
        host.Settings.PackageSource = TestHost.LocalFeed;
        var folder = host.Projects.CreateBlank(root, "TalkerService");
        host.Workspace.Open(folder);

        var tools = new List<string>();
        string? error = null;
        await foreach (var e in host.Assistant.SendAsync(
            "Make a ROS2.Net talker/listener app that also exposes an AddTwoInts service (example_interfaces) and calls it once at startup. Replace Program.cs, then build until it succeeds.",
            [], TestContext.Current.CancellationToken))
        {
            if (e.Kind == AssistantEventKind.ToolStarted)
            {
                tools.Add(e.ToolName!);
            }
            else if (e.Kind == AssistantEventKind.Error)
            {
                error = e.Text;
            }
        }

        error.ShouldBeNull();
        tools.ShouldContain("project.write_file");
        tools.ShouldContain("build.build_project");
        var program = File.ReadAllText(Path.Combine(folder, "Program.cs"));
        program.ShouldContain("AddTwoInts");
        var build = await host.Build.BuildAsync(folder, null, TestContext.Current.CancellationToken);
        build.Success.ShouldBeTrue(build.Output);
    }
}
