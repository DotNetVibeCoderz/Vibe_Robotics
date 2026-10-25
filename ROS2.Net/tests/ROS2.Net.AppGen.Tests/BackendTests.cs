using Microsoft.SemanticKernel;
using ROS2.Net.AppGen.Ai;
using ROS2.Net.AppGen.Plugins;
using Shouldly;

namespace ROS2.Net.AppGen.Tests;

public sealed class SettingsTests
{
    [Fact]
    public void Settings_round_trip_through_app_config()
    {
        var dir = TestHost.TempFolder();
        var path = Path.Combine(dir, "App.config");
        var store = new AppConfigSettingsStore(path);

        var first = store.Load(); // creates the file with defaults
        File.Exists(path).ShouldBeTrue();
        first.Profiles.Select(p => p.Provider).ShouldBe([LlmProviderKind.Claude, LlmProviderKind.OpenAI, LlmProviderKind.AzureOpenAI, LlmProviderKind.Gemini, LlmProviderKind.Ollama, LlmProviderKind.DeepSeek], ignoreOrder: true);
        first.Profiles.ShouldAllBe(p => p.ApiKey == string.Empty);
        first.RunTargets.Count.ShouldBe(4);

        first.ActiveProfile = "DeepSeek";
        first.Profiles[0].ApiKey = "env:MY_KEY";
        first.Profiles[0].ExtraHeaders = "anthropic-workspace-id: w1";
        first.SystemPrompt = "custom \"prompt\" <xml>";
        first.EditorFontSize = 15.5;
        first.ShowLineNumbers = false;
        first.RecentProjects = ["C:/a", "C:/b"];
        first.RunTargets.Add(new RunTarget { Name = "Lab TurtleBot", Kind = RunTargetKind.PhysicalRobot, Rmw = "rosbridge", RosbridgeUrl = "ws://10.0.0.5:9090", RobotNamespace = "tb3", ExtraEnvironment = { ["FOO"] = "bar" } });
        first.ActiveRunTarget = "Lab TurtleBot";
        store.Save(first);

        var second = new AppConfigSettingsStore(path).Load();
        second.ActiveProfile.ShouldBe("DeepSeek");
        second.Profiles[0].ApiKey.ShouldBe("env:MY_KEY");
        second.Profiles[0].ExtraHeaders.ShouldBe("anthropic-workspace-id: w1");
        second.SystemPrompt.ShouldBe("custom \"prompt\" <xml>");
        second.EditorFontSize.ShouldBe(15.5);
        second.ShowLineNumbers.ShouldBeFalse();
        second.RecentProjects.ShouldBe(["C:/a", "C:/b"]);
        second.RunTargets.Single(t => t.Name == "Lab TurtleBot").ExtraEnvironment["FOO"].ShouldBe("bar");
        second.ActiveRunTarget.ShouldBe("Lab TurtleBot");
    }

    [Fact]
    public void Template_config_contains_no_secrets_and_is_valid_xml()
    {
        var xml = DefaultSettings.ConfigTemplateXml();
        var doc = System.Xml.Linq.XDocument.Parse(xml);
        doc.Root!.Name.LocalName.ShouldBe("configuration");
        xml.ShouldNotContain("sk-");
        xml.ShouldContain("AppGen.Profiles");
    }

    [Fact]
    public void Secrets_resolve_from_environment_and_mask()
    {
        Environment.SetEnvironmentVariable("ROS2APPGEN_TEST_SECRET", "abc123secret");
        AppConfigSettingsStore.ResolveSecret("env:ROS2APPGEN_TEST_SECRET").ShouldBe("abc123secret");
        AppConfigSettingsStore.Mask("abc123secret").ShouldBe("abc1********");
        AppConfigSettingsStore.Mask("").ShouldBe("(empty)");
    }

    [Fact]
    public void Profile_validation_explains_missing_pieces()
    {
        Should.Throw<LlmConfigurationException>(() => KernelFactory.Validate(new LlmProfile { Name = "x", Provider = LlmProviderKind.Claude, Model = "claude-sonnet-5-5" })).Message.ShouldContain("API key");
        Should.Throw<LlmConfigurationException>(() => KernelFactory.Validate(new LlmProfile { Name = "x", Provider = LlmProviderKind.AzureOpenAI, Model = "m", ApiKey = "k", Endpoint = "https://<resource>.openai.azure.com/" })).Message.ShouldContain("endpoint");
        KernelFactory.Validate(new LlmProfile { Name = "local", Provider = LlmProviderKind.Ollama, Model = "llama3.2" }); // no key needed
        KernelFactory.ParseHeaders("a: 1; b: two\nc:3").Select(h => h.Key).ShouldBe(["a", "b", "c"]);
    }
}

public sealed class WorkspaceAndPluginTests
{
    private static (AppGenHost Host, FileSystemWorkspace Ws, ToolContext Tools) Open()
    {
        var host = TestHost.Create();
        var dir = TestHost.TempFolder();
        File.WriteAllText(Path.Combine(dir, "Demo.csproj"), "<Project Sdk=\"Microsoft.NET.Sdk\"><PropertyGroup><TargetFramework>net10.0</TargetFramework><OutputType>Exe</OutputType></PropertyGroup><ItemGroup><PackageReference Include=\"ROS2.Net.Core\" Version=\"0.1.0\" /></ItemGroup></Project>");
        host.Workspace.Open(dir);
        return (host, (FileSystemWorkspace)host.Workspace, TestHost.Tools(host));
    }

    [Theory]
    [InlineData("../outside.txt")]
    [InlineData("..\\..\\x.cs")]
    [InlineData("sub/../../x.cs")]
    [InlineData("C:/Windows/win.ini")]
    public void Path_traversal_is_rejected(string path)
    {
        var (_, ws, _) = Open();
        Should.Throw<UnauthorizedAccessException>(() => ws.WriteFile(path, "x"));
    }

    [Fact]
    public void Project_plugin_writes_edits_searches_and_describes()
    {
        var (_, ws, tools) = Open();
        var p = new ProjectPlugin(tools);
        p.WriteFile("src/A.cs", "class A\n{\n    int x = 1;\n}\n").ShouldStartWith("Created");
        p.ReplaceInFile("src/A.cs", "int x = 1;", "int x = 2;").ShouldStartWith("Replaced 1");
        p.ReplaceInFile("src/A.cs", "nope", "x").ShouldStartWith("Error");
        p.ReadFile("src/A.cs", 3, 3).ShouldContain("3:     int x = 2;");
        p.SearchInFiles("int x", "*.cs").ShouldContain("src/A.cs:3:");
        p.ListFiles().ShouldContain("src/A.cs");
        p.GetProjectInfo().ShouldContain("package ROS2.Net.Core 0.1.0");
        p.WriteFile("src/A.cs", "class A { }").ShouldStartWith("Updated src/A.cs");
        p.RenamePath("src/A.cs", "src/B.cs");
        File.Exists(ws.Resolve("src/B.cs")).ShouldBeTrue();
        ws.ActiveSelection = "selected";
        p.GetSelection().ShouldBe("selected");
    }

    [Fact]
    public async Task Plugins_run_inside_a_kernel_without_an_llm()
    {
        var (_, _, tools) = Open();
        var kernel = new Kernel();
        kernel.Plugins.AddFromObject(new MathPlugin(), "math");
        kernel.Plugins.AddFromObject(new ProjectPlugin(tools), "project");
        kernel.Plugins.AddFromObject(new Ros2NetKnowledgePlugin(), "ros2net");
        kernel.Plugins.AddFromObject(new BuildPlugin(tools), "build");

        (await kernel.InvokeAsync("math", "calculate", new() { ["expression"] = "2^10 + sqrt(16)" })).GetValue<string>().ShouldBe("1028");
        (await kernel.InvokeAsync("project", "write_file", new() { ["path"] = "x.txt", ["content"] = "hi" })).GetValue<string>()!.ShouldStartWith("Created");
        (await kernel.InvokeAsync("ros2net", "get_api_reference", new() { ["topic"] = "simulation" })).GetValue<string>()!.ShouldContain("SimulationNode");
        (await kernel.InvokeAsync("ros2net", "get_type_api", new() { ["typeName"] = "SimWorld" })).GetValue<string>()!.ShouldContain("StepFor");
        (await kernel.InvokeAsync("ros2net", "get_message_type", new() { ["rosType"] = "sensor_msgs/msg/LaserScan" })).GetValue<string>()!.ShouldContain("Single[] Ranges".Replace("Single", "float"));
        (await kernel.InvokeAsync("ros2net", "get_message_type", new() { ["rosType"] = "std_srvs/srv/SetBool" })).GetValue<string>()!.ShouldContain("SetBoolRequest");
        (await kernel.InvokeAsync("build", "list_templates", new() { ["filter"] = "Industry" })).GetValue<string>()!.ShouldContain("conveyor-plc-bridge");
        (await kernel.InvokeAsync("build", "get_run_targets")).GetValue<string>()!.ShouldContain("ROS2NET_TARGET=robot");
    }

    [Fact]
    public void Tool_event_arguments_are_summarized()
    {
        var args = new KernelArguments { ["path"] = "Program.cs", ["content"] = new string('x', 500) };
        ToolEventFilter.Summarize(args).ShouldBe("path=Program.cs, content=" + new string('x', 60) + "…");
    }
}

public sealed class MathAndTimeTests
{
    [Theory]
    [InlineData("1 + 2 * 3", 7)]
    [InlineData("(1 + 2) * 3", 9)]
    [InlineData("2^3^2", 512)]
    [InlineData("-2^2", -4)]
    [InlineData("10 % 4", 2)]
    [InlineData("sqrt(2) * sin(rad(45))", 1.0)]
    [InlineData("max(1, 5, 3) - min(4, 2)", 3)]
    [InlineData("round(pi, 3)", 3.142)]
    [InlineData("log(100) + ln(e)", 3)]
    [InlineData("1.5e3 / 2", 750)]
    [InlineData("hypot(3, 4)", 5)]
    public void Evaluates_expressions(string expression, double expected) =>
        ExpressionEvaluator.Evaluate(expression).ShouldBe(expected, 1e-9);

    [Theory]
    [InlineData("1 +")]
    [InlineData("foo(1)")]
    [InlineData("(1")]
    [InlineData("2 $ 3")]
    public void Rejects_invalid_expressions(string expression) =>
        Should.Throw<FormatException>(() => ExpressionEvaluator.Evaluate(expression));

    [Fact]
    public void Math_plugin_reports_errors_and_converts_units()
    {
        var m = new MathPlugin();
        m.Calculate("1/").ShouldStartWith("Error");
        m.ConvertUnits(90, "deg", "rad").ShouldStartWith("1.5707963267");
        m.ConvertUnits(36, "km/h", "m/s").ShouldBe("10 m/s");
        m.ConvertUnits(1, "m", "kg").ShouldStartWith("Error");
    }

    [Fact]
    public void Date_time_functions_work_with_a_fixed_clock()
    {
        var clock = new FixedTime(new DateTimeOffset(2026, 10, 25, 1, 30, 0, TimeSpan.Zero));
        var t = new DateTimePlugin(clock);
        t.Now().ShouldContain("utc: 2026-10-25 01:30:00Z");
        t.DateDiff("2026-10-25", "2026-10-27T12:00:00Z").ShouldStartWith("2.5 days");
        t.AddDuration("2026-10-25T00:00:00Z", "1d2h30m").ShouldStartWith("2026-10-26 02:30:00");
        t.AddDuration("now", "-90m").ShouldStartWith("2026-10-25 00:00:00");
        DateTimePlugin.TryParseDuration("3x", out _).ShouldBeFalse();
        t.ConvertTimeZone("2026-10-25 08:00", "UTC", "UTC").ShouldStartWith("2026-10-25 08:00:00");
    }

    private sealed class FixedTime(DateTimeOffset now) : TimeProvider
    {
        public override DateTimeOffset GetUtcNow() => now;

        public override TimeZoneInfo LocalTimeZone => TimeZoneInfo.Utc;
    }
}

public sealed class FormatterTests
{
    private readonly CodeFormatter _f = new();

    [Fact]
    public void Formats_csharp_json_and_xml()
    {
        _f.Format("A.cs", "class A{void M(){int x=1;}}").ShouldContain("int x = 1;");
        _f.Format("a.json", "{\"a\":1,\"b\":[1,2]}").ShouldContain("\n  \"a\": 1");
        var xml = _f.Format("p.csproj", "<Project><PropertyGroup><A>1</A></PropertyGroup></Project>");
        xml.ShouldContain("\n  <PropertyGroup>");
        _f.CanFormat("x.razor").ShouldBeTrue();
        _f.CanFormat("x.png").ShouldBeFalse();
    }
}

public sealed class RunTargetTests
{
    [Fact]
    public void Simulator_target_maps_to_environment()
    {
        var env = RunTargetEnvironment.ToEnvironment(RunTargetEnvironment.DefaultSimulator);
        env[RunTargetEnvironment.Target].ShouldBe("sim");
        env[RunTargetEnvironment.Rmw].ShouldBe("inprocess");
        env[RunTargetEnvironment.LocalhostOnly].ShouldBe("0");
        env.ShouldNotContainKey(RunTargetEnvironment.RobotNamespace);
        RunTargetEnvironment.ToArguments(RunTargetEnvironment.DefaultSimulator).ShouldBeEmpty();
    }

    [Fact]
    public void Physical_robot_target_maps_to_environment_and_namespace_arguments()
    {
        var target = new RunTarget
        {
            Name = "tb3",
            Kind = RunTargetKind.PhysicalRobot,
            Rmw = "rosbridge",
            RosbridgeUrl = "ws://10.0.0.5:9090",
            DomainId = 7,
            RobotNamespace = "tb3/",
            NetworkInterface = "10.0.0.2",
            LocalhostOnly = false,
            Scenario = "Warehouse",          // ignored for robots
            MaxLinearSpeed = 0.25,
            ExtraEnvironment = { ["X"] = "1" },
        };
        var env = RunTargetEnvironment.ToEnvironment(target);
        env[RunTargetEnvironment.Target].ShouldBe("robot");
        env[RunTargetEnvironment.Rmw].ShouldBe("rosbridge");
        env[RunTargetEnvironment.RosbridgeUrl].ShouldBe("ws://10.0.0.5:9090");
        env[RunTargetEnvironment.DomainId].ShouldBe("7");
        env[RunTargetEnvironment.RobotNamespace].ShouldBe("/tb3");
        env[RunTargetEnvironment.NetworkInterface].ShouldBe("10.0.0.2");
        env[RunTargetEnvironment.MaxLinear].ShouldBe("0.25");
        env.ShouldNotContainKey(RunTargetEnvironment.Scenario);
        env["X"].ShouldBe("1");
        RunTargetEnvironment.ToArguments(target).ShouldBe(["--ros-args", "-r", "__ns:=/tb3"]);
    }

    [Fact]
    public void Physical_robot_never_uses_the_in_process_middleware()
    {
        var env = RunTargetEnvironment.ToEnvironment(new RunTarget { Kind = RunTargetKind.PhysicalRobot, Rmw = "inprocess" });
        env[RunTargetEnvironment.Rmw].ShouldBe("rtps");
    }

    [Fact]
    public void Every_project_gets_the_app_target_helper()
    {
        var host = TestHost.Create();
        var projects = (ProjectService)host.Projects;
        foreach (var kind in new[] { "console", "avalonia", "blazor" })
        {
            var folder = projects.CreateBlank(TestHost.TempFolder(), "Blank" + kind, kind);
            File.ReadAllText(Path.Combine(folder, "AppTarget.cs")).ShouldContain("ROS2NET_TARGET");
            File.ReadAllText(Path.Combine(folder, "Blank" + kind + ".csproj")).ShouldContain("ROS2.Net.Rmw.Rtps");
        }

        foreach (var t in projects.Templates)
        {
            projects.RenderTemplate(t.Id, "T").ShouldContainKey("AppTarget.cs");
        }
    }
}

public sealed class TemplateCatalogTests
{
    [Fact]
    public void Catalog_has_at_least_30_templates_across_categories()
    {
        var host = TestHost.Create();
        host.Projects.Templates.Count.ShouldBeGreaterThanOrEqualTo(30);
        host.Projects.Categories.ShouldBe(
            ["Agriculture", "Disaster & Safety", "Education", "Entertainment", "Healthcare", "Housekeeping", "Industry", "Logistics", "Office", "Science", "Simulator", "Tools", "Work"],
            ignoreOrder: true);
        host.Projects.Templates.Select(t => t.AppKind).Distinct().ShouldBe(["console", "avalonia", "blazor"], ignoreOrder: true);
    }

    [Fact]
    public void Rendering_substitutes_placeholders_and_writes_nuget_config_for_a_local_feed()
    {
        var host = TestHost.Create();
        host.Settings.PackageSource = TestHost.TempFolder();
        host.Settings.Ros2NetVersion = "9.9.9";
        var files = ((ProjectService)host.Projects).RenderTemplate("sim-desktop-dashboard", "my-robot.app");
        files.Keys.ShouldContain("my-robot.app.csproj");
        files["MainWindow.cs"].ShouldContain("namespace My_robot.App;");
        files["my-robot.app.csproj"].ShouldContain("Version=\"9.9.9\"");
        files["nuget.config"].ShouldContain("ros2net-local");
        files.Values.ShouldAllBe(v => !v.Contains("{{"));
    }

    [Fact]
    public void Invalid_project_names_are_rejected()
    {
        var host = TestHost.Create();
        Should.Throw<ArgumentException>(() => host.Projects.CreateBlank(TestHost.TempFolder(), "../evil"));
        Should.Throw<ArgumentException>(() => host.Projects.CreateBlank(TestHost.TempFolder(), "1abc"));
    }

    public static TheoryData<string> TemplateIds()
    {
        var data = new TheoryData<string>();
        foreach (var t in TestHost.Create().Projects.Templates)
        {
            data.Add(t.Id);
        }

        return data;
    }

    /// <summary>Every template restores and builds against the local package feed. Slow: opt-in with ROS2NET_APPGEN_TEMPLATES=1.</summary>
    [Theory]
    [MemberData(nameof(TemplateIds))]
    [Trait("Category", "TemplateBuild")]
    public async Task Template_builds(string templateId)
    {
        Assert.SkipUnless(Environment.GetEnvironmentVariable("ROS2NET_APPGEN_TEMPLATES") == "1", "Set ROS2NET_APPGEN_TEMPLATES=1 (and pack the local feed) to build every template.");
        Assert.SkipUnless(Directory.Exists(TestHost.LocalFeed), "Local feed missing: dotnet pack the src projects into artifacts/packages.");
        var host = TestHost.Create();
        host.Settings.PackageSource = TestHost.LocalFeed;
        var folder = host.Projects.CreateFromTemplate(templateId, TestHost.TempFolder(), "App" + templateId.Replace("-", string.Empty, StringComparison.Ordinal));
        var result = await host.Build.BuildAsync(folder, null, TestContext.Current.CancellationToken);
        result.Success.ShouldBeTrue(string.Join('\n', BuildService.ParseDiagnostics(result.Output).Take(15)) + "\n" + BuildPlugin.Tail(result.Output, 20));
    }

    /// <summary>A generated app actually starts in simulator mode (the default Run target) and drives the simulated robot.</summary>
    [Fact]
    [Trait("Category", "TemplateBuild")]
    public async Task Blank_app_starts_in_simulator_mode()
    {
        Assert.SkipUnless(Environment.GetEnvironmentVariable("ROS2NET_APPGEN_TEMPLATES") == "1", "Set ROS2NET_APPGEN_TEMPLATES=1 to build and run a generated app.");
        Assert.SkipUnless(Directory.Exists(TestHost.LocalFeed), "Local feed missing.");
        var host = TestHost.Create();
        host.Settings.PackageSource = TestHost.LocalFeed;
        var folder = host.Projects.CreateBlank(TestHost.TempFolder(), "SimCheck");
        var build = (BuildService)host.Build;
        (await build.BuildAsync(folder, null, TestContext.Current.CancellationToken)).Success.ShouldBeTrue();

        var run = await build.RunBrieflyAsync(folder, TimeSpan.FromSeconds(25), null, TestContext.Current.CancellationToken);
        run.Output.ShouldContain("target: simulator (inprocess)");
        run.Output.ShouldContain("I heard: Hello from SimCheck");
        run.Output.ShouldContain("robot at (");

        var robot = new RunTarget { Name = "robot", Kind = RunTargetKind.PhysicalRobot, Rmw = "rtps", LocalhostOnly = true, DomainId = 77 };
        var robotRun = await build.RunBrieflyAsync(folder, TimeSpan.FromSeconds(20), null, TestContext.Current.CancellationToken, target: robot);
        robotRun.Output.ShouldContain("target: PHYSICAL ROBOT via rtps");
        robotRun.Output.ShouldNotContain("robot at (");   // no simulator, no robot on domain 77
    }
}
