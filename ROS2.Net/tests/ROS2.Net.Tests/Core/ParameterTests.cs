using Shouldly;

namespace ROS2.Net.Tests.Core;

public class ParameterTests
{
    [Fact]
    public async Task Command_line_overrides_win_over_defaults()
    {
        await using var ctx = TestDomains.Create(args: ["--ros-args", "-p", "speed:=2.5", "-p", "other:name:=x", "-r", "__ns:=/fleet"]);
        var node = ctx.CreateNode("robot");
        node.FullyQualifiedName.ShouldBe("/fleet/robot");
        node.DeclareParameter("speed", 1.0).ShouldBe(2.5);
        node.DeclareParameter("name", "default").ShouldBe("default");
    }

    [Fact]
    public async Task Integer_override_is_accepted_for_a_double_parameter()
    {
        await using var ctx = TestDomains.Create(args: ["--ros-args", "-p", "gain:=3"]);
        var node = ctx.CreateNode("n");
        node.DeclareParameter("gain", 1.0).ShouldBe(3.0);
    }

    [Fact]
    public async Task Ranges_types_read_only_and_validators_are_enforced()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        node.DeclareParameter("rate", 10L, new ParameterDescriptor { IntegerRange = new IntegerRange(1, 100, 1) });
        node.DeclareParameter("id", "r1", readOnly: true);
        node.DeclareParameter("mode", "auto");

        node.SetParameter("rate", 500L).Successful.ShouldBeFalse();
        node.SetParameter("rate", "fast").Successful.ShouldBeFalse();
        node.SetParameter("id", "r2").Successful.ShouldBeFalse();
        node.SetParameter("missing", 1).Successful.ShouldBeFalse();

        using var _ = node.AddOnSetParametersCallback(ps =>
            ps.Any(p => p.Name == "mode" && p.Value.AsString() == "manual") ? SetParametersResult.Failure("manual is disabled") : SetParametersResult.Success);
        node.SetParameter("mode", "manual").Reason.ShouldBe("manual is disabled");

        node.SetParameter("rate", 50L).Successful.ShouldBeTrue();
        node.GetParameter<int>("rate").ShouldBe(50);
    }

    [Fact]
    public async Task Atomic_set_applies_all_or_nothing()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        node.DeclareParameter("a", 1L);
        node.DeclareParameter("b", 1L);
        node.SetParametersAtomically([new Parameter("a", 2L), new Parameter("b", "bad")]).Successful.ShouldBeFalse();
        node.GetParameter<long>("a").ShouldBe(1);
    }

    [Fact]
    public async Task Changes_raise_events()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        node.DeclareParameter("x", 1.0);
        ParameterChange? change = null;
        node.ParametersChanged += cs => change = cs.Single();
        node.SetParameter("x", 2.0);
        change!.OldValue!.AsDouble().ShouldBe(1.0);
        change.NewValue.AsDouble().ShouldBe(2.0);
    }

    [Fact]
    public void Yaml_files_follow_the_ros2_layout()
    {
        const string yaml = """
            /**:
              ros__parameters:
                use_sim_time: true
            /robot1/controller:
              ros__parameters:
                gains: { kp: 1.2, ki: 0 }
                joints: [a, b]
                limits: [1, 2.5]
                name: "ctl"
            other:
              ros__parameters:
                x: 1
            """;

        var ps = ParameterYaml.Load(yaml, "/robot1/controller").ToDictionary(p => p.Name, p => p.Value);
        ps["use_sim_time"].AsBool().ShouldBeTrue();
        ps["gains.kp"].AsDouble().ShouldBe(1.2);
        ps["gains.ki"].AsInteger().ShouldBe(0);
        ps["joints"].AsStringArray().ShouldBe(new[] { "a", "b" });
        ps["limits"].AsDoubleArray().ShouldBe(new[] { 1.0, 2.5 });
        ps["name"].AsString().ShouldBe("ctl");
        ps.ContainsKey("x").ShouldBeFalse();

        ParameterYaml.Load(yaml, "/other").Single(p => p.Name == "x").Value.AsInteger().ShouldBe(1);
    }

    [Fact]
    public async Task Remote_parameter_client_reads_and_writes()
    {
        await using var ctx = TestDomains.Create();
        var target = ctx.CreateNode("target");
        target.DeclareParameter("speed", 1.0);
        var tool = ctx.CreateNode("tool");
        using var client = new ParameterClient(tool, "/target");

        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);

        (await client.WaitForServiceAsync(TimeSpan.FromSeconds(2))).ShouldBeTrue();
        (await client.GetAsync("speed")).AsDouble().ShouldBe(1.0);
        (await client.SetAsync("speed", 3.0)).Successful.ShouldBeTrue();
        target.GetParameter<double>("speed").ShouldBe(3.0);
        (await client.ListAsync()).ShouldContain("speed");
        (await client.DescribeAsync(["speed"]))[0].Type.ShouldBe(ParameterType.Double);

        cts.Cancel();
        await spin;
    }
}
