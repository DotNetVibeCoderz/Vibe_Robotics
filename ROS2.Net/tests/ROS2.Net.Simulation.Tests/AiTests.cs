using ROS2.Net.AI.Diagnostics;
using ROS2.Net.AI.Gym;
using ROS2.Net.AI.Llm;
using ROS2.Net.AI.Onnx;
using ROS2.Net.Msgs.GeometryMsgs;
using Shouldly;
using Pose2D = ROS2.Net.Simulation.Pose2D;
using StdString = ROS2.Net.Msgs.StdMsgs.String;

namespace ROS2.Net.Simulation.Tests;

public class AiTests
{
    [Fact]
    public void Gym_env_exposes_spaces_and_rewards_progress()
    {
        var env = new RobotGymEnv(new RobotGymOptions { Sectors = 8, Start = new Pose2D(0, 0, 0), RandomizeStartHeading = false, Goal = new Vec2(2, 0) });
        env.ObservationSpace.FlatSize.ShouldBe(11);
        env.ActionSpace.Count.ShouldBe(5);
        env.Reset();
        env.Observation[9].ShouldBe(1f, 1e-3f);   // cos(bearing): goal straight ahead
        env.Observation[8].ShouldBeGreaterThan(0f);

        var forward = env.Step(0);
        forward.Reward.ShouldBeGreaterThan(0);
        forward.Done.ShouldBeFalse();
        var spin = env.Step(3);
        spin.Reward.ShouldBeLessThan(forward.Reward);
        env.ElapsedSteps.ShouldBe(2);
    }

    [Fact]
    public void Gym_env_terminates_on_goal_and_collision()
    {
        var env = new RobotGymEnv(new RobotGymOptions { Sectors = 0, Start = new Pose2D(0, 0, 0), RandomizeStartHeading = false, Goal = new Vec2(0.5, 0), ActionCount = 3 });
        env.Reset();
        RLNet.Environments.StepResult r = default;
        for (var i = 0; i < 30 && !r.Done; i++)
        {
            r = env.Step(0);
        }

        r.Terminated.ShouldBeTrue();
        env.ReachedGoal.ShouldBeTrue();

        var walled = new RobotGymEnv(new RobotGymOptions { Scenario = ScenarioKind.CorridorLoop, Sectors = 8, Start = new Pose2D(1, 1, -Math.PI / 2), RandomizeStartHeading = false, Goal = new Vec2(12, 1) });
        walled.Reset();
        r = default;
        for (var i = 0; i < 50 && !r.Done; i++)
        {
            r = walled.Step(0); // straight into the south wall
        }

        walled.Collided.ShouldBeTrue();
        r.Reward.ShouldBeLessThan(-5);
    }

    [Fact]
    public void Q_learning_beats_a_random_policy_on_a_simple_goal()
    {
        var options = new RobotGymOptions
        {
            Sectors = 0, ActionCount = 3, Start = new Pose2D(0, 0, 0), RandomizeStartHeading = true, Goal = new Vec2(1.2, 0),
            MaxEpisodeSteps = 40, ControlPeriod = 0.25, Seed = 3,
        };
        var env = new RobotGymEnv(options);
        var agent = AgentFactory.CreateQTable(env, bins: 6, seed: 1);

        var rng = new Random(2);
        var random = RlTrainer.Evaluate(env, _ => rng.Next(3), 20, seed: 100).Average(s => s.Return);
        var history = RlTrainer.Train(env, agent, 250, seed: 0);
        var trained = RlTrainer.Evaluate(env, agent, 20, seed: 100);

        history.Count.ShouldBe(250);
        trained.Average(s => s.Return).ShouldBeGreaterThan(random);
        trained.Count(s => s.Terminated).ShouldBeGreaterThanOrEqualTo(12);
    }

    [Fact]
    public void Onnx_runner_executes_a_tiny_model()
    {
        using var runner = new OnnxModelRunner(TinyOnnx.ReluModel());
        runner.InputName.ShouldBe("X");
        runner.Run([-1f, 2f, -3f]).ShouldBe(new[] { 0f, 2f, 0f });
        using var policy = new OnnxPolicy(new OnnxModelRunner(TinyOnnx.ReluModel()));
        policy.SelectAction([0.1f, -5f, 0.7f]).ShouldBe(2);
    }

    private sealed class FakeModel(params ToolCall[] calls) : IChatModel
    {
        public List<IReadOnlyList<ChatMessage>> Requests { get; } = [];

        public Task<ChatResult> CompleteAsync(IReadOnlyList<ChatMessage> messages, IReadOnlyList<ToolDefinition> tools, CancellationToken cancellationToken = default)
        {
            Requests.Add(messages);
            tools.Select(t => t.Name).ShouldBe(new[] { "move_forward", "rotate", "go_to_waypoint", "stop" });
            return Task.FromResult(new ChatResult(null, calls));
        }
    }

    [Fact]
    public async Task Commander_turns_tool_calls_into_limited_twists()
    {
        await using var ctx = Ctx.Create();
        var model = new FakeModel(new ToolCall("1", "move_forward", """{"distance_m":0.1,"speed_mps":5}"""));
        using var commander = new NaturalLanguageCommanderNode(ctx, model, new CommanderOptions { ScanTopic = null });
        var observer = ctx.CreateNode("observer");
        var twists = observer.CreateSubscription<Twist>("/cmd_vel", QosProfile.Default.KeepLast(100));

        var feedback = await commander.HandleCommandAsync("go forward 10 cm quickly");

        feedback.ShouldContain("moved 0.1 m");
        twists.TryTake(out var first).ShouldBeTrue();
        first!.Linear.X.ShouldBe(0.3); // clamped by the safety limiter
        Twist? last = first;
        while (twists.TryTake(out var t))
        {
            last = t;
        }

        last!.Linear.X.ShouldBe(0); // always ends stopped
        model.Requests.Single()[0].Role.ShouldBe("system");
    }

    [Fact]
    public async Task Commander_reads_commands_from_the_topic_and_publishes_waypoint_goals()
    {
        await using var ctx = Ctx.Create();
        using var commander = new NaturalLanguageCommanderNode(ctx, new RuleBasedChatModel(), new CommanderOptions
        {
            ScanTopic = null,
            Waypoints = { ["dock"] = (3, 4, 0) },
        });
        var node = ctx.CreateNode("user");
        var goals = node.CreateSubscription<PoseStamped>("/goal_pose");
        var feedback = node.CreateSubscription<StdString>("/nl_feedback");
        node.CreatePublisher<StdString>("/nl_command").Publish(new StdString { Data = "pergi ke dock" });

        var goal = await goals.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(5));
        goal.Pose.Position.X.ShouldBe(3);
        (await feedback.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(5))).Data.ShouldBe("navigating to dock");
    }

    [Theory]
    [InlineData("move forward 1.5 m", "move_forward", "distance_m", 1.5)]
    [InlineData("maju 50 cm", "move_forward", "distance_m", 0.5)]
    [InlineData("mundur 2 meter", "move_forward", "distance_m", -2)]
    [InlineData("turn left 90 degrees", "rotate", "angle_deg", 90)]
    [InlineData("belok kanan 45 derajat", "rotate", "angle_deg", -45)]
    public async Task Rule_based_model_understands_english_and_indonesian(string text, string tool, string arg, double value)
    {
        var result = await new RuleBasedChatModel().CompleteAsync([ChatMessage.User(text)], NaturalLanguageCommanderNode.Tools);
        var call = result.ToolCalls.ShouldHaveSingleItem();
        call.Name.ShouldBe(tool);
        call.GetDouble(arg).ShouldBe(value, 1e-9);
    }

    [Fact]
    public async Task Rule_based_model_splits_sequences()
    {
        var result = await new RuleBasedChatModel().CompleteAsync([ChatMessage.User("forward 1 m then turn right 90, then stop")], []);
        result.ToolCalls.Select(c => c.Name).ShouldBe(new[] { "move_forward", "rotate", "stop" });
    }

    [Fact]
    public void OpenAI_compatible_payloads_are_well_formed()
    {
        var model = new OpenAICompatibleChatModel(new OpenAICompatibleOptions { Model = "test-model", ApiKey = null });
        var json = model.BuildRequest([ChatMessage.System("s"), ChatMessage.User("hi")], NaturalLanguageCommanderNode.Tools);
        using (var doc = System.Text.Json.JsonDocument.Parse(json))
        {
            doc.RootElement.GetProperty("model").GetString().ShouldBe("test-model");
            doc.RootElement.GetProperty("tools").GetArrayLength().ShouldBe(4);
            doc.RootElement.GetProperty("tools")[0].GetProperty("function").GetProperty("parameters").GetProperty("type").GetString().ShouldBe("object");
        }

        var parsed = OpenAICompatibleChatModel.ParseResponse("""
            {"choices":[{"message":{"role":"assistant","content":null,"tool_calls":[
              {"id":"call_1","type":"function","function":{"name":"rotate","arguments":"{\"angle_deg\": 30}"}}]}}]}
            """);
        parsed.ToolCalls.Single().GetDouble("angle_deg").ShouldBe(30);
    }

    [Fact]
    public void Anomaly_detectors_flag_spikes_but_not_noise()
    {
        var ewma = new EwmaAnomalyDetector(alpha: 0.1, threshold: 5);
        var rolling = new RollingZScoreDetector(window: 50, threshold: 5);
        var rng = new SimRandom(4);
        for (var i = 0; i < 300; i++)
        {
            var v = 10 + rng.Gaussian(0.1);
            ewma.Update(v).IsAnomaly.ShouldBeFalse();
            rolling.Update(v).IsAnomaly.ShouldBeFalse();
        }

        ewma.Update(15).IsAnomaly.ShouldBeTrue();
        rolling.Update(15).IsAnomaly.ShouldBeTrue();
    }

    [Fact]
    public async Task Telemetry_monitor_publishes_diagnostics()
    {
        await using var ctx = Ctx.Create();
        var node = ctx.CreateNode("monitor");
        using var monitor = new TelemetryAnomalyMonitor<Msgs.SensorMsgs.BatteryState>(node, "/battery_state", [("current", b => b.Current)], () => new EwmaAnomalyDetector(warmup: 5));
        var diags = node.CreateSubscription<Msgs.DiagnosticMsgs.DiagnosticArray>("/diagnostics", QosProfile.Default.KeepLast(100));
        var anomalies = 0;
        monitor.AnomalyDetected += (_, _) => anomalies++;
        for (var i = 0; i < 20; i++)
        {
            monitor.OnMessage(new Msgs.SensorMsgs.BatteryState { Current = -0.5f + (i % 2 * 0.01f) });
        }

        monitor.OnMessage(new Msgs.SensorMsgs.BatteryState { Current = -8f });
        anomalies.ShouldBe(1);
        Msgs.DiagnosticMsgs.DiagnosticArray? last = null;
        while (diags.TryTake(out var d))
        {
            last = d;
        }

        last!.Status[0].Level.ShouldBe(Msgs.DiagnosticMsgs.DiagnosticStatus.WARN);
    }
}

/// <summary>Hand-encodes a minimal ONNX model (Y = Relu(X), X: float[1,3]) as protobuf, so tests need no model files.</summary>
internal static class TinyOnnx
{
    public static byte[] ReluModel()
    {
        var shape = Msg(Bytes(1, Msg(Varint(1, 1))), Bytes(1, Msg(Varint(1, 3))));
        var tensorType = Msg(Varint(1, 1), Bytes(2, shape));
        var type = Msg(Bytes(1, tensorType));
        byte[] ValueInfo(string name) => Msg(Str(1, name), Bytes(2, type));
        var node = Msg(Str(1, "X"), Str(2, "Y"), Str(4, "Relu"));
        var graph = Msg(Bytes(1, node), Str(2, "g"), Bytes(11, ValueInfo("X")), Bytes(12, ValueInfo("Y")));
        var opset = Msg(Str(1, string.Empty), Varint(2, 13));
        return Msg(Varint(1, 8), Bytes(7, graph), Bytes(8, opset));
    }

    private static byte[] Msg(params byte[][] fields) => fields.SelectMany(f => f).ToArray();

    private static byte[] Varint(int field, ulong value) => [.. Key(field, 0), .. EncodeVarint(value)];

    private static byte[] Str(int field, string value) => Bytes(field, System.Text.Encoding.UTF8.GetBytes(value));

    private static byte[] Bytes(int field, byte[] payload) => [.. Key(field, 2), .. EncodeVarint((ulong)payload.Length), .. payload];

    private static byte[] Key(int field, int wire) => EncodeVarint((ulong)((field << 3) | wire));

    private static byte[] EncodeVarint(ulong v)
    {
        var list = new List<byte>();
        do
        {
            var b = (byte)(v & 0x7F);
            v >>= 7;
            if (v != 0)
            {
                b |= 0x80;
            }

            list.Add(b);
        }
        while (v != 0);

        return [.. list];
    }
}
