using ROS2.Net.Msgs.ExampleInterfaces;
using ROS2.Net.Rmw.InProcess;
using Shouldly;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Cli.Tests;

/// <summary>Runs CLI commands in-process against the in-process middleware; each test uses its own domain.</summary>
public class CliTests
{
    private static int s_domain = 150;

    private static int NextDomain() => Interlocked.Increment(ref s_domain);

    private static async Task<(int Code, string Output, string Error)> Run(int domain, params string[] args)
    {
        var output = new StringWriter();
        var error = new StringWriter();
        var code = await CliApp.RunAsync([.. args, "--rmw", "inprocess", "--domain", domain.ToString()], output, error);
        return (code, output.ToString(), error.ToString());
    }

    private static Ros2Context Context(int domain) =>
        Ros2Context.Create(new Ros2ContextOptions { DomainId = domain, Rmw = new InProcessRmwFactory() });

    [Fact]
    public async Task Topic_and_node_list_see_nodes_in_the_same_process()
    {
        var domain = NextDomain();
        await using var ctx = Context(domain);
        var node = ctx.CreateNode("talker", "/demo");
        node.CreatePublisher<StdMsgs.String>("chatter");

        var (code, topics, _) = await Run(domain, "topic", "list", "-t");
        code.ShouldBe(0);
        topics.ShouldContain("/demo/chatter [std_msgs/msg/String]");

        var (_, nodes, _) = await Run(domain, "node", "list");
        nodes.ShouldContain("/demo/talker");
        nodes.ShouldNotContain("_ros2net_cli");

        var (_, info, _) = await Run(domain, "topic", "info", "/demo/chatter");
        info.ShouldContain("Type: std_msgs/msg/String");
        info.ShouldContain("Publisher count: 1");

        var (_, nodeInfo, _) = await Run(domain, "node", "info", "/demo/talker");
        nodeInfo.ShouldContain("/demo/chatter: std_msgs/msg/String");
        nodeInfo.ShouldContain("/demo/talker/get_parameters: rcl_interfaces/srv/GetParameters");
    }

    [Fact]
    public async Task Topic_pub_once_reaches_a_subscription()
    {
        var domain = NextDomain();
        await using var ctx = Context(domain);
        var sub = ctx.CreateNode("listener").CreateSubscription<StdMsgs.String>("/chatter");

        var (code, output, error) = await Run(domain, "topic", "pub", "/chatter", "std_msgs/msg/String", "{data: hello cli}", "--once");
        code.ShouldBe(0, error);
        output.ShouldContain("publishing #1");
        sub.TryTake(out var msg).ShouldBeTrue();
        msg!.Data.ShouldBe("hello cli");
    }

    [Fact]
    public async Task Topic_echo_once_prints_yaml_and_json()
    {
        var domain = NextDomain();
        await using var ctx = Context(domain);
        var node = ctx.CreateNode("talker");
        var pub = node.CreatePublisher<Msgs.GeometryMsgs.Twist>("cmd_vel");
        using var stop = new CancellationTokenSource();
        var publishing = Task.Run(async () =>
        {
            while (!stop.IsCancellationRequested)
            {
                pub.Publish(Msgs.MessageExtensions.Twist2D(0.5, -0.25));
                await Task.Delay(20);
            }
        });

        var (code, yaml, error) = await Run(domain, "topic", "echo", "/cmd_vel", "--once", "--timeout", "5");
        code.ShouldBe(0, error);
        yaml.ShouldContain("linear:");
        yaml.ShouldContain("  x: 0.5");
        yaml.ShouldContain("  z: -0.25");

        var (_, json, _) = await Run(domain, "topic", "echo", "/cmd_vel", "geometry_msgs/msg/Twist", "--once", "--json", "--timeout", "5");
        json.ShouldContain("\"linear\":{\"x\":0.5");
        stop.Cancel();
        await publishing;
    }

    [Fact]
    public async Task Service_call_and_parameters_work_against_a_spinning_node()
    {
        var domain = NextDomain();
        await using var ctx = Context(domain);
        var server = ctx.CreateNode("adder");
        server.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints", r => new AddTwoIntsResponse { Sum = r.A + r.B });
        server.DeclareParameter("gain", 1.5);
        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);

        var (code, output, error) = await Run(domain, "service", "call", "/add_two_ints", "example_interfaces/srv/AddTwoInts", "{a: 40, b: 2}");
        code.ShouldBe(0, error);
        output.ShouldContain("sum: 42");

        (await Run(domain, "service", "list", "-t")).Output.ShouldContain("/add_two_ints [example_interfaces/srv/AddTwoInts]");
        (await Run(domain, "param", "get", "/adder", "gain")).Output.ShouldContain("Double value is: 1.5");
        (await Run(domain, "param", "set", "/adder", "gain", "2.5")).Output.ShouldContain("Set parameter successful");
        server.GetParameter<double>("gain").ShouldBe(2.5);
        (await Run(domain, "param", "set", "/adder", "gain", "fast")).Output.ShouldContain("Setting parameter failed");
        (await Run(domain, "param", "list", "/adder")).Output.ShouldContain("  gain");
        (await Run(domain, "param", "dump", "/adder")).Output.ShouldContain("gain: 2.5");

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Action_send_goal_prints_feedback_and_result()
    {
        var domain = NextDomain();
        await using var ctx = Context(domain);
        ctx.CreateNode("fib").CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("fibonacci", (g, _) =>
        {
            g.PublishFeedback(new FibonacciFeedback { Sequence = [0, 1] });
            return Task.FromResult(new FibonacciResult { Sequence = [0, 1, 1, 2] });
        });
        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);

        (await Run(domain, "action", "list", "-t")).Output.ShouldContain("/fibonacci [example_interfaces/action/Fibonacci]");
        var (code, output, error) = await Run(domain, "action", "send_goal", "/fibonacci", "example_interfaces/action/Fibonacci", "{order: 3}", "--feedback");
        code.ShouldBe(0, error);
        output.ShouldContain("Goal finished with status: SUCCEEDED");
        output.ShouldContain("sequence: [0, 1, 1, 2]");

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Interface_show_and_list()
    {
        var (code, output, _) = await Run(NextDomain(), "interface", "show", "std_srvs/srv/SetBool");
        code.ShouldBe(0);
        output.ShouldContain("bool data");
        output.ShouldContain("---");
        output.ShouldContain("string message");

        (await Run(NextDomain(), "interface", "list", "-a")).Output.ShouldContain("example_interfaces/action/Fibonacci");
        (await Run(NextDomain(), "interface", "package", "geometry_msgs")).Output.ShouldContain("geometry_msgs/msg/Twist");
        (await Run(NextDomain(), "interface", "show", "no_pkg/msg/Nothing")).Code.ShouldBe(1);
    }

    [Fact]
    public async Task Interface_generate_emits_csharp_sources()
    {
        var root = Directory.CreateTempSubdirectory("ros2net-gen").FullName;
        var msgDir = Path.Combine(root, "in", "my_robot_msgs", "msg");
        var srvDir = Path.Combine(root, "in", "my_robot_msgs", "srv");
        Directory.CreateDirectory(msgDir);
        Directory.CreateDirectory(srvDir);
        await File.WriteAllTextAsync(Path.Combine(msgDir, "BatteryCell.msg"), "float32 voltage\nuint8 index\n");
        await File.WriteAllTextAsync(Path.Combine(srvDir, "Dock.srv"), "string station\n---\nbool success\n");
        var outDir = Path.Combine(root, "out");

        var (code, output, error) = await Run(NextDomain(), "interface", "generate", Path.Combine(root, "in"), "-o", outDir);
        code.ShouldBe(0, error);
        output.ShouldContain("Generated 2 interface file(s)");
        var msg = await File.ReadAllTextAsync(Path.Combine(outDir, "my_robot_msgs.msg.BatteryCell.g.cs"));
        msg.ShouldContain("namespace ROS2.Net.Msgs.MyRobotMsgs;");
        msg.ShouldContain("public float Voltage");
        File.Exists(Path.Combine(outDir, "my_robot_msgs.srv.Dock.g.cs")).ShouldBeTrue();
        File.Exists(Path.Combine(outDir, "RosTypeRegistration.g.cs")).ShouldBeTrue();
    }

    [Fact]
    public async Task Bag_record_info_and_export_round_trip()
    {
        var domain = NextDomain();
        var dir = Path.Combine(Directory.CreateTempSubdirectory("ros2net-bag").FullName, "bag");
        await using var ctx = Context(domain);
        var pub = ctx.CreateNode("talker").CreatePublisher<StdMsgs.Int32>("/numbers");
        using var stop = new CancellationTokenSource();
        var publishing = Task.Run(async () =>
        {
            var i = 0;
            while (!stop.IsCancellationRequested)
            {
                pub.Publish(new StdMsgs.Int32 { Data = i++ });
                await Task.Delay(10);
            }
        });

        var (code, output, error) = await Run(domain, "bag", "record", "/numbers", "-o", dir, "--duration", "1.5");
        stop.Cancel();
        await publishing;
        code.ShouldBe(0, error);
        output.ShouldContain("Recorded");

        var (_, info, _) = await Run(domain, "bag", "info", dir);
        info.ShouldContain("/numbers");
        info.ShouldContain("std_msgs/msg/Int32");

        var (_, jsonl, _) = await Run(domain, "bag", "export", dir, "--format", "jsonl");
        jsonl.ShouldContain("\"data\":");

        var (_, csv, _) = await Run(domain, "bag", "export", dir, "--format", "csv", "--topic", "/numbers");
        csv.Split('\n')[0].ShouldContain("data");
    }

    [Fact]
    public async Task Security_creates_keystore_and_enclave()
    {
        var dir = Path.Combine(Directory.CreateTempSubdirectory("ros2net-ks").FullName, "keystore");
        (await Run(NextDomain(), "security", "create_keystore", dir)).Code.ShouldBe(0);
        (await Run(NextDomain(), "security", "create_enclave", dir, "/demo/talker")).Code.ShouldBe(0);
        (await Run(NextDomain(), "security", "list_enclaves", dir)).Output.ShouldContain("/demo/talker");
        File.Exists(Path.Combine(dir, "enclaves", "demo", "talker", "cert.pem")).ShouldBeTrue();
    }

    [Fact]
    public async Task Errors_are_reported_without_stack_traces()
    {
        var (code, _, error) = await Run(NextDomain(), "topic", "pub", "/x", "no_pkg/msg/Nope", "{}", "--once");
        code.ShouldBe(1);
        error.ShouldContain("Unknown message type");
        error.ShouldNotContain("   at ");
    }

    [Fact]
    public void Yaml_input_converts_to_json()
    {
        YamlJson.ToJson("{linear: {x: 0.5}, name: 'r1', on: true}").ShouldBe("{\"linear\":{\"x\":0.5},\"name\":\"r1\",\"on\":true}");
        YamlJson.ToJson("""{"a": 1}""").ShouldBe("""{"a": 1}""");
        YamlJson.ToJson(null).ShouldBe("{}");
        YamlJson.ToYaml("""{"a":{"b":[1,2]},"c":"x: y"}""").ShouldBe("a:\n  b: [1, 2]\nc: \"x: y\"");
    }
}
