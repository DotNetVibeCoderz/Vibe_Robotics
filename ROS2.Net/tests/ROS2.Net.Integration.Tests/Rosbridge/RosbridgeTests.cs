using ROS2.Net.Msgs.ExampleInterfaces;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Rmw.Rosbridge;
using Shouldly;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Integration.Tests.Rosbridge;

public class RosbridgeTests
{
    private static Task<Ros2Context> Connect(FakeRosbridgeServer server) =>
        Ros2Context.CreateAsync(new Ros2ContextOptions().UseRosbridge(server.Url, o =>
        {
            o.GraphPollInterval = TimeSpan.FromMilliseconds(100);
            o.ReconnectDelay = TimeSpan.FromMilliseconds(100);
        }));

    [Fact]
    public async Task Talker_reaches_listener_through_rosbridge()
    {
        await using var server = new FakeRosbridgeServer();
        await using var a = await Connect(server);
        await using var b = await Connect(server);
        a.Rmw.Identifier.ShouldBe("rosbridge");

        var sub = b.CreateNode("listener").CreateSubscription<StdMsgs.String>("/chatter");
        var pub = a.CreateNode("talker").CreatePublisher<StdMsgs.String>("/chatter");
        await WaitUntil(() => server.Ops.Contains("subscribe:/chatter"));

        pub.Publish(new StdMsgs.String { Data = "hello over websocket" });
        (await sub.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(5))).Data.ShouldBe("hello over websocket");
    }

    [Fact]
    public async Task Complex_messages_keep_arrays_and_bytes()
    {
        await using var server = new FakeRosbridgeServer();
        await using var ctx = await Connect(server);
        var node = ctx.CreateNode("n");
        var sub = node.CreateSubscription<Image>("/camera/image");
        var pub = node.CreatePublisher<Image>("/camera/image");
        await WaitUntil(() => server.Ops.Contains("subscribe:/camera/image"));

        pub.Publish(new Image { Width = 2, Height = 2, Encoding = "mono8", Step = 2, Data = [1, 2, 3, 255] });
        var img = await sub.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(5));
        img.Data.ShouldBe(new byte[] { 1, 2, 3, 255 });
        img.Encoding.ShouldBe("mono8");
    }

    [Fact]
    public async Task Service_round_trip_and_graph_from_rosapi()
    {
        await using var server = new FakeRosbridgeServer();
        await using var serverCtx = await Connect(server);
        await using var clientCtx = await Connect(server);
        var serverNode = serverCtx.CreateNode("adder");
        serverNode.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("/add_two_ints", r => new AddTwoIntsResponse { Sum = r.A + r.B });
        var clientNode = clientCtx.CreateNode("caller");
        var client = clientNode.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("/add_two_ints");

        using var cts = new CancellationTokenSource();
        var spin = serverCtx.SpinAsync(cts.Token);

        (await client.WaitForServiceAsync(TimeSpan.FromSeconds(5))).ShouldBeTrue();
        (await client.CallAsync(new AddTwoIntsRequest { A = 20, B = 22 }, TimeSpan.FromSeconds(5))).Sum.ShouldBe(42);
        clientNode.GetServiceNamesAndTypes().ShouldContain(s => s.Name == "/add_two_ints");
        clientNode.GetNodeNames().ShouldContain(n => n.FullyQualifiedName == "/rosbridge_websocket");

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Fibonacci_action_round_trip_through_hidden_action_names()
    {
        await using var server = new FakeRosbridgeServer();
        await using var serverCtx = await Connect(server);
        await using var clientCtx = await Connect(server);
        serverCtx.CreateNode("fib_server").CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("/fibonacci", async (goal, ct) =>
        {
            var seq = new List<int> { 0, 1 };
            for (var i = 1; i < goal.Goal.Order; i++)
            {
                seq.Add(seq[i] + seq[i - 1]);
                goal.PublishFeedback(new FibonacciFeedback { Sequence = seq.ToArray() });
                await Task.Delay(10, ct);
            }

            return new FibonacciResult { Sequence = seq.ToArray() };
        });
        var client = clientCtx.CreateNode("fib_client").CreateActionClient<FibonacciGoal, FibonacciResult, FibonacciFeedback>("/fibonacci");

        using var cts = new CancellationTokenSource();
        var spin = serverCtx.SpinAsync(cts.Token);

        (await client.WaitForServerAsync(TimeSpan.FromSeconds(5))).ShouldBeTrue();
        var feedback = 0;
        var result = await client.SendGoalAndWaitAsync(new FibonacciGoal { Order = 6 }, _ => Interlocked.Increment(ref feedback)).WaitAsync(TimeSpan.FromSeconds(10));

        result.Status.ShouldBe(GoalStatus.Succeeded);
        result.Result.Sequence.ShouldBe(new[] { 0, 1, 1, 2, 3, 5, 8 });
        server.Ops.ShouldContain("advertise_service:/fibonacci/_action/send_goal");
        server.Ops.ShouldContain("subscribe:/fibonacci/_action/feedback");

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Reconnects_and_resubscribes_after_the_link_drops()
    {
        await using var server = new FakeRosbridgeServer();
        await using var ctx = await Connect(server);
        var node = ctx.CreateNode("n");
        var sub = node.CreateSubscription<StdMsgs.Int32>("/counter");
        var pub = node.CreatePublisher<StdMsgs.Int32>("/counter");
        await WaitUntil(() => server.Ops.Count(o => o == "subscribe:/counter") == 1);

        server.DisconnectAll();
        await WaitUntil(() => server.Ops.Count(o => o == "subscribe:/counter") >= 2, 10_000);
        await WaitUntil(() => ((RosbridgeRmw)ctx.Rmw).IsConnected);

        pub.Publish(new StdMsgs.Int32 { Data = 7 });
        (await sub.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(5))).Data.ShouldBe(7);
    }

    [Fact]
    public async Task Connecting_to_nothing_fails_fast()
    {
        var options = new Ros2ContextOptions().UseRosbridge("ws://127.0.0.1:1", o => o.ConnectTimeout = TimeSpan.FromSeconds(2));
        await Should.ThrowAsync<InvalidOperationException>(() => Ros2Context.CreateAsync(options));
    }

    private static async Task WaitUntil(Func<bool> condition, int timeoutMs = 5000)
    {
        var start = Environment.TickCount64;
        while (!condition())
        {
            if (Environment.TickCount64 - start > timeoutMs)
            {
                throw new TimeoutException("Condition not met in time.");
            }

            await Task.Delay(10);
        }
    }
}
