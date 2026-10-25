using ROS2.Net.Msgs.ExampleInterfaces;
using ROS2.Net.Msgs.StdSrvs;
using ROS2.Net.Rmw;
using Shouldly;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Tests.Core;

public class NodeTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task Talker_reaches_listener_through_the_executor(bool serialize)
    {
        await using var ctx = TestDomains.Create(serialize);
        var talker = ctx.CreateNode("talker");
        var listener = ctx.CreateNode("listener");
        var received = new TaskCompletionSource<string>();
        listener.CreateSubscription<StdMsgs.String>("chatter", m => received.TrySetResult(m.Data));
        var pub = talker.CreatePublisher<StdMsgs.String>("chatter");

        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);
        pub.SubscriptionCount.ShouldBe(1);
        pub.Publish(new StdMsgs.String { Data = "hello" });

        (await received.Task.WithTimeout()).ShouldBe("hello");
        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Subscription_without_callback_streams_with_await_foreach()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        var sub = node.CreateSubscription<StdMsgs.Int32>("numbers");
        var pub = node.CreatePublisher<StdMsgs.Int32>("numbers");
        for (var i = 0; i < 3; i++)
        {
            pub.Publish(new StdMsgs.Int32 { Data = i });
        }

        var got = new List<int>();
        await foreach (var m in sub.ReadAllAsync().WithCancellation(new CancellationTokenSource(2000).Token))
        {
            got.Add(m.Data);
            if (got.Count == 3)
            {
                break;
            }
        }

        got.ShouldBe(new[] { 0, 1, 2 });
    }

    [Fact]
    public async Task Keep_last_depth_drops_oldest_and_reports_lost_messages()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        var lost = 0;
        var got = new List<int>();
        node.CreateSubscription<StdMsgs.Int32>("t", m => got.Add(m.Data), QosProfile.Default.KeepLast(2),
            new SubscriptionOptions { EventCallback = e => { if (e.Kind == QosEventKind.MessageLost) { lost++; } } });
        var pub = node.CreatePublisher<StdMsgs.Int32>("t");
        for (var i = 0; i < 5; i++)
        {
            pub.Publish(new StdMsgs.Int32 { Data = i });
        }

        using var exec = new SingleThreadedExecutor();
        exec.AddNode(node);
        await exec.SpinSomeAsync();

        got.ShouldBe(new[] { 3, 4 });
        lost.ShouldBe(3);
    }

    [Fact]
    public async Task Transient_local_delivers_to_late_joiners()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        var pub = node.CreatePublisher<StdMsgs.String>("latched", QosProfile.Latched);
        pub.Publish(new StdMsgs.String { Data = "map" });

        var late = node.CreateSubscription<StdMsgs.String>("latched", QosProfile.Latched);
        (await late.TakeAsync().AsTask().WithTimeout()).Data.ShouldBe("map");

        var volatileSub = node.CreateSubscription<StdMsgs.String>("latched", QosProfile.Default);
        volatileSub.TryTake(out _).ShouldBeFalse();
    }

    [Fact]
    public async Task Incompatible_qos_does_not_match()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        var pub = node.CreatePublisher<StdMsgs.String>("q", QosProfile.SensorData);
        node.CreateSubscription<StdMsgs.String>("q", QosProfile.Default.AsReliable());
        pub.SubscriptionCount.ShouldBe(0);

        QosMatching.Check(QosProfile.SensorData, QosProfile.Default, out var reason).ShouldBe(QosCompatibility.Error);
        reason.ShouldContain("Best effort");
    }

    [Fact]
    public async Task Content_filter_rejects_before_queueing()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        var sub = node.CreateSubscription<StdMsgs.Int32>("f", options: new SubscriptionOptions { ContentFilter = m => ((StdMsgs.Int32)m).Data % 2 == 0 });
        var pub = node.CreatePublisher<StdMsgs.Int32>("f");
        foreach (var i in new[] { 1, 2, 3, 4 })
        {
            pub.Publish(new StdMsgs.Int32 { Data = i });
        }

        sub.TryTake(out var a).ShouldBeTrue();
        sub.TryTake(out var b).ShouldBeTrue();
        (a!.Data, b!.Data).ShouldBe((2, 4));
        sub.TryTake(out _).ShouldBeFalse();
    }

    [Fact]
    public async Task Service_call_does_not_require_the_client_to_spin()
    {
        await using var ctx = TestDomains.Create();
        var server = ctx.CreateNode("server");
        var client = ctx.CreateNode("client");
        server.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("add", req => new AddTwoIntsResponse { Sum = req.A + req.B });
        var cli = client.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("add");

        using var exec = new SingleThreadedExecutor();
        exec.AddNode(server);
        using var cts = new CancellationTokenSource();
        var spin = exec.SpinAsync(cts.Token);

        (await cli.WaitForServiceAsync(TimeSpan.FromSeconds(2))).ShouldBeTrue();
        var res = await cli.CallAsync(new AddTwoIntsRequest { A = 2, B = 40 }, TimeSpan.FromSeconds(2));
        res.Sum.ShouldBe(42);

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Service_call_times_out_without_a_spinning_server()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        node.CreateService<TriggerRequest, TriggerResponse>("t", _ => new TriggerResponse { Success = true });
        var cli = node.CreateClient<TriggerRequest, TriggerResponse>("t");

        await Should.ThrowAsync<ServiceTimeoutException>(() => cli.CallAsync(new TriggerRequest(), TimeSpan.FromMilliseconds(100)));
    }

    [Fact]
    public async Task Timer_fires_on_the_executor()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        var count = 0;
        var done = new TaskCompletionSource();
        node.CreateTimer(TimeSpan.FromMilliseconds(10), () =>
        {
            if (++count == 5)
            {
                done.TrySetResult();
            }
        });

        using var cts = new CancellationTokenSource();
        var spin = node.SpinAsync(cts.Token);
        await done.Task.WithTimeout();
        cts.Cancel();
        await spin;
        count.ShouldBeGreaterThanOrEqualTo(5);
    }

    [Fact]
    public async Task Mutually_exclusive_group_never_overlaps_on_a_multithreaded_executor()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("n");
        var active = 0;
        var maxActive = 0;
        var calls = 0;
        var done = new TaskCompletionSource();
        async ValueTask Callback(StdMsgs.Int32 _)
        {
            var now = Interlocked.Increment(ref active);
            InterlockedMax(ref maxActive, now);
            await Task.Delay(5);
            Interlocked.Decrement(ref active);
            if (Interlocked.Increment(ref calls) == 20)
            {
                done.TrySetResult();
            }
        }

        node.CreateSubscription<StdMsgs.Int32>("a", Callback, QosProfile.Default.KeepAll());
        node.CreateSubscription<StdMsgs.Int32>("b", Callback, QosProfile.Default.KeepAll());
        var pa = node.CreatePublisher<StdMsgs.Int32>("a");
        var pb = node.CreatePublisher<StdMsgs.Int32>("b");
        for (var i = 0; i < 10; i++)
        {
            pa.Publish(new StdMsgs.Int32());
            pb.Publish(new StdMsgs.Int32());
        }

        using var exec = new MultiThreadedExecutor(4);
        exec.AddNode(node);
        using var cts = new CancellationTokenSource();
        var spin = exec.SpinAsync(cts.Token);
        await done.Task.WithTimeout();
        cts.Cancel();
        await spin;
        maxActive.ShouldBe(1);
    }

    private static void InterlockedMax(ref int target, int value)
    {
        int current;
        while ((current = Volatile.Read(ref target)) < value && Interlocked.CompareExchange(ref target, value, current) != current)
        {
        }
    }

    [Fact]
    public async Task Graph_reports_nodes_topics_and_services()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateNode("graph_node", "/robot");
        node.CreatePublisher<StdMsgs.String>("status");
        node.CreateService<TriggerRequest, TriggerResponse>("reset", _ => new TriggerResponse());

        node.GetNodeNames().ShouldContain(n => n.FullyQualifiedName == "/robot/graph_node");
        node.GetTopicNamesAndTypes().ShouldContain(t => t.Name == "/robot/status" && t.Types.Contains("std_msgs/msg/String"));
        node.GetServiceNamesAndTypes().ShouldContain(s => s.Name == "/robot/reset" && s.Types.Contains("std_srvs/srv/Trigger"));
        node.GetServiceNamesAndTypes().ShouldContain(s => s.Name == "/robot/graph_node/get_parameters");
        node.CountPublishers("status").ShouldBe(1);
    }

    [Fact]
    public async Task Rosout_carries_log_messages()
    {
        await using var ctx = TestDomains.Create();
        var watcher = ctx.CreateNode("watcher");
        var logs = watcher.CreateSubscription<Msgs.RclInterfaces.Log>("/rosout", QosProfile.Rosout);
        var node = ctx.CreateNode("chatty");
        Microsoft.Extensions.Logging.LoggerExtensions.LogWarning(node.Logger, "battery at {Percent}%", 12);

        Msgs.RclInterfaces.Log? warn = null;
        await foreach (var m in logs.ReadAllAsync().WithCancellation(new CancellationTokenSource(2000).Token))
        {
            if (m.Name == "chatty")
            {
                warn = m;
                break;
            }
        }

        warn.ShouldNotBeNull();
        warn.Level.ShouldBe(Msgs.RclInterfaces.Log.WARN);
        warn.Msg.ShouldBe("battery at 12%");
    }
}
