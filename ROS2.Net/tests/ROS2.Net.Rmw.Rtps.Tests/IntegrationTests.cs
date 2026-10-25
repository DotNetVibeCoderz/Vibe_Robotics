using ROS2.Net.Msgs.ExampleInterfaces;
using ROS2.Net.Msgs.SensorMsgs;
using Shouldly;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Rmw.Rtps.Tests;

/// <summary>Two contexts in one process, each its own RTPS participant, talking over loopback UDP.</summary>
public class IntegrationTests
{
    private static int s_domain = 100 + (Environment.ProcessId % 40) * 2;

    private static int NextDomain() => Interlocked.Increment(ref s_domain) % 230;

    private static Ros2Context Create(int domain, Action<RtpsOptions>? configure = null, bool intra = false)
    {
        var options = new Ros2ContextOptions { DomainId = domain, UseIntraProcessComms = intra }.UseRtps(o =>
        {
            o.LocalhostOnly = true;
            o.ParticipantLeaseDuration = TimeSpan.FromSeconds(5);
            configure?.Invoke(o);
        });
        return Ros2Context.Create(options);
    }

    private static async Task Until(Func<bool> condition, int timeoutMs = 8000)
    {
        var start = Environment.TickCount64;
        while (!condition())
        {
            if (Environment.TickCount64 - start > timeoutMs)
            {
                throw new TimeoutException("Condition not met in time.");
            }

            await Task.Delay(20);
        }
    }

    [Fact]
    public async Task Talker_reaches_listener_in_another_participant()
    {
        var d = NextDomain();
        await using var a = Create(d);
        await using var b = Create(d);
        var pub = a.CreateNode("talker").CreatePublisher<StdMsgs.String>("chatter");
        var received = new TaskCompletionSource<string>(TaskCreationOptions.RunContinuationsAsynchronously);
        b.CreateNode("listener").CreateSubscription<StdMsgs.String>("chatter", m => received.TrySetResult(m.Data));
        using var cts = new CancellationTokenSource();
        var spin = b.SpinAsync(cts.Token);

        (await pub.WaitForSubscriptionsAsync(1, TimeSpan.FromSeconds(8))).ShouldBeTrue();
        pub.Publish(new StdMsgs.String { Data = "hello over rtps" });

        (await received.Task.WaitAsync(TimeSpan.FromSeconds(5))).ShouldBe("hello over rtps");
        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Large_messages_are_fragmented_and_reassembled_reliably()
    {
        var d = NextDomain();
        await using var a = Create(d);
        await using var b = Create(d);
        var pub = a.CreateNode("cloud").CreatePublisher<PointCloud2>("points");
        var sub = b.CreateNode("viewer").CreateSubscription<PointCloud2>("points");
        (await pub.WaitForSubscriptionsAsync(1, TimeSpan.FromSeconds(8))).ShouldBeTrue();

        var data = new byte[1024 * 1024];
        new Random(42).NextBytes(data);
        pub.Publish(new PointCloud2 { Width = (uint)(data.Length / 16), Height = 1, PointStep = 16, RowStep = (uint)data.Length, Data = data });

        var got = await sub.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(10));
        got.Data.Length.ShouldBe(data.Length);
        got.Data.AsSpan().SequenceEqual(data).ShouldBeTrue();
    }

    [Fact]
    public async Task Reliable_stream_is_delivered_in_order_without_loss()
    {
        var d = NextDomain();
        await using var a = Create(d);
        await using var b = Create(d);
        var pub = a.CreateNode("p").CreatePublisher<StdMsgs.Int32>("seq", QosProfile.Default.KeepLast(500));
        var sub = b.CreateNode("s").CreateSubscription<StdMsgs.Int32>("seq", QosProfile.Default.KeepLast(500));
        (await pub.WaitForSubscriptionsAsync(1, TimeSpan.FromSeconds(8))).ShouldBeTrue();
        for (var i = 0; i < 300; i++)
        {
            pub.Publish(new StdMsgs.Int32 { Data = i });
        }

        var values = new List<int>();
        using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(10));
        await foreach (var m in sub.ReadAllAsync(cts.Token))
        {
            values.Add(m.Data);
            if (values.Count == 300)
            {
                break;
            }
        }

        values.ShouldBe(Enumerable.Range(0, 300));
    }

    [Fact]
    public async Task Transient_local_reaches_a_late_joining_participant()
    {
        var d = NextDomain();
        await using var a = Create(d);
        var pub = a.CreateNode("map_server").CreatePublisher<StdMsgs.String>("map", QosProfile.Latched);
        pub.Publish(new StdMsgs.String { Data = "the map" });

        await using var b = Create(d);
        var sub = b.CreateNode("late").CreateSubscription<StdMsgs.String>("map", QosProfile.Latched);
        (await sub.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(10))).Data.ShouldBe("the map");
    }

    [Theory]
    [InlineData(ServiceMapping.FastDds)]
    [InlineData(ServiceMapping.CycloneDds)]
    public async Task Service_call_across_participants(ServiceMapping mapping)
    {
        var d = NextDomain();
        await using var a = Create(d, o => o.ServiceMapping = mapping);
        await using var b = Create(d, o => o.ServiceMapping = mapping);
        a.CreateNode("adder").CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints", r => new AddTwoIntsResponse { Sum = r.A + r.B });
        using var cts = new CancellationTokenSource();
        var spin = a.SpinAsync(cts.Token);
        var client = b.CreateNode("caller").CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints");

        (await client.WaitForServiceAsync(TimeSpan.FromSeconds(8))).ShouldBeTrue();
        (await client.CallAsync(new AddTwoIntsRequest { A = 40, B = 2 }, TimeSpan.FromSeconds(5))).Sum.ShouldBe(42);
        (await client.CallAsync(new AddTwoIntsRequest { A = 1, B = 1 }, TimeSpan.FromSeconds(5))).Sum.ShouldBe(2);

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Fibonacci_action_across_participants()
    {
        var d = NextDomain();
        await using var a = Create(d);
        await using var b = Create(d);
        a.CreateNode("fib_server").CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("fibonacci", async (goal, ct) =>
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
        using var cts = new CancellationTokenSource();
        var spin = a.SpinAsync(cts.Token);
        var client = b.CreateNode("fib_client").CreateActionClient<FibonacciGoal, FibonacciResult, FibonacciFeedback>("/fibonacci");

        (await client.WaitForServerAsync(TimeSpan.FromSeconds(8))).ShouldBeTrue();
        var feedback = 0;
        var result = await client.SendGoalAndWaitAsync(new FibonacciGoal { Order = 6 }, _ => Interlocked.Increment(ref feedback)).WaitAsync(TimeSpan.FromSeconds(10));
        result.Status.ShouldBe(GoalStatus.Succeeded);
        result.Result.Sequence.ShouldBe(new[] { 0, 1, 1, 2, 3, 5, 8 });

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Graph_shows_remote_nodes_topics_and_services()
    {
        var d = NextDomain();
        await using var a = Create(d);
        await using var b = Create(d);
        var remote = a.CreateNode("talker", "/robot");
        remote.CreatePublisher<StdMsgs.String>("chatter");
        var observer = b.CreateNode("observer");

        await Until(() => observer.GetNodeNames().Any(n => n.FullyQualifiedName == "/robot/talker"));
        await Until(() => observer.GetTopicNamesAndTypes().Any(t => t.Name == "/robot/chatter" && t.Types.Contains("std_msgs/msg/String")));
        await Until(() => observer.GetServiceNamesAndTypes().Any(s => s.Name == "/robot/talker/get_parameters" && s.Types.Contains("rcl_interfaces/srv/GetParameters")));
        await Until(() => observer.GetPublishersInfo("/robot/chatter").Any(e => e.NodeName == "talker" && e.NodeNamespace == "/robot"));
        ((RtpsRmw)b.Rmw).DiscoveredParticipantCount.ShouldBe(1);
    }

    [Fact]
    public async Task Incompatible_qos_is_reported_and_does_not_match()
    {
        var d = NextDomain();
        await using var a = Create(d);
        await using var b = Create(d);
        var pub = a.CreateNode("p").CreatePublisher<StdMsgs.String>("q", QosProfile.SensorData);
        var incompatible = new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
        b.CreateNode("s").CreateSubscription<StdMsgs.String>("q", QosProfile.Default.AsReliable(), new SubscriptionOptions
        {
            EventCallback = e =>
            {
                if (e.Kind == QosEventKind.RequestedIncompatibleQos)
                {
                    incompatible.TrySetResult();
                }
            },
        });

        await incompatible.Task.WaitAsync(TimeSpan.FromSeconds(8));
        pub.SubscriptionCount.ShouldBe(0);
    }

    [Fact]
    public async Task Participant_leaving_is_noticed()
    {
        var d = NextDomain();
        await using var a = Create(d);
        var b = Create(d);
        var pub = a.CreateNode("p").CreatePublisher<StdMsgs.String>("bye");
        b.CreateNode("s").CreateSubscription<StdMsgs.String>("bye");
        (await pub.WaitForSubscriptionsAsync(1, TimeSpan.FromSeconds(8))).ShouldBeTrue();

        await b.DisposeAsync();
        await Until(() => pub.SubscriptionCount == 0);
        ((RtpsRmw)a.Rmw).DiscoveredParticipantCount.ShouldBe(0);
    }

    [Fact]
    public async Task Same_participant_endpoints_match_locally_and_honor_ignore_local()
    {
        var d = NextDomain();
        await using var a = Create(d);
        var node = a.CreateNode("self");
        var pub = node.CreatePublisher<StdMsgs.Int32>("loop");
        var sub = node.CreateSubscription<StdMsgs.Int32>("loop");
        var ignoring = node.CreateSubscription<StdMsgs.Int32>("loop", options: new SubscriptionOptions { IgnoreLocalPublications = true });

        pub.SubscriptionCount.ShouldBe(1);
        pub.Publish(new StdMsgs.Int32 { Data = 7 });
        (await sub.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(2))).Data.ShouldBe(7);
        ignoring.TryTake(out _).ShouldBeFalse();
    }

    [Fact]
    public async Task Intra_process_delivers_local_messages_exactly_once()
    {
        var d = NextDomain();
        await using var a = Create(d, intra: true);
        a.UseIntraProcessComms.ShouldBeTrue();
        var node = a.CreateNode("ipc");
        var pub = node.CreatePublisher<StdMsgs.Int32>("ipc");
        var sub = node.CreateSubscription<StdMsgs.Int32>("ipc");
        var msg = new StdMsgs.Int32 { Data = 1 };
        pub.Publish(msg);

        (await sub.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(2))).ShouldBeSameAs(msg);
        await Task.Delay(200);
        sub.TryTake(out _).ShouldBeFalse();
    }
}
