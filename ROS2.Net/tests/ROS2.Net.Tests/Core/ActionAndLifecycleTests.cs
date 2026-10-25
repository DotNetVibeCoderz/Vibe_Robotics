using ROS2.Net.Msgs.ExampleInterfaces;
using Shouldly;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Tests.Core;

public class ActionAndLifecycleTests
{
    private static async Task<FibonacciResult> Fibonacci(ServerGoalHandle<FibonacciGoal, FibonacciResult, FibonacciFeedback> goal, CancellationToken ct)
    {
        var seq = new List<int> { 0, 1 };
        for (var i = 1; i < goal.Goal.Order; i++)
        {
            ct.ThrowIfCancellationRequested();
            seq.Add(seq[i] + seq[i - 1]);
            goal.PublishFeedback(new FibonacciFeedback { Sequence = seq.ToArray() });
            await Task.Delay(5, ct);
        }

        return new FibonacciResult { Sequence = seq.ToArray() };
    }

    [Fact]
    public async Task Fibonacci_goal_succeeds_with_feedback()
    {
        await using var ctx = TestDomains.Create();
        var server = ctx.CreateNode("fib_server");
        server.CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("fibonacci", Fibonacci);
        var clientNode = ctx.CreateNode("fib_client");
        var client = clientNode.CreateActionClient<FibonacciGoal, FibonacciResult, FibonacciFeedback>("/fibonacci");

        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);
        (await client.WaitForServerAsync(TimeSpan.FromSeconds(2))).ShouldBeTrue();

        var feedbackCount = 0;
        var result = await client.SendGoalAndWaitAsync(new FibonacciGoal { Order = 8 }, _ => Interlocked.Increment(ref feedbackCount)).WithTimeout();

        result.Status.ShouldBe(GoalStatus.Succeeded);
        result.Result.Sequence.ShouldBe(new[] { 0, 1, 1, 2, 3, 5, 8, 13, 21 });
        feedbackCount.ShouldBeGreaterThan(0);

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Goals_can_be_canceled_and_rejected()
    {
        await using var ctx = TestDomains.Create();
        var server = ctx.CreateNode("s");
        server.CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("slow", async (g, ct) =>
        {
            await Task.Delay(Timeout.Infinite, ct);
            return new FibonacciResult();
        }, new ActionServerOptions<FibonacciGoal, FibonacciResult, FibonacciFeedback>
        {
            GoalCallback = (_, goal) => goal.Order < 0 ? GoalResponse.Reject : GoalResponse.Accept,
        });
        var client = ctx.CreateNode("c").CreateActionClient<FibonacciGoal, FibonacciResult, FibonacciFeedback>("/slow");

        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);
        await client.WaitForServerAsync(TimeSpan.FromSeconds(2));

        await Should.ThrowAsync<GoalRejectedException>(() => client.SendGoalAsync(new FibonacciGoal { Order = -1 }));

        var handle = await client.SendGoalAsync(new FibonacciGoal { Order = 5 });
        (await handle.CancelAsync()).ShouldBeTrue();
        (await handle.GetResultAsync().WithTimeout()).Status.ShouldBe(GoalStatus.Canceled);

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Lifecycle_runs_the_state_machine_and_gates_publishers()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateLifecycleNode("managed");
        var configured = false;
        node.ConfigureHandler = _ =>
        {
            configured = true;
            return ValueTask.FromResult(CallbackReturn.Success);
        };
        var pub = node.CreateLifecyclePublisher<StdMsgs.String>("out");
        var sub = ctx.CreateNode("observer").CreateSubscription<StdMsgs.String>("/out");

        pub.Publish(new StdMsgs.String { Data = "dropped" });
        sub.TryTake(out _).ShouldBeFalse();

        (await node.ActivateAsync()).ShouldBe(LifecycleState.Unconfigured); // not available yet
        (await node.ConfigureAsync()).ShouldBe(LifecycleState.Inactive);
        configured.ShouldBeTrue();
        (await node.ActivateAsync()).ShouldBe(LifecycleState.Active);
        pub.Publish(new StdMsgs.String { Data = "live" });
        sub.TryTake(out var m).ShouldBeTrue();
        m!.Data.ShouldBe("live");

        (await node.DeactivateAsync()).ShouldBe(LifecycleState.Inactive);
        pub.IsActivated.ShouldBeFalse();
        (await node.ShutdownAsync()).ShouldBe(LifecycleState.Finalized);
    }

    [Fact]
    public async Task Lifecycle_error_goes_through_error_processing()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateLifecycleNode("faulty");
        node.ConfigureHandler = _ => throw new InvalidOperationException("boom");
        node.ErrorHandler = _ => ValueTask.FromResult(CallbackReturn.Success);
        var states = new List<LifecycleState>();
        node.StateChanged += (_, s) => states.Add(s);

        (await node.ConfigureAsync()).ShouldBe(LifecycleState.Unconfigured);
        states.ShouldBe(new[] { LifecycleState.Configuring, LifecycleState.ErrorProcessing, LifecycleState.Unconfigured });
    }

    [Fact]
    public async Task Lifecycle_is_controllable_through_its_services()
    {
        await using var ctx = TestDomains.Create();
        var node = ctx.CreateLifecycleNode("remote");
        var tool = ctx.CreateNode("tool");
        var change = tool.CreateClient<Msgs.LifecycleMsgs.ChangeStateRequest, Msgs.LifecycleMsgs.ChangeStateResponse>("/remote/change_state");
        var get = tool.CreateClient<Msgs.LifecycleMsgs.GetStateRequest, Msgs.LifecycleMsgs.GetStateResponse>("/remote/get_state");

        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);
        await change.WaitForServiceAsync(TimeSpan.FromSeconds(2));

        var ok = await change.CallAsync(new Msgs.LifecycleMsgs.ChangeStateRequest
        {
            Transition = new Msgs.LifecycleMsgs.Transition { Id = Msgs.LifecycleMsgs.Transition.TRANSITION_CONFIGURE },
        }, TimeSpan.FromSeconds(2));
        ok.Success.ShouldBeTrue();
        (await get.CallAsync(new Msgs.LifecycleMsgs.GetStateRequest(), TimeSpan.FromSeconds(2))).CurrentState.Label.ShouldBe("inactive");

        cts.Cancel();
        await spin;
    }
}
