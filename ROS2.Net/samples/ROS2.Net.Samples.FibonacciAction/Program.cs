// example_interfaces/action/Fibonacci: goal, feedback, result and cancellation.
using ROS2.Net;
using ROS2.Net.Msgs.ExampleInterfaces;

await using var context = await Ros2Context.CreateAsync(new Ros2ContextOptions { Arguments = args });

var server = context.CreateNode("fibonacci_action_server");
server.CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("fibonacci", async (goal, ct) =>
{
    Console.WriteLine($"Executing goal: order {goal.Goal.Order}");
    var sequence = new List<int> { 0, 1 };
    for (var i = 1; i < goal.Goal.Order; i++)
    {
        ct.ThrowIfCancellationRequested(); // honors cancel requests
        sequence.Add(sequence[i] + sequence[i - 1]);
        goal.PublishFeedback(new FibonacciFeedback { Sequence = sequence.ToArray() });
        await Task.Delay(200, ct);
    }

    return new FibonacciResult { Sequence = sequence.ToArray() };
});

using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);

var clientNode = context.CreateNode("fibonacci_action_client");
var client = clientNode.CreateActionClient<FibonacciGoal, FibonacciResult, FibonacciFeedback>("fibonacci");
await client.WaitForServerAsync(TimeSpan.FromSeconds(5));

var result = await client.SendGoalAndWaitAsync(new FibonacciGoal { Order = 10 },
    feedback => Console.WriteLine($"Feedback: {string.Join(", ", feedback.Sequence)}"));
Console.WriteLine($"Result ({result.Status}): {string.Join(", ", result.Result.Sequence)}");

// Cancel a second goal half-way.
var handle = await client.SendGoalAsync(new FibonacciGoal { Order = 20 });
await Task.Delay(700);
await handle.CancelAsync();
Console.WriteLine($"Second goal finished as {(await handle.GetResultAsync()).Status}");

cts.Cancel();
await spin;
