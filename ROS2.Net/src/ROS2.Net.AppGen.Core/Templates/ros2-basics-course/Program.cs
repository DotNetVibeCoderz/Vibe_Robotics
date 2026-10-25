// {{ProjectName}} — ROS 2 basics, one runnable lesson at a time (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Msgs.ExampleInterfaces;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await AppTarget.CreateContextAsync(args);
using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);
var node = context.CreateNode("student");

static void Lesson(string title, string text)
{
    Console.WriteLine();
    Console.WriteLine($"=== {title} ===");
    Console.WriteLine(text);
}

Lesson("1. Topics", "A publisher sends messages on a named topic; every matching subscription receives them.");
var got = new TaskCompletionSource<string>();
node.CreateSubscription<StdMsgs.String>("lesson/chatter", m => got.TrySetResult(m.Data));
node.CreatePublisher<StdMsgs.String>("lesson/chatter").Publish(new StdMsgs.String { Data = "hello, ROS 2" });
Console.WriteLine($"received: {await got.Task}");

Lesson("2. QoS", "Quality of Service must be compatible: a best-effort publisher cannot serve a reliable subscription.");
Console.WriteLine($"SensorData → Default: {QosMatching.Check(QosProfile.SensorData, QosProfile.Default, out var why)} ({why})");
Console.WriteLine($"Default → SensorData: {QosMatching.Check(QosProfile.Default, QosProfile.SensorData, out _)}");

Lesson("3. Services", "A service answers one request with one response, like a function call over the network.");
node.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("lesson/add", r => new AddTwoIntsResponse { Sum = r.A + r.B });
var add = node.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("lesson/add");
await add.WaitForServiceAsync(TimeSpan.FromSeconds(2));
Console.WriteLine($"2 + 40 = {(await add.CallAsync(new AddTwoIntsRequest { A = 2, B = 40 }, TimeSpan.FromSeconds(2))).Sum}");

Lesson("4. Actions", "An action is a long-running goal with feedback and a final result; it can be cancelled.");
node.CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("lesson/fibonacci", async (goal, ct) =>
{
    var seq = new List<int> { 0, 1 };
    for (var i = 1; i < goal.Goal.Order; i++)
    {
        seq.Add(seq[i] + seq[i - 1]);
        goal.PublishFeedback(new FibonacciFeedback { Sequence = seq.ToArray() });
        await Task.Delay(50, ct);
    }

    return new FibonacciResult { Sequence = seq.ToArray() };
});
var fib = node.CreateActionClient<FibonacciGoal, FibonacciResult, FibonacciFeedback>("lesson/fibonacci");
await fib.WaitForServerAsync(TimeSpan.FromSeconds(2));
var result = await fib.SendGoalAndWaitAsync(new FibonacciGoal { Order = 8 }, f => Console.Write($"{f.Sequence[^1]} "));
Console.WriteLine($"\nresult: [{string.Join(", ", result.Result.Sequence)}]");

Lesson("5. Parameters", "Parameters configure nodes at runtime; descriptors validate types and ranges.");
node.DeclareParameter("max_speed", 0.5, new ParameterDescriptor { FloatingPointRange = new FloatingPointRange(0, 1) });
Console.WriteLine($"set 0.8 → {node.SetParameter("max_speed", 0.8).Successful}; set 3.0 → {node.SetParameter("max_speed", 3.0).Reason}");

Lesson("6. Lifecycle", "Managed nodes move through Unconfigured → Inactive → Active → Finalized.");
var managed = context.CreateLifecycleNode("lesson_managed");
managed.StateChanged += (from, to) => Console.WriteLine($"  {LifecycleNode.Label(from)} → {LifecycleNode.Label(to)}");
await managed.ConfigureAsync();
await managed.ActivateAsync();
await managed.ShutdownAsync();

Console.WriteLine("\nCourse complete!");
cts.Cancel();
await spin;
