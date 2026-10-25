using System.CommandLine;
using ROS2.Net.Msgs.ExampleInterfaces;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net run-demo ...</c>: the classic demo_nodes, built in.</summary>
internal static class DemoCommands
{
    public static Command Build(CliEnvironment env)
    {
        var demo = new Argument<string>("demo") { Description = "talker | listener | add_two_ints_server | add_two_ints_client | fibonacci_server" };
        demo.AcceptOnlyFromAmong("talker", "listener", "add_two_ints_server", "add_two_ints_client", "fibonacci_server");
        var duration = new Option<double?>("--duration") { Description = "Stop after this many seconds." };
        var cmd = new Command("run-demo", "Run a built-in demo node (like ros2 run demo_nodes_cpp ...).") { demo, duration };
        cmd.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            using var limit = Cmd.Limit(p.GetValue(duration), ct);
            var ctx = s.Context;
            switch (p.GetValue(demo))
            {
                case "talker":
                {
                    var node = ctx.CreateNode("talker");
                    var pub = node.CreatePublisher<StdMsgs.String>("chatter");
                    var i = 0;
                    node.CreateTimer(TimeSpan.FromSeconds(1), () =>
                    {
                        var msg = new StdMsgs.String { Data = $"Hello World: {++i}" };
                        env.Out.WriteLine($"[INFO] [talker]: Publishing: '{msg.Data}'");
                        pub.Publish(msg);
                    });
                    break;
                }

                case "listener":
                {
                    var node = ctx.CreateNode("listener");
                    node.CreateSubscription<StdMsgs.String>("chatter", m => env.Out.WriteLine($"[INFO] [listener]: I heard: [{m.Data}]"));
                    break;
                }

                case "add_two_ints_server":
                {
                    var node = ctx.CreateNode("add_two_ints_server");
                    node.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints", r =>
                    {
                        env.Out.WriteLine($"[INFO] [add_two_ints_server]: Incoming request a: {r.A} b: {r.B}");
                        return new AddTwoIntsResponse { Sum = r.A + r.B };
                    });
                    break;
                }

                case "add_two_ints_client":
                {
                    var node = ctx.CreateNode("add_two_ints_client");
                    var client = node.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints");
                    if (!await client.WaitForServiceAsync(TimeSpan.FromSeconds(10), limit.Token))
                    {
                        throw new CliException("add_two_ints is not available.");
                    }

                    var res = await client.CallAsync(new AddTwoIntsRequest { A = 2, B = 3 }, TimeSpan.FromSeconds(5), limit.Token);
                    env.Out.WriteLine($"[INFO] [add_two_ints_client]: Result of add_two_ints: {res.Sum}");
                    return 0;
                }

                case "fibonacci_server":
                {
                    var node = ctx.CreateNode("fibonacci_action_server");
                    node.CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("fibonacci", async (goal, token) =>
                    {
                        env.Out.WriteLine($"[INFO] [fibonacci_action_server]: Executing goal (order {goal.Goal.Order})");
                        var seq = new List<int> { 0, 1 };
                        for (var i = 1; i < goal.Goal.Order; i++)
                        {
                            token.ThrowIfCancellationRequested();
                            seq.Add(seq[i] + seq[i - 1]);
                            goal.PublishFeedback(new FibonacciFeedback { Sequence = seq.ToArray() });
                            await Task.Delay(500, token);
                        }

                        return new FibonacciResult { Sequence = seq.ToArray() };
                    });
                    break;
                }
            }

            await ctx.SpinAsync(limit.Token);
            return 0;
        });
        return cmd;
    }
}
