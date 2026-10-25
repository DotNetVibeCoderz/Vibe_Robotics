// The classic ROS 2 demo: a talker publishes std_msgs/String on /chatter, a listener prints it.
// Runs on the in-process middleware by default; set ROS2NET_RMW=rtps (with ROS2.Net.Rmw.Rtps referenced)
// to talk to real ROS 2 nodes: `ros2 topic echo /chatter`.
using Microsoft.Extensions.Logging;
using ROS2.Net;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await Ros2Context.CreateAsync(new Ros2ContextOptions
{
    Arguments = args,
    HandleShutdownSignals = true,
});

var talker = context.CreateNode("talker");
var publisher = talker.CreatePublisher<StdMsgs.String>("chatter");
var count = 0;
talker.CreateTimer(TimeSpan.FromSeconds(0.5), () =>
{
    var msg = new StdMsgs.String { Data = $"Hello World: {++count}" };
    talker.Logger.LogInformation("Publishing: '{Data}'", msg.Data);
    publisher.Publish(msg);
});

var listener = context.CreateNode("listener");
listener.CreateSubscription<StdMsgs.String>("chatter", msg =>
    listener.Logger.LogInformation("I heard: [{Data}]", msg.Data));

// Print to the console without a logging provider: mirror /rosout.
var console = context.CreateNode("console");
console.CreateSubscription<ROS2.Net.Msgs.RclInterfaces.Log>("/rosout", log =>
    Console.WriteLine($"[{NodeLogger.FromRosLevel(log.Level)}] [{log.Name}]: {log.Msg}"), QosProfile.Rosout);

Console.WriteLine("Press Ctrl+C to stop.");
using var stopAfter = new CancellationTokenSource(args.Contains("--once") ? TimeSpan.FromSeconds(3) : Timeout.InfiniteTimeSpan);
await context.SpinAsync(stopAfter.Token);
