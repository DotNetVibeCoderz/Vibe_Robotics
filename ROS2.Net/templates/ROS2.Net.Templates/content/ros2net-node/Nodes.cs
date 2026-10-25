using Microsoft.Extensions.Logging;
using ROS2.Net;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace Ros2NetNode;

/// <summary>Publishes <c>std_msgs/msg/String</c> on <c>chatter</c> at <c>rate_hz</c>.</summary>
public sealed class TalkerNode : Node
{
    private int _count;

    public TalkerNode(Ros2Context context)
        : base(context, "talker")
    {
        var rate = DeclareParameter("rate_hz", 2.0, new ParameterDescriptor
        {
            Description = "Publishing rate [Hz]",
            FloatingPointRange = new FloatingPointRange(0.1, 100),
        });
        var publisher = CreatePublisher<StdMsgs.String>("chatter");
        var timer = CreateTimer(TimeSpan.FromSeconds(1 / rate), () =>
        {
            var msg = new StdMsgs.String { Data = $"Hello from Ros2NetNode: {++_count}" };
            Logger.LogInformation("Publishing: '{Data}'", msg.Data);
            publisher.Publish(msg);
        });

        // Live-tunable: `ros2net param set /talker rate_hz 10.0`.
        ParametersChanged += changes =>
        {
            foreach (var c in changes.Where(c => c.Name == "rate_hz"))
            {
                timer.Period = TimeSpan.FromSeconds(1 / c.NewValue.AsDouble());
            }
        };
    }
}

/// <summary>Prints everything received on <c>chatter</c>.</summary>
public sealed class ListenerNode : Node
{
    public ListenerNode(Ros2Context context)
        : base(context, "listener")
    {
        CreateSubscription<StdMsgs.String>("chatter", msg => Logger.LogInformation("I heard: [{Data}]", msg.Data));
    }
}
