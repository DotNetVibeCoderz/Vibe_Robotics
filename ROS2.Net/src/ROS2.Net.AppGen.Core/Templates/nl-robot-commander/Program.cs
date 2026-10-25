// {{ProjectName}} — natural-language robot commander (ROS2.Net template).
// Uses the offline RuleBasedChatModel by default; set ROS2NET_LLM_ENDPOINT/MODEL/API_KEY and switch to
// OpenAICompatibleChatModel to use Ollama, OpenAI, Azure OpenAI or any /v1/chat/completions server.
using ROS2.Net;
using ROS2.Net.AI.Llm;
using ROS2.Net.Simulation;

await using var context = await AppTarget.CreateContextAsync(args);
using var sim = AppTarget.StartSimulator(context, ScenarioKind.Warehouse);
Console.WriteLine(AppTarget.Describe());

// The commander's SafetyLimiter clamps speeds and stops for obstacles seen on the scan.
var options = new CommanderOptions { CmdVelTopic = AppTarget.Topic("cmd_vel"), ScanTopic = AppTarget.Topic("scan") };
foreach (var (name, p) in AppTarget.World?.Waypoints ?? [])
{
    options.Waypoints[name] = (p.X, p.Y, p.Theta);
}

IChatModel model = Environment.GetEnvironmentVariable("ROS2NET_LLM_ENDPOINT") is { Length: > 0 }
    ? new OpenAICompatibleChatModel()
    : new RuleBasedChatModel();
using var commander = new NaturalLanguageCommanderNode(context, model, options);
commander.Feedback += text => Console.WriteLine("  robot: " + text);

using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);
Console.WriteLine("Type a command (e.g. 'forward 1 m then turn right 90', 'maju 50 cm lalu belok kiri'), or 'quit'.");
while (Console.ReadLine() is { } line && !line.Equals("quit", StringComparison.OrdinalIgnoreCase))
{
    if (line.Trim().Length == 0)
    {
        continue;
    }

    var actions = await commander.HandleCommandAsync(line);
    Console.WriteLine($"  executed: {string.Join(", ", actions)}");
    if (AppTarget.World is { } w)
    {
        var r = w.Robots[0];
        Console.WriteLine($"  pose: ({r.Pose.X:0.00}, {r.Pose.Y:0.00}) heading {r.Pose.Theta * 180 / Math.PI:0}°");
    }
}

cts.Cancel();
await spin;
