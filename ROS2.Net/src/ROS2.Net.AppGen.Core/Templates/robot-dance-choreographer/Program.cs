// {{ProjectName}} — synchronized robot dance (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Simulation;

await using var context = await AppTarget.CreateContextAsync(args);
// Simulator: a troupe of four. Physical robot: the one real robot dances (slowly — see the limits).
using var sim = AppTarget.StartSimulator(context, ScenarioKind.Empty, robots: 4);
Console.WriteLine(AppTarget.Describe());

var bpm = 100.0;
var moves = new (string Name, double Linear, double Angular)[]
{
    ("step forward", 0.3, 0), ("spin left", 0, 3.0), ("step back", -0.3, 0), ("spin right", 0, -3.0),
    ("wiggle", 0.0, 6.0), ("wiggle", 0.0, -6.0), ("circle", 0.3, 1.5), ("freeze", 0, 0),
};

var choreographer = context.CreateNode("choreographer");
var publishers = AppTarget.RobotNamespaces().Select(ns => choreographer.CreatePublisher<Twist>($"{ns}/cmd_vel")).ToArray();
var beat = 0;
choreographer.CreateTimer(TimeSpan.FromMinutes(1 / bpm), () =>
{
    var move = moves[beat % moves.Length];
    for (var i = 0; i < publishers.Length; i++)
    {
        // Mirror the even dancers for a symmetric formation.
        var mirror = i % 2 == 0 ? 1 : -1;
        publishers[i].Publish(AppTarget.Limit(new Twist { Linear = new Vector3 { X = move.Linear }, Angular = new Vector3 { Z = move.Angular * mirror } }));
    }

    if (beat % 4 == 0)
    {
        Console.WriteLine($"bar {beat / 4 + 1}: {move.Name}");
    }

    beat++;
});

// Keep the commands alive between beats (commands older than 0.5 s are ignored by the simulator).
var keepAlive = context.CreateNode("metronome");
keepAlive.CreateTimer(TimeSpan.FromMilliseconds(200), () =>
{
    var move = moves[Math.Max(0, beat - 1) % moves.Length];
    for (var i = 0; i < publishers.Length; i++)
    {
        publishers[i].Publish(AppTarget.Limit(new Twist { Linear = new Vector3 { X = move.Linear }, Angular = new Vector3 { Z = move.Angular * (i % 2 == 0 ? 1 : -1) } }));
    }
});

Console.WriteLine($"Dancing at {bpm} BPM — Ctrl+C to stop.");
await context.SpinAsync();
