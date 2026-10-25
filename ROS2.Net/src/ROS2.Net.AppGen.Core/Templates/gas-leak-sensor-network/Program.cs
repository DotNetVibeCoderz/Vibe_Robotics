// {{ProjectName}} — distributed gas-leak detection (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Msgs.DiagnosticMsgs;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

await using var context = await AppTarget.CreateContextAsync(args);

var sensors = new (string Id, double X, double Y)[] { ("s1", 0, 0), ("s2", 10, 0), ("s3", 20, 0), ("s4", 0, 10), ("s5", 10, 10), ("s6", 20, 10) };
var leak = (X: 14.0, Y: 6.0, Start: 8.0);
var clock = 0.0;

// One node per sensor (they could run on separate devices with the RTPS middleware).
foreach (var s in sensors)
{
    var node = context.CreateNode($"gas_{s.Id}");
    var pub = node.CreatePublisher<StdMsgs.Float64>($"/gas/{s.Id}/ppm");
    var rnd = new Random(s.Id.GetHashCode());
    node.CreateTimer(TimeSpan.FromMilliseconds(500), () =>
    {
        var background = 2 + rnd.NextDouble();
        var spread = Math.Max(0, clock - leak.Start) * 0.8;       // the plume grows over time
        var d = Math.Sqrt(Math.Pow(s.X - leak.X, 2) + Math.Pow(s.Y - leak.Y, 2));
        var plume = clock > leak.Start ? 400 * Math.Exp(-d / Math.Max(1, spread)) : 0;
        pub.Publish(new StdMsgs.Float64 { Data = background + plume });
    });
}

var monitor = context.CreateNode("gas_monitor");
var diagnostics = monitor.CreatePublisher<DiagnosticArray>("/diagnostics");
var latest = new Dictionary<string, double>();
foreach (var s in sensors)
{
    monitor.CreateSubscription<StdMsgs.Float64>($"/gas/{s.Id}/ppm", m => latest[s.Id] = m.Data);
}

var lastLevel = "";
monitor.CreateTimer(TimeSpan.FromSeconds(1), () =>
{
    clock += 1;
    if (latest.Count < sensors.Length)
    {
        return;
    }

    var max = latest.Values.Max();
    var level = max > 100 ? "EVACUATE" : max > 25 ? "WARNING" : "normal";
    var weights = sensors.Select(s => (s, w: Math.Max(0, latest[s.Id] - 3))).ToList();
    var total = weights.Sum(x => x.w);
    var source = total > 0 ? (X: weights.Sum(x => x.s.X * x.w) / total, Y: weights.Sum(x => x.s.Y * x.w) / total) : (X: double.NaN, Y: double.NaN);
    if (level != lastLevel || level != "normal")
    {
        Console.WriteLine($"[{clock,3:0}s] {level,-8} max {max,6:0.0} ppm" + (total > 0 ? $", estimated source ({source.X:0.0}, {source.Y:0.0})" : string.Empty));
        lastLevel = level;
    }

    diagnostics.Publish(new DiagnosticArray
    {
        Status = sensors.Select(s => new DiagnosticStatus
        {
            Name = $"gas/{s.Id}",
            Level = latest[s.Id] > 100 ? DiagnosticStatus.ERROR : latest[s.Id] > 25 ? DiagnosticStatus.WARN : DiagnosticStatus.OK,
            Message = $"{latest[s.Id]:0.0} ppm",
            HardwareId = s.Id,
        }).ToArray(),
    });
});

Console.WriteLine("Gas network online (the leak starts at t=8 s) — Ctrl+C to stop.");
await context.SpinAsync();
