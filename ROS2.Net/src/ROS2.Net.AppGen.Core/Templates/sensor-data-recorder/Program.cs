// {{ProjectName}} — record a dataset from a simulated robot and export it (ROS2.Net template).
using ROS2.Net;
using ROS2.Net.Bag;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Simulation;

var seconds = args.Length > 0 && double.TryParse(args[0], out var s) ? s : 10.0;
var bagDir = Path.Combine(Environment.CurrentDirectory, "bags", $"run_{DateTime.Now:yyyyMMdd_HHmmss}");

await using (var context = await AppTarget.CreateContextAsync(args))
{
    using var sim = AppTarget.StartSimulator(context, ScenarioKind.Office);
    Console.WriteLine(AppTarget.Describe());

    // Only the simulated robot is driven automatically; a real robot is recorded as you drive it.
    if (AppTarget.IsSimulator)
    {
        var driver = context.CreateNode("driver");
        var cmd = driver.CreatePublisher<Twist>(AppTarget.Topic("cmd_vel"));
        driver.CreateTimer(TimeSpan.FromMilliseconds(100), () => cmd.Publish(new Twist { Linear = new Vector3 { X = 0.25 }, Angular = new Vector3 { Z = 0.3 } }));
    }

    var recorderNode = context.CreateNode("recorder");
    await using var recorder = new BagRecorder(recorderNode, bagDir, new BagRecorderOptions
    {
        Topics = { AppTarget.Topic("imu"), AppTarget.Topic("odom"), AppTarget.Topic("scan") },
    });
    recorder.Start();

    using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(seconds));
    Console.WriteLine($"Recording {seconds:0} s to {bagDir} ...");
    await context.SpinAsync(cts.Token);
    await recorder.StopAsync();
    Console.WriteLine($"Recorded {recorder.MessageCount} messages.");
}

Console.WriteLine(BagInfo.Read(bagDir));

using var bag = BagReader.Open(bagDir);
var csv = Path.Combine(bagDir, "imu.csv");
var jsonl = Path.Combine(bagDir, "odom.jsonl");
var rows = BagExporter.ToCsvFile(bag, AppTarget.Topic("imu"), csv);
var lines = BagExporter.ToJsonLinesFile(bag, jsonl, [AppTarget.Topic("odom")]);
Console.WriteLine($"Exported {rows} IMU rows to {csv} and {lines} odometry records to {jsonl}.");
