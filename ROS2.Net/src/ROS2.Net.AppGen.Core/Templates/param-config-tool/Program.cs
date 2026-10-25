// {{ProjectName}} — remote parameter management (ROS2.Net template).
using ROS2.Net;

await using var context = await AppTarget.CreateContextAsync(args);

// A demo node with typed, validated parameters (replace with any node on the network).
var controller = context.CreateNode("controller", "/robot1");
controller.DeclareParameter("max_speed", 0.5, new ParameterDescriptor { Description = "m/s", FloatingPointRange = new FloatingPointRange(0, 2) });
controller.DeclareParameter("pid.kp", 1.2);
controller.DeclareParameter("pid.ki", 0.1);
controller.DeclareParameter("frame_id", "base_link", "TF frame of the robot", readOnly: true);

using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);

var tool = context.CreateNode("param_tool");
using var client = new ParameterClient(tool, "/robot1/controller");
await client.WaitForServiceAsync(TimeSpan.FromSeconds(5));

Console.WriteLine("Parameters:");
foreach (var d in await client.DescribeAsync(await client.ListAsync()))
{
    var value = await client.GetAsync(d.Name);
    Console.WriteLine($"  {d.Name,-14} {d.Type,-8} = {value,-10} {(d.ReadOnly ? "(read-only) " : string.Empty)}{d.Description}");
}

const string Yaml = """
    /robot1/controller:
      ros__parameters:
        max_speed: 1.1
        pid:
          kp: 2.5
          ki: 0.2
    """;
var updates = ParameterYaml.Load(Yaml, "/robot1/controller");
var result = await client.SetAtomicallyAsync(updates);
Console.WriteLine($"\nLoaded {updates.Count} values atomically: {(result.Successful ? "ok" : result.Reason)}");
Console.WriteLine($"Rejected change: {(await client.SetAsync("max_speed", 9.0)).Reason}");

Console.WriteLine("\nDump:");
Console.WriteLine(ParameterYaml.Dump(controller.FullyQualifiedName, await client.GetAsync(await client.ListAsync())));
cts.Cancel();
await spin;
