// Parameters: declare with descriptors, override from the command line or YAML, validate, and
// change them remotely through the parameter services.
//   dotnet run -- --ros-args -p max_speed:=1.5 --params-file params.yaml
using ROS2.Net;

await using var context = await Ros2Context.CreateAsync(new Ros2ContextOptions { Arguments = args });
var node = context.CreateNode("controller");

var maxSpeed = node.DeclareParameter("max_speed", 0.5, new ParameterDescriptor
{
    Description = "Linear speed limit [m/s]",
    FloatingPointRange = new FloatingPointRange(0.0, 2.0),
});
var robotName = node.DeclareParameter("robot_name", "robot1", "Display name", readOnly: true);
var waypoints = node.DeclareParameter("waypoints", new[] { "dock", "kitchen" });
Console.WriteLine($"max_speed={maxSpeed} robot_name={robotName} waypoints=[{string.Join(", ", waypoints)}]");

node.AddOnSetParametersCallback(changes =>
    changes.Any(p => p.Name == "max_speed" && p.Value.AsDouble() > 1.0 && !node.GetParameter<bool>("use_sim_time"))
        ? SetParametersResult.Failure("speeds above 1.0 m/s are only allowed in simulation")
        : SetParametersResult.Success);
node.ParametersChanged += changes =>
{
    foreach (var c in changes)
    {
        Console.WriteLine($"changed: {c.Name} {c.OldValue} -> {c.NewValue}");
    }
};

using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);

// What `ros2 param set /controller max_speed 0.8` does:
var tool = context.CreateNode("param_tool");
using var client = new ParameterClient(tool, "/controller");
await client.WaitForServiceAsync(TimeSpan.FromSeconds(5));
Console.WriteLine($"set 0.8 -> {(await client.SetAsync("max_speed", 0.8)).Successful}");
var rejected = await client.SetAsync("max_speed", 1.8);
Console.WriteLine($"set 1.8 -> {rejected.Successful} ({rejected.Reason})");
Console.WriteLine($"set robot_name -> {(await client.SetAsync("robot_name", "x")).Reason}");
Console.WriteLine("All parameters:");
foreach (var p in await client.GetAsync(await client.ListAsync()))
{
    Console.WriteLine($"  {p}");
}

Console.WriteLine(ParameterYaml.Dump(node.FullyQualifiedName, node.Parameters));
cts.Cancel();
await spin;
