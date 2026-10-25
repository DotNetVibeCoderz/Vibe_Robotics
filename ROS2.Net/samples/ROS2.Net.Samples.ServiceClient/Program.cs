// example_interfaces/srv/AddTwoInts: a server node and a client calling it with async/await.
using ROS2.Net;
using ROS2.Net.Msgs.ExampleInterfaces;

await using var context = await Ros2Context.CreateAsync(new Ros2ContextOptions { Arguments = args });

var server = context.CreateNode("add_two_ints_server");
server.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints", request =>
{
    Console.WriteLine($"Incoming request: a={request.A} b={request.B}");
    return new AddTwoIntsResponse { Sum = request.A + request.B };
});

using var cts = new CancellationTokenSource();
var spin = context.SpinAsync(cts.Token);

var clientNode = context.CreateNode("add_two_ints_client");
var client = clientNode.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints");
if (!await client.WaitForServiceAsync(TimeSpan.FromSeconds(5)))
{
    Console.WriteLine("Service not available.");
    return;
}

foreach (var (a, b) in new[] { (2L, 3L), (40L, 2L), (-7L, 7L) })
{
    // Responses complete the task directly: no extra spinning is needed for the client.
    var response = await client.CallAsync(new AddTwoIntsRequest { A = a, B = b }, TimeSpan.FromSeconds(2));
    Console.WriteLine($"Result of add_two_ints: {a} + {b} = {response.Sum}");
}

cts.Cancel();
await spin;
