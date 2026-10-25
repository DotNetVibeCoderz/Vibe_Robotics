// {{ProjectName}} — battery-aware behavior tree (ROS2.Net template).
using ROS2.Net.Navigation.BehaviorTrees;

var battery = 1.0;          // state of charge, 0..1
var atDock = false;
var patrolStops = new[] { "kitchen", "living_room", "bedroom", "hallway" };
var stop = 0;

const string Xml = """
    <root BTCPP_format="4">
      <BehaviorTree ID="Housekeeper">
        <Fallback>
          <Sequence>
            <BatteryLow threshold="0.25"/>
            <GoToDock/>
            <Charge/>
          </Sequence>
          <Patrol/>
        </Fallback>
      </BehaviorTree>
    </root>
    """;

var factory = new BtFactory()
    .RegisterCondition("BatteryLow", n => battery < n.GetInput("threshold", 0.25))
    .RegisterAsyncAction("GoToDock", async (_, ct) =>
    {
        if (!atDock)
        {
            Console.WriteLine($"  battery {battery:P0}: heading to the dock");
            await Task.Delay(300, ct);
            atDock = true;
        }

        return NodeStatus.Success;
    })
    .RegisterAction("Charge", _ =>
    {
        battery = Math.Min(1.0, battery + 0.2);
        Console.WriteLine($"  charging... {battery:P0}");
        if (battery >= 1.0)
        {
            atDock = false;
            return NodeStatus.Success;
        }

        return NodeStatus.Running;
    })
    .RegisterAsyncAction("Patrol", async (_, ct) =>
    {
        await Task.Delay(100, ct);
        battery -= 0.07;
        Console.WriteLine($"Patrolling {patrolStops[stop++ % patrolStops.Length]} (battery {battery:P0})");
        return NodeStatus.Success;
    });

var tree = factory.CreateTreeFromText(Xml);
for (var tick = 0; tick < 60; tick++)
{
    await tree.TickWhileRunningAsync(TimeSpan.FromMilliseconds(50));
}

Console.WriteLine("Shift finished.");
