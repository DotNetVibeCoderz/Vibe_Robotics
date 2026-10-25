// {{ProjectName}} — generative turtle art (ROS2.Net template).
using ROS2.Net.Simulation;

if (AppTarget.IsPhysicalRobot)
{
    Console.WriteLine("This app is a simulation study (lock-step physics, ground truth). Run it with a Simulator target.");
    return;
}

var world = new TurtleWorld();
var turtle = world.Turtles[0].Name;
var palette = new[] { new SimColor(255, 99, 132), new SimColor(124, 77, 255), new SimColor(0, 209, 255), new SimColor(61, 220, 151), new SimColor(255, 183, 77) };

void Run(double linear, double angular, double seconds)
{
    world.Command(turtle, linear, angular);
    var steps = (int)(seconds / world.StepSize);
    for (var i = 0; i < steps; i++)
    {
        world.Step();
    }
}

for (var petal = 0; petal < 10; petal++)
{
    var angle = petal * Math.PI / 5;
    world.SetPen(turtle, palette[petal % palette.Length], 3, off: true);
    world.Teleport(turtle, new Pose2D(5.5 + (2.5 * Math.Cos(angle)), 5.5 + (2.5 * Math.Sin(angle)), angle));
    world.SetPen(turtle, palette[petal % palette.Length], 3, off: false);

    // A five-pointed star: 144° turns.
    for (var side = 0; side < 5; side++)
    {
        Run(1.2, 0, 1.0);
        Run(0, 4 * Math.PI / 5, 1.0);
    }
}

var snapshot = world.Snapshot();
Console.WriteLine($"Drew {snapshot.Segments.Count} pen segments with {world.Turtles.Count} turtle(s) in {world.Time:0.0} simulated seconds.");
