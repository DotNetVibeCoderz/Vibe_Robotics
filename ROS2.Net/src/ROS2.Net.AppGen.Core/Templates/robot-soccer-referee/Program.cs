// {{ProjectName}} — robot soccer with an automatic referee (ROS2.Net template). Runs in lock-step simulation.
using ROS2.Net.Simulation;

if (AppTarget.IsPhysicalRobot)
{
    Console.WriteLine("This app is a simulation study (lock-step physics, ground truth). Run it with a Simulator target.");
    return;
}

var world = new SimWorld(new SimWorldOptions { Name = "arena", Bounds = new Bounds2D(0, 0, 8, 5) });
world.AddObstacle(SimObstacle.Wall("north", new Vec2(0, 5), new Vec2(8, 5)));
world.AddObstacle(SimObstacle.Wall("south", new Vec2(0, 0), new Vec2(8, 0)));
foreach (var (x, name) in new[] { (0.0, "west"), (8.0, "east") })
{
    world.AddObstacle(SimObstacle.Wall(name + "_a", new Vec2(x, 0), new Vec2(x, 1.6)));
    world.AddObstacle(SimObstacle.Wall(name + "_b", new Vec2(x, 3.4), new Vec2(x, 5)));
}

var ball = world.AddObstacle(SimObstacle.Cylinder("ball", 4, 2.5, 0.12, height: 0.24, dynamic: true));
var blue = world.AddRobot(RobotFactory.Create(RobotKind.DiffDrive, "blue", new Pose2D(2, 2.5, 0)));
var red = world.AddRobot(RobotFactory.Create(RobotKind.DiffDrive, "red", new Pose2D(6, 2.5, Math.PI)));
var score = (Blue: 0, Red: 0);

void Chase(SimRobot player, double goalX)
{
    // Aim at a point behind the ball, on the line from the opponent goal through the ball.
    var b = ball.Center;
    var dir = new Vec2(b.X - goalX, b.Y - 2.5).Normalized();
    var target = new Vec2(b.X + (dir.X * 0.25), b.Y + (dir.Y * 0.25));
    var p = player.Pose;
    var bearing = Math.Atan2(target.Y - p.Y, target.X - p.X);
    var error = Math.Atan2(Math.Sin(bearing - p.Theta), Math.Cos(bearing - p.Theta));
    player.SetCommand(Math.Abs(error) < 0.6 ? 0.5 : 0.1, 0, 2.5 * error, world.Time);
}

while (world.Time < 180 && score.Blue + score.Red < 5)
{
    Chase(blue, 8);
    Chase(red, 0);
    world.Step();
    var b = ball.Center;
    if (b.X < 0.05 || b.X > 7.95)
    {
        if (b.X > 7.95) { score.Blue++; } else { score.Red++; }
        Console.WriteLine($"[{world.Time,5:0.0}s] GOAL for {(b.X > 7.95 ? "blue" : "red")}!  blue {score.Blue} – {score.Red} red");
        world.Reset();
    }
}

Console.WriteLine($"Full time: blue {score.Blue} – {score.Red} red ({world.Time:0} s simulated).");
