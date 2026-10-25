using Ros2NetSimulator;
using Shouldly;

namespace ROS2.Net.Avalonia.Tests;

/// <summary>The simulator window starts a world on the in-process middleware, publishes topics and renders.</summary>
[Collection("headless")]
public class SimulatorTests
{
    [Fact]
    [Trait("Category", "Screenshots")]
    public void Simulator_runs_and_renders_to_png()
    {
        Headless.Run(() =>
        {
            RosTheme.IsDark = true;
            var window = new SimulatorWindow("inprocess", "Warehouse") { Width = 1440, Height = 900 };
            window.Show();
            Headless.Pump(300);
            window.Ready.IsCompleted.ShouldBeTrue();
            window.World.ShouldNotBeNull();
            window.World!.Robots.Count.ShouldBe(2);

            // Drive robot1 so the picture shows motion, trails and scans.
            var r1 = window.World.Robots[0];
            var r2 = window.World.Robots[1];
            var end = Environment.TickCount64 + 3500;
            while (Environment.TickCount64 < end)
            {
                r1.SetCommand(0.2, 0, 0.35, window.World.Time);
                r2.SetCommand(0.18, 0, -0.25, window.World.Time);
                Headless.Pump(100);
            }

            window.World.Time.ShouldBeGreaterThan(2);
            var topics = window.Context!.Nodes[0].GetTopicNamesAndTypes().Select(t => t.Name).ToList();
            topics.ShouldContain("/robot1/scan");
            topics.ShouldContain("/robot2/odom");
            topics.ShouldContain("/clock");

            Headless.Capture(window, Path.Combine(Headless.RepoRoot, "docs", "images", "desktop", "simulator.png"));
            window.Close();
        });
    }
}
