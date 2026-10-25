using ROS2.Net.AI.Gym;
using ROS2.Net.Gallery.Web.Components.Shared;
using ROS2.Net.Gallery.Web.Services;
using ROS2.Net.Simulation;
using Shouldly;

namespace ROS2.Net.Blazor.Tests;

public class GalleryLogicTests
{
    [Fact]
    public void Q_learning_demo_trains_in_seconds()
    {
        var options = new RobotGymOptions { Scenario = ScenarioKind.Empty, MaxEpisodeSteps = 120, MinGoalDistance = 1.0, MaxGoalDistance = 2.5 };
        var env = new RobotGymEnv(options);
        var agent = AgentFactory.CreateQTable(env, bins: 6, seed: 1);
        var stats = RlTrainer.Train(env, agent, 60, seed: 1);
        stats.Count.ShouldBe(60);
    }

    [Fact]
    public void Wander_turns_away_from_a_close_wall()
    {
        var ranges = Enumerable.Repeat(5f, 360).ToArray();
        for (var i = 340; i < 360; i++)
        {
            ranges[i] = 0.3f; // just right of straight ahead
        }

        for (var i = 0; i < 20; i++)
        {
            ranges[i] = 0.3f;
        }

        var twist = GalleryRuntime.Wander(ranges, 0, Math.PI / 180);
        twist.Linear.X.ShouldBe(0);
        Math.Abs(twist.Angular.Z).ShouldBeGreaterThan(1);
    }

    [Fact]
    public void Code_highlighter_escapes_html_and_marks_tokens()
    {
        var html = CodeBlock.Highlight("var x = \"<b>\"; // done");
        html.ShouldContain("<span class=\"k\">var</span>");
        html.ShouldContain("&lt;b&gt;");
        html.ShouldContain("<span class=\"c\">// done</span>");
    }

    [Fact]
    public void Every_demo_has_both_languages()
    {
        foreach (var d in Demos.All)
        {
            d.TitleId.ShouldNotBeNullOrWhiteSpace();
            d.SummaryId.ShouldNotBe(d.Summary);
        }
    }
}
