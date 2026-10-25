using Shouldly;

namespace ROS2.Net.Tests.Core;

public class NamesAndTimeTests
{
    [Theory]
    [InlineData("chatter", "talker", "/", "/chatter")]
    [InlineData("chatter", "talker", "/robot", "/robot/chatter")]
    [InlineData("/abs", "talker", "/robot", "/abs")]
    [InlineData("~/private", "talker", "/robot", "/robot/talker/private")]
    [InlineData("~", "talker", "/", "/talker")]
    [InlineData("{node}/state", "arm", "/r", "/r/arm/state")]
    public void Expands_names(string name, string node, string ns, string expected) =>
        RosNames.ExpandTopicName(name, node, ns).ShouldBe(expected);

    [Theory]
    [InlineData("bad name")]
    [InlineData("/a//b")]
    [InlineData("/1abc")]
    [InlineData("/trailing/")]
    public void Rejects_invalid_names(string name) =>
        Should.Throw<InvalidRosNameException>(() => RosNames.ExpandTopicName(name, "n", "/"));

    [Fact]
    public void Remaps_by_written_or_expanded_name()
    {
        var rules = new[] { RemapRule.Parse("chatter:=/news"), RemapRule.Parse("other:cmd:=/x") };
        RosNames.Remap("/chatter", "chatter", "talker", "/", rules).ShouldBe("/news");
        RosNames.Remap("/cmd", "cmd", "talker", "/", rules).ShouldBe("/cmd");
        RosNames.Remap("/cmd", "cmd", "other", "/", rules).ShouldBe("/x");
    }

    [Fact]
    public async Task Node_applies_remaps_to_topics()
    {
        await using var ctx = TestDomains.Create(args: ["--ros-args", "-r", "chatter:=news"]);
        var node = ctx.CreateNode("talker");
        node.CreatePublisher<Msgs.StdMsgs.String>("chatter").TopicName.ShouldBe("/news");
    }

    [Fact]
    public void Time_converts_to_and_from_messages()
    {
        var t = new RosTime(1_700_000_000_123_456_789);
        var msg = t.ToMsg();
        msg.Sec.ShouldBe(1_700_000_000);
        msg.Nanosec.ShouldBe(123_456_789u);
        RosTime.FromMsg(msg).ShouldBe(t);

        var d = RosDuration.FromSeconds(-1.5);
        RosDuration.FromMsg(d.ToMsg()).ShouldBe(d);
    }

    [Fact]
    public async Task Sim_clock_waits_follow_set_time()
    {
        var clock = new RosClock();
        clock.EnableSimTime();
        clock.SetSimTime(RosTime.FromSeconds(10));
        var wait = clock.WaitUntilAsync(RosTime.FromSeconds(11), CancellationToken.None).AsTask();
        await Task.Delay(50);
        wait.IsCompleted.ShouldBeFalse();
        clock.SetSimTime(RosTime.FromSeconds(11));
        await wait.WithTimeout(2000);
        clock.Now().Seconds.ShouldBe(11);
    }
}
