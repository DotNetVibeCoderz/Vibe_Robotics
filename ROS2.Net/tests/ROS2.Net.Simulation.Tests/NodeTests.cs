using ROS2.Net.Msgs;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Msgs.RosgraphMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.StdSrvs;
using ROS2.Net.Msgs.Tf2Msgs;
using ROS2.Net.Rmw.InProcess;
using Shouldly;
using Pose2D = ROS2.Net.Simulation.Pose2D;
using TurtlePose = ROS2.Net.Msgs.Turtlesim.Pose;

namespace ROS2.Net.Simulation.Tests;

internal static class Ctx
{
    private static int s_domain = 150;

    public static Ros2Context Create() => Ros2Context.Create(new Ros2ContextOptions
    {
        DomainId = Interlocked.Increment(ref s_domain) % 230,
        Rmw = new InProcessRmwFactory(),
    });
}

public class NodeTests
{
    [Fact]
    public async Task Simulation_node_publishes_sensor_topics_and_obeys_cmd_vel()
    {
        await using var ctx = Ctx.Create();
        var world = Scenarios.Create(ScenarioKind.Empty, robots: 1);
        using var sim = new SimulationNode(ctx, world, new SimulationNodeOptions { RunWorld = false, MapResolution = 0.2 });
        var client = ctx.CreateNode("client");
        var odom = client.CreateSubscription<Odometry>("/robot1/odom", QosProfile.Default.KeepLast(1000));
        var scan = client.CreateSubscription<LaserScan>("/robot1/scan", QosProfile.SensorData.KeepLast(100));
        var clock = client.CreateSubscription<Clock>("/clock", QosProfile.Default.KeepLast(1000));
        var tf = client.CreateSubscription<TFMessage>("/tf", QosProfile.Default.KeepLast(1000));
        var tfStatic = client.CreateSubscription<TFMessage>("/tf_static", QosProfile.Latched);
        var map = client.CreateSubscription<OccupancyGrid>("/map", QosProfile.Latched);
        var cmd = client.CreatePublisher<Twist>("/robot1/cmd_vel");

        for (var i = 0; i < 100; i++)
        {
            cmd.Publish(MessageExtensions.Twist2D(0.2, 0));
            world.Step();
        }

        Odometry? last = null;
        while (odom.TryTake(out var o))
        {
            last = o;
        }

        last.ShouldNotBeNull();
        last.Pose.Pose.Position.X.ShouldBeGreaterThan(0.1);
        last.ChildFrameId.ShouldBe("robot1/base_link");
        last.Header.FrameId.ShouldBe("robot1/odom");

        scan.TryTake(out var s).ShouldBeTrue();
        s!.Ranges.Length.ShouldBe(360);
        s.Header.FrameId.ShouldBe("robot1/laser_link");

        Clock? lastClock = null;
        while (clock.TryTake(out var c))
        {
            lastClock = c;
        }

        RosTime.FromMsg(lastClock!.ClockValue).Seconds.ShouldBe(1.0, 0.011);

        tf.TryTake(out var t).ShouldBeTrue();
        t!.Transforms.ShouldContain(x => x.ChildFrameId == "robot1/base_link" && x.Header.FrameId == "robot1/odom");
        (await tfStatic.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(2))).Transforms
            .ShouldContain(x => x.ChildFrameId == "robot1/laser_link");
        (await map.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(2))).Info.Width.ShouldBe(100u);
    }

    [Fact]
    public async Task Simulation_services_reset_pause_and_spawn()
    {
        await using var ctx = Ctx.Create();
        var world = Scenarios.Create(ScenarioKind.Empty, robots: 1);
        using var sim = new SimulationNode(ctx, world, new SimulationNodeOptions { RunWorld = false, PublishMap = false });
        var client = ctx.CreateNode("client");
        var reset = client.CreateClient<EmptyRequest, EmptyResponse>("/sim/reset");
        var pause = client.CreateClient<SetBoolRequest, SetBoolResponse>("/sim/pause");
        var spawn = client.CreateClient<Msgs.Turtlesim.SpawnRequest, Msgs.Turtlesim.SpawnResponse>("/sim/spawn");

        using var cts = new CancellationTokenSource();
        var spin = sim.SpinAsync(cts.Token);

        var robot = world.Robots[0];
        for (var i = 0; i < 50; i++)
        {
            robot.SetCommand(0.2, 0, 0, world.Time);
            world.Step();
        }

        robot.Pose.X.ShouldBeGreaterThan(0.01);
        await reset.CallAsync(new EmptyRequest(), TimeSpan.FromSeconds(2));
        robot.Pose.X.ShouldBe(0, 1e-9);
        world.Time.ShouldBe(0);

        (await pause.CallAsync(new SetBoolRequest { Data = true }, TimeSpan.FromSeconds(2))).Success.ShouldBeTrue();
        world.Paused.ShouldBeTrue();

        var res = await spawn.CallAsync(new Msgs.Turtlesim.SpawnRequest { X = 2, Y = 1 }, TimeSpan.FromSeconds(2));
        res.Name.ShouldBe("robot2");
        world.Robots.Count.ShouldBe(2);
        client.GetTopicNamesAndTypes().ShouldContain(t => t.Name == "/robot2/odom");

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Simulation_node_runs_in_real_time()
    {
        await using var ctx = Ctx.Create();
        var world = Scenarios.Create(ScenarioKind.Empty, robots: 1);
        world.RealTimeFactor = 4;
        using var sim = new SimulationNode(ctx, world, new SimulationNodeOptions { PublishMap = false });
        sim.Start();
        await Task.Delay(300);
        await sim.StopAsync();
        world.Time.ShouldBeGreaterThan(0.4);
        world.Time.ShouldBeLessThan(2.5);
    }

    [Fact]
    public async Task Turtlesim_moves_turtles_and_spawns_new_ones()
    {
        await using var ctx = Ctx.Create();
        using var turtlesim = new TurtlesimNode(ctx);
        var client = ctx.CreateNode("teleop");
        var cmd = client.CreatePublisher<Twist>("/turtle1/cmd_vel");
        var pose = client.CreateSubscription<TurtlePose>("/turtle1/pose", QosProfile.Default.KeepLast(1000));

        cmd.Publish(MessageExtensions.Twist2D(2.0, 0));
        for (var i = 0; i < 30; i++)
        {
            turtlesim.Tick();
        }

        TurtlePose? last = null;
        while (pose.TryTake(out var p))
        {
            last = p;
        }

        last!.X.ShouldBe((float)(TurtleWorld.Size / 2 + (2.0 * 0.016 * 30)), 0.01f);
        turtlesim.World.Snapshot().Segments.Count.ShouldBeGreaterThan(0);

        var spawn = client.CreateClient<Msgs.Turtlesim.SpawnRequest, Msgs.Turtlesim.SpawnResponse>("/spawn");
        using var cts = new CancellationTokenSource();
        var spin = turtlesim.SpinAsync(cts.Token);
        (await spawn.CallAsync(new Msgs.Turtlesim.SpawnRequest { X = 1, Y = 1 }, TimeSpan.FromSeconds(2))).Name.ShouldBe("turtle2");
        client.GetTopicNamesAndTypes().ShouldContain(t => t.Name == "/turtle2/pose");
        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Turtlesim_rotate_absolute_action_reaches_heading()
    {
        await using var ctx = Ctx.Create();
        using var turtlesim = new TurtlesimNode(ctx);
        turtlesim.Start();
        var client = ctx.CreateNode("c").CreateActionClient<Msgs.Turtlesim.RotateAbsoluteGoal, Msgs.Turtlesim.RotateAbsoluteResult, Msgs.Turtlesim.RotateAbsoluteFeedback>("/turtle1/rotate_absolute");
        using var cts = new CancellationTokenSource();
        var spin = turtlesim.SpinAsync(cts.Token);
        await client.WaitForServerAsync(TimeSpan.FromSeconds(2));
        var result = await client.SendGoalAndWaitAsync(new Msgs.Turtlesim.RotateAbsoluteGoal { Theta = 1.0f }).WaitAsync(TimeSpan.FromSeconds(10));
        result.Status.ShouldBe(GoalStatus.Succeeded);
        turtlesim.World.Get("turtle1")!.Pose.Theta.ShouldBe(1.0, 0.05);
        cts.Cancel();
        await spin;
    }

    [Fact]
    public void Use_of_pose2d_alias_compiles()
    {
        new Pose2D(1, 2, 3).Position.ShouldBe(new Vec2(1, 2));
    }
}
