using ROS2.Net.Navigation.Behaviors;
using ROS2.Net.Navigation.BehaviorTrees;
using ROS2.Net.Navigation.Control;
using ROS2.Net.Navigation.Planning;
using Shouldly;

namespace ROS2.Net.Navigation.Tests;

public class ControlAndBtTests
{
    private static Pose2D Integrate(Pose2D p, Velocity2D v, double dt) =>
        new(p.X + (v.Linear * Math.Cos(p.Theta) * dt), p.Y + (v.Linear * Math.Sin(p.Theta) * dt), Angles.Normalize(p.Theta + (v.Angular * dt)));

    private static List<Pose2D> LPath()
    {
        var pts = new List<(double X, double Y)>();
        for (var x = 0.0; x <= 3.0; x += 0.05)
        {
            pts.Add((x, 0));
        }

        for (var y = 0.05; y <= 3.0; y += 0.05)
        {
            pts.Add((3, y));
        }

        return PathUtils.OrientAlongPath(pts, Math.PI / 2);
    }

    public static TheoryData<string> Controllers => ["PurePursuit", "RegulatedPurePursuit", "DWB"];

    [Theory]
    [MemberData(nameof(Controllers))]
    public void Controllers_follow_a_curved_path_to_the_goal(string name)
    {
        IController controller = name switch
        {
            "PurePursuit" => new PurePursuitController(),
            "RegulatedPurePursuit" => new RegulatedPurePursuitController(),
            _ => new DwbController(),
        };
        var path = LPath();
        controller.SetPlan(path);
        var checker = new GoalChecker();
        var pose = new Pose2D(0, 0.1, 0);
        var vel = Velocity2D.Zero;
        var maxDeviation = 0.0;
        var steps = 0;
        for (; steps < 2000 && !checker.IsGoalReached(pose, path[^1]); steps++)
        {
            vel = controller.ComputeVelocity(pose, vel, null);
            pose = Integrate(pose, vel, 0.05);
            maxDeviation = Math.Max(maxDeviation, path.Min(p => p.DistanceTo(pose)));
        }

        checker.IsGoalReached(pose, path[^1]).ShouldBeTrue($"{name} ended at {pose}");
        maxDeviation.ShouldBeLessThan(0.45);
    }

    [Fact]
    public void Regulated_pure_pursuit_slows_down_on_approach()
    {
        var path = PathUtils.OrientAlongPath(Enumerable.Range(0, 41).Select(i => (i * 0.05, 0.0)).ToList(), 0);
        var rpp = new RegulatedPurePursuitController { MaxLinearSpeed = 0.5 };
        rpp.SetPlan(path);
        var far = rpp.ComputeVelocity(new Pose2D(0, 0, 0), new Velocity2D(0.5, 0), null);
        var near = rpp.ComputeVelocity(new Pose2D(1.7, 0, 0), new Velocity2D(0.5, 0), null);
        near.Linear.ShouldBeLessThan(far.Linear);
    }

    [Fact]
    public void Progress_checker_detects_a_stuck_robot()
    {
        var pc = new ProgressChecker(0.5, 2);
        pc.Check(new Pose2D(0, 0, 0), 0).ShouldBeTrue();
        pc.Check(new Pose2D(0.1, 0, 0), 1.5).ShouldBeTrue();
        pc.Check(new Pose2D(0.1, 0, 0), 2.5).ShouldBeFalse();
        pc.Check(new Pose2D(1, 0, 0), 3).ShouldBeTrue();
    }

    [Fact]
    public void Spin_and_backup_recoveries_complete()
    {
        var spin = new SpinBehavior(Math.PI / 2);
        var pose = Pose2D.Identity;
        spin.Start(pose);
        BehaviorStatus s;
        var i = 0;
        do
        {
            s = spin.Update(pose, 0.05, out var cmd);
            pose = Integrate(pose, cmd, 0.05);
        }
        while (s == BehaviorStatus.Running && ++i < 1000);
        s.ShouldBe(BehaviorStatus.Succeeded);
        pose.Theta.ShouldBe(Math.PI / 2, 0.06);

        var back = new BackUpBehavior(0.3, 0.1);
        pose = Pose2D.Identity;
        back.Start(pose);
        i = 0;
        do
        {
            s = back.Update(pose, 0.05, out var cmd);
            pose = Integrate(pose, cmd, 0.05);
        }
        while (s == BehaviorStatus.Running && ++i < 1000);
        s.ShouldBe(BehaviorStatus.Succeeded);
        pose.X.ShouldBeLessThan(-0.29);

        var blocked = new BackUpBehavior(0.3) { IsColliding = _ => true };
        blocked.Start(Pose2D.Identity);
        blocked.Update(Pose2D.Identity, 0.05, out _).ShouldBe(BehaviorStatus.Failed);
    }

    [Fact]
    public void Bt_xml_loads_and_ticks_with_ports_and_blackboard()
    {
        var factory = new BtFactory();
        var log = new List<string>();
        var attempts = 0;
        factory.RegisterAction("Say", n =>
        {
            log.Add(n.GetInput("text", "?"));
            return NodeStatus.Success;
        });
        factory.RegisterAction("Flaky", _ => ++attempts < 3 ? NodeStatus.Failure : NodeStatus.Success);
        factory.RegisterCondition("HasKey", n => n.Blackboard.Contains(n.GetInput("key", "")));
        factory.RegisterAction("Store", n =>
        {
            n.SetOutput("out", 42);
            return NodeStatus.Success;
        });

        const string xml = """
            <root BTCPP_format="4" main_tree_to_execute="Main">
              <BehaviorTree ID="Main">
                <Sequence>
                  <Say text="start"/>
                  <RetryUntilSuccessful num_attempts="5"><Flaky/></RetryUntilSuccessful>
                  <Store out="{answer}"/>
                  <HasKey key="answer"/>
                  <SubTree ID="Tail"/>
                </Sequence>
              </BehaviorTree>
              <BehaviorTree ID="Tail">
                <Fallback>
                  <Inverter><AlwaysSuccess/></Inverter>
                  <Action ID="Say" text="tail"/>
                </Fallback>
              </BehaviorTree>
            </root>
            """;
        var tree = factory.CreateTreeFromText(xml);
        tree.TickOnce().ShouldBe(NodeStatus.Success);
        log.ShouldBe(new[] { "start", "tail" });
        attempts.ShouldBe(3);
        tree.Blackboard.Get<int>("answer").ShouldBe(42);
    }

    [Fact]
    public async Task Async_actions_run_and_reactive_sequences_halt_them()
    {
        var factory = new BtFactory();
        var halted = false;
        var allow = true;
        factory.RegisterCondition("Allowed", _ => allow);
        factory.RegisterAsyncAction("LongTask", async (_, ct) =>
        {
            try
            {
                await Task.Delay(Timeout.Infinite, ct);
            }
            catch (OperationCanceledException)
            {
                halted = true;
            }

            return NodeStatus.Success;
        });
        var tree = factory.CreateTreeFromText("""
            <root BTCPP_format="4"><BehaviorTree ID="T">
              <ReactiveSequence><Allowed/><LongTask/></ReactiveSequence>
            </BehaviorTree></root>
            """);
        tree.TickOnce().ShouldBe(NodeStatus.Running);
        allow = false;
        tree.TickOnce().ShouldBe(NodeStatus.Failure);
        await Task.Delay(50);
        halted.ShouldBeTrue();
    }

    [Fact]
    public void Recovery_and_pipeline_nodes_follow_nav2_semantics()
    {
        var factory = new BtFactory();
        var mainCalls = 0;
        var recoveries = 0;
        factory.RegisterAction("Main", _ => ++mainCalls < 3 ? NodeStatus.Failure : NodeStatus.Success);
        factory.RegisterAction("Recover", _ =>
        {
            recoveries++;
            return NodeStatus.Success;
        });
        var tree = factory.CreateTreeFromText("""
            <root><BehaviorTree ID="T">
              <RecoveryNode number_of_retries="5"><Main/><Recover/></RecoveryNode>
            </BehaviorTree></root>
            """);
        tree.TickOnce().ShouldBe(NodeStatus.Success);
        (mainCalls, recoveries).ShouldBe((3, 2));

        // The default Nav2 tree parses once its leaves are registered.
        var nav = new BtFactory();
        foreach (var id in new[] { "ComputePathToPose", "FollowPath", "ClearEntireCostmap", "Spin", "Wait", "BackUp" })
        {
            nav.RegisterAction(id, _ => NodeStatus.Success);
        }

        nav.RegisterCondition("GoalUpdated", _ => false);
        var navTree = nav.CreateTreeFromText(NavTrees.NavigateToPoseWithReplanningAndRecovery);
        navTree.TickOnce().ShouldBe(NodeStatus.Success);
        navTree.Nodes().Count().ShouldBeGreaterThan(15);
    }

    [Fact]
    public void Unknown_nodes_and_bad_xml_are_reported()
    {
        var factory = new BtFactory();
        Should.Throw<BehaviorTreeException>(() => factory.CreateTreeFromText("<root><BehaviorTree ID=\"T\"><Nope/></BehaviorTree></root>"))
            .Message.ShouldContain("Nope");
        Should.Throw<BehaviorTreeException>(() => factory.CreateTreeFromText("<tree/>"));
        Should.Throw<BehaviorTreeException>(() => factory.CreateTreeFromText("<root><BehaviorTree ID=\"T\"><Inverter/></BehaviorTree></root>"));
    }
}
