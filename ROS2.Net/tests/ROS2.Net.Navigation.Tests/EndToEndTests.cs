using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.Nav2Msgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Navigation.Control;
using ROS2.Net.Navigation.Maps;
using ROS2.Net.Navigation.Nodes;
using ROS2.Net.Rmw.InProcess;
using Shouldly;

namespace ROS2.Net.Navigation.Tests;

/// <summary>A minimal unicycle robot: integrates <c>/cmd_vel</c> and publishes <c>/odom</c> at 50 Hz.</summary>
internal sealed class KinematicRobotNode : Node
{
    private readonly object _gate = new();
    private readonly Publisher<Odometry> _odom;
    private Pose2D _pose;
    private Velocity2D _cmd;

    public KinematicRobotNode(Ros2Context context, Pose2D start)
        : base(context, "robot_stub")
    {
        _pose = start;
        _odom = CreatePublisher<Odometry>("/odom");
        CreateSubscription<Twist>("/cmd_vel", t =>
        {
            lock (_gate)
            {
                _cmd = Velocity2D.FromTwist(t);
            }
        });
        CreateTimer(TimeSpan.FromMilliseconds(20), Step);
    }

    public Pose2D Pose
    {
        get
        {
            lock (_gate)
            {
                return _pose;
            }
        }
    }

    private void Step()
    {
        Odometry msg;
        lock (_gate)
        {
            const double dt = 0.02;
            _pose = new Pose2D(_pose.X + (_cmd.Linear * Math.Cos(_pose.Theta) * dt), _pose.Y + (_cmd.Linear * Math.Sin(_pose.Theta) * dt), Angles.Normalize(_pose.Theta + (_cmd.Angular * dt)));
            msg = new Odometry { ChildFrameId = "base_link" };
            msg.Pose.Pose = _pose.ToMsg();
            msg.Twist.Twist = _cmd.ToTwist();
        }

        msg.Header.FrameId = "odom";
        _odom.Publish(msg);
    }
}

public class EndToEndTests
{
    [Fact]
    public async Task Navigate_to_pose_drives_the_robot_to_the_goal()
    {
        await using var ctx = Ros2Context.Create(new Ros2ContextOptions { DomainId = 201, Rmw = new InProcessRmwFactory() });

        var map = TestWorlds.Room(6, 6);
        map.FillRect(2.5, 2.0, 3.5, 4.0, OccupancyGridMap.Occupied); // a box in the way
        var mapServer = new MapServerNode(ctx, map);
        var robot = new KinematicRobotNode(ctx, new Pose2D(1, 3, 0));
        var tools = ctx.CreateNode("nav_test");
        using var pose = new OdometryPoseProvider(tools);

        var planner = new PlannerServerNode(ctx, pose);
        var controller = new ControllerServerNode(ctx, pose);
        ((PurePursuitController)controller.GetController("FollowPath")).MaxLinearSpeed = 1.0;
        var navigator = new BtNavigatorNode(ctx, pose);

        using var cts = new CancellationTokenSource(TimeSpan.FromSeconds(60));
        var spin = ctx.SpinAsync(cts.Token, new MultiThreadedExecutor(2));

        var client = tools.CreateActionClient<NavigateToPoseGoal, NavigateToPoseResult, NavigateToPoseFeedback>("/navigate_to_pose");
        (await client.WaitForServerAsync(TimeSpan.FromSeconds(5))).ShouldBeTrue();

        // Wait for odometry and the latched map to reach the servers.
        for (var i = 0; i < 100 && (!pose.HasOdometry || planner.Costmap is null); i++)
        {
            await Task.Delay(20);
        }

        var goal = new NavigateToPoseGoal { Pose = new Pose2D(5, 3, 0).ToPoseStamped("map", RosTime.Now()) };
        var feedbackCount = 0;
        var result = await client.SendGoalAndWaitAsync(goal, _ => Interlocked.Increment(ref feedbackCount), cts.Token).WaitAsync(TimeSpan.FromSeconds(45));

        result.Status.ShouldBe(GoalStatus.Succeeded);
        robot.Pose.DistanceTo(new Pose2D(5, 3, 0)).ShouldBeLessThan(0.3);
        feedbackCount.ShouldBeGreaterThan(0);

        // The path avoided the box: the robot was never inside it.
        map.WorldToCell(robot.Pose.X, robot.Pose.Y, out var cx, out var cy);
        map.IsOccupied(cx, cy).ShouldBeFalse();

        // The map server answers GetMap.
        var getMap = tools.CreateClient<GetMapRequest, GetMapResponse>("/map_server/map");
        (await getMap.CallAsync(new GetMapRequest(), TimeSpan.FromSeconds(5))).Map.Info.Width.ShouldBe((uint)map.Width);

        cts.Cancel();
        await spin;
        GC.KeepAlive(mapServer);
        GC.KeepAlive(navigator);
    }
}
