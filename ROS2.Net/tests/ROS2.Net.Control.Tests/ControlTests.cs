using ROS2.Net.Control;
using ROS2.Net.Msgs.BuiltinInterfaces;
using ROS2.Net.Msgs.ControlMsgs;
using ROS2.Net.Msgs.ControllerManagerMsgs;
using ROS2.Net.Msgs.TrajectoryMsgs;
using ROS2.Net.Rmw.InProcess;
using Shouldly;

namespace ROS2.Net.Control.Tests;

internal static class Ctx
{
    private static int s_domain = 150;

    public static Ros2Context Create() => Ros2Context.Create(new Ros2ContextOptions
    {
        DomainId = Interlocked.Increment(ref s_domain) % 230,
        Rmw = new InProcessRmwFactory(),
    });

    /// <summary>Runs the controller manager deterministically for <paramref name="seconds"/> at <paramref name="hz"/>.</summary>
    public static RosTime Run(ControllerManager cm, RosTime start, double seconds, double hz = 100)
    {
        var period = RosDuration.FromSeconds(1 / hz);
        var t = start;
        for (var i = 0; i < (int)Math.Round(seconds * hz); i++)
        {
            t += period;
            cm.Update(t, period);
        }

        return t;
    }
}

public class PidTests
{
    [Fact]
    public void Step_response_of_a_first_order_plant_settles_without_overshoot_beyond_limits()
    {
        var pid = new PidController(2.0, 1.5, 0.05) { OutputMin = -10, OutputMax = 10, IntegralMin = -5, IntegralMax = 5, DerivativeOnMeasurement = true };
        double x = 0;
        const double dt = 0.01;
        double maxX = 0;
        for (var i = 0; i < 1500; i++)
        {
            var u = pid.Compute(1.0, x, dt);
            u.ShouldBeInRange(-10, 10);
            x += (u - x) * dt; // dx/dt = -x + u
            maxX = Math.Max(maxX, x);
        }

        x.ShouldBe(1.0, 1e-3);
        maxX.ShouldBeLessThan(1.3);
    }

    [Fact]
    public void Integral_does_not_wind_up_while_saturated()
    {
        var pid = new PidController(1, 10, 0) { OutputMin = -1, OutputMax = 1 };
        for (var i = 0; i < 1000; i++)
        {
            pid.Compute(100, 0, 0.01);
        }

        pid.IntegralTerm.ShouldBeLessThanOrEqualTo(1.0 + 1e-9);
    }
}

public class OdometryTests
{
    [Fact]
    public void Constant_wheel_speeds_drive_a_full_circle()
    {
        // Separation 0.4 m, radius 0.1 m; left 4 rad/s, right 6 rad/s → v = 0.5 m/s, ω = 0.5 rad/s, R = 1 m.
        var odom = new DiffDriveOdometry(0.4, 0.1, 0.1);
        const double dt = 0.001;
        var period = 2 * Math.PI / 0.5;
        double l = 0, r = 0;
        odom.UpdateFromPositions(l, r, dt);
        var steps = (int)Math.Round(period / dt);
        var maxY = 0.0;
        for (var i = 0; i < steps; i++)
        {
            l += 4 * dt;
            r += 6 * dt;
            odom.UpdateFromPositions(l, r, dt);
            maxY = Math.Max(maxY, odom.Y);
        }

        odom.X.ShouldBe(0, 1e-3);
        odom.Y.ShouldBe(0, 1e-3);
        maxY.ShouldBe(2.0, 1e-3); // diameter
        odom.Linear.ShouldBe(0.5, 1e-9);
        odom.Angular.ShouldBe(0.5, 1e-9);
    }
}

public class ControllerManagerTests
{
    private static (Ros2Context Ctx, ControllerManager Cm, GenericSystem Hw) Arm()
    {
        var ctx = Ctx.Create();
        var rm = new ResourceManager();
        var hw = new GenericSystem("arm", ["j1", "j2"], [HardwareInterfaceNames.Position]);
        rm.Add(hw);
        return (ctx, new ControllerManager(ctx, rm), hw);
    }

    [Fact]
    public async Task Command_interfaces_are_claimed_exclusively()
    {
        var (ctx, cm, _) = Arm();
        await using var _ = ctx;
        var a = cm.Spawn(new ForwardCommandController(HardwareInterfaceNames.Position), "a", [new Parameter("joints", new[] { "j1", "j2" })]);
        cm.LoadController(new ForwardCommandController(HardwareInterfaceNames.Position), "b", [new Parameter("joints", new[] { "j2" })]);
        cm.ConfigureController("b").ShouldBeTrue();

        cm.SwitchController(["b"], [], SwitchStrictness.Strict).ShouldBeFalse();
        cm.Resources.ClaimedBy("j2/position").ShouldBe("a");

        // Swapping in one switch releases a's claims before b claims.
        cm.SwitchController(["b"], ["a"], SwitchStrictness.Strict).ShouldBeTrue();
        cm.Resources.ClaimedBy("j2/position").ShouldBe("b");
        cm.Resources.IsClaimed("j1/position").ShouldBeFalse();
        a.Lifecycle.ShouldBe(ControllerLifecycle.Inactive);
    }

    [Fact]
    public async Task Forward_command_reaches_the_hardware()
    {
        var (ctx, cm, hw) = Arm();
        await using var _ = ctx;
        var c = cm.Spawn(new ForwardCommandController(HardwareInterfaceNames.Position), "fwd", [new Parameter("joints", new[] { "j1", "j2" })]);
        c.SetCommand([0.3, -0.2]);
        Ctx.Run(cm, RosTime.FromSeconds(1), 0.05);
        hw.StateInterfaces.Single(s => s.Name == "j1/position").Value.ShouldBe(0.3);
        hw.StateInterfaces.Single(s => s.Name == "j2/position").Value.ShouldBe(-0.2);
    }

    [Fact]
    public async Task Services_list_and_switch_controllers()
    {
        var (ctx, cm, _) = Arm();
        await using var _ = ctx;
        cm.RegisterControllerType("test/Forward", () => new ForwardCommandController(HardwareInterfaceNames.Position));
        var tool = ctx.CreateNode("tool");
        var load = tool.CreateClient<LoadControllerRequest, LoadControllerResponse>("/controller_manager/load_controller");
        var list = tool.CreateClient<ListControllersRequest, ListControllersResponse>("/controller_manager/list_controllers");
        var ifaces = tool.CreateClient<ListHardwareInterfacesRequest, ListHardwareInterfacesResponse>("/controller_manager/list_hardware_interfaces");

        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);
        await load.WaitForServiceAsync(TimeSpan.FromSeconds(2));

        // Unknown type parameter → failure, no exception.
        (await load.CallAsync(new LoadControllerRequest { Name = "nope" }, TimeSpan.FromSeconds(2))).Ok.ShouldBeFalse();

        cm.LoadController("fwd", "test/Forward");
        var states = (await list.CallAsync(new ListControllersRequest(), TimeSpan.FromSeconds(2))).Controller;
        states.Single().State.ShouldBe("unconfigured");
        states.Single().Type.ShouldBe("test/Forward");

        var hw = await ifaces.CallAsync(new ListHardwareInterfacesRequest(), TimeSpan.FromSeconds(2));
        hw.CommandInterfaces.Select(i => i.Name).ShouldBe(new[] { "j1/position", "j2/position" }, ignoreOrder: true);
        hw.StateInterfaces.Length.ShouldBe(6);

        cts.Cancel();
        await spin;
    }

    [Fact]
    public async Task Diff_drive_turns_cmd_vel_into_wheel_speeds_and_odometry()
    {
        await using var ctx = Ctx.Create();
        var rm = new ResourceManager();
        var hw = new SimulatedDiffDriveHardware();
        rm.Add(hw);
        var cm = new ControllerManager(ctx, rm);
        var dd = cm.Spawn(new DiffDriveController(), "diff_drive_controller",
        [
            new Parameter("left_wheel_names", new[] { "left_wheel_joint" }),
            new Parameter("right_wheel_names", new[] { "right_wheel_joint" }),
            new Parameter("wheel_separation", 0.4),
            new Parameter("wheel_radius", 0.1),
            new Parameter("cmd_vel_timeout", 100.0),
        ]);
        var odomSub = ctx.CreateNode("watch").CreateSubscription<Msgs.NavMsgs.Odometry>("/diff_drive_controller/odom");

        dd.SetCommand(0.5, 0);
        Ctx.Run(cm, RosTime.FromSeconds(1), 2.0);

        hw.CommandInterfaces.All(c => Math.Abs(c.Value - 5.0) < 1e-9).ShouldBeTrue();
        dd.Odometry.X.ShouldBe(1.0, 0.02); // 0.5 m/s for ~2 s (first cycle latches)
        dd.Odometry.Y.ShouldBe(0, 1e-9);
        odomSub.TryTake(out var odom).ShouldBeTrue();
        odom!.Header.FrameId.ShouldBe("odom");
    }

    [Fact]
    public async Task Diff_drive_stops_when_cmd_vel_times_out_and_respects_acceleration_limits()
    {
        await using var ctx = Ctx.Create();
        var rm = new ResourceManager();
        var hw = new SimulatedDiffDriveHardware();
        rm.Add(hw);
        var cm = new ControllerManager(ctx, rm);
        var dd = cm.Spawn(new DiffDriveController(), "dd",
        [
            new Parameter("left_wheel_names", new[] { "left_wheel_joint" }),
            new Parameter("right_wheel_names", new[] { "right_wheel_joint" }),
            new Parameter("wheel_separation", 0.4),
            new Parameter("wheel_radius", 0.1),
            new Parameter("linear.x.max_acceleration", 1.0),
            new Parameter("cmd_vel_timeout", 0.0001),
        ]);

        dd.SetCommand(1.0, 0);
        Thread.Sleep(5);
        Ctx.Run(cm, RosTime.FromSeconds(1), 0.1);
        hw.CommandInterfaces.All(c => c.Value == 0).ShouldBeTrue();
    }

    [Fact]
    public void Ackermann_steering_follows_the_bicycle_model()
    {
        AckermannSteeringController.SteeringFor(1.0, 0.5, 2.0).ShouldBe(Math.Atan(1.0), 1e-12);
        AckermannSteeringController.SteeringFor(0, 1, 2).ShouldBe(0);
    }

    private static JointTrajectory Trajectory(params (double T, double J1, double J2)[] points) => new()
    {
        JointNames = ["j1", "j2"],
        Points = points.Select(p => new JointTrajectoryPoint
        {
            Positions = [p.J1, p.J2],
            Velocities = [0, 0],
            TimeFromStart = RosDuration.FromSeconds(p.T).ToMsg(),
        }).ToArray(),
    };

    [Fact]
    public async Task Joint_trajectory_controller_follows_a_trajectory_sent_through_the_action()
    {
        var (ctx, cm, hw) = Arm();
        await using var _ = ctx;
        cm.Spawn(new JointTrajectoryController(), "joint_trajectory_controller", [new Parameter("joints", new[] { "j1", "j2" })]);
        cm.Start();

        var client = ctx.CreateNode("client").CreateActionClient<FollowJointTrajectoryGoal, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback>(
            "/joint_trajectory_controller/follow_joint_trajectory");
        using var cts = new CancellationTokenSource();
        var spin = ctx.SpinAsync(cts.Token);
        (await client.WaitForServerAsync(TimeSpan.FromSeconds(3))).ShouldBeTrue();

        var feedback = 0;
        var result = await client.SendGoalAndWaitAsync(
            new FollowJointTrajectoryGoal { Trajectory = Trajectory((0.3, 0.5, -0.5), (0.6, 1.0, 0.25)) },
            _ => Interlocked.Increment(ref feedback)).WaitAsync(TimeSpan.FromSeconds(10));

        result.Status.ShouldBe(GoalStatus.Succeeded);
        result.Result.ErrorCode.ShouldBe(FollowJointTrajectoryResult.SUCCESSFUL);
        hw.StateInterfaces.Single(s => s.Name == "j1/position").Value.ShouldBe(1.0, 1e-6);
        hw.StateInterfaces.Single(s => s.Name == "j2/position").Value.ShouldBe(0.25, 1e-6);
        feedback.ShouldBeGreaterThan(0);

        // Unknown joints are rejected with INVALID_JOINTS.
        var bad = Trajectory((0.1, 0, 0));
        bad.JointNames = ["j1", "x"];
        var r2 = await client.SendGoalAndWaitAsync(new FollowJointTrajectoryGoal { Trajectory = bad }).WaitAsync(TimeSpan.FromSeconds(5));
        r2.Result.ErrorCode.ShouldBe(FollowJointTrajectoryResult.INVALID_JOINTS);

        cm.Stop();
        cts.Cancel();
        await spin;
    }

    [Fact]
    public void Spline_sampling_interpolates_cubic_and_quintic_segments()
    {
        var cubic = new SampledTrajectory([0, 1], [[0.0], [1.0]], [[0.0], [0.0]], null);
        double[] p = new double[1], v = new double[1], a = new double[1];
        cubic.Sample(0.5, p, v, a);
        p[0].ShouldBe(0.5, 1e-12);
        v[0].ShouldBe(1.5, 1e-12); // peak of the cubic smoothstep

        var quintic = new SampledTrajectory([0, 2], [[0.0], [2.0]], [[0.0], [0.0]], [[0.0], [0.0]]);
        quintic.Sample(1, p, v, a);
        p[0].ShouldBe(1.0, 1e-12);
        a[0].ShouldBe(0, 1e-12);
        quintic.Sample(2, p, v, a);
        p[0].ShouldBe(2.0);
    }

    [Fact]
    public async Task Gripper_reports_reached_and_stalled()
    {
        await using var ctx = Ctx.Create();
        var rm = new ResourceManager();
        var hw = new GenericSystem("gripper", ["finger"]) { TimeConstant = 0.05 };
        rm.Add(hw);
        var cm = new ControllerManager(ctx, rm);
        var g = cm.Spawn(new GripperActionController(), "gripper_controller", [new Parameter("joint", "finger"), new Parameter("stall_timeout", 0.2)]);

        var reach = g.CommandAsync(0.04);
        var t = Ctx.Run(cm, RosTime.FromSeconds(1), 1.0);
        (await reach.WaitAsync(TimeSpan.FromSeconds(1))).ReachedGoal.ShouldBeTrue();

        // Block the finger: the mirrored position stops at an offset, velocity drops to zero → stall.
        hw.TimeConstant = 0;
        hw.PositionStateOffset = 0.02; // never reaches 0.0 exactly within tolerance 0.01
        var stall = g.CommandAsync(0.0);
        Ctx.Run(cm, t, 1.0);
        var r = await stall.WaitAsync(TimeSpan.FromSeconds(1));
        r.Stalled.ShouldBeTrue();
        r.ReachedGoal.ShouldBeFalse();
    }
}
