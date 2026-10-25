using ROS2.Net.Manipulation;
using Shouldly;

namespace ROS2.Net.Control.Tests;

public class UrdfAndKinematicsTests
{
    [Fact]
    public void Parses_the_bundled_ur5()
    {
        var m = UrdfModel.LoadBuiltIn("ur5");
        m.Name.ShouldBe("ur5");
        m.RootLink.ShouldBe("world");
        m.ActuatedJoints.Count.ShouldBe(6);
        m.GetJoint("elbow_joint").Limits!.Upper.ShouldBe(3.1416);
        m.GetJoint("shoulder_lift_joint").Axis.ShouldBe(Vec3.UnitY);
        m.GetLink("upper_arm_link").Collisions.Single().Geometry.ShouldBeOfType<CylinderGeometry>();
        m.GetLink("base_link").Inertial!.Mass.ShouldBe(4.0);
        UrdfModel.LoadBuiltIn("diffbot").GetJoint("left_wheel_joint").Type.ShouldBe(JointType.Continuous);
    }

    [Fact]
    public void Rejects_broken_urdf()
    {
        Should.Throw<UrdfException>(() => UrdfModel.Parse("<robot name='r'><link name='a'/><link name='b'/></robot>"));
        Should.Throw<UrdfException>(() => UrdfModel.Parse("<robot><link name='a'/><link name='b'/><joint name='j' type='revolute'><parent link='a'/><child link='b'/></joint></robot>"));
    }

    [Theory]
    [InlineData(0, 0, 1.8, 0)]
    [InlineData(Math.PI / 2, 0, 0, 1.8)]
    [InlineData(0, Math.PI / 2, 1.0, 0.8)]
    [InlineData(Math.PI / 4, -Math.PI / 2, 0.70710678 + (0.8 * 0.70710678), 0.70710678 - (0.8 * 0.70710678))]
    public void Planar_forward_kinematics_matches_hand_computation(double q1, double q2, double x, double y)
    {
        var chain = new KinematicChain(UrdfModel.LoadBuiltIn("planar2"), "base", "tip");
        var p = chain.ForwardKinematics([q1, q2]);
        p.Position.X.ShouldBe(x, 1e-6);
        p.Position.Y.ShouldBe(y, 1e-6);
        p.Position.Z.ShouldBe(0, 1e-12);
        p.Rotation.ToRpy().Yaw.ShouldBe(Math.IEEERemainder(q1 + q2, 2 * Math.PI), 1e-9);
    }

    [Fact]
    public void Analytic_two_link_ik_inverts_fk()
    {
        var chain = new KinematicChain(UrdfModel.LoadBuiltIn("planar2"), "base", "tip");
        var sol = IkSolver.TwoLinkPlanar(1.2, 0.7, 1.0, 0.8)!.Value;
        var p = chain.ForwardKinematics([sol.Q1, sol.Q2]);
        p.Position.X.ShouldBe(1.2, 1e-9);
        p.Position.Y.ShouldBe(0.7, 1e-9);
        IkSolver.TwoLinkPlanar(3, 0, 1, 0.8).ShouldBeNull();
    }

    [Fact]
    public void Jacobian_matches_finite_differences()
    {
        var chain = new KinematicChain(UrdfModel.LoadBuiltIn("ur5"), "base_link", "tool0");
        var random = new Random(3);
        for (var trial = 0; trial < 5; trial++)
        {
            var q = chain.RandomConfiguration(random);
            var jac = new double[6 * chain.Dof];
            var pose = chain.Jacobian(q, jac);
            const double h = 1e-6;
            for (var j = 0; j < chain.Dof; j++)
            {
                var qp = (double[])q.Clone();
                qp[j] += h;
                var p2 = chain.ForwardKinematics(qp);
                var dp = (p2.Position - pose.Position) / h;
                var dr = (p2.Rotation * pose.Rotation.Conjugate()).ToRotationVector() / h;
                jac[j].ShouldBe(dp.X, 1e-4);
                jac[chain.Dof + j].ShouldBe(dp.Y, 1e-4);
                jac[(2 * chain.Dof) + j].ShouldBe(dp.Z, 1e-4);
                jac[(3 * chain.Dof) + j].ShouldBe(dr.X, 1e-4);
                jac[(4 * chain.Dof) + j].ShouldBe(dr.Y, 1e-4);
                jac[(5 * chain.Dof) + j].ShouldBe(dr.Z, 1e-4);
            }
        }
    }

    [Theory]
    [InlineData(IkMethod.LevenbergMarquardt)]
    [InlineData(IkMethod.DampedLeastSquares)]
    public void Ik_round_trips_random_reachable_poses(IkMethod method)
    {
        var chain = new KinematicChain(UrdfModel.LoadBuiltIn("ur5"), "base_link", "tool0");
        var ik = new IkSolver(chain, new IkOptions { Method = method, MaxIterations = 300 });
        var random = new Random(11);
        for (var trial = 0; trial < 20; trial++)
        {
            var truth = chain.RandomConfiguration(random);
            var target = chain.ForwardKinematics(truth);
            var r = ik.Solve(target, new double[chain.Dof]);
            r.Success.ShouldBeTrue($"trial {trial}");
            var reached = chain.ForwardKinematics(r.Solution);
            (reached.Position - target.Position).Length.ShouldBeLessThan(1e-4);
            reached.Rotation.AngleTo(target.Rotation).ShouldBeLessThan(1e-4);
            chain.WithinLimits(r.Solution).ShouldBeTrue();
        }
    }

    [Fact]
    public void Position_only_ik_ignores_orientation()
    {
        var chain = new KinematicChain(UrdfModel.LoadBuiltIn("planar2"), "base", "tip");
        var ik = new IkSolver(chain, new IkOptions { PositionOnly = true });
        var r = ik.Solve(new Pose3(new Vec3(0.5, 1.2, 0), Quat.FromRpy(1, 1, 1)), [0.1, 0.1]);
        r.Success.ShouldBeTrue();
        chain.ForwardKinematics(r.Solution).Position.X.ShouldBe(0.5, 1e-5);
    }
}

public class CollisionAndPlanningTests
{
    [Fact]
    public void Primitive_distances_are_correct()
    {
        CollisionShape.Distance(new SphereShape(Vec3.Zero, 1), new SphereShape(new Vec3(3, 0, 0), 1)).ShouldBe(1, 1e-12);
        CollisionShape.Distance(new CapsuleShape(new Vec3(-1, 0, 0), new Vec3(1, 0, 0), 0.1), new CapsuleShape(new Vec3(0, -1, 1), new Vec3(0, 1, 1), 0.1)).ShouldBe(0.8, 1e-12);
        var box = new BoxShape(Pose3.Identity, new Vec3(0.5, 0.5, 0.5));
        CollisionShape.Distance(new SphereShape(new Vec3(2, 0, 0), 0.5), box).ShouldBe(1.0, 1e-12);
        CollisionShape.Distance(new CapsuleShape(new Vec3(-2, 0, 1.5), new Vec3(2, 0, 1.5), 0.25), box).ShouldBe(0.75, 1e-6);
        BoxShape.Overlaps(box, new BoxShape(Pose3.FromXyzRpy(0.9, 0, 0, 0, 0, 0.5), new Vec3(0.5, 0.5, 0.5))).ShouldBeTrue();
        BoxShape.Overlaps(box, new BoxShape(Pose3.FromXyzRpy(1.8, 0, 0, 0, 0, 0.3), new Vec3(0.5, 0.5, 0.5))).ShouldBeFalse();
    }

    [Fact]
    public void Scene_detects_obstacles_and_attached_objects()
    {
        var model = UrdfModel.LoadBuiltIn("planar2");
        var chain = new KinematicChain(model, "base", "tip");
        var scene = new PlanningScene(model);
        scene.IsStateValid(chain, [0, 0]).ShouldBeTrue();
        scene.AddSphere("ball", new Vec3(1.4, 0, 0), 0.1);
        scene.IsStateValid(chain, [0, 0]).ShouldBeFalse();
        scene.IsStateValid(chain, [Math.PI / 2, 0]).ShouldBeTrue();

        scene.RemoveWorldObject("ball");
        scene.AddBox("crate", new Vec3(1.8, 0, 0), new Vec3(0.1, 0.1, 0.1));
        var state = scene.StateFor(chain, [0, 0]);
        scene.Attach("crate", "tip", state, ["tip", "link2"]);
        scene.IsStateValid(chain, [0, 0]).ShouldBeTrue();
        scene.AddSphere("post", new Vec3(0, 1.8, 0), 0.05);
        scene.IsStateValid(chain, [Math.PI / 2, 0]).ShouldBeFalse(); // the carried crate hits the post
    }

    [Fact]
    public void Rrt_connect_plans_around_an_obstacle()
    {
        var model = UrdfModel.LoadBuiltIn("planar2");
        var chain = new KinematicChain(model, "base", "tip");
        var scene = new PlanningScene(model);

        // A wall in the first quadrant blocks the direct sweep from 0 to π/2 of the outstretched arm.
        scene.AddBox("wall", new Vec3(1.2, 1.2, 0), new Vec3(0.3, 0.3, 0.5));
        var planner = new RrtConnectPlanner(chain, scene);
        double[] start = [0, 0];
        double[] goal = [Math.PI / 2, 0];
        planner.IsEdgeValid(start, goal).ShouldBeFalse();

        var plan = planner.Plan(start, goal);
        plan.Success.ShouldBeTrue(plan.Message);
        plan.Waypoints[0].ShouldBe(start);
        plan.Waypoints[^1].ShouldBe(goal);
        for (var i = 1; i < plan.Waypoints.Count; i++)
        {
            planner.IsEdgeValid(plan.Waypoints[i - 1], plan.Waypoints[i]).ShouldBeTrue();
        }
    }

    [Theory]
    [InlineData(VelocityProfile.Trapezoidal)]
    [InlineData(VelocityProfile.Quintic)]
    public void Time_parameterization_respects_limits(VelocityProfile profile)
    {
        double[] vmax = [1.0, 0.5, 2.0];
        double[] amax = [2.0, 1.0, 4.0];
        var waypoints = new List<double[]> { new[] { 0.0, 0, 0 }, new[] { 1.5, -0.4, 0.1 }, new[] { 1.4, 0.6, 2.0 } };
        var traj = TimeParameterization.Compute(["a", "b", "c"], waypoints, vmax, amax, profile, 0.005);

        traj.Points[0].TimeFromStart.Sec.ShouldBe(0);
        traj.Points[^1].Positions.ShouldBe(waypoints[^1], 1e-9);
        double lastT = -1;
        foreach (var p in traj.Points)
        {
            var t = RosDuration.FromMsg(p.TimeFromStart).TotalSeconds;
            t.ShouldBeGreaterThan(lastT);
            lastT = t;
            for (var j = 0; j < 3; j++)
            {
                Math.Abs(p.Velocities[j]).ShouldBeLessThanOrEqualTo(vmax[j] + 1e-6);
                Math.Abs(p.Accelerations[j]).ShouldBeLessThanOrEqualTo(amax[j] + 1e-6);
            }
        }

        // The waypoint positions are hit exactly at segment ends (rest-to-rest).
        traj.Points.Any(p => p.Positions.Zip(waypoints[1]).All(z => Math.Abs(z.First - z.Second) < 1e-9)).ShouldBeTrue();
    }

    [Fact]
    public void Cartesian_path_moves_the_tip_in_a_straight_line()
    {
        var chain = new KinematicChain(UrdfModel.LoadBuiltIn("ur5"), "base_link", "tool0");
        var planner = new CartesianPlanner(chain);
        double[] start = [0, -1.2, 1.5, -1.9, -1.57, 0];
        var origin = chain.ForwardKinematics(start);
        var to = origin with { Position = origin.Position + new Vec3(0, 0, -0.1) };

        var r = planner.Plan(start, [to]);
        r.Complete.ShouldBeTrue();
        foreach (var q in r.Waypoints)
        {
            var p = chain.ForwardKinematics(q).Position;
            p.X.ShouldBe(origin.Position.X, 1e-4);
            p.Y.ShouldBe(origin.Position.Y, 1e-4);
        }

        (chain.ForwardKinematics(r.Waypoints[^1]).Position - to.Position).Length.ShouldBeLessThan(1e-4);
    }

    [Fact]
    public async Task Robot_state_publisher_emits_static_and_dynamic_transforms()
    {
        await using var ctx = Ctx.Create();
        var rsp = new RobotStatePublisher(ctx, UrdfModel.LoadBuiltIn("planar2"));
        var watcher = ctx.CreateNode("w");
        var statics = watcher.CreateSubscription<Msgs.Tf2Msgs.TFMessage>("/tf_static", QosProfile.Latched);
        var dynamic = watcher.CreateSubscription<Msgs.Tf2Msgs.TFMessage>("/tf", QosProfile.Default.KeepLast(100));
        var description = watcher.CreateSubscription<Msgs.StdMsgs.String>("/robot_description", QosProfile.Latched);

        statics.TryTake(out var s).ShouldBeTrue();
        s!.Transforms.Single().ChildFrameId.ShouldBe("tip");
        description.TryTake(out var d).ShouldBeTrue();
        d!.Data.ShouldContain("planar2");

        rsp.PublishJointPositions(new Dictionary<string, double> { ["j1"] = 0.5, ["j2"] = 0 });
        dynamic.TryTake(out var tf).ShouldBeTrue();
        tf!.Transforms.Select(t => t.ChildFrameId).ShouldBe(new[] { "link1", "link2" }, ignoreOrder: true);
        rsp.LinkPoses()["tip"].Position.X.ShouldBe(1.8 * Math.Cos(0.5), 1e-9);
    }
}
