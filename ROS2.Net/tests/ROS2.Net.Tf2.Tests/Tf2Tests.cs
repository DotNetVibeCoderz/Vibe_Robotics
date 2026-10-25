using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Rmw.InProcess;
using ROS2.Net.Tf2;
using Shouldly;

namespace ROS2.Net.Tf2.Tests;

internal static class Ctx
{
    private static int s_domain = 150;

    public static Ros2Context Create() => Ros2Context.Create(new Ros2ContextOptions
    {
        DomainId = Interlocked.Increment(ref s_domain) % 230,
        Rmw = new InProcessRmwFactory(),
    });
}

public class Tf2Tests
{
    private static RosTime T(double s) => RosTime.FromSeconds(s);

    [Fact]
    public void Math_composes_and_inverts()
    {
        var a = new Transform3d(new Vector3d(1, 0, 0), Quaterniond.FromYaw(Math.PI / 2));
        var p = a.TransformPoint(new Vector3d(1, 0, 0));
        p.X.ShouldBe(1, 1e-12);
        p.Y.ShouldBe(1, 1e-12);
        var back = a.Inverse().TransformPoint(p);
        back.X.ShouldBe(1, 1e-12);
        back.Y.ShouldBe(0, 1e-12);
        (a * a.Inverse()).Translation.Length.ShouldBe(0, 1e-12);

        var q = Quaterniond.FromRpy(0.1, 0.2, 0.3);
        var (r, pi, y) = q.ToRpy();
        r.ShouldBe(0.1, 1e-12);
        pi.ShouldBe(0.2, 1e-12);
        y.ShouldBe(0.3, 1e-12);
        var (axis, angle) = Quaterniond.FromAxisAngle(Vector3d.UnitZ, 0.7).ToAxisAngle();
        angle.ShouldBe(0.7, 1e-12);
        axis.Z.ShouldBe(1, 1e-12);
        Quaterniond.Slerp(Quaterniond.Identity, Quaterniond.FromYaw(1.0), 0.5).Yaw.ShouldBe(0.5, 1e-9);
    }

    [Fact]
    public void Chained_lookup_goes_through_the_common_ancestor()
    {
        var buf = new TfBuffer();
        buf.SetTransform("map", "odom", Transform3d.From2D(10, 0, 0), T(1), isStatic: true);
        buf.SetTransform("odom", "base_link", Transform3d.From2D(1, 2, Math.PI / 2), T(1));
        buf.SetTransform("base_link", "laser", new Transform3d(new Vector3d(0.5, 0, 0.2), Quaterniond.Identity), T(1), isStatic: true);
        buf.SetTransform("odom", "other", Transform3d.From2D(0, -1, 0), T(1));

        // laser origin in map: odom(10,0) + base(1,2) + R(90°)*(0.5,0) = (11, 2.5)
        var tf = buf.LookupTransform3d("map", "laser", T(1));
        tf.Translation.X.ShouldBe(11, 1e-9);
        tf.Translation.Y.ShouldBe(2.5, 1e-9);
        tf.Translation.Z.ShouldBe(0.2, 1e-9);

        // Sibling branch: laser expressed in 'other'.
        var sib = buf.LookupTransform3d("other", "laser", T(1));
        sib.Translation.X.ShouldBe(1, 1e-9);
        sib.Translation.Y.ShouldBe(3.5, 1e-9);

        var stamped = buf.LookupTransform("map", "laser", RosTime.Zero);
        stamped.Header.FrameId.ShouldBe("map");
        stamped.ChildFrameId.ShouldBe("laser");
        buf.GetParent("laser").ShouldBe("base_link");
    }

    [Fact]
    public void Interpolates_and_reports_extrapolation()
    {
        var buf = new TfBuffer();
        buf.SetTransform("odom", "base", Transform3d.From2D(0, 0, 0), T(10));
        buf.SetTransform("odom", "base", Transform3d.From2D(2, 0, 1.0), T(12));

        var mid = buf.LookupTransform3d("odom", "base", T(11));
        mid.Translation.X.ShouldBe(1, 1e-9);
        mid.Rotation.Yaw.ShouldBe(0.5, 1e-9);

        Should.Throw<ExtrapolationException>(() => buf.LookupTransform("odom", "base", T(13))).Message.ShouldContain("future");
        Should.Throw<ExtrapolationException>(() => buf.LookupTransform("odom", "base", T(9)));
        Should.Throw<LookupException>(() => buf.LookupTransform("odom", "nope", T(11)));
        buf.CanTransform("odom", "base", T(11)).ShouldBeTrue();
        buf.CanTransform("odom", "base", T(14)).ShouldBeFalse();
    }

    [Fact]
    public void Latest_common_time_is_the_oldest_newest_link()
    {
        var buf = new TfBuffer();
        buf.SetTransform("map", "odom", Transform3d.From2D(0, 0, 0), T(1));
        buf.SetTransform("map", "odom", Transform3d.From2D(1, 0, 0), T(5));
        buf.SetTransform("odom", "base", Transform3d.From2D(0, 0, 0), T(1));
        buf.SetTransform("odom", "base", Transform3d.From2D(0, 1, 0), T(3));

        var tf = buf.LookupTransform("map", "base", RosTime.Zero);
        RosTime.FromMsg(tf.Header.Stamp).ShouldBe(T(3));
        tf.Transform.Translation.X.ShouldBe(0.5, 1e-9); // map->odom interpolated at t=3
        tf.Transform.Translation.Y.ShouldBe(1, 1e-9);
    }

    [Fact]
    public void Disconnected_trees_raise_connectivity_errors()
    {
        var buf = new TfBuffer();
        buf.SetTransform("a", "b", Transform3d.Identity, T(1));
        buf.SetTransform("c", "d", Transform3d.Identity, T(1));
        Should.Throw<ConnectivityException>(() => buf.LookupTransform("a", "d", RosTime.Zero));
        Should.Throw<InvalidTransformArgumentException>(() => buf.SetTransform("a", "a", Transform3d.Identity, T(1)));
    }

    [Fact]
    public void Time_travel_uses_the_fixed_frame()
    {
        var buf = new TfBuffer();
        buf.SetTransform("odom", "base", Transform3d.From2D(0, 0, 0), T(1));
        buf.SetTransform("odom", "base", Transform3d.From2D(5, 0, 0), T(2));

        // Where was the robot at t=1, as seen from the robot at t=2?
        var tf = buf.LookupTransform("base", T(2), "base", T(1), "odom");
        tf.Transform.Translation.X.ShouldBe(-5, 1e-9);
    }

    [Fact]
    public void Transforms_stamped_data()
    {
        var buf = new TfBuffer();
        buf.SetTransform("map", "base", Transform3d.From2D(1, 1, Math.PI), T(1), isStatic: true);
        var p = new PointStamped { Point = new Point { X = 1 } };
        p.Header.FrameId = "base";
        var inMap = buf.TransformPoint(p, "map");
        inMap.Header.FrameId.ShouldBe("map");
        inMap.Point.X.ShouldBe(0, 1e-9);
        inMap.Point.Y.ShouldBe(1, 1e-9);

        var v = new Vector3Stamped { Vector = new Vector3 { X = 1 } };
        v.Header.FrameId = "base";
        buf.TransformVector(v, "map").Vector.X.ShouldBe(-1, 1e-9);

        var pose = new PoseStamped();
        pose.Header.FrameId = "base";
        pose.Pose.Orientation.W = 1;
        Quaterniond.FromMsg(buf.TransformPose(pose, "map").Pose.Orientation).Yaw.ShouldBe(Math.PI, 1e-9);
    }

    [Fact]
    public void Introspection_lists_frames()
    {
        var buf = new TfBuffer();
        buf.SetTransform("map", "odom", Transform3d.Identity, T(1), isStatic: true);
        buf.SetTransform("odom", "base_link", Transform3d.Identity, T(1));
        buf.AllFramesAsString().ShouldContain("Frame base_link exists with parent odom.");
        buf.AllFramesAsYaml().ShouldContain("parent: 'map'");
        buf.ToMermaid().ShouldContain("graph TD");
        buf.ToDot().ShouldContain("\"odom\" -> \"base_link\"");
        buf.FrameNames.ShouldBe(new[] { "base_link", "map", "odom" });
    }

    [Fact]
    public async Task Wait_for_transform_completes_when_data_arrives()
    {
        var buf = new TfBuffer();
        var wait = buf.WaitForTransformAsync("odom", "base", RosTime.Zero, TimeSpan.FromSeconds(5));
        await Task.Delay(30);
        wait.IsCompleted.ShouldBeFalse();
        buf.SetTransform("odom", "base", Transform3d.From2D(3, 0, 0), T(1));
        (await wait.WaitAsync(TimeSpan.FromSeconds(5))).Transform.Translation.X.ShouldBe(3);

        await Should.ThrowAsync<TransformTimeoutException>(() => buf.WaitForTransformAsync("odom", "ghost", RosTime.Zero, TimeSpan.FromMilliseconds(50)));
    }

    [Fact]
    public async Task Listener_receives_broadcast_and_static_transforms()
    {
        await using var ctx = Ctx.Create();
        var pubNode = ctx.CreateNode("tf_pub");
        var statics = new StaticTransformBroadcaster(pubNode);
        statics.SendTransform("base_link", "laser", new Transform3d(new Vector3d(0.2, 0, 0), Quaterniond.Identity));
        statics.SendTransform("base_link", "camera", new Transform3d(new Vector3d(0, 0, 0.5), Quaterniond.Identity));

        var buffer = new TfBuffer();
        using var listener = new TransformListener(buffer, ctx.CreateNode("tf_listen"));

        var dynamic = new TransformBroadcaster(pubNode);
        dynamic.SendTransform("odom", "base_link", Transform3d.From2D(1, 0, 0), RosTime.Now());

        var tf = await buffer.WaitForTransformAsync("odom", "camera", RosTime.Zero, TimeSpan.FromSeconds(5));
        tf.Transform.Translation.X.ShouldBe(1, 1e-9);
        tf.Transform.Translation.Z.ShouldBe(0.5, 1e-9);
        buffer.CanTransform("odom", "laser", RosTime.Zero).ShouldBeTrue(); // both statics survived the accumulate-and-republish
    }
}
