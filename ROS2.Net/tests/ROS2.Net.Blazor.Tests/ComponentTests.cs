using Bunit;
using Microsoft.Extensions.DependencyInjection;
using ROS2.Net.Blazor.Components;
using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.NavMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Rmw.InProcess;
using Shouldly;
using LogMsg = ROS2.Net.Msgs.RclInterfaces.Log;

namespace ROS2.Net.Blazor.Tests;

public sealed class ComponentTests : IAsyncDisposable
{
    private static int s_domain = 150;
    private readonly Bunit.TestContext _bunit = new();
    private readonly Ros2Context _ros;

    public ComponentTests()
    {
        _ros = Ros2Context.Create(new Ros2ContextOptions { DomainId = Interlocked.Increment(ref s_domain) % 230, Rmw = new InProcessRmwFactory() });
        _bunit.Services.AddSingleton(_ros);
        _bunit.JSInterop.Mode = JSRuntimeMode.Loose;
    }

    [Fact]
    public async Task Joystick_publishes_twist_on_cmd_vel()
    {
        var probe = _ros.CreateNode("probe");
        var sub = probe.CreateSubscription<Twist>("/cmd_vel");
        var cut = _bunit.RenderComponent<TeleopJoystick>(p => p.Add(x => x.MaxLinear, 0.5).Add(x => x.MaxAngular, 1.0));

        await cut.InvokeAsync(() => cut.Instance.SetStick(0, -1)); // full forward

        var twist = await sub.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(3));
        twist.Linear.X.ShouldBe(0.5, 1e-9);
        twist.Angular.Z.ShouldBe(0, 1e-9);
        cut.WaitForAssertion(() => cut.Markup.ShouldContain("+0.50"), TimeSpan.FromSeconds(2));
    }

    [Fact]
    public void Laser_scan_view_draws_hits_and_reports_counts()
    {
        var scan = new LaserScan { AngleMin = -MathF.PI, AngleIncrement = MathF.PI / 2, RangeMin = 0.1f, RangeMax = 10, Ranges = [1, 2, float.PositiveInfinity, 3] };
        var cut = _bunit.RenderComponent<LaserScanView>(p => p.Add(x => x.Scan, scan));
        cut.Markup.ShouldContain("4 beams · 3 hits · min 1.00 m");
        cut.FindAll("path.hit").Count.ShouldBe(1);
    }

    [Fact]
    public void Occupancy_grid_view_rasterizes_the_map_to_png()
    {
        var map = new OccupancyGrid();
        map.Info.Width = 4;
        map.Info.Height = 3;
        map.Info.Resolution = 0.5f;
        map.Data = [0, 0, 100, -1, 0, 50, 0, 0, 100, 100, 100, 100];
        var cut = _bunit.RenderComponent<OccupancyGridView>(p => p.Add(x => x.Map, map).Add(x => x.RobotPose, new PlanarPose(1, 1, 0)));
        var image = cut.Find("image");
        image.GetAttribute("href")!.ShouldStartWith("data:image/png;base64,");
        image.GetAttribute("width").ShouldBe("2");
        cut.Markup.ShouldContain("4 × 3");
        cut.FindAll("circle.robot-body").Count.ShouldBe(1);
    }

    [Fact]
    public void Battery_gauge_shows_percentage_and_level()
    {
        var cut = _bunit.RenderComponent<BatteryGauge>(p => p.Add(x => x.Percentage, 0.15));
        cut.Markup.ShouldContain("15%");
        cut.Find("circle.value").ClassList.ShouldContain("low");
    }

    [Fact]
    public void Joint_state_panel_lists_joints_in_degrees()
    {
        var js = new JointState { Name = ["shoulder", "elbow"], Position = [Math.PI / 2, -Math.PI / 4] };
        var cut = _bunit.RenderComponent<JointStatePanel>(p => p.Add(x => x.State, js));
        cut.Markup.ShouldContain("shoulder");
        cut.Markup.ShouldContain("+90.0°");
        cut.Markup.ShouldContain("-45.0°");
    }

    [Fact]
    public void Log_console_shows_rosout_entries()
    {
        var cut = _bunit.RenderComponent<LogConsole>(p => p.Add(x => x.RefreshHz, 60));
        var node = _ros.CreateNode("noisy");
        var pub = node.CreatePublisher<LogMsg>("/rosout", QosProfile.Rosout);
        pub.Publish(new LogMsg { Level = LogMsg.WARN, Name = "noisy", Msg = "battery low" });
        cut.WaitForAssertion(() => cut.Markup.ShouldContain("battery low"), TimeSpan.FromSeconds(3));
        cut.Markup.ShouldContain("WARN");
    }

    [Fact]
    public void Topic_monitor_lists_published_topics_with_rate()
    {
        var node = _ros.CreateNode("talker");
        var pub = node.CreatePublisher<ROS2.Net.Msgs.StdMsgs.String>("/chatter");
        using var timer = new Timer(_ => pub.Publish(new ROS2.Net.Msgs.StdMsgs.String { Data = "hi" }), null, 0, 20);
        var cut = _bunit.RenderComponent<TopicMonitor>(p => p.Add(x => x.Interval, TimeSpan.FromMilliseconds(300)));
        cut.WaitForAssertion(() =>
        {
            cut.Markup.ShouldContain("/chatter");
            cut.Markup.ShouldContain("std_msgs/msg/String");
            cut.Markup.ShouldContain(" Hz");
        }, TimeSpan.FromSeconds(5));
    }

    [Fact]
    public async Task Parameter_editor_reads_and_sets_remote_parameters()
    {
        var target = _ros.CreateNode("controller");
        target.DeclareParameter("gain", 1.5, "Proportional gain");
        using var cts = new CancellationTokenSource();
        var spin = _ros.SpinAsync(cts.Token);
        var cut = _bunit.RenderComponent<ParameterEditor>(p => p.Add(x => x.TargetNode, "/controller"));
        cut.WaitForAssertion(() => cut.Markup.ShouldContain("Proportional gain"), TimeSpan.FromSeconds(5));

        var input = cut.Find("input[aria-label=gain]");
        await cut.InvokeAsync(() => input.Change("2.5"));
        cut.WaitForAssertion(() => cut.Markup.ShouldContain("Saved"), TimeSpan.FromSeconds(5));
        target.GetParameter<double>("gain").ShouldBe(2.5);

        cts.Cancel();
        await spin;
    }

    public async ValueTask DisposeAsync()
    {
        _bunit.Dispose();
        await _ros.DisposeAsync();
    }
}
