using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace Ros2NetGallery.Pages;

/// <summary>Parameters with descriptors, ranges and a validation callback, edited remotely.</summary>
public sealed class ParametersPage : GalleryPage
{
    public override string Id => "parameters";

    public override PageGroup Group => PageGroup.Communication;

    public override T2 Title => new("Parameters", "Parameter");

    public override T2 Summary => new(
        "A controller node declares typed parameters with ranges and a validation callback. The editor changes them through the standard parameter services, exactly like `ros2 param set`.",
        "Node controller mendeklarasikan parameter bertipe dengan rentang nilai dan callback validasi. Editor mengubahnya melalui service parameter standar, persis seperti `ros2 param set`.");

    public override IReadOnlyList<string> Interfaces => ["rcl_interfaces/srv/SetParameters", "/parameter_events", "~/describe_parameters"];

    public override string Code => """
        var node = context.CreateNode("controller");
        node.DeclareParameter("max_speed", 0.5, new ParameterDescriptor
        {
            Description = "Linear speed limit [m/s]",
            FloatingPointRange = new FloatingPointRange(0.0, 2.0),
        });
        node.DeclareParameter("mode", "auto", "auto | manual | docking");

        node.AddOnSetParametersCallback(ps =>
            ps.Any(p => p.Name == "mode" && p.Value.AsString() is not ("auto" or "manual" or "docking"))
                ? SetParametersResult.Failure("unknown mode")
                : SetParametersResult.Success);

        // From another node (or `ros2 param set /controller max_speed 0.8`):
        using var client = new ParameterClient(toolNode, "/controller");
        await client.SetAsync("max_speed", 0.8);
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var ctx = runtime.Context;
        var node = Own(ctx.CreateNode("controller", "/gallery"));
        var events = new LogConsole { Title = "/parameter_events" };
        node.ParametersChanged += changes => RosUi.OnUi(() =>
        {
            foreach (var c in changes)
            {
                events.Append(DateTime.Now, RosLog.INFO, "controller", c.IsNew ? $"declared {c.Name} = {c.NewValue}" : $"{c.Name}: {c.OldValue} → {c.NewValue}");
            }
        });

        node.DeclareParameter("max_speed", 0.5, new ParameterDescriptor { Description = Loc.T("Linear speed limit [m/s]", "Batas kecepatan linear [m/s]"), FloatingPointRange = new FloatingPointRange(0.0, 2.0) });
        node.DeclareParameter("max_turn_rate", 1.2, new ParameterDescriptor { Description = Loc.T("Angular speed limit [rad/s]", "Batas kecepatan sudut [rad/s]"), FloatingPointRange = new FloatingPointRange(0.1, 3.0) });
        node.DeclareParameter("mode", "auto", Loc.T("auto | manual | docking", "auto | manual | docking"));
        node.DeclareParameter("control_rate_hz", 50L, new ParameterDescriptor { Description = Loc.T("Control loop rate", "Laju loop kontrol"), IntegerRange = new IntegerRange(10, 500, 10) });
        node.DeclareParameter("use_pid", true, Loc.T("Closed-loop velocity control", "Kontrol kecepatan loop tertutup"));
        node.DeclareParameter("pid_gains", new[] { 1.2, 0.05, 0.01 }, Loc.T("Kp, Ki, Kd", "Kp, Ki, Kd"));
        node.DeclareParameter("robot_id", "amr-07", Loc.T("Fixed at startup", "Ditetapkan saat start"), readOnly: true);
        node.AddOnSetParametersCallback(ps =>
            ps.Any(p => p.Name == "mode" && p.Value.Type == ParameterType.String && p.Value.AsString() is not ("auto" or "manual" or "docking"))
                ? SetParametersResult.Failure("mode must be auto, manual or docking")
                : SetParametersResult.Success);

        var tool = Own(ctx.CreateNode("gallery_param_tool"));
        var editor = new ParameterEditor();
        Dispatcher.UIThread.Post(async () => await editor.AttachAsync(tool, "/gallery/controller"), DispatcherPriority.Background);

        var hint = Ui.Card(Ui.Col(6,
            Ui.Text(Loc.T("Try it", "Coba"), 13, FontWeight.SemiBold),
            Ui.Muted(Loc.T("Set max_speed to 3.0 (outside its range), mode to \"turbo\" (rejected by the callback) or robot_id (read-only). Accepted changes appear in /parameter_events.",
                "Ubah max_speed menjadi 3.0 (di luar rentang), mode menjadi \"turbo\" (ditolak callback) atau robot_id (read-only). Perubahan yang diterima muncul di /parameter_events."), 12)), padding: 14);
        return Ui.Columns("*,*", 14, editor, Ui.Rows("Auto,*", 14, hint, events));
    }
}

/// <summary>The lifecycle state machine, driven by buttons, with a lifecycle publisher.</summary>
public sealed class LifecyclePage : GalleryPage
{
    public override string Id => "lifecycle";

    public override PageGroup Group => PageGroup.Communication;

    public override T2 Title => new("Lifecycle nodes", "Lifecycle node");

    public override T2 Summary => new(
        "A managed node walks Unconfigured → Inactive → Active → Finalized. Its lifecycle publisher only sends while Active; transitions are also reachable through ~/change_state.",
        "Node terkelola melewati Unconfigured → Inactive → Active → Finalized. Lifecycle publisher-nya hanya mengirim saat Active; transisi juga bisa dipicu lewat ~/change_state.");

    public override IReadOnlyList<string> Interfaces => ["lifecycle_msgs/srv/ChangeState", "lifecycle_msgs/msg/TransitionEvent", "/lc_camera/transition_event"];

    public override string Code => """
        sealed class Camera(Ros2Context ctx) : LifecycleNode(ctx, "lc_camera")
        {
            LifecyclePublisher<StdMsgs.String>? _frames;

            protected override ValueTask<CallbackReturn> OnConfigure(LifecycleState previous)
            {
                _frames = CreateLifecyclePublisher<StdMsgs.String>("frames");
                CreateTimer(TimeSpan.FromMilliseconds(200), () => _frames.Publish(new() { Data = "frame" }));
                return ValueTask.FromResult(CallbackReturn.Success);
            }
        }

        await camera.ConfigureAsync();   // Unconfigured → Inactive
        await camera.ActivateAsync();    // Inactive → Active: frames start flowing
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var ctx = runtime.Context;
        var node = Own(ctx.CreateLifecycleNode("lc_camera"));
        LifecyclePublisher<StdMsgs.String>? frames = null;
        var frameNo = 0;
        node.ConfigureHandler = _ =>
        {
            frames = node.CreateLifecyclePublisher<StdMsgs.String>("frames");
            node.CreateTimer(TimeSpan.FromMilliseconds(200), () => frames.Publish(new StdMsgs.String { Data = $"frame {++frameNo}" }));
            return ValueTask.FromResult(CallbackReturn.Success);
        };

        var diagram = new LifecycleDiagram();
        var log = new LogConsole { Title = Loc.T("Transitions & received frames", "Transisi & frame yang diterima") };
        node.StateChanged += (from, to) => RosUi.OnUi(() =>
        {
            diagram.State = to;
            log.Append(DateTime.Now, RosLog.WARN, "lc_camera", $"{LifecycleNode.Label(from)} → {LifecycleNode.Label(to)}");
        });

        var viewer = Own(ctx.CreateNode("gallery_frame_viewer"));
        var received = 0;
        var (recvView, recvText) = Ui.Stat(Loc.T("Frames received", "Frame diterima"), "0");
        viewer.CreateSubscription<StdMsgs.String>("/frames", m =>
        {
            var n = Interlocked.Increment(ref received);
            RosUi.OnUi(() =>
            {
                recvText.Text = n.ToString();
                if (n % 5 == 0)
                {
                    log.Append(DateTime.Now, RosLog.INFO, "viewer", m.Data);
                }
            });
        });

        Button T(string en, string id, Func<ValueTask<LifecycleState>> run) => Ui.Button(Loc.T(en, id), () => _ = run().AsTask());
        var buttons = Ui.Card(Ui.Columns("Auto,*,Auto", 16, Ui.Row(8,
            T("Configure", "Configure", () => node.ConfigureAsync()),
            T("Activate", "Activate", () => node.ActivateAsync()),
            T("Deactivate", "Deactivate", () => node.DeactivateAsync()),
            T("Cleanup", "Cleanup", () => node.CleanupAsync()),
            T("Shutdown", "Shutdown", () => node.ShutdownAsync())), new Border(), recvView), padding: 14);

        Dispatcher.UIThread.Post(async () =>
        {
            await node.ConfigureAsync();
            await node.ActivateAsync();
        }, DispatcherPriority.Background);
        return Ui.Rows("Auto,2*,*", 14, buttons, diagram, log);
    }

    /// <summary>Draws the four primary states and highlights the current one.</summary>
    private sealed class LifecycleDiagram : RosDrawnControl
    {
        private LifecycleState _state = LifecycleState.Unconfigured;

        public LifecycleDiagram() => Title = Loc.T("State machine", "Mesin status");

        public LifecycleState State
        {
            get => _state;
            set
            {
                _state = value;
                InvalidateVisual();
            }
        }

        protected override void RenderContent(DrawingContext context, Rect bounds)
        {
            var states = new[] { LifecycleState.Unconfigured, LifecycleState.Inactive, LifecycleState.Active, LifecycleState.Finalized };
            var w = Math.Min(170, (bounds.Width - 80) / 4);
            var gap = (bounds.Width - (4 * w)) / 5;
            var y = bounds.Center.Y - 24;
            var transient = _state switch
            {
                LifecycleState.Configuring or LifecycleState.CleaningUp => LifecycleState.Inactive,
                LifecycleState.Activating => LifecycleState.Active,
                LifecycleState.Deactivating => LifecycleState.Inactive,
                LifecycleState.ShuttingDown or LifecycleState.ErrorProcessing => LifecycleState.Finalized,
                _ => (LifecycleState?)null,
            };

            for (var i = 0; i < states.Length; i++)
            {
                var r = new Rect(gap + (i * (w + gap)), y, w, 48);
                if (i < states.Length - 1)
                {
                    var a = new Point(r.Right + 4, r.Center.Y);
                    var b = new Point(r.Right + gap - 4, r.Center.Y);
                    context.DrawLine(P.Pen(P.Seam, 2), a, b);
                    context.DrawLine(P.Pen(P.Seam, 2), b, new Point(b.X - 7, b.Y - 5));
                    context.DrawLine(P.Pen(P.Seam, 2), b, new Point(b.X - 7, b.Y + 5));
                }

                var current = states[i] == _state;
                var pending = transient == states[i];
                var accent = states[i] switch
                {
                    LifecycleState.Active => P.Ok,
                    LifecycleState.Finalized => P.Fault,
                    LifecycleState.Inactive => P.Alert,
                    _ => P.Haze,
                };
                context.DrawRectangle(current ? new SolidColorBrush(accent, 0.22) : P.Brush(P.SurfaceRaised), new Pen(new SolidColorBrush(current || pending ? accent : P.Seam), current ? 2.5 : 1.2, dashStyle: pending ? new DashStyle([3, 3], 0) : null), r, 12, 12);
                DrawLabel(context, LifecycleNode.Label(states[i]), r.Center, 13, current ? P.Ink : P.Haze, RosTheme.SansSemiBold, TextAnchor.Center, verticalCenter: true);
            }

            DrawLabel(context, $"{Loc.T("current", "saat ini")}: {LifecycleNode.Label(_state)}", new Point(bounds.Center.X, y + 76), 12, P.Haze, RosTheme.Mono, TextAnchor.Center);
        }
    }
}
