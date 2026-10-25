using System.Diagnostics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using Microsoft.Extensions.Logging;
using ROS2.Net.Avalonia;
using ROS2.Net.Avalonia.Controls;
using ROS2.Net.Msgs.ExampleInterfaces;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace Ros2NetGallery.Pages;

/// <summary>Publisher, subscription and timer: the "hello world" of ROS 2, plus a live waveform.</summary>
public sealed class TalkerListenerPage : GalleryPage
{
    public override string Id => "talker";

    public override PageGroup Group => PageGroup.Communication;

    public override T2 Title => new("Talker & listener", "Talker & listener");

    public override T2 Summary => new(
        "A timer publishes std_msgs/String on /chatter and a sine wave on /wave; a second node subscribes to both. Change the rate and watch the stream.",
        "Sebuah timer menerbitkan std_msgs/String di /chatter dan gelombang sinus di /wave; node kedua berlangganan keduanya. Ubah laju dan amati alirannya.");

    public override IReadOnlyList<string> Interfaces => ["std_msgs/msg/String", "std_msgs/msg/Float64", "/chatter", "/wave"];

    public override string Code => """
        var talker = context.CreateNode("talker");
        var chatter = talker.CreatePublisher<StdMsgs.String>("chatter");
        var count = 0;
        var timer = talker.CreateTimer(TimeSpan.FromSeconds(0.2), () =>
            chatter.Publish(new() { Data = $"Hello ROS2.Net #{++count}" }));

        var listener = context.CreateNode("listener");
        listener.CreateSubscription<StdMsgs.String>("chatter", msg =>
            listener.Logger.LogInformation("I heard: {Data}", msg.Data));

        // Subscriptions without a callback are async streams:
        var wave = listener.CreateSubscription<StdMsgs.Float64>("wave");
        await foreach (var sample in wave.ReadAllAsync(ct))
            plot.AddSample("wave", now, sample.Data);

        await context.SpinAsync();
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var ctx = runtime.Context;
        var talker = Own(ctx.CreateNode("gallery_talker"));
        var listener = Own(ctx.CreateNode("gallery_listener"));
        var chatter = talker.CreatePublisher<StdMsgs.String>("/chatter");
        var wave = talker.CreatePublisher<StdMsgs.Float64>("/wave");
        var prefix = "Hello ROS2.Net";
        var sent = 0;
        var received = 0;
        var paused = false;
        var clock = Stopwatch.StartNew();

        var console = new LogConsole { Title = "Listener  /chatter" };
        var plot = new PlotView { Title = "/wave  std_msgs/Float64", WindowSeconds = 6, YRange = (-1.2, 1.2) };
        var (sentView, sentText) = Ui.Stat(Loc.T("Published", "Diterbitkan"), "0");
        var (recvView, recvText) = Ui.Stat(Loc.T("Received", "Diterima"), "0");

        var timer = talker.CreateTimer(TimeSpan.FromSeconds(0.2), () =>
        {
            if (paused)
            {
                return;
            }

            chatter.Publish(new StdMsgs.String { Data = $"{prefix} #{++sent}" });
            RosUi.OnUi(() => sentText.Text = sent.ToString());
        });
        talker.CreateTimer(TimeSpan.FromMilliseconds(40), () =>
        {
            if (!paused)
            {
                var t = clock.Elapsed.TotalSeconds;
                wave.Publish(new StdMsgs.Float64 { Data = Math.Sin(t * 2.2) * Math.Cos(t * 0.37) });
            }
        });

        listener.CreateSubscription<StdMsgs.String>("/chatter", msg =>
        {
            var n = Interlocked.Increment(ref received);
            RosUi.OnUi(() =>
            {
                console.Append(DateTime.Now, RosLog.INFO, "listener", $"I heard: [{msg.Data}]");
                recvText.Text = n.ToString();
            });
        });
        Own(RosUi.SubscribeAll<StdMsgs.Float64>(listener, "/wave", batch =>
        {
            var t = clock.Elapsed.TotalSeconds;
            for (var i = 0; i < batch.Count; i++)
            {
                plot.AddSample("wave", t - ((batch.Count - 1 - i) * 0.04), batch[i].Data);
            }
        }));

        var text = new TextBox { Text = prefix, MinWidth = 220 };
        text.TextChanged += (_, _) => prefix = text.Text ?? string.Empty;
        var pause = new ToggleSwitch { OnContent = Loc.T("Paused", "Jeda"), OffContent = Loc.T("Running", "Berjalan") };
        pause.IsCheckedChanged += (_, _) => paused = pause.IsChecked == true;

        var controls = Ui.Card(Ui.Columns("*,Auto,Auto,Auto", 24,
            Ui.Col(4, Ui.Muted(Loc.T("Message text", "Teks pesan"), 12), text),
            Ui.Slider(Loc.T("Rate", "Laju"), 1, 30, 5, v => timer.Period = TimeSpan.FromSeconds(1 / v), "0", " Hz"),
            Ui.Row(20, sentView, recvView),
            pause), padding: 14);

        return Ui.Rows("Auto,*,*", 14, controls, plot, console);
    }
}

/// <summary>Service server and async client with latency measurement.</summary>
public sealed class ServicesPage : GalleryPage
{
    public override string Id => "services";

    public override PageGroup Group => PageGroup.Communication;

    public override T2 Title => new("Services", "Service");

    public override T2 Summary => new(
        "Request/response over example_interfaces/srv/AddTwoInts. The client awaits the response directly — no spinning needed on the calling side.",
        "Request/response melalui example_interfaces/srv/AddTwoInts. Client langsung menunggu respons dengan await — sisi pemanggil tidak perlu spin.");

    public override IReadOnlyList<string> Interfaces => ["example_interfaces/srv/AddTwoInts", "/add_two_ints"];

    public override string Code => """
        var server = context.CreateNode("add_two_ints_server");
        server.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints",
            req => new AddTwoIntsResponse { Sum = req.A + req.B });

        var client = clientNode.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("add_two_ints");
        await client.WaitForServiceAsync(TimeSpan.FromSeconds(5));
        var res = await client.CallAsync(new AddTwoIntsRequest { A = 2, B = 40 }, TimeSpan.FromSeconds(1));
        Console.WriteLine(res.Sum); // 42
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var ctx = runtime.Context;
        var server = Own(ctx.CreateNode("gallery_add_server"));
        var calls = 0;
        server.CreateService<AddTwoIntsRequest, AddTwoIntsResponse>("/add_two_ints", req =>
        {
            Interlocked.Increment(ref calls);
            return new AddTwoIntsResponse { Sum = req.A + req.B };
        });
        var clientNode = Own(ctx.CreateNode("gallery_add_client"));
        var client = clientNode.CreateClient<AddTwoIntsRequest, AddTwoIntsResponse>("/add_two_ints");

        var a = new NumericUpDown { Value = 2, Minimum = -1_000_000, Maximum = 1_000_000, Increment = 1, FormatString = "0", MinWidth = 150 };
        var b = new NumericUpDown { Value = 40, Minimum = -1_000_000, Maximum = 1_000_000, Increment = 1, FormatString = "0", MinWidth = 150 };
        var (resultView, result) = Ui.Stat(Loc.T("Sum", "Jumlah"), "–");
        var (latencyView, latency) = Ui.Stat(Loc.T("Round trip", "Pulang-pergi"), "–");
        var plot = new PlotView { Title = Loc.T("Latency per call (µs)", "Latensi per panggilan (µs)"), WindowSeconds = 20, Unit = " µs" };
        var history = new LogConsole { Title = Loc.T("Calls", "Panggilan") };
        var clock = Stopwatch.StartNew();

        async Task CallAsync(long x, long y)
        {
            var sw = Stopwatch.StartNew();
            try
            {
                var res = await client.CallAsync(new AddTwoIntsRequest { A = x, B = y }, TimeSpan.FromSeconds(2));
                var us = sw.Elapsed.TotalMilliseconds * 1000;
                result.Text = res.Sum.ToString();
                latency.Text = $"{us:0} µs";
                plot.AddSample("latency", clock.Elapsed.TotalSeconds, us);
                history.Append(DateTime.Now, RosLog.INFO, "client", $"{x} + {y} = {res.Sum}   ({us:0} µs)");
            }
            catch (Exception ex) when (ex is TimeoutException or OperationCanceledException)
            {
                history.Append(DateTime.Now, RosLog.ERROR, "client", ex.Message);
            }
        }

        var call = Ui.Button(Loc.T("Call service", "Panggil service"), () => _ = CallAsync((long)(a.Value ?? 0), (long)(b.Value ?? 0)), primary: true);
        var burst = Ui.Button(Loc.T("Burst ×50", "Beruntun ×50"), async () =>
        {
            var r = new Random();
            for (var i = 0; i < 50; i++)
            {
                await CallAsync(r.Next(-100, 100), r.Next(-100, 100));
            }
        });

        var form = Ui.Card(Ui.Columns("Auto,Auto,Auto,*,Auto", 18,
            Ui.Col(4, Ui.Muted("a (int64)", 12), a),
            Ui.Col(4, Ui.Muted("b (int64)", 12), b),
            Ui.Row(8, call, burst),
            new Border(),
            Ui.Row(28, resultView, latencyView)), padding: 14);

        Dispatcher.UIThread.Post(() => _ = CallAsync(2, 40), DispatcherPriority.Background);
        return Ui.Rows("Auto,*,*", 14, form, plot, history);
    }
}

/// <summary>Action server/client: goal, streaming feedback, result and cancellation.</summary>
public sealed class ActionsPage : GalleryPage
{
    public override string Id => "actions";

    public override PageGroup Group => PageGroup.Communication;

    public override T2 Title => new("Actions", "Action");

    public override T2 Summary => new(
        "Long-running goals with feedback: example_interfaces/action/Fibonacci over the standard _action/* services and topics, interoperable with `ros2 action send_goal`.",
        "Goal jangka panjang dengan feedback: example_interfaces/action/Fibonacci melalui service dan topic standar _action/*, kompatibel dengan `ros2 action send_goal`.");

    public override IReadOnlyList<string> Interfaces => ["example_interfaces/action/Fibonacci", "/fibonacci/_action/feedback", "/fibonacci/_action/status"];

    public override string Code => """
        server.CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("fibonacci",
            async (goal, ct) =>
            {
                var seq = new List<int> { 0, 1 };
                for (var i = 1; i < goal.Goal.Order; i++)
                {
                    ct.ThrowIfCancellationRequested();   // honors cancel requests
                    seq.Add(seq[i] + seq[i - 1]);
                    goal.PublishFeedback(new() { Sequence = seq.ToArray() });
                    await Task.Delay(250, ct);
                }
                return new FibonacciResult { Sequence = seq.ToArray() };
            });

        var client = node.CreateActionClient<FibonacciGoal, FibonacciResult, FibonacciFeedback>("fibonacci");
        var handle = await client.SendGoalAsync(new() { Order = 12 }, fb => progress = fb.Sequence.Length);
        var result = await handle.GetResultAsync();   // Succeeded / Canceled / Aborted
        """;

    public override Control Build(GalleryRuntime runtime)
    {
        var p = RosTheme.Current;
        var ctx = runtime.Context;
        var server = Own(ctx.CreateNode("gallery_fibonacci_server"));
        server.CreateActionServer<FibonacciGoal, FibonacciResult, FibonacciFeedback>("/fibonacci", async (goal, ct) =>
        {
            var seq = new List<int> { 0, 1 };
            for (var i = 1; i < goal.Goal.Order; i++)
            {
                ct.ThrowIfCancellationRequested();
                seq.Add(seq[i] + seq[i - 1]);
                goal.PublishFeedback(new FibonacciFeedback { Sequence = seq.ToArray() });
                await Task.Delay(220, ct);
            }

            return new FibonacciResult { Sequence = seq.ToArray() };
        });
        var clientNode = Own(ctx.CreateNode("gallery_fibonacci_client"));
        var client = clientNode.CreateActionClient<FibonacciGoal, FibonacciResult, FibonacciFeedback>("/fibonacci");

        var order = 14;
        var progress = new ProgressBar { Minimum = 0, Maximum = order, Height = 10, CornerRadius = new CornerRadius(5), Foreground = p.SignalGradient() };
        var status = Ui.Chip("idle", p.Haze);
        var sequence = Ui.Text("–", 15, mono: true);
        var bars = new FibonacciBars();
        ClientGoalHandle<FibonacciGoal, FibonacciResult, FibonacciFeedback>? handle = null;

        void SetStatus(string s, Color c)
        {
            status.Child = Ui.Text(s, 11, mono: true, wrap: false);
            status.Background = new SolidColorBrush(c, 0.15);
            status.BorderBrush = new SolidColorBrush(c, 0.6);
        }

        async Task SendAsync()
        {
            if (handle is { Status: GoalStatus.Executing or GoalStatus.Accepted })
            {
                return;
            }

            progress.Maximum = order;
            progress.Value = 0;
            try
            {
                handle = await client.SendGoalAsync(new FibonacciGoal { Order = order }, fb => RosUi.OnUi(() =>
                {
                    progress.Value = fb.Sequence.Length - 1;
                    sequence.Text = string.Join(", ", fb.Sequence);
                    bars.Values = fb.Sequence;
                }));
                SetStatus("executing", p.Telemetry);
                var r = await handle.GetResultAsync();
                SetStatus(r.Status.ToString().ToLowerInvariant(), r.Status == GoalStatus.Succeeded ? p.Ok : p.Alert);
                if (r.Result.Sequence.Length > 0)
                {
                    sequence.Text = string.Join(", ", r.Result.Sequence);
                    bars.Values = r.Result.Sequence;
                }
            }
            catch (Exception ex) when (ex is GoalRejectedException or TimeoutException)
            {
                SetStatus("rejected", p.Fault);
            }
        }

        var controls = Ui.Card(Ui.Columns("Auto,Auto,*,Auto", 20,
            Ui.Slider(Loc.T("Order", "Orde"), 3, 30, order, v => order = (int)v, "0"),
            Ui.Row(8,
                Ui.Button(Loc.T("Send goal", "Kirim goal"), () => _ = SendAsync(), primary: true),
                Ui.Button(Loc.T("Cancel", "Batalkan"), () => _ = handle?.CancelAsync())),
            new Border(),
            status), padding: 14);

        var feedback = Ui.Card(Ui.Col(12, progress, sequence), Loc.T("Feedback stream", "Aliran feedback"));
        Dispatcher.UIThread.Post(() => _ = SendAsync(), DispatcherPriority.Background);
        return Ui.Rows("Auto,Auto,*", 14, controls, feedback, bars);
    }

    /// <summary>Bars of the sequence, log-scaled.</summary>
    private sealed class FibonacciBars : RosDrawnControl
    {
        private int[] _values = [];

        public FibonacciBars() => Title = Loc.T("Sequence (log scale)", "Deret (skala log)");

        public int[] Values
        {
            get => _values;
            set
            {
                _values = value;
                InvalidateVisual();
            }
        }

        protected override void RenderContent(DrawingContext context, Rect bounds)
        {
            if (_values.Length == 0)
            {
                DrawEmpty(context, bounds, "–");
                return;
            }

            var max = Math.Log10(Math.Max(10, _values.Max()) + 1);
            var w = (bounds.Width - 32) / Math.Max(_values.Length, 12);
            for (var i = 0; i < _values.Length; i++)
            {
                var h = (bounds.Height - 60) * Math.Log10(_values[i] + 1) / max;
                var r = new Rect(16 + (i * w) + 2, bounds.Bottom - 22 - h, Math.Max(2, w - 4), Math.Max(2, h));
                context.DrawRectangle(P.SignalGradient(new RelativePoint(0, 1, RelativeUnit.Relative), new RelativePoint(0, 0, RelativeUnit.Relative)), null, r, 3, 3);
                if (w > 26)
                {
                    DrawLabel(context, _values[i].ToString(), new Point(r.Center.X, r.Y - 15), 9.5, P.Haze, RosTheme.Mono, TextAnchor.Center);
                }
            }
        }
    }
}
