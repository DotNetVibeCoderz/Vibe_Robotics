namespace ROS2.Net.AppGen;

// Project skeleton sources shared by every template. Templates provide Program.cs (console),
// MainWindow.cs (Avalonia: a Window with a (Ros2Context) constructor) or Components/Pages/Home.razor
// (Blazor, optionally with Services/AppRuntime.cs as a hosted service).
public sealed partial class ProjectService
{
    private const string BlankConsoleProgram = """
        // {{ProjectName}} — a ROS2.Net console app (made with ROS2AppGen).
        // Runs against the simulator by default; ROS2NET_TARGET=robot drives a real robot (see AppTarget.cs).
        using ROS2.Net;
        using ROS2.Net.Msgs.GeometryMsgs;
        using ROS2.Net.Msgs.NavMsgs;
        using ROS2.Net.Simulation;
        using StdMsgs = ROS2.Net.Msgs.StdMsgs;

        await using var context = await AppTarget.CreateContextAsync(args);
        using var sim = AppTarget.StartSimulator(context, ScenarioKind.Empty);
        Console.WriteLine(AppTarget.Describe());

        var talker = context.CreateNode("talker");
        var publisher = talker.CreatePublisher<StdMsgs.String>("chatter");
        var count = 0;
        talker.CreateTimer(TimeSpan.FromSeconds(1), () => publisher.Publish(new StdMsgs.String { Data = $"Hello from {{ProjectName}} #{++count}" }));

        var listener = context.CreateNode("listener");
        listener.CreateSubscription<StdMsgs.String>("chatter", msg => Console.WriteLine($"I heard: {msg.Data}"));
        listener.CreateSubscription<Odometry>(AppTarget.Topic("odom"), odom =>
        {
            if (count % 5 == 0)
            {
                Console.WriteLine($"robot at ({odom.Pose.Pose.Position.X:0.00}, {odom.Pose.Pose.Position.Y:0.00})");
            }
        });

        // Drive slowly in a circle. SAFETY: every command passes through AppTarget.Limit (velocity limits).
        var cmd = talker.CreatePublisher<Twist>(AppTarget.Topic("cmd_vel"));
        talker.CreateTimer(TimeSpan.FromMilliseconds(100), () =>
            cmd.Publish(AppTarget.Limit(new Twist { Linear = new Vector3 { X = 0.2 }, Angular = new Vector3 { Z = 0.3 } })));

        Console.WriteLine("Running — press Ctrl+C to stop.");
        try
        {
            await context.SpinAsync();
        }
        finally
        {
            cmd.Publish(new Twist()); // stop the robot on exit
        }
        """;

    private const string BlankAvaloniaWindow = """
        using Avalonia.Controls;
        using Avalonia.Layout;
        using Avalonia.Media;
        using ROS2.Net;
        using ROS2.Net.Avalonia;
        using ROS2.Net.Avalonia.Controls;
        using ROS2.Net.Simulation;

        namespace {{Namespace}};

        /// <summary>Main window: drive the robot (simulated, or real with ROS2NET_TARGET=robot) with the joystick.</summary>
        public sealed class MainWindow : Window
        {
            public MainWindow(Ros2Context context)
            {
                Title = $"{{ProjectName}} — {AppTarget.Describe()}";
                Width = 1100;
                Height = 700;
                Background = new SolidColorBrush(RosTheme.Current.Background);

                var sim = AppTarget.StartSimulator(context, ScenarioKind.Empty);
                var node = context.CreateNode("ui");

                // Simulator: top-down world view. Physical robot: the robot's own LiDAR.
                Control main;
                if (AppTarget.World is { } world)
                {
                    var view = new SimWorldView { ShowLidar = true, ShowTrails = true };
                    view.Attach(world);
                    main = view;
                }
                else
                {
                    var scan = new LaserScanView();
                    scan.Attach(node, AppTarget.Topic("scan"));
                    main = scan;
                }

                // SAFETY: the joystick's limits come from AppTarget (conservative for real robots).
                var joystick = new TeleopJoystick { Width = 220, Height = 220, MaxLinear = AppTarget.MaxLinearSpeed, MaxAngular = AppTarget.MaxAngularSpeed };
                joystick.Attach(node, AppTarget.Topic("cmd_vel"));

                var side = new StackPanel { Margin = new Avalonia.Thickness(16), Spacing = 12, Width = 260 };
                side.Children.Add(new TextBlock { Text = "{{ProjectName}}", FontSize = 22, FontWeight = FontWeight.Bold, Foreground = new SolidColorBrush(RosTheme.Current.Ink) });
                side.Children.Add(new TextBlock { Text = AppTarget.Describe(), Foreground = new SolidColorBrush(RosTheme.Current.Haze), TextWrapping = TextWrapping.Wrap });
                side.Children.Add(joystick);

                var root = new DockPanel();
                DockPanel.SetDock(side, Dock.Right);
                root.Children.Add(side);
                root.Children.Add(main);
                Content = root;
                Closed += (_, _) => sim?.Dispose();
            }
        }
        """;

    private const string BlankBlazorHome = """
        @page "/"
        @using ROS2.Net.Blazor.Components

        <PageTitle>{{ProjectName}}</PageTitle>

        <h1>{{ProjectName}}</h1>
        <p class="muted">A ROS2.Net web dashboard. Topics in this process (and on the network, with the RTPS middleware) appear below.</p>

        <section class="grid">
            <div class="card"><h2>Topics</h2><TopicMonitor /></div>
            <div class="card"><h2>Log</h2><LogConsole /></div>
        </section>
        """;

    private const string AvaloniaProgram = """
        using Avalonia;

        namespace {{Namespace}};

        internal static class Program
        {
            [STAThread]
            public static void Main(string[] args) => BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);

            public static AppBuilder BuildAvaloniaApp() => AppBuilder.Configure<App>()
                .UsePlatformDetect()
                .WithInterFont()
                .LogToTrace();
        }
        """;

    private const string AvaloniaApp = """
        using Avalonia;
        using Avalonia.Controls.ApplicationLifetimes;
        using Avalonia.Styling;
        using Avalonia.Themes.Fluent;
        using ROS2.Net;

        namespace {{Namespace}};

        /// <summary>Application: one ROS context for the app, spun in the background (in-process middleware by default).</summary>
        public sealed class App : Application
        {
            private Ros2Context? _context;
            private readonly CancellationTokenSource _spin = new();

            public override void Initialize()
            {
                Styles.Add(new FluentTheme());
                RequestedThemeVariant = ThemeVariant.Dark;
            }

            public override void OnFrameworkInitializationCompleted()
            {
                if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
                {
                    // Simulator or physical robot, per AppTarget (ROS2NET_TARGET / ROS2NET_RMW ...).
                    _context = Ros2Context.Create(AppTarget.ContextOptions(desktop.Args));
                    desktop.MainWindow = new MainWindow(_context);
                    _ = _context.SpinAsync(_spin.Token);
                    desktop.ShutdownRequested += (_, _) => _spin.Cancel();
                    desktop.Exit += (_, _) => _context.Dispose();
                }

                base.OnFrameworkInitializationCompleted();
            }
        }
        """;

    private const string AppManifest = """
        <?xml version="1.0" encoding="utf-8"?>
        <assembly manifestVersion="1.0" xmlns="urn:schemas-microsoft-com:asm.v1">
          <assemblyIdentity version="1.0.0.0" name="{{ProjectName}}.Desktop" />
          <application xmlns="urn:schemas-microsoft-com:asm.v3">
            <windowsSettings>
              <dpiAware xmlns="http://schemas.microsoft.com/SMI/2005/WindowsSettings">true</dpiAware>
            </windowsSettings>
          </application>
        </assembly>
        """;

    private static string BlazorProgram(bool hasRuntime) =>
        "using ROS2.Net.Blazor;" + Environment.NewLine +
        "using {{Namespace}}.Components;" + Environment.NewLine +
        (hasRuntime ? "using {{Namespace}}.Services;" + Environment.NewLine : string.Empty) +
        """

        var builder = WebApplication.CreateBuilder(args);
        builder.Services.AddRazorComponents().AddInteractiveServerComponents();

        // One ROS context for the app; AppTarget picks simulator (in-process by default) or a physical robot.
        builder.Services.AddRos2Blazor(AppTargetHosting.Configure);

        """ +
        (hasRuntime
            ? "builder.Services.AddSingleton<AppRuntime>();" + Environment.NewLine + "builder.Services.AddHostedService(sp => sp.GetRequiredService<AppRuntime>());" + Environment.NewLine
            : "builder.Services.AddHostedService<AppTargetSimulator>(); // starts the simulator in simulator mode" + Environment.NewLine) +
        """

        var app = builder.Build();
        if (!app.Environment.IsDevelopment())
        {
            app.UseExceptionHandler("/Error", createScopeForErrors: true);
        }

        app.UseAntiforgery();
        app.MapStaticAssets();
        app.MapRazorComponents<App>().AddInteractiveServerRenderMode();
        app.Run();

        """;

    private const string BlazorApp = """
        <!DOCTYPE html>
        <html lang="en" data-theme="dark">
        <head>
            <meta charset="utf-8" />
            <meta name="viewport" content="width=device-width, initial-scale=1.0" />
            <base href="/" />
            <title>{{ProjectName}}</title>
            <link rel="stylesheet" href="@Assets["_content/ROS2.Net.Blazor/ros2net.css"]" />
            <link rel="stylesheet" href="@Assets["app.css"]" />
            <ImportMap />
            <HeadOutlet @rendermode="ServerNoPrerender" />
        </head>
        <body>
            <Routes @rendermode="ServerNoPrerender" />
            <script src="_framework/blazor.web.js"></script>
        </body>
        </html>

        @code {
            // Components subscribe to topics when they initialize; prerendering would subscribe twice.
            private static readonly IComponentRenderMode ServerNoPrerender = new InteractiveServerRenderMode(prerender: false);
        }
        """;

    private const string BlazorRoutes = """
        <Router AppAssembly="typeof(Program).Assembly">
            <Found Context="routeData">
                <RouteView RouteData="routeData" DefaultLayout="typeof(Layout.MainLayout)" />
                <FocusOnNavigate RouteData="routeData" Selector="h1" />
            </Found>
        </Router>
        """;

    private const string BlazorImports = """
        @using System.Net.Http
        @using Microsoft.AspNetCore.Components.Routing
        @using Microsoft.AspNetCore.Components.Web
        @using static Microsoft.AspNetCore.Components.Web.RenderMode
        @using ROS2.Net
        @using ROS2.Net.Blazor
        @using ROS2.Net.Blazor.Components
        @using {{Namespace}}
        @using {{Namespace}}.Components
        """;

    private const string BlazorLayout = """
        @inherits LayoutComponentBase

        <header class="bar">
            <span class="logo">◉ {{ProjectName}}</span>
            <span class="muted">powered by ROS2.Net</span>
        </header>
        <main>@Body</main>
        <footer class="muted">Made with ROS2.Net · Gravicode Studios, led by Kang Fadhil</footer>
        """;

    private const string BlazorCss = """
        :root { color-scheme: dark; }
        body { margin: 0; background: var(--rn-bg, #0f1020); color: var(--rn-ink, #e8e9f3); font-family: var(--rn-font, Inter, system-ui, sans-serif); }
        .bar { display: flex; gap: 16px; align-items: baseline; padding: 14px 28px; border-bottom: 1px solid var(--rn-seam, #2a2c4a); }
        .logo { font-weight: 700; font-size: 18px; background: linear-gradient(90deg, #7c4dff, #00d1ff); -webkit-background-clip: text; color: transparent; }
        main { padding: 24px 28px; }
        h1 { margin: 0 0 6px; font-size: 28px; }
        h2 { margin: 0 0 10px; font-size: 13px; letter-spacing: .08em; text-transform: uppercase; color: var(--rn-haze, #9aa0c0); }
        .muted { color: var(--rn-haze, #9aa0c0); }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(380px, 1fr)); gap: 16px; margin-top: 18px; }
        .card { background: var(--rn-surface, #16182e); border: 1px solid var(--rn-seam, #2a2c4a); border-radius: 12px; padding: 16px; }
        .row { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; }
        button { background: linear-gradient(90deg, #7c4dff, #00d1ff); border: 0; color: white; padding: 8px 14px; border-radius: 8px; font-weight: 600; cursor: pointer; }
        button.secondary { background: var(--rn-surface-raised, #1e2140); border: 1px solid var(--rn-seam, #2a2c4a); }
        table { width: 100%; border-collapse: collapse; font-size: 14px; }
        td, th { text-align: left; padding: 6px 8px; border-bottom: 1px solid var(--rn-seam, #2a2c4a); }
        .ok { color: #3ddc97; } .warn { color: #ffb74d; } .bad { color: #ff5c7a; }
        footer { padding: 16px 28px; font-size: 12px; }
        """;

    private const string LaunchSettings = """
        {
          "profiles": {
            "http": {
              "commandName": "Project",
              "launchBrowser": true,
              "applicationUrl": "http://localhost:5280",
              "environmentVariables": { "ASPNETCORE_ENVIRONMENT": "Development" }
            }
          }
        }
        """;
}
