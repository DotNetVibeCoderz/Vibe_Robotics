using Ros2AppGen;
using Ros2AppGen.Ui;
using Ros2AppGen.Views;
using Shouldly;

namespace ROS2.Net.AppGen.UI.Tests;

/// <summary>
/// Renders the app headlessly with Skia and writes the PNGs used in the docs to <c>docs/images/appgen</c>.
/// Regenerate with <c>dotnet test tests/ROS2.Net.AppGen.UI.Tests --filter Category=Screenshots</c>.
/// </summary>
[Collection("headless")]
public class ScreenshotTests
{
    private static string OutDir => Path.Combine(Headless.RepoRoot, "docs", "images", "appgen");

    [Fact]
    [Trait("Category", "Screenshots")]
    public void App_renders_to_png()
    {
        var files = new List<string>();
        Headless.RunAsync(async () =>
        {
            Look.IsDark = true;
            var root = FakeHost.CreateSampleProject();
            var host = FakeHost.Create();
            var w = new MainWindow(host) { Width = 1600, Height = 960 };
            w.Show();
            try
            {
                // Welcome.
                Headless.Pump(400);
                Capture(w, "appgen-welcome.png");

                // A project with a conversation.
                w.OpenProject(root);
                w.Editor.Open("Nodes/SafetyMonitor.cs");
                w.Editor.GoToLine(22);
                await w.ChatViewModel.SendAsync("Make FleetStatusNode track the odometry of every robot, then build.");
                w.Editor.Open("Nodes/FleetStatusNode.cs");
                w.Logs.WriteOutput("> dotnet build WarehouseMonitor");
                w.Logs.WriteOutput("Build succeeded.    0 Warning(s)    0 Error(s)");
                w.Explorer.Reveal(Path.Combine(root, "Nodes", "FleetStatusNode.cs"));
                Headless.Pump(700);
                Capture(w, "appgen-main.png");

                // New project gallery.
                var np = new NewProjectDialog(host);
                np.Show(w);
                np.SelectTemplate(host.Projects.Templates.FirstOrDefault(t => t.Id.Contains("warehouse", StringComparison.OrdinalIgnoreCase))?.Id ?? host.Projects.Templates[0].Id);
                Headless.Pump(400);
                Capture(np, "appgen-new-project.png");
                np.Close();

                // Settings.
                var settings = new SettingsDialog(host);
                settings.Show(w);
                Headless.Pump(400);
                Capture(settings, "appgen-settings.png");
                settings.SelectTarget("Robot over DDS/RTPS (LAN)");
                Headless.Pump(400);
                Capture(settings, "appgen-run-targets.png");
                settings.Close();

                // The safety confirmation shown before the first run on a robot.
                var safety = new RobotSafetyDialog(host.Settings.RunTargets.First(t => t.Kind == RunTargetKind.PhysicalRobot));
                safety.Show(w);
                Headless.Pump(300);
                Capture(safety, "appgen-robot-safety.png");
                safety.Close();

                // Light theme.
                w.SetActiveTarget("Robot over DDS/RTPS (LAN)");
                w.SetDark(false);
                w.Editor.Open("Nodes/SafetyMonitor.cs");
                Headless.Pump(600);
                Capture(w, "appgen-light.png");
                w.SetDark(true);
            }
            finally
            {
                w.Close();
            }
        });

        void Capture(Avalonia.Controls.TopLevel top, string name)
        {
            var path = Path.Combine(OutDir, name);
            Headless.Capture(top, path);
            files.Add(path);
        }

        files.Count.ShouldBe(7);
        files.ShouldAllBe(f => new FileInfo(f).Length > 20_000);
    }
}
