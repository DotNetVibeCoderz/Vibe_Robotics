using Ros2AppGen;
using Ros2AppGen.Ui;
using Ros2AppGen.Views;
using Shouldly;

namespace ROS2.Net.AppGen.UI.Tests;

[Collection("headless")]
public class MainWindowTests
{
    private static MainWindow NewWindow(out AppGenHost host, string? root = null)
    {
        Look.IsDark = true;
        host = FakeHost.Create(root);
        var w = new MainWindow(host) { Width = 1440, Height = 900 };
        w.Show();
        return w;
    }

    [Fact]
    public void Opening_a_project_shows_its_tree_and_entry_point()
    {
        Headless.Run(() =>
        {
            var root = FakeHost.CreateSampleProject();
            var w = NewWindow(out var host);
            try
            {
                w.OpenProject(root);
                Headless.Pump(200);
                host.Workspace.RootPath.ShouldBe(root);
                w.Editor.Active.ShouldNotBeNull();
                w.Editor.Active!.FileName.ShouldBe("Program.cs");
                w.Explorer.VisibleRowCount.ShouldBeGreaterThan(5);
                host.Settings.RecentProjects[0].ShouldBe(root);
                w.Title!.ShouldContain(Path.GetFileName(root));
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void Editor_tracks_dirty_state_saves_and_goes_to_line()
    {
        Headless.Run(() =>
        {
            var root = FakeHost.CreateSampleProject();
            var w = NewWindow(out _, root);
            try
            {
                var doc = w.Editor.Open("Nodes/SafetyMonitor.cs")!;
                doc.IsDirty.ShouldBeFalse();
                w.Editor.Editor.Document.Insert(0, "// edited\n");
                doc.IsDirty.ShouldBeTrue();
                w.Editor.Save();
                doc.IsDirty.ShouldBeFalse();
                File.ReadAllText(Path.Combine(root, "Nodes", "SafetyMonitor.cs")).ShouldStartWith("// edited");

                w.Editor.GoToLine(5).ShouldBeTrue();
                w.Editor.Editor.TextArea.Caret.Line.ShouldBe(5);

                w.Editor.Editor.Document.Insert(0, "int x = 1;   \n");
                w.FormatCode();
                w.Editor.Editor.Document.Text.ShouldStartWith("int x = 1;\n");
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void Panels_and_line_numbers_toggle_and_persist_in_settings()
    {
        Headless.Run(() =>
        {
            var w = NewWindow(out var host);
            try
            {
                w.SetChatVisible(false);
                host.Settings.ChatPanelVisible.ShouldBeFalse();
                w.SetChatVisible(true);
                host.Settings.ChatPanelVisible.ShouldBeTrue();
                w.SetLogsVisible(false);
                w.Logs.IsVisible.ShouldBeFalse();
                w.SetLogsVisible(true);
                w.Logs.IsVisible.ShouldBeTrue();
                w.SetLineNumbers(false);
                w.Editor.ShowLineNumbers.ShouldBeFalse();
                host.Settings.ShowLineNumbers.ShouldBeFalse();
                w.SetDark(false);
                host.Settings.Theme.ShouldBe("Light");
                w.SetDark(true);
                w.PersistOnExit();
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void Chat_streams_tool_calls_and_reloads_files_jack_edits()
    {
        Headless.RunAsync(async () =>
        {
            var root = FakeHost.CreateSampleProject();
            var w = NewWindow(out _, root);
            try
            {
                var doc = w.Editor.Open("Nodes/FleetStatusNode.cs")!;
                await w.ChatViewModel.SendAsync("Track every robot's odometry in FleetStatusNode");
                Headless.Pump(300);

                w.ChatViewModel.Messages.Count.ShouldBe(2);
                var reply = w.ChatViewModel.Messages[1];
                reply.Tools.Count.ShouldBe(4);
                reply.Tools.ShouldAllBe(t => t.IsDone);
                reply.Text.ShouldContain("FleetStatusNode");
                doc.Document.Text.ShouldContain("Subscription<Odometry>".Replace("Subscription", "CreateSubscription"));
                w.Logs.JackText.ShouldContain("Project.WriteFile");

                w.ChatViewModel.Clear();
                w.ChatViewModel.Messages.ShouldBeEmpty();
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void Build_writes_output_and_status()
    {
        Headless.RunAsync(async () =>
        {
            var root = FakeHost.CreateSampleProject();
            var w = NewWindow(out _, root);
            try
            {
                var result = await w.BuildAsync();
                result!.Success.ShouldBeTrue();
                w.Logs.OutputText.ShouldContain("Build succeeded");
                w.Status.BuildText.ShouldStartWith("Build succeeded");
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void Run_uses_the_active_target_and_logs_its_environment()
    {
        Headless.RunAsync(async () =>
        {
            var root = FakeHost.CreateSampleProject();
            var w = NewWindow(out var host, root);
            try
            {
                var asked = 0;
                w.ConfirmRobotRun = _ =>
                {
                    asked++;
                    return Task.FromResult(true);
                };

                w.ActiveTarget.Kind.ShouldBe(RunTargetKind.Simulator);
                w.Status.TargetIsRobot.ShouldBeFalse();
                await w.RunAsync();
                var build = (FakeBuild)host.Build;
                build.LastTarget!.Name.ShouldBe("Simulator (in-process)");
                w.Logs.OutputText.ShouldContain("Run target: Simulator (in-process) (simulator)");
                w.Logs.OutputText.ShouldContain("ROS2NET_TARGET=sim");
                asked.ShouldBe(0);

                w.SetActiveTarget("Robot via rosbridge");
                w.Status.TargetIsRobot.ShouldBeTrue();
                w.Status.TargetText.ShouldContain("ROBOT");
                await w.RunAsync();
                await w.RunAsync();
                asked.ShouldBe(1, "the robot confirmation is asked once per target per session");
                build.LastTarget!.Kind.ShouldBe(RunTargetKind.PhysicalRobot);
                w.Logs.OutputText.ShouldContain("ROS2NET_ROSBRIDGE_URL=ws://");
                w.Logs.OutputText.ShouldContain("ROS2NET_MAX_LINEAR=0.3");
                host.Settings.ActiveRunTarget.ShouldBe("Robot via rosbridge");
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void Cancelling_the_robot_confirmation_does_not_run()
    {
        Headless.RunAsync(async () =>
        {
            var root = FakeHost.CreateSampleProject();
            var w = NewWindow(out var host, root);
            try
            {
                w.ConfirmRobotRun = _ => Task.FromResult(false);
                var robot = host.Settings.RunTargets.First(t => t.Kind == RunTargetKind.PhysicalRobot);
                await w.RunAsync(robot);
                ((FakeBuild)host.Build).LastTarget.ShouldBeNull();
                w.Logs.OutputText.ShouldContain("cancelled");
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void Secrets_in_extra_environment_are_masked()
    {
        MainWindow.MaskSecret("ROBOT_API_KEY", "abc").ShouldBe("••••");
        MainWindow.MaskSecret("ROS2NET_RMW", "rtps").ShouldBe("rtps");
    }

    [Fact]
    public void Settings_dialog_edits_a_copy_until_saved()
    {
        Headless.Run(() =>
        {
            var w = NewWindow(out var host);
            try
            {
                var dialog = new SettingsDialog(host);
                dialog.Show(w);
                dialog.Draft.Profiles[0].Temperature = 1.3;
                host.Settings.Profiles[0].Temperature.ShouldNotBe(1.3);
                foreach (var section in new[] { "Run targets", "Jack", "Web tools", "Projects", "Editor", "Layout", "Models" })
                {
                    dialog.ShowSection(section);
                    Headless.Pump(30);
                }

                dialog.Close();
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void New_project_dialog_lists_templates_by_category()
    {
        Headless.Run(() =>
        {
            var w = NewWindow(out var host);
            try
            {
                var dialog = new NewProjectDialog(host);
                dialog.Show(w);
                Headless.Pump(50);
                dialog.SetMode("blank");
                Headless.Pump(30);
                dialog.SetMode("template");
                dialog.SelectTemplate(host.Projects.Templates[^1].Id);
                Headless.Pump(30);
                dialog.Close();
                host.Projects.Categories.ShouldNotBeEmpty();
            }
            finally
            {
                w.Close();
            }
        });
    }

    [Fact]
    public void Markdown_renders_code_blocks_lists_and_inline_code()
    {
        Headless.Run(() =>
        {
            var view = MarkdownView.Render("# Title\n\nUse `QosProfile.SensorData` and **bold**.\n\n- one\n- two\n\n```csharp\nvar x = 1;\n```\n");
            view.ShouldBeOfType<Avalonia.Controls.StackPanel>().Children.Count.ShouldBe(5);
        });
    }
}
