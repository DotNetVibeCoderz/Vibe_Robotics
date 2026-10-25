using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Platform.Storage;
using Ros2AppGen.Ui;

namespace Ros2AppGen.Views;

/// <summary>Options for <c>dotnet publish</c>.</summary>
public sealed record DeployOptions(string OutputFolder, string Runtime, bool SelfContained);

/// <summary>Chooses where and for which platform to publish the open project.</summary>
public sealed class DeployDialog : AppDialog
{
    public static readonly string[] Runtimes = ["win-x64", "win-arm64", "linux-x64", "linux-arm64", "osx-arm64", "osx-x64"];

    public DeployDialog(string projectFolder)
        : base("Deploy", "Publish the app with dotnet publish. Linux ARM64 targets Raspberry Pi and Jetson boards.", 560, 400)
    {
        var output = Kit.Input(Path.Combine(projectFolder, "publish"), mono: true);
        var browse = Kit.Button("Browse…", () => _ = BrowseAsync(output));
        browse.Margin = new Thickness(6, 0, 0, 0);
        var runtime = new ComboBox
        {
            ItemsSource = Runtimes,
            SelectedItem = OperatingSystem.IsWindows() ? "win-x64" : OperatingSystem.IsMacOS() ? "osx-arm64" : "linux-x64",
            Width = 200,
        };
        var selfContained = new CheckBox { Content = Kit.Text("Self-contained (no .NET runtime needed on the target)", 12.5), IsChecked = true };

        SetBody(Kit.Col(16,
            Kit.Field("Output folder", new DockPanel { Children = { Kit.Dock(browse, Avalonia.Controls.Dock.Right), output } }),
            Kit.Field("Target runtime", runtime),
            selfContained));

        AddButton("Cancel", () => Close(null));
        AddButton("Deploy", () => Close(new DeployOptions(output.Text ?? string.Empty, runtime.SelectedItem as string ?? "win-x64", selfContained.IsChecked == true)), primary: true);
    }

    private async Task BrowseAsync(TextBox target)
    {
        var folders = await StorageProvider.OpenFolderPickerAsync(new FolderPickerOpenOptions { Title = "Publish to" });
        if (folders.FirstOrDefault()?.TryGetLocalPath() is { } path)
        {
            target.Text = path;
        }
    }
}

/// <summary>Asks for a line number (Ctrl+G).</summary>
public sealed class GoToLineDialog : AppDialog
{
    public GoToLineDialog(int current, int max)
        : base("Go to line", $"Line 1 – {max}", 380, 210)
    {
        var box = Kit.Input(current.ToString(System.Globalization.CultureInfo.InvariantCulture), mono: true);
        void Submit()
        {
            if (int.TryParse((box.Text ?? string.Empty).Split(':')[0].Trim(), out var line))
            {
                Close(line);
            }
        }

        box.KeyDown += (_, e) =>
        {
            if (e.Key == Key.Enter)
            {
                Submit();
            }
        };
        SetBody(Kit.Field("Line number", box));
        AddButton("Cancel", () => Close(null));
        AddButton("Go", Submit, primary: true);
        Opened += (_, _) =>
        {
            box.Focus();
            box.SelectAll();
        };
    }
}
