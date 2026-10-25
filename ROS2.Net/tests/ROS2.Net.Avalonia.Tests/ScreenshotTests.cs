using Ros2NetGallery;
using Shouldly;

namespace ROS2.Net.Avalonia.Tests;

/// <summary>
/// Renders every gallery page (and the simulator) headlessly with Skia and writes the PNGs used in the
/// docs to <c>docs/images/desktop</c>. Regenerate with:
/// <c>dotnet test tests/ROS2.Net.Avalonia.Tests --filter Category=Screenshots</c>.
/// </summary>
[Collection("headless")]
public class ScreenshotTests
{
    private static string OutDir => Path.Combine(Headless.RepoRoot, "docs", "images", "desktop");

    [Fact]
    [Trait("Category", "Screenshots")]
    public void Gallery_pages_render_to_png()
    {
        var written = Headless.Run(() =>
        {
            RosTheme.IsDark = true;
            Loc.IsIndonesian = false;
            var runtime = new GalleryRuntime(domainId: 191);
            var window = new GalleryWindow(runtime) { Width = 1440, Height = 900 };
            window.Show();
            var files = new List<string>();
            try
            {
                foreach (var id in window.PageIds)
                {
                    window.Navigate(id);
                    Headless.Pump(id switch { "bag" => 6000, "home" or "teleop" or "lidar" or "rl" or "commander" or "control" => 2500, _ => 1500 });
                    var path = Path.Combine(OutDir, id == "home" ? "gallery-home.png" : $"gallery-{id}.png");
                    Headless.Capture(window, path);
                    files.Add(path);
                }

                // The Indonesian and light variants of the home page, for the docs.
                Loc.IsIndonesian = true;
                window.Navigate("home");
                Headless.Pump(1500);
                Headless.Capture(window, Path.Combine(OutDir, "gallery-home-id.png"));
                Loc.IsIndonesian = false;
                window.SetDark(false);
                window.Navigate("home");
                Headless.Pump(1500);
                Headless.Capture(window, Path.Combine(OutDir, "gallery-home-light.png"));
                window.SetDark(true);
            }
            finally
            {
                window.Close();
                runtime.Dispose();
            }

            return files;
        });

        written.Count.ShouldBeGreaterThan(5);
        foreach (var f in written)
        {
            new FileInfo(f).Length.ShouldBeGreaterThan(20_000, f);
        }
    }
}
