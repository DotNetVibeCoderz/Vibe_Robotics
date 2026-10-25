using System.Diagnostics;
using System.Net.Http;
using Microsoft.Playwright;

namespace ROS2.Net.Blazor.Tests;

/// <summary>
/// Captures the gallery pages into docs/images/web. Opt-in (it starts the web app and a browser):
/// set <c>ROS2NET_SCREENSHOTS=1</c>, or <c>ROS2NET_GALLERY_URL</c> to use a server that is already running.
/// </summary>
public class Screenshots
{
    private static readonly string[] s_pages =
    [
        "", "talker", "services", "actions", "parameters", "lifecycle", "teleop", "lidar", "navigation", "slam", "tf",
        "pointcloud", "arm", "bag", "rl", "commander", "connectivity",
    ];

    [Fact]
    public async Task Capture_gallery_pages()
    {
        var external = Environment.GetEnvironmentVariable("ROS2NET_GALLERY_URL");
        Assert.SkipUnless(external is not null || Environment.GetEnvironmentVariable("ROS2NET_SCREENSHOTS") == "1",
            "Set ROS2NET_SCREENSHOTS=1 (or ROS2NET_GALLERY_URL) to capture screenshots.");

        var root = FindRepoRoot();
        var outDir = Path.Combine(root, "docs", "images", "web");
        Directory.CreateDirectory(outDir);

        Process? server = null;
        var baseUrl = external ?? "http://localhost:5329";
        if (external is null)
        {
            server = Process.Start(new ProcessStartInfo("dotnet", $"run --no-build --project \"{Path.Combine(root, "apps", "ROS2.Net.Gallery.Web")}\" --urls {baseUrl}")
            {
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
            })!;
            server.OutputDataReceived += (_, _) => { };
            server.ErrorDataReceived += (_, _) => { };
            server.BeginOutputReadLine();
            server.BeginErrorReadLine();
            await WaitForServerAsync(baseUrl);
        }

        try
        {
            using var playwright = await Playwright.CreateAsync();
            // Prefer the installed Edge; fall back to Playwright's bundled Chromium.
            IBrowser browser;
            try
            {
                browser = await playwright.Chromium.LaunchAsync(new() { Headless = true, Channel = "msedge" });
            }
            catch (PlaywrightException)
            {
                browser = await playwright.Chromium.LaunchAsync(new() { Headless = true });
            }

            await using var _ = browser;
            foreach (var theme in new[] { "dark" })
            {
                var page = await browser.NewPageAsync(new() { ViewportSize = new() { Width = 1440, Height = 900 }, DeviceScaleFactor = 1 });
                await page.AddInitScriptAsync($"try {{ localStorage.setItem('rn-theme', '{theme}'); }} catch {{}}");
                var only = Environment.GetEnvironmentVariable("ROS2NET_SCREENSHOT_PAGES")?.Split(',', StringSplitOptions.TrimEntries);
                foreach (var p in s_pages.Where(x => only is null || only.Contains(x.Length == 0 ? "home" : x)))
                {
                    await page.GotoAsync($"{baseUrl}/{p}");
                    await page.WaitForSelectorAsync("h1", new() { Timeout = 20000 });
                    await Interact(page, p);
                    await page.WaitForTimeoutAsync(p is "rl" or "slam" or "navigation" or "bag" ? 9000 : 3500);
                    var name = p.Length == 0 ? "gallery-home.png" : $"gallery-{p}.png";
                    await page.ScreenshotAsync(new() { Path = Path.Combine(outDir, name) });
                }

                // Indonesian, then light theme.
                await page.GotoAsync(baseUrl + "/");
                await page.WaitForSelectorAsync("h1", new() { Timeout = 20000 });
                await page.ClickAsync(".seg button:has-text('ID')");
                await page.WaitForTimeoutAsync(2500);
                await page.ScreenshotAsync(new() { Path = Path.Combine(outDir, "gallery-home-id.png") });
                await page.ClickAsync(".seg button:has-text('EN')");
                await page.ClickAsync("button.theme");
                await page.WaitForTimeoutAsync(2500);
                await page.ScreenshotAsync(new() { Path = Path.Combine(outDir, "gallery-home-light.png") });
                await page.ClickAsync("button.theme");
                await page.CloseAsync();
            }
        }
        finally
        {
            if (server is not null)
            {
                try
                {
                    server.Kill(entireProcessTree: true);
                }
                catch (InvalidOperationException)
                {
                }
            }
        }
    }

    /// <summary>Puts some pages into an interesting state before capturing.</summary>
    private static async Task Interact(IPage page, string name)
    {
        try
        {
            switch (name)
            {
                case "actions":
                    await page.ClickAsync("button:has-text('Send goal')", new() { Timeout = 4000 });
                    break;
                case "services":
                    await page.ClickAsync("button:has-text('Call')", new() { Timeout = 4000 });
                    break;
                case "navigation":
                    await page.WaitForTimeoutAsync(1500);
                    await page.ClickAsync("button.goal-preset >> nth=1", new() { Timeout = 4000 });
                    break;
                case "bag":
                    await page.ClickAsync("button:has-text('Record 5 s')", new() { Timeout = 4000 });
                    break;
                case "rl":
                    await page.ClickAsync("button:has-text('Train')", new() { Timeout = 4000 });
                    break;
                case "commander":
                    await page.FillAsync("input.command", "rotate 90 degrees then move forward 1 meter", new() { Timeout = 4000 });
                    await page.ClickAsync("button:has-text('Send')", new() { Timeout = 4000 });
                    break;
                case "pointcloud":
                    await page.WaitForTimeoutAsync(500);
                    break;
            }
        }
        catch (TimeoutException)
        {
            // The page changed; capture it as it is.
        }
    }

    private static async Task WaitForServerAsync(string url)
    {
        using var http = new HttpClient();
        for (var i = 0; i < 120; i++)
        {
            try
            {
                if ((await http.GetAsync(url)).IsSuccessStatusCode)
                {
                    return;
                }
            }
            catch (HttpRequestException)
            {
            }

            await Task.Delay(500);
        }

        throw new TimeoutException($"Gallery did not start at {url}.");
    }

    internal static string FindRepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Directory.Build.props")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName ?? throw new DirectoryNotFoundException("Repository root not found.");
    }
}
