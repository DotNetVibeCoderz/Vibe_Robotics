using System.ComponentModel;
using System.Net;
using System.Net.Http.Headers;
using System.Net.Http.Json;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Text.RegularExpressions;
using HtmlAgilityPack;
using Microsoft.SemanticKernel;

namespace ROS2.Net.AppGen.Plugins;

/// <summary>Internet access for the assistant: Tavily web search and readable-text page scraping.</summary>
public sealed partial class WebPlugin(ToolContext ctx)
{
    [KernelFunction("search_internet")]
    [Description("Searches the web with Tavily and returns an answer summary plus the top results (title, url, snippet). Use for library docs, error messages and facts outside ROS2.Net.")]
    public async Task<string> SearchInternetAsync(string query, [Description("Number of results (1-10)")] int maxResults = 5, CancellationToken cancellationToken = default)
    {
        var key = AppConfigSettingsStore.ResolveSecret(ctx.Settings().TavilyApiKey);
        if (string.IsNullOrWhiteSpace(key))
        {
            return "Error: internet search is not configured (set the Tavily API key in Settings).";
        }

        using var request = new HttpRequestMessage(HttpMethod.Post, "https://api.tavily.com/search")
        {
            Content = JsonContent.Create(new
            {
                api_key = key,
                query,
                max_results = Math.Clamp(maxResults, 1, 10),
                search_depth = "basic",
                include_answer = true,
            }),
        };
        request.Headers.Authorization = new AuthenticationHeaderValue("Bearer", key);
        try
        {
            using var response = await ctx.Http.SendAsync(request, cancellationToken).ConfigureAwait(false);
            var body = await response.Content.ReadAsStringAsync(cancellationToken).ConfigureAwait(false);
            if (!response.IsSuccessStatusCode)
            {
                return $"Error: Tavily returned {(int)response.StatusCode}: {Shorten(body, 300)}";
            }

            using var doc = JsonDocument.Parse(body);
            var sb = new StringBuilder();
            if (doc.RootElement.TryGetProperty("answer", out var answer) && answer.ValueKind == JsonValueKind.String)
            {
                sb.AppendLine("Answer: " + answer.GetString());
            }

            if (doc.RootElement.TryGetProperty("results", out var results))
            {
                var i = 0;
                foreach (var r in results.EnumerateArray())
                {
                    sb.AppendLine($"{++i}. {Str(r, "title")}\n   {Str(r, "url")}\n   {Shorten(Str(r, "content"), 500)}");
                }
            }

            ctx.Log($"Jack searched the web: {query}");
            return sb.Length == 0 ? "No results." : ctx.Truncate(sb.ToString());
        }
        catch (Exception ex) when (ex is HttpRequestException or TaskCanceledException or JsonException)
        {
            return "Error: search failed: " + ex.Message;
        }
    }

    [KernelFunction("scrape_webpage")]
    [Description("Downloads a web page (http/https) and returns its readable text (headings, paragraphs, lists, code), without scripts and navigation.")]
    public async Task<string> ScrapeWebPageAsync(string url, [Description("Maximum characters to return")] int maxChars = 12000, CancellationToken cancellationToken = default)
    {
        if (!Uri.TryCreate(url, UriKind.Absolute, out var uri) || (uri.Scheme != Uri.UriSchemeHttp && uri.Scheme != Uri.UriSchemeHttps))
        {
            return "Error: only absolute http(s) URLs are allowed.";
        }

        if (await IsPrivateHostAsync(uri, cancellationToken).ConfigureAwait(false))
        {
            return "Error: local and private network addresses are not allowed.";
        }

        try
        {
            using var response = await ctx.Http.GetAsync(uri, HttpCompletionOption.ResponseHeadersRead, cancellationToken).ConfigureAwait(false);
            if (!response.IsSuccessStatusCode)
            {
                return $"Error: {(int)response.StatusCode} {response.ReasonPhrase}";
            }

            var media = response.Content.Headers.ContentType?.MediaType ?? string.Empty;
            var raw = await response.Content.ReadAsStringAsync(cancellationToken).ConfigureAwait(false);
            if (raw.Length > 3_000_000)
            {
                raw = raw[..3_000_000];
            }

            var text = media.Contains("html", StringComparison.OrdinalIgnoreCase) || raw.TrimStart().StartsWith('<') ? HtmlToText(raw) : raw;
            ctx.Log($"Jack read {uri}");
            return ctx.Truncate($"{uri}\n\n{text}", Math.Clamp(maxChars, 500, 60_000));
        }
        catch (Exception ex) when (ex is HttpRequestException or TaskCanceledException)
        {
            return "Error: download failed: " + ex.Message;
        }
    }

    /// <summary>Extracts readable text from HTML: drops scripts, styles and page chrome, keeps structure.</summary>
    public static string HtmlToText(string html)
    {
        var doc = new HtmlDocument();
        doc.LoadHtml(html);
        foreach (var node in doc.DocumentNode.SelectNodes("//script|//style|//noscript|//svg|//nav|//footer|//header|//form|//iframe|//aside") ?? Enumerable.Empty<HtmlNode>())
        {
            node.Remove();
        }

        var title = doc.DocumentNode.SelectSingleNode("//title")?.InnerText.Trim();
        var root = doc.DocumentNode.SelectSingleNode("//main") ?? doc.DocumentNode.SelectSingleNode("//article") ?? doc.DocumentNode.SelectSingleNode("//body") ?? doc.DocumentNode;
        var sb = new StringBuilder();
        if (!string.IsNullOrEmpty(title))
        {
            sb.AppendLine("# " + WebUtility.HtmlDecode(title)).AppendLine();
        }

        Render(root, sb);
        return MultiBlank().Replace(sb.ToString(), "\n\n").Trim();
    }

    private static void Render(HtmlNode node, StringBuilder sb)
    {
        foreach (var child in node.ChildNodes)
        {
            switch (child.Name)
            {
                case "#text":
                    var t = Spaces().Replace(WebUtility.HtmlDecode(child.InnerText), " ");
                    if (t.Trim().Length > 0)
                    {
                        sb.Append(t);
                    }

                    break;
                case "h1" or "h2" or "h3" or "h4" or "h5" or "h6":
                    sb.AppendLine().AppendLine().Append(new string('#', child.Name[1] - '0')).Append(' ').AppendLine(Clean(child.InnerText)).AppendLine();
                    break;
                case "pre":
                    sb.AppendLine().AppendLine("```").AppendLine(WebUtility.HtmlDecode(child.InnerText).TrimEnd()).AppendLine("```");
                    break;
                case "li":
                    sb.AppendLine().Append("- ");
                    Render(child, sb);
                    break;
                case "p" or "div" or "section" or "tr" or "table" or "ul" or "ol" or "blockquote":
                    sb.AppendLine();
                    Render(child, sb);
                    sb.AppendLine();
                    break;
                case "br":
                    sb.AppendLine();
                    break;
                case "a":
                    var href = child.GetAttributeValue("href", string.Empty);
                    sb.Append(Clean(child.InnerText));
                    if (href.StartsWith("http", StringComparison.OrdinalIgnoreCase))
                    {
                        sb.Append(" (").Append(href).Append(')');
                    }

                    break;
                case "td" or "th":
                    Render(child, sb);
                    sb.Append(" | ");
                    break;
                default:
                    Render(child, sb);
                    break;
            }
        }
    }

    private static async Task<bool> IsPrivateHostAsync(Uri uri, CancellationToken ct)
    {
        if (uri.IsLoopback || uri.Host.Equals("localhost", StringComparison.OrdinalIgnoreCase))
        {
            return true;
        }

        try
        {
            var addresses = IPAddress.TryParse(uri.Host, out var ip) ? [ip] : await Dns.GetHostAddressesAsync(uri.Host, ct).ConfigureAwait(false);
            return addresses.Any(a => IPAddress.IsLoopback(a) || IsPrivate(a));
        }
        catch (SocketException)
        {
            return false; // Let the request fail with a normal error.
        }
    }

    private static bool IsPrivate(IPAddress a)
    {
        if (a.AddressFamily == AddressFamily.InterNetworkV6)
        {
            return a.IsIPv6LinkLocal || a.IsIPv6SiteLocal || a.IsIPv6UniqueLocal;
        }

        var b = a.GetAddressBytes();
        return b[0] == 10 || (b[0] == 172 && b[1] >= 16 && b[1] <= 31) || (b[0] == 192 && b[1] == 168) || (b[0] == 169 && b[1] == 254) || b[0] == 127 || b[0] == 0;
    }

    private static string Clean(string s) => Spaces().Replace(WebUtility.HtmlDecode(s), " ").Trim();

    private static string Str(JsonElement e, string name) => e.TryGetProperty(name, out var v) && v.ValueKind == JsonValueKind.String ? v.GetString() ?? string.Empty : string.Empty;

    private static string Shorten(string s, int n) => s.Length <= n ? s : s[..n] + "…";

    [GeneratedRegex(@"[ \t\r\n]+")]
    private static partial Regex Spaces();

    [GeneratedRegex(@"\n\s*\n\s*\n+")]
    private static partial Regex MultiBlank();
}

/// <summary>NuGet package discovery.</summary>
public sealed class NuGetPlugin(ToolContext ctx)
{
    [KernelFunction("search_nuget")]
    [Description("Searches nuget.org for packages; returns id, latest version, downloads and description.")]
    public async Task<string> SearchNuGetAsync(string query, int take = 8, CancellationToken cancellationToken = default)
    {
        try
        {
            var url = $"https://azuresearch-usnc.nuget.org/query?q={Uri.EscapeDataString(query)}&take={Math.Clamp(take, 1, 20)}&prerelease=false&semVerLevel=2.0.0";
            using var doc = JsonDocument.Parse(await ctx.Http.GetStringAsync(url, cancellationToken).ConfigureAwait(false));
            var sb = new StringBuilder();
            foreach (var p in doc.RootElement.GetProperty("data").EnumerateArray())
            {
                var desc = p.TryGetProperty("description", out var d) ? d.GetString() ?? string.Empty : string.Empty;
                sb.AppendLine($"{p.GetProperty("id").GetString()} {p.GetProperty("version").GetString()} ({p.GetProperty("totalDownloads").GetInt64():N0} downloads) — {(desc.Length > 160 ? desc[..160] + "…" : desc)}");
            }

            return sb.Length == 0 ? "No packages found." : sb.ToString();
        }
        catch (Exception ex) when (ex is HttpRequestException or TaskCanceledException or JsonException or KeyNotFoundException)
        {
            return "Error: NuGet search failed: " + ex.Message;
        }
    }
}
