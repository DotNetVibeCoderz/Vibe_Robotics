using System.Text.Json;
using System.Text.Json.Nodes;
using System.Xml;
using System.Xml.Linq;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.CSharp;
using Microsoft.CodeAnalysis.Formatting;

namespace ROS2.Net.AppGen;

/// <summary>
/// Formats code by file type: C# with Roslyn's formatter, JSON with System.Text.Json, XML-family
/// files (csproj, props, axaml, xaml, config, resx) with <see cref="XDocument"/>. Razor files only get
/// whitespace normalisation, since no Razor formatter ships with the SDK.
/// </summary>
public sealed class CodeFormatter : ICodeFormatter
{
    private static readonly string[] s_xml = [".xml", ".csproj", ".props", ".targets", ".axaml", ".xaml", ".config", ".resx", ".slnx", ".nuspec", ".urdf"];
    private readonly Lazy<AdhocWorkspace> _workspace = new(() => new AdhocWorkspace());

    public bool CanFormat(string fileName)
    {
        var ext = Path.GetExtension(fileName).ToLowerInvariant();
        return ext is ".cs" or ".json" or ".razor" or ".csx" || s_xml.Contains(ext);
    }

    public string Format(string fileName, string text)
    {
        var ext = Path.GetExtension(fileName).ToLowerInvariant();
        var newline = text.Contains("\r\n", StringComparison.Ordinal) ? "\r\n" : "\n";
        return ext switch
        {
            ".cs" or ".csx" => FormatCSharp(text, newline),
            ".json" => FormatJson(text),
            ".razor" => NormalizeWhitespace(text, newline),
            _ when s_xml.Contains(ext) => FormatXml(text),
            _ => text,
        };
    }

    private string FormatCSharp(string text, string newline)
    {
        var tree = CSharpSyntaxTree.ParseText(text, new CSharpParseOptions(LanguageVersion.Preview));
        var options = _workspace.Value.Options
            .WithChangedOption(FormattingOptions.NewLine, LanguageNames.CSharp, newline)
            .WithChangedOption(FormattingOptions.UseTabs, LanguageNames.CSharp, false)
            .WithChangedOption(FormattingOptions.IndentationSize, LanguageNames.CSharp, 4);
        var formatted = Formatter.Format(tree.GetRoot(), _workspace.Value, options);
        return formatted.ToFullString();
    }

    private static string FormatJson(string text)
    {
        var node = JsonNode.Parse(text, documentOptions: new JsonDocumentOptions { CommentHandling = JsonCommentHandling.Skip, AllowTrailingCommas = true });
        return node?.ToJsonString(new JsonSerializerOptions { WriteIndented = true }) ?? text;
    }

    private static string FormatXml(string text)
    {
        var doc = XDocument.Parse(text, LoadOptions.PreserveWhitespace);
        foreach (var t in doc.DescendantNodes().OfType<XText>().Where(t => string.IsNullOrWhiteSpace(t.Value)).ToList())
        {
            t.Remove();
        }

        var settings = new XmlWriterSettings
        {
            Indent = true,
            IndentChars = "  ",
            OmitXmlDeclaration = doc.Declaration is null,
            NewLineOnAttributes = false,
        };
        using var sw = new Utf8StringWriter();
        using (var writer = XmlWriter.Create(sw, settings))
        {
            doc.Save(writer);
        }

        return sw.ToString() + Environment.NewLine;
    }

    private static string NormalizeWhitespace(string text, string newline)
    {
        var lines = text.Replace("\r\n", "\n").Split('\n').Select(l => l.TrimEnd().Replace("\t", "    "));
        return string.Join(newline, lines).TrimEnd() + newline;
    }

    private sealed class Utf8StringWriter : StringWriter
    {
        public override System.Text.Encoding Encoding => System.Text.Encoding.UTF8;
    }
}
