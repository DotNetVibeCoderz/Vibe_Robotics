using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.IO;
using System.Linq;
using System.Text;
using Microsoft.CodeAnalysis;
using Microsoft.CodeAnalysis.Text;

namespace ROS2.Net.Rosidl.Generator;

/// <summary>
/// Turns ROS 2 interface files passed as <c>AdditionalFiles</c> into C# message, service and action types.
/// </summary>
[Generator(LanguageNames.CSharp)]
public sealed class RosInterfaceGenerator : IIncrementalGenerator
{
    private static readonly DiagnosticDescriptor s_parseError = new(
        id: "ROSIDL001",
        title: "Invalid ROS 2 interface definition",
        messageFormat: "{0}",
        category: "ROS2.Net.Rosidl",
        defaultSeverity: DiagnosticSeverity.Error,
        isEnabledByDefault: true);

    private static readonly DiagnosticDescriptor s_layoutWarning = new(
        id: "ROSIDL002",
        title: "Interface file outside a <package>/<msg|srv|action> folder",
        messageFormat: "'{0}' is not under <package>/msg, <package>/srv or <package>/action and was skipped",
        category: "ROS2.Net.Rosidl",
        defaultSeverity: DiagnosticSeverity.Warning,
        isEnabledByDefault: true);

    public void Initialize(IncrementalGeneratorInitializationContext context)
    {
        var files = context.AdditionalTextsProvider
            .Where(static f => f.Path.EndsWith(".msg", StringComparison.OrdinalIgnoreCase)
                            || f.Path.EndsWith(".srv", StringComparison.OrdinalIgnoreCase)
                            || f.Path.EndsWith(".action", StringComparison.OrdinalIgnoreCase))
            .Select(static (f, ct) => Parse(f.Path, f.GetText(ct)?.ToString() ?? string.Empty));

        context.RegisterSourceOutput(files, static (spc, parsed) =>
        {
            if (parsed.Diagnostic is { } d)
            {
                spc.ReportDiagnostic(d);
                return;
            }

            if (parsed.Source != null)
            {
                spc.AddSource(parsed.HintName!, SourceText.From(parsed.Source, Encoding.UTF8));
            }
        });

        var all = files.Collect().Combine(context.CompilationProvider.Select(static (c, _) => c.AssemblyName ?? "Assembly"));
        context.RegisterSourceOutput(all, static (spc, input) =>
        {
            var (items, assemblyName) = input;
            var ok = items.Where(i => i.Diagnostic == null && i.Source != null).ToList();
            if (ok.Count == 0)
            {
                return;
            }

            var source = CSharpEmitter.EmitRegistration(
                assemblyName,
                ok.Where(i => i.Message != null).Select(i => i.Message!),
                ok.Where(i => i.Service != null).Select(i => i.Service!),
                ok.Where(i => i.Action != null).Select(i => i.Action!));
            spc.AddSource("RosTypeRegistration.g.cs", SourceText.From(source, Encoding.UTF8));
        });
    }

    private static ParsedInterface Parse(string path, string text)
    {
        if (!RosidlParser.TryParsePath(path, out var package, out var kind, out var name))
        {
            return new ParsedInterface { Diagnostic = Diagnostic.Create(s_layoutWarning, Location.None, path) };
        }

        try
        {
            var hint = $"{package}.{MessageDef.KindDirectory(kind)}.{name}.g.cs";
            switch (kind)
            {
                case InterfaceKind.Message:
                {
                    var m = RosidlParser.ParseMessage(package, name, text, path);
                    return new ParsedInterface { Message = m, Source = CSharpEmitter.EmitMessage(m), HintName = hint };
                }

                case InterfaceKind.Service:
                {
                    var s = RosidlParser.ParseService(package, name, text, path);
                    return new ParsedInterface { Service = s, Source = CSharpEmitter.EmitService(s), HintName = hint };
                }

                default:
                {
                    var a = RosidlParser.ParseAction(package, name, text, path);
                    return new ParsedInterface { Action = a, Source = CSharpEmitter.EmitAction(a), HintName = hint };
                }
            }
        }
        catch (RosidlParseException ex)
        {
            var location = Location.Create(path, default, new LinePositionSpan(new LinePosition(Math.Max(0, ex.Line - 1), 0), new LinePosition(Math.Max(0, ex.Line - 1), 0)));
            return new ParsedInterface { Diagnostic = Diagnostic.Create(s_parseError, location, ex.Message) };
        }
        catch (Exception ex) when (ex is FormatException or OverflowException)
        {
            return new ParsedInterface { Diagnostic = Diagnostic.Create(s_parseError, Location.None, $"{Path.GetFileName(path)}: {ex.Message}") };
        }
    }

    private sealed class ParsedInterface : IEquatable<ParsedInterface>
    {
        public MessageDef? Message { get; set; }

        public ServiceDef? Service { get; set; }

        public ActionDef? Action { get; set; }

        public string? Source { get; set; }

        public string? HintName { get; set; }

        public Diagnostic? Diagnostic { get; set; }

        // Incremental caching compares outputs; generated source text is a complete fingerprint.
        public bool Equals(ParsedInterface? other) =>
            other != null && other.Source == Source && other.HintName == HintName && Equals(other.Diagnostic, Diagnostic);

        public override bool Equals(object? obj) => Equals(obj as ParsedInterface);

        public override int GetHashCode() => (Source ?? string.Empty).GetHashCode();
    }
}
