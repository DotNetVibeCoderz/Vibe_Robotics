using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace ROS2.Net.Rosidl;

/// <summary>
/// Parses ROS 2 interface files (<c>.msg</c>, <c>.srv</c>, <c>.action</c>) using the same rules as
/// <c>rosidl_adapter</c>: one field or constant per line, <c>#</c> comments, <c>---</c> separators.
/// </summary>
public static class RosidlParser
{
    private const string Separator = "---";

    /// <summary>Parses a <c>.msg</c> file.</summary>
    public static MessageDef ParseMessage(string package, string name, string text, string? fileName = null)
    {
        var file = fileName ?? $"{package}/msg/{name}.msg";
        var sections = SplitSections(text, file);
        if (sections.Count != 1)
        {
            throw new RosidlParseException(file, sections[1].StartLine, "a .msg file must not contain '---'");
        }

        return ParseStructure(package, name, InterfaceKind.Message, sections[0], file);
    }

    /// <summary>Parses a <c>.srv</c> file into request and response structures.</summary>
    public static ServiceDef ParseService(string package, string name, string text, string? fileName = null)
    {
        var file = fileName ?? $"{package}/srv/{name}.srv";
        var sections = SplitSections(text, file);
        if (sections.Count != 2)
        {
            throw new RosidlParseException(file, 1, $"a .srv file needs exactly one '---' separator, found {sections.Count - 1}");
        }

        var request = ParseStructure(package, name + "_Request", InterfaceKind.Service, sections[0], file);
        var response = ParseStructure(package, name + "_Response", InterfaceKind.Service, sections[1], file);
        return new ServiceDef(package, name, request, response, text);
    }

    /// <summary>Parses a <c>.action</c> file into goal, result and feedback structures.</summary>
    public static ActionDef ParseAction(string package, string name, string text, string? fileName = null)
    {
        var file = fileName ?? $"{package}/action/{name}.action";
        var sections = SplitSections(text, file);
        if (sections.Count != 3)
        {
            throw new RosidlParseException(file, 1, $"an .action file needs exactly two '---' separators, found {sections.Count - 1}");
        }

        var goal = ParseStructure(package, name + "_Goal", InterfaceKind.Action, sections[0], file);
        var result = ParseStructure(package, name + "_Result", InterfaceKind.Action, sections[1], file);
        var feedback = ParseStructure(package, name + "_Feedback", InterfaceKind.Action, sections[2], file);
        return new ActionDef(package, name, goal, result, feedback, text);
    }

    /// <summary>
    /// Infers package, kind and name from a path such as <c>interfaces/std_msgs/msg/String.msg</c>.
    /// Returns false when the path does not follow the <c>&lt;package&gt;/&lt;msg|srv|action&gt;/&lt;Name&gt;.&lt;ext&gt;</c> layout.
    /// </summary>
    public static bool TryParsePath(string path, out string package, out InterfaceKind kind, out string name)
    {
        package = string.Empty;
        name = string.Empty;
        kind = InterfaceKind.Message;

        var parts = path.Replace('\\', '/').Split('/');
        if (parts.Length < 3)
        {
            return false;
        }

        var fileName = parts[parts.Length - 1];
        var dot = fileName.LastIndexOf('.');
        if (dot <= 0)
        {
            return false;
        }

        var ext = fileName.Substring(dot + 1);
        name = fileName.Substring(0, dot);
        var dir = parts[parts.Length - 2];
        package = parts[parts.Length - 3];

        switch (ext)
        {
            case "msg" when dir == "msg":
                kind = InterfaceKind.Message;
                return true;
            case "srv" when dir == "srv":
                kind = InterfaceKind.Service;
                return true;
            case "action" when dir == "action":
                kind = InterfaceKind.Action;
                return true;
            default:
                return false;
        }
    }

    /// <summary>Parses a type expression such as <c>float64[9]</c>, <c>string&lt;=10[]</c> or <c>geometry_msgs/Point[&lt;=4]</c>.</summary>
    public static FieldType ParseType(string text, string currentPackage, string file = "<type>", int line = 0)
    {
        var type = new FieldType();
        var baseText = text;

        var bracket = text.IndexOf('[');
        if (bracket >= 0)
        {
            if (!text.EndsWith("]", StringComparison.Ordinal))
            {
                throw new RosidlParseException(file, line, $"malformed array type '{text}'");
            }

            baseText = text.Substring(0, bracket);
            var inner = text.Substring(bracket + 1, text.Length - bracket - 2).Trim();
            if (inner.Length == 0)
            {
                type.Array = ArrayKind.Unbounded;
            }
            else if (inner.StartsWith("<=", StringComparison.Ordinal))
            {
                type.Array = ArrayKind.Bounded;
                type.ArraySize = ParsePositiveInt(inner.Substring(2), file, line);
            }
            else
            {
                type.Array = ArrayKind.Fixed;
                type.ArraySize = ParsePositiveInt(inner, file, line);
            }
        }

        var bound = baseText.IndexOf("<=", StringComparison.Ordinal);
        if (bound >= 0)
        {
            var prim = baseText.Substring(0, bound);
            if (prim != "string" && prim != "wstring")
            {
                throw new RosidlParseException(file, line, $"only strings may be bounded, got '{baseText}'");
            }

            type.StringBound = ParsePositiveInt(baseText.Substring(bound + 2), file, line);
            baseText = prim;
        }

        if (PrimitiveNames.TryParse(baseText, out var kind))
        {
            type.Primitive = kind;
            return type;
        }

        type.Primitive = PrimitiveKind.None;
        var segments = baseText.Split('/');
        switch (segments.Length)
        {
            case 1:
                // ROS 2 keeps the ROS 1 convenience: a bare 'Header' means std_msgs/Header.
                type.Package = baseText == "Header" ? "std_msgs" : currentPackage;
                type.Name = baseText;
                break;
            case 2:
                type.Package = segments[0];
                type.Name = segments[1];
                break;
            case 3 when segments[1] == "msg":
                type.Package = segments[0];
                type.Name = segments[2];
                break;
            default:
                throw new RosidlParseException(file, line, $"invalid type '{baseText}'");
        }

        if (!IsValidTypeName(type.Name!))
        {
            throw new RosidlParseException(file, line, $"invalid type name '{type.Name}'");
        }

        return type;
    }

    private static MessageDef ParseStructure(string package, string name, InterfaceKind kind, Section section, string file)
    {
        var def = new MessageDef(package, name, kind, section.Text);
        var pendingComment = new StringBuilder();
        var sawMember = false;
        var fileComment = new StringBuilder();

        for (var i = 0; i < section.Lines.Count; i++)
        {
            var lineNo = section.StartLine + i;
            var raw = section.Lines[i];
            SplitComment(raw, out var content, out var comment);
            content = content.Trim();

            if (content.Length == 0)
            {
                if (comment != null)
                {
                    (sawMember ? pendingComment : fileComment).AppendLine(comment);
                }
                else if (!sawMember && fileComment.Length > 0 && def.Comment == null)
                {
                    // A blank line ends the file-level comment block.
                    def.Comment = fileComment.ToString().Trim();
                }

                continue;
            }

            if (!sawMember && def.Comment == null && fileComment.Length > 0)
            {
                // The comment block touches the first member: it documents that member, not the file.
                pendingComment.Append(fileComment);
                fileComment.Clear();
            }

            sawMember = true;
            var typeEnd = IndexOfWhitespace(content);
            if (typeEnd < 0)
            {
                throw new RosidlParseException(file, lineNo, $"expected '<type> <name>', got '{content}'");
            }

            var typeText = content.Substring(0, typeEnd);
            var rest = content.Substring(typeEnd).Trim();
            var type = ParseType(typeText, package, file, lineNo);

            var eq = rest.IndexOf('=');
            if (eq > 0 && IsConstantName(rest.Substring(0, eq).Trim()))
            {
                if (type.IsArray || type.IsComplex)
                {
                    throw new RosidlParseException(file, lineNo, "constants must be primitive scalars");
                }

                var constName = rest.Substring(0, eq).Trim();
                var value = rest.Substring(eq + 1).Trim();
                if (type.IsString)
                {
                    value = Unquote(value);
                }

                def.Constants.Add(new ConstantDef(type, constName, value));
            }
            else
            {
                var nameEnd = IndexOfWhitespace(rest);
                var fieldName = nameEnd < 0 ? rest : rest.Substring(0, nameEnd);
                if (!IsValidFieldName(fieldName))
                {
                    throw new RosidlParseException(file, lineNo, $"invalid field name '{fieldName}' (must be lower snake_case)");
                }

                var field = new FieldDef(type, fieldName);
                if (nameEnd >= 0)
                {
                    field.DefaultValue = rest.Substring(nameEnd).Trim();
                }

                if (comment != null)
                {
                    pendingComment.AppendLine(comment);
                }

                if (pendingComment.Length > 0)
                {
                    field.Comment = pendingComment.ToString().Trim();
                    pendingComment.Clear();
                }

                def.Fields.Add(field);
            }

            pendingComment.Clear();
        }

        if (def.Comment == null && fileComment.Length > 0)
        {
            def.Comment = fileComment.ToString().Trim();
        }

        return def;
    }

    private sealed class Section
    {
        public int StartLine { get; set; }

        public List<string> Lines { get; } = new();

        public string Text { get; set; } = string.Empty;
    }

    private static List<Section> SplitSections(string text, string file)
    {
        var sections = new List<Section>();
        var current = new Section { StartLine = 1 };
        var sb = new StringBuilder();
        var lines = text.Replace("\r\n", "\n").Split('\n');
        for (var i = 0; i < lines.Length; i++)
        {
            if (lines[i].Trim() == Separator)
            {
                current.Text = sb.ToString();
                sections.Add(current);
                current = new Section { StartLine = i + 2 };
                sb.Clear();
                continue;
            }

            current.Lines.Add(lines[i]);
            sb.Append(lines[i]).Append('\n');
        }

        current.Text = sb.ToString();
        sections.Add(current);
        _ = file;
        return sections;
    }

    private static void SplitComment(string line, out string content, out string? comment)
    {
        var inQuote = '\0';
        for (var i = 0; i < line.Length; i++)
        {
            var c = line[i];
            if (inQuote != '\0')
            {
                if (c == '\\')
                {
                    i++;
                }
                else if (c == inQuote)
                {
                    inQuote = '\0';
                }
            }
            else if (c == '"' || c == '\'')
            {
                inQuote = c;
            }
            else if (c == '#')
            {
                content = line.Substring(0, i);
                comment = line.Substring(i + 1).Trim();
                return;
            }
        }

        content = line;
        comment = null;
    }

    private static int IndexOfWhitespace(string s)
    {
        for (var i = 0; i < s.Length; i++)
        {
            if (char.IsWhiteSpace(s[i]))
            {
                return i;
            }
        }

        return -1;
    }

    private static int ParsePositiveInt(string s, string file, int line)
    {
        if (!int.TryParse(s.Trim(), NumberStyles.None, CultureInfo.InvariantCulture, out var v) || v <= 0)
        {
            throw new RosidlParseException(file, line, $"expected a positive integer, got '{s}'");
        }

        return v;
    }

    private static bool IsConstantName(string s)
    {
        if (s.Length == 0 || !char.IsUpper(s[0]))
        {
            return false;
        }

        foreach (var c in s)
        {
            if (!(char.IsUpper(c) || char.IsDigit(c) || c == '_'))
            {
                return false;
            }
        }

        return true;
    }

    private static bool IsValidFieldName(string s)
    {
        if (s.Length == 0 || !(s[0] >= 'a' && s[0] <= 'z'))
        {
            return false;
        }

        foreach (var c in s)
        {
            if (!((c >= 'a' && c <= 'z') || char.IsDigit(c) || c == '_'))
            {
                return false;
            }
        }

        return !s.EndsWith("_", StringComparison.Ordinal) && s.IndexOf("__", StringComparison.Ordinal) < 0;
    }

    private static bool IsValidTypeName(string s)
    {
        if (s.Length == 0 || !char.IsUpper(s[0]))
        {
            return false;
        }

        foreach (var c in s)
        {
            if (!char.IsLetterOrDigit(c))
            {
                return false;
            }
        }

        return true;
    }

    internal static string Unquote(string value)
    {
        if (value.Length >= 2 && (value[0] == '"' || value[0] == '\'') && value[value.Length - 1] == value[0])
        {
            var inner = value.Substring(1, value.Length - 2);
            var sb = new StringBuilder(inner.Length);
            for (var i = 0; i < inner.Length; i++)
            {
                if (inner[i] == '\\' && i + 1 < inner.Length)
                {
                    i++;
                    sb.Append(inner[i] switch
                    {
                        'n' => '\n',
                        't' => '\t',
                        'r' => '\r',
                        _ => inner[i],
                    });
                }
                else
                {
                    sb.Append(inner[i]);
                }
            }

            return sb.ToString();
        }

        return value;
    }
}
