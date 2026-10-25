using System;
using System.Collections.Generic;
using System.Text;

namespace ROS2.Net.Rosidl;

/// <summary>Naming rules that map ROS 2 interface names onto C# identifiers.</summary>
public static class Naming
{
    /// <summary>The namespace every generated package lives under.</summary>
    public const string DefaultRootNamespace = "ROS2.Net.Msgs";

    private static readonly HashSet<string> s_reservedMembers = new(StringComparer.Ordinal)
    {
        "TypeName", "DdsTypeName", "Definition", "Dependencies", "RosTypeName", "ServiceTypeName",
        "ActionTypeName", "Serialize", "Deserialize", "WriteJson", "ReadJson", "Clone", "CloneMessage",
        "ToString", "Equals", "GetHashCode", "GetType", "MemberwiseClone", "Finalize",
    };

    /// <summary><c>geometry_msgs</c> → <c>GeometryMsgs</c>.</summary>
    public static string PackageToPascal(string package) => SnakeToPascal(package);

    /// <summary>The C# namespace of a ROS package.</summary>
    public static string PackageNamespace(string package, string rootNamespace = DefaultRootNamespace) =>
        rootNamespace + "." + PackageToPascal(package);

    /// <summary>
    /// The C# class name of a structure: messages keep their name; service and action parts drop
    /// the underscore (<c>SetBool_Request</c> → <c>SetBoolRequest</c>).
    /// </summary>
    public static string ClassName(MessageDef def) => def.Name.Replace("_", string.Empty);

    /// <summary>The C# property name of a field, avoiding clashes with the class and generated members.</summary>
    public static string PropertyName(FieldDef field, MessageDef owner)
    {
        var name = SnakeToPascal(field.Name);
        var className = ClassName(owner);
        if (name == className || s_reservedMembers.Contains(name) || owner.Constants.Exists(c => c.Name == name))
        {
            name += "Value";
        }

        return name;
    }

    /// <summary>Converts <c>linear_acceleration</c> to <c>LinearAcceleration</c>.</summary>
    public static string SnakeToPascal(string snake)
    {
        var sb = new StringBuilder(snake.Length);
        var upper = true;
        foreach (var c in snake)
        {
            if (c == '_')
            {
                upper = true;
                continue;
            }

            sb.Append(upper ? char.ToUpperInvariant(c) : c);
            upper = false;
        }

        return sb.ToString();
    }

    /// <summary>The fully qualified C# type of a complex reference.</summary>
    public static string ComplexTypeReference(string package, string name, string rootNamespace = DefaultRootNamespace) =>
        "global::" + PackageNamespace(package, rootNamespace) + "." + name;
}
