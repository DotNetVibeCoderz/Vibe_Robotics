using ROS2.Net.Rosidl;
using Shouldly;

namespace ROS2.Net.Tests.Core;

public class RosidlTests
{
    [Fact]
    public void Parses_fields_constants_defaults_and_bounds()
    {
        var m = RosidlParser.ParseMessage("demo", "Thing", """
            # A demo message.

            uint8 MODE_A=1
            string GREETING="hi # not a comment"
            float64 gain 1.5   # default with comment
            int32[3] triple [1, 2, 3]
            string<=8 name
            geometry_msgs/Point[<=4] points
            Header header
            """);

        m.Comment.ShouldBe("A demo message.");
        m.Constants.Count.ShouldBe(2);
        m.Constants[1].Value.ShouldBe("hi # not a comment");
        m.Fields.Select(f => f.Name).ShouldBe(new[] { "gain", "triple", "name", "points", "header" });
        m.Fields[0].DefaultValue.ShouldBe("1.5");
        m.Fields[0].Comment.ShouldBe("default with comment");
        m.Fields[1].Type.Array.ShouldBe(ArrayKind.Fixed);
        m.Fields[1].Type.ArraySize.ShouldBe(3);
        m.Fields[2].Type.StringBound.ShouldBe(8);
        m.Fields[3].Type.Array.ShouldBe(ArrayKind.Bounded);
        m.Fields[3].Type.ComplexRosName.ShouldBe("geometry_msgs/Point");
        m.Fields[4].Type.ComplexRosName.ShouldBe("std_msgs/Header");
    }

    [Fact]
    public void Parses_services_and_actions()
    {
        var s = RosidlParser.ParseService("std_srvs", "SetBool", "bool data\n---\nbool success\nstring message\n");
        s.Request.FullName.ShouldBe("std_srvs/srv/SetBool_Request");
        s.Response.Fields.Count.ShouldBe(2);

        var a = RosidlParser.ParseAction("example_interfaces", "Fibonacci", "int32 order\n---\nint32[] sequence\n---\nint32[] sequence\n");
        a.Goal.DdsName.ShouldBe("example_interfaces::action::dds_::Fibonacci_Goal_");
        a.Feedback.Fields[0].Type.IsSequence.ShouldBeTrue();
    }

    [Theory]
    [InlineData("int32 BadName")]
    [InlineData("float32[0] x")]
    [InlineData("int32[] X=3")]
    [InlineData("notatype")]
    public void Rejects_invalid_definitions(string text)
    {
        Should.Throw<RosidlParseException>(() => RosidlParser.ParseMessage("p", "M", text));
    }

    [Fact]
    public void Infers_package_from_path()
    {
        RosidlParser.TryParsePath("interfaces/std_msgs/msg/String.msg", out var pkg, out var kind, out var name).ShouldBeTrue();
        (pkg, kind, name).ShouldBe(("std_msgs", InterfaceKind.Message, "String"));
        RosidlParser.TryParsePath("foo/String.msg", out _, out _, out _).ShouldBeFalse();
    }

    [Fact]
    public void Emitter_renames_members_that_would_clash()
    {
        var m = RosidlParser.ParseMessage("sensor_msgs", "Range", "float32 range\nfloat32 type\n");
        var code = CSharpEmitter.EmitMessage(m);
        code.ShouldContain("public float RangeValue");
        code.ShouldContain("public float Type");
    }
}
