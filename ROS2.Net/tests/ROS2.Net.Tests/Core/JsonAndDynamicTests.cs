using ROS2.Net.Msgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Serialization;
using Shouldly;

namespace ROS2.Net.Tests.Core;

public class JsonAndDynamicTests
{
    [Fact]
    public void Json_uses_ros_field_names_and_base64_for_bytes()
    {
        var img = new Image { Width = 2, Height = 1, Encoding = "mono8", Step = 2, Data = [1, 2] };
        var json = RosJson.ToJsonString(img);

        json.ShouldContain("\"encoding\":\"mono8\"");
        json.ShouldContain("\"data\":\"AQI=\"");
        json.ShouldContain("\"is_bigendian\":0");

        var back = RosJson.FromJson<Image>(json);
        back.Data.ShouldBe(new byte[] { 1, 2 });
        back.Width.ShouldBe(2u);
    }

    [Fact]
    public void Json_accepts_number_arrays_for_bytes_and_strings_for_non_finite_floats()
    {
        var scan = RosJson.FromJson<LaserScan>("""{"ranges":[1.5,"Infinity","NaN"],"range_max":"Infinity"}""");
        scan.Ranges[0].ShouldBe(1.5f);
        float.IsPositiveInfinity(scan.Ranges[1]).ShouldBeTrue();
        float.IsNaN(scan.Ranges[2]).ShouldBeTrue();
        float.IsPositiveInfinity(scan.RangeMax).ShouldBeTrue();

        RosJson.FromJson<Image>("""{"data":[3,4]}""").Data.ShouldBe(new byte[] { 3, 4 });
    }

    [Fact]
    public void Registry_resolves_by_every_name_form()
    {
        MessageExtensions.EnsureRegistered();
        RosTypeRegistry.TryGetMessage("sensor_msgs/msg/Imu", out var a).ShouldBeTrue();
        RosTypeRegistry.TryGetMessage("sensor_msgs/Imu", out var b).ShouldBeTrue();
        RosTypeRegistry.TryGetMessage("sensor_msgs::msg::dds_::Imu_", out var c).ShouldBeTrue();
        a.ShouldBeSameAs(b);
        b.ShouldBeSameAs(c);
        RosTypeRegistry.TryGetService("std_srvs/srv/SetBool", out _).ShouldBeTrue();
        RosTypeRegistry.TryGetAction("example_interfaces/action/Fibonacci", out _).ShouldBeTrue();
    }

    [Fact]
    public void Full_definition_includes_transitive_dependencies()
    {
        MessageExtensions.EnsureRegistered();
        var def = RosTypeRegistry.GetFullDefinition("sensor_msgs/msg/Imu");
        def.ShouldContain("MSG: std_msgs/Header");
        def.ShouldContain("MSG: builtin_interfaces/Time");
        def.ShouldContain("MSG: geometry_msgs/Quaternion");
    }

    [Fact]
    public void Dynamic_type_decodes_cdr_like_the_generated_type()
    {
        MessageExtensions.EnsureRegistered();
        var imu = new Imu();
        imu.Header.FrameId = "base";
        imu.Orientation.W = 1;
        imu.AngularVelocity.Z = 0.5;
        var cdr = CdrWriter.Serialize(imu);

        DynamicMessageType.TryFromRegistry("sensor_msgs/msg/Imu", out var dyn).ShouldBeTrue();
        var json = dyn!.ToJson(cdr);

        json.ShouldBe(RosJson.ToJsonString(imu));
    }

    [Fact]
    public void Dynamic_type_encodes_json_to_the_same_cdr()
    {
        MessageExtensions.EnsureRegistered();
        DynamicMessageType.TryFromRegistry("geometry_msgs/msg/Twist", out var dyn).ShouldBeTrue();
        var cdr = dyn!.FromJson("""{"linear":{"x":0.5},"angular":{"z":-1.0}}""");
        cdr.ShouldBe(CdrWriter.Serialize(MessageExtensions.Twist2D(0.5, -1.0)));
    }

    [Fact]
    public void Quaternion_helpers_round_trip_yaw()
    {
        var q = MessageExtensions.QuaternionFromYaw(1.2);
        q.Yaw().ShouldBe(1.2, 1e-12);
        var (r, p, y) = MessageExtensions.QuaternionFromRpy(0.1, -0.2, 0.3).ToRpy();
        r.ShouldBe(0.1, 1e-12);
        p.ShouldBe(-0.2, 1e-12);
        y.ShouldBe(0.3, 1e-12);
    }
}
