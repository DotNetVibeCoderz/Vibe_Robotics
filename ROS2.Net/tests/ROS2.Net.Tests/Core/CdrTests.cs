using ROS2.Net.Msgs.GeometryMsgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Serialization;
using Shouldly;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Tests.Core;

public class CdrTests
{
    [Fact]
    public void String_matches_the_bytes_rmw_fastrtps_produces()
    {
        var bytes = CdrWriter.Serialize(new StdMsgs.String { Data = "hello" });

        // CDR_LE header, uint32 length 6 (incl. NUL), "hello\0".
        bytes.ShouldBe(new byte[] { 0x00, 0x01, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, (byte)'h', (byte)'e', (byte)'l', (byte)'l', (byte)'o', 0x00 });
    }

    [Fact]
    public void Doubles_are_aligned_to_eight_bytes_relative_to_the_payload()
    {
        // Header: int32 sec, uint32 nanosec, string "a" (len 2) → 4+4+4+2 = 14 bytes, then pad to 16 for the first double.
        var imu = new Imu();
        imu.Header.Stamp.Sec = 1;
        imu.Header.FrameId = "a";
        imu.Orientation.W = 1;
        var bytes = CdrWriter.Serialize(imu);

        BitConverter.ToInt32(bytes, 4).ShouldBe(1);
        BitConverter.ToUInt32(bytes, 12).ShouldBe(2u); // frame_id length
        bytes[16].ShouldBe((byte)'a');
        bytes[18].ShouldBe((byte)0); // padding
        BitConverter.ToDouble(bytes, 4 + 16 + 24).ShouldBe(1.0); // orientation.w after x,y,z
    }

    [Fact]
    public void Imu_round_trips()
    {
        var imu = new Imu();
        imu.Header.FrameId = "imu_link";
        imu.Header.Stamp.Sec = 42;
        imu.Header.Stamp.Nanosec = 7;
        imu.Orientation = new Quaternion { X = 0.1, Y = 0.2, Z = 0.3, W = 0.9 };
        imu.AngularVelocity.Z = 1.5;
        imu.LinearAccelerationCovariance[8] = 0.01;

        var copy = CdrReader.Deserialize<Imu>(CdrWriter.Serialize(imu));

        copy.Header.FrameId.ShouldBe("imu_link");
        copy.Header.Stamp.Sec.ShouldBe(42);
        copy.Header.Stamp.Nanosec.ShouldBe(7u);
        copy.Orientation.Z.ShouldBe(0.3);
        copy.AngularVelocity.Z.ShouldBe(1.5);
        copy.LinearAccelerationCovariance[8].ShouldBe(0.01);
    }

    [Fact]
    public void Nested_sequences_round_trip()
    {
        var cloud = new PointCloud2
        {
            Height = 1,
            Width = 2,
            PointStep = 12,
            RowStep = 24,
            IsDense = true,
            Fields =
            [
                new PointField { Name = "x", Offset = 0, Datatype = PointField.FLOAT32, Count = 1 },
                new PointField { Name = "y", Offset = 4, Datatype = PointField.FLOAT32, Count = 1 },
            ],
            Data = Enumerable.Range(0, 24).Select(i => (byte)i).ToArray(),
        };

        var copy = CdrReader.Deserialize<PointCloud2>(CdrWriter.Serialize(cloud));

        copy.Fields.Length.ShouldBe(2);
        copy.Fields[1].Name.ShouldBe("y");
        copy.Fields[1].Offset.ShouldBe(4u);
        copy.Data.ShouldBe(cloud.Data);
        copy.IsDense.ShouldBeTrue();
    }

    [Fact]
    public void String_arrays_and_empty_messages_round_trip()
    {
        var js = new JointState { Name = ["a", "bb", ""], Position = [1, 2, 3] };
        var copy = CdrReader.Deserialize<JointState>(CdrWriter.Serialize(js));
        copy.Name.ShouldBe(new[] { "a", "bb", "" });
        copy.Position.ShouldBe(new[] { 1.0, 2.0, 3.0 });

        // Empty structures carry one byte.
        CdrWriter.Serialize(new StdMsgs.Empty()).Length.ShouldBe(5);
        CdrReader.Deserialize<StdMsgs.Empty>(new byte[] { 0, 1, 0, 0, 0 }).ShouldNotBeNull();
    }

    [Fact]
    public void Fixed_arrays_must_have_the_declared_length()
    {
        var imu = new Imu { OrientationCovariance = new double[3] };
        Should.Throw<CdrException>(() => CdrWriter.Serialize(imu));
    }

    [Fact]
    public void Big_endian_payloads_are_decoded()
    {
        // CDR_BE header, then std_msgs/Int32 = 0x01020304.
        var msg = CdrReader.Deserialize<StdMsgs.Int32>(new byte[] { 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04 });
        msg.Data.ShouldBe(0x01020304);
    }

    [Fact]
    public void Truncated_data_throws_instead_of_reading_garbage()
    {
        var bytes = CdrWriter.Serialize(new StdMsgs.String { Data = "hello world" });
        Should.Throw<CdrException>(() => CdrReader.Deserialize<StdMsgs.String>(bytes.AsSpan(0, 8)));
    }

    [Fact]
    public void Deserialize_reuses_arrays_of_the_same_length()
    {
        var scan = new LaserScan { Ranges = new float[360] };
        var bytes = CdrWriter.Serialize(scan);
        var target = new LaserScan { Ranges = new float[360] };
        var before = target.Ranges;

        CdrReader.Deserialize(bytes, target);

        target.Ranges.ShouldBeSameAs(before);
    }

    [Fact]
    public void Clone_is_deep()
    {
        var path = new Msgs.NavMsgs.Path { Poses = [new PoseStamped()] };
        path.Poses[0].Pose.Position.X = 1;
        var clone = path.Clone();
        clone.Poses[0].Pose.Position.X = 2;
        path.Poses[0].Pose.Position.X.ShouldBe(1);
    }
}
