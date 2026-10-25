using System.Net;
using ROS2.Net.Rmw.Rtps.Discovery;
using ROS2.Net.Rmw.Rtps.Transport;
using ROS2.Net.Rmw.Rtps.Wire;
using ROS2.Net.Serialization;
using Shouldly;

namespace ROS2.Net.Rmw.Rtps.Tests;

public class WireTests
{
    private static readonly GuidPrefix s_prefix = GuidPrefix.Read([0x01, 0xa2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

    [Fact]
    public void Locator_encodes_as_in_the_spec()
    {
        var bytes = new byte[24];
        Locator.UdpV4(IPAddress.Loopback, 7410).Write(bytes);
        bytes.ShouldBe(new byte[] { 1, 0, 0, 0, 0xf2, 0x1c, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 127, 0, 0, 1 });
        Locator.Read(bytes, littleEndian: true).ShouldBe(Locator.UdpV4(IPAddress.Loopback, 7410));
    }

    [Fact]
    public void Guid_and_entity_ids_are_big_endian()
    {
        var b = new byte[16];
        new RtpsGuid(s_prefix, EntityId.Participant).Write(b);
        b[..12].ShouldBe(new byte[] { 0x01, 0xa2, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 });
        b[12..].ShouldBe(new byte[] { 0x00, 0x00, 0x01, 0xc1 });
        RtpsGuid.Read(b).ShouldBe(new RtpsGuid(s_prefix, EntityId.Participant));
        RtpsGuid.FromGid(new RtpsGuid(s_prefix, EntityId.SedpPublicationsWriter).ToGid()).EntityId.ShouldBe(EntityId.SedpPublicationsWriter);
        EntityId.Create(5, EntityId.KindWriterNoKey).Value.ShouldBe(0x00000503u);
    }

    [Theory]
    [InlineData(0, 0, 7400, 7410, 7401, 7411)]
    [InlineData(1, 2, 7650, 7664, 7651, 7665)]
    public void Well_known_ports(int domain, int participant, int spdp, int meta, int userMc, int user)
    {
        RtpsPorts.SpdpMulticast(domain).ShouldBe(spdp);
        RtpsPorts.MetatrafficUnicast(domain, participant).ShouldBe(meta);
        RtpsPorts.UserMulticast(domain).ShouldBe(userMc);
        RtpsPorts.UserUnicast(domain, participant).ShouldBe(user);
    }

    [Fact]
    public void Time_uses_two_to_the_minus_32_fractions()
    {
        RtpsTime.FromNanoseconds(1_500_000_000).ShouldBe((1, 0x80000000u));
        RtpsTime.ToNanoseconds(1, 0x80000000u).ShouldBe(1_500_000_000);
        RtpsTime.FromDuration(null).ShouldBe((0x7fffffff, 0xffffffffu));
        RtpsTime.ToDuration(0x7fffffff, 0xffffffff).ShouldBeNull();
    }

    [Fact]
    public void Sequence_number_set_bits_are_msb_first()
    {
        var set = new SequenceNumberSet(10, 32);
        set.Add(10);
        set.Add(41);
        set.Bitmap[0].ShouldBe(0x80000001u);
        set.Members().ShouldBe(new long[] { 10, 41 });
    }

    [Fact]
    public void Heartbeat_matches_hand_computed_bytes()
    {
        using var b = new RtpsMessageBuilder(s_prefix, VendorId.Ros2Net);
        b.Begin().Heartbeat(EntityId.SedpPublicationsReader, EntityId.SedpPublicationsWriter, 1, 5, 7, final: true);
        var bytes = b.Buffer.Span.ToArray();

        bytes[..8].ShouldBe(new byte[] { (byte)'R', (byte)'T', (byte)'P', (byte)'S', 2, 3, 0x01, 0xa2 });
        bytes[20..24].ShouldBe(new byte[] { SubmessageId.Heartbeat, 0x03, 28, 0 }); // E|F flags, 28 octets
        bytes[24..28].ShouldBe(new byte[] { 0, 0, 3, 0xc7 });
        bytes[28..32].ShouldBe(new byte[] { 0, 0, 3, 0xc2 });
        bytes[32..40].ShouldBe(new byte[] { 0, 0, 0, 0, 1, 0, 0, 0 });
        bytes[40..48].ShouldBe(new byte[] { 0, 0, 0, 0, 5, 0, 0, 0 });
        bytes[48..52].ShouldBe(new byte[] { 7, 0, 0, 0 });
        bytes.Length.ShouldBe(52);
    }

    [Fact]
    public void Parameter_list_strings_are_padded()
    {
        using var buffer = new RtpsBuffer();
        var w = new ParameterListWriter(buffer);
        w.WriteString(ParameterId.TopicName, "rt/chatter");
        w.WriteSentinel();
        var bytes = buffer.Span.ToArray();
        bytes[..4].ShouldBe(new byte[] { 0x05, 0x00, 0x10, 0x00 });
        bytes[4..8].ShouldBe(new byte[] { 11, 0, 0, 0 });
        bytes[18].ShouldBe((byte)0); // NUL then one pad byte
        bytes[^4..].ShouldBe(new byte[] { 1, 0, 0, 0 });

        var pl = ParameterList.Parse(bytes, littleEndian: true, out var consumed);
        consumed.ShouldBe(bytes.Length);
        pl.GetString(ParameterId.TopicName).ShouldBe("rt/chatter");
    }

    private sealed class Collector : IRtpsMessageHandler
    {
        public List<string> Events { get; } = [];

        public byte[] Payload { get; private set; } = [];

        public string? TopicFromQos { get; private set; }

        public long? Timestamp { get; private set; }

        public void OnData(in ReceiveState state, in DataSubmessage data)
        {
            Events.Add($"data {data.WriterId} {data.SequenceNumber}");
            Payload = data.Payload.ToArray();
            Timestamp = state.Timestamp;
            if (!data.InlineQos.IsEmpty)
            {
                TopicFromQos = ParameterList.Parse(data.InlineQos, data.LittleEndian, out _).GetString(ParameterId.TopicName);
            }
        }

        public void OnDataFrag(in ReceiveState state, in DataFragSubmessage frag) => Events.Add($"frag {frag.FragmentStart}/{frag.SampleSize}/{frag.Fragments.Length}");

        public void OnHeartbeat(in ReceiveState state, HeartbeatSubmessage heartbeat) => Events.Add($"hb {heartbeat.First}-{heartbeat.Last} {heartbeat.Final}");

        public void OnAckNack(in ReceiveState state, AckNackSubmessage ackNack) => Events.Add($"ack {ackNack.State.Base} [{string.Join(",", ackNack.State.Members())}] {ackNack.Count}");

        public void OnGap(in ReceiveState state, GapSubmessage gap) => Events.Add($"gap {gap.GapStart}..{gap.GapList.Base - 1}");

        public void OnNackFrag(in ReceiveState state, NackFragSubmessage nackFrag) => Events.Add($"nackfrag {nackFrag.SequenceNumber} [{string.Join(",", nackFrag.State.Members())}]");
    }

    [Fact]
    public void Submessages_round_trip_through_the_parser()
    {
        var self = GuidPrefix.Create(VendorId.Ros2Net);
        using var b = new RtpsMessageBuilder(s_prefix, VendorId.Ros2Net);
        using var qosBuffer = new RtpsBuffer();
        var qw = new ParameterListWriter(qosBuffer);
        qw.WriteString(ParameterId.TopicName, "x");
        qw.WriteSentinel();

        var payload = CdrWriter.Serialize(new Msgs.StdMsgs.String { Data = "hello" }); // 14 bytes → 2 pad bytes
        var ack = new SequenceNumberSet(3, 8);
        ack.Add(4);
        ack.Add(6);
        var frags = new FragmentNumberSet(2, 4);
        frags.Add(3);

        b.Begin().InfoDst(self).InfoTs(1_500_000_000)
            .Data(EntityId.Unknown, new EntityId(0x103), 9, qosBuffer.Span, payload)
            .DataFrag(EntityId.Unknown, new EntityId(0x103), 10, 1, 4, 10, default, [1, 2, 3, 4])
            .Heartbeat(EntityId.Unknown, new EntityId(0x103), 1, 10, 1, final: false)
            .AckNack(new EntityId(0x104), new EntityId(0x103), ack, 2, final: false)
            .Gap(EntityId.Unknown, new EntityId(0x103), 5, new SequenceNumberSet(8, 0))
            .NackFrag(new EntityId(0x104), new EntityId(0x103), 10, frags, 1);

        var c = new Collector();
        RtpsMessageParser.Parse(b.Buffer.Span, self, c).ShouldBeTrue();

        c.Events.ShouldBe(new[]
        {
            "data 00000103 9",
            "frag 1/10/4",
            "hb 1-10 False",
            "ack 3 [4,6] 2",
            "gap 5..7",
            "nackfrag 10 [3]",
        });
        // Padding is stripped; the encapsulation options record the two pad bytes, as on the wire.
        c.Payload.Length.ShouldBe(payload.Length);
        c.Payload[3].ShouldBe((byte)2);
        c.Payload[4..].ShouldBe(payload[4..]);
        c.TopicFromQos.ShouldBe("x");
        c.Timestamp.ShouldBe(1_500_000_000);
    }

    [Fact]
    public void Messages_for_another_participant_are_skipped()
    {
        using var b = new RtpsMessageBuilder(s_prefix, VendorId.Ros2Net);
        b.Begin().InfoDst(GuidPrefix.Create(VendorId.Ros2Net)).Heartbeat(EntityId.Unknown, new EntityId(0x103), 1, 1, 1, true);
        var c = new Collector();
        RtpsMessageParser.Parse(b.Buffer.Span, GuidPrefix.Create(VendorId.Ros2Net), c);
        c.Events.ShouldBeEmpty();
        RtpsMessageParser.Parse("not rtps at all......"u8, s_prefix, c).ShouldBeFalse();
    }

    [Fact]
    public void Participant_data_round_trips()
    {
        var pd = new ParticipantData
        {
            Guid = new RtpsGuid(s_prefix, EntityId.Participant),
            DomainId = 7,
            LeaseDuration = TimeSpan.FromSeconds(20),
            BuiltinEndpoints = BuiltinEndpoints.PublicationsDetector | BuiltinEndpoints.SubscriptionsAnnouncer,
            UserData = "enclave=/robot;"u8.ToArray(),
        };
        pd.MetatrafficUnicast.Add(Locator.UdpV4(IPAddress.Loopback, 9010));
        pd.DefaultUnicast.Add(Locator.UdpV4(IPAddress.Parse("10.0.0.2"), 9011));
        using var buffer = new RtpsBuffer();
        pd.Write(buffer);

        var back = ParticipantData.Read(ParameterList.ParsePayload(buffer.Span));
        back.Guid.ShouldBe(pd.Guid);
        back.DomainId.ShouldBe(7);
        back.LeaseDuration.ShouldBe(TimeSpan.FromSeconds(20));
        back.BuiltinEndpoints.ShouldBe(pd.BuiltinEndpoints);
        back.Enclave.ShouldBe("/robot");
        back.MetatrafficUnicast.Single().Port.ShouldBe(9010u);
        back.DefaultUnicast.Single().Address.ShouldBe(IPAddress.Parse("10.0.0.2"));
    }

    [Fact]
    public void Endpoint_data_round_trips_with_qos()
    {
        var ed = new EndpointData
        {
            Guid = new RtpsGuid(s_prefix, EntityId.Create(3, EntityId.KindWriterNoKey)),
            ParticipantGuid = new RtpsGuid(s_prefix, EntityId.Participant),
            TopicName = "rt/scan",
            TypeName = "sensor_msgs::msg::dds_::LaserScan_",
            Qos = QosProfile.SensorData.WithDeadline(TimeSpan.FromMilliseconds(250)).AsTransientLocal(),
        };
        using var buffer = new RtpsBuffer();
        ed.Write(buffer);
        var back = EndpointData.Read(ParameterList.ParsePayload(buffer.Span));
        back.Guid.ShouldBe(ed.Guid);
        back.IsWriter.ShouldBeTrue();
        back.TopicName.ShouldBe("rt/scan");
        back.TypeName.ShouldBe(ed.TypeName);
        back.Qos.Reliability.ShouldBe(ReliabilityPolicy.BestEffort);
        back.Qos.Durability.ShouldBe(DurabilityPolicy.TransientLocal);
        back.Qos.Depth.ShouldBe(5);
        back.Qos.Deadline!.Value.TotalMilliseconds.ShouldBe(250, 0.001);
        back.Qos.Lifespan.ShouldBeNull();
    }

    [Theory]
    [InlineData(16)]
    [InlineData(24)]
    public void Discovery_info_round_trips_for_both_gid_layouts(int gidSize)
    {
        var g1 = new RtpsGuid(s_prefix, EntityId.Participant).ToGid();
        var g2 = new RtpsGuid(s_prefix, EntityId.Create(1, 3)).ToGid();
        var info = new ParticipantEntitiesInfo(gidSize) { ParticipantGid = g1, Nodes = [new NodeEntitiesInfo("/ns", "talker", [], [g2])] };
        var bytes = CdrWriter.Serialize(info);
        bytes.Length.ShouldBe(4 + gidSize + 4 + 8 + 11 + 1 + 4 + 4 + gidSize);

        var back = new ParticipantEntitiesInfo(gidSize);
        var r = new CdrReader(bytes);
        back.Deserialize(ref r);
        back.ParticipantGid.ShouldBe(g1);
        back.Nodes.Single().Name.ShouldBe("talker");
        back.Nodes.Single().Writers.Single().ShouldBe(g2);
    }

    [Theory]
    [InlineData("rt/chatter", "std_msgs::msg::dds_::String_", true, "/chatter", "std_msgs/msg/String", EndpointKind.Publisher)]
    [InlineData("rq/add_two_intsRequest", "example_interfaces::srv::dds_::AddTwoInts_Request_", false, "/add_two_ints", "example_interfaces/srv/AddTwoInts", EndpointKind.ServiceServer)]
    [InlineData("rq/fib/_action/send_goalRequest", "example_interfaces::action::dds_::Fibonacci_SendGoal_Request_", true, "/fib/_action/send_goal", "example_interfaces/action/Fibonacci_SendGoal", EndpointKind.ServiceClient)]
    public void Ros_names_map_to_and_from_dds(string dds, string type, bool writer, string name, string rosType, EndpointKind kind)
    {
        RosDdsNames.ToRos(dds, type, writer).ShouldBe((name, rosType, kind));
        RosDdsNames.ToRos("ros_discovery_info", "x", true).ShouldBeNull();
        RosDdsNames.Topic("/chatter").ShouldBe("rt/chatter");
        RosDdsNames.Request("/s").ShouldBe("rq/sRequest");
        RosDdsNames.Reply("/s").ShouldBe("rr/sReply");
    }
}
