using ROS2.Net.Bag;
using ROS2.Net.Bag.Mcap;
using ROS2.Net.Msgs;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Serialization;
using Shouldly;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Tf2.Tests;

public sealed class BagTests : IDisposable
{
    private readonly string _root = Path.Combine(Path.GetTempPath(), "ros2net-bag-tests", Guid.NewGuid().ToString("N"));

    public BagTests()
    {
        MessageExtensions.EnsureRegistered();
        Directory.CreateDirectory(_root);
    }

    public void Dispose()
    {
        try
        {
            Directory.Delete(_root, recursive: true);
        }
        catch (IOException)
        {
        }
    }

    private static RosTime T(double s) => new(1_700_000_000_000_000_000L + (long)Math.Round(s * 1e9));

    private string WriteSampleBag(string name, McapCompression compression = McapCompression.None, int chunkSize = 1024 * 1024)
    {
        var dir = Path.Combine(_root, name);
        using var w = new BagWriter(dir, new BagWriterOptions { Compression = compression, ChunkSize = chunkSize });
        w.AddTopic("/chatter", "std_msgs/msg/String", [QosProfile.Default]);
        w.AddTopic("/imu", "sensor_msgs/msg/Imu", [QosProfile.SensorData]);
        for (var i = 0; i < 50; i++)
        {
            w.Write("/chatter", new StdMsgs.String { Data = $"hello {i}" }, T(i * 0.1));
            var imu = new Imu();
            imu.Header.FrameId = "imu";
            imu.AngularVelocity.Z = i;
            w.Write("/imu", imu, T((i * 0.1) + 0.05));
        }

        return dir;
    }

    [Theory]
    [InlineData(McapCompression.None)]
    [InlineData(McapCompression.Zstd)]
    [InlineData(McapCompression.Lz4)]
    public void Write_then_read_round_trips_in_time_order(McapCompression compression)
    {
        var dir = WriteSampleBag("rt_" + compression, compression, chunkSize: 2048);
        using var reader = BagReader.Open(dir);

        reader.Metadata.MessageCount.ShouldBe(100ul);
        reader.Topics.Select(t => t.Name).ShouldBe(new[] { "/chatter", "/imu" });
        var all = reader.ReadMessages().ToList();
        all.Count.ShouldBe(100);
        all.Select(m => m.LogTime).ShouldBeInOrder();
        all[0].Topic.ShouldBe("/chatter");
        BagReader.Deserialize<StdMsgs.String>(all[0]).Data.ShouldBe("hello 0");
        BagReader.Deserialize<Imu>(all[3]).AngularVelocity.Z.ShouldBe(1);
        ((StdMsgs.String)BagReader.Deserialize(all[2])!).Data.ShouldBe("hello 1");
    }

    [Fact]
    public void File_layout_and_crc_are_valid()
    {
        var dir = WriteSampleBag("crc", chunkSize: 1024);
        var file = Directory.GetFiles(dir, "*.mcap").Single();
        var bytes = File.ReadAllBytes(file);
        bytes.AsSpan(0, 8).ToArray().ShouldBe(new byte[] { 0x89, (byte)'M', (byte)'C', (byte)'A', (byte)'P', (byte)'0', 13, 10 });
        bytes.AsSpan(bytes.Length - 8).ToArray().ShouldBe(bytes.AsSpan(0, 8).ToArray());

        using (var r = McapReader.Open(file))
        {
            r.HasSummary.ShouldBeTrue();
            r.Profile.ShouldBe("ros2");
            r.Statistics!.MessageCount.ShouldBe(100ul);
            r.ChunkIndexes.Count.ShouldBeGreaterThan(1);
            r.Schemas.Values.Single(s => s.Name == "sensor_msgs/msg/Imu").DataText.ShouldContain("MSG: std_msgs/Header");
            r.Channels.Values.Single(c => c.Topic == "/imu").Metadata["offered_qos_profiles"].ShouldContain("reliability: 2");
            r.Verify();
        }

        // Flip a byte inside the first chunk: the chunk CRC must catch it.
        using (var r = McapReader.Open(file))
        {
            var chunk = r.ChunkIndexes[0];
            bytes[(int)chunk.ChunkStartOffset + (int)chunk.ChunkLength - 3] ^= 0xFF;
        }

        var corrupt = Path.Combine(_root, "corrupt.mcap");
        File.WriteAllBytes(corrupt, bytes);
        using var bad = McapReader.Open(corrupt);
        Should.Throw<McapException>(() => bad.ReadMessages().ToList());
    }

    [Fact]
    public void Time_range_reads_only_the_overlapping_chunks()
    {
        var dir = WriteSampleBag("range", chunkSize: 512);
        var file = Directory.GetFiles(dir, "*.mcap").Single();
        using var r = McapReader.Open(file);
        var total = r.ChunkIndexes.Count;
        total.ShouldBeGreaterThan(4);

        var start = (ulong)T(2.0).Nanoseconds;
        var end = (ulong)T(2.45).Nanoseconds;
        var msgs = r.ReadMessages(start, end).ToList();
        msgs.ShouldAllBe(m => m.LogTime >= start && m.LogTime <= end);
        msgs.Count.ShouldBe(10); // 2.0, 2.05, ..., 2.45 alternating between the two topics
        r.ChunksRead.ShouldBeLessThan(total);

        using var reader = BagReader.Open(dir);
        reader.ReadMessages(T(2.0), T(2.45), ["/imu"]).Count().ShouldBe(5);
    }

    [Fact]
    public void Metadata_yaml_follows_rosbag2()
    {
        var dir = WriteSampleBag("meta");
        var yaml = File.ReadAllText(Path.Combine(dir, "metadata.yaml"));
        yaml.ShouldContain("rosbag2_bagfile_information:");
        yaml.ShouldContain("storage_identifier: mcap");
        yaml.ShouldContain("message_count: 100");
        yaml.ShouldContain("name: /imu");
        yaml.ShouldContain("type: sensor_msgs/msg/Imu");
        yaml.ShouldContain("- meta_0.mcap");

        var meta = BagMetadata.Parse(yaml);
        meta.Topics.Single(t => t.Name == "/imu").OfferedQos[0].Reliability.ShouldBe(ReliabilityPolicy.BestEffort);
        meta.Topics.Single(t => t.Name == "/chatter").MessageCount.ShouldBe(50ul);
        meta.Duration.Nanoseconds.ShouldBe((T(4.95) - T(0)).Nanoseconds);

        var info = BagInfo.Read(dir).ToString();
        info.ShouldContain("Messages:          100");
        info.ShouldContain("Topic: /chatter | Type: std_msgs/msg/String | Count: 50 | Serialization Format: cdr");
    }

    [Fact]
    public void Splits_files_by_size()
    {
        var dir = Path.Combine(_root, "split");
        using (var w = new BagWriter(dir, new BagWriterOptions { MaxFileSize = 4096, ChunkSize = 1024 }))
        {
            for (var i = 0; i < 400; i++)
            {
                w.Write("/n", new StdMsgs.Int32 { Data = i }, T(i * 0.01));
            }
        }

        using var reader = BagReader.Open(dir);
        reader.Files.Count.ShouldBeGreaterThan(1);
        reader.ReadMessages().Select(m => BagReader.Deserialize<StdMsgs.Int32>(m).Data).ShouldBe(Enumerable.Range(0, 400));
    }

    [Fact]
    public void Messages_decode_to_json_from_the_stored_schema()
    {
        var dir = WriteSampleBag("json");
        using var reader = BagReader.Open(dir);
        var imu = reader.ReadMessages(topics: ["/imu"]).First();
        var json = reader.ToJson(imu);
        json.ShouldBe(RosJson.ToJsonString(BagReader.Deserialize<Imu>(imu)));

        using var sw = new StringWriter();
        BagExporter.ToJsonLines(reader, sw, ["/chatter"]).ShouldBe(50);
        sw.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries)[1].ShouldContain("\"data\":\"hello 1\"");

        using var csv = new StringWriter();
        BagExporter.ToCsv(reader, "/imu", csv).ShouldBe(50);
        var lines = csv.ToString().Split('\n', StringSplitOptions.RemoveEmptyEntries);
        lines[0].ShouldStartWith("log_time_ns,header.stamp.sec");
        lines[0].ShouldContain("angular_velocity.z");
        lines[0].ShouldContain("orientation_covariance[8]");
    }

    [Fact]
    public async Task Recorder_and_player_round_trip_over_the_middleware()
    {
        var dir = Path.Combine(_root, "recorded");
        await using (var ctx = Ctx.Create())
        {
            var talker = ctx.CreateNode("talker");
            var chatter = talker.CreatePublisher<StdMsgs.String>("chatter");
            var counter = talker.CreatePublisher<StdMsgs.Int32>("count");

            var recNode = ctx.CreateNode("recorder");
            await using var recorder = new BagRecorder(recNode, dir, new BagRecorderOptions { All = true, Exclude = "^/(rosout|parameter_events)$" });
            recorder.Start();
            (await recorder.WaitForTopicsAsync(2, TimeSpan.FromSeconds(5))).ShouldBeTrue();

            for (var i = 0; i < 10; i++)
            {
                chatter.Publish(new StdMsgs.String { Data = $"m{i}" });
                counter.Publish(new StdMsgs.Int32 { Data = i });
                await Task.Delay(10);
            }

            await Task.Delay(100);
            await recorder.StopAsync();
            recorder.MessageCount.ShouldBe(20ul);
        }

        using var reader = BagReader.Open(dir);
        reader.Topics.Select(t => t.Name).ShouldBe(new[] { "/chatter", "/count" });
        var duration = reader.Metadata.Duration.TotalSeconds;

        await using var playCtx = Ctx.Create();
        var listener = playCtx.CreateNode("listener");
        var chatterSub = listener.CreateSubscription<StdMsgs.String>("/replay_chatter", QosProfile.Default.KeepAll());
        var countSub = listener.CreateSubscription<StdMsgs.Int32>("/count", QosProfile.Default.KeepAll());

        var player = new BagPlayer(playCtx.CreateNode("player"), dir, new BagPlayerOptions { Rate = 2.0, Remappings = { ["/chatter"] = "/replay_chatter" } });
        var sw = System.Diagnostics.Stopwatch.StartNew();
        await player.PlayAsync().WaitAsync(TimeSpan.FromSeconds(10));
        sw.Stop();
        player.PlayedCount.ShouldBe(20);
        sw.Elapsed.TotalSeconds.ShouldBeGreaterThan(duration / 2 * 0.7);

        var got = new List<string>();
        while (chatterSub.TryTake(out var m))
        {
            got.Add(m.Data);
        }

        got.ShouldBe(Enumerable.Range(0, 10).Select(i => $"m{i}"));
        var counts = new List<int>();
        while (countSub.TryTake(out var c))
        {
            counts.Add(c.Data);
        }

        counts.ShouldBe(Enumerable.Range(0, 10));
        player.Dispose();
    }

    [Fact]
    public async Task Player_steps_while_paused_and_publishes_clock()
    {
        var dir = WriteSampleBag("step");
        await using var ctx = Ctx.Create();
        var node = ctx.CreateNode("observer");
        var sub = node.CreateSubscription<StdMsgs.String>("/chatter", QosProfile.Default.KeepAll());
        var clock = node.CreateSubscription<Msgs.RosgraphMsgs.Clock>("/clock", QosProfile.Default.KeepLast(1));

        using var player = new BagPlayer(ctx.CreateNode("player"), dir, new BagPlayerOptions { StartPaused = true, Topics = { "/chatter" }, ClockRate = 100 });
        using var cts = new CancellationTokenSource();
        var play = player.PlayAsync(cts.Token);

        (await player.StepAsync().WaitAsync(TimeSpan.FromSeconds(5))).ShouldBeTrue();
        (await player.StepAsync().WaitAsync(TimeSpan.FromSeconds(5))).ShouldBeTrue();
        sub.TryTake(out var a).ShouldBeTrue();
        sub.TryTake(out var b).ShouldBeTrue();
        (a!.Data, b!.Data).ShouldBe(("hello 0", "hello 1"));
        sub.TryTake(out _).ShouldBeFalse();

        var tick = await clock.TakeAsync().AsTask().WaitAsync(TimeSpan.FromSeconds(5));
        RosTime.FromMsg(tick.ClockValue).Seconds.ShouldBeGreaterThanOrEqualTo(1_700_000_000); // bag time, not wall time

        cts.Cancel();
        await Should.ThrowAsync<OperationCanceledException>(() => play);
    }
}
