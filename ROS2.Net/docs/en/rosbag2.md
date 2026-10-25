# rosbag2 — recording, playback and datasets

`ROS2.Net.Bag` records and replays ROS 2 topics in the rosbag2 MCAP format. A bag is a directory with `<name>_0.mcap` (plus `_1`, `_2`, ... after splits) and a rosbag2 `metadata.yaml`.

```bash
dotnet add package ROS2.Net.Bag
```

## Recording

```csharp
var node = context.CreateNode("recorder");
await using var recorder = new BagRecorder(node, "bags/run1", new BagRecorderOptions
{
    All = true,                                    // or Topics = { "/scan", "/odom" }
    Exclude = "^/(rosout|parameter_events)$",      // regex, like ros2 bag record -x
    Writer = new BagWriterOptions { Compression = McapCompression.Zstd, MaxFileSize = 512 << 20 },
});
recorder.Start();
// ...
await recorder.StopAsync();                        // flushes and writes metadata.yaml
```

- Topics are found through the graph: polling every `DiscoveryInterval`, and waking early when the graph changes. Topics that appear later are picked up automatically. A topic is recorded once it has a publisher. Use `TopicTypes` to start recording a topic before anyone publishes it.
- Messages arrive as raw CDR and are never deserialized. They are copied into pooled buffers and written by one background task, so middleware receive threads never block on disk I/O.
- **QoS adaptation** works like rosbag2: the recorder subscribes reliable / transient-local only if every publisher offers that, and best-effort / volatile otherwise. The publishers' offered profiles are stored per topic.
- **Splitting:** `MaxFileSize` (bytes) and `MaxFileDuration`.

`BagWriter` is the lower-level API for writing data you already have: `Write(topic, cdr, time)` or `Write<T>(topic, message, time)`.

## Playback

```csharp
using var player = new BagPlayer(context.CreateNode("player"), "bags/run1", new BagPlayerOptions
{
    Rate = 2.0,
    Loop = false,
    StartOffset = TimeSpan.FromSeconds(5),
    ClockRate = 100,                               // publish /clock for use_sim_time nodes
    Remappings = { ["/scan"] = "/scan_replay" },
});
await player.PlayAsync(ct);
```

- Inter-message timing is preserved and scaled by `Rate`, which you can change while playing.
- `Pause()`, `Resume()` and `TogglePause()` control playback. While paused, `StepAsync()` publishes exactly one message.
- Each topic is published with its recorded QoS, so latched topics such as `/map` and `/tf_static` stay latched. `QosOverrides` replaces the QoS for specific topics.
- `Topics` filters which topics play. `Delay` waits before the first message.

## Reading and inspecting

```csharp
using var bag = BagReader.Open("bags/run1");      // directory, or a single .mcap file
foreach (var m in bag.ReadMessages(start, end, topics: ["/imu"]))
{
    var imu = BagReader.Deserialize<Imu>(m);      // typed
    string json = bag.ToJson(m);                  // uses the schema stored in the bag: no generated type needed
}

Console.WriteLine(BagInfo.Read("bags/run1"));     // same layout as `ros2 bag info`
```

Messages come in log-time order. With an indexed file (all files written by ROS2.Net are indexed), a time-range read only decompresses the chunks that overlap the range.

## Datasets for AI training

```csharp
BagExporter.ToJsonLinesFile(bag, "imu.jsonl", ["/imu"]);    // {"topic","type","log_time_ns","publish_time_ns","message":{...}}
BagExporter.ToCsvFile(bag, "/imu", "imu.csv");              // flattened columns: angular_velocity.z, orientation_covariance[8], ...
```

The CSV columns are `log_time_ns` plus every leaf field of the first message. Nested fields are written as `a.b` and arrays as `a[i]`, up to `maxArrayElements`. Base64 byte blobs such as image data are skipped.

## MCAP details

`McapWriter` and `McapReader` implement the [MCAP spec](https://mcap.dev/spec):
- Records: Header (profile `ros2`), Schema (`ros2msg`, holding the self-contained definition from `RosTypeRegistry.GetFullDefinition`), Channel (`cdr`, with `offered_qos_profiles` metadata), Message, Chunk, Message Index, Chunk Index, Statistics, Metadata, Metadata Index, Data End, Summary Offset and Footer.
- **CRC32:** chunks, the data section and the summary section, all checked when reading.
- **Compression:** none, zstd (ZstdSharp) and lz4 frame (K4os). All three are pure managed code.
- **Files without a summary** are read by scanning sequentially.

`metadata.yaml` is written in rosbag2 format version 5, with `offered_qos_profiles` in the Humble integer form. The reader accepts both the integer and the Iron/Jazzy string forms. The bag-level `compression_format` stays empty because MCAP chunk compression is transparent to rosbag2.

## Verification status

Tests cover the following:
- round trips with all three compressions
- file magic and layout, CRC detection of corrupted chunks and index-based time-range reads
- `metadata.yaml` content, file splitting and JSON decoding from the stored schema
- JSONL and CSV export
- recorder → player end to end over the in-process middleware at 2× speed, with remapping
- stepping while paused and `/clock`

**Not verified:** these bags have not been opened with the real `ros2 bag` or Foxglove Studio, because no ROS 2 installation is available in the build environment. The format follows the specifications, but treat cross-tool compatibility as untested.
