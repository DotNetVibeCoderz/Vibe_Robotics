using System.CommandLine;
using ROS2.Net.Bag;
using ROS2.Net.Bag.Mcap;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net bag ...</c>: record, play, inspect and export rosbag2 (MCAP) bags.</summary>
internal static class BagCommands
{
    public static Command Build(CliEnvironment env)
    {
        var bag = new Command("bag", "Record, play, inspect and export rosbag2 MCAP bags.");
        bag.Add(Record(env));
        bag.Add(Play(env));
        bag.Add(Info(env));
        bag.Add(Export(env));
        return bag;
    }

    private static Command Record(CliEnvironment env)
    {
        var topics = new Argument<string[]>("topics") { Arity = ArgumentArity.ZeroOrMore, Description = "Topics to record." };
        var all = new Option<bool>("--all", "-a") { Description = "Record all topics." };
        var output = new Option<string?>("--output", "-o") { Description = "Bag directory (default rosbag2_<timestamp>)." };
        var include = new Option<string?>("--regex", "-e") { Description = "Record topics matching this regular expression." };
        var exclude = new Option<string?>("--exclude", "-x") { Description = "Exclude topics matching this regular expression." };
        var hidden = new Option<bool>("--include-hidden-topics");
        var compression = new Option<string>("--compression") { Description = "none | zstd | lz4 (MCAP chunk compression).", DefaultValueFactory = _ => "none" };
        var maxSize = new Option<long>("--max-bag-size", "-b") { Description = "Split files at this many bytes (0 = no split)." };
        var maxDuration = new Option<double?>("--max-bag-duration", "-d") { Description = "Split files after this many seconds." };
        var duration = new Option<double?>("--duration") { Description = "Stop recording after this many seconds." };
        var overwrite = new Option<bool>("--overwrite") { Description = "Replace an existing bag directory." };
        var cmd = new Command("record", "Record topics into a bag.") { topics, all, output, include, exclude, hidden, compression, maxSize, maxDuration, duration, overwrite };
        cmd.Handle(env, async (p, ct) =>
        {
            var list = p.GetValue(topics) ?? [];
            if (list.Length == 0 && !p.GetValue(all) && p.GetValue(include) is null)
            {
                throw new CliException("Specify topics, --all or --regex.");
            }

            var options = new BagRecorderOptions
            {
                All = p.GetValue(all),
                Include = p.GetValue(include),
                Exclude = p.GetValue(exclude),
                IncludeHidden = p.GetValue(hidden),
                Writer = new BagWriterOptions
                {
                    Compression = p.GetValue(compression)!.ToLowerInvariant() switch
                    {
                        "none" or "" => McapCompression.None,
                        "zstd" => McapCompression.Zstd,
                        "lz4" => McapCompression.Lz4,
                        var c => throw new CliException($"Unknown compression '{c}'."),
                    },
                    MaxFileSize = p.GetValue(maxSize),
                    MaxFileDuration = p.GetValue(maxDuration) is { } d ? TimeSpan.FromSeconds(d) : null,
                    Overwrite = p.GetValue(overwrite),
                },
            };
            options.Topics.AddRange(list);

            var dir = p.GetValue(output) ?? $"rosbag2_{DateTime.Now:yyyy_MM_dd-HH_mm_ss}";
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            var recorder = new BagRecorder(s.Node, dir, options);
            recorder.TopicAdded += (topic, type) => env.Out.WriteLine($"Subscribed to topic '{topic}' [{type}]");
            recorder.Start();
            env.Out.WriteLine($"Recording to '{recorder.BagDirectory}'... press Ctrl+C to stop.");
            using var limit = Cmd.Limit(p.GetValue(duration), ct);
            try
            {
                await Task.Delay(Timeout.Infinite, limit.Token);
            }
            catch (OperationCanceledException)
            {
            }

            await recorder.StopAsync();
            env.Out.WriteLine($"Recorded {recorder.MessageCount} message(s) on {recorder.RecordedTopics.Count} topic(s).");
            await recorder.DisposeAsync();
            return 0;
        });
        return cmd;
    }

    private static Command Play(CliEnvironment env)
    {
        var path = new Argument<string>("bag_path");
        var rate = new Option<double>("--rate", "-r") { Description = "Playback speed factor.", DefaultValueFactory = _ => 1.0 };
        var loop = new Option<bool>("--loop", "-l") { Description = "Loop playback." };
        var topics = new Option<string[]>("--topics") { Description = "Only play these topics.", AllowMultipleArgumentsPerToken = true };
        var remap = new Option<string[]>("--remap", "-m") { Description = "Remap topics: old:=new.", AllowMultipleArgumentsPerToken = true };
        var clock = new Option<double?>("--clock") { Description = "Publish /clock at this rate in Hz (e.g. 100).", Arity = ArgumentArity.ZeroOrOne };
        var start = new Option<double>("--start-offset") { Description = "Start this many seconds into the bag." };
        var paused = new Option<bool>("--start-paused");
        var delay = new Option<double>("--delay", "-d") { Description = "Seconds to wait (for discovery) before playing.", DefaultValueFactory = _ => 0.5 };
        var cmd = new Command("play", "Play back a bag.") { path, rate, loop, topics, remap, clock, start, paused, delay };
        cmd.Handle(env, async (p, ct) =>
        {
            var options = new BagPlayerOptions
            {
                Rate = p.GetValue(rate),
                Loop = p.GetValue(loop),
                StartOffset = TimeSpan.FromSeconds(p.GetValue(start)),
                StartPaused = p.GetValue(paused),
                Delay = TimeSpan.FromSeconds(p.GetValue(delay)),
                ClockRate = p.GetResult(clock) is null ? 0 : p.GetValue(clock) ?? 100,
            };
            options.Topics.AddRange(p.GetValue(topics) ?? []);
            foreach (var r in p.GetValue(remap) ?? [])
            {
                var sep = r.IndexOf(":=", StringComparison.Ordinal);
                if (sep <= 0)
                {
                    throw new CliException($"Remap '{r}' must be old:=new.");
                }

                options.Remappings[r[..sep]] = r[(sep + 2)..];
            }

            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            using var player = new BagPlayer(s.Node, p.GetValue(path)!, options);
            env.Out.WriteLine($"Playing '{p.GetValue(path)}' ({player.Reader.Metadata.MessageCount} messages) at {options.Rate}x...");
            await player.PlayAsync(ct);
            env.Out.WriteLine($"Played {player.PlayedCount} message(s).");
            return 0;
        });
        return cmd;
    }

    private static Command Info(CliEnvironment env)
    {
        var path = new Argument<string>("bag_path");
        var cmd = new Command("info", "Print information about a bag.") { path };
        cmd.Handle(env, (p, _) =>
        {
            env.Out.WriteLine(BagInfo.Read(p.GetValue(path)!).ToString());
            return Task.FromResult(0);
        });
        return cmd;
    }

    private static Command Export(CliEnvironment env)
    {
        var path = new Argument<string>("bag_path");
        var format = new Option<string>("--format", "-f") { Description = "jsonl | csv", DefaultValueFactory = _ => "jsonl" };
        var topic = new Option<string[]>("--topic", "-t") { Description = "Topic(s) to export (csv needs exactly one).", AllowMultipleArgumentsPerToken = true };
        var output = new Option<string?>("--output", "-o") { Description = "Output file (stdout when omitted)." };
        var cmd = new Command("export", "Export messages as JSON Lines or CSV (datasets for AI training).") { path, format, topic, output };
        cmd.Handle(env, async (p, ct) =>
        {
            using var reader = BagReader.Open(p.GetValue(path)!);
            var topics = p.GetValue(topic) ?? [];
            await using var file = p.GetValue(output) is { } o ? new StreamWriter(o) : null;
            var writer = (TextWriter?)file ?? env.Out;
            long count;
            switch (p.GetValue(format)!.ToLowerInvariant())
            {
                case "jsonl":
                    count = BagExporter.ToJsonLines(reader, writer, topics.Length > 0 ? topics : null);
                    break;
                case "csv":
                    if (topics.Length != 1)
                    {
                        throw new CliException("CSV export needs exactly one --topic.");
                    }

                    count = BagExporter.ToCsv(reader, topics[0], writer);
                    break;
                default:
                    throw new CliException($"Unknown format '{p.GetValue(format)}'.");
            }

            await writer.FlushAsync(ct);
            if (file is not null)
            {
                env.Out.WriteLine($"Exported {count} message(s) to {p.GetValue(output)}");
            }

            return 0;
        });
        return cmd;
    }
}
