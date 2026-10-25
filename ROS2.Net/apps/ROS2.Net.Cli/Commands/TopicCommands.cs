using System.CommandLine;
using System.Diagnostics;
using System.Threading.Channels;
using ROS2.Net.Serialization;

namespace ROS2.Net.Cli.Commands;

/// <summary><c>ros2net topic ...</c></summary>
internal static class TopicCommands
{
    public static Command Build(CliEnvironment env)
    {
        var topic = new Command("topic", "Inspect and interact with topics.");
        topic.Add(List(env));
        topic.Add(Info(env));
        topic.Add(TypeCmd(env));
        topic.Add(Echo(env));
        topic.Add(Pub(env));
        topic.Add(Hz(env, bandwidth: false));
        topic.Add(Hz(env, bandwidth: true));
        return topic;
    }

    private static Command List(CliEnvironment env)
    {
        var showTypes = new Option<bool>("--show-types", "-t") { Description = "Show topic types." };
        var count = new Option<bool>("--count-topics", "-c") { Description = "Only print the number of topics." };
        var hidden = new Option<bool>("--include-hidden-topics") { Description = "Include hidden topics." };
        var cmd = new Command("list", "List topics.") { showTypes, count, hidden };
        cmd.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            var topics = s.Node.GetTopicNamesAndTypes(p.GetValue(hidden));
            if (p.GetValue(count))
            {
                env.Out.WriteLine(topics.Count);
                return 0;
            }

            foreach (var t in topics)
            {
                env.Out.WriteLine(p.GetValue(showTypes) ? $"{t.Name} [{string.Join(", ", t.Types)}]" : t.Name);
            }

            return 0;
        });
        return cmd;
    }

    private static Command Info(CliEnvironment env)
    {
        var name = new Argument<string>("topic_name");
        var verbose = new Option<bool>("--verbose", "-v") { Description = "Print endpoint details (node, GID, QoS)." };
        var cmd = new Command("info", "Print information about a topic.") { name, verbose };
        cmd.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            var topicName = s.Node.ResolveTopicName(p.GetValue(name)!);
            var pubs = s.Node.GetPublishersInfo(topicName);
            var subs = s.Node.GetSubscriptionsInfo(topicName).Where(e => e.NodeName != s.Node.Name).ToList();
            var types = pubs.Concat(subs).Select(e => e.TypeName).Distinct().ToList();
            env.Out.WriteLine($"Type: {(types.Count == 0 ? "<unknown>" : string.Join(", ", types))}");
            env.Out.WriteLine();
            env.Out.WriteLine($"Publisher count: {pubs.Count}");
            Endpoints(env, pubs, p.GetValue(verbose));
            env.Out.WriteLine($"Subscription count: {subs.Count}");
            Endpoints(env, subs, p.GetValue(verbose));
            return 0;
        });
        return cmd;
    }

    private static void Endpoints(CliEnvironment env, IEnumerable<Rmw.EndpointInfo> endpoints, bool verbose)
    {
        if (!verbose)
        {
            env.Out.WriteLine();
            return;
        }

        foreach (var e in endpoints)
        {
            env.Out.WriteLine();
            env.Out.WriteLine($"Node name: {e.NodeName}");
            env.Out.WriteLine($"Node namespace: {e.NodeNamespace}");
            env.Out.WriteLine($"Topic type: {e.TypeName}");
            env.Out.WriteLine($"Endpoint type: {e.Kind.ToString().ToUpperInvariant()}");
            env.Out.WriteLine($"GID: {e.Gid}");
            env.Out.WriteLine("QoS profile:");
            env.Out.WriteLine($"  Reliability: {e.Qos.Reliability}");
            env.Out.WriteLine($"  History (Depth): {e.Qos.History} ({e.Qos.Depth})");
            env.Out.WriteLine($"  Durability: {e.Qos.Durability}");
            env.Out.WriteLine($"  Lifespan: {(e.Qos.Lifespan is { } l ? l.ToString() : "Infinite")}");
            env.Out.WriteLine($"  Deadline: {(e.Qos.Deadline is { } d ? d.ToString() : "Infinite")}");
            env.Out.WriteLine($"  Liveliness: {e.Qos.Liveliness}");
        }

        env.Out.WriteLine();
    }

    private static Command TypeCmd(CliEnvironment env)
    {
        var name = new Argument<string>("topic_name");
        var cmd = new Command("type", "Print a topic's type.") { name };
        cmd.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            var fqn = s.Node.ResolveTopicName(p.GetValue(name)!);
            var t = s.Node.GetTopicNamesAndTypes(includeHidden: true).FirstOrDefault(x => x.Name == fqn);
            if (t is null)
            {
                throw new CliException($"Unknown topic '{fqn}'.");
            }

            foreach (var type in t.Types)
            {
                env.Out.WriteLine(type);
            }

            return 0;
        });
        return cmd;
    }

    private static Command Echo(CliEnvironment env)
    {
        var name = new Argument<string>("topic_name");
        var type = new Argument<string?>("message_type") { Arity = ArgumentArity.ZeroOrOne, Description = "Type, e.g. std_msgs/msg/String (discovered from the graph when omitted)." };
        var once = new Option<bool>("--once") { Description = "Print the first message and exit." };
        var count = new Option<int?>("--times", "-n") { Description = "Exit after N messages." };
        var json = new Option<bool>("--json") { Description = "Print each message as one line of JSON." };
        var timeout = new Option<double?>("--timeout") { Description = "Give up after this many seconds." };
        var qos = new QosOptions();
        var cmd = new Command("echo", "Print messages from a topic.") { name, type, once, count, json, timeout };
        qos.AddTo(cmd);
        cmd.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            using var limit = Cmd.Limit(p.GetValue(timeout), ct);
            var topicName = s.Node.ResolveTopicName(p.GetValue(name)!);
            var typeName = p.GetValue(type) ?? await s.WaitForTopicTypeAsync(topicName, TimeSpan.FromSeconds(p.GetValue(timeout) ?? 5), limit.Token)
                ?? throw new CliException($"Could not determine the type of '{topicName}'. Pass it explicitly.");
            var info = Types.Resolve(typeName);
            if (!DynamicMessageType.TryFromRegistry(info.Name, out var dynamic))
            {
                throw new CliException($"The definition of '{typeName}' is unknown to ros2net; cannot decode it.");
            }

            var max = p.GetValue(once) ? 1 : p.GetValue(count) ?? int.MaxValue;
            var channel = Channel.CreateBounded<byte[]>(new BoundedChannelOptions(1000) { FullMode = BoundedChannelFullMode.DropOldest });
            using var sub = s.Node.CreateSerializedSubscription(topicName, info, (cdr, _) => channel.Writer.TryWrite(cdr.ToArray()), qos.Resolve(p, s.Node, topicName));
            var printed = 0;
            try
            {
                await foreach (var cdr in channel.Reader.ReadAllAsync(limit.Token))
                {
                    var text = dynamic.ToJson(cdr);
                    if (p.GetValue(json))
                    {
                        env.Out.WriteLine(text);
                    }
                    else
                    {
                        env.Out.WriteLine(YamlJson.ToYaml(text));
                        env.Out.WriteLine("---");
                    }

                    env.Out.Flush();
                    if (++printed >= max)
                    {
                        break;
                    }
                }
            }
            catch (OperationCanceledException) when (!ct.IsCancellationRequested)
            {
                if (printed == 0)
                {
                    throw new CliException($"No message received on '{topicName}' before the timeout.");
                }
            }

            return 0;
        });
        return cmd;
    }

    private static Command Pub(CliEnvironment env)
    {
        var name = new Argument<string>("topic_name");
        var type = new Argument<string>("message_type") { Description = "e.g. geometry_msgs/msg/Twist" };
        var values = new Argument<string?>("values") { Arity = ArgumentArity.ZeroOrOne, Description = "Message as YAML or JSON, e.g. \"{linear: {x: 0.5}}\"." };
        var rate = new Option<double>("--rate", "-r") { Description = "Publishing rate in Hz.", DefaultValueFactory = _ => 1.0 };
        var once = new Option<bool>("--once", "-1") { Description = "Publish one message and exit." };
        var times = new Option<int?>("--times", "-t") { Description = "Publish N messages and exit." };
        var wait = new Option<double>("--wait-matching") { Description = "Seconds to wait for a subscription before the first publish.", DefaultValueFactory = _ => 1.0 };
        var qos = new QosOptions();
        var cmd = new Command("pub", "Publish messages to a topic.") { name, type, values, rate, once, times, wait };
        qos.AddTo(cmd);
        cmd.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            var info = Types.Resolve(p.GetValue(type)!);
            if (!DynamicMessageType.TryFromRegistry(info.Name, out var dynamic))
            {
                throw new CliException($"Unknown message type '{p.GetValue(type)}'.");
            }

            var payload = dynamic.FromJson(YamlJson.ToJson(p.GetValue(values)));
            var topicName = s.Node.ResolveTopicName(p.GetValue(name)!);
            var qosProfile = p.GetValue(qos.Profile) is null && p.GetValue(qos.Reliability) is null && p.GetValue(qos.Durability) is null
                ? QosProfile.Default.KeepLast(p.GetValue(qos.Depth) ?? 10)
                : qos.Resolve(p, s.Node, topicName);
            using var pub = s.Node.CreateSerializedPublisher(topicName, info, qosProfile);
            env.Out.WriteLine($"publisher: beginning loop");

            var waitUntil = Stopwatch.GetTimestamp() + (long)(p.GetValue(wait) * Stopwatch.Frequency);
            while (pub.SubscriptionCount == 0 && Stopwatch.GetTimestamp() < waitUntil)
            {
                await Task.Delay(20, ct);
            }

            var total = p.GetValue(once) ? 1 : p.GetValue(times) ?? int.MaxValue;
            var period = TimeSpan.FromSeconds(1.0 / Math.Max(1e-3, p.GetValue(rate)));
            for (var i = 0; i < total; i++)
            {
                pub.Publish(payload);
                env.Out.WriteLine($"publishing #{i + 1}: {dynamic.ToJson(payload)}");
                env.Out.Flush();
                if (i + 1 < total)
                {
                    await Task.Delay(period, ct);
                }
            }

            // Give reliable middlewares a moment to deliver before the context is torn down.
            await Task.Delay(s.Rmw == "inprocess" ? 0 : 300, ct);
            return 0;
        });
        return cmd;
    }

    private static Command Hz(CliEnvironment env, bool bandwidth)
    {
        var name = new Argument<string>("topic_name");
        var window = new Option<int>("--window", "-w") { Description = "Window size in messages.", DefaultValueFactory = _ => 10000 };
        var duration = new Option<double?>("--duration") { Description = "Stop after this many seconds." };
        var qos = new QosOptions();
        var cmd = bandwidth
            ? new Command("bw", "Print the bandwidth used by a topic.") { name, window, duration }
            : new Command("hz", "Print the average publishing rate of a topic.") { name, window, duration };
        qos.AddTo(cmd);
        cmd.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            using var limit = Cmd.Limit(p.GetValue(duration), ct);
            var topicName = s.Node.ResolveTopicName(p.GetValue(name)!);
            var typeName = await s.WaitForTopicTypeAsync(topicName, TimeSpan.FromSeconds(5), limit.Token)
                ?? throw new CliException($"Topic '{topicName}' is not published.");
            var stats = new RateStats(p.GetValue(window));
            using var sub = s.Node.CreateSerializedSubscription(topicName, Types.Resolve(typeName), (cdr, _) => stats.Add(cdr.Length), qos.Resolve(p, s.Node, topicName));
            try
            {
                while (true)
                {
                    await Task.Delay(1000, limit.Token);
                    env.Out.WriteLine(bandwidth ? stats.FormatBandwidth() : stats.FormatRate());
                    env.Out.Flush();
                }
            }
            catch (OperationCanceledException) when (!ct.IsCancellationRequested)
            {
            }

            return 0;
        });
        return cmd;
    }

    /// <summary>Sliding-window arrival statistics for <c>hz</c> and <c>bw</c>.</summary>
    internal sealed class RateStats(int window)
    {
        private readonly Queue<(long Ticks, int Bytes)> _samples = new();
        private readonly object _gate = new();

        public void Add(int bytes)
        {
            lock (_gate)
            {
                _samples.Enqueue((Stopwatch.GetTimestamp(), bytes));
                while (_samples.Count > window)
                {
                    _samples.Dequeue();
                }
            }
        }

        public string FormatRate()
        {
            lock (_gate)
            {
                if (_samples.Count < 2)
                {
                    return "no new messages";
                }

                var arr = _samples.ToArray();
                var deltas = new double[arr.Length - 1];
                for (var i = 1; i < arr.Length; i++)
                {
                    deltas[i - 1] = (arr[i].Ticks - arr[i - 1].Ticks) / (double)Stopwatch.Frequency;
                }

                var mean = deltas.Average();
                var std = Math.Sqrt(deltas.Select(d => (d - mean) * (d - mean)).Average());
                return $"average rate: {1 / mean:0.000}\n\tmin: {deltas.Min():0.000}s max: {deltas.Max():0.000}s std dev: {std:0.00000}s window: {arr.Length}";
            }
        }

        public string FormatBandwidth()
        {
            lock (_gate)
            {
                if (_samples.Count < 2)
                {
                    return "no new messages";
                }

                var arr = _samples.ToArray();
                var seconds = (arr[^1].Ticks - arr[0].Ticks) / (double)Stopwatch.Frequency;
                var bytes = arr.Sum(a => (long)a.Bytes);
                return $"{Bag.BagInfo.FormatSize((long)(bytes / Math.Max(seconds, 1e-9)))}/s from {arr.Length} messages\n\tMessage size mean: {Bag.BagInfo.FormatSize(bytes / arr.Length)} min: {Bag.BagInfo.FormatSize(arr.Min(a => a.Bytes))} max: {Bag.BagInfo.FormatSize(arr.Max(a => a.Bytes))}";
            }
        }
    }
}
