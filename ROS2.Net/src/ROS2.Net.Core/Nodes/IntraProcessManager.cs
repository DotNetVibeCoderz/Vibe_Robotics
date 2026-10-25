using ROS2.Net.Rmw;

namespace ROS2.Net;

/// <summary>
/// Zero-copy delivery between publishers and subscriptions of the same <see cref="Ros2Context"/>,
/// like rclcpp's intra-process manager. Local subscriptions receive the publisher's instance
/// directly; the middleware still carries the message to remote subscribers, and is told to drop
/// the local copy it would otherwise loop back.
/// </summary>
internal sealed class IntraProcessManager
{
    private readonly object _gate = new();
    private readonly Dictionary<string, List<LocalSubscription>> _byTopic = new(StringComparer.Ordinal);

    internal sealed class LocalSubscription
    {
        public required string Topic { get; init; }
        public required string TypeName { get; init; }
        public required QosProfile Qos { get; init; }
        public required Action<IRosMessage, MessageInfo> Deliver { get; init; }
    }

    public LocalSubscription Add(string topic, string typeName, QosProfile qos, Action<IRosMessage, MessageInfo> deliver)
    {
        var entry = new LocalSubscription { Topic = topic, TypeName = typeName, Qos = qos, Deliver = deliver };
        lock (_gate)
        {
            if (!_byTopic.TryGetValue(topic, out var list))
            {
                _byTopic[topic] = list = [];
            }

            // Copy-on-write so Publish can iterate without the lock.
            _byTopic[topic] = [.. list, entry];
        }

        return entry;
    }

    public void Remove(LocalSubscription entry)
    {
        lock (_gate)
        {
            if (_byTopic.TryGetValue(entry.Topic, out var list))
            {
                var next = list.Where(e => e != entry).ToList();
                if (next.Count == 0)
                {
                    _byTopic.Remove(entry.Topic);
                }
                else
                {
                    _byTopic[entry.Topic] = next;
                }
            }
        }
    }

    public int Count(string topic, string typeName, QosProfile publisherQos)
    {
        List<LocalSubscription>? list;
        lock (_gate)
        {
            _byTopic.TryGetValue(topic, out list);
        }

        return list?.Count(s => s.TypeName == typeName && QosMatching.IsCompatible(publisherQos, s.Qos)) ?? 0;
    }

    public void Publish(string topic, string typeName, QosProfile publisherQos, IRosMessage message, Gid publisherGid, long sequence)
    {
        List<LocalSubscription>? list;
        lock (_gate)
        {
            _byTopic.TryGetValue(topic, out list);
        }

        if (list is null)
        {
            return;
        }

        var now = RosTime.Now();
        var info = new MessageInfo(now, now, publisherGid, sequence, FromIntraProcess: true);
        foreach (var s in list)
        {
            if (s.TypeName == typeName && QosMatching.IsCompatible(publisherQos, s.Qos))
            {
                s.Deliver(message, info);
            }
        }
    }
}
