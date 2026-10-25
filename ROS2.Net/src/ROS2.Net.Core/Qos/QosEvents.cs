namespace ROS2.Net;

/// <summary>Kinds of QoS events, mirroring <c>rmw_event_type_t</c>.</summary>
public enum QosEventKind
{
    /// <summary>A subscription did not receive a sample within its deadline.</summary>
    RequestedDeadlineMissed,
    /// <summary>A publisher did not publish within its deadline.</summary>
    OfferedDeadlineMissed,
    /// <summary>The number of alive publishers seen by a subscription changed.</summary>
    LivelinessChanged,
    /// <summary>A manual-liveliness publisher failed to assert liveliness within its lease.</summary>
    LivelinessLost,
    /// <summary>A publisher was found whose QoS is incompatible with this subscription.</summary>
    RequestedIncompatibleQos,
    /// <summary>A subscription was found whose QoS is incompatible with this publisher.</summary>
    OfferedIncompatibleQos,
    /// <summary>Samples were dropped because the subscription queue overflowed.</summary>
    MessageLost,
    /// <summary>The number of matched endpoints changed.</summary>
    Matched,
}

/// <summary>A QoS event raised on a publisher or subscription.</summary>
/// <param name="Kind">Event kind.</param>
/// <param name="TotalCount">Cumulative count since the endpoint was created.</param>
/// <param name="TotalCountChange">Change since the previous event of this kind.</param>
/// <param name="CurrentCount">Current value where meaningful (alive publishers, matched endpoints), else 0.</param>
/// <param name="Detail">Human-readable detail, e.g. the incompatible policy.</param>
public readonly record struct QosEvent(QosEventKind Kind, int TotalCount, int TotalCountChange, int CurrentCount = 0, string? Detail = null);
