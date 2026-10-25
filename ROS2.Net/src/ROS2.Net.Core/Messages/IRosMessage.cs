using System.Text.Json;
using ROS2.Net.Serialization;

namespace ROS2.Net;

/// <summary>
/// A ROS 2 message (or the request/response/goal/result/feedback part of a service or action).
/// Implemented by code generated from <c>.msg</c>, <c>.srv</c> and <c>.action</c> files.
/// </summary>
public interface IRosMessage
{
    /// <summary>The ROS 2 type name, e.g. <c>std_msgs/msg/String</c>.</summary>
    string RosTypeName { get; }

    /// <summary>Encodes the message as CDR. The writer has already emitted the encapsulation header.</summary>
    void Serialize(CdrWriter writer);

    /// <summary>Decodes the message from CDR, reusing existing arrays and nested messages where possible.</summary>
    void Deserialize(ref CdrReader reader);

    /// <summary>Writes the message as a rosbridge-compatible JSON object.</summary>
    void WriteJson(Utf8JsonWriter writer);

    /// <summary>Populates the message from a JSON object. Missing fields are left unchanged.</summary>
    void ReadJson(JsonElement element);

    /// <summary>Returns a deep copy.</summary>
    IRosMessage CloneMessage();
}

/// <summary>Statically typed view of a generated message, carrying its type metadata.</summary>
public interface IRosMessage<TSelf> : IRosMessage
    where TSelf : class, IRosMessage<TSelf>, new()
{
    /// <summary>The ROS 2 type name, e.g. <c>geometry_msgs/msg/Twist</c>.</summary>
    static abstract string TypeName { get; }

    /// <summary>The DDS type name, e.g. <c>geometry_msgs::msg::dds_::Twist_</c>.</summary>
    static abstract string DdsTypeName { get; }

    /// <summary>The interface definition text the type was generated from.</summary>
    static abstract string Definition { get; }

    /// <summary>Full type names (<c>pkg/msg/Name</c>) of the nested message types.</summary>
    static abstract IReadOnlyList<string> Dependencies { get; }
}

/// <summary>Implemented by a service request type; ties it to its response and to the service's type name.</summary>
public interface IRosServiceRequest<TRequest, TResponse>
    where TRequest : class, IRosMessage<TRequest>, IRosServiceRequest<TRequest, TResponse>, new()
    where TResponse : class, IRosMessage<TResponse>, new()
{
    /// <summary>The service type name, e.g. <c>std_srvs/srv/SetBool</c>.</summary>
    static abstract string ServiceTypeName { get; }
}

/// <summary>Implemented by an action goal type; ties it to the result and feedback types and the action's type name.</summary>
public interface IRosActionGoal<TGoal, TResult, TFeedback>
    where TGoal : class, IRosMessage<TGoal>, IRosActionGoal<TGoal, TResult, TFeedback>, new()
    where TResult : class, IRosMessage<TResult>, new()
    where TFeedback : class, IRosMessage<TFeedback>, new()
{
    /// <summary>The action type name, e.g. <c>example_interfaces/action/Fibonacci</c>.</summary>
    static abstract string ActionTypeName { get; }
}
