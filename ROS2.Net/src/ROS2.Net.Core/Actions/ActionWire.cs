using System.Text.Json;
using ROS2.Net.Msgs.BuiltinInterfaces;
using ROS2.Net.Msgs.UniqueIdentifierMsgs;
using ROS2.Net.Serialization;

namespace ROS2.Net;

/// <summary>
/// The wire structures of the ROS 2 action protocol. They are generic over the user's goal, result
/// and feedback types; their ROS type names (<c>pkg/action/Name_SendGoal_Request</c>, ...) are supplied
/// at runtime by <see cref="ActionTypes"/>.
/// </summary>
internal static class ActionTypes
{
    public static ServiceTypeInfo SendGoal<TGoal>(string action)
        where TGoal : class, IRosMessage<TGoal>, new() =>
        new(action + "_SendGoal",
            MessageTypeInfo.Create(action + "_SendGoal_Request", typeof(SendGoalRequest<TGoal>), static () => new SendGoalRequest<TGoal>(),
                "unique_identifier_msgs/UUID goal_id\n" + TGoal.Definition, ["unique_identifier_msgs/msg/UUID", TGoal.TypeName]),
            MessageTypeInfo.Create(action + "_SendGoal_Response", typeof(SendGoalResponse), static () => new SendGoalResponse(),
                "bool accepted\nbuiltin_interfaces/Time stamp\n", ["builtin_interfaces/msg/Time"]));

    public static ServiceTypeInfo GetResult<TResult>(string action)
        where TResult : class, IRosMessage<TResult>, new() =>
        new(action + "_GetResult",
            MessageTypeInfo.Create(action + "_GetResult_Request", typeof(GetResultRequest), static () => new GetResultRequest(),
                "unique_identifier_msgs/UUID goal_id\n", ["unique_identifier_msgs/msg/UUID"]),
            MessageTypeInfo.Create(action + "_GetResult_Response", typeof(GetResultResponse<TResult>), static () => new GetResultResponse<TResult>(),
                "int8 status\n" + TResult.Definition, [TResult.TypeName]));

    public static MessageTypeInfo Feedback<TFeedback>(string action)
        where TFeedback : class, IRosMessage<TFeedback>, new() =>
        MessageTypeInfo.Create(action + "_FeedbackMessage", typeof(FeedbackMessage<TFeedback>), static () => new FeedbackMessage<TFeedback>(),
            "unique_identifier_msgs/UUID goal_id\n" + TFeedback.Definition, ["unique_identifier_msgs/msg/UUID", TFeedback.TypeName]);

    public static ServiceTypeInfo CancelGoal() =>
        new(Msgs.ActionMsgs.CancelGoal.TypeName, MessageTypeInfo.Of<Msgs.ActionMsgs.CancelGoalRequest>(), MessageTypeInfo.Of<Msgs.ActionMsgs.CancelGoalResponse>());

    public static UUID ToUuid(Guid id)
    {
        var bytes = new byte[16];
        id.TryWriteBytes(bytes, bigEndian: true, out _);
        return new UUID { Uuid = bytes };
    }

    public static Guid FromUuid(UUID uuid) => uuid.Uuid.Length == 16 ? new Guid(uuid.Uuid, bigEndian: true) : Guid.Empty;
}

/// <summary><c>Name_SendGoal_Request</c>: goal id + goal. Instance type names are placeholders; middlewares use the spec's <see cref="MessageTypeInfo"/>.</summary>
internal sealed class SendGoalRequest<TGoal> : IRosMessage
    where TGoal : class, IRosMessage<TGoal>, new()
{
    public UUID GoalId { get; set; } = new();

    public TGoal Goal { get; set; } = new();

    public string RosTypeName => "action/SendGoal_Request";

    public void Serialize(CdrWriter writer)
    {
        GoalId.Serialize(writer);
        Goal.Serialize(writer);
    }

    public void Deserialize(ref CdrReader reader)
    {
        GoalId.Deserialize(ref reader);
        Goal.Deserialize(ref reader);
    }

    public void WriteJson(Utf8JsonWriter writer)
    {
        writer.WriteStartObject();
        writer.WritePropertyName("goal_id");
        GoalId.WriteJson(writer);
        writer.WritePropertyName("goal");
        Goal.WriteJson(writer);
        writer.WriteEndObject();
    }

    public void ReadJson(JsonElement element)
    {
        if (element.TryGetProperty("goal_id", out var id))
        {
            GoalId.ReadJson(id);
        }

        if (element.TryGetProperty("goal", out var g))
        {
            Goal.ReadJson(g);
        }
    }

    public IRosMessage CloneMessage() => new SendGoalRequest<TGoal> { GoalId = GoalId.Clone(), Goal = (TGoal)Goal.CloneMessage() };
}

/// <summary><c>Name_SendGoal_Response</c>: accepted + stamp.</summary>
internal sealed class SendGoalResponse : IRosMessage
{
    public bool Accepted { get; set; }

    public Time Stamp { get; set; } = new();

    public string RosTypeName => "action/SendGoal_Response";

    public void Serialize(CdrWriter writer)
    {
        writer.WriteBool(Accepted);
        Stamp.Serialize(writer);
    }

    public void Deserialize(ref CdrReader reader)
    {
        Accepted = reader.ReadBool();
        Stamp.Deserialize(ref reader);
    }

    public void WriteJson(Utf8JsonWriter writer)
    {
        writer.WriteStartObject();
        writer.WriteBoolean("accepted", Accepted);
        writer.WritePropertyName("stamp");
        Stamp.WriteJson(writer);
        writer.WriteEndObject();
    }

    public void ReadJson(JsonElement element)
    {
        if (element.TryGetProperty("accepted", out var a))
        {
            Accepted = RosJson.ReadBool(a);
        }

        if (element.TryGetProperty("stamp", out var s))
        {
            Stamp.ReadJson(s);
        }
    }

    public IRosMessage CloneMessage() => new SendGoalResponse { Accepted = Accepted, Stamp = Stamp.Clone() };
}

/// <summary><c>Name_GetResult_Request</c>: goal id.</summary>
internal sealed class GetResultRequest : IRosMessage
{
    public UUID GoalId { get; set; } = new();

    public string RosTypeName => "action/GetResult_Request";

    public void Serialize(CdrWriter writer) => GoalId.Serialize(writer);

    public void Deserialize(ref CdrReader reader) => GoalId.Deserialize(ref reader);

    public void WriteJson(Utf8JsonWriter writer)
    {
        writer.WriteStartObject();
        writer.WritePropertyName("goal_id");
        GoalId.WriteJson(writer);
        writer.WriteEndObject();
    }

    public void ReadJson(JsonElement element)
    {
        if (element.TryGetProperty("goal_id", out var id))
        {
            GoalId.ReadJson(id);
        }
    }

    public IRosMessage CloneMessage() => new GetResultRequest { GoalId = GoalId.Clone() };
}

/// <summary><c>Name_GetResult_Response</c>: status + result.</summary>
internal sealed class GetResultResponse<TResult> : IRosMessage
    where TResult : class, IRosMessage<TResult>, new()
{
    public sbyte Status { get; set; }

    public TResult Result { get; set; } = new();

    public string RosTypeName => "action/GetResult_Response";

    public void Serialize(CdrWriter writer)
    {
        writer.WriteSByte(Status);
        Result.Serialize(writer);
    }

    public void Deserialize(ref CdrReader reader)
    {
        Status = reader.ReadSByte();
        Result.Deserialize(ref reader);
    }

    public void WriteJson(Utf8JsonWriter writer)
    {
        writer.WriteStartObject();
        writer.WriteNumber("status", Status);
        writer.WritePropertyName("result");
        Result.WriteJson(writer);
        writer.WriteEndObject();
    }

    public void ReadJson(JsonElement element)
    {
        if (element.TryGetProperty("status", out var s))
        {
            Status = RosJson.ReadSByte(s);
        }

        if (element.TryGetProperty("result", out var r))
        {
            Result.ReadJson(r);
        }
    }

    public IRosMessage CloneMessage() => new GetResultResponse<TResult> { Status = Status, Result = (TResult)Result.CloneMessage() };
}

/// <summary><c>Name_FeedbackMessage</c>: goal id + feedback.</summary>
internal sealed class FeedbackMessage<TFeedback> : IRosMessage
    where TFeedback : class, IRosMessage<TFeedback>, new()
{
    public UUID GoalId { get; set; } = new();

    public TFeedback Feedback { get; set; } = new();

    public string RosTypeName => "action/FeedbackMessage";

    public void Serialize(CdrWriter writer)
    {
        GoalId.Serialize(writer);
        Feedback.Serialize(writer);
    }

    public void Deserialize(ref CdrReader reader)
    {
        GoalId.Deserialize(ref reader);
        Feedback.Deserialize(ref reader);
    }

    public void WriteJson(Utf8JsonWriter writer)
    {
        writer.WriteStartObject();
        writer.WritePropertyName("goal_id");
        GoalId.WriteJson(writer);
        writer.WritePropertyName("feedback");
        Feedback.WriteJson(writer);
        writer.WriteEndObject();
    }

    public void ReadJson(JsonElement element)
    {
        if (element.TryGetProperty("goal_id", out var id))
        {
            GoalId.ReadJson(id);
        }

        if (element.TryGetProperty("feedback", out var f))
        {
            Feedback.ReadJson(f);
        }
    }

    public IRosMessage CloneMessage() => new FeedbackMessage<TFeedback> { GoalId = GoalId.Clone(), Feedback = (TFeedback)Feedback.CloneMessage() };
}
