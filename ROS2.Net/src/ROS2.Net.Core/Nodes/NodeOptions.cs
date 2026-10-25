namespace ROS2.Net;

/// <summary>Options for creating a <see cref="Node"/>, mirroring <c>rclcpp::NodeOptions</c>.</summary>
public sealed class NodeOptions
{
    /// <summary>Node namespace, e.g. <c>/robot1</c>. Relative names are made absolute.</summary>
    public string? Namespace { get; set; }

    /// <summary>Node-specific remapping rules (applied before the context's global rules).</summary>
    public List<RemapRule> Remappings { get; } = [];

    /// <summary>Parameter values that take precedence over declared defaults.</summary>
    public List<Parameter> ParameterOverrides { get; } = [];

    /// <summary>Node-specific <c>--ros-args</c>.</summary>
    public List<string> Arguments { get; } = [];

    /// <summary>Apply the context's command-line remaps and parameters to this node.</summary>
    public bool UseGlobalArguments { get; set; } = true;

    /// <summary>Expose <c>~/get_parameters</c>, <c>~/set_parameters</c> etc. so tools can manage parameters remotely.</summary>
    public bool StartParameterServices { get; set; } = true;

    /// <summary>Publish changes on <c>/parameter_events</c>.</summary>
    public bool StartParameterEventPublisher { get; set; } = true;

    /// <summary>Also publish log output on <c>/rosout</c>.</summary>
    public bool EnableRosout { get; set; } = true;

    /// <summary>Allow setting parameters that were never declared.</summary>
    public bool AllowUndeclaredParameters { get; set; }

    /// <summary>Declare every override automatically (useful for generic nodes driven by YAML).</summary>
    public bool AutomaticallyDeclareParametersFromOverrides { get; set; }

    /// <summary>Override the context's intra-process setting for entities of this node.</summary>
    public bool? UseIntraProcessComms { get; set; }

    public QosProfile ParameterEventQos { get; set; } = QosProfile.ParameterEvents;

    public NodeOptions WithNamespace(string ns)
    {
        Namespace = ns;
        return this;
    }

    public NodeOptions WithParameter(string name, object? value)
    {
        ParameterOverrides.Add(new Parameter(name, value));
        return this;
    }

    public NodeOptions WithRemap(string from, string to)
    {
        Remappings.Add(new RemapRule(from, to));
        return this;
    }
}
