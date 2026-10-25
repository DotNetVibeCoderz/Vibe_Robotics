using ROS2.Net.Msgs.RclInterfaces;

namespace ROS2.Net;

/// <summary>
/// The six standard parameter services (<c>~/get_parameters</c>, <c>~/set_parameters</c>, ...),
/// which let <c>ros2 param</c>, <c>rqt_reconfigure</c> and <see cref="ParameterClient"/> manage a node remotely.
/// </summary>
internal static class ParameterServices
{
    public static void Install(Node node)
    {
        var qos = QosProfile.Parameters;

        node.CreateService<GetParametersRequest, GetParametersResponse>("~/get_parameters", req => new GetParametersResponse
        {
            Values = req.Names.Select(n => node.TryGetParameter(n, out var v) ? v.ToMsg() : ParameterValue.NotSet.ToMsg()).ToArray(),
        }, qos);

        node.CreateService<GetParameterTypesRequest, GetParameterTypesResponse>("~/get_parameter_types", req => new GetParameterTypesResponse
        {
            Types = req.Names.Select(n => node.TryGetParameter(n, out var v) ? (byte)v.Type : (byte)0).ToArray(),
        }, qos);

        node.CreateService<SetParametersRequest, SetParametersResponse>("~/set_parameters", req => new SetParametersResponse
        {
            Results = node.SetParameters(req.Parameters.Select(Parameter.FromMsg))
                .Select(r => new Msgs.RclInterfaces.SetParametersResult { Successful = r.Successful, Reason = r.Reason })
                .ToArray(),
        }, qos);

        node.CreateService<SetParametersAtomicallyRequest, SetParametersAtomicallyResponse>("~/set_parameters_atomically", req =>
        {
            var r = node.SetParametersAtomically(req.Parameters.Select(Parameter.FromMsg));
            return new SetParametersAtomicallyResponse { Result = new Msgs.RclInterfaces.SetParametersResult { Successful = r.Successful, Reason = r.Reason } };
        }, qos);

        node.CreateService<DescribeParametersRequest, DescribeParametersResponse>("~/describe_parameters", req => new DescribeParametersResponse
        {
            Descriptors = req.Names
                .Select(n => node.HasParameter(n) ? node.DescribeParameter(n).ToMsg() : new Msgs.RclInterfaces.ParameterDescriptor { Name = n })
                .ToArray(),
        }, qos);

        node.CreateService<ListParametersRequest, ListParametersResponse>("~/list_parameters", req =>
        {
            var (names, prefixes) = node.ListParameters(req.Prefixes, req.Depth);
            return new ListParametersResponse { Result = new ListParametersResult { Names = names.ToArray(), Prefixes = prefixes.ToArray() } };
        }, qos);
    }
}

/// <summary>Reads and writes the parameters of another node through its parameter services.</summary>
public sealed class ParameterClient : IDisposable
{
    private readonly ServiceClient<GetParametersRequest, GetParametersResponse> _get;
    private readonly ServiceClient<SetParametersRequest, SetParametersResponse> _set;
    private readonly ServiceClient<SetParametersAtomicallyRequest, SetParametersAtomicallyResponse> _setAtomic;
    private readonly ServiceClient<ListParametersRequest, ListParametersResponse> _list;
    private readonly ServiceClient<DescribeParametersRequest, DescribeParametersResponse> _describe;
    private readonly ServiceClient<GetParameterTypesRequest, GetParameterTypesResponse> _types;

    /// <param name="node">Local node used to make the calls.</param>
    /// <param name="remoteNode">Fully qualified name of the target node, e.g. <c>/robot1/controller</c>.</param>
    public ParameterClient(Node node, string remoteNode)
    {
        RemoteNode = remoteNode.StartsWith('/') ? remoteNode : "/" + remoteNode;
        _get = node.CreateClient<GetParametersRequest, GetParametersResponse>(RemoteNode + "/get_parameters", QosProfile.Parameters);
        _set = node.CreateClient<SetParametersRequest, SetParametersResponse>(RemoteNode + "/set_parameters", QosProfile.Parameters);
        _setAtomic = node.CreateClient<SetParametersAtomicallyRequest, SetParametersAtomicallyResponse>(RemoteNode + "/set_parameters_atomically", QosProfile.Parameters);
        _list = node.CreateClient<ListParametersRequest, ListParametersResponse>(RemoteNode + "/list_parameters", QosProfile.Parameters);
        _describe = node.CreateClient<DescribeParametersRequest, DescribeParametersResponse>(RemoteNode + "/describe_parameters", QosProfile.Parameters);
        _types = node.CreateClient<GetParameterTypesRequest, GetParameterTypesResponse>(RemoteNode + "/get_parameter_types", QosProfile.Parameters);
    }

    public string RemoteNode { get; }

    public TimeSpan Timeout { get; set; } = TimeSpan.FromSeconds(5);

    public Task<bool> WaitForServiceAsync(TimeSpan? timeout = null, CancellationToken cancellationToken = default) =>
        _get.WaitForServiceAsync(timeout, cancellationToken);

    public async Task<IReadOnlyList<Parameter>> GetAsync(IEnumerable<string> names, CancellationToken cancellationToken = default)
    {
        var list = names.ToArray();
        var res = await _get.CallAsync(new GetParametersRequest { Names = list }, Timeout, cancellationToken).ConfigureAwait(false);
        return list.Zip(res.Values, (n, v) => new Parameter(n, ParameterValue.FromMsg(v))).ToList();
    }

    public async Task<ParameterValue> GetAsync(string name, CancellationToken cancellationToken = default) =>
        (await GetAsync([name], cancellationToken).ConfigureAwait(false))[0].Value;

    public async Task<IReadOnlyList<SetParametersResult>> SetAsync(IEnumerable<Parameter> parameters, CancellationToken cancellationToken = default)
    {
        var res = await _set.CallAsync(new SetParametersRequest { Parameters = parameters.Select(p => p.ToMsg()).ToArray() }, Timeout, cancellationToken).ConfigureAwait(false);
        return res.Results.Select(r => new SetParametersResult(r.Successful, r.Reason)).ToList();
    }

    public async Task<SetParametersResult> SetAsync(string name, object? value, CancellationToken cancellationToken = default) =>
        (await SetAsync([new Parameter(name, value)], cancellationToken).ConfigureAwait(false))[0];

    public async Task<SetParametersResult> SetAtomicallyAsync(IEnumerable<Parameter> parameters, CancellationToken cancellationToken = default)
    {
        var res = await _setAtomic.CallAsync(new SetParametersAtomicallyRequest { Parameters = parameters.Select(p => p.ToMsg()).ToArray() }, Timeout, cancellationToken).ConfigureAwait(false);
        return new SetParametersResult(res.Result.Successful, res.Result.Reason);
    }

    public async Task<IReadOnlyList<string>> ListAsync(IEnumerable<string>? prefixes = null, ulong depth = 0, CancellationToken cancellationToken = default)
    {
        var res = await _list.CallAsync(new ListParametersRequest { Prefixes = prefixes?.ToArray() ?? [], Depth = depth }, Timeout, cancellationToken).ConfigureAwait(false);
        return res.Result.Names;
    }

    public async Task<IReadOnlyList<ParameterDescriptor>> DescribeAsync(IEnumerable<string> names, CancellationToken cancellationToken = default)
    {
        var res = await _describe.CallAsync(new DescribeParametersRequest { Names = names.ToArray() }, Timeout, cancellationToken).ConfigureAwait(false);
        return res.Descriptors.Select(ParameterDescriptor.FromMsg).ToList();
    }

    public async Task<IReadOnlyList<ParameterType>> GetTypesAsync(IEnumerable<string> names, CancellationToken cancellationToken = default)
    {
        var res = await _types.CallAsync(new GetParameterTypesRequest { Names = names.ToArray() }, Timeout, cancellationToken).ConfigureAwait(false);
        return res.Types.Select(t => (ParameterType)t).ToList();
    }

    public void Dispose()
    {
        _get.Dispose();
        _set.Dispose();
        _setAtomic.Dispose();
        _list.Dispose();
        _describe.Dispose();
        _types.Dispose();
    }
}
