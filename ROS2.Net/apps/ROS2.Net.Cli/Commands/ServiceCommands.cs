using System.CommandLine;
using System.Reflection;
using System.Text.Json;
using ROS2.Net.Serialization;

namespace ROS2.Net.Cli.Commands;

/// <summary>
/// <c>ros2net service ...</c>. Calls use the generated request/response types (resolved by name from
/// <see cref="RosTypeRegistry"/>), so any service type compiled into ROS2.Net — all standard
/// interfaces — can be called with a JSON/YAML request.
/// </summary>
internal static class ServiceCommands
{
    public static Command Build(CliEnvironment env)
    {
        var service = new Command("service", "Inspect and call services.");

        var showTypes = new Option<bool>("--show-types", "-t") { Description = "Show service types." };
        var hidden = new Option<bool>("--include-hidden-services") { Description = "Include hidden services." };
        var list = new Command("list", "List services.") { showTypes, hidden };
        list.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            foreach (var svc in s.Node.GetServiceNamesAndTypes(p.GetValue(hidden)))
            {
                env.Out.WriteLine(p.GetValue(showTypes) ? $"{svc.Name} [{string.Join(", ", svc.Types)}]" : svc.Name);
            }

            return 0;
        });

        var typeName = new Argument<string>("service_name");
        var type = new Command("type", "Print a service's type.") { typeName };
        type.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct);
            var t = await s.WaitForTopicTypeAsync(p.GetValue(typeName)!, TimeSpan.FromSeconds(1), ct, services: true)
                ?? throw new CliException($"Unknown service '{p.GetValue(typeName)}'.");
            env.Out.WriteLine(t);
            return 0;
        });

        var name = new Argument<string>("service_name");
        var svcType = new Argument<string?>("service_type") { Arity = ArgumentArity.ZeroOrOne, Description = "e.g. std_srvs/srv/SetBool (discovered when omitted)." };
        var values = new Argument<string?>("values") { Arity = ArgumentArity.ZeroOrOne, Description = "Request as YAML or JSON." };
        var timeout = new Option<double>("--timeout") { Description = "Seconds to wait for the service and the response.", DefaultValueFactory = _ => 10 };
        var call = new Command("call", "Call a service.") { name, svcType, values, timeout };
        call.Handle(env, async (p, ct) =>
        {
            await using var s = await env.OpenAsync(p, ct, waitForDiscovery: false);
            var serviceName = s.Node.ResolveServiceName(p.GetValue(name)!);
            var t = p.GetValue(svcType);
            var requestText = p.GetValue(values);

            // Allow `call /svc '{a: 1}'` without a type: the second positional is then the request.
            if (t is not null && (t.TrimStart().StartsWith('{') || !t.Contains('/')))
            {
                requestText = t;
                t = null;
            }

            t ??= await s.WaitForTopicTypeAsync(serviceName, TimeSpan.FromSeconds(p.GetValue(timeout)), ct, services: true)
                ?? throw new CliException($"Could not determine the type of '{serviceName}'. Pass it explicitly.");
            var response = await CallAsync(s.Node, serviceName, t, YamlJson.ToJson(requestText), TimeSpan.FromSeconds(p.GetValue(timeout)), env.Out, ct);
            env.Out.WriteLine("response:");
            env.Out.WriteLine(YamlJson.ToYaml(response));
            return 0;
        });

        service.Add(list);
        service.Add(type);
        service.Add(call);
        return service;
    }

    /// <summary>Calls a registry-known service with a JSON request and returns the JSON response.</summary>
    public static async Task<string> CallAsync(Node node, string serviceName, string serviceType, string requestJson, TimeSpan timeout, TextWriter output, CancellationToken ct)
    {
        var normalized = serviceType.Split('/') is { Length: 2 } parts ? $"{parts[0]}/srv/{parts[1]}" : serviceType;
        if (!RosTypeRegistry.TryGetService(normalized, out var info))
        {
            throw new CliException($"Service type '{serviceType}' is not known to ros2net (only compiled-in types can be called).");
        }

        var method = typeof(ServiceCommands).GetMethod(nameof(CallTypedAsync), BindingFlags.NonPublic | BindingFlags.Static)!
            .MakeGenericMethod(info.Request.ClrType, info.Response.ClrType);
        output.WriteLine($"requester: making request: {requestJson}");
        try
        {
            return await (Task<string>)method.Invoke(null, [node, serviceName, info, requestJson, timeout, ct])!;
        }
        catch (TargetInvocationException ex) when (ex.InnerException is not null)
        {
            throw ex.InnerException;
        }
    }

    private static async Task<string> CallTypedAsync<TRequest, TResponse>(Node node, string serviceName, ServiceTypeInfo info, string requestJson, TimeSpan timeout, CancellationToken ct)
        where TRequest : class, IRosMessage, new()
        where TResponse : class, IRosMessage, new()
    {
        using var client = node.CreateClientCore<TRequest, TResponse>(serviceName, info);
        if (!await client.WaitForServiceAsync(timeout, ct))
        {
            throw new CliException($"Service '{serviceName}' is not available.");
        }

        var request = new TRequest();
        using (var doc = JsonDocument.Parse(requestJson))
        {
            request.ReadJson(doc.RootElement);
        }

        var response = await client.CallAsync(request, timeout, ct);
        return RosJson.ToJsonString(response);
    }
}
