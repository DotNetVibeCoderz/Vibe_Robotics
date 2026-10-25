using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using ROS2.Net.Hosting;
using Shouldly;
using StdMsgs = ROS2.Net.Msgs.StdMsgs;

namespace ROS2.Net.Integration.Tests.Hosting;

public sealed class Greeter
{
    public string Greeting => "hello from DI";
}

public sealed class GreeterNode : Node
{
    public GreeterNode(Ros2Context context, NodeOptions options, Greeter greeter)
        : base(context, "greeter", options)
    {
        var pub = CreatePublisher<StdMsgs.String>("/greetings");
        CreateTimer(TimeSpan.FromMilliseconds(20), () => pub.Publish(new StdMsgs.String { Data = greeter.Greeting }));
    }
}

public class HostingTests
{
    private static int s_domain = 200;

    [Fact]
    public async Task Host_creates_nodes_from_di_and_spins_them()
    {
        var domain = Interlocked.Increment(ref s_domain);
        var builder = Host.CreateApplicationBuilder();
        builder.Services.AddSingleton<Greeter>();
        builder.Services.AddRos2(o =>
        {
            o.DomainId = domain;
            o.Rmw = "inprocess";
            o.Executor = Ros2ExecutorKind.MultiThreaded;
        });
        builder.Services.AddRos2Node<GreeterNode>(_ => new NodeOptions().WithParameter("rate", 5));
        var received = new TaskCompletionSource<string>(TaskCreationOptions.RunContinuationsAsynchronously);
        builder.Services.AddRos2Node("listener", (_, node) =>
            node.CreateSubscription<StdMsgs.String>("/greetings", m => received.TrySetResult(m.Data)));

        using var host = builder.Build();
        await host.StartAsync();

        (await received.Task.WaitAsync(TimeSpan.FromSeconds(5))).ShouldBe("hello from DI");
        var context = host.Services.GetRequiredService<Ros2Context>();
        context.DomainId.ShouldBe(domain);
        context.Nodes.Select(n => n.Name).ShouldBe(new[] { "greeter", "listener" }, ignoreOrder: true);
        host.Services.GetRequiredService<GreeterNode>().ParameterOverrides["rate"].AsInteger().ShouldBe(5);

        await host.StopAsync();
        context.IsShutdown.ShouldBeTrue();
    }

    [Fact]
    public async Task Options_bind_from_configuration()
    {
        var domain = Interlocked.Increment(ref s_domain);
        var config = new ConfigurationBuilder().AddInMemoryCollection(new Dictionary<string, string?>
        {
            ["Ros2:DomainId"] = domain.ToString(System.Globalization.CultureInfo.InvariantCulture),
            ["Ros2:Rmw"] = "inprocess",
            ["Ros2:Arguments:0"] = "-p",
            ["Ros2:Arguments:1"] = "speed:=2.5",
        }).Build();

        var services = new ServiceCollection();
        services.AddRos2(config.GetSection("Ros2"));
        await using var sp = services.BuildServiceProvider();
        var ctx = sp.GetRequiredService<Ros2Context>();

        ctx.DomainId.ShouldBe(domain);
        ctx.CreateNode("n").DeclareParameter("speed", 1.0).ShouldBe(2.5);
    }

    [Fact]
    public void Unknown_middleware_is_reported()
    {
        var services = new ServiceCollection();
        services.AddRos2(o => o.Rmw = "does-not-exist");
        using var sp = services.BuildServiceProvider();
        Should.Throw<InvalidOperationException>(() => sp.GetRequiredService<Ros2Context>()).Message.ShouldContain("does-not-exist");
    }

    [Fact]
    public async Task Context_shutdown_stops_the_application()
    {
        var domain = Interlocked.Increment(ref s_domain);
        var builder = Host.CreateApplicationBuilder();
        builder.Services.AddRos2(o =>
        {
            o.DomainId = domain;
            o.Rmw = "inprocess";
        });
        using var host = builder.Build();
        await host.StartAsync();
        var lifetime = host.Services.GetRequiredService<IHostApplicationLifetime>();
        var stopping = new TaskCompletionSource();
        lifetime.ApplicationStopping.Register(() => stopping.TrySetResult());

        host.Services.GetRequiredService<Ros2Context>().Shutdown("test");

        await stopping.Task.WaitAsync(TimeSpan.FromSeconds(5));
        await host.StopAsync();
    }
}
