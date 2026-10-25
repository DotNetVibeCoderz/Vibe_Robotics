// Ros2NetBlazor — a Blazor Server dashboard for ROS 2, built with ROS2.Net
// (made by Gravicode Studios, led by Kang Fadhil).
using ROS2.Net.Hosting;
using ROS2.Net.Rmw;
using ROS2.Net.Rmw.Rosbridge;
using ROS2.Net.Rmw.Rtps;
using Ros2NetBlazor.Components;
using Ros2NetBlazor.Services;

RmwRegistry.Register(new RtpsRmwFactory());
RmwRegistry.Register(new RosbridgeRmwFactory());

var builder = WebApplication.CreateBuilder(args);
builder.Services.AddRazorComponents().AddInteractiveServerComponents();

// One ROS context for the whole app, spun by a hosted service.
builder.Services.AddRos2(builder.Configuration.GetSection("Ros2"));
builder.Services.AddRos2Node<RobotLink>(); // also injectable into components as RobotLink

var app = builder.Build();
if (!app.Environment.IsDevelopment())
{
    app.UseExceptionHandler("/Error", createScopeForErrors: true);
}

app.UseAntiforgery();
app.MapStaticAssets();
app.MapRazorComponents<App>().AddInteractiveServerRenderMode();
app.Run();
