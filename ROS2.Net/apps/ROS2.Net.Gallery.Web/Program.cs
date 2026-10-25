using ROS2.Net.Blazor;
using ROS2.Net.Gallery.Web.Components;
using ROS2.Net.Gallery.Web.Services;

var builder = WebApplication.CreateBuilder(args);

builder.Services.AddRazorComponents().AddInteractiveServerComponents();

// Everything runs on the in-process middleware: the gallery needs no ROS 2 installation.
builder.Services.AddRos2Blazor(o =>
{
    o.Rmw = "inprocess";
    o.DomainId = 42;
    o.ThreadCount = 4;
});
builder.Services.AddSingleton<GalleryRuntime>();
builder.Services.AddHostedService(sp => sp.GetRequiredService<GalleryRuntime>());
builder.Services.AddScoped<Lang>();

var app = builder.Build();

if (!app.Environment.IsDevelopment())
{
    app.UseExceptionHandler("/error", createScopeForErrors: true);
}

app.UseAntiforgery();
app.MapStaticAssets();
app.MapRazorComponents<App>().AddInteractiveServerRenderMode();

app.Run();

/// <summary>Entry point marker for integration tests.</summary>
public partial class Program;
