using Microsoft.AspNetCore.Components;
using ROS2.Net.Blazor;
using ROS2.Net.Gallery.Web.Services;

namespace ROS2.Net.Gallery.Web.Components.Shared;

/// <summary>Base of every demo page: language, the shared runtime, and a private ROS node.</summary>
public abstract class GalleryPage : RosComponentBase
{
    [Inject]
    protected Lang L { get; set; } = default!;

    [Inject]
    protected GalleryRuntime Runtime { get; set; } = default!;

    protected string T(string en, string id) => L.T(en, id);

    protected override void OnInitialized()
    {
        L.Changed += OnLanguageChanged;
        base.OnInitialized();
    }

    private void OnLanguageChanged() => InvokeAsync(StateHasChanged);

    protected override ValueTask DisposeCoreAsync()
    {
        L.Changed -= OnLanguageChanged;
        return base.DisposeCoreAsync();
    }
}
