namespace ROS2.Net.Gallery.Web.Services;

/// <summary>Per-circuit UI language (English / Bahasa Indonesia).</summary>
public sealed class Lang
{
    public bool IsIndonesian { get; private set; }

    public string Code => IsIndonesian ? "id" : "en";

    public event Action? Changed;

    /// <summary>Picks the string for the current language.</summary>
    public string T(string en, string id) => IsIndonesian ? id : en;

    public void Set(bool indonesian)
    {
        if (IsIndonesian != indonesian)
        {
            IsIndonesian = indonesian;
            Changed?.Invoke();
        }
    }

    public void Toggle() => Set(!IsIndonesian);
}
