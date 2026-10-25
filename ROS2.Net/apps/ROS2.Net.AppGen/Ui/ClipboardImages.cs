using Avalonia.Input.Platform;
using Avalonia.Media.Imaging;

namespace Ros2AppGen.Ui;

/// <summary>Clipboard helpers (Avalonia 12 data-transfer API).</summary>
public static class ClipboardImages
{
    public static Task<Bitmap?> TryGetBitmapAsync(IClipboard clipboard) => clipboard.TryGetBitmapAsync();

    public static Task SetTextAsync(IClipboard clipboard, string text) => clipboard.SetTextAsync(text);
}
