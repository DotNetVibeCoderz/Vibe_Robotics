namespace Ros2NetGallery;

/// <summary>A string in both supported languages.</summary>
public readonly record struct T2(string En, string Id)
{
    public override string ToString() => Loc.IsIndonesian ? Id : En;

    public static implicit operator string(T2 t) => t.ToString();
}

/// <summary>Gallery language state (English / Bahasa Indonesia).</summary>
public static class Loc
{
    public static bool IsIndonesian { get; set; }

    public static string Code => IsIndonesian ? "ID" : "EN";

    public static string T(string en, string id) => IsIndonesian ? id : en;

    public static readonly T2 Credit = new("Made by Gravicode Studios, led by Kang Fadhil", "Dibuat oleh Gravicode Studios dipimpin oleh Kang Fadhil");
    public static readonly T2 ViewCode = new("Code", "Kode");
    public static readonly T2 LiveDemo = new("Live demo", "Demo langsung");
    public static readonly T2 Communication = new("Communication", "Komunikasi");
    public static readonly T2 Robotics = new("Robotics", "Robotika");
    public static readonly T2 Intelligence = new("Intelligence", "Kecerdasan");
    public static readonly T2 Tools = new("Tools", "Perkakas");
    public static readonly T2 Overview = new("Overview", "Ringkasan");
}
