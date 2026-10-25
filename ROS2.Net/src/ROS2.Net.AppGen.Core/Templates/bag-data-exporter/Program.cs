// {{ProjectName}} — export a rosbag2 (MCAP) bag to JSONL and CSV (ROS2.Net template).
// Usage: dotnet run -- <bag directory or .mcap file> [output directory]
using ROS2.Net.Bag;

if (args.Length == 0)
{
    Console.WriteLine("usage: {{ProjectName}} <bag> [output-dir]");
    return 1;
}

var bagPath = args[0];
var output = args.Length > 1 ? args[1] : Path.Combine(Environment.CurrentDirectory, Path.GetFileName(Path.TrimEndingDirectorySeparator(bagPath)) + "_export");
Directory.CreateDirectory(output);

Console.WriteLine(BagInfo.Read(bagPath));
using var bag = BagReader.Open(bagPath);
var all = BagExporter.ToJsonLinesFile(bag, Path.Combine(output, "all.jsonl"));
Console.WriteLine($"all.jsonl: {all} messages");
foreach (var topic in bag.Topics)
{
    var file = Path.Combine(output, topic.Name.Trim('/').Replace('/', '_') + ".csv");
    try
    {
        var rows = BagExporter.ToCsvFile(bag, topic.Name, file);
        Console.WriteLine($"{topic.Name}: {rows} rows → {Path.GetFileName(file)}");
    }
    catch (Exception ex)
    {
        Console.WriteLine($"{topic.Name}: skipped ({ex.Message})");
    }
}

return 0;
