namespace ROS2.Net.AppGen.UI.Tests;

/// <summary>Headless Avalonia runs one platform per process; serialize the UI tests.</summary>
[CollectionDefinition("headless", DisableParallelization = true)]
public sealed class HeadlessCollection;
