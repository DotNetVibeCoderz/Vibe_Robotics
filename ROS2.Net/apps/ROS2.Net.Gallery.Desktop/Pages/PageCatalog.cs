namespace Ros2NetGallery.Pages;

/// <summary>Every gallery page, in sidebar order.</summary>
public static class PageCatalog
{
    public static Func<GalleryPage>[] Factories { get; } =
    [
        () => new HomePage(),
        () => new TalkerListenerPage(),
        () => new ServicesPage(),
        () => new ActionsPage(),
        () => new ParametersPage(),
        () => new LifecyclePage(),
        () => new TeleopPage(),
        () => new LidarPage(),
        () => new NavigationPage(),
        () => new TfPage(),
        () => new ControlPage(),
        () => new ArmPage(),
        () => new PerceptionPage(),
        () => new RlPage(),
        () => new CommanderPage(),
        () => new BagPage(),
        () => new ConnectivityPage(),
    ];
}
