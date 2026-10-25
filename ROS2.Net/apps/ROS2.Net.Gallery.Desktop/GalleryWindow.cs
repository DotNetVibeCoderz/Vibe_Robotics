using Avalonia;
using Avalonia.Controls;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using ROS2.Net.Avalonia;
using Ros2NetGallery.Pages;

namespace Ros2NetGallery;

/// <summary>
/// The gallery shell: a sidebar grouped by layer of the ROS stack, and a page area with the page's
/// interface chips, title, live demo and code.
/// </summary>
public sealed class GalleryWindow : Window
{
    private readonly GalleryRuntime _runtime;
    private readonly Func<GalleryPage>[] _factories;
    private readonly List<GalleryPage> _catalog;
    private GalleryPage? _current;
    private string _currentId = "home";
    private bool _showCode = true;

    public GalleryWindow(GalleryRuntime runtime)
    {
        _runtime = runtime;
        _factories = PageCatalog.Factories;
        _catalog = _factories.Select(f => f()).ToList(); // metadata only; Build is never called on these
        Title = "ROS2.Net Gallery";
        Width = 1440;
        Height = 900;
        MinWidth = 1000;
        MinHeight = 640;
        try
        {
            Icon = new WindowIcon(AssetLoader.Open(new Uri("avares://ROS2.Net.Gallery.Desktop/Assets/icon-128.png")));
        }
        catch (Exception)
        {
            // Icon is cosmetic.
        }

        GalleryNavigation.Navigate = id => global::Avalonia.Threading.Dispatcher.UIThread.Post(() => Navigate(id));
        Rebuild();
    }

    public IReadOnlyList<string> PageIds => _catalog.Select(p => p.Id).ToList();

    public GalleryPage? CurrentPage => _current;

    /// <summary>Opens a page by id.</summary>
    public void Navigate(string id)
    {
        _currentId = id;
        Rebuild();
    }

    public void SetLanguage(bool indonesian)
    {
        Loc.IsIndonesian = indonesian;
        Rebuild();
    }

    public void SetDark(bool dark)
    {
        RosTheme.IsDark = dark;
        Rebuild();
    }

    public void SetShowCode(bool show)
    {
        _showCode = show;
        Rebuild();
    }

    private void Rebuild()
    {
        _current?.Dispose();
        var factory = _factories[Math.Max(0, _catalog.FindIndex(p => p.Id == _currentId))];
        _current = factory();
        var p = RosTheme.Current;
        Background = new SolidColorBrush(p.Background);

        Control body;
        try
        {
            body = BuildPage(_current);
        }
        catch (Exception ex)
        {
            body = Ui.Card(Ui.Text($"This demo could not start: {ex.Message}", color: p.Fault));
        }

        Content = Ui.Columns("264,*", 0, BuildSidebar(), body);
    }

    private Control BuildSidebar()
    {
        var p = RosTheme.Current;
        var items = new StackPanel { Spacing = 2 };

        // Brand.
        Control logo;
        try
        {
            logo = new Image { Source = new Bitmap(AssetLoader.Open(new Uri("avares://ROS2.Net.Gallery.Desktop/Assets/icon-128.png"))), Width = 36, Height = 36 };
        }
        catch (Exception)
        {
            logo = new Border { Width = 36, Height = 36, CornerRadius = new CornerRadius(9), Background = p.SignalGradient() };
        }

        items.Children.Add(Ui.Row(12, logo, Ui.Col(0,
            Ui.Text("ROS2.Net", 17, FontWeight.Bold, wrap: false),
            Ui.Text("Gallery", 12, color: p.Haze, wrap: false))));
        items.Children.Add(new Border { Height = 18 });

        foreach (var group in Enum.GetValues<PageGroup>())
        {
            var pages = _catalog.Where(x => x.Group == group).ToList();
            if (pages.Count == 0)
            {
                continue;
            }

            var label = group switch
            {
                PageGroup.Communication => Loc.Communication,
                PageGroup.Robotics => Loc.Robotics,
                PageGroup.Intelligence => Loc.Intelligence,
                PageGroup.Tools => Loc.Tools,
                _ => Loc.Overview,
            };
            items.Children.Add(new TextBlock
            {
                Text = label.ToString().ToUpperInvariant(),
                FontSize = 10.5,
                FontWeight = FontWeight.SemiBold,
                LetterSpacing = 1.2,
                Foreground = new SolidColorBrush(p.Haze),
                Margin = new Thickness(10, 14, 0, 6),
            });
            foreach (var page in pages)
            {
                items.Children.Add(NavItem(page));
            }
        }

        var lang = Ui.Button(Loc.IsIndonesian ? "EN · ID ●" : "● EN · ID", () => SetLanguage(!Loc.IsIndonesian));
        var theme = Ui.Button(RosTheme.IsDark ? Loc.T("Light", "Terang") : Loc.T("Dark", "Gelap"), () => SetDark(!RosTheme.IsDark));
        var footer = Ui.Col(10,
            Ui.Row(8, lang, theme),
            Ui.Muted(Loc.Credit, 11));

        var dock = new DockPanel { LastChildFill = true };
        dock.Children.Add(Ui.Dock(footer, Dock.Bottom));
        dock.Children.Add(new ScrollViewer { Content = items, Margin = new Thickness(0, 0, 0, 12) });

        return new Border
        {
            Background = new SolidColorBrush(p.Surface),
            BorderBrush = new SolidColorBrush(p.Seam),
            BorderThickness = new Thickness(0, 0, 1, 0),
            Padding = new Thickness(16, 20, 16, 18),
            Child = dock,
        };
    }

    private Control NavItem(GalleryPage page)
    {
        var p = RosTheme.Current;
        var selected = page.Id == _currentId;
        var text = new TextBlock
        {
            Text = page.Title,
            FontSize = 13,
            FontWeight = selected ? FontWeight.SemiBold : FontWeight.Normal,
            Foreground = new SolidColorBrush(selected ? p.Ink : p.Haze),
            VerticalAlignment = VerticalAlignment.Center,
        };
        var bar = new Border { Width = 3, CornerRadius = new CornerRadius(2), Background = selected ? p.SignalGradient(new RelativePoint(0, 0, RelativeUnit.Relative), new RelativePoint(0, 1, RelativeUnit.Relative)) : Brushes.Transparent, Margin = new Thickness(0, 4, 10, 4) };
        var button = new Button
        {
            HorizontalAlignment = HorizontalAlignment.Stretch,
            HorizontalContentAlignment = HorizontalAlignment.Left,
            Background = selected ? new SolidColorBrush(p.SurfaceRaised) : Brushes.Transparent,
            BorderThickness = new Thickness(0),
            CornerRadius = new CornerRadius(8),
            Padding = new Thickness(4, 6),
            Content = new DockPanel { Children = { Ui.Dock(bar, Dock.Left), text } },
        };
        button.Click += (_, _) => Navigate(page.Id);
        return button;
    }

    private Control BuildPage(GalleryPage page)
    {
        var p = RosTheme.Current;
        var chips = new WrapPanel { Orientation = Orientation.Horizontal };
        foreach (var i in page.Interfaces)
        {
            var chip = Ui.Chip(i, i.StartsWith('/') ? p.Signal : p.Telemetry);
            chip.Margin = new Thickness(0, 0, 6, 6);
            chips.Children.Add(chip);
        }

        var codeToggle = Ui.Button(_showCode ? Loc.T("Hide code", "Sembunyikan kode") : Loc.T("Show code", "Tampilkan kode"), () => SetShowCode(!_showCode));
        var titleRow = new DockPanel
        {
            Children =
            {
                Ui.Dock(page.Code.Length > 0 ? codeToggle : (Control)new Border(), Dock.Right),
                Ui.Text(page.Title, 30, FontWeight.Bold, wrap: false),
            },
        };

        var header = Ui.Col(8,
            chips,
            titleRow,
            new TextBlock { Text = page.Summary, FontSize = 14, TextWrapping = TextWrapping.Wrap, MaxWidth = 900, HorizontalAlignment = HorizontalAlignment.Left, Foreground = new SolidColorBrush(p.Haze) },
            new Border { Height = 6 },
            Ui.SignalTrace());

        var demo = page.Build(_runtime);
        Control content = demo;
        if (_showCode && page.Code.Length > 0)
        {
            var code = Ui.Card(Ui.Code(page.Code), Loc.ViewCode);
            content = Ui.Columns("13*,7*", 18, demo, code);
        }

        var root = new DockPanel { Margin = new Thickness(32, 24, 32, 24) };
        root.Children.Add(Ui.Dock(new Border { Child = header, Margin = new Thickness(0, 0, 0, 20) }, Dock.Top));
        root.Children.Add(content);
        return root;
    }

    protected override void OnClosed(EventArgs e)
    {
        _current?.Dispose();
        base.OnClosed(e);
    }
}
