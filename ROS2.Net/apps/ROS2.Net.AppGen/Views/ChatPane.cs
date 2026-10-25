using System.Collections.Specialized;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform.Storage;
using Avalonia.Threading;
using ROS2.Net.AppGen;
using Ros2AppGen.Ui;
using Ros2AppGen.ViewModels;

namespace Ros2AppGen.Views;

/// <summary>
/// Jack's panel: model picker and thread actions on top, the conversation in the middle, and a
/// composer with image attachments at the bottom (Ctrl+Enter sends).
/// </summary>
public sealed class ChatPane : DockPanel
{
    private readonly AppGenHost _host;
    private readonly StackPanel _messages = new() { Spacing = 14, Margin = new Thickness(14, 12, 14, 16) };
    private readonly ScrollViewer _scroll;
    private readonly TextBox _input;
    private readonly WrapPanel _attachments = new() { Orientation = Orientation.Horizontal };
    private readonly ComboBox _model = new() { MinWidth = 150, FontSize = 12, VerticalAlignment = VerticalAlignment.Center };
    private readonly Border _pulse;
    private readonly Button _send;
    private readonly Dictionary<ChatMessage, Border> _views = [];
    private readonly List<Action> _unsubscribe = [];
    private readonly DispatcherTimer _pulseTimer;
    private double _pulsePhase;
    private bool _scrollPending;

    public ChatPane(AppGenHost host, ChatViewModel vm, Action hide)
    {
        _host = host;
        ViewModel = vm;
        var p = Look.P;
        Background = Look.Brush(p.Surface);

        // ---- header
        var avatar = new Border
        {
            Width = 30,
            Height = 30,
            CornerRadius = new CornerRadius(9),
            Background = Look.SignalGradient(),
            Child = Kit.Icon(Icons.Bot, 17, Colors.White, 2),
        };
        var title = Kit.Col(0,
            Kit.Text("Jack", 14, FontWeight.Bold),
            Kit.Muted("The Code Bender", 11));
        RefreshProfiles();
        _model.SelectionChanged += (_, _) =>
        {
            if (_model.SelectedItem is string name && name != _host.Settings.ActiveProfile)
            {
                ViewModel.UseProfile(name);
            }
        };
        ToolTip.SetTip(_model, "Model used by Jack");

        var header = new DockPanel { Margin = new Thickness(14, 10, 8, 10) };
        header.Children.Add(Kit.Dock(Kit.Row(2,
            Kit.IconButton(Icons.Trash, "Clear chat thread", () => ViewModel.Clear()),
            Kit.IconButton(Icons.Close, "Hide chat panel", hide)), Avalonia.Controls.Dock.Right));
        header.Children.Add(Kit.Row(10, avatar, title, new Border { Width = 4 }, _model));

        // The signature: a violet→cyan trace that pulses while Jack works, like a live topic.
        _pulse = new Border { Height = 2, Background = Look.SignalGradient(), Opacity = 0.25 };
        _pulseTimer = new DispatcherTimer(TimeSpan.FromMilliseconds(50), DispatcherPriority.Background, (_, _) =>
        {
            _pulsePhase += 0.18;
            _pulse.Opacity = 0.55 + (0.45 * Math.Sin(_pulsePhase));
        });

        var top = new StackPanel
        {
            Children =
            {
                header,
                _pulse,
            },
        };
        Children.Add(Kit.Dock(top, Avalonia.Controls.Dock.Top));

        // ---- composer
        _input = new TextBox
        {
            AcceptsReturn = true,
            TextWrapping = TextWrapping.Wrap,
            MinHeight = 64,
            MaxHeight = 220,
            FontSize = 13,
            PlaceholderText = "Describe the app you want, or ask Jack to change the open project…",
            BorderThickness = new Thickness(0),
            Background = Brushes.Transparent,
            Padding = new Thickness(10, 8),
        };
        _input.KeyDown += OnInputKeyDown;
        _input.AddHandler(KeyDownEvent, OnPreviewKeyDown, Avalonia.Interactivity.RoutingStrategies.Tunnel);

        _send = Kit.Button("Send", () => _ = SendAsync(), primary: true, icon: Icons.Send, tip: "Send (Ctrl+Enter)");
        var composerBar = new DockPanel { Margin = new Thickness(6, 0, 8, 8) };
        composerBar.Children.Add(Kit.Dock(_send, Avalonia.Controls.Dock.Right));
        composerBar.Children.Add(Kit.Row(2,
            Kit.IconButton(Icons.Attach, "Attach image", () => _ = PickImagesAsync()),
            Kit.IconButton(Icons.Image, "Paste image from clipboard (Ctrl+V)", () => _ = PasteImageAsync()),
            Kit.Muted("Ctrl+Enter to send", 11)));

        var composer = new Border
        {
            Margin = new Thickness(12, 4, 12, 12),
            CornerRadius = new CornerRadius(10),
            BorderBrush = Look.Brush(p.Seam),
            BorderThickness = new Thickness(1),
            Background = Look.Brush(p.Sunken),
            Child = new StackPanel
            {
                Children =
                {
                    new Border { Padding = new Thickness(8, 6, 8, 0), Child = _attachments },
                    _input,
                    composerBar,
                },
            },
        };
        Children.Add(Kit.Dock(composer, Avalonia.Controls.Dock.Bottom));

        _scroll = new ScrollViewer { Content = _messages, HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Disabled };
        Children.Add(_scroll);

        NotifyCollectionChangedEventHandler pending = (_, _) => RenderAttachments();
        Action state = () => Dispatcher.UIThread.Post(UpdateBusy);
        vm.Messages.CollectionChanged += OnMessagesChanged;
        vm.PendingImages.CollectionChanged += pending;
        vm.StateChanged += state;
        _unsubscribe.Add(() =>
        {
            vm.Messages.CollectionChanged -= OnMessagesChanged;
            vm.PendingImages.CollectionChanged -= pending;
            vm.StateChanged -= state;
        });
        RenderAttachments();
        foreach (var m in vm.Messages)
        {
            Add(m);
        }

        RenderEmptyState();
        UpdateBusy();
    }

    public ChatViewModel ViewModel { get; }

    /// <summary>Unsubscribes from the view-model (call before discarding the pane, e.g. on theme change).</summary>
    public void Release()
    {
        foreach (var u in _unsubscribe)
        {
            u();
        }

        _unsubscribe.Clear();
        _pulseTimer.Stop();
    }

    /// <summary>The composer text box (exposed for tests and focus handling).</summary>
    public TextBox Input => _input;

    public void RefreshProfiles()
    {
        var names = _host.Settings.Profiles.Select(p => p.Name).ToList();
        _model.ItemsSource = names;
        _model.SelectedItem = names.Contains(_host.Settings.ActiveProfile) ? _host.Settings.ActiveProfile : names.FirstOrDefault();
        _model.PlaceholderText = names.Count == 0 ? "Add a model in Settings" : null;
    }

    public void FocusInput() => _input.Focus();

    public async Task SendAsync()
    {
        var text = _input.Text ?? string.Empty;
        if (ViewModel.IsBusy)
        {
            ViewModel.Stop();
            return;
        }

        if (text.Trim().Length == 0 && ViewModel.PendingImages.Count == 0)
        {
            return;
        }

        _input.Text = string.Empty;
        await ViewModel.SendAsync(text);
    }

    private void OnPreviewKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == Key.Enter && e.KeyModifiers.HasFlag(KeyModifiers.Control))
        {
            e.Handled = true;
            _ = SendAsync();
        }
        else if (e.Key == Key.V && e.KeyModifiers.HasFlag(KeyModifiers.Control))
        {
            // Let text paste happen; additionally pick up an image if the clipboard holds one.
            _ = PasteImageAsync(silent: true);
        }
    }

    private void OnInputKeyDown(object? sender, KeyEventArgs e)
    {
        if (e.Key == Key.Escape && ViewModel.IsBusy)
        {
            ViewModel.Stop();
            e.Handled = true;
        }
    }

    private async Task PickImagesAsync()
    {
        var top = TopLevel.GetTopLevel(this);
        if (top is null)
        {
            return;
        }

        var files = await top.StorageProvider.OpenFilePickerAsync(new FilePickerOpenOptions
        {
            Title = "Attach images",
            AllowMultiple = true,
            FileTypeFilter = [new FilePickerFileType("Images") { Patterns = ["*.png", "*.jpg", "*.jpeg", "*.gif", "*.webp", "*.bmp"] }],
        });
        foreach (var f in files)
        {
            if (f.TryGetLocalPath() is { } path)
            {
                ViewModel.AttachImage(path);
            }
        }
    }

    private async Task PasteImageAsync(bool silent = false)
    {
        try
        {
            var clipboard = TopLevel.GetTopLevel(this)?.Clipboard;
            if (clipboard is null)
            {
                return;
            }

            var bitmap = await ClipboardImages.TryGetBitmapAsync(clipboard);
            if (bitmap is null)
            {
                if (!silent)
                {
                    _host.WriteLog("The clipboard holds no image.");
                }

                return;
            }

            using var ms = new MemoryStream();
#pragma warning disable CS0618 // PNG is the default encoding; the options overload adds nothing here.
            bitmap.Save(ms);
#pragma warning restore CS0618
            ViewModel.AttachImage(new ImageAttachment($"pasted-{DateTime.Now:HHmmss}.png", "image/png", ms.ToArray()));
        }
        catch (Exception ex) when (!silent)
        {
            _host.WriteLog("Could not paste the image: " + ex.Message);
        }
        catch (Exception)
        {
        }
    }

    private void RenderAttachments()
    {
        _attachments.Children.Clear();
        var p = Look.P;
        foreach (var img in ViewModel.PendingImages.ToList())
        {
            var thumb = Thumbnail(img, 44);
            var remove = Kit.IconButton(Icons.Close, "Remove", () => ViewModel.PendingImages.Remove(img), size: 10);
            remove.Padding = new Thickness(3);
            remove.HorizontalAlignment = HorizontalAlignment.Right;
            remove.VerticalAlignment = VerticalAlignment.Top;
            remove.Background = Look.Brush(p.Raised);
            _attachments.Children.Add(new Panel { Margin = new Thickness(0, 0, 6, 0), Children = { thumb, remove } });
        }
    }

    private static Control Thumbnail(ImageAttachment img, double size)
    {
        Control content;
        try
        {
            using var ms = new MemoryStream(img.Data);
            content = new Image { Source = new Bitmap(ms), Stretch = Stretch.UniformToFill };
        }
        catch (Exception)
        {
            content = Kit.Icon(Icons.Image, 18);
        }

        var b = new Border
        {
            Width = size,
            Height = size,
            CornerRadius = new CornerRadius(6),
            ClipToBounds = true,
            BorderBrush = Look.Brush(Look.P.Seam),
            BorderThickness = new Thickness(1),
            Child = content,
        };
        ToolTip.SetTip(b, img.FileName);
        return b;
    }

    private void OnMessagesChanged(object? sender, NotifyCollectionChangedEventArgs e)
    {
        if (e.Action == NotifyCollectionChangedAction.Reset)
        {
            _messages.Children.Clear();
            _views.Clear();
            RenderEmptyState();
            return;
        }

        foreach (ChatMessage m in e.NewItems ?? Array.Empty<ChatMessage>())
        {
            Add(m);
        }
    }

    private void Add(ChatMessage m)
    {
        if (_messages.Children.Count == 1 && _messages.Children[0].Tag as string == "empty")
        {
            _messages.Children.Clear();
        }

        var view = new Border();
        _views[m] = view;
        Fill(view, m);
        _messages.Children.Add(view);
        Action changed = () => Dispatcher.UIThread.Post(() =>
        {
            Fill(view, m);
            ScrollToEnd();
        }, DispatcherPriority.Background);
        m.Changed += changed;
        _unsubscribe.Add(() => m.Changed -= changed);
        ScrollToEnd();
    }

    private void Fill(Border view, ChatMessage m)
    {
        var p = Look.P;
        if (m.Role == "user")
        {
            var col = new StackPanel { Spacing = 6 };
            if (m.Images.Count > 0)
            {
                var row = new WrapPanel();
                foreach (var img in m.Images)
                {
                    var t = Thumbnail(img, 72);
                    t.Margin = new Thickness(0, 0, 6, 0);
                    row.Children.Add(t);
                }

                col.Children.Add(row);
            }

            if (m.Text.Length > 0)
            {
                col.Children.Add(MarkdownView.Inline(m.Text, 13, FontWeight.Normal, p.Ink));
            }

            view.HorizontalAlignment = HorizontalAlignment.Right;
            view.MaxWidth = 520;
            view.Margin = new Thickness(36, 0, 0, 0);
            view.Background = Look.Brush(p.Raised);
            view.BorderBrush = Look.Brush(p.Seam);
            view.BorderThickness = new Thickness(1);
            view.CornerRadius = new CornerRadius(12, 12, 3, 12);
            view.Padding = new Thickness(12, 9);
            view.Child = col;
            return;
        }

        // Assistant: no bubble — text runs full width with a thin signal rule, like an annotated log.
        var body = new StackPanel { Spacing = 8 };
        var who = Kit.Row(6,
            Kit.Text("Jack", 11.5, FontWeight.SemiBold, p.Telemetry),
            Kit.Muted(m.Time.ToString("HH:mm"), 10.5));
        body.Children.Add(who);

        if (m.Tools.Count > 0)
        {
            var tools = new StackPanel { Spacing = 3 };
            foreach (var t in m.Tools)
            {
                tools.Children.Add(ToolRow(t));
            }

            body.Children.Add(new Border
            {
                Background = Look.Brush(p.Sunken),
                CornerRadius = new CornerRadius(7),
                Padding = new Thickness(8, 6),
                Child = tools,
            });
        }

        if (m.Text.Length > 0)
        {
            body.Children.Add(MarkdownView.Render(m.Text, m.IsError ? p.Fault : null));
        }
        else if (m.IsStreaming && m.Tools.Count == 0)
        {
            body.Children.Add(Kit.Muted("Thinking…", 12.5));
        }

        if (m.IsStreaming && m.Text.Length > 0)
        {
            body.Children.Add(new Border { Width = 8, Height = 14, Background = Look.Brush(p.Telemetry), HorizontalAlignment = HorizontalAlignment.Left, Opacity = 0.8 });
        }

        view.HorizontalAlignment = HorizontalAlignment.Stretch;
        view.BorderBrush = Look.SignalGradient(90);
        view.BorderThickness = new Thickness(2, 0, 0, 0);
        view.Padding = new Thickness(12, 2, 0, 2);
        view.Child = body;
    }

    private static Control ToolRow(ToolActivity t)
    {
        var p = Look.P;
        var status = t.IsDone
            ? Kit.Icon(Icons.Check, 11, p.Ok, 2.6)
            : new Border { Width = 7, Height = 7, CornerRadius = new CornerRadius(4), Background = Look.Brush(p.Warn), Margin = new Thickness(2, 0) };
        var row = new DockPanel();
        var time = Kit.Text(t.IsDone ? $"{t.Elapsed.TotalMilliseconds:0} ms" : "running", 10.5, color: p.Faint, mono: true);
        row.Children.Add(Kit.Dock(time, Avalonia.Controls.Dock.Right));
        row.Children.Add(Kit.Row(7, status, new TextBlock
        {
            Text = t.Detail,
            FontFamily = Look.Mono,
            FontSize = 11.5,
            Foreground = Look.Brush(t.IsDone ? p.Haze : p.Ink),
            TextTrimming = TextTrimming.CharacterEllipsis,
            MaxWidth = 300,
        }));
        if (t.Result is { Length: > 0 } r)
        {
            ToolTip.SetTip(row, r.Length > 600 ? r[..600] + "…" : r);
        }

        return row;
    }

    private void RenderEmptyState()
    {
        if (_messages.Children.Count > 0)
        {
            return;
        }

        var p = Look.P;
        var ideas = new[]
        {
            "Build a warehouse robot dashboard with a LiDAR view and teleop",
            "Make a lab sensor logger that records /imu to an MCAP bag",
            "Add a safety stop node that halts /cmd_vel when /scan sees < 0.4 m",
        };
        var col = new StackPanel { Spacing = 10, Margin = new Thickness(0, 24, 0, 0) };
        col.Children.Add(Kit.Text("Tell Jack what to build.", 15, FontWeight.Bold));
        col.Children.Add(Kit.Muted("Jack reads and edits the open project, builds it, fixes errors and knows the ROS2.Net API. Try:", 12.5, wrap: true));
        foreach (var idea in ideas)
        {
            var b = new Button
            {
                HorizontalAlignment = HorizontalAlignment.Stretch,
                HorizontalContentAlignment = HorizontalAlignment.Left,
                Background = Look.Brush(p.Raised),
                BorderBrush = Look.Brush(p.Seam),
                BorderThickness = new Thickness(1),
                CornerRadius = new CornerRadius(8),
                Padding = new Thickness(10, 8),
                Content = Kit.Text(idea, 12.5, wrap: true),
            };
            b.Click += (_, _) =>
            {
                _input.Text = idea;
                _input.Focus();
            };
            col.Children.Add(b);
        }

        col.Tag = "empty";
        _messages.Children.Add(col);
    }

    private void UpdateBusy()
    {
        var busy = ViewModel.IsBusy;
        if (busy && !_pulseTimer.IsEnabled)
        {
            _pulseTimer.Start();
        }
        else if (!busy)
        {
            _pulseTimer.Stop();
            _pulse.Opacity = 0.25;
        }
        if (_send.Content is StackPanel sp && sp.Children.OfType<TextBlock>().FirstOrDefault() is { } label)
        {
            label.Text = busy ? "Stop" : "Send";
        }

        ToolTip.SetTip(_send, busy ? "Stop Jack (Esc)" : "Send (Ctrl+Enter)");
    }

    private void ScrollToEnd()
    {
        if (_scrollPending)
        {
            return;
        }

        _scrollPending = true;
        Dispatcher.UIThread.Post(() =>
        {
            _scrollPending = false;
            _scroll.ScrollToEnd();
        }, DispatcherPriority.Background);
    }
}
