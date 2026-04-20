using System;
using System.Collections.Generic;
using Myra;
using Myra.Graphics2D.Brushes;
using Myra.Graphics2D.UI;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;

namespace LibreRally.HUD;

/// <summary>
/// Represents an item in the pause menu.
/// </summary>
/// <param name="Title">The title of the menu item.</param>
/// <param name="Description">The detailed description of what the item does.</param>
public readonly record struct PauseMenuItem(string Title, string Description);

/// <summary>
/// UI overlay for the in-game pause menu, allowing access to settings, car reset, and vehicle selection.
/// </summary>
public sealed class PauseMenuOverlay : GameSystemBase
{
    private static readonly Color BackdropColor = new(5, 8, 14, 180);
    private static readonly Color ShellColor = new(16, 22, 30, 242);
    private static readonly Color AccentColor = new(214, 148, 78, 255);
    private static readonly Color AccentSoftColor = new(214, 148, 78, 84);
    private static readonly Color PanelColor = new(26, 32, 43, 236);
    private static readonly Color TitleColor = new(240, 243, 247, 255);
    private static readonly Color CopyColor = new(183, 193, 205, 255);
    private static readonly Color ValueColor = new(255, 230, 204, 255);
    private static readonly SolidBrush BackdropBrush = new(BackdropColor);
    private static readonly SolidBrush ShellBrush = new(ShellColor);
    private static readonly SolidBrush SelectedItemBrush = new(AccentColor);
    private static readonly SolidBrush UnselectedItemBrush = new(PanelColor);

    private readonly List<(Button Button, Label Title, Label Description)> _buttons = [];
    private IReadOnlyList<PauseMenuItem> _items = Array.Empty<PauseMenuItem>();
    private bool _overlayVisible;
    private string _statusText = string.Empty;
    private string _vehicleName = string.Empty;
    private int _selectedIndex;
    private Game? _game;
    private Desktop? _desktop;
    private Label? _vehicleNameLabel;
    private Label? _statusLabel;

    /// <summary>
    /// Gets or sets the list of items to display in the menu.
    /// </summary>
    public IReadOnlyList<PauseMenuItem> Items
    {
        get => _items;
        set
        {
            _items = value ?? Array.Empty<PauseMenuItem>();
            RebuildRoot();
        }
    }

    /// <summary>
    /// Gets or sets the index of the currently selected menu item.
    /// </summary>
    public int SelectedIndex
    {
        get => _selectedIndex;
        set
        {
            _selectedIndex = value;
            UpdateSelectionStyles();
        }
    }

    /// <summary>
    /// Gets or sets a value indicating whether the pause menu overlay is visible.
    /// </summary>
    public bool OverlayVisible
    {
        get => _overlayVisible;
        set
        {
            _overlayVisible = value;
            FocusSelectedButton();
        }
    }

    /// <summary>
    /// Gets or sets the status text displayed in the menu.
    /// </summary>
    public string StatusText
    {
        get => _statusText;
        set
        {
            _statusText = value ?? string.Empty;
            UpdateLabels();
        }
    }

    /// <summary>
    /// Gets or sets the name of the current vehicle displayed in the menu.
    /// </summary>
    public string VehicleName
    {
        get => _vehicleName;
        set
        {
            _vehicleName = value ?? string.Empty;
            UpdateLabels();
        }
    }

    /// <summary>
    /// Occurs when a menu item is activated (e.g., clicked or confirmed).
    /// </summary>
    public Action<int>? ItemActivated { get; set; }

    /// <summary>
    /// Initializes a new instance of the <see cref="PauseMenuOverlay"/> class.
    /// </summary>
    /// <param name="services">The service registry.</param>
    public PauseMenuOverlay(IServiceRegistry services) : base(services)
    {
        Enabled = true;
        Visible = true;
        DrawOrder = 9995;
        UpdateOrder = 9995;
    }

    /// <inheritdoc/>
    public override void Initialize()
    {
        base.Initialize();

        _game = (Game?)Services.GetService<IGame>();
        if (_game == null)
        {
            return;
        }

        MyraEnvironment.Game = _game;
        _desktop = new Desktop
        {
            Root = BuildRoot(),
        };
        FocusSelectedButton();
    }

    protected override void Destroy()
    {
        _desktop?.Dispose();
        base.Destroy();
    }

    /// <inheritdoc/>
    public override void Draw(GameTime gameTime)
    {
        if (!OverlayVisible || _game == null || _desktop == null)
        {
            return;
        }

        var presenter = _game.GraphicsDevice.Presenter;
        if (presenter?.BackBuffer == null)
        {
            return;
        }

        var context = _game.GraphicsContext;
        context.CommandList.SetRenderTargetAndViewport(presenter.DepthStencilBuffer, presenter.BackBuffer);
        _desktop.Render();
    }

    private Widget BuildRoot()
    {
        _buttons.Clear();

        var root = new Panel
        {
            Background = BackdropBrush,
        };

        var shellFrame = new Panel
        {
            Width = 760,
            Height = 560,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            Background = ShellBrush,
        };

        var shell = new VerticalStackPanel
        {
            Width = 700,
            Height = 500,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            Spacing = 12,
        };

        shell.Widgets.Add(new Label
        {
            Text = "Paused",
            TextColor = TitleColor,
        });

        _vehicleNameLabel = new Label
        {
            TextColor = ValueColor,
        };
        shell.Widgets.Add(_vehicleNameLabel);

        shell.Widgets.Add(new Label
        {
            Text = "D-Pad Up/Down select  •  A activate  •  B/Esc/Start back out",
            TextColor = CopyColor,
            Wrap = true,
        });

        shell.Widgets.Add(CreateSpacer(6));

        for (var index = 0; index < _items.Count; index++)
        {
            shell.Widgets.Add(CreateMenuButton(_items[index], index));
        }

        shell.Widgets.Add(CreateSpacer(10));

        _statusLabel = new Label
        {
            TextColor = CopyColor,
            Wrap = true,
        };
        shell.Widgets.Add(_statusLabel);

        shellFrame.Widgets.Add(shell);
        root.Widgets.Add(shellFrame);

        UpdateSelectionStyles();
        UpdateLabels();
        return root;
    }

    private Button CreateMenuButton(PauseMenuItem item, int index)
    {
        var titleLabel = new Label
        {
            Text = item.Title,
            TextColor = TitleColor,
        };
        var descriptionLabel = new Label
        {
            Text = item.Description,
            TextColor = CopyColor,
            Wrap = true,
        };

        var content = new VerticalStackPanel
        {
            Spacing = 4,
        };
        content.Widgets.Add(titleLabel);
        content.Widgets.Add(descriptionLabel);

        var button = new Button
        {
            Height = 78,
            HorizontalAlignment = HorizontalAlignment.Stretch,
            Content = content,
        };
        button.Click += (_, _) =>
        {
            SelectedIndex = index;
            ItemActivated?.Invoke(index);
        };

        _buttons.Add((button, titleLabel, descriptionLabel));
        return button;
    }

    private void RebuildRoot()
    {
        if (_desktop == null)
        {
            return;
        }

        _desktop.Root = BuildRoot();
        FocusSelectedButton();
    }

    private void UpdateSelectionStyles()
    {
        var clampedIndex = _buttons.Count == 0
            ? 0
            : Math.Clamp(SelectedIndex, 0, _buttons.Count - 1);

        for (var i = 0; i < _buttons.Count; i++)
        {
            var (button, title, description) = _buttons[i];
            var isSelected = i == clampedIndex;
            button.Background = isSelected ? SelectedItemBrush : UnselectedItemBrush;
            title.TextColor = isSelected ? ValueColor : TitleColor;
            description.TextColor = isSelected ? new Color(255, 239, 220, 220) : CopyColor;
        }
    }

    private void UpdateLabels()
    {
        if (_vehicleNameLabel != null)
        {
            _vehicleNameLabel.Text = string.IsNullOrWhiteSpace(VehicleName) ? "LibreRally" : VehicleName;
        }

        if (_statusLabel != null)
        {
            _statusLabel.Text = StatusText;
        }
    }

    private void FocusSelectedButton()
    {
        if (!_overlayVisible || _desktop == null || _buttons.Count == 0)
        {
            return;
        }

        var clampedIndex = Math.Clamp(SelectedIndex, 0, _buttons.Count - 1);
        _desktop.FocusedKeyboardWidget = _buttons[clampedIndex].Button;
    }

    private static Panel CreateSpacer(int height) => new()
    {
        Height = height,
    };

}
