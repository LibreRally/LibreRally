using System;
using System.Collections.Generic;
using Myra;
using Myra.Graphics2D.Brushes;
using Myra.Graphics2D.UI;
using LibreRally.Vehicle.Content;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;

namespace LibreRally.HUD;

/// <summary>
/// UI overlay for selecting a vehicle from the catalog.
/// </summary>
public sealed class VehicleSelectionOverlay : GameSystemBase
{
    private static readonly Color BackdropColor = new(5, 8, 14, 180);
    private static readonly Color ShellColor = new(16, 22, 30, 242);
    private static readonly Color AccentSoftColor = new(214, 148, 78, 84);
    private static readonly Color PanelColor = new(26, 32, 43, 236);
    private static readonly Color TitleColor = new(240, 243, 247, 255);
    private static readonly Color CopyColor = new(183, 193, 205, 255);
    private static readonly Color ValueColor = new(255, 230, 204, 255);
    private static readonly SolidBrush BackdropBrush = new(BackdropColor);
    private static readonly SolidBrush ShellBrush = new(ShellColor);
    private static readonly SolidBrush PanelBrush = new(PanelColor);
    private static readonly SolidBrush SelectedItemBrush = new(AccentSoftColor);
    private static readonly SolidBrush UnselectedItemBrush = new(PanelColor);

    private readonly List<(Button Button, Label Title, Label Detail)> _buttons = [];
    private IReadOnlyList<BeamNgVehicleDescriptor> _vehicles = Array.Empty<BeamNgVehicleDescriptor>();
    private int _selectedIndex;
    private bool _overlayVisible;
    private string _statusText = string.Empty;
    private Game? _game;
    private Desktop? _desktop;
    private Label? _statusLabel;
    private Label? _selectionLabel;

    /// <summary>
    /// Gets or sets the list of vehicles available for selection.
    /// </summary>
    public IReadOnlyList<BeamNgVehicleDescriptor> Vehicles
    {
        get => _vehicles;
        set
        {
            _vehicles = value ?? Array.Empty<BeamNgVehicleDescriptor>();
            RebuildRoot();
        }
    }

    /// <summary>
    /// Gets or sets the index of the currently selected vehicle.
    /// </summary>
    public int SelectedIndex
    {
        get => _selectedIndex;
        set
        {
            _selectedIndex = value;
            UpdateSelectionStyles();
            UpdateLabels();
            FocusSelectedButton();
        }
    }

    /// <summary>
    /// Gets or sets a value indicating whether the vehicle selection overlay is visible.
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
    /// Gets or sets the status text displayed in the overlay.
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
    /// Occurs when a vehicle is activated (e.g., confirmed for loading).
    /// </summary>
    public Action<int>? ItemActivated { get; set; }

    /// <summary>
    /// Initializes a new instance of the <see cref="VehicleSelectionOverlay"/> class.
    /// </summary>
    /// <param name="services">The service registry.</param>
    public VehicleSelectionOverlay(IServiceRegistry services) : base(services)
    {
        Enabled = true;
        Visible = true;
        DrawOrder = 9996;
        UpdateOrder = 9996;
    }

    /// <summary>
    /// Initializes the vehicle-selection desktop UI.
    /// </summary>
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

    /// <summary>
    /// Draws the vehicle-selection overlay when it is visible.
    /// </summary>
    /// <param name="gameTime">Stride timing data for the current frame.</param>
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
            Width = 880,
            Height = 620,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            Background = ShellBrush,
        };

        var shell = new VerticalStackPanel
        {
            Width = 820,
            Height = 560,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            Spacing = 12,
        };

        shell.Widgets.Add(new Label
        {
            Text = "Vehicle Select",
            TextColor = TitleColor,
        });

        shell.Widgets.Add(new Label
        {
            Text = "Choose a bundled BeamNG vehicle. D-Pad Up/Down select  •  A load  •  B/Esc/Start back out",
            TextColor = CopyColor,
            Wrap = true,
        });

        _selectionLabel = new Label
        {
            TextColor = ValueColor,
            Wrap = true,
        };
        shell.Widgets.Add(_selectionLabel);

        var listFrame = new Panel
        {
            Height = 410,
            Background = PanelBrush,
        };

        Widget listWidget;
        if (_vehicles.Count == 0)
        {
            listWidget = new Label
            {
                Text = "No vehicles found in Resources\\BeamNG Vehicles.",
                TextColor = TitleColor,
                HorizontalAlignment = HorizontalAlignment.Center,
                VerticalAlignment = VerticalAlignment.Center,
            };
        }
        else
        {
            var list = new VerticalStackPanel
            {
                Width = 760,
                HorizontalAlignment = HorizontalAlignment.Center,
                VerticalAlignment = VerticalAlignment.Top,
                Spacing = 8,
            };

            for (var index = 0; index < _vehicles.Count; index++)
            {
                list.Widgets.Add(CreateVehicleButton(_vehicles[index], index));
            }

            listWidget = new ScrollViewer
            {
                Content = list,
                Height = 382,
                VerticalAlignment = VerticalAlignment.Stretch,
            };
        }

        listFrame.Widgets.Add(listWidget);
        shell.Widgets.Add(listFrame);

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

    private Button CreateVehicleButton(BeamNgVehicleDescriptor vehicle, int index)
    {
        var titleLabel = new Label
        {
            Text = vehicle.DisplayName,
            TextColor = TitleColor,
        };
        var detailLabel = new Label
        {
            Text = $"{vehicle.SourceLabel}  //  {vehicle.VehicleId}",
            TextColor = CopyColor,
            Wrap = true,
        };

        var content = new VerticalStackPanel
        {
            Spacing = 4,
        };
        content.Widgets.Add(titleLabel);
        content.Widgets.Add(detailLabel);

        var button = new Button
        {
            Height = 70,
            HorizontalAlignment = HorizontalAlignment.Stretch,
            Content = content,
        };
        button.Click += (_, _) =>
        {
            SelectedIndex = index;
            ItemActivated?.Invoke(index);
        };

        _buttons.Add((button, titleLabel, detailLabel));
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
        for (var i = 0; i < _buttons.Count; i++)
        {
            var isSelected = i == Math.Clamp(_selectedIndex, 0, Math.Max(_buttons.Count - 1, 0));
            var (button, title, detail) = _buttons[i];
            button.Background = isSelected ? SelectedItemBrush : UnselectedItemBrush;
            title.TextColor = isSelected ? ValueColor : TitleColor;
            detail.TextColor = isSelected ? ValueColor : CopyColor;
        }
    }

    private void UpdateLabels()
    {
        if (_statusLabel != null)
        {
            _statusLabel.Text = _statusText;
        }

        if (_selectionLabel == null)
        {
            return;
        }

        if (_vehicles.Count == 0)
        {
            _selectionLabel.Text = "Catalog status: no BeamNG vehicles discovered.";
            return;
        }

        var index = Math.Clamp(_selectedIndex, 0, _vehicles.Count - 1);
        var vehicle = _vehicles[index];
        _selectionLabel.Text = $"Selected: {vehicle.DisplayName}  //  {vehicle.SourceLabel}";
    }

    private void FocusSelectedButton()
    {
        if (!_overlayVisible || _desktop == null || _buttons.Count == 0)
        {
            return;
        }

        var index = Math.Clamp(_selectedIndex, 0, _buttons.Count - 1);
        _desktop.FocusedKeyboardWidget = _buttons[index].Button;
    }
}
