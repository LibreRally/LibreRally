using System;
using System.Collections.Generic;
using System.Linq;
using Myra;
using Myra.Graphics2D.Brushes;
using Myra.Graphics2D.UI;
using LibreRally.Vehicle;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Input;

namespace LibreRally.HUD;

public sealed class SetupUiShellOverlay : GameSystemBase
{
    private const int MaxVisibleEditorFields = 3;
    private const int FooterActionCount = 3;

    private enum SetupPane
    {
        Categories,
        Fields,
        Footer,
    }

    private static readonly Color BackdropColor = new(5, 8, 14, 196);
    private static readonly Color ShellColor = new(16, 22, 30, 244);
    private static readonly Color PanelColor = new(24, 31, 42, 236);
    private static readonly Color PanelAltColor = new(19, 25, 34, 236);
    private static readonly Color PanelSelectedColor = new(38, 50, 66, 244);
    private static readonly Color AccentColor = new(214, 148, 78, 255);
    private static readonly Color AccentSoftColor = new(214, 148, 78, 80);
    private static readonly Color TitleColor = new(240, 243, 247, 255);
    private static readonly Color CopyColor = new(183, 193, 205, 255);
    private static readonly Color MutedColor = new(132, 145, 160, 255);
    private static readonly Color ValueColor = new(255, 230, 204, 255);

    private SetupUiShellModel _shell = SetupUiShellModel.CreatePreview();
    private readonly List<(Button Button, Label Label)> _categoryButtons = [];

    private Game? _game;
    private Desktop? _desktop;
    private Label? _vehicleNameLabel;
    private Label? _statusLabel;
    private Label? _categoryTitleLabel;
    private Label? _categoryTaglineLabel;
    private Label? _categoryDescriptionLabel;
    private Label? _fieldWindowLabel;
    private VerticalStackPanel? _editorStack;
    private VerticalStackPanel? _summaryStack;
    private Label? _pendingSummaryLabel;
    private Button? _applyButton;
    private Button? _discardButton;
    private Button? _resetDefaultsButton;
    private bool _overlayVisible;
    private SetupPane _selectedPane = SetupPane.Categories;
    private int _selectedFieldIndex;
    private int _fieldScrollOffset;
    private int _selectedFooterIndex = FooterActionCount - 1;

    public Action<IReadOnlyDictionary<string, float>>? ApplyRequested { get; set; }

    public bool OverlayVisible
    {
        get => _overlayVisible;
        set
        {
            _overlayVisible = value;
            if (_overlayVisible && _desktop != null && _categoryButtons.Count > 0)
            {
                _desktop.FocusedKeyboardWidget = _categoryButtons[_shell.SelectedCategoryIndex].Button;
            }
        }
    }

    public string StatusText
    {
        get => _shell.StatusText;
        set
        {
            _shell.UpdateVehicleContext(_shell.VehicleName, value);
            UpdateStaticLabels();
        }
    }

    public string VehicleName
    {
        get => _shell.VehicleName;
        set
        {
            _shell.UpdateVehicleContext(value, _shell.StatusText);
            UpdateStaticLabels();
        }
    }

    public SetupUiShellOverlay(IServiceRegistry services) : base(services)
    {
        Enabled = true;
        Visible = true;
        DrawOrder = 9997;
        UpdateOrder = 9997;
    }

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

        if (_categoryButtons.Count > 0)
        {
            _desktop.FocusedKeyboardWidget = _categoryButtons[0].Button;
        }
    }

    protected override void Destroy()
    {
        _desktop?.Dispose();
        base.Destroy();
    }

    public void BindVehicle(LoadedVehicle? vehicle, string? statusText = null)
    {
        _shell = vehicle == null
            ? SetupUiShellModel.CreatePreview(vehicleName: VehicleName, statusText: statusText ?? StatusText)
            : SetupUiShellModel.CreateFromRuntimeSetup(
                vehicle.CarComponent.RuntimeSetup,
                vehicle.Definition.VehicleName,
                statusText ?? StatusText);
        ResetNavigationState();
        RebuildRoot();
    }

    public override void Update(GameTime gameTime)
    {
        if (!OverlayVisible || _game == null)
        {
            return;
        }

        HandleNavigationInput();
    }

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
        var root = new Panel
        {
            Background = Brush(BackdropColor),
        };

        var shellFrame = new Panel
        {
            Width = 1360,
            Height = 800,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            Background = Brush(ShellColor),
        };

        var shell = new Grid
        {
            Width = 1312,
            Height = 752,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            ColumnSpacing = 18,
            RowSpacing = 18,
        };
        shell.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
        shell.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
        shell.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
        shell.RowsProportions.Add(new Proportion(ProportionType.Auto));
        shell.RowsProportions.Add(new Proportion(ProportionType.Fill));
        shell.RowsProportions.Add(new Proportion(ProportionType.Auto));

        var header = BuildHeader();
        Grid.SetColumnSpan(header, 3);
        shell.Widgets.Add(header);

        var navigationCard = BuildNavigationCard();
        Grid.SetRow(navigationCard, 1);
        shell.Widgets.Add(navigationCard);

        var editorsCard = BuildEditorsCard();
        Grid.SetColumn(editorsCard, 1);
        Grid.SetRow(editorsCard, 1);
        shell.Widgets.Add(editorsCard);

        var summaryCard = BuildSummaryCard();
        Grid.SetColumn(summaryCard, 2);
        Grid.SetRow(summaryCard, 1);
        shell.Widgets.Add(summaryCard);

        var footer = BuildFooter();
        Grid.SetColumnSpan(footer, 3);
        Grid.SetRow(footer, 2);
        shell.Widgets.Add(footer);

        shellFrame.Widgets.Add(shell);
        root.Widgets.Add(shellFrame);

        RefreshCategoryContent();
        UpdateCategoryButtonStyles();
        UpdateStaticLabels();

        return root;
    }

    private Panel BuildHeader()
    {
        var panel = new Panel
        {
            Height = 96,
            Background = Brush(PanelAltColor),
        };

        var header = new Grid
        {
            Width = 1272,
            Height = 72,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            ColumnSpacing = 12,
        };
        header.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
        header.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
        header.RowsProportions.Add(new Proportion(ProportionType.Auto));
        header.RowsProportions.Add(new Proportion(ProportionType.Auto));

        header.Widgets.Add(new Label
        {
            Text = _shell.Title,
            TextColor = TitleColor,
        });

        var subtitle = new Label
        {
            Text = _shell.Subtitle,
            TextColor = CopyColor,
            Wrap = true,
        };
        Grid.SetRow(subtitle, 1);
        header.Widgets.Add(subtitle);

        var previewBadge = new Label
        {
            Text = "MYRA  //  GARAGE SETUP",
            TextColor = ValueColor,
            Background = Brush(AccentSoftColor),
            HorizontalAlignment = HorizontalAlignment.Right,
            VerticalAlignment = VerticalAlignment.Center,
        };
        Grid.SetColumn(previewBadge, 1);
        Grid.SetRowSpan(previewBadge, 2);
        header.Widgets.Add(previewBadge);

        panel.Widgets.Add(header);
        return panel;
    }

    private Panel BuildNavigationCard()
    {
        var panel = CreateCardPanel(width: 250, innerWidth: 226);

        var content = CreateCardContent();
        content.Width = 226;
        content.HorizontalAlignment = HorizontalAlignment.Center;
        content.VerticalAlignment = VerticalAlignment.Top;
        content.Widgets.Add(CreateSectionTitle("Garage Systems"));
        content.Widgets.Add(CreateSectionBody("Choose a setup category. D-Pad left/right or shoulders changes pane focus."));
        content.Widgets.Add(CreateSpacer(6));

        foreach (var category in _shell.Categories)
        {
            content.Widgets.Add(CreateCategoryButton(category));
        }

        panel.Widgets.Add(content);
        return panel;
    }

    private Panel BuildEditorsCard()
    {
        var panel = CreateCardPanel(width: 680, innerWidth: 648);
        var content = CreateCardContent();
        content.Width = 648;
        content.HorizontalAlignment = HorizontalAlignment.Center;
        content.VerticalAlignment = VerticalAlignment.Top;

        _categoryTitleLabel = CreateSectionTitle(string.Empty);
        _categoryTaglineLabel = new Label
        {
            TextColor = ValueColor,
        };
        _categoryDescriptionLabel = CreateSectionBody(string.Empty);
        _fieldWindowLabel = new Label
        {
            TextColor = MutedColor,
        };

        _editorStack = new VerticalStackPanel
        {
            Spacing = 12,
        };

        content.Widgets.Add(_categoryTitleLabel);
        content.Widgets.Add(_categoryTaglineLabel);
        content.Widgets.Add(_categoryDescriptionLabel);
        content.Widgets.Add(_fieldWindowLabel);
        content.Widgets.Add(CreateSpacer(8));
        content.Widgets.Add(_editorStack);

        panel.Widgets.Add(content);
        return panel;
    }

    private Panel BuildSummaryCard()
    {
        var panel = CreateCardPanel(width: 300, innerWidth: 272);
        var content = CreateCardContent();
        content.Width = 272;
        content.HorizontalAlignment = HorizontalAlignment.Center;
        content.VerticalAlignment = VerticalAlignment.Top;

        content.Widgets.Add(CreateSectionTitle("Current Values"));
        content.Widgets.Add(CreateSectionBody("Vehicle context and the currently visible setup slice."));
        content.Widgets.Add(CreateSpacer(8));

        _vehicleNameLabel = new Label
        {
            TextColor = TitleColor,
        };
        _statusLabel = CreateSectionBody(string.Empty);

        content.Widgets.Add(_vehicleNameLabel);
        content.Widgets.Add(_statusLabel);
        content.Widgets.Add(CreateSpacer(10));
        content.Widgets.Add(new Label
        {
            Text = "Visible values",
            TextColor = MutedColor,
        });

        _summaryStack = new VerticalStackPanel
        {
            Spacing = 8,
        };
        content.Widgets.Add(_summaryStack);
        content.Widgets.Add(CreateSpacer(8));
        content.Widgets.Add(new Label
        {
            Text = "D-Pad Up/Down moves the active list. Shoulders or D-Pad Left/Right change focus between categories, values, and footer.",
            TextColor = MutedColor,
            Wrap = true,
        });

        panel.Widgets.Add(content);
        return panel;
    }

    private Widget BuildFooter()
    {
        var footer = new Grid
        {
            ColumnSpacing = 12,
            HorizontalAlignment = HorizontalAlignment.Stretch,
        };
        footer.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
        footer.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
        footer.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
        footer.ColumnsProportions.Add(new Proportion(ProportionType.Auto));

        _pendingSummaryLabel = new Label
        {
            TextColor = CopyColor,
            Wrap = true,
            HorizontalAlignment = HorizontalAlignment.Stretch,
        };
        footer.Widgets.Add(_pendingSummaryLabel);

        _discardButton = new Button
        {
            Background = Brush(PanelAltColor),
            Content = new Label
            {
                Text = "Revert",
                TextColor = CopyColor,
            },
        };
        _discardButton.Click += (_, _) =>
        {
            _selectedPane = SetupPane.Footer;
            _selectedFooterIndex = 0;
            _shell.ResetPendingChanges();
            RefreshCategoryContent();
        };
        Grid.SetColumn(_discardButton, 1);
        footer.Widgets.Add(_discardButton);

        _resetDefaultsButton = new Button
        {
            Background = Brush(PanelAltColor),
            Content = new Label
            {
                Text = "Defaults",
                TextColor = CopyColor,
            },
        };
        _resetDefaultsButton.Click += (_, _) =>
        {
            _selectedPane = SetupPane.Footer;
            _selectedFooterIndex = 1;
            _shell.ResetToDefaults();
            RefreshCategoryContent();
        };
        Grid.SetColumn(_resetDefaultsButton, 2);
        footer.Widgets.Add(_resetDefaultsButton);

        _applyButton = new Button
        {
            Background = Brush(AccentSoftColor),
            Content = new Label
            {
                Text = "Apply Setup",
                TextColor = ValueColor,
            },
        };
        _applyButton.Click += (_, _) =>
        {
            _selectedPane = SetupPane.Footer;
            _selectedFooterIndex = 2;
            var payload = _shell.CreateApplyPayload();
            if (payload.Count == 0)
            {
                return;
            }

            ApplyRequested?.Invoke(payload);
        };
        Grid.SetColumn(_applyButton, 3);
        footer.Widgets.Add(_applyButton);

        return footer;
    }

    private Button CreateCategoryButton(SetupUiCategoryModel category)
    {
        var label = new Label
        {
            Text = $"{category.Title}\n{category.Tagline}",
            TextColor = CopyColor,
            Wrap = true,
        };

        var button = new Button
        {
            Background = Brush(PanelAltColor),
            HorizontalAlignment = HorizontalAlignment.Stretch,
            Content = label,
        };

        var buttonIndex = _categoryButtons.Count;
        button.Click += (_, _) =>
        {
            _selectedPane = SetupPane.Categories;
            _shell.SelectCategory(buttonIndex);
            _selectedFieldIndex = 0;
            _fieldScrollOffset = 0;
            RefreshCategoryContent();
            UpdateCategoryButtonStyles();
            if (_desktop != null)
            {
                _desktop.FocusedKeyboardWidget = button;
            }
        };

        _categoryButtons.Add((button, label));
        return button;
    }

    private Widget CreateFieldEditor(SetupUiFieldModel field, bool isSelected)
    {
        var panel = CreateCardPanel(width: 648, innerWidth: 616, background: isSelected ? PanelSelectedColor : PanelAltColor);
        var content = CreateCardContent();
        content.Width = 616;
        content.HorizontalAlignment = HorizontalAlignment.Center;
        content.VerticalAlignment = VerticalAlignment.Center;
        content.Spacing = 6;

        var header = new Grid
        {
            ColumnSpacing = 8,
        };
        header.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
        header.ColumnsProportions.Add(new Proportion(ProportionType.Auto));

        header.Widgets.Add(new Label
        {
            Text = field.Label,
            TextColor = TitleColor,
        });

        var currentValue = new Label
        {
            Text = field.DisplayValue,
            TextColor = isSelected ? TitleColor : ValueColor,
        };
        Grid.SetColumn(currentValue, 1);
        header.Widgets.Add(currentValue);

        content.Widgets.Add(header);
        content.Widgets.Add(CreateSectionBody(field.Description));
        content.Widgets.Add(new Label
        {
            Text = field.ApplyHint,
            TextColor = isSelected ? TitleColor : field.ApplyMode == VehicleSetupApplyMode.Live ? ValueColor : MutedColor,
        });

        var defaultRow = new Grid
        {
            ColumnSpacing = 8,
        };
        defaultRow.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
        defaultRow.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
        defaultRow.Widgets.Add(new Label
        {
            Text = $"Default • {field.DefaultDisplayValue}",
            TextColor = field.CanResetToDefault ? ValueColor : MutedColor,
            Wrap = true,
        });
        if (field.CanResetToDefault)
        {
            var resetButton = new Button
            {
                Background = Brush(isSelected ? AccentColor : AccentSoftColor),
                HorizontalAlignment = HorizontalAlignment.Right,
                Content = new Label
                {
                    Text = "Reset",
                    TextColor = ValueColor,
                },
            };
            resetButton.Click += (_, _) =>
            {
                field.ResetToDefaultValue();
                RefreshCategoryContent();
            };
            Grid.SetColumn(resetButton, 1);
            defaultRow.Widgets.Add(resetButton);
        }

        content.Widgets.Add(defaultRow);

        Widget control = field.EditorKind switch
        {
            SetupUiEditorKind.Numeric => CreateNumericEditor(field, isSelected),
            SetupUiEditorKind.Choice => CreateChoiceEditor(field, isSelected),
            SetupUiEditorKind.Toggle => CreateToggleEditor(field, isSelected),
            _ => new Label { Text = field.DisplayValue, TextColor = ValueColor },
        };
        content.Widgets.Add(control);

        panel.Widgets.Add(content);
        return panel;
    }

    private Widget CreateNumericEditor(SetupUiFieldModel field, bool isSelected)
    {
        var clampedValue = Math.Clamp(field.NumericValue, field.Minimum, field.Maximum);

        var editor = new Grid
        {
            ColumnSpacing = 10,
        };
        editor.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
        editor.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
        editor.ColumnsProportions.Add(new Proportion(ProportionType.Auto));

        var slider = new HorizontalSlider
        {
            Minimum = field.Minimum,
            Maximum = field.Maximum,
            Value = clampedValue,
            VerticalAlignment = VerticalAlignment.Center,
        };
        slider.ValueChangedByUser += (_, args) =>
        {
            field.SetNumericValue(args.NewValue);
            RefreshCategoryContent();
        };
        editor.Widgets.Add(slider);

        var spinButton = new SpinButton
        {
            Width = 92,
            Minimum = field.Minimum,
            Maximum = field.Maximum,
            Value = clampedValue,
            Increment = Math.Max(field.Step, 0.01f),
            DecimalPlaces = ResolveDecimalPlaces(field.Step),
            HorizontalAlignment = HorizontalAlignment.Right,
        };
        spinButton.ValueChangedByUser += (_, args) =>
        {
            if (args.NewValue.HasValue)
            {
                field.SetNumericValue(args.NewValue.Value);
                RefreshCategoryContent();
            }
        };
        Grid.SetColumn(spinButton, 1);
        editor.Widgets.Add(spinButton);

        var unitLabel = new Label
        {
            Text = field.Unit,
            TextColor = isSelected ? ValueColor : MutedColor,
            VerticalAlignment = VerticalAlignment.Center,
        };
        Grid.SetColumn(unitLabel, 2);
        editor.Widgets.Add(unitLabel);

        return editor;
    }

    private Widget CreateChoiceEditor(SetupUiFieldModel field, bool isSelected)
    {
        var choiceButton = new Button
        {
            Background = Brush(isSelected ? AccentColor : AccentSoftColor),
            HorizontalAlignment = HorizontalAlignment.Left,
            Content = new Label
            {
                Text = field.DisplayValue,
                TextColor = ValueColor,
            },
        };
        choiceButton.Click += (_, _) =>
        {
            field.CycleChoice();
            RefreshCategoryContent();
        };

        return choiceButton;
    }

    private Widget CreateToggleEditor(SetupUiFieldModel field, bool isSelected)
    {
        var toggleButton = new Button
        {
            Background = Brush(isSelected ? AccentColor : field.ToggleValue ? AccentSoftColor : PanelColor),
            HorizontalAlignment = HorizontalAlignment.Left,
            Content = new Label
            {
                Text = field.DisplayValue,
                TextColor = field.ToggleValue || isSelected ? ValueColor : CopyColor,
            },
        };
        toggleButton.Click += (_, _) =>
        {
            field.SetToggleValue(!field.ToggleValue);
            RefreshCategoryContent();
        };

        return toggleButton;
    }

    private void RefreshCategoryContent()
    {
        if (_editorStack == null || _summaryStack == null)
        {
            return;
        }

        ClampNavigationState();
        _editorStack.Widgets.Clear();
        _summaryStack.Widgets.Clear();

        var category = _shell.SelectedCategory;
        if (_categoryTitleLabel != null)
        {
            _categoryTitleLabel.Text = category.Title;
        }

        if (_categoryTaglineLabel != null)
        {
            _categoryTaglineLabel.Text = category.Tagline;
        }

        if (_categoryDescriptionLabel != null)
        {
            _categoryDescriptionLabel.Text = category.Description;
        }

        if (_fieldWindowLabel != null)
        {
            var first = category.Fields.Count == 0 ? 0 : _fieldScrollOffset + 1;
            var last = Math.Min(category.Fields.Count, _fieldScrollOffset + MaxVisibleEditorFields);
            _fieldWindowLabel.Text = category.Fields.Count == 0
                ? "No editable values in this category."
                : $"Showing {first}-{last} of {category.Fields.Count} values  •  D-Pad Left/Right adjusts";
        }

        var visibleFields = category.Fields
            .Skip(_fieldScrollOffset)
            .Take(MaxVisibleEditorFields)
            .ToList();

        for (var i = 0; i < visibleFields.Count; i++)
        {
            var fieldIndex = _fieldScrollOffset + i;
            var field = visibleFields[i];
            var isSelected = _selectedPane == SetupPane.Fields && fieldIndex == _selectedFieldIndex;
            _editorStack.Widgets.Add(CreateFieldEditor(field, isSelected));
            _summaryStack.Widgets.Add(CreateSummaryRow(field, isSelected));
        }

        UpdateFooterState();
    }

    private Widget CreateSummaryRow(SetupUiFieldModel field, bool isSelected)
    {
        var panel = CreateCardPanel(width: 272, innerWidth: 248, background: isSelected ? PanelSelectedColor : PanelAltColor);

        var row = new Grid
        {
            Width = 248,
            HorizontalAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
            ColumnSpacing = 8,
        };
        row.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
        row.ColumnsProportions.Add(new Proportion(ProportionType.Auto));

        row.Widgets.Add(new Label
        {
            Text = field.Label,
            TextColor = CopyColor,
            Wrap = true,
        });

        var value = new Label
        {
            Text = field.DisplayValue,
            TextColor = isSelected ? TitleColor : ValueColor,
        };
        Grid.SetColumn(value, 1);
        row.Widgets.Add(value);

        panel.Widgets.Add(row);
        return panel;
    }

    private void UpdateStaticLabels()
    {
        if (_vehicleNameLabel != null)
        {
            _vehicleNameLabel.Text = _shell.VehicleName;
        }

        if (_statusLabel != null)
        {
            _statusLabel.Text = _shell.StatusText;
        }

        UpdateFooterState();
    }

    private void UpdateCategoryButtonStyles()
    {
        for (var i = 0; i < _categoryButtons.Count; i++)
        {
            var (button, label) = _categoryButtons[i];
            var selected = i == _shell.SelectedCategoryIndex;
            button.Background = Brush(selected
                ? _selectedPane == SetupPane.Categories ? AccentColor : AccentSoftColor
                : PanelAltColor);
            label.TextColor = selected ? ValueColor : CopyColor;
        }
    }

    private void UpdateFooterState()
    {
        if (_pendingSummaryLabel != null)
        {
            var stateSummary = _shell.PendingChangeCount == 0
                ? (_shell.CanResetToDefaults
                    ? $"{_shell.NonDefaultChangeCount} value(s) off default  •  D-Pad Up/Down move  •  Left/Right adjust  •  Shoulders switch pane  •  A activate"
                    : "Setup synced  •  D-Pad Up/Down move  •  Left/Right adjust  •  Shoulders switch pane  •  A activate")
                : $"{_shell.PendingChangeCount} pending change(s) • {_shell.PendingLiveChangeCount} live • {_shell.PendingReloadChangeCount} reload";
            _pendingSummaryLabel.Text = stateSummary;
        }

        if (_applyButton != null)
        {
            _applyButton.Enabled = _shell.CanApply;
            _applyButton.Background = Brush(_selectedPane == SetupPane.Footer && _selectedFooterIndex == 2 ? AccentColor : AccentSoftColor);
        }

        if (_discardButton != null)
        {
            _discardButton.Enabled = _shell.CanApply;
            _discardButton.Background = Brush(_selectedPane == SetupPane.Footer && _selectedFooterIndex == 0 ? AccentColor : PanelAltColor);
        }

        if (_resetDefaultsButton != null)
        {
            _resetDefaultsButton.Enabled = _shell.CanResetToDefaults;
            _resetDefaultsButton.Background = Brush(_selectedPane == SetupPane.Footer && _selectedFooterIndex == 1 ? AccentColor : PanelAltColor);
        }
    }

    private void RebuildRoot()
    {
        _categoryButtons.Clear();
        _vehicleNameLabel = null;
        _statusLabel = null;
        _categoryTitleLabel = null;
        _categoryTaglineLabel = null;
        _categoryDescriptionLabel = null;
        _fieldWindowLabel = null;
        _editorStack = null;
        _summaryStack = null;
        _pendingSummaryLabel = null;
        _applyButton = null;
        _discardButton = null;
        _resetDefaultsButton = null;

        if (_desktop != null)
        {
            _desktop.Root = BuildRoot();
            if (_categoryButtons.Count > 0)
            {
                _desktop.FocusedKeyboardWidget = _categoryButtons[_shell.SelectedCategoryIndex].Button;
            }
        }
    }

    private void HandleNavigationInput()
    {
        if (_game == null)
        {
            return;
        }

        var pad = _game.Input.GamePads.FirstOrDefault();
        var moveUp = _game.Input.IsKeyPressed(Keys.Up) || pad?.IsButtonPressed(GamePadButton.PadUp) == true;
        var moveDown = _game.Input.IsKeyPressed(Keys.Down) || pad?.IsButtonPressed(GamePadButton.PadDown) == true;
        var moveLeft = _game.Input.IsKeyPressed(Keys.Left) || pad?.IsButtonPressed(GamePadButton.PadLeft) == true;
        var moveRight = _game.Input.IsKeyPressed(Keys.Right) || pad?.IsButtonPressed(GamePadButton.PadRight) == true;
        var previousPane = _game.Input.IsKeyPressed(Keys.PageUp) || pad?.IsButtonPressed(GamePadButton.LeftShoulder) == true;
        var nextPane = _game.Input.IsKeyPressed(Keys.PageDown) || pad?.IsButtonPressed(GamePadButton.RightShoulder) == true;
        var confirm = _game.Input.IsKeyPressed(Keys.Enter) || pad?.IsButtonPressed(GamePadButton.A) == true;

        if (previousPane)
        {
            MovePane(-1);
        }

        if (nextPane)
        {
            MovePane(1);
        }

        if (moveUp)
        {
            MoveWithinPane(-1);
        }

        if (moveDown)
        {
            MoveWithinPane(1);
        }

        if (moveLeft)
        {
            AdjustWithinPane(-1);
        }

        if (moveRight)
        {
            AdjustWithinPane(1);
        }

        if (confirm)
        {
            ActivateSelection();
        }
    }

    private void MovePane(int delta)
    {
        _selectedPane = delta switch
        {
            < 0 when _selectedPane == SetupPane.Footer => SetupPane.Fields,
            < 0 when _selectedPane == SetupPane.Fields => SetupPane.Categories,
            > 0 when _selectedPane == SetupPane.Categories => SetupPane.Fields,
            > 0 when _selectedPane == SetupPane.Fields => SetupPane.Footer,
            _ => _selectedPane,
        };

        RefreshCategoryContent();
        UpdateCategoryButtonStyles();
    }

    private void MoveWithinPane(int delta)
    {
        switch (_selectedPane)
        {
            case SetupPane.Categories:
                if (_categoryButtons.Count == 0)
                {
                    return;
                }

                _shell.SelectCategory(_shell.SelectedCategoryIndex + delta);
                _selectedFieldIndex = 0;
                _fieldScrollOffset = 0;
                RefreshCategoryContent();
                UpdateCategoryButtonStyles();
                if (_desktop != null && _categoryButtons.Count > 0)
                {
                    _desktop.FocusedKeyboardWidget = _categoryButtons[_shell.SelectedCategoryIndex].Button;
                }
                break;
            case SetupPane.Fields:
                if (_shell.SelectedCategory.Fields.Count == 0)
                {
                    return;
                }

                _selectedFieldIndex = Math.Clamp(_selectedFieldIndex + delta, 0, _shell.SelectedCategory.Fields.Count - 1);
                EnsureSelectedFieldVisible();
                RefreshCategoryContent();
                break;
            case SetupPane.Footer:
                _selectedFooterIndex = Math.Clamp(_selectedFooterIndex + delta, 0, FooterActionCount - 1);
                UpdateFooterState();
                break;
        }
    }

    private void AdjustWithinPane(int delta)
    {
        switch (_selectedPane)
        {
            case SetupPane.Categories:
            case SetupPane.Footer when delta < 0:
            case SetupPane.Footer when delta > 0 && _selectedFooterIndex == FooterActionCount - 1:
                MovePane(delta);
                break;
            case SetupPane.Fields:
                if (_shell.SelectedCategory.Fields.Count == 0)
                {
                    return;
                }

                var field = _shell.SelectedCategory.Fields[_selectedFieldIndex];
                switch (field.EditorKind)
                {
                    case SetupUiEditorKind.Numeric:
                        field.SetNumericValue(field.NumericValue + field.Step * delta);
                        RefreshCategoryContent();
                        break;
                    case SetupUiEditorKind.Choice:
                        field.CycleChoice(delta);
                        RefreshCategoryContent();
                        break;
                    case SetupUiEditorKind.Toggle:
                        field.SetToggleValue(!field.ToggleValue);
                        RefreshCategoryContent();
                        break;
                }
                break;
            case SetupPane.Footer:
                _selectedFooterIndex = Math.Clamp(_selectedFooterIndex + delta, 0, FooterActionCount - 1);
                UpdateFooterState();
                break;
        }
    }

    private void ActivateSelection()
    {
        switch (_selectedPane)
        {
            case SetupPane.Categories:
                _selectedPane = SetupPane.Fields;
                RefreshCategoryContent();
                UpdateCategoryButtonStyles();
                break;
            case SetupPane.Fields:
                AdjustWithinPane(1);
                break;
            case SetupPane.Footer:
                if (_selectedFooterIndex == 0)
                {
                    _shell.ResetPendingChanges();
                    RefreshCategoryContent();
                }
                else if (_selectedFooterIndex == 1)
                {
                    _shell.ResetToDefaults();
                    RefreshCategoryContent();
                }
                else
                {
                    var payload = _shell.CreateApplyPayload();
                    if (payload.Count > 0)
                    {
                        ApplyRequested?.Invoke(payload);
                    }
                }
                break;
        }
    }

    private void ResetNavigationState()
    {
        _selectedPane = SetupPane.Categories;
        _selectedFieldIndex = 0;
        _fieldScrollOffset = 0;
        _selectedFooterIndex = FooterActionCount - 1;
        ClampNavigationState();
    }

    private void ClampNavigationState()
    {
        var fieldCount = _shell.SelectedCategory.Fields.Count;
        if (fieldCount == 0)
        {
            _selectedFieldIndex = 0;
            _fieldScrollOffset = 0;
            if (_selectedPane == SetupPane.Fields)
            {
                _selectedPane = SetupPane.Footer;
            }
            return;
        }

        _selectedFieldIndex = Math.Clamp(_selectedFieldIndex, 0, fieldCount - 1);
        EnsureSelectedFieldVisible();
    }

    private void EnsureSelectedFieldVisible()
    {
        var fieldCount = _shell.SelectedCategory.Fields.Count;
        if (fieldCount <= 0)
        {
            _fieldScrollOffset = 0;
            return;
        }

        if (_selectedFieldIndex < _fieldScrollOffset)
        {
            _fieldScrollOffset = _selectedFieldIndex;
        }
        else if (_selectedFieldIndex >= _fieldScrollOffset + MaxVisibleEditorFields)
        {
            _fieldScrollOffset = _selectedFieldIndex - MaxVisibleEditorFields + 1;
        }

        _fieldScrollOffset = Math.Clamp(_fieldScrollOffset, 0, Math.Max(0, fieldCount - MaxVisibleEditorFields));
    }

    private static Label CreateSectionTitle(string text) => new()
    {
        Text = text,
        TextColor = TitleColor,
    };

    private static Label CreateSectionBody(string text) => new()
    {
        Text = text,
        TextColor = CopyColor,
        Wrap = true,
    };

    private static Panel CreateCardPanel(int width, int innerWidth, Color? background = null)
    {
        return new Panel
        {
            Width = width,
            Background = Brush(background ?? PanelColor),
        };
    }

    private static VerticalStackPanel CreateCardContent() => new()
    {
        Spacing = 8,
    };

    private static Panel CreateSpacer(int height) => new()
    {
        Height = height,
    };

    private static int ResolveDecimalPlaces(float step)
    {
        if (step >= 1f || step <= 0f)
        {
            return 0;
        }

        if (step >= 0.1f)
        {
            return 1;
        }

        return step >= 0.01f ? 2 : 3;
    }

    private static SolidBrush Brush(Color color) => new(color);
}
