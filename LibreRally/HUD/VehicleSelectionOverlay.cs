using System;
using System.Collections.Generic;
using System.IO;
using Myra;
using Myra.Graphics2D.Brushes;
using Myra.Graphics2D.TextureAtlases;
using Myra.Graphics2D.UI;
using LibreRally.Vehicle.Content;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Graphics;
using StrideTextureImage = Stride.Graphics.Image;

namespace LibreRally.HUD
{
	/// <summary>
	/// UI overlay for selecting a vehicle from the catalog.
	/// </summary>
	public sealed class VehicleSelectionOverlay : GameSystemBase
	{
		private const int ButtonHeight = 116;
		private const int ButtonSpacing = 8;
		private const int ScrollViewportHeight = 442;
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

		private readonly List<(Button Button, Label Title, Label Detail, Label Description)> _buttons = [];
		private readonly Dictionary<string, Texture> _thumbnailTextures = new(StringComparer.OrdinalIgnoreCase);
		private readonly Dictionary<string, TextureRegion?> _thumbnailRegions = new(StringComparer.OrdinalIgnoreCase);
		private IReadOnlyList<BeamNgVehicleVariantDescriptor> _vehicles = [];
		private int _selectedIndex;
		private bool _overlayVisible;
		private string _statusText = string.Empty;
		private Game? _game;
		private Desktop? _desktop;
		private ScrollViewer? _listScrollViewer;
		private Label? _statusLabel;
		private Label? _selectionLabel;

		/// <summary>
		/// Gets or sets the list of vehicles available for selection.
		/// </summary>
		public IReadOnlyList<BeamNgVehicleVariantDescriptor> Vehicles
		{
			get => _vehicles;
			set
			{
				_vehicles = value;
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
			_desktop = new Desktop { Root = BuildRoot(), };
			FocusSelectedButton();
		}

		protected override void Destroy()
		{
			foreach (var texture in _thumbnailTextures.Values)
			{
				texture.Dispose();
			}

			_thumbnailRegions.Clear();
			_thumbnailTextures.Clear();
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
			_listScrollViewer = null;

			var root = new Panel { Background = BackdropBrush, };

			var shellFrame = new Panel
			{
				Width = 980,
				Height = 690,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Background = ShellBrush,
			};

			var shell = new VerticalStackPanel
			{
				Width = 920,
				Height = 630,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Spacing = 12,
			};

			shell.Widgets.Add(new Label { Text = "Vehicle Select", TextColor = TitleColor, });

			shell.Widgets.Add(new Label { Text = "Choose a bundled BeamNG vehicle variant.", TextColor = CopyColor, Wrap = true, });
			shell.Widgets.Add(GamePadPromptWidgets.CreatePromptStrip(
				_game,
				CopyColor,
				"D-Pad Up/Down select  •  A load  •  B/Menu back out",
				GamePadPromptWidgets.Prompt("Select", GamePadPromptIcon.DPadUp, GamePadPromptIcon.DPadDown),
				GamePadPromptWidgets.Prompt("Load", GamePadPromptIcon.A),
				GamePadPromptWidgets.Prompt("Back", GamePadPromptIcon.B, GamePadPromptIcon.Menu)));

			_selectionLabel = new Label { TextColor = ValueColor, Wrap = true, };
			shell.Widgets.Add(_selectionLabel);

			var listFrame = new Panel { Height = 470, Background = PanelBrush, };

			Widget listWidget;
			if (_vehicles.Count == 0)
			{
				listWidget = new Label
				{
					Text = "No vehicles found in Resources\\BeamNG Vehicles.", TextColor = TitleColor, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Center,
				};
			}
			else
			{
				var list = new VerticalStackPanel
				{
					Width = 860, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Top, Spacing = 8,
				};

				for (var index = 0; index < _vehicles.Count; index++)
				{
					list.Widgets.Add(CreateVehicleButton(_vehicles[index], index));
				}

				_listScrollViewer = new ScrollViewer { Content = list, Height = 442, VerticalAlignment = VerticalAlignment.Stretch, };
				listWidget = _listScrollViewer;
			}

			listFrame.Widgets.Add(listWidget);
			shell.Widgets.Add(listFrame);

			_statusLabel = new Label { TextColor = CopyColor, Wrap = true, };
			shell.Widgets.Add(_statusLabel);

			shellFrame.Widgets.Add(shell);
			root.Widgets.Add(shellFrame);

			UpdateSelectionStyles();
			UpdateLabels();
			return root;
		}

		private Button CreateVehicleButton(BeamNgVehicleVariantDescriptor vehicle, int index)
		{
			var preview = CreateThumbnailWidget(vehicle);
			var titleLabel = new Label
			{
				Text = vehicle.IsDefaultVariant
					? $"{vehicle.VariantDisplayName}  [default]"
					: vehicle.VariantDisplayName,
				TextColor = TitleColor,
			};
			var detailLabel = new Label
			{
				Text = string.IsNullOrWhiteSpace(vehicle.ConfigType)
					? $"{vehicle.VehicleDisplayName}  //  {vehicle.SourceLabel}"
					: $"{vehicle.VehicleDisplayName}  //  {vehicle.ConfigType}  //  {vehicle.SourceLabel}",
				TextColor = CopyColor,
				Wrap = true,
			};
			var descriptionLabel = new Label
			{
				Text = string.IsNullOrWhiteSpace(vehicle.Description)
					? $"Config: {vehicle.ConfigFileName ?? "<jbeam defaults>"}"
					: vehicle.Description,
				TextColor = CopyColor,
				Wrap = true,
			};

			var textContent = new VerticalStackPanel { Spacing = 4, };
			textContent.Widgets.Add(titleLabel);
			textContent.Widgets.Add(detailLabel);
			textContent.Widgets.Add(descriptionLabel);

			var content = new Grid { Width = 824, Height = 102, ColumnSpacing = 12, };
			content.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
			content.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
			content.RowsProportions.Add(new Proportion(ProportionType.Fill));
			content.Widgets.Add(preview);
			Grid.SetColumn(textContent, 1);
			content.Widgets.Add(textContent);

			var button = new Button { Height = ButtonHeight, HorizontalAlignment = HorizontalAlignment.Stretch, Content = content, };
			button.Click += (_, _) =>
			{
				SelectedIndex = index;
				ItemActivated?.Invoke(index);
			};

			_buttons.Add((button, titleLabel, detailLabel, descriptionLabel));
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
				var (button, title, detail, description) = _buttons[i];
				button.Background = isSelected ? SelectedItemBrush : UnselectedItemBrush;
				title.TextColor = isSelected ? ValueColor : TitleColor;
				detail.TextColor = isSelected ? ValueColor : CopyColor;
				description.TextColor = isSelected ? ValueColor : CopyColor;
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
			_selectionLabel.Text = $"Selected: {vehicle.VehicleDisplayName}  //  {vehicle.VariantDisplayName}";
		}

		private void FocusSelectedButton()
		{
			if (!_overlayVisible || _desktop == null || _buttons.Count == 0)
			{
				return;
			}

			var index = Math.Clamp(_selectedIndex, 0, _buttons.Count - 1);
			var button = _buttons[index].Button;
			_desktop.FocusedKeyboardWidget = button;
			EnsureSelectedButtonVisible();
		}

		private void EnsureSelectedButtonVisible()
		{
			if (_listScrollViewer == null)
			{
				return;
			}

			var scrollPosition = _listScrollViewer.ScrollPosition;
			var targetScroll = MenuScrollHelper.ComputeVisibleVerticalScrollForFixedList(
				scrollPosition.Y,
				ScrollViewportHeight,
				_selectedIndex,
				ButtonHeight,
				ButtonSpacing,
				_buttons.Count);
			if (targetScroll != scrollPosition.Y)
			{
				_listScrollViewer.ScrollPosition = new Point(scrollPosition.X, targetScroll);
			}
		}

		private Widget CreateThumbnailWidget(BeamNgVehicleVariantDescriptor vehicle)
		{
			var frame = new Panel { Width = 160, Height = 90, Background = UnselectedItemBrush, };

			var renderable = TryGetThumbnailRegion(vehicle.ThumbnailPath);
			if (renderable != null)
			{
				frame.Widgets.Add(new Myra.Graphics2D.UI.Image { Width = 160, Height = 90, Renderable = renderable, });
				return frame;
			}

			frame.Widgets.Add(new Label
			{
				Text = "No preview", TextColor = CopyColor, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Center,
			});
			return frame;
		}

		private TextureRegion? TryGetThumbnailRegion(string? thumbnailPath)
		{
			if (_game == null || string.IsNullOrWhiteSpace(thumbnailPath) || !File.Exists(thumbnailPath))
			{
				return null;
			}

			if (_thumbnailRegions.TryGetValue(thumbnailPath, out var cached))
			{
				return cached;
			}

			try
			{
				using var stream = File.OpenRead(thumbnailPath);
				using var image = StrideTextureImage.Load(stream);
				var texture = Texture.New(_game.GraphicsDevice, image);
				var region = new TextureRegion(texture);
				_thumbnailTextures[thumbnailPath] = texture;
				_thumbnailRegions[thumbnailPath] = region;
				return region;
			}
			catch
			{
				_thumbnailRegions[thumbnailPath] = null;
				return null;
			}
		}
	}
}
