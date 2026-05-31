using System;
using System.Collections.Generic;
using Myra;
using Myra.Graphics2D.Brushes;
using Myra.Graphics2D.UI;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;

namespace LibreRally.HUD
{
	/// <summary>
	/// UI overlay for switching between playable track and world environments.
	/// </summary>
	public sealed class TrackSelectionOverlay : GameSystemBase
	{
		private const int ButtonHeight = 96;
		private const int ButtonSpacing = 8;
		private const int ScrollViewportHeight = 360;
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

		private readonly List<(Button Button, Label Title, Label Description)> _buttons = [];
		private IReadOnlyList<PauseMenuItem> _items = [];
		private int _selectedIndex;
		private bool _overlayVisible;
		private string _statusText = string.Empty;
		private string _currentTrackName = string.Empty;
		private Game? _game;
		private Desktop? _desktop;
		private ScrollViewer? _listScrollViewer;
		private Label? _statusLabel;
		private Label? _selectionLabel;

		/// <summary>
		/// Gets or sets the track entries available for selection.
		/// </summary>
		public IReadOnlyList<PauseMenuItem> Items
		{
			get => _items;
			set
			{
				_items = value;
				RebuildRoot();
			}
		}

		/// <summary>
		/// Gets or sets the index of the currently selected track entry.
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
		/// Gets or sets a value indicating whether the overlay is visible.
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
		/// Gets or sets the current gameplay status text.
		/// </summary>
		public string StatusText
		{
			get => _statusText;
			set
			{
				_statusText = value;
				UpdateLabels();
			}
		}

		/// <summary>
		/// Gets or sets the label for the currently active track.
		/// </summary>
		public string CurrentTrackName
		{
			get => _currentTrackName;
			set
			{
				_currentTrackName = value;
				UpdateLabels();
			}
		}

		/// <summary>
		/// Occurs when a track is confirmed for loading.
		/// </summary>
		public Action<int>? ItemActivated { get; set; }

		/// <summary>
		/// Initializes a new instance of the <see cref="TrackSelectionOverlay"/> class.
		/// </summary>
		/// <param name="services">The service registry.</param>
		public TrackSelectionOverlay(IServiceRegistry services) : base(services)
		{
			Enabled = true;
			Visible = true;
			DrawOrder = 9995;
			UpdateOrder = 9995;
		}

		/// <summary>
		/// Initializes the desktop UI.
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

		/// <inheritdoc />
		protected override void Destroy()
		{
			_desktop?.Dispose();
			base.Destroy();
		}

		/// <summary>
		/// Draws the overlay when it is visible.
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
				Width = 920,
				Height = 620,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Background = ShellBrush,
			};
			var shell = new VerticalStackPanel
			{
				Width = 860,
				Height = 560,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Spacing = 12,
			};

			shell.Widgets.Add(new Label { Text = "Track Select", TextColor = TitleColor, });
			shell.Widgets.Add(new Label { Text = "Switch between the local test track and the configured Cesium stage.", TextColor = CopyColor, Wrap = true, });
			shell.Widgets.Add(GamePadPromptWidgets.CreatePromptStrip(
				_game,
				CopyColor,
				"D-Pad Up/Down select  •  A load  •  B back out",
				GamePadPromptWidgets.Prompt("Select", GamePadPromptIcon.DPadUp, GamePadPromptIcon.DPadDown),
				GamePadPromptWidgets.Prompt("Load", GamePadPromptIcon.A),
				GamePadPromptWidgets.Prompt("Back", GamePadPromptIcon.B)));

			_selectionLabel = new Label { TextColor = ValueColor, Wrap = true, };
			shell.Widgets.Add(_selectionLabel);

			var listFrame = new Panel { Height = 390, Background = PanelBrush, };
			Widget listWidget;
			if (_items.Count == 0)
			{
				listWidget = new Label
				{
					Text = "No track environments are available.",
					TextColor = TitleColor,
					HorizontalAlignment = HorizontalAlignment.Center,
					VerticalAlignment = VerticalAlignment.Center,
				};
			}
			else
			{
				var list = new VerticalStackPanel
				{
					Width = 800,
					HorizontalAlignment = HorizontalAlignment.Center,
					VerticalAlignment = VerticalAlignment.Top,
					Spacing = ButtonSpacing,
				};
				for (var index = 0; index < _items.Count; index++)
				{
					list.Widgets.Add(CreateTrackButton(_items[index], index));
				}

				_listScrollViewer = new ScrollViewer
				{
					Content = list,
					Height = ScrollViewportHeight,
					VerticalAlignment = VerticalAlignment.Stretch,
				};
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

		private Button CreateTrackButton(PauseMenuItem item, int index)
		{
			var titleLabel = new Label { Text = item.Title, TextColor = TitleColor, Wrap = true, };
			var descriptionLabel = new Label { Text = item.Description, TextColor = CopyColor, Wrap = true, };

			var content = new VerticalStackPanel
			{
				Spacing = 6,
			};
			content.Widgets.Add(titleLabel);
			content.Widgets.Add(descriptionLabel);

			var button = new Button
			{
				Content = content,
				Width = 800,
				Height = ButtonHeight,
				HorizontalAlignment = HorizontalAlignment.Center,
				Background = UnselectedItemBrush,
			};
			button.Click += (_, _) => ItemActivated?.Invoke(index);
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
			for (var index = 0; index < _buttons.Count; index++)
			{
				var isSelected = index == Math.Clamp(_selectedIndex, 0, Math.Max(0, _buttons.Count - 1));
				_buttons[index].Button.Background = isSelected ? SelectedItemBrush : UnselectedItemBrush;
			}
		}

		private void UpdateLabels()
		{
			if (_selectionLabel != null)
			{
				_selectionLabel.Text = string.IsNullOrWhiteSpace(CurrentTrackName)
					? "Current track: Unavailable"
					: $"Current track: {CurrentTrackName}";
			}

			if (_statusLabel != null)
			{
				_statusLabel.Text = string.IsNullOrWhiteSpace(StatusText) ? " " : StatusText;
			}
		}

		private void FocusSelectedButton()
		{
			if (!_overlayVisible || _desktop == null || _buttons.Count == 0)
			{
				return;
			}

			var clampedIndex = Math.Clamp(_selectedIndex, 0, _buttons.Count - 1);
			var button = _buttons[clampedIndex].Button;
			_desktop.FocusedKeyboardWidget = button;
		}
	}
}
