using System;
using Myra;
using Myra.Graphics2D.Brushes;
using Myra.Graphics2D.UI;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using LibreRally.Vehicle;

namespace LibreRally.HUD
{
	/// <summary>
	/// Full-screen loading overlay with progress bar and status text, displayed during vehicle loading.
	/// </summary>
	public sealed class LoadingOverlay : GameSystemBase
	{
		private const int BarWidth = 400;
		private const int BarHeight = 24;

		private static readonly Color ShellColor = new(16, 22, 30, 242);
		private static readonly Color StatusColor = new(240, 243, 247, 255);
		private static readonly Color ProgressColor = new(214, 148, 78, 255);
		private static readonly Color BarBgColor = new(26, 32, 43, 236);

		private Game? _game;
		private Desktop? _desktop;
		private Panel? _barFill;
		private Label? _statusLabel;
		private Label? _titleLabel;
		private int _barWidth;

		/// <summary>
		/// Gets or sets the current loading stage description text.
		/// </summary>
		public string StatusText
		{
			get => _statusLabel?.Text ?? string.Empty;
			set { if (_statusLabel != null) _statusLabel.Text = value; }
		}

		/// <summary>
		/// Gets or sets the title text displayed above the progress bar.
		/// </summary>
		public string TitleText
		{
			get => _titleLabel?.Text ?? "Loading...";
			set { if (_titleLabel != null) _titleLabel.Text = value; }
		}

		/// <summary>
		/// Initializes a new instance of the <see cref="LoadingOverlay"/> class.
		/// </summary>
		/// <param name="services">The Stride service registry.</param>
		public LoadingOverlay(IServiceRegistry services) : base(services)
		{
			Enabled = true;
			Visible = true;
			DrawOrder = 9999;
			UpdateOrder = 9999;
		}

		public override void Initialize()
		{
			base.Initialize();
			_game = (Game?)Services.GetService<IGame>();
			if (_game == null) return;

			MyraEnvironment.Game = _game;
			_desktop = new Desktop { Background = null };
			_desktop.Root = BuildRoot();
		}

		/// <summary>
		/// Updates the progress bar fill width.
		/// </summary>
		/// <param name="progress">Progress value between 0 and 1.</param>
		public void SetProgress(float progress)
		{
			if (_barFill == null) return;
			var clamped = Math.Clamp(progress, 0f, 1f);
			_barFill.Width = (int)Math.Round(_barWidth * clamped);
		}

		protected override void Destroy()
		{
			_desktop?.Dispose();
			base.Destroy();
		}

		public override void Update(GameTime gameTime)
		{
			// Nothing to update - progress is pushed externally
		}

		public override void Draw(GameTime gameTime)
		{
			if (!Visible || !Enabled) return;
			if (_game == null || _desktop == null) return;
			var presenter = _game.GraphicsDevice.Presenter;
			if (presenter?.BackBuffer == null) return;

			var context = _game.GraphicsContext;
			context.CommandList.SetRenderTargetAndViewport(presenter.DepthStencilBuffer, presenter.BackBuffer);
			_desktop.Render();
		}

		private Widget BuildRoot()
		{
			var root = new Panel();

			var frame = new Panel
			{
				Width = 480,
				Height = 160,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Background = new SolidBrush(ShellColor),
			};

			var stack = new VerticalStackPanel
			{
				Width = BarWidth,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Spacing = 12,
			};

			_titleLabel = new Label
			{
				Text = "Loading...",
				TextColor = StatusColor,
				HorizontalAlignment = HorizontalAlignment.Center,
			};
			stack.Widgets.Add(_titleLabel);

			var barTrack = new Panel
			{
				Width = BarWidth,
				Height = BarHeight,
				Background = new SolidBrush(BarBgColor),
			};

			var barInner = new Panel
			{
				Width = BarWidth - 4,
				Height = BarHeight - 4,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Background = null,
			};

			_barFill = new Panel
			{
				Width = 0,
				Height = BarHeight - 4,
				HorizontalAlignment = HorizontalAlignment.Left,
				VerticalAlignment = VerticalAlignment.Center,
				Background = new SolidBrush(ProgressColor),
			};

			_barWidth = BarWidth - 4;
			barInner.Widgets.Add(_barFill);
			barTrack.Widgets.Add(barInner);
			stack.Widgets.Add(barTrack);

			_statusLabel = new Label
			{
				Text = "Preparing...",
				TextColor = StatusColor,
				HorizontalAlignment = HorizontalAlignment.Center,
			};
			stack.Widgets.Add(_statusLabel);

			frame.Widgets.Add(stack);
			root.Widgets.Add(frame);
			return root;
		}
	}
}
