using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using LibreRally.Race;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Physics;
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
	/// UI overlay providing real-time vehicle telemetry, speed, RPM, and other driving metrics.
	/// </summary>
	public sealed class DrivingHudOverlay : GameSystemBase
	{
		private static readonly string[] WheelNames = ["FL", "FR", "RL", "RR"];

		private static readonly Color ShellColor = new(16, 22, 30, 232);
		private static readonly Color PanelColor = new(26, 32, 43, 236);
		private static readonly Color PanelAltColor = new(19, 25, 34, 236);
		private static readonly Color AccentColor = new(214, 148, 78, 255);
		private static readonly Color AccentSoftColor = new(214, 148, 78, 84);
		private static readonly Color TitleColor = new(240, 243, 247, 255);
		private static readonly Color CopyColor = new(183, 193, 205, 255);
		private static readonly Color MutedColor = new(132, 145, 160, 255);
		private static readonly Color ValueColor = new(255, 230, 204, 255);
		private static readonly Color PositiveColor = new(72, 220, 112, 230);
		private static readonly Color WarningColor = new(244, 196, 68, 230);
		private static readonly Color DangerColor = new(255, 92, 70, 230);
		private static readonly Color InfoColor = new(118, 214, 255, 230);
		private static readonly Color BarTrackColor = new(16, 18, 22, 194);
		private static readonly Color BarInnerColor = new(42, 46, 54, 205);
		private static readonly ConcurrentDictionary<Color, SolidBrush> BrushCache = [];

		private sealed class HudBar
		{
			public required Panel Fill { get; init; }
			public required Label ValueLabel { get; init; }
			public int Width { get; init; }
		}

		/// <summary>Gets or sets the associated rally car component to display telemetry for.</summary>
		public RallyCarComponent? Car { get; set; }
		/// <summary>Gets or sets the associated race timer.</summary>
		public RaceTimer? Timer { get; set; }
		/// <summary>Gets or sets the associated race controller.</summary>
		public RaceController? RaceController { get; set; }
		/// <summary>Gets or sets the status text displayed in the debug overlay.</summary>
		public string StatusText { get; set; } = "Waiting for vehicle...";
		/// <summary>Gets or sets a value indicating whether the debug overlay is visible.</summary>
		public bool DebugOverlayVisible { get; set; } = true;

		private readonly List<Label> _wheelDebugLabels = [];
		private Game? _game;
		private Desktop? _desktop;
		private Panel? _debugFrame;
		private Label? _debugStatusLabel;
		private Label? _speedSummaryLabel;
		private Label? _bodySummaryLabel;
		private Label? _assistSummaryLabel;
		private Label? _inputSummaryLabel;
		private Label? _hudStatusLabel;
		private Label? _timerLabel;
		private Label? _speedValueLabel;
		private Label? _rpmValueLabel;
		private Label? _gearValueLabel;
		private Label? _shiftLabel;
		private Label? _handbrakeLampLabel;
		private Label? _tractionLampLabel;
		private Label? _speedUnitLabel;
		private Label? _rpmUnitLabel;
		private HudBar? _debugThrottleBar;
		private HudBar? _debugBrakeBar;
		private HudBar? _debugSteerBar;
		private HudBar? _hudThrottleBar;
		private HudBar? _hudBrakeBar;
		private HudBar? _boostBar;
		private HudBar? _tempBar;
		private HudBar? _fuelBar;
		private HudBar? _oilBar;

		/// <summary>
		/// Initializes a new instance of the <see cref="DrivingHudOverlay"/> class.
		/// </summary>
		/// <param name="services">The service registry.</param>
		public DrivingHudOverlay(IServiceRegistry services) : base(services)
		{
			Enabled = true;
			Visible = true;
			DrawOrder = 9989;
			UpdateOrder = 9989;
		}

		/// <summary>
		/// Initializes the driving HUD widgets and desktop root.
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
				Background = null,
				Root = BuildRoot(),
			};
			UpdateWidgets();
		}

		protected override void Destroy()
		{
			_desktop?.Dispose();
			base.Destroy();
		}

		/// <summary>
		/// Refreshes HUD widget values for the current frame.
		/// </summary>
		/// <param name="gameTime">Stride timing data for the current frame.</param>
		public override void Update(GameTime gameTime)
		{
			if (_desktop == null)
			{
				return;
			}

			UpdateWidgets();
		}

		/// <summary>
		/// Draws the driving HUD when the overlay is active.
		/// </summary>
		/// <param name="gameTime">Stride timing data for the current frame.</param>
		public override void Draw(GameTime gameTime)
		{
			if (_game == null || _desktop == null)
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
			var root = new Panel();
			root.Widgets.Add(BuildDebugFrame());
			root.Widgets.Add(BuildDrivingFrame());
			return root;
		}

		private Widget BuildDebugFrame()
		{
			_wheelDebugLabels.Clear();

			_debugFrame = new Panel
			{
				Width = 720,
				Height = 360,
				HorizontalAlignment = HorizontalAlignment.Left,
				VerticalAlignment = VerticalAlignment.Top,
				Background = Brush(ShellColor),
			};

			var content = new VerticalStackPanel
			{
				Width = 688,
				Height = 328,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Spacing = 10,
			};

			content.Widgets.Add(CreateSectionTitle("Vehicle Telemetry (F3)"));
			_debugStatusLabel = CreateBodyLabel(string.Empty);
			content.Widgets.Add(_debugStatusLabel);

			_speedSummaryLabel = CreateBodyLabel(string.Empty);
			_bodySummaryLabel = CreateBodyLabel(string.Empty);
			_assistSummaryLabel = CreateBodyLabel(string.Empty);
			_inputSummaryLabel = CreateBodyLabel(string.Empty);
			content.Widgets.Add(_speedSummaryLabel);
			content.Widgets.Add(_bodySummaryLabel);
			content.Widgets.Add(_assistSummaryLabel);
			content.Widgets.Add(_inputSummaryLabel);

			_debugThrottleBar = CreateBarRow(content, "Throttle", 260);
			_debugBrakeBar = CreateBarRow(content, "Brake", 260);
			_debugSteerBar = CreateBarRow(content, "Steer", 260);

			content.Widgets.Add(CreateSectionBody("g=grounded  sr=slip  load/Fx/Fy=kN  dt/bt/rt=Nm  om=rad/s"));
			for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
			{
				var label = CreateBodyLabel(string.Empty);
				_wheelDebugLabels.Add(label);
				content.Widgets.Add(label);
			}

			_debugFrame.Widgets.Add(content);
			return _debugFrame;
		}

		private Widget BuildDrivingFrame()
		{
			var frame = new Panel
			{
				Width = 516,
				Height = 314,
				HorizontalAlignment = HorizontalAlignment.Right,
				VerticalAlignment = VerticalAlignment.Bottom,
				Background = Brush(ShellColor),
			};

			var layout = new Grid
			{
				Width = 476,
				Height = 274,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				ColumnSpacing = 12,
				RowSpacing = 12,
			};
			layout.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
			layout.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
			layout.RowsProportions.Add(new Proportion(ProportionType.Auto));
			layout.RowsProportions.Add(new Proportion(ProportionType.Fill));
			layout.RowsProportions.Add(new Proportion(ProportionType.Auto));

			var header = new Grid
			{
				ColumnSpacing = 12,
			};
			header.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
			header.ColumnsProportions.Add(new Proportion(ProportionType.Auto));

			_timerLabel = new Label
			{
				TextColor = ValueColor,
			};
			header.Widgets.Add(_timerLabel);

			_hudStatusLabel = new Label
			{
				TextColor = CopyColor,
				HorizontalAlignment = HorizontalAlignment.Right,
			};
			Grid.SetColumn(_hudStatusLabel, 1);
			header.Widgets.Add(_hudStatusLabel);

			Grid.SetColumnSpan(header, 2);
			layout.Widgets.Add(header);

			var gauges = BuildGaugeColumns();
			Grid.SetRow(gauges, 1);
			layout.Widgets.Add(gauges);

			var gearCard = BuildGearCard();
			Grid.SetColumn(gearCard, 1);
			Grid.SetRow(gearCard, 1);
			layout.Widgets.Add(gearCard);

			var footer = new VerticalStackPanel
			{
				Spacing = 8,
			};
			footer.Widgets.Add(CreateSectionBody("Throttle / Brake"));
			_hudThrottleBar = CreateBarRow(footer, "Drive", 188);
			_hudBrakeBar = CreateBarRow(footer, "Brake", 188);
			footer.Widgets.Add(CreateSectionBody("Aux Systems"));
			_boostBar = CreateBarRow(footer, "Boost", 188);
			_tempBar = CreateBarRow(footer, "Temp", 188);
			_fuelBar = CreateBarRow(footer, "Fuel", 188);
			_oilBar = CreateBarRow(footer, "Oil", 188);

			Grid.SetColumnSpan(footer, 2);
			Grid.SetRow(footer, 2);
			layout.Widgets.Add(footer);

			frame.Widgets.Add(layout);
			return frame;
		}

		private Widget BuildGaugeColumns()
		{
			var grid = new Grid
			{
				ColumnSpacing = 12,
			};
			grid.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
			grid.ColumnsProportions.Add(new Proportion(ProportionType.Fill));

			var speedCard = CreateMetricCard("Speed", out _speedValueLabel, out _speedUnitLabel);
			grid.Widgets.Add(speedCard);

			var rpmCard = CreateMetricCard("Engine", out _rpmValueLabel, out _rpmUnitLabel);
			Grid.SetColumn(rpmCard, 1);
			grid.Widgets.Add(rpmCard);

			return grid;
		}

		private Widget BuildGearCard()
		{
			var panel = new Panel
			{
				Width = 128,
				Background = Brush(PanelColor),
			};

			var stack = new VerticalStackPanel
			{
				Width = 104,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Spacing = 8,
			};
			stack.Widgets.Add(CreateSectionBody("Gear"));

			_gearValueLabel = new Label
			{
				TextColor = TitleColor,
				HorizontalAlignment = HorizontalAlignment.Center,
			};
			stack.Widgets.Add(_gearValueLabel);

			_shiftLabel = new Label
			{
				TextColor = DangerColor,
				HorizontalAlignment = HorizontalAlignment.Center,
			};
			stack.Widgets.Add(_shiftLabel);

			_handbrakeLampLabel = CreateLampLabel();
			_tractionLampLabel = CreateLampLabel();
			stack.Widgets.Add(_handbrakeLampLabel);
			stack.Widgets.Add(_tractionLampLabel);

			panel.Widgets.Add(stack);
			return panel;
		}

		private Widget CreateMetricCard(string title, out Label? valueLabel, out Label? unitLabel)
		{
			var panel = new Panel
			{
				Background = Brush(PanelColor),
			};

			var stack = new VerticalStackPanel
			{
				Width = 150,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Spacing = 8,
			};
			stack.Widgets.Add(CreateSectionBody(title));

			valueLabel = new Label
			{
				TextColor = TitleColor,
				HorizontalAlignment = HorizontalAlignment.Center,
			};
			stack.Widgets.Add(valueLabel);

			unitLabel = new Label
			{
				TextColor = CopyColor,
				HorizontalAlignment = HorizontalAlignment.Center,
			};
			stack.Widgets.Add(unitLabel);

			panel.Widgets.Add(stack);
			return panel;
		}

		private HudBar CreateBarRow(VerticalStackPanel parent, string name, int width)
		{
			var row = new Grid
			{
				ColumnSpacing = 8,
			};
			row.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
			row.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
			row.ColumnsProportions.Add(new Proportion(ProportionType.Auto));

			row.Widgets.Add(new Label
			{
				Text = name,
				TextColor = CopyColor,
			});

			var track = new Panel
			{
				Width = width,
				Height = 12,
				Background = Brush(BarTrackColor),
			};
			Grid.SetColumn(track, 1);

			var inner = new Panel
			{
				Width = width - 2,
				Height = 10,
				HorizontalAlignment = HorizontalAlignment.Center,
				VerticalAlignment = VerticalAlignment.Center,
				Background = Brush(BarInnerColor),
			};
			var fill = new Panel
			{
				Width = 0,
				Height = 10,
				HorizontalAlignment = HorizontalAlignment.Left,
				VerticalAlignment = VerticalAlignment.Center,
				Background = Brush(PositiveColor),
			};

			inner.Widgets.Add(fill);
			track.Widgets.Add(inner);
			row.Widgets.Add(track);

			var valueLabel = new Label
			{
				TextColor = ValueColor,
				HorizontalAlignment = HorizontalAlignment.Right,
			};
			Grid.SetColumn(valueLabel, 2);
			row.Widgets.Add(valueLabel);

			parent.Widgets.Add(row);
			return new HudBar
			{
				Fill = fill,
				ValueLabel = valueLabel,
				Width = width - 2,
			};
		}

		private void UpdateWidgets()
		{
			if (_debugFrame != null)
			{
				_debugFrame.Visible = DebugOverlayVisible;
			}

			if (_debugStatusLabel != null)
			{
				_debugStatusLabel.Text = StatusText;
			}

			if (_hudStatusLabel != null)
			{
				_hudStatusLabel.Text = Car == null
					? "No vehicle  |  -- km/h  |  -- rpm"
					: $"Myra HUD  |  {Car.SpeedKmh:F0} km/h  |  {Car.EngineRpm:F0} rpm";
			}

			if (Car == null)
			{
				ApplyNoVehicleState();
				return;
			}

			UpdateRaceBanner();

			if (_speedValueLabel != null)
			{
				_speedValueLabel.Text = $"{Car.SpeedKmh:F0}";
			}

			if (_speedUnitLabel != null)
			{
				_speedUnitLabel.Text = "km/h";
			}

			if (_rpmValueLabel != null)
			{
				_rpmValueLabel.Text = $"{Car.EngineRpm / 1000f:F1}";
			}

			if (_rpmUnitLabel != null)
			{
				_rpmUnitLabel.Text = $"x1000 / {Math.Max(Car.MaxRpm, 0f) / 1000f:F1}";
			}

			if (_gearValueLabel != null)
			{
				_gearValueLabel.Text = FormatGear(Car.CurrentGear);
			}

			if (_shiftLabel != null)
			{
				_shiftLabel.Text = Car.EngineRpm > Car.MaxRpm * 0.92f ? "SHIFT!" : string.Empty;
			}

			UpdateLamp(_handbrakeLampLabel, Car.HandbrakeEngaged, "HANDBRAKE", DangerColor);
			UpdateTractionLamp(_tractionLampLabel, Car);

			UpdateBar(_hudThrottleBar, Car.ThrottleInput, PositiveColor, $"{Car.ThrottleInput:F2}");
			UpdateBar(_hudBrakeBar, Car.BrakeInput, DangerColor, $"{Car.BrakeInput:F2}");

			var boostMax = Math.Max(Car.TurboMaxBoostPsi * RallyCarComponent.PsiToBar, 0.01f);
			UpdateBar(_boostBar, Car.HasTurbo ? Car.TurboBoostBar / boostMax : 0f, InfoColor, Car.HasTurbo ? $"{Car.TurboBoostBar:F2} bar" : "--");
			UpdateBar(_tempBar, Car.EngineBlockTempDamageThreshold > 0f ? Car.EngineTemp / Car.EngineBlockTempDamageThreshold : 0f, GetTempColor(Car.EngineTemp, Car.EngineBlockTempDamageThreshold), $"{Car.EngineTemp:F0} C");
			var fuelFraction = Car.FuelCapacityLiters > 0f ? Car.FuelLiters / Car.FuelCapacityLiters : 0f;
			UpdateBar(_fuelBar, Car.FuelCapacityLiters > 0f ? fuelFraction : 0f, GetFuelColor(fuelFraction), Car.FuelCapacityLiters > 0f ? $"{fuelFraction * 100f:F0}%" : "--");
			var oilFraction = Math.Clamp(Car.OilPressure / 5f, 0f, 1f);
			UpdateBar(_oilBar, Car.OilVolumeLiters > 0f ? oilFraction : 0f, GetOilColor(oilFraction), Car.OilVolumeLiters > 0f ? $"{Car.OilPressure:F1} bar" : "--");

			if (_speedSummaryLabel != null)
			{
				_speedSummaryLabel.Text = $"Speed {Car.SpeedKmh,6:F1} km/h  |  Gear {FormatGear(Car.CurrentGear)}  |  Engine {Car.EngineRpm,6:F0} rpm  |  Driveline {Car.DrivelineRpm,6:F0} rpm";
			}

			if (_bodySummaryLabel != null)
			{
				_bodySummaryLabel.Text = $"Body fwd {Car.ForwardSpeedMs:F2} m/s  |  lat {Car.LateralSpeedMs:F2} m/s  |  yaw {Car.YawRateRad:F2} rad/s  |  HB {(Car.HandbrakeEngaged ? "ON" : "OFF")}";
			}

			if (_assistSummaryLabel != null)
			{
				_assistSummaryLabel.Text =
					$"Slip {Car.DrivenWheelSlipRatio:F2}  |  ABS {FormatAssistState(Car.AbsEnabled, Car.AbsActive)}  |  " +
					$"TCS {FormatTcsState(Car)}  |  scale {Car.TractionControlTorqueScale:F2}  |  " +
					$"{FormatDrivetrainTorque(Car.Dynamics)}";
			}

			if (_inputSummaryLabel != null)
			{
				_inputSummaryLabel.Text = $"Inputs D {Car.DriveInput:F2}  |  SB {Car.ServiceBrakeInput:F2}  |  T {Car.ThrottleInput:F2}  |  B {Car.BrakeInput:F2}  |  Steer {Car.SteeringInput:F2}";
			}

			UpdateBar(_debugThrottleBar, Car.ThrottleInput, PositiveColor, $"{Car.ThrottleInput:F2}");
			UpdateBar(_debugBrakeBar, Car.BrakeInput, DangerColor, $"{Car.BrakeInput:F2}");
			UpdateBar(_debugSteerBar, Math.Clamp((Car.SteeringInput + 1f) * 0.5f, 0f, 1f), InfoColor, $"{Car.SteeringInput:F2}");

			UpdateWheelDebug(Car.Dynamics);
		}

		private void ApplyNoVehicleState()
		{
			UpdateRaceBanner();
			if (_speedValueLabel != null)
			{
				_speedValueLabel.Text = "--";
			}

			if (_speedUnitLabel != null)
			{
				_speedUnitLabel.Text = "km/h";
			}

			if (_rpmValueLabel != null)
			{
				_rpmValueLabel.Text = "--";
			}

			if (_rpmUnitLabel != null)
			{
				_rpmUnitLabel.Text = "x1000";
			}

			if (_gearValueLabel != null)
			{
				_gearValueLabel.Text = "-";
			}

			if (_shiftLabel != null)
			{
				_shiftLabel.Text = string.Empty;
			}
			UpdateLamp(_handbrakeLampLabel, false, "HANDBRAKE", DangerColor);
			UpdateLamp(_tractionLampLabel, false, "TRACTION", WarningColor);

			UpdateBar(_hudThrottleBar, 0f, PositiveColor, "--");
			UpdateBar(_hudBrakeBar, 0f, DangerColor, "--");
			UpdateBar(_boostBar, 0f, InfoColor, "--");
			UpdateBar(_tempBar, 0f, MutedColor, "--");
			UpdateBar(_fuelBar, 0f, MutedColor, "--");
			UpdateBar(_oilBar, 0f, MutedColor, "--");
			UpdateBar(_debugThrottleBar, 0f, PositiveColor, "--");
			UpdateBar(_debugBrakeBar, 0f, DangerColor, "--");
			UpdateBar(_debugSteerBar, 0.5f, InfoColor, "--");

			if (_speedSummaryLabel != null)
			{
				_speedSummaryLabel.Text = "Telemetry unavailable: vehicle not loaded.";
			}

			if (_bodySummaryLabel != null)
			{
				_bodySummaryLabel.Text = string.Empty;
			}

			if (_assistSummaryLabel != null)
			{
				_assistSummaryLabel.Text = string.Empty;
			}

			if (_inputSummaryLabel != null)
			{
				_inputSummaryLabel.Text = string.Empty;
			}
			for (var i = 0; i < _wheelDebugLabels.Count; i++)
			{
				_wheelDebugLabels[i].Text = string.Empty;
			}
		}

		private void UpdateRaceBanner()
		{
			if (_timerLabel == null)
			{
				return;
			}

			if (RaceController != null && !RaceController.IsRacing && RaceController.CountdownRemaining > 0f)
			{
				_timerLabel.TextColor = WarningColor;
				_timerLabel.Text = $"GO IN: {RaceController.CountdownBeats}";
				return;
			}

			if (Timer != null)
			{
				_timerLabel.TextColor = ValueColor;
				_timerLabel.Text = Timer.GetTimeString();
				return;
			}

			_timerLabel.Text = string.Empty;
		}

		private void UpdateWheelDebug(VehicleDynamicsSystem? dynamics)
		{
			if (dynamics == null)
			{
				for (var i = 0; i < _wheelDebugLabels.Count; i++)
				{
					_wheelDebugLabels[i].Text = i == 0 ? "Wheel dynamics unavailable." : string.Empty;
				}

				return;
			}

			for (var i = 0; i < Math.Min(_wheelDebugLabels.Count, VehicleDynamicsSystem.WheelCount); i++)
			{
				_wheelDebugLabels[i].Text =
					$"{WheelNames[i]} g:{(dynamics.WheelGrounded[i] ? "Y" : "N")} " +
					$"sr:{dynamics.WheelStates[i].SlipRatio,5:F2} " +
					$"load:{dynamics.CurrentNormalLoads[i] / 1000f,5:F2} " +
					$"fx:{dynamics.LongitudinalForces[i] / 1000f,6:F2} " +
					$"fy:{dynamics.LateralForces[i] / 1000f,6:F2} " +
					$"dt:{dynamics.WheelDriveTorques[i],6:F0} " +
					$"bt:{dynamics.WheelBrakeTorques[i],6:F0} " +
					$"rt:{dynamics.WheelTyreReactionTorques[i],6:F0} " +
					$"om:{dynamics.WheelStates[i].AngularVelocity,6:F1}";
			}
		}

		private static void UpdateLamp(Label? label, bool active, string text, Color activeColor)
		{
			if (label == null)
			{
				return;
			}

			label.Text = text;
			label.TextColor = active ? TitleColor : MutedColor;
			label.Background = Brush(active ? activeColor : PanelAltColor);
		}

		private static void UpdateTractionLamp(Label? label, RallyCarComponent car)
		{
			if (label == null)
			{
				return;
			}

			if (car.TractionControlActive)
			{
				label.Text = "TCS ACTIVE";
				label.TextColor = TitleColor;
				label.Background = Brush(WarningColor);
				return;
			}

			if (car.TractionLossDetected)
			{
				label.Text = "SLIP";
				label.TextColor = TitleColor;
				label.Background = Brush(AccentColor);
				return;
			}

			label.Text = car.AbsEnabled || car.TractionControlEnabled ? "ASSISTS RDY" : "ASSISTS OFF";
			label.TextColor = CopyColor;
			label.Background = Brush(PanelAltColor);
		}

		private static string FormatDrivetrainTorque(VehicleDynamicsSystem? dynamics)
		{
			if (dynamics == null)
			{
				return "drv --/--";
			}

			return $"drv {dynamics.DeliveredDrivetrainTorque,6:F0}/{dynamics.RequestedDrivetrainTorque,6:F0}";
		}

		private static void UpdateBar(HudBar? bar, float fraction, Color color, string valueText)
		{
			if (bar == null)
			{
				return;
			}

			var clamped = Math.Clamp(fraction, 0f, 1f);
			bar.Fill.Width = (int)Math.Round(bar.Width * clamped);
			bar.Fill.Background = Brush(color);
			bar.ValueLabel.Text = valueText;
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

		private static Label CreateBodyLabel(string text) => new()
		{
			Text = text,
			TextColor = CopyColor,
			Wrap = true,
		};

		private static Label CreateLampLabel() => new()
		{
			TextColor = CopyColor,
			Background = Brush(PanelAltColor),
			HorizontalAlignment = HorizontalAlignment.Center,
		};

		private static string FormatGear(int currentGear) => currentGear <= 0 ? "R" : currentGear.ToString();

		private static string FormatAssistState(bool enabled, bool active) => !enabled ? "OFF" : active ? "ON" : "RDY";

		private static string FormatTcsState(RallyCarComponent car)
		{
			if (!car.TractionControlEnabled)
			{
				return "OFF";
			}

			return car.TractionControlActive ? "ON" : "RDY";
		}

		private static Color GetTempColor(float value, float dangerThreshold)
		{
			if (dangerThreshold <= 0f)
			{
				return MutedColor;
			}

			var fraction = value / dangerThreshold;
			if (fraction >= 0.85f)
			{
				return DangerColor;
			}

			if (fraction >= 0.7f)
			{
				return WarningColor;
			}

			return PositiveColor;
		}

		private static Color GetFuelColor(float fraction)
		{
			if (fraction <= 0.1f)
			{
				return DangerColor;
			}

			if (fraction <= 0.3f)
			{
				return WarningColor;
			}

			return PositiveColor;
		}

		private static Color GetOilColor(float fraction)
		{
			if (fraction <= 0.2f)
			{
				return DangerColor;
			}

			if (fraction >= 0.8f)
			{
				return WarningColor;
			}

			return PositiveColor;
		}

		private static SolidBrush Brush(Color color)
		{
			return BrushCache.GetOrAdd(color, static c => new SolidBrush(c));
		}
	}
}
