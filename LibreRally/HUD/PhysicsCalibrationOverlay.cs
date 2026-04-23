using System;
using System.Collections.Generic;
using System.Globalization;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Physics;
using Myra;
using Myra.Graphics2D.Brushes;
using Myra.Graphics2D.UI;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Input;

namespace LibreRally.HUD
{
	/// <summary>
	/// Identifies one editable parameter on the physics calibration page.
	/// Each entry owns a getter, a setter, and the label/description used in the UI.
	/// </summary>
	internal sealed class PhysCalField
	{
		/// <summary>Gets the stable field identifier.</summary>
		public string Id { get; init; } = string.Empty;
		/// <summary>Gets the display label.</summary>
		public string Label { get; init; } = string.Empty;
		/// <summary>Gets the detailed description, including the value source tag.</summary>
		public string Description { get; init; } = string.Empty;
		/// <summary>Gets the value read from the live car/tyre model.</summary>
		public Func<float> GetValue { get; init; } = static () => 0f;
		/// <summary>Writes a validated value back to the live car/tyre model.</summary>
		public Action<float> SetValue { get; init; } = static _ => { };
		/// <summary>Gets the minimum allowed value.</summary>
		public float Minimum { get; init; }
		/// <summary>Gets the maximum allowed value.</summary>
		public float Maximum { get; init; }
		/// <summary>Gets the UI step increment.</summary>
		public float Step { get; init; } = 0.01f;
		/// <summary>Gets the unit label.</summary>
		public string Unit { get; init; } = string.Empty;
	}

	/// <summary>
	/// One category of physics calibration fields.
	/// </summary>
	internal sealed class PhysCalCategory
	{
		/// <summary>Gets the category identifier.</summary>
		public string Id { get; init; } = string.Empty;
		/// <summary>Gets the display title.</summary>
		public string Title { get; init; } = string.Empty;
		/// <summary>Gets the short tagline.</summary>
		public string Tagline { get; init; } = string.Empty;
		/// <summary>Gets the detailed description shown when the category is selected.</summary>
		public string Description { get; init; } = string.Empty;
		/// <summary>Gets the list of fields in this category.</summary>
		public IReadOnlyList<PhysCalField> Fields { get; init; } = [];
	}

	/// <summary>
	/// In-pause physics calibration overlay.
	/// Lets developers select the active tyre force model (Auto / Brush Only / Pacejka Only)
	/// and tune the key tyre, TCS, ABS / ECS, and launch-control parameters live without
	/// reloading the vehicle.
	///
	/// <para>Source annotations in field descriptions indicate whether a value is loaded from
	/// JBeam or is a fixed default coded in the simulation ("magic number").</para>
	/// </summary>
	public sealed class PhysicsCalibrationOverlay : GameSystemBase
	{
		// ── Colour palette (matches the rest of the HUD family) ─────────────────
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

		private readonly Dictionary<Color, SolidBrush> _brushCache = [];

		// ── State ────────────────────────────────────────────────────────────────
		private Game? _game;
		private Desktop? _desktop;
		private bool _overlayVisible;
		private RallyCarComponent? _car;
		private int _selectedCategoryIndex;
		private int _selectedFieldIndex;
		private int _fieldScrollOffset;

		private IReadOnlyList<PhysCalCategory> _categories = [];

		// ── Cached label references for live updates ─────────────────────────────
		private Label? _vehicleNameLabel;
		private Label? _statusLabel;
		private Label? _tyreModelBadge;
		private VerticalStackPanel? _editorStack;
		private Label? _categoryTitleLabel;
		private Label? _categoryTaglineLabel;
		private Label? _categoryDescriptionLabel;
		private Label? _fieldWindowLabel;
		private Label? _suspensionTelemetryLabel;
		private readonly List<(Button Button, Label Label)> _categoryButtons = [];

		private const int MaxVisibleFields = 3;
		private const float MetersToMillimeters = 1000f;
		private const float NewtonsToKilonewtons = 1000f;

		// ── Public API ───────────────────────────────────────────────────────────

		/// <summary>Gets or sets a value indicating whether this overlay is visible.</summary>
		public bool OverlayVisible
		{
			get => _overlayVisible;
			set
			{
				_overlayVisible = value;
				if (_overlayVisible && _desktop != null && _categoryButtons.Count > 0)
				{
					_desktop.FocusedKeyboardWidget = _categoryButtons[_selectedCategoryIndex].Button;
				}
			}
		}

		private string _vehicleName = "LibreRally";
		private string _statusText = string.Empty;

		/// <summary>Gets or sets the vehicle name shown in the header.</summary>
		public string VehicleName
		{
			get => _vehicleName;
			set
			{
				var newValue = value ?? "LibreRally";
				if (string.Equals(_vehicleName, newValue, StringComparison.Ordinal))
				{
					return;
				}

				_vehicleName = newValue;
				UpdateStaticLabels();
			}
		}

		/// <summary>Gets or sets the status text shown in the summary card.</summary>
		public string StatusText
		{
			get => _statusText;
			set
			{
				var newValue = value ?? string.Empty;
				if (string.Equals(_statusText, newValue, StringComparison.Ordinal))
				{
					return;
				}

				_statusText = newValue;
				UpdateStaticLabels();
			}
		}

		/// <summary>Raised when the user requests to close this overlay (Esc / B / Start).</summary>
		public Action? CloseRequested { get; set; }

		/// <summary>
		/// Initializes a new instance of <see cref="PhysicsCalibrationOverlay"/>.
		/// </summary>
		/// <param name="services">The Stride service registry.</param>
		public PhysicsCalibrationOverlay(IServiceRegistry services) : base(services)
		{
			Enabled = true;
			Visible = true;
			DrawOrder = 9998;
			UpdateOrder = 9998;
		}

		/// <summary>Initializes the overlay desktop and builds the initial (empty) UI.</summary>
		public override void Initialize()
		{
			base.Initialize();
			_game = (Game?)Services.GetService<IGame>();
			if (_game == null)
			{
				return;
			}

			MyraEnvironment.Game = _game;
			_desktop = new Desktop { Root = BuildRoot() };
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

		/// <summary>
		/// Binds a loaded vehicle to the overlay so its live parameters can be tuned.
		/// Call this whenever a new vehicle is loaded or the overlay is opened.
		/// </summary>
		/// <param name="car">The active rally car component.</param>
		public void BindVehicle(RallyCarComponent? car)
		{
			_car = car;
			_categories = BuildCategories(car);
			_selectedCategoryIndex = 0;
			_selectedFieldIndex = 0;
			_fieldScrollOffset = 0;
			RebuildRoot();
		}

		/// <summary>Handles navigation input for the physics calibration overlay.</summary>
		/// <param name="gameTime">Stride frame timing.</param>
		public override void Update(GameTime gameTime)
		{
			if (!OverlayVisible || _game == null)
			{
				return;
			}

			HandleNavigationInput();
			UpdateLiveTelemetryLabels();
		}

		/// <summary>Draws the overlay when visible.</summary>
		/// <param name="gameTime">Stride frame timing.</param>
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

			_game.GraphicsContext.CommandList.SetRenderTargetAndViewport(
				presenter.DepthStencilBuffer,
				presenter.BackBuffer);
			_desktop.Render();
		}

		// ── Category / field builder ─────────────────────────────────────────────

		private static IReadOnlyList<PhysCalCategory> BuildCategories(RallyCarComponent? car)
		{
			if (car?.Dynamics == null)
			{
				return [];
			}

			return
			[
				BuildTyreModelCategory(car),
				BuildBrushCategory(car),
				BuildPacejkaCategory(car),
				BuildTcsCategory(car),
				BuildAbsEcsCategory(car),
				BuildLaunchControlCategory(car),
			];
		}

		private static PhysCalCategory BuildTyreModelCategory(RallyCarComponent car)
		{
			// Tyre model selection is handled via the choice field on the first category.
			// Each wheel shares the same ActiveMode so we read/write the FL tyre model.
			return new PhysCalCategory
			{
				Id = "tyre-model",
				Title = "Tyre Model",
				Tagline = "Isolation switch",
				Description = "Switch the active force model to isolate brush or Pacejka contributions. " +
				              "Auto blends both based on speed and slip.",
				Fields = BuildTyreCommonFields(car),
			};
		}

		private static IReadOnlyList<PhysCalField> BuildTyreCommonFields(RallyCarComponent car)
		{
			var tyreFL = GetTyreModel(car, VehicleDynamicsSystem.FL);
			if (tyreFL == null)
			{
				return [];
			}

			return
			[
				new PhysCalField
				{
					Id = "peak-friction",
					Label = "Peak friction coefficient",
					Description = "µ peak on the reference surface at the reference load. " +
					              "Scales all grip — lower = less traction everywhere. " +
					              "(magic number — default 1.05)",
					GetValue = () => tyreFL.PeakFrictionCoefficient,
					SetValue = v => ApplyToAllTyres(car, t => t.PeakFrictionCoefficient = v),
					Minimum = 0.4f,
					Maximum = 2.0f,
					Step = 0.01f,
					Unit = "µ",
				},
				new PhysCalField
				{
					Id = "load-sensitivity",
					Label = "Load sensitivity",
					Description = "Non-linear grip reduction as vertical load rises. " +
					              "Higher → grip falls away faster under heavy cornering loads. " +
					              "(magic number — default 0.15)",
					GetValue = () => tyreFL.LoadSensitivity,
					SetValue = v => ApplyToAllTyres(car, t => t.LoadSensitivity = v),
					Minimum = 0f,
					Maximum = 0.40f,
					Step = 0.005f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "friction-ellipse-ratio",
					Label = "Friction ellipse ratio",
					Description = "Ratio of lateral grip limit to longitudinal grip limit. " +
					              "1.0 = circular friction envelope, <1 = more longitudinal than lateral grip. " +
					              "(magic number — default 1.0)",
					GetValue = () => tyreFL.FrictionEllipseRatio,
					SetValue = v => ApplyToAllTyres(car, t => t.FrictionEllipseRatio = v),
					Minimum = 0.5f,
					Maximum = 1.5f,
					Step = 0.01f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "tyre-pressure",
					Label = "Tyre pressure (all wheels)",
					Description = "Inflation pressure applied to every wheel. " +
					              "Affects contact-patch size, carcass stiffness, and sidewall response. " +
					              "(from JBeam / setup overrides — default 220 kPa)",
					GetValue = () => tyreFL.TyrePressure,
					SetValue = v => ApplyToAllTyres(car, t => t.TyrePressure = v),
					Minimum = 100f,
					Maximum = 400f,
					Step = 5f,
					Unit = "kPa",
				},
				new PhysCalField
				{
					Id = "sidewall-stiffness",
					Label = "Sidewall stiffness",
					Description = "Multiplier on lateral cornering-stiffness B. " +
					              "Increase for sharper turn-in; decrease for softer, more compliant response. " +
					              "(magic number — default 1.0)",
					GetValue = () => tyreFL.SidewallStiffness,
					SetValue = v => ApplyToAllTyres(car, t => t.SidewallStiffness = v),
					Minimum = 0.3f,
					Maximum = 3.0f,
					Step = 0.05f,
					Unit = "×",
				},
				new PhysCalField
				{
					Id = "carcass-stiffness",
					Label = "Carcass stiffness",
					Description = "Multiplier reducing transient relaxation delay. " +
					              "Increase for a more direct, instant force buildup; decrease for more lag. " +
					              "(magic number — default 1.0)",
					GetValue = () => tyreFL.CarcassStiffness,
					SetValue = v => ApplyToAllTyres(car, t => t.CarcassStiffness = v),
					Minimum = 0.3f,
					Maximum = 3.0f,
					Step = 0.05f,
					Unit = "×",
				},
				new PhysCalField
				{
					Id = "high-slip-retention",
					Label = "High-slip force retention",
					Description = "Fraction of peak lateral force kept at large slip angles (rally drift support). " +
					              "Higher → car maintains cornering force at extreme attitude angles. " +
					              "(magic number — default 0.65)",
					GetValue = () => tyreFL.HighSlipForceRetention,
					SetValue = v => ApplyToAllTyres(car, t => t.HighSlipForceRetention = v),
					Minimum = 0f,
					Maximum = 1f,
					Step = 0.01f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "pneumatic-trail",
					Label = "Pneumatic trail",
					Description = "Zero-slip pneumatic trail used as the baseline self-aligning torque lever arm. " +
					              "Higher = heavier steering build-up before the limit. " +
					              "(magic number — default 0.025 m)",
					GetValue = () => tyreFL.PneumaticTrail,
					SetValue = v => ApplyToAllTyres(car, t => t.PneumaticTrail = v),
					Minimum = 0f,
					Maximum = 0.08f,
					Step = 0.001f,
					Unit = "m",
				},
				new PhysCalField
				{
					Id = "aligning-torque-residual",
					Label = "Aligning torque residual",
					Description = "Small residual Mz retained after the pneumatic trail collapses. " +
					              "Higher = more steering torque remains near the limit. " +
					              "(magic number — default 0.08)",
					GetValue = () => tyreFL.AligningTorqueResidualFactor,
					SetValue = v => ApplyToAllTyres(car, t => t.AligningTorqueResidualFactor = v),
					Minimum = 0f,
					Maximum = 0.25f,
					Step = 0.005f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "aligning-torque-fx-arm",
					Label = "Fx aligning moment arm",
					Description = "Longitudinal-force moment arm used in combined-slip aligning torque while cornering. " +
					              "Higher = stronger brake/drive influence on steering torque. " +
					              "(magic number — default 0.008 m)",
					GetValue = () => tyreFL.AligningTorqueFxMomentArm,
					SetValue = v => ApplyToAllTyres(car, t => t.AligningTorqueFxMomentArm = v),
					Minimum = 0f,
					Maximum = 0.04f,
					Step = 0.001f,
					Unit = "m",
				},
				new PhysCalField
				{
					Id = "overturning-couple-factor",
					Label = "Overturning couple",
					Description = "Scale factor for tyre overturning couple Mx from lateral load and camber. " +
					              "Higher = stronger wheel-axis roll moment reaction into the chassis. " +
					              "(magic number — default 0.015)",
					GetValue = () => tyreFL.OverturningCoupleFactor,
					SetValue = v => ApplyToAllTyres(car, t => t.OverturningCoupleFactor = v),
					Minimum = 0f,
					Maximum = 0.05f,
					Step = 0.001f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "overturning-camber-factor",
					Label = "Overturning camber sensitivity",
					Description = "Additional camber contribution into tyre overturning couple Mx. " +
					              "Higher = cambered tyres feed more roll moment into the chassis. " +
					              "(magic number — default 0.35)",
					GetValue = () => tyreFL.OverturningCamberFactor,
					SetValue = v => ApplyToAllTyres(car, t => t.OverturningCamberFactor = v),
					Minimum = 0f,
					Maximum = 1.5f,
					Step = 0.025f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "rolling-resistance-moment-factor",
					Label = "Rolling resistance moment",
					Description = "Scale factor for tyre rolling-resistance moment My about the wheel axle. " +
					              "Higher = stronger rolling drag moment reaction into the chassis. " +
					              "(magic number — default 0.006)",
					GetValue = () => tyreFL.RollingResistanceMomentFactor,
					SetValue = v => ApplyToAllTyres(car, t => t.RollingResistanceMomentFactor = v),
					Minimum = 0f,
					Maximum = 0.03f,
					Step = 0.001f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "rolling-resistance-moment-fx-factor",
					Label = "Rolling moment Fx coupling",
					Description = "Additional drive/brake-force coupling into rolling-resistance moment My. " +
					              "Higher = longitudinal tyre force changes axle reaction more strongly. " +
					              "(magic number — default 0.08)",
					GetValue = () => tyreFL.RollingResistanceMomentFxFactor,
					SetValue = v => ApplyToAllTyres(car, t => t.RollingResistanceMomentFxFactor = v),
					Minimum = 0f,
					Maximum = 0.4f,
					Step = 0.01f,
					Unit = "",
				},
			];
		}

		private static PhysCalCategory BuildBrushCategory(RallyCarComponent car)
		{
			var tyreFL = GetTyreModel(car, VehicleDynamicsSystem.FL);
			if (tyreFL == null)
			{
				return new PhysCalCategory { Id = "brush", Title = "Brush Model", Tagline = "No tyre", Description = "No tyre model found.", Fields = [] };
			}

			return new PhysCalCategory
			{
				Id = "brush",
				Title = "Brush Model",
				Tagline = "Contact-patch transients",
				Description = "Contact-patch deflection parameters. " +
				              "These control how quickly lateral and longitudinal forces build up from rest. " +
				              "All values are magic numbers unless specified.",
				Fields =
				[
					new PhysCalField
					{
						Id = "brush-patch-stiffness",
						Label = "Contact patch stiffness",
						Description = "Brush spring rate (N/m) at reference pressure and width. " +
						              "Higher = stiffer, faster force buildup in the brush model. " +
						              "(magic number — default 65000 N/m)",
						GetValue = () => tyreFL.ContactPatchStiffness,
						SetValue = v => ApplyToAllTyres(car, t => t.ContactPatchStiffness = v),
						Minimum = 5000f,
						Maximum = 200000f,
						Step = 1000f,
						Unit = "N/m",
					},
					new PhysCalField
					{
						Id = "brush-patch-damping",
						Label = "Contact patch damping",
						Description = "Lateral damping of the brush contact patch (N·s/m). " +
						              "Reduces oscillations at low speed. " +
						              "(magic number — default 4500 N·s/m)",
						GetValue = () => tyreFL.ContactPatchDamping,
						SetValue = v => ApplyToAllTyres(car, t => t.ContactPatchDamping = v),
						Minimum = 500f,
						Maximum = 20000f,
						Step = 250f,
						Unit = "N·s/m",
					},
					new PhysCalField
					{
						Id = "lateral-relaxation",
						Label = "Lateral relaxation length",
						Description = "Baseline distance the tyre travels before lateral force reaches steady state (m). " +
						              "The live value is also scaled by surface grip and local force-curve slope. " +
						              "(magic number — default 0.30 m)",
						GetValue = () => tyreFL.RelaxationLength,
						SetValue = v => ApplyToAllTyres(car, t => t.RelaxationLength = v),
						Minimum = 0.05f,
						Maximum = 2.0f,
						Step = 0.01f,
						Unit = "m",
					},
					new PhysCalField
					{
						Id = "longitudinal-relaxation",
						Label = "Longitudinal relaxation length",
						Description = "Baseline distance for drive/brake force to reach steady state (m). " +
						              "The live value is also scaled by surface grip and local force-curve slope. " +
						              "(magic number — default 0.28 m)",
						GetValue = () => tyreFL.LongitudinalRelaxationLength,
						SetValue = v => ApplyToAllTyres(car, t => t.LongitudinalRelaxationLength = v),
						Minimum = 0.05f,
						Maximum = 1.5f,
						Step = 0.01f,
						Unit = "m",
					},
					new PhysCalField
					{
						Id = "lateral-surface-relaxation-sensitivity",
						Label = "Lateral surface relaxation sensitivity",
						Description = "How much loose/low-grip surfaces extend the lateral relaxation length. " +
						              "Higher → more sluggish turn-in on gravel/snow. " +
						              "(magic number — default 0.6)",
						GetValue = () => tyreFL.LateralSurfaceRelaxationSensitivity,
						SetValue = v => ApplyToAllTyres(car, t => t.LateralSurfaceRelaxationSensitivity = v),
						Minimum = 0f,
						Maximum = 2f,
						Step = 0.05f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "long-surface-relaxation-sensitivity",
						Label = "Longitudinal surface relaxation sensitivity",
						Description = "How much loose surfaces delay drive/brake force buildup. " +
						              "Higher → more torque delay on gravel and snow. " +
						              "(magic number — default 0.85)",
						GetValue = () => tyreFL.LongitudinalSurfaceRelaxationSensitivity,
						SetValue = v => ApplyToAllTyres(car, t => t.LongitudinalSurfaceRelaxationSensitivity = v),
						Minimum = 0f,
						Maximum = 2f,
						Step = 0.05f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "combined-slip-coupling",
						Label = "Combined-slip coupling",
						Description = "MF-style cross-slip weighting strength for Fx/Fy interaction. " +
						              "Higher = the other slip channel reduces force sooner. " +
						              "(magic number — default 1.0)",
						GetValue = () => tyreFL.CombinedSlipCoupling,
						SetValue = v => ApplyToAllTyres(car, t => t.CombinedSlipCoupling = v),
						Minimum = 0f,
						Maximum = 2f,
						Step = 0.05f,
						Unit = "",
					},
				],
			};
		}

		private static PhysCalCategory BuildPacejkaCategory(RallyCarComponent car)
		{
			var tyreFL = GetTyreModel(car, VehicleDynamicsSystem.FL);
			if (tyreFL == null)
			{
				return new PhysCalCategory { Id = "pacejka", Title = "Pacejka", Tagline = "No tyre", Description = "No tyre model found.", Fields = [] };
			}

			return new PhysCalCategory
			{
				Id = "pacejka",
				Title = "Pacejka",
				Tagline = "Magic Formula coefficients",
				Description = "Pacejka Magic Formula: F = D·sin(C·atan(B·x − E·(B·x − atan(B·x)))). " +
				              "B = stiffness, C = shape, D = peak (from µ·Fz), E = curvature. " +
				              "All B/C/E coefficients are magic numbers.",
				Fields =
				[
					new PhysCalField
					{
						Id = "long-b",
						Label = "Longitudinal B (stiffness factor)",
						Description = "Longitudinal Pacejka stiffness factor. " +
						              "Higher = peak force at a smaller slip ratio; sharper drive/brake response. " +
						              "(magic number — default 12.0)",
						GetValue = () => tyreFL.LongitudinalB,
						SetValue = v => ApplyToAllTyres(car, t => t.LongitudinalB = v),
						Minimum = 2f,
						Maximum = 30f,
					Step = 0.1f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "long-b-load-sensitivity",
					Label = "Longitudinal B load sensitivity",
					Description = "Load sensitivity for longitudinal pure-slip stiffness evaluation. " +
					              "Higher = drive/brake stiffness rises more with vertical load. " +
					              "(magic number — default 0.04)",
					GetValue = () => tyreFL.LongitudinalLoadStiffnessSensitivity,
					SetValue = v => ApplyToAllTyres(car, t => t.LongitudinalLoadStiffnessSensitivity = v),
					Minimum = -0.5f,
					Maximum = 0.5f,
					Step = 0.01f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "long-c",
					Label = "Longitudinal C (shape factor)",
						Description = "Pacejka shape factor for the longitudinal curve. " +
						              "Typical range 1.5–1.8; higher rounds the peak less sharply. " +
						              "(magic number — default 1.65)",
						GetValue = () => tyreFL.LongitudinalC,
						SetValue = v => ApplyToAllTyres(car, t => t.LongitudinalC = v),
						Minimum = 0.5f,
						Maximum = 3f,
						Step = 0.01f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "long-e",
						Label = "Longitudinal E (curvature factor)",
						Description = "Pacejka curvature factor for longitudinal. " +
						              "Negative values extend force past the peak (realistic slip-curve tail). " +
						              "(magic number — default −0.5)",
						GetValue = () => tyreFL.LongitudinalE,
						SetValue = v => ApplyToAllTyres(car, t => t.LongitudinalE = v),
						Minimum = -5f,
						Maximum = 1f,
						Step = 0.05f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "lat-b",
						Label = "Lateral B (stiffness factor)",
						Description = "Lateral Pacejka stiffness factor. " +
						              "Higher = peak cornering force at a smaller slip angle; sharper steering response. " +
						              "(magic number — default 10.0)",
						GetValue = () => tyreFL.LateralB,
						SetValue = v => ApplyToAllTyres(car, t => t.LateralB = v),
						Minimum = 2f,
						Maximum = 30f,
					Step = 0.1f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "lat-b-load-sensitivity",
					Label = "Lateral B load sensitivity",
					Description = "Load sensitivity for lateral pure-slip stiffness evaluation. " +
					              "Higher = cornering stiffness rises more with vertical load. " +
					              "(magic number — default 0.06)",
					GetValue = () => tyreFL.LateralLoadStiffnessSensitivity,
					SetValue = v => ApplyToAllTyres(car, t => t.LateralLoadStiffnessSensitivity = v),
					Minimum = -0.5f,
					Maximum = 0.5f,
					Step = 0.01f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "lat-c",
					Label = "Lateral C (shape factor)",
						Description = "Pacejka shape factor for the lateral curve. " +
						              "Typical range 1.1–1.4 for passenger/rally tyres. " +
						              "(magic number — default 1.3)",
						GetValue = () => tyreFL.LateralC,
						SetValue = v => ApplyToAllTyres(car, t => t.LateralC = v),
						Minimum = 0.5f,
						Maximum = 3f,
						Step = 0.01f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "lat-e",
						Label = "Lateral E (curvature factor)",
						Description = "Pacejka curvature factor for lateral. " +
						              "Negative values let force tail off past the peak slip angle. " +
						              "(magic number — default −0.6)",
						GetValue = () => tyreFL.LateralE,
						SetValue = v => ApplyToAllTyres(car, t => t.LateralE = v),
						Minimum = -5f,
						Maximum = 1f,
					Step = 0.05f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "lat-b-camber-sensitivity",
					Label = "Lateral B camber sensitivity",
					Description = "Camber sensitivity for lateral pure-slip stiffness evaluation. " +
					              "Higher = cambered tyres build cornering force more aggressively. " +
					              "(magic number — default 0.35)",
					GetValue = () => tyreFL.LateralCamberStiffnessSensitivity,
					SetValue = v => ApplyToAllTyres(car, t => t.LateralCamberStiffnessSensitivity = v),
					Minimum = 0f,
					Maximum = 1.5f,
					Step = 0.025f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "lat-e-camber-sensitivity",
					Label = "Lateral E camber sensitivity",
					Description = "Camber sensitivity for lateral pure-slip curvature evaluation. " +
					              "More negative = stronger camber-driven reshaping of the lateral falloff. " +
					              "(magic number — default -0.15)",
					GetValue = () => tyreFL.LateralCamberCurvatureSensitivity,
					SetValue = v => ApplyToAllTyres(car, t => t.LateralCamberCurvatureSensitivity = v),
					Minimum = -1.0f,
					Maximum = 1.0f,
					Step = 0.025f,
					Unit = "",
				},
				new PhysCalField
				{
					Id = "high-slip-start",
						Label = "High-slip transition start",
						Description = "Slip angle (rad) at which the rally high-slip blending begins. " +
						              "Below this, pure Pacejka applies. ~15° by default. " +
						              "(magic number — default 0.26 rad ≈ 15°)",
						GetValue = () => tyreFL.HighSlipTransitionStart,
						SetValue = v => ApplyToAllTyres(car, t => t.HighSlipTransitionStart = v),
						Minimum = 0.05f,
						Maximum = 1.0f,
						Step = 0.01f,
						Unit = "rad",
					},
					new PhysCalField
					{
						Id = "high-slip-end",
						Label = "High-slip transition end",
						Description = "Slip angle (rad) at which the high-slip retention plateau is fully active. " +
						              "~40° by default; below start the curve is unmodified Pacejka. " +
						              "(magic number — default 0.70 rad ≈ 40°)",
						GetValue = () => tyreFL.HighSlipTransitionEnd,
						SetValue = v => ApplyToAllTyres(car, t => t.HighSlipTransitionEnd = v),
						Minimum = 0.1f,
						Maximum = 1.2f,
						Step = 0.01f,
						Unit = "rad",
					},
				],
			};
		}

		private static PhysCalCategory BuildTcsCategory(RallyCarComponent car)
		{
			return new PhysCalCategory
			{
				Id = "tcs",
				Title = "TCS",
				Tagline = "Traction control",
				Description = "Traction Control System parameters. " +
				              "TCS reduces engine torque when driven-wheel slip exceeds the target. " +
				              "Enabled/target/window may be sourced from JBeam; other values are magic numbers.",
				Fields =
				[
					new PhysCalField
					{
						Id = "tcs-slip-target",
						Label = "Slip ratio target",
						Description = "Driven-wheel slip ratio TCS attempts to maintain. " +
						              "Lower = tighter grip control; higher = more wheelspin before intervention. " +
						              "(from JBeam traction-control definition if present — default 0.15)",
						GetValue = () => car.TractionControlSlipRatioTarget,
						SetValue = v => car.TractionControlSlipRatioTarget = v,
						Minimum = 0.02f,
						Maximum = 0.50f,
						Step = 0.005f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "tcs-slip-window",
						Label = "Slip ratio window",
						Description = "Slip margin above the target over which torque reduction reaches the minimum. " +
						              "Narrow window = aggressive cut; wide = gradual reduction. " +
						              "(from JBeam traction-control definition if present — default 0.10)",
						GetValue = () => car.TractionControlSlipRatioWindow,
						SetValue = v => car.TractionControlSlipRatioWindow = v,
						Minimum = 0.01f,
						Maximum = 0.40f,
						Step = 0.005f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "tcs-min-speed",
						Label = "Minimum activation speed",
						Description = "TCS ignores slip below this speed where ratio estimates are noisy. " +
						              "(magic number — default 5 km/h)",
						GetValue = () => car.TractionControlMinimumSpeedKmh,
						SetValue = v => car.TractionControlMinimumSpeedKmh = v,
						Minimum = 0f,
						Maximum = 30f,
						Step = 0.5f,
						Unit = "km/h",
					},
					new PhysCalField
					{
						Id = "tcs-apply-rate",
						Label = "Apply rate",
						Description = "How fast TCS removes torque once excess slip is detected (1/s). " +
						              "Higher = more aggressive intervention. " +
						              "(magic number — default 16 /s)",
						GetValue = () => car.TractionControlApplyRate,
						SetValue = v => car.TractionControlApplyRate = v,
						Minimum = 1f,
						Maximum = 60f,
						Step = 0.5f,
						Unit = "/s",
					},
					new PhysCalField
					{
						Id = "tcs-release-rate",
						Label = "Release rate",
						Description = "How fast TCS restores torque after wheel slip recovers (1/s). " +
						              "Lower = smoother, more gradual torque return. " +
						              "(magic number — default 8 /s)",
						GetValue = () => car.TractionControlReleaseRate,
						SetValue = v => car.TractionControlReleaseRate = v,
						Minimum = 0.5f,
						Maximum = 30f,
						Step = 0.5f,
						Unit = "/s",
					},
					new PhysCalField
					{
						Id = "tcs-min-torque-scale",
						Label = "Min torque scale",
						Description = "Minimum fraction of engine torque TCS allows when fully active. " +
						              "0 = can cut to zero; higher values keep idle torque flowing. " +
						              "(magic number — default 0.08)",
						GetValue = () => car.TractionControlMinTorqueScale,
						SetValue = v => car.TractionControlMinTorqueScale = v,
						Minimum = 0f,
						Maximum = 0.5f,
						Step = 0.01f,
						Unit = "",
					},
				],
			};
		}

		private static PhysCalCategory BuildAbsEcsCategory(RallyCarComponent car)
		{
			return new PhysCalCategory
			{
				Id = "abs-ecs",
				Title = "ABS / ECS",
				Tagline = "Electronic braking control",
				Description = "Anti-lock Braking System parameters. " +
				              "ABS reduces brake torque when a wheel approaches lock. " +
				              "Target and window may come from JBeam brake definitions; other values are magic numbers.",
				Fields =
				[
					new PhysCalField
					{
						Id = "abs-slip-target",
						Label = "Slip ratio target",
						Description = "Braking slip magnitude ABS tries to maintain per wheel. " +
						              "Lower = more conservative, higher = allows more slip before cutting. " +
						              "(from JBeam brake definition if present — default 0.15)",
						GetValue = () => car.AbsSlipRatioTarget,
						SetValue = v => car.AbsSlipRatioTarget = v,
						Minimum = 0.02f,
						Maximum = 0.50f,
						Step = 0.005f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "abs-slip-window",
						Label = "Slip ratio window",
						Description = "Window above the ABS target over which brake torque ramps down to minimum. " +
						              "Narrow = aggressive cut; wide = gradual. " +
						              "(from JBeam brake definition if present — default 0.10)",
						GetValue = () => car.AbsSlipRatioWindow,
						SetValue = v => car.AbsSlipRatioWindow = v,
						Minimum = 0.01f,
						Maximum = 0.40f,
						Step = 0.005f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "abs-min-brake-scale",
						Label = "Min brake torque scale",
						Description = "Minimum fraction of service-brake torque ABS still allows near lock. " +
						              "Higher = more residual brake force when near lock-up threshold. " +
						              "(magic number — default 0.18)",
						GetValue = () => car.AbsMinBrakeTorqueScale,
						SetValue = v => car.AbsMinBrakeTorqueScale = v,
						Minimum = 0f,
						Maximum = 0.5f,
						Step = 0.01f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "abs-min-speed",
						Label = "Minimum activation speed",
						Description = "ABS ignores lock-up below this speed to prevent chatter at a stop. " +
						              "(magic number — default 4 km/h)",
						GetValue = () => car.AbsMinimumSpeedKmh,
						SetValue = v => car.AbsMinimumSpeedKmh = v,
						Minimum = 0f,
						Maximum = 20f,
						Step = 0.5f,
						Unit = "km/h",
					},
					new PhysCalField
					{
						Id = "abs-apply-rate",
						Label = "Apply rate",
						Description = "How fast ABS removes brake torque once lock-up is detected (1/s). " +
						              "(magic number — default 24 /s)",
						GetValue = () => car.AbsApplyRate,
						SetValue = v => car.AbsApplyRate = v,
						Minimum = 1f,
						Maximum = 80f,
						Step = 0.5f,
						Unit = "/s",
					},
					new PhysCalField
					{
						Id = "abs-release-rate",
						Label = "Release rate",
						Description = "How fast ABS restores brake torque after wheel speed recovers (1/s). " +
						              "(magic number — default 12 /s)",
						GetValue = () => car.AbsReleaseRate,
						SetValue = v => car.AbsReleaseRate = v,
						Minimum = 0.5f,
						Maximum = 40f,
						Step = 0.5f,
						Unit = "/s",
					},
				],
			};
		}

		private static PhysCalCategory BuildLaunchControlCategory(RallyCarComponent car)
		{
			return new PhysCalCategory
			{
				Id = "launch",
				Title = "Launch Control",
				Tagline = "Auto-clutch start line",
				Description = "Automatic clutch launch parameters. " +
				              "These shape start-line behaviour — how the engine holds RPM and " +
				              "how aggressively the clutch limits sustained burnout wheelspin. " +
				              "LaunchRpm may come from JBeam; other values are magic numbers.",
				Fields =
				[
					new PhysCalField
					{
						Id = "launch-rpm",
						Label = "Launch RPM",
						Description = "Target engine RPM the auto-clutch holds during a standing start. " +
						              "Higher = more aggressive launch bite, more wheelspin risk. " +
						              "(from JBeam engine definition if present — default 4500 rpm)",
						GetValue = () => car.AutoClutchLaunchRpm,
						SetValue = v => car.AutoClutchLaunchRpm = v,
						Minimum = 1000f,
						Maximum = 8000f,
						Step = 50f,
						Unit = "rpm",
					},
					new PhysCalField
					{
						Id = "launch-wheelspin-window",
						Label = "Wheelspin window",
						Description = "Equivalent RPM of excess driven-wheel spin over which the auto-clutch " +
						              "reduces transmitted torque to prevent a sustained full-throttle burnout. " +
						              "(magic number — default 1400 rpm)",
						GetValue = () => car.AutoClutchWheelspinWindowRpm,
						SetValue = v => car.AutoClutchWheelspinWindowRpm = v,
						Minimum = 100f,
						Maximum = 4000f,
						Step = 50f,
						Unit = "rpm",
					},
					new PhysCalField
					{
						Id = "launch-min-torque-scale",
						Label = "Min clutch torque scale",
						Description = "Minimum torque fraction the auto-clutch still transmits while limiting wheelspin. " +
						              "0 = full cut possible; higher preserves a torque floor. " +
						              "(magic number — default 0.25)",
						GetValue = () => car.AutoClutchMinTorqueScale,
						SetValue = v => car.AutoClutchMinTorqueScale = v,
						Minimum = 0f,
						Maximum = 1f,
						Step = 0.01f,
						Unit = "",
					},
					new PhysCalField
					{
						Id = "launch-shift-up-rpm",
						Label = "Auto shift-up RPM",
						Description = "RPM at which the auto-gearbox shifts up. " +
						              "Used during launch and general driving. " +
						              "(from JBeam engine / transmission definition — default 6500 rpm)",
						GetValue = () => car.ShiftUpRpm,
						SetValue = v => car.ShiftUpRpm = v,
						Minimum = 2000f,
						Maximum = 9000f,
						Step = 50f,
						Unit = "rpm",
					},
					new PhysCalField
					{
						Id = "launch-shift-down-rpm",
						Label = "Auto shift-down RPM",
						Description = "RPM at which the auto-gearbox shifts down. " +
						              "Should be well below shift-up to avoid hunting. " +
						              "(from JBeam engine / transmission definition — default 2200 rpm)",
						GetValue = () => car.ShiftDownRpm,
						SetValue = v => car.ShiftDownRpm = v,
						Minimum = 500f,
						Maximum = 6000f,
						Step = 50f,
						Unit = "rpm",
					},
				],
			};
		}

		// ── Helpers ───────────────────────────────────────────────────────────────

		private static TyreModel? GetTyreModel(RallyCarComponent car, int wheelIndex)
		{
			return car.Dynamics?.TyreModels[wheelIndex];
		}

		private static void ApplyToAllTyres(RallyCarComponent car, Action<TyreModel> apply)
		{
			if (car.Dynamics == null)
			{
				return;
			}

			for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
			{
				if (car.Dynamics.TyreModels[i] is { } tyre)
				{
					apply(tyre);
				}
			}
		}

		private static string GetTyreModelLabel(RallyCarComponent? car)
		{
			var fl = car?.Dynamics?.TyreModels[VehicleDynamicsSystem.FL];
			return fl?.ActiveMode switch
			{
				TyreModelMode.BrushOnly => "Brush Only",
				TyreModelMode.PacejkaOnly => "Pacejka Only",
				_ => "Auto (Blended)",
			};
		}

		private void SetTyreModelMode(TyreModelMode mode)
		{
			if (_car?.Dynamics == null)
			{
				return;
			}

			for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
			{
				if (_car.Dynamics.TyreModels[i] is { } tyre)
				{
					tyre.ActiveMode = mode;
				}
			}

			UpdateTyreModelBadge();
		}

		private void UpdateTyreModelBadge()
		{
			if (_tyreModelBadge != null)
			{
				_tyreModelBadge.Text = $"MODEL: {GetTyreModelLabel(_car)}";
			}
		}

		// ── UI construction ───────────────────────────────────────────────────────

		private void RebuildRoot()
		{
			if (_desktop == null)
			{
				return;
			}

			_categoryButtons.Clear();
			_desktop.Root = BuildRoot();

			if (_categoryButtons.Count > 0)
			{
				_selectedCategoryIndex = Math.Clamp(_selectedCategoryIndex, 0, _categoryButtons.Count - 1);
				_desktop.FocusedKeyboardWidget = _categoryButtons[_selectedCategoryIndex].Button;
			}
		}

		private Widget BuildRoot()
		{
			var root = new Panel { Background = Brush(BackdropColor) };

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

			var nav = BuildNavigationCard();
			Grid.SetRow(nav, 1);
			shell.Widgets.Add(nav);

			var editors = BuildEditorsCard();
			Grid.SetColumn(editors, 1);
			Grid.SetRow(editors, 1);
			shell.Widgets.Add(editors);

			var summary = BuildSummaryCard();
			Grid.SetColumn(summary, 2);
			Grid.SetRow(summary, 1);
			shell.Widgets.Add(summary);

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
				Text = "Physics Calibration",
				TextColor = TitleColor,
			});
			var subtitle = new Label
			{
				Text = "Isolate tyre model contributions and tune anti-slip parameters live. " +
				       "All changes apply immediately without reloading the vehicle.",
				TextColor = CopyColor,
				Wrap = true,
			};
			Grid.SetRow(subtitle, 1);
			header.Widgets.Add(subtitle);

			_tyreModelBadge = new Label
			{
				Text = $"MODEL: {GetTyreModelLabel(_car)}",
				TextColor = ValueColor,
				Background = Brush(AccentSoftColor),
				HorizontalAlignment = HorizontalAlignment.Right,
				VerticalAlignment = VerticalAlignment.Center,
			};
			Grid.SetColumn(_tyreModelBadge, 1);
			Grid.SetRowSpan(_tyreModelBadge, 2);
			header.Widgets.Add(_tyreModelBadge);

			panel.Widgets.Add(header);
			return panel;
		}

		private Panel BuildNavigationCard()
		{
			var panel = CreateCardPanel(width: 250, background: PanelColor);
			var content = CreateCardContent();
			content.Width = 226;
			content.HorizontalAlignment = HorizontalAlignment.Center;
			content.VerticalAlignment = VerticalAlignment.Top;
			content.Widgets.Add(CreateSectionTitle("Categories"));
			content.Widgets.Add(CreateSectionBody("D-Pad Up/Down to move. Left/Right adjust field or change pane."));
			content.Widgets.Add(CreateSpacer(6));

			// Tyre model quick-select buttons
			content.Widgets.Add(CreateSectionTitle("Tyre Model"));
			foreach (var (label, mode) in new[] { ("Auto (Blended)", TyreModelMode.Auto), ("Brush Only", TyreModelMode.BrushOnly), ("Pacejka Only", TyreModelMode.PacejkaOnly) })
			{
				var localMode = mode;
				var btn = new Button
				{
					Background = Brush(PanelAltColor),
					HorizontalAlignment = HorizontalAlignment.Stretch,
					Content = new Label { Text = label, TextColor = CopyColor, Wrap = true },
				};
				btn.Click += (_, _) =>
				{
					SetTyreModelMode(localMode);
					RefreshCategoryContent();
				};
				content.Widgets.Add(btn);
			}

			content.Widgets.Add(CreateSpacer(6));
			content.Widgets.Add(CreateSectionTitle("Parameters"));

			foreach (var category in _categories)
			{
				content.Widgets.Add(CreateCategoryButton(category));
			}

			panel.Widgets.Add(content);
			return panel;
		}

		private Panel BuildEditorsCard()
		{
			var panel = CreateCardPanel(width: 680, background: PanelColor);
			var content = CreateCardContent();
			content.Width = 648;
			content.HorizontalAlignment = HorizontalAlignment.Center;
			content.VerticalAlignment = VerticalAlignment.Top;

			_categoryTitleLabel = CreateSectionTitle(string.Empty);
			_categoryTaglineLabel = new Label { TextColor = ValueColor };
			_categoryDescriptionLabel = CreateSectionBody(string.Empty);
			_fieldWindowLabel = new Label { TextColor = MutedColor };
			_editorStack = new VerticalStackPanel { Spacing = 12 };

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
			var panel = CreateCardPanel(width: 300, background: PanelColor);
			var content = CreateCardContent();
			content.Width = 272;
			content.HorizontalAlignment = HorizontalAlignment.Center;
			content.VerticalAlignment = VerticalAlignment.Top;

			content.Widgets.Add(CreateSectionTitle("Live Values"));
			content.Widgets.Add(CreateSectionBody("Changes apply immediately to the active vehicle."));
			content.Widgets.Add(CreateSpacer(8));

			_vehicleNameLabel = new Label { TextColor = TitleColor };
			_statusLabel = CreateSectionBody(string.Empty);

			content.Widgets.Add(_vehicleNameLabel);
			content.Widgets.Add(_statusLabel);
			content.Widgets.Add(CreateSpacer(10));
			content.Widgets.Add(CreateSectionTitle("Suspension + surface telemetry"));
			_suspensionTelemetryLabel = CreateSectionBody(string.Empty);
			_suspensionTelemetryLabel.Wrap = true;
			content.Widgets.Add(_suspensionTelemetryLabel);
			content.Widgets.Add(CreateSpacer(10));
			content.Widgets.Add(new Label
			{
				Text = "D-Pad Up/Down moves list.\nLeft/Right adjusts or changes pane.\nEsc/B/Start goes back.",
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

			footer.Widgets.Add(new Label
			{
				Text = "Physics calibration — all changes are live. " +
				       "Reset reloads the vehicle to restore JBeam defaults.",
				TextColor = CopyColor,
				Wrap = true,
			});

			var autoBtn = new Button
			{
				Background = Brush(PanelAltColor),
				Content = new Label { Text = "Auto Model", TextColor = CopyColor },
			};
			autoBtn.Click += (_, _) =>
			{
				SetTyreModelMode(TyreModelMode.Auto);
				RefreshCategoryContent();
			};
			Grid.SetColumn(autoBtn, 1);
			footer.Widgets.Add(autoBtn);

			var closeBtn = new Button
			{
				Background = Brush(AccentSoftColor),
				Content = new Label { Text = "Back to Pause", TextColor = ValueColor },
			};
			closeBtn.Click += (_, _) => CloseRequested?.Invoke();
			Grid.SetColumn(closeBtn, 2);
			footer.Widgets.Add(closeBtn);

			return footer;
		}

		private Button CreateCategoryButton(PhysCalCategory category)
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
				_selectedCategoryIndex = buttonIndex;
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

		private Widget CreateFieldEditor(PhysCalField field, bool isSelected)
		{
			var currentValue = field.GetValue();

			var panel = CreateCardPanel(width: 648, background: isSelected ? PanelSelectedColor : PanelAltColor);
			var content = CreateCardContent();
			content.Width = 616;
			content.HorizontalAlignment = HorizontalAlignment.Center;
			content.VerticalAlignment = VerticalAlignment.Center;
			content.Spacing = 6;

			var headerRow = new Grid { ColumnSpacing = 8 };
			headerRow.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
			headerRow.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
			headerRow.Widgets.Add(new Label { Text = field.Label, TextColor = TitleColor });

			var valueLabel = new Label
			{
				Text = FormatValue(currentValue, field.Unit, field.Step),
				TextColor = isSelected ? TitleColor : ValueColor,
			};
			Grid.SetColumn(valueLabel, 1);
			headerRow.Widgets.Add(valueLabel);
			content.Widgets.Add(headerRow);
			content.Widgets.Add(CreateSectionBody(field.Description));

			// Numeric editor: slider + spinbutton
			var editor = new Grid { ColumnSpacing = 10 };
			editor.ColumnsProportions.Add(new Proportion(ProportionType.Fill));
			editor.ColumnsProportions.Add(new Proportion(ProportionType.Auto));
			editor.ColumnsProportions.Add(new Proportion(ProportionType.Auto));

			var clampedValue = Math.Clamp(currentValue, field.Minimum, field.Maximum);
			var slider = new HorizontalSlider
			{
				Minimum = field.Minimum,
				Maximum = field.Maximum,
				Value = clampedValue,
				VerticalAlignment = VerticalAlignment.Center,
			};
			slider.ValueChangedByUser += (_, args) =>
			{
				field.SetValue(args.NewValue);
				RefreshCategoryContent();
			};
			editor.Widgets.Add(slider);

			var spinButton = new SpinButton
			{
				Width = 92,
				Minimum = field.Minimum,
				Maximum = field.Maximum,
				Value = clampedValue,
				Increment = MathF.Max(field.Step, 0.001f),
				DecimalPlaces = ResolveDecimalPlaces(field.Step),
				HorizontalAlignment = HorizontalAlignment.Right,
			};
			spinButton.ValueChangedByUser += (_, args) =>
			{
				if (args.NewValue.HasValue)
				{
					field.SetValue(args.NewValue.Value);
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

			content.Widgets.Add(editor);
			panel.Widgets.Add(content);
			return panel;
		}

		// ── Refresh helpers ───────────────────────────────────────────────────────

		private void RefreshCategoryContent()
		{
			if (_editorStack == null || _categoryTitleLabel == null ||
			    _categoryTaglineLabel == null || _categoryDescriptionLabel == null ||
			    _fieldWindowLabel == null)
			{
				return;
			}

			if (_categories.Count == 0 || _car == null)
			{
				_categoryTitleLabel.Text = "No vehicle loaded";
				_categoryTaglineLabel.Text = string.Empty;
				_categoryDescriptionLabel.Text = "Load a vehicle first, then open Physics Calibration from the pause menu.";
				_fieldWindowLabel.Text = string.Empty;
				_editorStack.Widgets.Clear();
				return;
			}

			var category = _categories[Math.Clamp(_selectedCategoryIndex, 0, _categories.Count - 1)];
			_categoryTitleLabel.Text = category.Title;
			_categoryTaglineLabel.Text = category.Tagline;
			_categoryDescriptionLabel.Text = category.Description;

			var fieldCount = category.Fields.Count;
			var maxScroll = Math.Max(0, fieldCount - MaxVisibleFields);
			_fieldScrollOffset = Math.Clamp(_fieldScrollOffset, 0, maxScroll);
			_selectedFieldIndex = Math.Clamp(_selectedFieldIndex, 0, Math.Max(0, fieldCount - 1));

			var visibleStart = _fieldScrollOffset;
			var visibleEnd = Math.Min(visibleStart + MaxVisibleFields, fieldCount);
			_fieldWindowLabel.Text = fieldCount > MaxVisibleFields
				? $"Fields {visibleStart + 1}–{visibleEnd} of {fieldCount}"
				: string.Empty;

			_editorStack.Widgets.Clear();
			for (var i = visibleStart; i < visibleEnd; i++)
			{
				_editorStack.Widgets.Add(CreateFieldEditor(category.Fields[i], i == _selectedFieldIndex));
			}

			UpdateTyreModelBadge();
		}

		private void UpdateCategoryButtonStyles()
		{
			for (var i = 0; i < _categoryButtons.Count; i++)
			{
				var (btn, lbl) = _categoryButtons[i];
				var selected = i == _selectedCategoryIndex;
				btn.Background = Brush(selected ? AccentColor : PanelAltColor);
				lbl.TextColor = selected ? ValueColor : CopyColor;
			}
		}

		private void UpdateStaticLabels()
		{
			if (_vehicleNameLabel != null)
			{
				_vehicleNameLabel.Text = string.IsNullOrWhiteSpace(_vehicleName) ? "LibreRally" : _vehicleName;
			}

			if (_statusLabel != null)
			{
				_statusLabel.Text = _statusText;
			}
		}

		private void UpdateLiveTelemetryLabels()
		{
			if (_suspensionTelemetryLabel == null)
			{
				return;
			}

			var dynamics = _car?.Dynamics;
			if (dynamics == null)
			{
				_suspensionTelemetryLabel.Text = "Suspension telemetry unavailable.";
				return;
			}

			_suspensionTelemetryLabel.Text =
				$"{FormatSuspensionTelemetryLine(dynamics, VehicleDynamicsSystem.FL, "FL")}\n" +
				$"{FormatSuspensionTelemetryLine(dynamics, VehicleDynamicsSystem.FR, "FR")}\n" +
				$"{FormatSuspensionTelemetryLine(dynamics, VehicleDynamicsSystem.RL, "RL")}\n" +
				$"{FormatSuspensionTelemetryLine(dynamics, VehicleDynamicsSystem.RR, "RR")}";
		}

		private string FormatSuspensionTelemetryLine(VehicleDynamicsSystem dynamics, int wheelIndex, string wheelLabel)
		{
			var wheelSettings = _car != null && wheelIndex < _car.Wheels.Count
				? _car.Wheels[wheelIndex].Get<WheelSettings>()
				: null;
			var surfaceType = wheelSettings?.CurrentSurface ?? SurfaceType.Tarmac;
			var surface = dynamics.WheelSurfaces[wheelIndex];
			var effectivePeakFriction = dynamics.EffectivePeakFrictionCoefficients[wheelIndex];
			return $"{wheelLabel} {surfaceType,-9} µs:{surface.FrictionCoefficient,4:F2} " +
			       $"ss:{surface.SlipStiffnessScale,4:F2} µe:{effectivePeakFriction,4:F2} " +
			       $"c:{dynamics.SuspensionCompression[wheelIndex] * MetersToMillimeters,5:F0}mm " +
			       $"v:{dynamics.SuspensionVelocity[wheelIndex],5:F2} " +
			       $"sf:{dynamics.SpringForces[wheelIndex] / NewtonsToKilonewtons,5:F2} " +
			       $"df:{dynamics.DamperForces[wheelIndex] / NewtonsToKilonewtons,5:F2} " +
			       $"bf:{dynamics.BumpStopForces[wheelIndex] / NewtonsToKilonewtons,5:F2}";
		}

		// ── Input handling ────────────────────────────────────────────────────────

		private void HandleNavigationInput()
		{
			if (_game == null)
			{
				return;
			}

			var input = _game.Input;
			var pad = input.GamePads.Count > 0 ? input.GamePads[0] : null;

			var closeRequested =
				input.IsKeyPressed(Keys.Escape) ||
				(pad?.IsButtonPressed(GamePadButton.B) ?? false) ||
				(pad?.IsButtonPressed(GamePadButton.Start) ?? false);

			if (closeRequested)
			{
				CloseRequested?.Invoke();
				return;
			}

			if (_categories.Count == 0)
			{
				return;
			}

			var up = input.IsKeyPressed(Keys.Up) || (pad?.IsButtonPressed(GamePadButton.PadUp) ?? false);
			var down = input.IsKeyPressed(Keys.Down) || (pad?.IsButtonPressed(GamePadButton.PadDown) ?? false);
			var left = input.IsKeyPressed(Keys.Left) || (pad?.IsButtonPressed(GamePadButton.PadLeft) ?? false);
			var right = input.IsKeyPressed(Keys.Right) || (pad?.IsButtonPressed(GamePadButton.PadRight) ?? false);
			var enter = input.IsKeyPressed(Keys.Enter) || (pad?.IsButtonPressed(GamePadButton.A) ?? false);
			var leftBumper = pad?.IsButtonPressed(GamePadButton.LeftShoulder) ?? false;
			var rightBumper = pad?.IsButtonPressed(GamePadButton.RightShoulder) ?? false;

			var category = _categories[Math.Clamp(_selectedCategoryIndex, 0, _categories.Count - 1)];
			var fieldCount = category.Fields.Count;

			// Switch category with shoulder buttons
			if (leftBumper)
			{
				_selectedCategoryIndex = (_selectedCategoryIndex - 1 + _categories.Count) % _categories.Count;
				_selectedFieldIndex = 0;
				_fieldScrollOffset = 0;
				RefreshCategoryContent();
				UpdateCategoryButtonStyles();
				return;
			}

			if (rightBumper)
			{
				_selectedCategoryIndex = (_selectedCategoryIndex + 1) % _categories.Count;
				_selectedFieldIndex = 0;
				_fieldScrollOffset = 0;
				RefreshCategoryContent();
				UpdateCategoryButtonStyles();
				return;
			}

			if (up && fieldCount > 0)
			{
				_selectedFieldIndex--;
				if (_selectedFieldIndex < _fieldScrollOffset)
				{
					_fieldScrollOffset = Math.Max(0, _fieldScrollOffset - 1);
				}

				if (_selectedFieldIndex < 0)
				{
					_selectedFieldIndex = fieldCount - 1;
					_fieldScrollOffset = Math.Max(0, fieldCount - MaxVisibleFields);
				}

				RefreshCategoryContent();
				return;
			}

			if (down && fieldCount > 0)
			{
				_selectedFieldIndex++;
				if (_selectedFieldIndex >= _fieldScrollOffset + MaxVisibleFields)
				{
					_fieldScrollOffset = Math.Min(_selectedFieldIndex - MaxVisibleFields + 1, fieldCount - MaxVisibleFields);
					_fieldScrollOffset = Math.Max(0, _fieldScrollOffset);
				}

				if (_selectedFieldIndex >= fieldCount)
				{
					_selectedFieldIndex = 0;
					_fieldScrollOffset = 0;
				}

				RefreshCategoryContent();
				return;
			}

			// Adjust the selected field value
			if ((left || right || enter) && fieldCount > 0)
			{
				var clampedField = Math.Clamp(_selectedFieldIndex, 0, fieldCount - 1);
				var field = category.Fields[clampedField];
				var step = left ? -field.Step : field.Step;
				var newValue = Math.Clamp(field.GetValue() + step, field.Minimum, field.Maximum);
				field.SetValue(newValue);
				RefreshCategoryContent();
			}
		}

		// ── Widget factory helpers ────────────────────────────────────────────────

		private SolidBrush Brush(Color color)
		{
			if (!_brushCache.TryGetValue(color, out var brush))
			{
				brush = new SolidBrush(color);
				_brushCache[color] = brush;
			}

			return brush;
		}

		private Panel CreateCardPanel(int width, Color background) => new()
		{
			Width = width,
			Background = Brush(background),
		};

		private static VerticalStackPanel CreateCardContent() => new()
		{
			Spacing = 8,
		};

		private static Label CreateSectionTitle(string text) => new()
		{
			Text = text,
			TextColor = new Color(240, 243, 247, 255),
		};

		private static Label CreateSectionBody(string text) => new()
		{
			Text = text,
			TextColor = new Color(183, 193, 205, 255),
			Wrap = true,
		};

		private static Panel CreateSpacer(int height) => new() { Height = height };

		private static string FormatValue(float value, string unit, float step)
		{
			var decimals = ResolveDecimalPlaces(step);
			var formatted = value.ToString($"F{decimals}", CultureInfo.InvariantCulture);
			return string.IsNullOrWhiteSpace(unit) ? formatted : $"{formatted} {unit}";
		}

		private static int ResolveDecimalPlaces(float step)
		{
			if (step >= 100f)
			{
				return 0;
			}

			if (step >= 1f)
			{
				return 1;
			}

			if (step >= 0.1f)
			{
				return 2;
			}

			if (step >= 0.01f)
			{
				return 3;
			}

			return 4;
		}
	}
}
