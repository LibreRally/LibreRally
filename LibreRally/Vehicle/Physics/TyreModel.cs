using System;

namespace LibreRally.Vehicle.Physics
{
	/// <summary>
	/// Selects which tyre force model is active during physics simulation.
	/// Used by the physics calibration overlay to isolate and compare model contributions.
	/// </summary>
	public enum TyreModelMode
	{
		/// <summary>Default blended behaviour: brush transients remain active while Pacejka provides the steady-state force curve.</summary>
		Auto,
		/// <summary>Exclusively use the contact-patch brush model. Pacejka forces are ignored.</summary>
		BrushOnly,
		/// <summary>Exclusively use the Pacejka Magic Formula. Brush model transients are bypassed.</summary>
		PacejkaOnly,
	}


	/// <summary>
	/// Mutable per-wheel thermal and wear state.
	/// Updated every physics step by <see cref="TyreModel.Update(ref TyreState, float, float, float, float, float, float, in SurfaceProperties, float, out float, out float, out float)"/>.
	///
	/// Temperature model reference: Salaani et al., "An Analytical Tire Model for
	/// Use in Vehicle Dynamics Simulations", SAE 2007-01-0816.
	/// Wear model: simplified abrasion proportional to slip energy dissipation.
	/// </summary>
	public struct TyreState
	{
		/// <summary>Current tyre surface temperature (°C). Starts at ambient (~30 °C).</summary>
		public float Temperature;

		/// <summary>Current tyre carcass/core temperature (°C). Changes slower than the tread surface.</summary>
		public float CoreTemperature;

		/// <summary>Remaining tread fraction (1.0 = new, 0.0 = fully worn).</summary>
		public float TreadLife;

		/// <summary>Wheel angular velocity (rad/s). Positive = forward rolling.</summary>
		public float AngularVelocity;

		/// <summary>Lateral deflection state for the brush contact-patch model (m).</summary>
		public float LateralDeflection;

		/// <summary>Longitudinal deflection state for the brush contact-patch model (m).</summary>
		public float LongitudinalDeflection;

		/// <summary>Current longitudinal slip ratio (dimensionless).</summary>
		public float SlipRatio;

		/// <summary>Current lateral slip angle (rad).</summary>
		public float SlipAngle;

		/// <summary>Current drivetrain torque applied to the wheel before tyre reaction (N·m).</summary>
		public float DriveTorque;

		/// <summary>Current brake torque applied to the wheel (N·m).</summary>
		public float BrakeTorque;

		/// <summary>Current tyre reaction torque opposing wheel rotation (N·m).</summary>
		public float TyreReactionTorque;

		/// <summary>Creates a fresh tyre state with ambient temperature, full tread life, and zero slip state.</summary>
		/// <returns>A new <see cref="TyreState"/> initialized to the model defaults.</returns>
		public static TyreState CreateDefault() => new()
		{
			Temperature = 30f,
			CoreTemperature = 30f,
			TreadLife = 1.0f,
			AngularVelocity = 0f,
			LateralDeflection = 0f,
			LongitudinalDeflection = 0f,
			SlipRatio = 0f,
			SlipAngle = 0f,
			DriveTorque = 0f,
			BrakeTorque = 0f,
			TyreReactionTorque = 0f,
		};
	}

	/// <summary>
	/// Slip-based tyre physics model combining brush tyre theory with Pacejka-like force curves.
	///
	/// <para><b>Slip ratio</b> (longitudinal):
	/// <c>κ = (ω·R − Vx) / max(|Vx|, ε)</c>
	/// where ω = wheel angular velocity, R = tyre radius, Vx = longitudinal velocity.</para>
	///
	/// <para><b>Slip angle</b> (lateral):
	/// <c>α = atan(Vy / |Vx|)</c>
	/// where Vy = lateral velocity at the contact patch.</para>
	///
	/// <para>Forces are computed using a simplified Pacejka "Magic Formula":
	/// <c>F = D · sin(C · atan(B·x − E·(B·x − atan(B·x))))</c>
	/// with load-sensitive peak factor D = µ·Fz and appropriate stiffness B.</para>
	///
	/// <para>Key references:
	/// <list type="bullet">
	///   <item>Pacejka, "Tire and Vehicle Dynamics", 3rd Ed., Chapters 3–4.</item>
	///   <item>Brush tyre model: Milliken &amp; Milliken, "Race Car Vehicle Dynamics", §2.5.</item>
	///   <item>Rally high-slip: Abdulrahim, "Measurement and Analysis of Rally Car
	///         Dynamics at High Attitude Angles", SAE 2007.</item>
	/// </list></para>
	///
	/// Designed for struct-based per-wheel state (<see cref="TyreState"/>) to avoid
	/// per-frame heap allocations when simulating 50+ vehicles.
	/// </summary>
	public sealed class TyreModel
	{
		private readonly TyreMagicFormulaSet _longitudinalPureSlipSet = new(12.0f, 1.65f, -0.5f)
		{
			LoadStiffnessSensitivity = 0.04f,
			SurfaceDeformationSensitivity = 0.3f,
		};

		private readonly TyreMagicFormulaSet _lateralPureSlipSet = new(10.0f, 1.30f, -0.6f)
		{
			LoadStiffnessSensitivity = 0.06f,
			CamberStiffnessSensitivity = 0.35f,
			CamberCurvatureSensitivity = -0.15f,
			SurfaceDeformationSensitivity = 0.2f,
		};

		// ── Model selection ──────────────────────────────────────────────────────

		/// <summary>
		/// Selects the active tyre force model.
		/// <list type="bullet">
		///   <item><see cref="TyreModelMode.Auto"/> — default blended brush+Pacejka behaviour.</item>
		///   <item><see cref="TyreModelMode.BrushOnly"/> — bypass Pacejka; use only contact-patch brush forces.</item>
		///   <item><see cref="TyreModelMode.PacejkaOnly"/> — bypass brush transients; use only Magic Formula forces.</item>
		/// </list>
		/// Changed at runtime via the physics calibration overlay.
		/// </summary>
		public TyreModelMode ActiveMode { get; set; } = TyreModelMode.Auto;

		// ── Tyre geometry ────────────────────────────────────────────────────────

		/// <summary>Unloaded tyre radius (m).</summary>
		public float Radius { get; }

		/// <summary>Tyre width (m), used for contact patch size estimation.</summary>
		public float Width { get; set; } = ReferenceTyreWidth;

		// ── Tyre construction parameters ─────────────────────────────────────────
		// These properties support BeamNG-style tyre definitions and can be loaded
		// from JBeam tyre data. They influence vertical stiffness, contact patch
		// geometry, transient response, and steering feel.

		/// <summary>
		/// Gets or sets the tyre inflation pressure in kilopascals.
		/// Higher values increase carcass stiffness and reduce the effective contact-patch size.
		/// Lower values increase compliance and usually improve low-speed grip.
		/// </summary>
		public float TyrePressure { get; set; } = 220f;

		/// <summary>
		/// Gets or sets the sidewall stiffness multiplier, where 1.0 is the baseline response.
		/// Increasing this value sharpens turn-in by building lateral force more quickly.
		/// Decreasing this value makes the tyre feel softer and slower to respond.
		/// </summary>
		public float SidewallStiffness { get; set; } = 1.0f;

		/// <summary>
		/// Gets or sets the carcass stiffness multiplier, where 1.0 is the baseline tyre-body compliance.
		/// Increasing this value reduces transient delay before forces build at the contact patch.
		/// Decreasing this value allows more carcass deformation and slower transient response.
		/// </summary>
		public float CarcassStiffness { get; set; } = 1.0f;

		/// <summary>
		/// Contact patch length scale factor (dimensionless) applied to the theoretical
		/// rectangular footprint length computed from load, pressure, and width:
		/// <c>length ≈ normalLoad / (pressure × width)</c>.
		/// Values above 1.0 lengthen the effective patch for extra compliance; values below 1.0 shorten it.
		/// Reference: The Contact Patch, "The contact patch"; Pacejka §3.2.
		/// </summary>
		public float ContactPatchLengthScale { get; set; } = 1.0f;

		/// <summary>
		/// Obsolete alias for <see cref="ContactPatchLengthScale"/> retained for backward compatibility.
		/// This value is a dimensionless scale factor, not an absolute length in metres.
		/// </summary>
		[Obsolete("ContactPatchLength is a dimensionless scale factor. Use ContactPatchLengthScale instead.")]
		public float ContactPatchLength
		{
			get => ContactPatchLengthScale;
			set => ContactPatchLengthScale = value;
		}

		/// <summary>
		/// Baseline vertical stiffness of the tyre carcass (N/m) at the 220 kPa reference pressure.
		/// Used to estimate loaded-radius deflection and therefore effective rolling radius.
		/// Higher values reduce loaded deflection and slip generated by carcass compression.
		/// </summary>
		public float VerticalStiffness { get; set; } = 200000f;

		/// <summary>
		/// Wheel rotational inertia (kg·m²). Governs spin-up/spin-down response.
		/// Typical values: 1.2–1.8 kg·m² for rally wheels.
		/// When set to 0 (default), inertia is estimated from normal load and radius.
		/// <para>angularAccel = appliedTorque / WheelInertia</para>
		/// Reference: Milliken, RCVD §2.3, wheel inertia.
		/// </summary>
		public float WheelInertia { get; set; } = 0f;

		/// <summary>BeamNG-derived hub radius hint (m) used to refine wheel inertia estimation when explicit inertia is unavailable.</summary>
		public float? HubRadius { get; set; }

		/// <summary>BeamNG-derived hub width hint (m) used to refine wheel inertia estimation when explicit inertia is unavailable.</summary>
		public float? HubWidth { get; set; }

		/// <summary>BeamNG-derived wheel offset hint (m). Carried through for wheel-geometry-aware physics work.</summary>
		public float? WheelOffset { get; set; }

		// ── Pacejka Magic Formula coefficients ───────────────────────────────────
		// F = D * sin(C * atan(B*x - E*(B*x - atan(B*x))))
		// B = stiffness factor, C = shape factor, D = peak factor, E = curvature

		/// <summary>Longitudinal stiffness factor B (cornering stiffness / peak force).</summary>
		public float LongitudinalB
		{
			get => _longitudinalPureSlipSet.BaseB;
			set => _longitudinalPureSlipSet.BaseB = value;
		}

		/// <summary>Longitudinal shape factor C. Typically 1.5–1.8 for combined slip models.</summary>
		public float LongitudinalC
		{
			get => _longitudinalPureSlipSet.BaseC;
			set => _longitudinalPureSlipSet.BaseC = value;
		}

		/// <summary>Longitudinal curvature factor E. Negative values extend the curve past the peak.</summary>
		public float LongitudinalE
		{
			get => _longitudinalPureSlipSet.BaseE;
			set => _longitudinalPureSlipSet.BaseE = value;
		}

		/// <summary>
		/// Load sensitivity applied to the longitudinal pure-slip stiffness factor.
		/// Higher values make Fx build up more sharply as vertical load rises.
		/// </summary>
		public float LongitudinalLoadStiffnessSensitivity
		{
			get => _longitudinalPureSlipSet.LoadStiffnessSensitivity;
			set => _longitudinalPureSlipSet.LoadStiffnessSensitivity = value;
		}

		/// <summary>Lateral stiffness factor B.</summary>
		public float LateralB
		{
			get => _lateralPureSlipSet.BaseB;
			set => _lateralPureSlipSet.BaseB = value;
		}

		/// <summary>Lateral shape factor C. Typically 1.1–1.4 for passenger/rally tyres.</summary>
		public float LateralC
		{
			get => _lateralPureSlipSet.BaseC;
			set => _lateralPureSlipSet.BaseC = value;
		}

		/// <summary>Lateral curvature factor E.</summary>
		public float LateralE
		{
			get => _lateralPureSlipSet.BaseE;
			set => _lateralPureSlipSet.BaseE = value;
		}

		/// <summary>
		/// Load sensitivity applied to the lateral pure-slip stiffness factor.
		/// Higher values make the cornering-stiffness ramp respond more strongly to load.
		/// </summary>
		public float LateralLoadStiffnessSensitivity
		{
			get => _lateralPureSlipSet.LoadStiffnessSensitivity;
			set => _lateralPureSlipSet.LoadStiffnessSensitivity = value;
		}

		/// <summary>
		/// Camber sensitivity applied to the lateral pure-slip stiffness factor.
		/// Higher values make cambered tyres build lateral force more aggressively.
		/// </summary>
		public float LateralCamberStiffnessSensitivity
		{
			get => _lateralPureSlipSet.CamberStiffnessSensitivity;
			set => _lateralPureSlipSet.CamberStiffnessSensitivity = value;
		}

		/// <summary>
		/// Camber sensitivity applied to the lateral pure-slip curvature factor.
		/// Higher values reshape the lateral force falloff more strongly under camber.
		/// </summary>
		public float LateralCamberCurvatureSensitivity
		{
			get => _lateralPureSlipSet.CamberCurvatureSensitivity;
			set => _lateralPureSlipSet.CamberCurvatureSensitivity = value;
		}

		/// <summary>
		/// Ratio of lateral grip limit to longitudinal grip limit.
		/// 1.0 gives an equal-grip friction circle; values below 1.0 produce a friction ellipse.
		/// </summary>
		public float FrictionEllipseRatio { get; set; } = 1.0f;

		// ── Friction ─────────────────────────────────────────────────────────────

		/// <summary>Peak friction coefficient on reference surface (tarmac).</summary>
		public float PeakFrictionCoefficient { get; set; } = 1.05f;

		/// <summary>
		/// Load sensitivity exponent for the tyre's non-linear grip response to vertical load.
		/// <para><c>µ_effective = µ_peak × (Fz / Fz_ref)^(-LoadSensitivity)</c></para>
		/// Higher values make grip fall away faster as load rises.
		/// Typical range 0.0–0.20; 0.15 is a representative road-tyre value.
		/// </summary>
		public float LoadSensitivity { get; set; } = 0.15f;

		/// <summary>BeamNG-style zero-load friction coefficient, when provided by JBeam tyre metadata.</summary>
		public float? BeamNgNoLoadFrictionCoefficient { get; set; }

		/// <summary>BeamNG-style asymptotic full-load friction coefficient, when provided by JBeam tyre metadata.</summary>
		public float? BeamNgFullLoadFrictionCoefficient { get; set; }

		/// <summary>BeamNG-style exponential load-sensitivity slope (1/N), when provided by JBeam tyre metadata.</summary>
		public float? BeamNgLoadSensitivitySlope { get; set; }

		/// <summary>Reference vertical load for µ rating (N).</summary>
		public float ReferenceLoad { get; set; } = 3000f;

		/// <summary>
		/// Gets or sets the exponent used for contact-patch-area grip scaling.
		/// Larger values make grip more sensitive to contact-patch area changes under load.
		/// Smaller values keep grip closer to the baseline friction curve.
		/// </summary>
		public float ContactAreaGripExponent { get; set; } = 0.05f;

		// ── Brush contact-patch model ────────────────────────────────────────────

		/// <summary>
		/// Brush stiffness calibration (N/m at 220 kPa and 205 mm width).
		/// The effective brush stiffness scales from the pressure-membrane estimate <c>pressure × width</c>.
		/// </summary>
		public float ContactPatchStiffness { get; set; } = ReferenceContactPatchStiffness;

		/// <summary>Lateral contact-patch damping (N·s/m).</summary>
		public float ContactPatchDamping { get; set; } = 4500f;

		/// <summary>
		/// Baseline lateral relaxation length (m) for the brush tyre transient model.
		/// The effective value is further scaled by loose-surface modifiers and the local force-curve slope.
		/// </summary>
		public float RelaxationLength { get; set; } = 0.30f;

		/// <summary>
		/// Baseline longitudinal relaxation length (m) for the brush tyre transient model.
		/// Typically shorter than lateral because the tyre carcass is stiffer in
		/// the rolling direction. The effective value is further scaled by surface and local force-curve slope.
		/// Default 0.28 m.
		/// Reference: Pacejka §5.4; brush model braking transient.
		/// </summary>
		public float LongitudinalRelaxationLength { get; set; } = 0.28f;

		/// <summary>
		/// Surface-coupling sensitivity for lateral relaxation length.
		/// Higher values make loose/low-grip surfaces build lateral force more slowly.
		/// </summary>
		public float LateralSurfaceRelaxationSensitivity { get; set; } = 0.6f;

		/// <summary>
		/// Surface-coupling sensitivity for longitudinal relaxation length.
		/// Higher values make loose/low-grip surfaces delay drive/brake force buildup more strongly.
		/// </summary>
		public float LongitudinalSurfaceRelaxationSensitivity { get; set; } = 0.85f;

		// ── Camber ───────────────────────────────────────────────────────────────

		/// <summary>Camber thrust coefficient (N per rad of camber per N of Fz).</summary>
		public float CamberThrustCoefficient { get; set; } = 0.08f;

		// ── Self-aligning torque ─────────────────────────────────────────────────

		/// <summary>Pneumatic trail at zero slip (m). Mz = trail × Fy. Decreases with slip angle.</summary>
		public float PneumaticTrail { get; set; } = 0.025f;

		/// <summary>
		/// Small residual aligning-torque fraction retained after the pneumatic trail has largely collapsed.
		/// This captures the non-trail remainder of Mz near the limit without dominating low-slip steering feel.
		/// </summary>
		public float AligningTorqueResidualFactor { get; set; } = 0.08f;

		/// <summary>
		/// Effective longitudinal-force moment arm (m) used for the <c>s * Fx</c>-style aligning-torque term.
		/// This only contributes while the tyre is already generating meaningful lateral force.
		/// </summary>
		public float AligningTorqueFxMomentArm { get; set; } = 0.008f;

		/// <summary>
		/// Scale factor for the tyre overturning couple <c>Mx</c>.
		/// Higher values increase the wheel-axis roll moment generated by lateral load and camber.
		/// </summary>
		public float OverturningCoupleFactor { get; set; } = 0.015f;

		/// <summary>
		/// Additional camber sensitivity for the overturning couple.
		/// Higher values make cambered tyres generate more wheel-axis roll moment.
		/// </summary>
		public float OverturningCamberFactor { get; set; } = 0.35f;

		/// <summary>
		/// Scale factor for the rolling-resistance moment <c>My</c>.
		/// Higher values increase the axle-axis moment that resists wheel rotation.
		/// </summary>
		public float RollingResistanceMomentFactor { get; set; } = 0.006f;

		/// <summary>
		/// Coupling between longitudinal tyre force and rolling-resistance moment.
		/// Higher values make drive/brake force overlap contribute more strongly to <c>My</c>.
		/// </summary>
		public float RollingResistanceMomentFxFactor { get; set; } = 0.08f;

		// ── Thermal / wear ───────────────────────────────────────────────────────

		/// <summary>Thermal mass of the tyre surface (J/°C). Higher = slower temp change.</summary>
		public float ThermalMass { get; set; } = 8000f;

		/// <summary>Cooling rate to ambient (W/°C).</summary>
		public float CoolingRate { get; set; } = 15f;

		/// <summary>Thermal mass of the tyre core/carcass (J/°C). Higher = slower but more persistent heat storage.</summary>
		public float CoreThermalMass { get; set; } = 24000f;

		/// <summary>Conductance from tread surface to core (W/°C).</summary>
		public float SurfaceToCoreConductance { get; set; } = 55f;

		/// <summary>Cooling rate from core/carcass to ambient (W/°C).</summary>
		public float CoreCoolingRate { get; set; } = 4f;

		/// <summary>Road-contact heat transfer rate from tread surface (W/°C).</summary>
		public float RoadHeatTransferRate { get; set; } = 25f;

		/// <summary>Additional road heat-transfer gain per metre of water depth (1/m).</summary>
		public float WaterCoolingGain { get; set; } = 400f;

		/// <summary>Blend of surface and core temperatures used for grip calculations (0 = core only, 1 = surface only).</summary>
		public float GripTemperatureSurfaceWeight { get; set; } = 0.7f;

		/// <summary>Ambient temperature (°C).</summary>
		public float AmbientTemperature { get; set; } = 30f;

		/// <summary>Optimal tyre temperature for peak grip (°C).</summary>
		public float OptimalTemperature { get; set; } = 85f;

		/// <summary>
		/// Temperature window width (°C). Grip falls off outside [optimal ± window].
		/// </summary>
		public float TemperatureWindow { get; set; } = 40f;

		/// <summary>Wear rate: tread life lost per unit of slip energy (1 / J).</summary>
		public float WearRate { get; set; } = 1e-9f;

		/// <summary>Minimum grip multiplier when tyre is completely worn (fraction of peak).</summary>
		public float WornGripFraction { get; set; } = 0.65f;

		/// <summary>
		/// Combined-slip weighting stiffness for the MF-style <c>Gxα</c>/<c>Gyκ</c> reduction functions.
		/// Higher values make cross-slip reduce the other force channel sooner.
		/// </summary>
		public float CombinedSlipCoupling { get; set; } = 0.6f;

		/// <summary>
		/// Combined-slip weighting shape factor for the MF-style <c>Gxα</c>/<c>Gyκ</c> reduction functions.
		/// Values near 1 are gentle; values near 2 collapse force capacity more aggressively at high cross-slip.
		/// </summary>
		public float CombinedSlipExponent { get; set; } = 2.0f;

		// ── Rally high-slip extension ────────────────────────────────────────────

		/// <summary>
		/// Fraction of peak lateral force retained at extreme slip angles (30–40°+).
		/// Rally tyres maintain ~60–70% of peak force at large slip to allow controlled drifts.
		/// Reference: Abdulrahim, "Measurement and Analysis of Rally Car Dynamics at High Attitude Angles".
		/// </summary>
		public float HighSlipForceRetention { get; set; } = 0.65f;

		/// <summary>
		/// Slip angle (rad) at which the rally high-slip retention curve begins blending.
		/// Below this angle the standard Pacejka curve applies fully. ~15° by default.
		/// Adjust per surface type or tyre compound for different drift-entry behaviour.
		/// </summary>
		public float HighSlipTransitionStart { get; set; } = 0.26f;

		/// <summary>
		/// Slip angle (rad) at which the rally high-slip retention curve is fully active. ~40° by default.
		/// </summary>
		public float HighSlipTransitionEnd { get; set; } = 0.70f;

		// ── Rolling resistance ───────────────────────────────────────────────────

		/// <summary>Base rolling-resistance coefficient (dimensionless). Typical 0.01–0.015 for tarmac tyres.</summary>
		public float RollingResistanceCoefficient { get; set; } = 0.012f;

		/// <summary>
		/// Approximate carcass wave speed (m/s) above which standing-wave losses grow rapidly.
		/// Lower values increase high-speed rolling resistance sooner.
		/// </summary>
		public float StandingWaveCriticalSpeed { get; set; } = 65f;

		/// <summary>
		/// Gain applied to the standing-wave rolling-resistance rise.
		/// Effective rolling resistance scales as <c>1 + gain × (speed / criticalSpeed)²</c>.
		/// </summary>
		public float StandingWaveResistanceGain { get; set; } = 1.0f;

		/// <summary>
		/// Scaling factor for free-rolling carcass shear resistance.
		/// The carcass shear contribution arises from the difference between tread-element speed
		/// (Ω·r) and road speed (Ω·rₑ) as the tread compresses through the contact patch.
		/// Reference: "A simple model for tyre deformation", The Contact Patch, §Free rolling, Eq. 8.
		/// </summary>
		public float CarcassShearCoefficient { get; set; } = 0.5f;

		// ── Constants ────────────────────────────────────────────────────────────

		private const float MinSpeed = 0.05f;      // velocity floor for slip calculations (m/s)
		private const float MaxSlipRatio = 1.5f;    // clamp slip ratio to prevent numerical blow-up
		private const float MaxSlipAngle = 1.2f;    // ~69° — beyond this we clamp
		private const float DirectionResolutionAngularVelocityThreshold = 0.5f;
		private const float DirectionResolutionMinimumRollingRadius = 0.05f;
		private const float MinimumDriveTorqueThreshold = 1e-4f;
		/// <summary>
		/// Approximate wheel rotational inertia scalar.
		/// Wheel inertia ≈ effective_mass × R² × scalar.
		/// The 1.2 factor accounts for the distribution of tyre mass (annular ring rather than
		/// point mass). Typical road-tyre I ≈ 0.5–1.5 kg·m²; this scalar yields values in that range.
		/// Reference: Milliken, RCVD §2.3, approximate wheel inertia.
		/// </summary>
		private const float InertiaScalar = 1.2f;
		private const float ReferenceTyreWidth = 0.205f;
		private const float ReferenceContactPatchStiffness = 40000f;
		private const float KilopascalsToPascals = 1000f;
		private const float MinimumRealisticWheelInertia = 1.2f;
		private const float RollingRadiusDeflectionDivisor = 3f;
		private const float SurfaceDeformationBrushSoftening = 0.15f;
		/// <summary>
		/// Converts water-film depth (m) into an equivalent road sink-temperature drop scaling.
		/// Higher values increase cooling sensitivity to standing water.
		/// </summary>
		private const float WaterDepthRoadCoolingScale = 5000f;
		/// <summary>Maximum road sink-temperature reduction from water film (°C).</summary>
		private const float MaxWaterRoadTemperatureOffset = 12f;
		private const float ReferencePressureBrushStiffness = ReferencePressure * KilopascalsToPascals * ReferenceTyreWidth;
		private const float SurfaceRelaxationLowGripBase = 0.2f;
		private const float SurfaceRelaxationLowGripSensitivityScale = 0.25f;
		private const float SurfaceRelaxationRoughnessBase = 0.1f;
		private const float SurfaceRelaxationRoughnessSensitivityScale = 0.1f;
		private const float RelaxationSlopeMinFactor = 0.2f;
		private const float RelaxationSlopeMaxFactor = 2.0f;
		private const float RelaxationDerivativeStep = 0.0025f;

		/// <summary>
		/// Reference tyre pressure (kPa) used to normalise pressure-dependent effects.
		/// 220 kPa ≈ 32 psi, a typical rally tarmac cold-start pressure.
		/// </summary>
		private const float ReferencePressure = 220f;

		// ── Wet grip / hydroplaning constants ─────────────────────────────────────
		// Reference: The Contact Patch, C1603, §Effect of surface water, §Aqua-planing, Table 1.

		/// <summary>
		/// Reference water depth (m) for normalizing wet grip loss.
		/// 4 mm = continuous sheet of water above asperity peaks.
		/// </summary>
		private const float WetGripReferenceDepth = 0.004f;

		/// <summary>
		/// Maximum base grip loss fraction from a water film at reference depth.
		/// Typical dry→wet friction ratio: 0.55/1.0 = 0.45 loss at worst.
		/// Reference: The Contact Patch, C1603, Table 1.
		/// </summary>
		private const float WetGripBaseLoss = 0.60f;

		/// <summary>
		/// Fraction of base wet grip loss recovered by full macrotexture drainage (0–1).
		/// High macrotexture surfaces (positive texture, porous asphalt) drain water
		/// from the contact patch and retain more adhesion grip.
		/// Reference: The Contact Patch, C1603, §Macro-texture.
		/// </summary>
		private const float WetGripDrainageRecovery = 0.50f;

		/// <summary>
		/// Hydroplaning speed constant: V_hydro = C / √(depth_m).
		/// Calibrated so that at 2.5 mm depth, V_hydro ≈ 22 m/s (80 km/h).
		/// C = 22 × √(0.0025) = 22 × 0.05 ≈ 1.1.
		/// Reference: The Contact Patch, C1603, §Aqua-planing.
		/// </summary>
		private const float HydroplaningSpeedConstant = 1.1f;

		/// <summary>
		/// Fraction of V_hydro at which grip reduction begins ramping.
		/// Onset at 70% of critical speed gives a smooth transition zone.
		/// </summary>
		private const float HydroplaningOnsetFraction = 0.7f;

		/// <summary>
		/// Boost to effective hydroplaning speed from surface macrotexture.
		/// Full macrotexture raises V_hydro by 30% (better drainage delays onset).
		/// </summary>
		private const float HydroplaningMacrotextureBoost = 0.30f;

		/// <summary>
		/// Minimum grip fraction during full hydroplaning.
		/// Even at full aquaplaning, a small residual grip remains from hysteresis.
		/// </summary>
		private const float HydroplaningMinGrip = 0.05f;

		/// <summary>
		/// Maximum grip perturbation amplitude from road noise at full NoiseFactor.
		/// ±8% variation provides realistic road-surface PSD feel without destabilizing physics.
		/// Reference: The Contact Patch, C1603, §Statistical modelling of the road surface profile.
		/// </summary>
		private const float RoadNoiseGripAmplitude = 0.08f;

		/// <summary>
		/// Initializes a tyre model for a wheel radius.
		/// </summary>
		/// <param name="radius">Unloaded tyre radius in metres.</param>
		public TyreModel(float radius)
		{
			Radius = MathF.Max(0.1f, radius);
		}

		/// <summary>
		/// Computes all tyre forces and torques for one wheel for one physics step.
		///
		/// <para>Outputs:
		/// <list type="bullet">
		///   <item><paramref name="longitudinalForce"/>: Fx along wheel heading (N). Positive = forward.</item>
		///   <item><paramref name="lateralForce"/>: Fy perpendicular to heading (N). Positive = rightward.</item>
		///   <item><paramref name="selfAligningTorque"/>: Mz about the vertical axis (N·m). Opposes steering.</item>
		/// </list></para>
		/// </summary>
		/// <param name="state">Per-wheel mutable state (angular velocity, temperature, wear, deflection).</param>
		/// <param name="longitudinalVelocity">Forward velocity at the contact patch (m/s).</param>
		/// <param name="lateralVelocity">Lateral velocity at the contact patch (m/s). Positive = rightward.</param>
		/// <param name="normalLoad">Vertical load on the tyre (N). Must be &gt; 0 for contact.</param>
		/// <param name="driveTorque">Torque applied to the wheel from the drivetrain (N·m).</param>
		/// <param name="brakeTorque">Braking torque magnitude (N·m). Always opposes rotation.</param>
		/// <param name="camberAngle">Camber angle (rad). Positive = top of wheel tilted outward.</param>
		/// <param name="surface">Surface material properties affecting grip.</param>
		/// <param name="dt">Physics timestep (s).</param>
		/// <param name="longitudinalForce">Output: longitudinal force (N).</param>
		/// <param name="lateralForce">Output: lateral force (N).</param>
		/// <param name="selfAligningTorque">Output: self-aligning torque (N·m).</param>
		public void Update(
			ref TyreState state,
			float longitudinalVelocity,
			float lateralVelocity,
			float normalLoad,
			float driveTorque,
			float brakeTorque,
			float camberAngle,
			in SurfaceProperties surface,
			float dt,
			out float longitudinalForce,
			out float lateralForce,
			out float selfAligningTorque)
		{
			Update(
				ref state,
				longitudinalVelocity,
				lateralVelocity,
				normalLoad,
				driveTorque,
				brakeTorque,
				camberAngle,
				in surface,
				dt,
				out longitudinalForce,
				out lateralForce,
				out selfAligningTorque,
				out _,
				out _);
		}

		/// <summary>
		/// Computes all tyre forces and moments for one wheel for one physics step.
		/// </summary>
		/// <param name="state">Per-wheel mutable state (angular velocity, temperature, wear, deflection).</param>
		/// <param name="longitudinalVelocity">Forward velocity at the contact patch (m/s).</param>
		/// <param name="lateralVelocity">Lateral velocity at the contact patch (m/s). Positive = rightward.</param>
		/// <param name="normalLoad">Vertical load on the tyre (N). Must be &gt; 0 for contact.</param>
		/// <param name="driveTorque">Torque applied to the wheel from the drivetrain (N·m).</param>
		/// <param name="brakeTorque">Braking torque magnitude (N·m). Always opposes rotation.</param>
		/// <param name="camberAngle">Camber angle (rad). Positive = top of wheel tilted outward.</param>
		/// <param name="surface">Surface material properties affecting grip.</param>
		/// <param name="dt">Physics timestep (s).</param>
		/// <param name="longitudinalForce">Output: longitudinal force (N).</param>
		/// <param name="lateralForce">Output: lateral force (N).</param>
		/// <param name="selfAligningTorque">Output: self-aligning torque (N·m).</param>
		/// <param name="overturningCouple">Output: overturning couple <c>Mx</c> about the wheel longitudinal axis (N·m).</param>
		/// <param name="rollingResistanceMoment">Output: rolling-resistance moment <c>My</c> about the wheel lateral axis (N·m).</param>
		public void Update(
			ref TyreState state,
			float longitudinalVelocity,
			float lateralVelocity,
			float normalLoad,
			float driveTorque,
			float brakeTorque,
			float camberAngle,
			in SurfaceProperties surface,
			float dt,
			out float longitudinalForce,
			out float lateralForce,
			out float selfAligningTorque,
			out float overturningCouple,
			out float rollingResistanceMoment)
		{
			longitudinalForce = 0f;
			lateralForce = 0f;
			selfAligningTorque = 0f;
			overturningCouple = 0f;
			rollingResistanceMoment = 0f;
			state.DriveTorque = driveTorque;
			state.BrakeTorque = brakeTorque;

			if (dt < 1e-6f)
			{
				state.LateralDeflection = 0f;
				state.LongitudinalDeflection = 0f;
				state.TyreReactionTorque = 0f;
				return;
			}

			var wheelInertia = WheelInertia > 0f
				? WheelInertia
				: ComputeEstimatedWheelInertia(MathF.Max(normalLoad, ReferenceLoad));
			wheelInertia = MathF.Max(wheelInertia, MinimumRealisticWheelInertia);
			var brakeReactionTorque = ComputeBrakeReactionTorque(
				brakeTorque,
				state.AngularVelocity,
				longitudinalVelocity,
				driveTorque,
				normalLoad > 0f ? ComputeEffectiveRollingRadius(normalLoad) : Radius);

			// When airborne (normalLoad ≤ 0), skip force computation but still integrate
			// angular velocity from drive/brake torque so wheels spin up in the air.
			// This is important for AWD diffs and realistic landing behaviour.
			var airborne = normalLoad <= 0f;
			if (airborne)
			{
				state.LateralDeflection = 0f;
				state.LongitudinalDeflection = 0f;
				state.SlipRatio = 0f;
				state.SlipAngle = 0f;
				state.TyreReactionTorque = 0f;
				IntegrateWheelAngularVelocity(
					ref state,
					driveTorque,
					brakeReactionTorque,
					0f,
					wheelInertia,
					longitudinalVelocity,
					Radius,
					dt);
				return;
			}

			// ── Effective friction coefficient ───────────────────────────────────
			// Combines surface µ, load sensitivity, temperature, wear, wet grip,
			// hydroplaning, and road roughness noise effects.
			var gripTemperatureSurfaceWeight = Math.Clamp(GripTemperatureSurfaceWeight, 0f, 1f);
			var gripTemperature = gripTemperatureSurfaceWeight * state.Temperature
			                      + (1f - gripTemperatureSurfaceWeight) * state.CoreTemperature;
			var mu = ComputeEffectiveFriction(normalLoad, surface, gripTemperature,
				state.TreadLife, absVx: MathF.Abs(longitudinalVelocity), dt: dt);

			var effectiveRollingRadius = ComputeEffectiveRollingRadius(normalLoad);
			state.TyreReactionTorque = 0f;
			var predictedAngularVelocity = state.AngularVelocity
			                              + ((driveTorque - brakeReactionTorque) / wheelInertia) * dt;

			// ── Slip ratio (longitudinal) ────────────────────────────────────────
			// κ = (ω·R − Vx) / max(|Vx|, ε)
			// Positive κ means wheel spinning faster than ground (acceleration).
			// Reference: Pacejka, §2.2, Eq. 2.5.
			var wheelLinearSpeed = predictedAngularVelocity * effectiveRollingRadius;
			var absVx = MathF.Abs(longitudinalVelocity);
			var denominator = MathF.Max(absVx, MinSpeed);
			var slipRatio = (wheelLinearSpeed - longitudinalVelocity) / denominator;
			slipRatio = Math.Clamp(slipRatio, -MaxSlipRatio, MaxSlipRatio);
			state.SlipRatio = slipRatio;

			// ── Slip angle (lateral) ─────────────────────────────────────────────
			// α = atan(-Vy / |Vx|)
			// Positive contact-patch lateral velocity means the wheel is moving to the right
			// relative to its heading, so the tyre reaction force must act to the left.
			// Reference: Pacejka, §1.3, Eq. 1.4.
			var slipAngle = MathF.Atan2(-lateralVelocity, MathF.Max(absVx, MinSpeed));
			slipAngle = Math.Clamp(slipAngle, -MaxSlipAngle, MaxSlipAngle);
			state.SlipAngle = slipAngle;

			// ── Pacejka Magic Formula forces ─────────────────────────────────────
			// F = D · sin(C · atan(B·x − E·(B·x − atan(B·x))))
			// D = µ · Fz (peak force proportional to load)
			var peakForce = mu * normalLoad;
			var referenceLoad = MathF.Max(ReferenceLoad, 1f);
			var loadRatio = MathF.Max(normalLoad / referenceLoad, 1e-3f);
			var longitudinalCoefficients = EvaluateLongitudinalPureSlipCoefficients(
				peakForce,
				loadRatio,
				surface);
			var lateralCoefficients = EvaluateLateralPureSlipCoefficients(
				peakForce,
				loadRatio,
				camberAngle,
				surface);

			// Longitudinal force Fx(κ)
			var rawFx = MagicFormula(
				slipRatio,
				longitudinalCoefficients.B,
				longitudinalCoefficients.C,
				longitudinalCoefficients.D,
				longitudinalCoefficients.E);

			// Lateral force Fy(α) — with rally high-slip extension.
			var rawFy = MagicFormulaRally(
				slipAngle,
				lateralCoefficients.B,
				lateralCoefficients.C,
				lateralCoefficients.D,
				lateralCoefficients.E);

			// ── Contact-patch brush model (lateral transient) ────────────────────
			// The brush model smooths lateral force buildup through a deflection state.
			// dδ/dt = Vy − δ · |Vx| / L     (L = relaxation length)
			// CarcassStiffness modifies relaxation length: stiffer carcass → shorter relaxation.
			//   effectiveRelaxationLength = RelaxationLength / CarcassStiffness
			// Reference: Pacejka §5.4, Eq. 5.31 (relaxation length model).
			var referenceMu = ComputeEffectiveFriction(referenceLoad, surface, gripTemperature,
				state.TreadLife, absVx: absVx, dt: dt);
			var referencePeakForce = referenceMu * referenceLoad;
			var referenceLongitudinalCoefficients = EvaluateLongitudinalPureSlipCoefficients(
				referencePeakForce,
				loadRatio: 1f,
				surface);
			var referenceLateralCoefficients = EvaluateLateralPureSlipCoefficients(
				referencePeakForce,
				loadRatio: 1f,
				camberAngle,
				surface);
			var lateralRelaxationSlopeScale = ComputeRelaxationLengthOperatingPointScale(
				longitudinal: false,
				slipRatio,
				slipAngle,
				lateralCoefficients,
				referenceLateralCoefficients);
			var effectiveRelaxation = ComputeEffectiveRelaxationLength(
				surface,
				longitudinal: false,
				lateralRelaxationSlopeScale);
			var relaxSpeed = MathF.Max(absVx, 2f);
			var deflectionRate = lateralVelocity -
			                     (state.LateralDeflection * relaxSpeed / MathF.Max(effectiveRelaxation, 0.05f));
			state.LateralDeflection += deflectionRate * dt;

			// ── Contact-patch dimensions for dual-zone brush model ───────────────
			// Contact patch half-length a = patchLength / 2.
			// Brush stiffness c per unit length relates to the overall brush stiffness.
			// Reference: "A simple model for tyre deformation", The Contact Patch, §Cornering.
			var effectivePatchLength = ComputeEffectivePatchLength(normalLoad);
			var halfPatch = MathF.Max(effectivePatchLength * 0.5f, 0.005f);
			var effectiveBrushStiffness = ComputeEffectiveBrushStiffness()
			                              * ComputeSurfaceSlipStiffnessScale(surface)
			                              * (1f - surface.DeformationFactor * SurfaceDeformationBrushSoftening);

			// ── Dual-zone lateral brush: adhesion/sliding frontier (λ) ───────────
			// λ is the adhesion fraction of the contact patch (1 = full adhesion, 0 = full slide).
			// In the adhesion-only regime (small α):  Qy = 2a²c·tan(α)
			// Transition threshold: tan(α) = µP / (4a²c)  →  λ = µP / (4a²c·tan(α))
			// In the mixed regime:  Qy = (1 − λ/2) · µ · P
			// Reference: The Contact Patch, C2015, Eq. 12, 15, 18.
			var absTanAlpha = MathF.Abs(MathF.Tan(slipAngle));
			var brushCperLength = effectiveBrushStiffness / MathF.Max(effectivePatchLength, 0.01f);
			var lambda = ComputeAdhesionFraction(absTanAlpha, peakForce, halfPatch, brushCperLength);

			// Clamp lateral deflection using the adhesion zone length rather than full patch
			var maxLateralDeflection = MathF.Max(halfPatch * lambda, 0.005f);
			state.LateralDeflection = Math.Clamp(state.LateralDeflection, -maxLateralDeflection, maxLateralDeflection);

			// Blend brush-model transient force with steady-state Pacejka.
			// ActiveMode overrides the blend: BrushOnly forces blendAlpha=0 (pure brush),
			// PacejkaOnly forces blendAlpha=1 (pure Pacejka), Auto uses slip-velocity heuristic.
			var brushForce = -effectiveBrushStiffness * state.LateralDeflection;
			float blendAlphaLat;
			if (ActiveMode == TyreModelMode.BrushOnly)
			{
				blendAlphaLat = 0f;
			}
			else if (ActiveMode == TyreModelMode.PacejkaOnly)
			{
				blendAlphaLat = 1f;
			}
			else
			{
				var latSlipVel = MathF.Abs(lateralVelocity);
				var latSlideBlend = Math.Clamp(latSlipVel - 1f, 0f, 1f);
				blendAlphaLat = MathF.Min(latSlideBlend, 0.85f);
			}
			var blendedFy = blendAlphaLat * rawFy + (1f - blendAlphaLat) * brushForce;

			// ── Contact-patch brush model (longitudinal transient) ───────────────
			// Mirrors the lateral transient for longitudinal forces.
			// dδx/dt = (ωR − Vx) − δx · |Vx| / Lx
			// This gives smoother brake-pedal feel and torque application.
			// Reference: brush model braking, The Contact Patch C2015, Eq. 21–28.
			var longitudinalRelaxationSlopeScale = ComputeRelaxationLengthOperatingPointScale(
				longitudinal: true,
				slipRatio,
				slipAngle,
				longitudinalCoefficients,
				referenceLongitudinalCoefficients);
			var effectiveLongRelaxation = ComputeEffectiveRelaxationLength(
				surface,
				longitudinal: true,
				longitudinalRelaxationSlopeScale);
			var longSlipVelocity = wheelLinearSpeed - longitudinalVelocity;
			var longDeflectionRate = longSlipVelocity -
			                         (state.LongitudinalDeflection * relaxSpeed / MathF.Max(effectiveLongRelaxation, 0.05f));
			state.LongitudinalDeflection += longDeflectionRate * dt;

			// Clamp longitudinal deflection using the adhesion zone length.
			// The braking analogue of λ uses K = S/(1−S) in place of tan(α).
			var absK = MathF.Abs(slipRatio) / MathF.Max(1f - MathF.Abs(slipRatio), 0.01f);
			var lambdaLong = ComputeAdhesionFraction(absK, peakForce, halfPatch, brushCperLength);
			var maxLongDeflection = MathF.Max(halfPatch * lambdaLong, 0.005f);
			state.LongitudinalDeflection = Math.Clamp(state.LongitudinalDeflection,
				-maxLongDeflection, maxLongDeflection);

			var brushFx = effectiveBrushStiffness * state.LongitudinalDeflection;
			var longSlipVelMag = MathF.Abs(wheelLinearSpeed - longitudinalVelocity);
			float blendAlphaLong;
			if (ActiveMode == TyreModelMode.BrushOnly)
			{
				blendAlphaLong = 0f;
			}
			else if (ActiveMode == TyreModelMode.PacejkaOnly)
			{
				blendAlphaLong = 1f;
			}
			else
			{
				var longSlideBlend = Math.Clamp((longSlipVelMag - 1f) / 2f, 0f, 1f);
				blendAlphaLong = MathF.Min(longSlideBlend, 0.85f);
			}
			var blendedFx = blendAlphaLong * rawFx + (1f - blendAlphaLong) * brushFx;
			ApplyCombinedSlipInteraction(ref blendedFx, ref blendedFy, slipRatio, slipAngle);

			// ── Camber thrust ────────────────────────────────────────────────────
			// Small lateral force from wheel inclination. Fy_camber = γ · Cγ · Fz.
			// Reference: Pacejka §4.3.
			var camberForce = camberAngle * CamberThrustCoefficient * normalLoad;
			var totalFy = blendedFy + camberForce;

			// ── Rolling resistance ───────────────────────────────────────────────
			// Frr = Crr · Fz, opposing wheel direction of travel.
			// Use a small deadband near zero longitudinal speed so rolling resistance
			// does not inject a non-zero force at rest due to floating-point noise.
			// Reference: Milliken, "Race Car Vehicle Dynamics", §2.2.
			var rollingReferenceSpeed = MathF.Max(MathF.Abs(longitudinalVelocity), MathF.Abs(wheelLinearSpeed));
			var rollingResistance = MathF.Max(RollingResistanceCoefficient + surface.RollingResistance, 0f)
			                        * ComputeStandingWaveResistanceFactor(rollingReferenceSpeed)
			                        * normalLoad;
			const float rollingResistanceDeadband = 0.01f;
			var rrForce = 0f;
			if (MathF.Abs(longitudinalVelocity) > rollingResistanceDeadband)
			{
				rrForce = longitudinalVelocity > 0f ? -rollingResistance : rollingResistance;
			}

			// ── Free-rolling carcass shear resistance ────────────────────────────
			// Even in free rolling, carcass compression creates fore-aft shear in the
			// contact patch. The net retarding force scales with (r − rₑ) and the
			// brush stiffness. The contact patch half-angle θ ≈ a / r.
			// Reference: The Contact Patch, C2015, §Free rolling, Eq. 7–8.
			var carcassShearForce = 0f;
			if (MathF.Abs(longitudinalVelocity) > rollingResistanceDeadband)
			{
				var radiusDelta = Radius - effectiveRollingRadius;
				var shearMagnitude = ComputeCarcassShearForce(radiusDelta, halfPatch,
					brushCperLength, effectivePatchLength);
				carcassShearForce = longitudinalVelocity > 0f ? -shearMagnitude : shearMagnitude;
			}

			var tyreLongitudinalForce = blendedFx;
			var tyreLateralForce = totalFy;
			ClampToFrictionEllipse(ref tyreLongitudinalForce, ref tyreLateralForce, peakForce);

			longitudinalForce = tyreLongitudinalForce + rrForce + carcassShearForce;
			lateralForce = tyreLateralForce;

			// ── Self-aligning torque (combined-slip trail + residual + Fx moment arm) ───
			// The pneumatic trail follows a characteristic rise-then-collapse curve from
			// the brush model's adhesion/sliding frontier λ.
			// Pure adhesion:  trail = a/3  (resultant 2/3 along the patch)
			// Mixed regime:   trail = a · λ · (1 − 2λ/3) / (2 · (1 − λ/2))
			// This gives the realistic steering-going-light effect near the grip limit.
			// For combined slip, evaluate trail on an equivalent side-slip input so brake/drive
			// overlap also unloads steering torque. Reference: Pacejka Eq. 4.E71-4.E76.
			var corneringStiffness = MathF.Abs(ComputeLateralForceSlopePerTanAlpha(
				0f,
				lateralCoefficients.B,
				lateralCoefficients.C,
				lateralCoefficients.D,
				lateralCoefficients.E));
			var longitudinalStiffness = MathF.Abs(ComputeLongitudinalForceSlope(
				0f,
				longitudinalCoefficients.B,
				longitudinalCoefficients.C,
				longitudinalCoefficients.D,
				longitudinalCoefficients.E));
			var stiffnessRatio = longitudinalStiffness / MathF.Max(corneringStiffness, 1f);
			var equivalentTrailSlip = ComputeEquivalentTrailSlip(MathF.Tan(slipAngle), slipRatio, stiffnessRatio);
			var trailLambda = ComputeAdhesionFraction(MathF.Abs(equivalentTrailSlip), peakForce, halfPatch, brushCperLength);
			var pneumaticTrail = ComputeBrushPneumaticTrail(halfPatch, trailLambda);

			// Scale by the configured PneumaticTrail property as a reference calibration.
			// The brush model predicts trail ≈ a/3 at zero slip; PneumaticTrail is the user's
			// desired value at zero slip, so we scale accordingly.
			var referenceTrail = halfPatch / 3f;
			var trailScale = PneumaticTrail / MathF.Max(referenceTrail, 0.001f);
			pneumaticTrail *= trailScale;

			var residualAligningTorque = ComputeResidualAligningTorque(equivalentTrailSlip, peakForce, trailLambda);
			var fxMomentArm = ComputeFxAligningMomentArm(tyreLateralForce, peakForce, camberAngle);
			selfAligningTorque = pneumaticTrail * lateralForce + residualAligningTorque + fxMomentArm * tyreLongitudinalForce;
			overturningCouple = ComputeOverturningCouple(
				normalLoad,
				tyreLateralForce,
				peakForce * MathF.Max(FrictionEllipseRatio, 0.1f),
				camberAngle,
				effectiveRollingRadius);
			rollingResistanceMoment = ComputeRollingResistanceMoment(
				effectiveRollingRadius,
				rrForce,
				carcassShearForce,
				rollingReferenceSpeed,
				tyreLongitudinalForce,
				peakForce,
				longitudinalVelocity != 0f ? longitudinalVelocity : wheelLinearSpeed);

			// ── Wheel angular velocity integration ───────────────────────────────
			// Iω̇ = T_drive − T_brake − T_tyre
			// Tyre reaction torque is generated from the longitudinal contact-patch force
			// and must oppose wheel rotation. Keep this term separate from rolling/carcass
			// resistances to preserve explicit torque flow through the tyre model.
			var tyreReactionTorque = tyreLongitudinalForce * effectiveRollingRadius;
			state.TyreReactionTorque = tyreReactionTorque;
			IntegrateWheelAngularVelocity(
				ref state,
				driveTorque,
				brakeReactionTorque,
				tyreReactionTorque,
				wheelInertia,
				longitudinalVelocity,
				effectiveRollingRadius,
				dt);

			// ── Thermal model ────────────────────────────────────────────────────
			// Two-node thermal model:
			//  - surface tread node (fast response from slip dissipation + road/air exchange)
			//  - core/carcass node (slow storage with lagged cooling)
			// This captures warm-up transients and hysteresis better than a single-lump model.
			var slipSpeed = MathF.Sqrt(
				(wheelLinearSpeed - longitudinalVelocity) * (wheelLinearSpeed - longitudinalVelocity)
				+ lateralVelocity * lateralVelocity);
			var slipForceMag = MathF.Sqrt(longitudinalForce * longitudinalForce + lateralForce * lateralForce);
			var slipPower = slipForceMag * slipSpeed;
			var surfaceToCorePower = SurfaceToCoreConductance * (state.Temperature - state.CoreTemperature);
			var clampedWaterDepth = MathF.Max(surface.WaterDepth, 0f);
			var roadCoolingRate = RoadHeatTransferRate * (1f + surface.Macrotexture * 0.5f
			                                                 + clampedWaterDepth * WaterCoolingGain);
			var roadSinkTemperature = AmbientTemperature - MathF.Min(
				clampedWaterDepth * WaterDepthRoadCoolingScale,
				MaxWaterRoadTemperatureOffset);
			var surfaceAirPower = CoolingRate * (state.Temperature - AmbientTemperature);
			var surfaceRoadPower = roadCoolingRate * (state.Temperature - roadSinkTemperature);
			var coreAmbientPower = CoreCoolingRate * (state.CoreTemperature - AmbientTemperature);

			state.Temperature += (slipPower - surfaceToCorePower - surfaceAirPower - surfaceRoadPower) * dt
			                     / MathF.Max(ThermalMass, 1f);
			state.CoreTemperature += (surfaceToCorePower - coreAmbientPower) * dt
			                         / MathF.Max(CoreThermalMass, 1f);
			var minimumTemperature = AmbientTemperature - MaxWaterRoadTemperatureOffset;
			state.Temperature = MathF.Max(state.Temperature, minimumTemperature);
			state.CoreTemperature = MathF.Max(state.CoreTemperature, minimumTemperature);

			// ── Wear model ───────────────────────────────────────────────────────
			// Tread life decreases proportionally to slip energy.
			// Reference: simplified abrasion model from RCVD §2.8.
			var wearEnergy = slipForceMag * slipSpeed * dt;
			state.TreadLife -= wearEnergy * WearRate;
			state.TreadLife = MathF.Max(state.TreadLife, 0f);
		}

		private static float ComputeBrakeReactionTorque(
			float brakeTorque,
			float angularVelocity,
			float longitudinalVelocity,
			float driveTorque,
			float rollingRadius)
		{
			if (brakeTorque <= 0f)
			{
				return 0f;
			}

			var rollingDirection = ResolveRotationDirection(angularVelocity, longitudinalVelocity, driveTorque, rollingRadius);
			return MathF.Abs(rollingDirection) < 1e-6f ? 0f : brakeTorque * rollingDirection;
		}

		private static void IntegrateWheelAngularVelocity(
			ref TyreState state,
			float driveTorque,
			float brakeReactionTorque,
			float tyreReactionTorque,
			float wheelInertia,
			float longitudinalVelocity,
			float rollingRadius,
			float dt)
		{
			var previousAngularVelocity = state.AngularVelocity;
			var netTorque = driveTorque - brakeReactionTorque - tyreReactionTorque;
			state.AngularVelocity += (netTorque / MathF.Max(wheelInertia, 1e-3f)) * dt;

			if (MathF.Abs(brakeReactionTorque) <= 1e-6f)
			{
				return;
			}

			var brakingDirection = ResolveRotationDirection(previousAngularVelocity, longitudinalVelocity, driveTorque, rollingRadius);
			if (brakingDirection > 0f && state.AngularVelocity < 0f)
			{
				state.AngularVelocity = 0f;
			}
			else if (brakingDirection < 0f && state.AngularVelocity > 0f)
			{
				state.AngularVelocity = 0f;
			}
		}

		private static float ResolveRotationDirection(
			float angularVelocity,
			float longitudinalVelocity,
			float driveTorque,
			float rollingRadius)
		{
			if (MathF.Abs(angularVelocity) > DirectionResolutionAngularVelocityThreshold)
			{
				return MathF.Sign(angularVelocity);
			}

			if (MathF.Abs(longitudinalVelocity) > MinSpeed)
			{
				return MathF.Sign(longitudinalVelocity);
			}

			var wheelLinearSpeed = angularVelocity * MathF.Max(rollingRadius, DirectionResolutionMinimumRollingRadius);
			if (MathF.Abs(wheelLinearSpeed) > MinSpeed)
			{
				return MathF.Sign(wheelLinearSpeed);
			}

			if (MathF.Abs(driveTorque) > MinimumDriveTorqueThreshold)
			{
				return MathF.Sign(driveTorque);
			}

			return 0f;
		}

		/// <summary>
		/// Computes effective friction coefficient accounting for surface, load sensitivity,
		/// temperature window, tread wear, wet grip reduction, hydroplaning, microtexture/macrotexture,
		/// and road roughness noise.
		///
		/// <para>Wet grip model (three-zone contact patch):
		/// Water on the road creates a leading wedge that lifts the tyre tread.
		/// Macrotexture drains water from the contact patch; microtexture penetrates the
		/// remaining thin film to restore adhesion. At higher speeds or deeper water, the
		/// drainage capacity is overwhelmed and grip collapses toward hydroplaning.
		/// Reference: The Contact Patch, C1603, §Dispersal of the water film, §Aqua-planing.</para>
		///
		/// <para>Hydroplaning onset:
		/// Grip collapses above a critical speed V_hydro ∝ 1/√(waterDepth).
		/// At 2.5 mm film depth, V_hydro ≈ 80 km/h (22 m/s).
		/// The transition uses a smooth squared ramp to avoid discontinuities.
		/// Reference: The Contact Patch, C1603, §Aqua-planing; O'Flaherty, film depth ≈ 2.5 mm.</para>
		///
		/// <para>Road roughness (NoiseFactor):
		/// A deterministic per-frame micro-variation in µ derived from speed and a
		/// low-frequency hash, modelling road-surface PSD excitation at the contact patch.
		/// Amplitude scales with NoiseFactor; frequency increases with speed.
		/// Reference: The Contact Patch, C1603, §Power spectral density curves.</para>
		/// </summary>
		internal float ComputeEffectiveFriction(float normalLoad, in SurfaceProperties surface,
			float temperature, float treadLife, float absVx = 0f, float dt = 0.01f)
		{
			// Base µ from surface type. SurfaceProperties.FrictionCoefficient is the
			// calibrated overall peak-µ multiplier for the surface, so do not apply an
			// additional dry-texture multiplier here.
			var mu = PeakFrictionCoefficient * surface.FrictionCoefficient;

			// Clamp texture descriptors for downstream effects such as wet-grip
			// retention. They are not applied again to dry baseline µ here.
			// Reference: The Contact Patch, C1603, §Road surface texture.
			var microLevel = Math.Clamp(surface.Microtexture, 0f, 1f);
			var macroLevel = Math.Clamp(surface.Macrotexture, 0f, 1f);

			// ── Wet grip reduction ───────────────────────────────────────────────
			// Water film reduces adhesion (microtexture) grip while hysteresis
			// (macrotexture) grip is partially retained. Macrotexture helps evacuate
			// water from the contact patch, preserving more grip.
			// Reference: The Contact Patch, C1603, §Effect of surface water.
			var waterDepth = MathF.Max(surface.WaterDepth, 0f);
			if (waterDepth > 0f)
			{
				mu *= ComputeWetGripFactor(waterDepth, macroLevel, absVx);
			}

			var referenceLoad = MathF.Max(ReferenceLoad, 1f);
			var beamNgLoadScale = ComputeBeamNgLoadScale(normalLoad, referenceLoad);
			if (beamNgLoadScale is { } loadScale)
			{
				mu *= loadScale;
			}
			else
			{
				// Load sensitivity: rubber grip follows a non-linear power-law with load.
				var loadRatio = MathF.Max(normalLoad / referenceLoad, 1e-3f);
				mu *= MathF.Pow(loadRatio, -LoadSensitivity);
			}

			// Effective patch area contributes a small secondary grip change.
			var areaGripExponent = MathF.Max(ContactAreaGripExponent, 0f);
			if (areaGripExponent > 0f)
			{
				var effectivePatchArea = ComputeEffectivePatchArea(normalLoad);
				var referencePatchArea = ComputeReferencePatchArea();
				var patchAreaRatio = effectivePatchArea / MathF.Max(referencePatchArea, 1e-6f);
				mu *= MathF.Pow(MathF.Max(patchAreaRatio, 1e-3f), areaGripExponent);
			}

			// Temperature effect: peak grip at optimal temperature, falls off outside window.
			// Reference: Salaani, SAE 2007-01-0816, Fig. 7.
			var tempDelta = MathF.Abs(temperature - OptimalTemperature);
			var tempWindow = MathF.Max(TemperatureWindow, 1f);
			var tempFactor = MathF.Max(0.7f, 1f - 0.3f * (tempDelta / tempWindow));
			mu *= tempFactor;

			// Wear effect: worn tyre has reduced grip.
			var wearFactor = WornGripFraction + (1f - WornGripFraction) * Math.Clamp(treadLife, 0f, 1f);
			mu *= wearFactor;

			// ── Road roughness micro-variation (NoiseFactor) ─────────────────────
			// Applies a small deterministic perturbation to µ based on speed, modelling
			// the effect of road-surface PSD on instantaneous grip at the contact patch.
			// Uses a fast integer hash of speed-scaled position to avoid heap allocations.
			// Reference: The Contact Patch, C1603, §Power spectral density curves.
			var noiseFactor = Math.Clamp(surface.NoiseFactor, 0f, 1f);
			if (noiseFactor > 0f)
			{
				mu *= ComputeRoadNoiseGripFactor(noiseFactor, absVx, dt);
			}

			return MathF.Max(mu, 0.05f);
		}

		internal static float ComputeBeamNgLoadCoefficient(
			float normalLoad,
			float noLoadCoef,
			float fullLoadCoef,
			float loadSensitivitySlope)
		{
			var load = MathF.Max(normalLoad, 0f);
			var slope = MathF.Max(loadSensitivitySlope, 0f);
			return fullLoadCoef + (noLoadCoef - fullLoadCoef) * MathF.Exp(-slope * load);
		}

		private float? ComputeBeamNgLoadScale(float normalLoad, float referenceLoad)
		{
			if (BeamNgNoLoadFrictionCoefficient is not { } noLoadCoef ||
			    BeamNgFullLoadFrictionCoefficient is not { } fullLoadCoef ||
			    BeamNgLoadSensitivitySlope is not { } loadSensitivitySlope)
			{
				return null;
			}

			var referenceMu = ComputeBeamNgLoadCoefficient(referenceLoad, noLoadCoef, fullLoadCoef, loadSensitivitySlope);
			if (referenceMu <= 1e-4f)
			{
				return null;
			}

			var loadMu = ComputeBeamNgLoadCoefficient(normalLoad, noLoadCoef, fullLoadCoef, loadSensitivitySlope);
			return MathF.Max(loadMu / referenceMu, 0.05f);
		}

		/// <summary>
		/// Computes the wet grip reduction factor from water film depth, surface macrotexture,
		/// and vehicle speed. Includes hydroplaning onset.
		///
		/// <para>The model has two components:</para>
		/// <list type="number">
		///   <item>Base wet grip loss: thin film reduces adhesion grip by ~30–50%.
		///         Macrotexture drainage capacity mitigates this loss.
		///         Reference: The Contact Patch, C1603, Table 1 (wet µ ≈ 0.2–0.65 vs dry 0.8–1.0).</item>
		///   <item>Speed-dependent hydroplaning: above a critical speed V_hydro, the
		///         leading water wedge lifts the tyre and grip collapses.
		///         V_hydro = HydroplaningSpeedConstant / √(waterDepth).
		///         Reference: The Contact Patch, C1603, §Aqua-planing.</item>
		/// </list>
		/// </summary>
		/// <param name="waterDepth">Surface water film thickness (m).</param>
		/// <param name="macrotexture">Surface macrotexture level (0–1).</param>
		/// <param name="absSpeed">Absolute longitudinal speed at the contact patch (m/s).</param>
		/// <returns>Grip multiplier in range [HydroplaningMinGrip, 1.0].</returns>
		internal static float ComputeWetGripFactor(float waterDepth, float macrotexture, float absSpeed)
		{
			if (waterDepth <= 0f)
			{
				return 1f;
			}

			// Clamp inputs to documented ranges.
			macrotexture = Math.Clamp(macrotexture, 0f, 1f);
			absSpeed = MathF.Max(absSpeed, 0f);

			// ── 1. Base wet grip loss ────────────────────────────────────────────
			// Water film depth normalized to a reference heavy-rain depth (4 mm).
			// At reference depth with zero macrotexture, grip drops to ~40% of dry.
			// Macrotexture drainage recovers up to 50% of the loss.
			var normalizedDepth = Math.Clamp(waterDepth / WetGripReferenceDepth, 0f, 1f);
			var drainageRecovery = macrotexture * WetGripDrainageRecovery;
			var baseLoss = normalizedDepth * WetGripBaseLoss * (1f - drainageRecovery);
			var wetFactor = 1f - baseLoss;

			// ── 2. Speed-dependent hydroplaning ──────────────────────────────────
			// Critical speed: V_hydro = C / √(depth_m)
			// Above this speed, the water wedge extends to cover the full contact patch.
			// Transition uses a smooth squared ramp for numerical stability.
			var hydroSpeed = HydroplaningSpeedConstant / MathF.Sqrt(waterDepth);

			// Macrotexture raises the effective hydroplaning speed (better drainage delays onset).
			hydroSpeed *= 1f + macrotexture * HydroplaningMacrotextureBoost;

			if (absSpeed > hydroSpeed * HydroplaningOnsetFraction)
			{
				var onset = hydroSpeed * HydroplaningOnsetFraction;
				var t = Math.Clamp((absSpeed - onset) / MathF.Max(hydroSpeed - onset, 0.1f), 0f, 1f);
				var hydroFactor = 1f - t * t * (1f - HydroplaningMinGrip);
				wetFactor *= hydroFactor;
			}

			return MathF.Max(wetFactor, HydroplaningMinGrip);
		}

		/// <summary>
		/// Computes a deterministic grip micro-variation factor from road roughness (NoiseFactor).
		/// Models the effect of road-surface PSD on instantaneous grip at the contact patch.
		/// Uses a fast integer hash (Murmur-style finalizer) of a distance-proportional phase counter.
		/// Reference: The Contact Patch, C1603, §Power spectral density curves.
		/// </summary>
		/// <param name="noiseFactor">Road roughness amplitude (0–1).</param>
		/// <param name="absSpeed">Absolute longitudinal speed (m/s).</param>
		/// <param name="dt">Physics timestep for this update (s).</param>
		/// <returns>Grip multiplier in range [1 − noiseFactor × 0.08, 1 + noiseFactor × 0.08].</returns>
		internal static float ComputeRoadNoiseGripFactor(float noiseFactor, float absSpeed, float dt)
		{
			// At very low speed there is negligible PSD excitation — return unity
			// to avoid a constant bias from a stalled hash value.
			const float noiseSpeedEpsilon = 0.5f;
			if (absSpeed < noiseSpeedEpsilon)
			{
				return 1f;
			}

			// Advance a per-thread phase counter proportional to distance travelled this step.
			// The hash rate increases with speed, modelling higher-frequency PSD excitation.
			// ThreadStatic ensures each physics thread gets its own independent phase —
			// this is intentionally non-reproducible across threads since the noise
			// represents stochastic road-surface variation.
			_noisePhase += absSpeed * dt * 0.1f;
			if (_noisePhase > 1e6f) _noisePhase -= 1e6f;

			// Murmur3-style integer finalizer for good hash distribution.
			// Input: phase scaled to integer range for bit mixing.
			var input = (uint)(int)(_noisePhase * 1000f);
			input ^= input >> 16;
			input *= 0x85ebca6b;
			input ^= input >> 13;
			input *= 0xc2b2ae35;
			input ^= input >> 16;

			// Map to [-1, 1] range.
			var normalized = (input & 0x7FFFFFFF) / (float)0x7FFFFFFF * 2f - 1f;

			// Scale perturbation: ±8% at full NoiseFactor.
			return 1f + normalized * noiseFactor * RoadNoiseGripAmplitude;
		}

		/// <summary>
		/// Accumulated phase for road noise grip perturbation.
		/// ThreadStatic so each physics thread maintains independent state without contention.
		/// Intentionally non-deterministic across threads — the noise represents stochastic
		/// road-surface PSD variation rather than a reproducible simulation input.
		/// </summary>
		[ThreadStatic] private static float _noisePhase;

		internal float ComputeEffectivePatchLength(float normalLoad)
		{
			var pressurePascals = MathF.Max(TyrePressure, 50f) * KilopascalsToPascals;
			var effectiveWidth = MathF.Max(Width, 0.05f);
			var patchLengthScale = MathF.Max(ContactPatchLengthScale, 0.1f);
			var theoreticalLength = normalLoad / MathF.Max(pressurePascals * effectiveWidth, 1f);
			return MathF.Max(theoreticalLength * patchLengthScale, 0.01f);
		}

		internal float ComputeEffectivePatchArea(float normalLoad)
		{
			var effectiveWidth = MathF.Max(Width, 0.05f);
			return ComputeEffectivePatchLength(normalLoad) * effectiveWidth;
		}

		internal float ComputeReferencePatchArea()
		{
			var referenceLoad = MathF.Max(ReferenceLoad, 1f);
			var referencePressurePascals = ReferencePressure * KilopascalsToPascals;
			var theoreticalLength = referenceLoad / MathF.Max(referencePressurePascals * ReferenceTyreWidth, 1f);
			return MathF.Max(theoreticalLength * MathF.Max(ContactPatchLengthScale, 0.1f) * ReferenceTyreWidth, 1e-6f);
		}

		internal TyreMagicFormulaCoefficients EvaluateLongitudinalPureSlipCoefficients(
			float peakForce,
			float loadRatio,
			in SurfaceProperties surface)
		{
			var stiffnessScale = ComputeSurfaceSlipStiffnessScale(surface) / ComputeSurfacePeakSlipRatioScale(surface);
			return _longitudinalPureSlipSet.Evaluate(
				peakForce,
				loadRatio,
				camberAngle: 0f,
				surface.DeformationFactor,
				stiffnessScale: stiffnessScale);
		}

		internal TyreMagicFormulaCoefficients EvaluateLateralPureSlipCoefficients(
			float peakForce,
			float loadRatio,
			float camberAngle,
			in SurfaceProperties surface)
		{
			var pressureStiffnessFactor = MathF.Sqrt(MathF.Max(TyrePressure, 50f) / ReferencePressure);
			var stiffnessScale = SidewallStiffness
			                     * pressureStiffnessFactor
			                     * ComputeSurfaceSlipStiffnessScale(surface);
			return _lateralPureSlipSet.Evaluate(
				peakForce,
				loadRatio,
				camberAngle,
				surface.DeformationFactor,
				stiffnessScale: stiffnessScale);
		}

		internal float ComputeEffectiveRelaxationLength(in SurfaceProperties surface, bool longitudinal)
		{
			return ComputeEffectiveRelaxationLength(surface, longitudinal, 1f);
		}

		internal float ComputeEffectiveRelaxationLength(
			in SurfaceProperties surface,
			bool longitudinal,
			float operatingPointSlopeScale)
		{
			var baseLength = longitudinal ? LongitudinalRelaxationLength : RelaxationLength;
			var sensitivity = longitudinal
				? LongitudinalSurfaceRelaxationSensitivity
				: LateralSurfaceRelaxationSensitivity;
			var baseSurfaceScale = Math.Clamp(surface.RelaxationLengthScale, 0.5f, 3.0f);
			var deformationFactor = 1f + Math.Clamp(surface.DeformationFactor, 0f, 1f) * MathF.Max(sensitivity, 0f);
			// Low-grip surfaces delay carcass force buildup beyond the deformation term alone.
			var lowGripFactor = 1f + MathF.Max(1f - Math.Clamp(surface.FrictionCoefficient, 0.1f, 1.25f), 0f)
				* (SurfaceRelaxationLowGripBase + MathF.Max(sensitivity, 0f) * SurfaceRelaxationLowGripSensitivityScale);
			// Rough aggregate adds extra tread-block shuffle, so relaxation grows slightly with surface noise.
			var roughnessFactor = 1f + Math.Clamp(surface.NoiseFactor, 0f, 1f)
				* (SurfaceRelaxationRoughnessBase + MathF.Max(sensitivity, 0f) * SurfaceRelaxationRoughnessSensitivityScale);
			var slopeScale = Math.Clamp(operatingPointSlopeScale, RelaxationSlopeMinFactor, RelaxationSlopeMaxFactor);
			var surfaceAdjustedLength = baseLength * baseSurfaceScale * deformationFactor * lowGripFactor * roughnessFactor * slopeScale;
			return MathF.Max(surfaceAdjustedLength / MathF.Max(CarcassStiffness, 0.1f), 0.05f);
		}

		internal static float ComputeSurfaceSlipStiffnessScale(in SurfaceProperties surface)
		{
			return Math.Clamp(surface.SlipStiffnessScale, 0.15f, 1.5f);
		}

		internal static float ComputeSurfacePeakSlipRatioScale(in SurfaceProperties surface)
		{
			return Math.Clamp(surface.PeakSlipRatioScale, 0.75f, 4.0f);
		}

		internal float ComputeRelaxationLengthOperatingPointScale(
			bool longitudinal,
			float slipRatio,
			float slipAngle,
			in TyreMagicFormulaCoefficients coefficients,
			in TyreMagicFormulaCoefficients referenceCoefficients)
		{
			var currentSlope = longitudinal
				? ComputeLongitudinalForceSlope(slipRatio, coefficients.B, coefficients.C, coefficients.D, coefficients.E)
				: ComputeLateralForceSlopePerTanAlpha(slipAngle, coefficients.B, coefficients.C, coefficients.D, coefficients.E);
			var referenceSlope = longitudinal
				? ComputeLongitudinalForceSlope(0f, referenceCoefficients.B, referenceCoefficients.C, referenceCoefficients.D, referenceCoefficients.E)
				: ComputeLateralForceSlopePerTanAlpha(0f, referenceCoefficients.B, referenceCoefficients.C, referenceCoefficients.D, referenceCoefficients.E);
			if (MathF.Abs(referenceSlope) <= 1e-4f)
			{
				return 1f;
			}

			return Math.Clamp(currentSlope / referenceSlope, RelaxationSlopeMinFactor, RelaxationSlopeMaxFactor);
		}

		internal static float ComputeEquivalentTrailSlip(float tanSlipAngle, float slipRatio, float stiffnessRatio)
		{
			if (MathF.Abs(tanSlipAngle) <= 1e-6f)
			{
				return 0f;
			}

			var scaledSlipRatio = MathF.Abs(MathF.Max(stiffnessRatio, 0f) * slipRatio);
			var magnitude = MathF.Sqrt(tanSlipAngle * tanSlipAngle + scaledSlipRatio * scaledSlipRatio);
			return MathF.Sign(tanSlipAngle) * magnitude;
		}

		internal float ComputeResidualAligningTorque(float equivalentTrailSlip, float peakForce, float adhesionFraction)
		{
			if (MathF.Abs(equivalentTrailSlip) <= 1e-6f || peakForce <= 1e-3f)
			{
				return 0f;
			}

			var buildFactor = Math.Clamp((1f - Math.Clamp(adhesionFraction, 0f, 1f))
			                             * (0.5f + MathF.Abs(equivalentTrailSlip)), 0f, 1f);
			var lever = MathF.Max(PneumaticTrail, 0.001f) * MathF.Max(AligningTorqueResidualFactor, 0f);
			return -MathF.Sign(equivalentTrailSlip) * peakForce * lever * buildFactor;
		}

		internal float ComputeFxAligningMomentArm(float lateralForce, float peakForce, float camberAngle)
		{
			if (MathF.Abs(lateralForce) <= 1f || peakForce <= 1e-3f)
			{
				return 0f;
			}

			var lateralLoadFactor = Math.Clamp(MathF.Abs(lateralForce) / peakForce, 0f, 1f);
			var baseMomentArm = MathF.Max(AligningTorqueFxMomentArm, 0f) + MathF.Abs(WheelOffset ?? 0f) * 0.5f;
			var camberFactor = 1f + Math.Clamp(MathF.Abs(camberAngle), 0f, 0.5f) * 0.5f;
			return MathF.Sign(lateralForce) * baseMomentArm * lateralLoadFactor * camberFactor;
		}

		internal float ComputeOverturningCouple(
			float normalLoad,
			float lateralForce,
			float peakLateralForce,
			float camberAngle,
			float effectiveRollingRadius)
		{
			if (normalLoad <= 1e-3f || peakLateralForce <= 1e-3f)
			{
				return 0f;
			}

			var sectionHalfWidth = ComputeEffectiveSectionHalfWidth();
			var radiusRatio = Math.Clamp(effectiveRollingRadius / MathF.Max(Radius, 0.1f), 0.75f, 1.05f);
			var lateralArm = sectionHalfWidth * MathF.Max(OverturningCoupleFactor, 0f) * (0.6f + 0.4f * radiusRatio);
			var camberArm = sectionHalfWidth * MathF.Max(OverturningCamberFactor, 0f) * radiusRatio;
			var lateralUtilization = Math.Clamp(MathF.Abs(lateralForce) / peakLateralForce, 0f, 1f);
			var lateralMoment = lateralForce * lateralArm * (0.75f + 0.25f * lateralUtilization);
			var camberMoment = normalLoad * camberArm * Math.Clamp(camberAngle, -0.5f, 0.5f);
			return lateralMoment + camberMoment;
		}

		internal float ComputeRollingResistanceMoment(
			float effectiveRollingRadius,
			float rollingResistanceForce,
			float carcassShearForce,
			float rollingSpeed,
			float longitudinalTyreForce,
			float peakForce,
			float signedRollingDirection)
		{
			if (effectiveRollingRadius <= 1e-3f)
			{
				return 0f;
			}

			var baseResistiveForce = MathF.Abs(rollingResistanceForce + carcassShearForce);
			var baseMoment = baseResistiveForce * effectiveRollingRadius;
			var standingWaveExcess = MathF.Max(ComputeStandingWaveResistanceFactor(rollingSpeed) - 1f, 0f);
			var standingWaveMoment = baseMoment * standingWaveExcess;
			var fxCoupling = peakForce > 1e-3f
				? Math.Abs(longitudinalTyreForce / peakForce) * MathF.Max(RollingResistanceMomentFxFactor, 0f)
				: 0f;
			var slipCouplingMoment = MathF.Abs(longitudinalTyreForce) * effectiveRollingRadius * fxCoupling * 0.1f;
			var magnitude = (baseMoment + standingWaveMoment) * MathF.Max(RollingResistanceMomentFactor, 0f)
			                + slipCouplingMoment;
			var direction = MathF.Abs(signedRollingDirection) > 1e-4f ? MathF.Sign(signedRollingDirection) : 0f;
			return -direction * magnitude;
		}

		internal float ComputeEffectiveSectionHalfWidth()
		{
			var geometryWidth = HubWidth is { } hubWidth && hubWidth > 0f
				? MathF.Max(hubWidth, Width)
				: Width;
			return MathF.Max(geometryWidth * 0.5f, 0.05f);
		}

		private void ClampToFrictionEllipse(ref float longitudinalForce, ref float lateralForce, float peakForce)
		{
			const float forceEpsilon = 1e-6f;
			var fxMax = MathF.Max(peakForce, forceEpsilon);
			var fyMax = MathF.Max(peakForce * MathF.Max(FrictionEllipseRatio, 0.1f), forceEpsilon);
			var ellipseValue = (longitudinalForce * longitudinalForce) / (fxMax * fxMax)
			                   + (lateralForce * lateralForce) / (fyMax * fyMax);
			if (ellipseValue > 1f)
			{
				var scale = 1f / MathF.Sqrt(ellipseValue);
				longitudinalForce *= scale;
				lateralForce *= scale;
			}
		}

		/// <summary>
		/// Applies MF-style combined-slip weighting using explicit slip inputs rather than force utilization.
		/// The current force channels remain the existing hybrid brush/Pacejka blend, but the cross-slip
		/// reduction now follows <c>Gxα</c>/<c>Gyκ</c>-like cosine weights instead of the old power-law penalty.
		/// </summary>
		private void ApplyCombinedSlipInteraction(
			ref float longitudinalForce,
			ref float lateralForce,
			float slipRatio,
			float slipAngle)
		{
			longitudinalForce *= ComputeCombinedSlipLongitudinalWeight(slipAngle, slipRatio);
			lateralForce *= ComputeCombinedSlipLateralWeight(slipRatio, slipAngle);
		}

		internal float ComputeCombinedSlipLongitudinalWeight(float slipAngle, float slipRatio)
		{
			var lateralSlip = MathF.Abs(MathF.Tan(slipAngle));
			var stiffness = ComputeCombinedSlipStiffness(MathF.Abs(slipRatio));
			return ComputeCombinedSlipWeight(lateralSlip, stiffness, CombinedSlipExponent, ComputeCombinedSlipCurvature());
		}

		internal float ComputeCombinedSlipLateralWeight(float slipRatio, float slipAngle)
		{
			var longitudinalSlip = MathF.Abs(slipRatio);
			var stiffness = ComputeCombinedSlipStiffness(MathF.Abs(MathF.Tan(slipAngle)));
			return ComputeCombinedSlipWeight(longitudinalSlip, stiffness, CombinedSlipExponent, ComputeCombinedSlipCurvature());
		}

		private float ComputeCombinedSlipStiffness(float coupledSlipMagnitude)
		{
			var coupling = MathF.Max(CombinedSlipCoupling, 0f);
			if (coupling <= 1e-6f)
			{
				return 0f;
			}

			// Pacejka's combined-slip weighting coefficients soften as the paired slip grows.
			// We mirror that behavior with a compact cosine/atan scaling rather than a fixed penalty.
			var coupledSlipScale = MathF.Cos(MathF.Atan(coupledSlipMagnitude));
			return MathF.Max(coupling * coupledSlipScale, 0f);
		}

		private float ComputeCombinedSlipCurvature()
		{
			return Math.Clamp(0.35f + 0.15f * MathF.Max(CombinedSlipExponent - 1f, 0f), 0f, 0.95f);
		}

		internal static float ComputeCombinedSlipWeight(
			float slipInput,
			float stiffnessFactor,
			float shapeFactor,
			float curvatureFactor,
			float horizontalShift = 0f)
		{
			var stiffness = MathF.Max(stiffnessFactor, 0f);
			if (stiffness <= 1e-6f)
			{
				return 1f;
			}

			var shape = MathF.Max(shapeFactor, 1f);
			var curvature = Math.Clamp(curvatureFactor, -0.99f, 0.99f);
			var shiftedSlip = MathF.Abs(slipInput) + horizontalShift;
			var rawWeight = MagicFormulaCosineWeight(shiftedSlip, stiffness, shape, curvature);
			var referenceWeight = MagicFormulaCosineWeight(horizontalShift, stiffness, shape, curvature);
			if (MathF.Abs(referenceWeight) <= 1e-6f)
			{
				return Math.Clamp(rawWeight, 0f, 1f);
			}

			return Math.Clamp(rawWeight / referenceWeight, 0f, 1f);
		}

		internal float ComputeVerticalStiffness()
		{
			var pressureRatio = MathF.Max(TyrePressure, 50f) / ReferencePressure;
			return MathF.Max(VerticalStiffness * pressureRatio, 10000f);
		}

		internal float ComputeEstimatedWheelInertia(float normalLoad)
		{
			var effectiveLoad = MathF.Max(normalLoad, 1f);
			var effectiveMass = effectiveLoad / 9.81f * 0.05f;
			var outerRadius = MathF.Max(Radius, 0.05f);
			var innerRadius = HubRadius is > 0f and var hubRadius
				? Math.Clamp(hubRadius, 0.03f, outerRadius * 0.98f)
				: outerRadius * 0.55f;
			var widthRatio = HubWidth is > 0f and var hubWidth
				? Math.Clamp(hubWidth / MathF.Max(Width, 0.05f), 0.6f, 1.5f)
				: 1f;
			var polarRadiusSquared = 0.5f * (outerRadius * outerRadius + innerRadius * innerRadius);
			return MathF.Max(MinimumRealisticWheelInertia, effectiveMass * polarRadiusSquared * widthRatio * InertiaScalar);
		}

		internal float ComputeEffectiveRollingRadius(float normalLoad)
		{
			var verticalStiffness = ComputeVerticalStiffness();
			var deflection = normalLoad / verticalStiffness;
			var maxDeflection = Radius * 0.35f;
			deflection = Math.Clamp(deflection, 0f, maxDeflection);

			var rollingRadius = Radius * (1f - deflection / MathF.Max(RollingRadiusDeflectionDivisor * Radius, 1e-4f));
			return Math.Clamp(rollingRadius, Radius * 0.6f, Radius);
		}

		internal float ComputeEffectiveBrushStiffness()
		{
			var pressurePascals = MathF.Max(TyrePressure, 50f) * KilopascalsToPascals;
			var effectiveWidth = MathF.Max(Width, 0.05f);
			var calibration = MathF.Max(ContactPatchStiffness, 1000f) / ReferenceContactPatchStiffness;
			var stiffnessCalibrationFactor = ReferenceContactPatchStiffness / ReferencePressureBrushStiffness;
			return MathF.Max((pressurePascals * effectiveWidth * calibration)
			                 * stiffnessCalibrationFactor, 1000f);
		}

		internal float ComputeStandingWaveResistanceFactor(float speed)
		{
			var criticalSpeed = MathF.Max(StandingWaveCriticalSpeed, 1f);
			var speedRatio = MathF.Abs(speed) / criticalSpeed;
			return 1f + MathF.Max(StandingWaveResistanceGain, 0f) * speedRatio * speedRatio;
		}

		/// <summary>
		/// Computes the adhesion fraction λ of the contact patch for a given slip input.
		/// λ = 1 means full adhesion (low slip); λ → 0 means nearly full sliding.
		/// <para>The slip threshold is <c>µP / (4a²c)</c>.</para>
		/// <para>When <c>|slip| ≤ threshold</c>, λ = 1 (pure adhesion).</para>
		/// <para>When <c>|slip| > threshold</c>, λ = threshold / |slip| (mixed).</para>
		/// Reference: The Contact Patch, C2015, Eq. 15.
		/// </summary>
		/// <param name="absSlip">Absolute value of the slip input (tan(α) for lateral, K for longitudinal).</param>
		/// <param name="peakForce">µ · Fz — the maximum friction force (N).</param>
		/// <param name="halfPatch">Contact patch half-length a (m).</param>
		/// <param name="brushCperLength">Brush stiffness per unit length of tread (N/m²).</param>
		internal static float ComputeAdhesionFraction(float absSlip, float peakForce,
			float halfPatch, float brushCperLength)
		{
			var threshold = peakForce / MathF.Max(4f * halfPatch * halfPatch * brushCperLength, 1f);
			if (absSlip <= threshold)
			{
				return 1f;
			}

			return Math.Clamp(threshold / MathF.Max(absSlip, 1e-6f), 0f, 1f);
		}

		/// <summary>
		/// Computes the pneumatic trail from the brush-model adhesion fraction λ.
		/// <para>Pure adhesion (λ = 1):  trail = a / 3.</para>
		/// <para>Mixed adhesion/slip (λ &lt; 1):  trail = a · λ · (1 − 2λ/3) / (2 · (1 − λ/2)).</para>
		/// Reference: The Contact Patch, C2015, Eq. 13, 20.
		/// </summary>
		/// <param name="halfPatch">Contact patch half-length a (m).</param>
		/// <param name="lambda">Adhesion fraction (0–1).</param>
		internal static float ComputeBrushPneumaticTrail(float halfPatch, float lambda)
		{
			lambda = Math.Clamp(lambda, 0f, 1f);
			if (lambda >= 1f)
			{
				return halfPatch / 3f;
			}

			var numerator = halfPatch * lambda * (1f - (2f / 3f) * lambda);
			var denominator = 2f * (1f - 0.5f * lambda);
			return numerator / MathF.Max(denominator, 0.01f);
		}

		/// <summary>
		/// Computes the free-rolling carcass shear resistance force magnitude.
		/// Arises from the difference between unloaded radius and effective rolling radius.
		/// Reference: The Contact Patch, C2015, §Free rolling, Eq. 7–8.
		/// </summary>
		/// <param name="radiusDelta">r − rₑ (m). Positive when tyre is deflected.</param>
		/// <param name="halfPatch">Contact patch half-length a (m).</param>
		/// <param name="brushCperLength">Brush stiffness per unit length of tread (N/m²).</param>
		/// <param name="patchLength">Full contact patch length 2a (m).</param>
		internal float ComputeCarcassShearForce(float radiusDelta, float halfPatch,
			float brushCperLength, float patchLength)
		{
			var halfAngle = MathF.Asin(Math.Clamp(halfPatch / MathF.Max(Radius, 0.1f), 0f, 1f));
			return CarcassShearCoefficient * brushCperLength * radiusDelta * halfAngle * patchLength;
		}

		/// <summary>
		/// Pacejka Magic Formula: F = D · sin(C · atan(B·x − E·(B·x − atan(B·x)))).
		/// Reference: Pacejka, "Tire and Vehicle Dynamics", Eq. 4.6.
		/// </summary>
		private static float MagicFormula(float x, float b, float c, float d, float e)
		{
			var bx = b * x;
			return d * MathF.Sin(c * MathF.Atan(bx - e * (bx - MathF.Atan(bx))));
		}

		internal static float ComputeLongitudinalForceSlope(float slipRatio, float b, float c, float d, float e)
		{
			return ComputeMagicFormulaSlope(slipRatio, b, c, d, e);
		}

		internal float ComputeLateralForceSlopePerTanAlpha(float slipAngle, float b, float c, float d, float e)
		{
			var tanAlpha = MathF.Tan(slipAngle);
			var forwardTan = tanAlpha + RelaxationDerivativeStep;
			var backwardTan = tanAlpha - RelaxationDerivativeStep;
			var forwardAngle = MathF.Atan(forwardTan);
			var backwardAngle = MathF.Atan(backwardTan);
			var forwardForce = MagicFormulaRally(forwardAngle, b, c, d, e);
			var backwardForce = MagicFormulaRally(backwardAngle, b, c, d, e);
			var denominator = forwardTan - backwardTan;
			if (MathF.Abs(denominator) <= 1e-6f)
			{
				return 0f;
			}

			return (forwardForce - backwardForce) / denominator;
		}

		internal static float ComputeMagicFormulaSlope(float x, float b, float c, float d, float e)
		{
			var bx = b * x;
			var atanBx = MathF.Atan(bx);
			var shapeInput = bx - e * (bx - atanBx);
			var shapeInputDerivative = b * ((1f - e) + e / (1f + bx * bx));
			var atanShape = MathF.Atan(shapeInput);
			var atanShapeDerivative = shapeInputDerivative / (1f + shapeInput * shapeInput);
			return d * MathF.Cos(c * atanShape) * c * atanShapeDerivative;
		}

		private static float MagicFormulaCosineWeight(float x, float b, float c, float e)
		{
			var bx = b * x;
			return MathF.Cos(c * MathF.Atan(bx - e * (bx - MathF.Atan(bx))));
		}

		/// <summary>
		/// Modified Magic Formula for lateral force with rally high-slip behaviour.
		///
		/// Standard Pacejka Fy drops off sharply past the peak slip angle, which causes
		/// snap oversteer. Rally tyres maintain ~60-70% of peak force at large slip angles
		/// (20-40°+), enabling controlled drifts.
		///
		/// This implementation blends the normal Pacejka curve with a sustained force floor
		/// at high slip angles, referenced from:
		/// Abdulrahim, "Measurement and Analysis of Rally Car Dynamics at High Attitude Angles".
		/// </summary>
		private float MagicFormulaRally(float x, float b, float c, float d, float e)
		{
			var pacejkaForce = MagicFormula(x, b, c, d, e);

			// The sustained force at high slip: HighSlipForceRetention × D × sign(α).
			// At low slip angles, the normal Pacejka curve dominates.
			// At high slip angles (past ~15-20°), we ensure force doesn't drop below the retention floor.
			var absX = MathF.Abs(x);
			var signX = x >= 0f ? 1f : -1f;
			var retentionFloor = HighSlipForceRetention * d * signX;

			// Transition: uses configurable HighSlipTransitionStart and HighSlipTransitionEnd.
			var transitionRange = MathF.Max(HighSlipTransitionEnd - HighSlipTransitionStart, 0.01f);
			var t = Math.Clamp((absX - HighSlipTransitionStart) / transitionRange, 0f, 1f);

			// Use the higher-magnitude force: normal Pacejka at low slip, retention floor at high slip.
			if (t <= 0f)
			{
				return pacejkaForce;
			}

			// Smooth blend — don't let force drop below retention floor past transition
			var absPacejka = MathF.Abs(pacejkaForce);
			var absFloor = MathF.Abs(retentionFloor);
			if (absPacejka >= absFloor)
			{
				return pacejkaForce;
			}

			return pacejkaForce + t * (retentionFloor - pacejkaForce);
		}
	}
}
