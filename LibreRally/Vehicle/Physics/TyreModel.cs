using System;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Mutable per-wheel thermal and wear state.
/// Updated every physics step by <see cref="TyreModel.Update"/>.
///
/// Temperature model reference: Salaani et al., "An Analytical Tire Model for
/// Use in Vehicle Dynamics Simulations", SAE 2007-01-0816.
/// Wear model: simplified abrasion proportional to slip energy dissipation.
/// </summary>
public struct TyreState
{
    /// <summary>Current tyre surface temperature (°C). Starts at ambient (~30 °C).</summary>
    public float Temperature;

    /// <summary>Remaining tread fraction (1.0 = new, 0.0 = fully worn).</summary>
    public float TreadLife;

    /// <summary>Wheel angular velocity (rad/s). Positive = forward rolling.</summary>
    public float AngularVelocity;

    /// <summary>Lateral deflection state for the brush contact-patch model (m).</summary>
    public float LateralDeflection;

    /// <summary>Creates a fresh tyre state with default ambient temperature and full tread.</summary>
    public static TyreState CreateDefault() => new()
    {
        Temperature = 30f,
        TreadLife = 1.0f,
        AngularVelocity = 0f,
        LateralDeflection = 0f,
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
    /// Tyre inflation pressure (kPa). Affects vertical stiffness, contact patch size,
    /// and brush stiffness.
    /// Higher pressure → stiffer tyre, smaller contact patch, less grip.
    /// Reference pressure for defaults: 220 kPa (~32 psi), typical rally tarmac tyre.
    /// <para>verticalStiffness = baseStiffness × (TyrePressure / referencePressure)</para>
    /// <para>contactPatchLength ≈ normalLoad / (pressure × width)</para>
    /// <para>brushStiffness ∝ pressure × width</para>
    /// </summary>
    public float TyrePressure { get; set; } = 220f;

    /// <summary>
    /// Sidewall stiffness multiplier (dimensionless, 1.0 = baseline).
    /// Controls how quickly slip angle builds and affects steering response.
    /// Higher stiffness → sharper lateral response, less compliant ride.
    /// Applied as a multiplier to the lateral cornering stiffness (LateralB).
    /// <para>corneringStiffness *= SidewallStiffness</para>
    /// </summary>
    public float SidewallStiffness { get; set; } = 1.0f;

    /// <summary>
    /// Carcass stiffness multiplier (dimensionless, 1.0 = baseline).
    /// Controls deformation of the tyre body and transient response.
    /// Higher carcass stiffness → shorter relaxation length → faster transient buildup.
    /// <para>effectiveRelaxationLength = RelaxationLength / CarcassStiffness</para>
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
    /// Baseline vertical stiffness of the tyre carcass (N/m) at <see cref="ReferencePressure"/>.
    /// Used to estimate loaded-radius deflection and therefore effective rolling radius.
    /// Higher values reduce loaded deflection and slip generated by carcass compression.
    /// </summary>
    public float VerticalStiffness { get; set; } = 200000f;

    /// <summary>
    /// Wheel rotational inertia (kg·m²). Governs spin-up/spin-down response.
    /// Typical values: 0.5–1.5 kg·m² for passenger/rally tyres.
    /// When set to 0 (default), inertia is estimated from normal load and radius.
    /// <para>angularAccel = appliedTorque / WheelInertia</para>
    /// Reference: Milliken, RCVD §2.3, wheel inertia.
    /// </summary>
    public float WheelInertia { get; set; } = 0f;

    // ── Pacejka Magic Formula coefficients ───────────────────────────────────
    // F = D * sin(C * atan(B*x - E*(B*x - atan(B*x))))
    // B = stiffness factor, C = shape factor, D = peak factor, E = curvature

    /// <summary>Longitudinal stiffness factor B (cornering stiffness / peak force).</summary>
    public float LongitudinalB { get; set; } = 12.0f;

    /// <summary>Longitudinal shape factor C. Typically 1.5–1.8 for combined slip models.</summary>
    public float LongitudinalC { get; set; } = 1.65f;

    /// <summary>Longitudinal curvature factor E. Negative values extend the curve past the peak.</summary>
    public float LongitudinalE { get; set; } = -0.5f;

    /// <summary>Lateral stiffness factor B.</summary>
    public float LateralB { get; set; } = 10.0f;

    /// <summary>Lateral shape factor C. Typically 1.1–1.4 for passenger/rally tyres.</summary>
    public float LateralC { get; set; } = 1.30f;

    /// <summary>Lateral curvature factor E.</summary>
    public float LateralE { get; set; } = -0.6f;

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

    /// <summary>Reference vertical load for µ rating (N).</summary>
    public float ReferenceLoad { get; set; } = 3000f;

    /// <summary>
    /// Grip gain from effective contact-patch area, applied as a small power-law multiplier.
    /// <para><c>µ_effective *= (patchArea / referencePatchArea)^ContactAreaGripExponent</c></para>
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

    /// <summary>Relaxation length (m) for the brush tyre transient model.</summary>
    public float RelaxationLength { get; set; } = 0.6f;

    // ── Camber ───────────────────────────────────────────────────────────────

    /// <summary>Camber thrust coefficient (N per rad of camber per N of Fz).</summary>
    public float CamberThrustCoefficient { get; set; } = 0.08f;

    // ── Self-aligning torque ─────────────────────────────────────────────────

    /// <summary>Pneumatic trail at zero slip (m). Mz = trail × Fy. Decreases with slip angle.</summary>
    public float PneumaticTrail { get; set; } = 0.025f;

    // ── Thermal / wear ───────────────────────────────────────────────────────

    /// <summary>Thermal mass of the tyre surface (J/°C). Higher = slower temp change.</summary>
    public float ThermalMass { get; set; } = 8000f;

    /// <summary>Cooling rate to ambient (W/°C).</summary>
    public float CoolingRate { get; set; } = 15f;

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

    // ── Constants ────────────────────────────────────────────────────────────

    private const float MinSpeed = 0.5f;       // velocity floor for slip calculations (m/s)
    private const float MaxSlipRatio = 1.5f;    // clamp slip ratio to prevent numerical blow-up
    private const float MaxSlipAngle = 1.2f;    // ~69° — beyond this we clamp
    /// <summary>
    /// Approximate wheel rotational inertia scalar.
    /// Wheel inertia ≈ effective_mass × R² × scalar.
    /// The 1.2 factor accounts for the distribution of tyre mass (annular ring rather than
    /// point mass). Typical road-tyre I ≈ 0.5–1.5 kg·m²; this scalar yields values in that range.
    /// Reference: Milliken, RCVD §2.3, approximate wheel inertia.
    /// </summary>
    private const float InertiaScalar = 1.2f;
    private const float ReferenceTyreWidth = 0.205f;
    private const float ReferenceContactPatchStiffness = 65000f;
    private const float KilopascalsToPascals = 1000f;
    private const float RollingRadiusDeflectionDivisor = 3f;
    private const float SurfaceDeformationBrushSoftening = 0.15f;
    private const float ReferencePressureBrushStiffness = ReferencePressure * KilopascalsToPascals * ReferenceTyreWidth;

    /// <summary>
    /// Reference tyre pressure (kPa) used to normalise pressure-dependent effects.
    /// 220 kPa ≈ 32 psi, a typical rally tarmac cold-start pressure.
    /// </summary>
    private const float ReferencePressure = 220f;

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
        longitudinalForce = 0f;
        lateralForce = 0f;
        selfAligningTorque = 0f;

        if (dt < 1e-6f)
        {
            state.LateralDeflection = 0f;
            return;
        }

        // When airborne (normalLoad ≤ 0), skip force computation but still integrate
        // angular velocity from drive/brake torque so wheels spin up in the air.
        // This is important for AWD diffs and realistic landing behaviour.
        bool airborne = normalLoad <= 0f;
        if (airborne)
        {
            state.LateralDeflection = 0f;

            // Integrate angular velocity from torque even without ground contact
            float airInertia = WheelInertia > 0f
                ? WheelInertia
                : MathF.Max(0.5f, Radius * Radius * InertiaScalar);
            float airBrakeDir = state.AngularVelocity > 0f ? -1f : (state.AngularVelocity < 0f ? 1f : 0f);
            float airNetTorque = driveTorque + brakeTorque * airBrakeDir;
            state.AngularVelocity += (airNetTorque / airInertia) * dt;
            return;
        }

        // ── Effective friction coefficient ───────────────────────────────────
        // Combines surface µ, load sensitivity, temperature and wear effects.
        float mu = ComputeEffectiveFriction(normalLoad, surface, state.Temperature, state.TreadLife);

        float effectiveRollingRadius = ComputeEffectiveRollingRadius(normalLoad);

        // ── Slip ratio (longitudinal) ────────────────────────────────────────
        // κ = (ω·R − Vx) / max(|Vx|, ε)
        // Positive κ means wheel spinning faster than ground (acceleration).
        // Reference: Pacejka, §2.2, Eq. 2.5.
        float wheelLinearSpeed = state.AngularVelocity * effectiveRollingRadius;
        float absVx = MathF.Abs(longitudinalVelocity);
        float denominator = MathF.Max(absVx, MinSpeed);
        float slipRatio = (wheelLinearSpeed - longitudinalVelocity) / denominator;
        slipRatio = Math.Clamp(slipRatio, -MaxSlipRatio, MaxSlipRatio);

        // ── Slip angle (lateral) ─────────────────────────────────────────────
        // α = atan(-Vy / |Vx|)
        // Positive contact-patch lateral velocity means the wheel is moving to the right
        // relative to its heading, so the tyre reaction force must act to the left.
        // Reference: Pacejka, §1.3, Eq. 1.4.
        float slipAngle = MathF.Atan2(-lateralVelocity, MathF.Max(absVx, MinSpeed));
        slipAngle = Math.Clamp(slipAngle, -MaxSlipAngle, MaxSlipAngle);

        // ── Pacejka Magic Formula forces ─────────────────────────────────────
        // F = D · sin(C · atan(B·x − E·(B·x − atan(B·x))))
        // D = µ · Fz (peak force proportional to load)
        float peakForce = mu * normalLoad;

        // Longitudinal force Fx(κ)
        // Deformable surfaces tolerate more slip before saturation — widen the curve.
        float effectiveLongB = LongitudinalB * (1f - surface.DeformationFactor * 0.3f);
        float rawFx = MagicFormula(slipRatio, effectiveLongB, LongitudinalC, peakForce, LongitudinalE);

        // Lateral force Fy(α) — with rally high-slip extension.
        // SidewallStiffness scales the lateral cornering stiffness B:
        //   corneringStiffness *= SidewallStiffness
        // Higher sidewall stiffness → sharper response.
        float pressureStiffnessFactor = MathF.Sqrt(MathF.Max(TyrePressure, 50f) / ReferencePressure);
        float effectiveLatB = LateralB * SidewallStiffness * pressureStiffnessFactor
            * (1f - surface.DeformationFactor * 0.2f);
        float rawFy = MagicFormulaRally(slipAngle, effectiveLatB, LateralC, peakForce, LateralE);

        // ── Contact-patch brush model (lateral transient) ────────────────────
        // The brush model smooths lateral force buildup through a deflection state.
        // dδ/dt = Vy − δ · |Vx| / L     (L = relaxation length)
        // CarcassStiffness modifies relaxation length: stiffer carcass → shorter relaxation.
        //   effectiveRelaxationLength = RelaxationLength / CarcassStiffness
        // Reference: Pacejka §5.4, Eq. 5.31 (relaxation length model).
        float effectiveRelaxation = RelaxationLength / MathF.Max(CarcassStiffness, 0.1f);
        float relaxSpeed = MathF.Max(absVx, 2f);
        float deflectionRate = lateralVelocity -
            (state.LateralDeflection * relaxSpeed / MathF.Max(effectiveRelaxation, 0.05f));
        state.LateralDeflection += deflectionRate * dt;

        // Clamp deflection to contact-patch limit (brush saturation).
        // For a roughly rectangular footprint, patch length ≈ Fz / (P × width).
        // Lower pressure or higher load produces a longer patch and more available deflection.
        float effectivePatchLength = ComputeEffectivePatchLength(normalLoad);
        float maxDeflection = MathF.Max(effectivePatchLength * 0.5f, 0.01f);
        state.LateralDeflection = Math.Clamp(state.LateralDeflection, -maxDeflection, maxDeflection);

        // Blend brush-model transient force with steady-state Pacejka.
        // Treat the inflated carcass as a pressure membrane: lateral brush stiffness scales with
        // inflation pressure × tread width, with ContactPatchStiffness acting as a calibration factor.
        float effectiveBrushStiffness = ComputeEffectiveBrushStiffness()
            * (1f - surface.DeformationFactor * SurfaceDeformationBrushSoftening);
        float brushForce = -effectiveBrushStiffness * state.LateralDeflection;
        float blendAlpha = Math.Clamp(absVx / 5f, 0f, 1f); // transition from brush to Pacejka
        float blendedFy = blendAlpha * rawFy + (1f - blendAlpha) * brushForce;

        // ── Camber thrust ────────────────────────────────────────────────────
        // Small lateral force from wheel inclination. Fy_camber = γ · Cγ · Fz.
        // Reference: Pacejka §4.3.
        float camberForce = camberAngle * CamberThrustCoefficient * normalLoad;
        float totalFy = blendedFy + camberForce;

        // ── Rolling resistance ───────────────────────────────────────────────
        // Frr = Crr · Fz, opposing wheel direction of travel.
        // Use a small deadband near zero longitudinal speed so rolling resistance
        // does not inject a non-zero force at rest due to floating-point noise.
        // Reference: Milliken, "Race Car Vehicle Dynamics", §2.2.
        float rollingReferenceSpeed = MathF.Max(MathF.Abs(longitudinalVelocity), MathF.Abs(wheelLinearSpeed));
        float rollingResistance = (RollingResistanceCoefficient + surface.RollingResistance)
            * ComputeStandingWaveResistanceFactor(rollingReferenceSpeed)
            * normalLoad;
        const float rollingResistanceDeadband = 0.01f;
        float rrForce = 0f;
        if (MathF.Abs(longitudinalVelocity) > rollingResistanceDeadband)
        {
            rrForce = longitudinalVelocity > 0f ? -rollingResistance : rollingResistance;
        }

        longitudinalForce = rawFx + rrForce;
        lateralForce = totalFy;

        // Clamp the final applied force vector to a friction ellipse:
        // (Fx/Fx_max)^2 + (Fy/Fy_max)^2 ≤ 1
        ClampToFrictionEllipse(ref longitudinalForce, ref lateralForce, peakForce);

        // ── Self-aligning torque ─────────────────────────────────────────────
        // Mz = t_p · Fy, where t_p decreases with slip angle magnitude.
        // At high slip angles the pneumatic trail collapses to near zero.
        // Reference: Pacejka §4.3.3.
        float absAlpha = MathF.Abs(slipAngle);
        float trailFactor = MathF.Max(0f, 1f - absAlpha / (MathF.PI * 0.25f));
        selfAligningTorque = PneumaticTrail * trailFactor * lateralForce;

        // ── Wheel angular velocity integration ───────────────────────────────
        // Iω̇ = T_drive − T_brake − Fx·R
        // Use the same net longitudinal force that is applied to the chassis so
        // rolling resistance produces a matching resistive wheel torque.
        // Use explicit WheelInertia if set (> 0), otherwise estimate from load and geometry.
        // Reference: basic rotational dynamics, F = ma analogy for rotation.
        float wheelInertia = WheelInertia > 0f
            ? WheelInertia
            : MathF.Max(0.5f, (normalLoad / 9.81f * 0.05f) * Radius * Radius * InertiaScalar);
        float brakeDir = state.AngularVelocity > 0f ? -1f : (state.AngularVelocity < 0f ? 1f : 0f);
        float netTorque = driveTorque + brakeTorque * brakeDir - longitudinalForce * effectiveRollingRadius;
        state.AngularVelocity += (netTorque / wheelInertia) * dt;

        // ── Thermal model ────────────────────────────────────────────────────
        // Temperature rises with slip energy dissipation and cools toward ambient.
        // Q_in = |F_slip| · |V_slip| · dt   (slip power → heat)
        // Q_out = coolingRate · (T − T_ambient) · dt
        // Reference: Salaani, "Analytical Tire Model", SAE 2007-01-0816.
        float slipSpeed = MathF.Sqrt(
            (wheelLinearSpeed - longitudinalVelocity) * (wheelLinearSpeed - longitudinalVelocity)
            + lateralVelocity * lateralVelocity);
        float slipForceMag = MathF.Sqrt(longitudinalForce * longitudinalForce + lateralForce * lateralForce);
        float heatInput = slipForceMag * slipSpeed * dt;
        float heatLoss = CoolingRate * (state.Temperature - AmbientTemperature) * dt;
        state.Temperature += (heatInput - heatLoss) / MathF.Max(ThermalMass, 1f);
        state.Temperature = MathF.Max(state.Temperature, AmbientTemperature - 10f);

        // ── Wear model ───────────────────────────────────────────────────────
        // Tread life decreases proportionally to slip energy.
        // Reference: simplified abrasion model from RCVD §2.8.
        float wearEnergy = slipForceMag * slipSpeed * dt;
        state.TreadLife -= wearEnergy * WearRate;
        state.TreadLife = MathF.Max(state.TreadLife, 0f);
    }

    /// <summary>
    /// Computes effective friction coefficient accounting for surface, load sensitivity,
    /// temperature window, and tread wear.
    /// </summary>
    internal float ComputeEffectiveFriction(float normalLoad, in SurfaceProperties surface,
        float temperature, float treadLife)
    {
        // Base µ from surface type
        float mu = PeakFrictionCoefficient * surface.FrictionCoefficient;

        // Load sensitivity: rubber grip follows a non-linear power-law with load.
        float referenceLoad = MathF.Max(ReferenceLoad, 1f);
        float loadRatio = MathF.Max(normalLoad / referenceLoad, 1e-3f);
        mu *= MathF.Pow(loadRatio, -LoadSensitivity);

        // Effective patch area contributes a small secondary grip change.
        float areaGripExponent = MathF.Max(ContactAreaGripExponent, 0f);
        if (areaGripExponent > 0f)
        {
            float effectivePatchArea = ComputeEffectivePatchArea(normalLoad);
            float referencePatchArea = ComputeReferencePatchArea();
            float patchAreaRatio = effectivePatchArea / MathF.Max(referencePatchArea, 1e-6f);
            mu *= MathF.Pow(MathF.Max(patchAreaRatio, 1e-3f), areaGripExponent);
        }

        // Temperature effect: peak grip at optimal temperature, falls off outside window.
        // Reference: Salaani, SAE 2007-01-0816, Fig. 7.
        float tempDelta = MathF.Abs(temperature - OptimalTemperature);
        float tempWindow = MathF.Max(TemperatureWindow, 1f);
        float tempFactor = MathF.Max(0.7f, 1f - 0.3f * (tempDelta / tempWindow));
        mu *= tempFactor;

        // Wear effect: worn tyre has reduced grip.
        float wearFactor = WornGripFraction + (1f - WornGripFraction) * Math.Clamp(treadLife, 0f, 1f);
        mu *= wearFactor;

        return MathF.Max(mu, 0.05f);
    }

    internal float ComputeEffectivePatchLength(float normalLoad)
    {
        float pressurePascals = MathF.Max(TyrePressure, 50f) * KilopascalsToPascals;
        float effectiveWidth = MathF.Max(Width, 0.05f);
        float patchLengthScale = MathF.Max(ContactPatchLengthScale, 0.1f);
        float theoreticalLength = normalLoad / MathF.Max(pressurePascals * effectiveWidth, 1f);
        return MathF.Max(theoreticalLength * patchLengthScale, 0.01f);
    }

    internal float ComputeEffectivePatchArea(float normalLoad)
    {
        float effectiveWidth = MathF.Max(Width, 0.05f);
        return ComputeEffectivePatchLength(normalLoad) * effectiveWidth;
    }

    internal float ComputeReferencePatchArea()
    {
        float referenceLoad = MathF.Max(ReferenceLoad, 1f);
        float referencePressurePascals = ReferencePressure * KilopascalsToPascals;
        float theoreticalLength = referenceLoad / MathF.Max(referencePressurePascals * ReferenceTyreWidth, 1f);
        return MathF.Max(theoreticalLength * MathF.Max(ContactPatchLengthScale, 0.1f) * ReferenceTyreWidth, 1e-6f);
    }

    private void ClampToFrictionEllipse(ref float longitudinalForce, ref float lateralForce, float peakForce)
    {
        const float forceEpsilon = 1e-6f;
        float fxMax = MathF.Max(peakForce, forceEpsilon);
        float fyMax = MathF.Max(peakForce * MathF.Max(FrictionEllipseRatio, 0.1f), forceEpsilon);
        float ellipseValue = (longitudinalForce * longitudinalForce) / (fxMax * fxMax)
            + (lateralForce * lateralForce) / (fyMax * fyMax);
        if (ellipseValue > 1f)
        {
            float scale = 1f / MathF.Sqrt(ellipseValue);
            longitudinalForce *= scale;
            lateralForce *= scale;
        }
    }

    internal float ComputeVerticalStiffness()
    {
        float pressureRatio = MathF.Max(TyrePressure, 50f) / ReferencePressure;
        return MathF.Max(VerticalStiffness * pressureRatio, 10000f);
    }

    internal float ComputeEffectiveRollingRadius(float normalLoad)
    {
        float verticalStiffness = ComputeVerticalStiffness();
        float deflection = normalLoad / verticalStiffness;
        float maxDeflection = Radius * 0.35f;
        deflection = Math.Clamp(deflection, 0f, maxDeflection);

        float rollingRadius = Radius * (1f - deflection / MathF.Max(RollingRadiusDeflectionDivisor * Radius, 1e-4f));
        return Math.Clamp(rollingRadius, Radius * 0.6f, Radius);
    }

    internal float ComputeEffectiveBrushStiffness()
    {
        float pressurePascals = MathF.Max(TyrePressure, 50f) * KilopascalsToPascals;
        float effectiveWidth = MathF.Max(Width, 0.05f);
        float calibration = MathF.Max(ContactPatchStiffness, 1000f) / ReferenceContactPatchStiffness;
        float stiffnessCalibrationFactor = ReferenceContactPatchStiffness / ReferencePressureBrushStiffness;
        return MathF.Max((pressurePascals * effectiveWidth * calibration)
            * stiffnessCalibrationFactor, 1000f);
    }

    internal float ComputeStandingWaveResistanceFactor(float speed)
    {
        float criticalSpeed = MathF.Max(StandingWaveCriticalSpeed, 1f);
        float speedRatio = MathF.Abs(speed) / criticalSpeed;
        return 1f + MathF.Max(StandingWaveResistanceGain, 0f) * speedRatio * speedRatio;
    }

    /// <summary>
    /// Pacejka Magic Formula: F = D · sin(C · atan(B·x − E·(B·x − atan(B·x)))).
    /// Reference: Pacejka, "Tire and Vehicle Dynamics", Eq. 4.6.
    /// </summary>
    private static float MagicFormula(float x, float b, float c, float d, float e)
    {
        float bx = b * x;
        return d * MathF.Sin(c * MathF.Atan(bx - e * (bx - MathF.Atan(bx))));
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
        float pacejkaForce = MagicFormula(x, b, c, d, e);

        // The sustained force at high slip: HighSlipForceRetention × D × sign(α).
        // At low slip angles, the normal Pacejka curve dominates.
        // At high slip angles (past ~15-20°), we ensure force doesn't drop below the retention floor.
        float absX = MathF.Abs(x);
        float signX = x >= 0f ? 1f : -1f;
        float retentionFloor = HighSlipForceRetention * d * signX;

        // Transition: uses configurable HighSlipTransitionStart and HighSlipTransitionEnd.
        float transitionRange = MathF.Max(HighSlipTransitionEnd - HighSlipTransitionStart, 0.01f);
        float t = Math.Clamp((absX - HighSlipTransitionStart) / transitionRange, 0f, 1f);

        // Use the higher-magnitude force: normal Pacejka at low slip, retention floor at high slip.
        if (t <= 0f)
            return pacejkaForce;

        // Smooth blend — don't let force drop below retention floor past transition
        float absPacejka = MathF.Abs(pacejkaForce);
        float absFloor = MathF.Abs(retentionFloor);
        if (absPacejka >= absFloor)
            return pacejkaForce;

        return pacejkaForce + t * (retentionFloor - pacejkaForce);
    }
}
