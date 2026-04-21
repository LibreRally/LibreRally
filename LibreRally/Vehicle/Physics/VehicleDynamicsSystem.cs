using System;
using Stride.BepuPhysics;
using Stride.Core.Mathematics;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Central vehicle dynamics coordinator.
///
/// Computes all vehicle forces each frame and applies them as impulses to BEPU bodies.
/// BEPU remains responsible for rigid-body integration and collision; this system
/// calculates tyre forces, load transfer, drivetrain torque, suspension reactions,
/// and anti-roll bar effects externally.
///
/// <para>Per-frame update order:
/// <list type="number">
///   <item>Compute longitudinal and lateral load transfer from force-derived chassis acceleration.</item>
///   <item>Compute anti-roll bar forces from suspension compression difference.</item>
///   <item>Apply chassis body roll torque from suspension compression difference.</item>
///   <item>Split engine torque through the drivetrain (center diff → axle diffs → wheels).</item>
///   <item>Evaluate tyre model for each wheel (slip ratio, slip angle → Fx, Fy, Mz).</item>
///   <item>Estimate next-step chassis acceleration from the summed tyre forces.</item>
///   <item>Apply all computed forces/impulses to BEPU bodies.</item>
/// </list></para>
///
/// <para>Designed for 50+ simultaneous vehicles: uses fixed-size arrays, no per-frame
/// allocations, struct-based per-wheel state.</para>
///
/// <para>Key references:
/// <list type="bullet">
///   <item>Pacejka, "Tire and Vehicle Dynamics", 3rd Ed.</item>
///   <item>Milliken &amp; Milliken, "Race Car Vehicle Dynamics".</item>
///   <item>Abdulrahim, "Measurement and Analysis of Rally Car Dynamics at High Attitude Angles".</item>
/// </list></para>
/// </summary>
public sealed class VehicleDynamicsSystem
{
    private const float MinimumWakeForce = 1f;

    // Wheel indices — fixed order matching VehiclePhysicsBuilder output.
    /// <summary>Front-left wheel index.</summary>
    public const int FL = 0;
    /// <summary>Front-right wheel index.</summary>
    public const int FR = 1;
    /// <summary>Rear-left wheel index.</summary>
    public const int RL = 2;
    /// <summary>Rear-right wheel index.</summary>
    public const int RR = 3;
    /// <summary>Total number of wheels modeled by the system.</summary>
    public const int WheelCount = 4;

    // ── Vehicle-level configuration ──────────────────────────────────────────

    /// <summary>Total vehicle mass including chassis + wheels (kg).</summary>
    public float VehicleMass { get; set; } = 1200f;

    /// <summary>Centre-of-gravity height above ground (m). Affects load transfer magnitude.</summary>
    public float CgHeight { get; set; } = 0.45f;

    /// <summary>Front axle roll-center height above ground (m). Reduces the sprung-mass roll moment arm.</summary>
    public float FrontRollCenterHeight { get; set; } = 0.14f;

    /// <summary>Rear axle roll-center height above ground (m). Reduces the sprung-mass roll moment arm.</summary>
    public float RearRollCenterHeight { get; set; } = 0.18f;

    /// <summary>Front-to-rear axle distance (m).</summary>
    public float Wheelbase { get; set; } = 2.55f;

    /// <summary>Left-to-right wheel centre distance (m). Used for lateral load transfer.</summary>
    public float TrackWidth { get; set; } = 1.50f;

    /// <summary>Anti-dive geometry fraction in the range [0, 1]. Higher values reduce braking pitch/load transfer.</summary>
    public float AntiDiveFactor { get; set; } = 0.12f;

    /// <summary>Anti-squat geometry fraction in the range [0, 1]. Higher values reduce acceleration squat/load transfer.</summary>
    public float AntiSquatFactor { get; set; } = 0.18f;

    /// <summary>Front anti-roll bar stiffness (N/m of compression difference). 0 = disabled.</summary>
    public float FrontAntiRollStiffness { get; set; } = 8000f;

    /// <summary>Rear anti-roll bar stiffness (N/m of compression difference). 0 = disabled.</summary>
    public float RearAntiRollStiffness { get; set; } = 5000f;

    /// <summary>
    /// Chassis body roll system. Generates roll torque from left/right suspension
    /// compression difference and applies it as an angular impulse to the chassis.
    /// </summary>
    public ChassisBodyRollSystem BodyRoll;

    /// <summary>Front axle differential configuration.</summary>
    public DifferentialConfig FrontDiff { get; set; } = DifferentialConfig.CreateLimitedSlip(2.0f, 0.25f);

    /// <summary>Rear axle differential configuration.</summary>
    public DifferentialConfig RearDiff { get; set; } = DifferentialConfig.CreateLimitedSlip(3.0f, 0.35f);

    /// <summary>Center differential configuration (AWD torque split front/rear).</summary>
    public DifferentialConfig CenterDiff { get; set; } = DifferentialConfig.CreateLimitedSlip(1.8f, 0.3f);

    /// <summary>Whether the front axle receives drive torque.</summary>
    public bool DriveFrontAxle { get; set; } = true;

    /// <summary>Whether the rear axle receives drive torque.</summary>
    public bool DriveRearAxle { get; set; } = true;

    // ── Per-wheel state (struct arrays — no allocations) ─────────────────────

    /// <summary>Mutable per-wheel tyre state (temperature, wear, angular velocity, deflection).</summary>
    public readonly TyreState[] WheelStates = new TyreState[WheelCount];

    /// <summary>Tyre model for each wheel (may share instances if identical).</summary>
    public readonly TyreModel?[] TyreModels = new TyreModel?[WheelCount];

    /// <summary>Surface properties currently under each wheel.</summary>
    public readonly SurfaceProperties[] WheelSurfaces = new SurfaceProperties[WheelCount];

    /// <summary>Static normal load per wheel (N), set during vehicle build.</summary>
    public readonly float[] StaticNormalLoads = new float[WheelCount];

    /// <summary>Current normal load per wheel after load transfer (N). Read by telemetry.</summary>
    public readonly float[] CurrentNormalLoads = new float[WheelCount];

    /// <summary>Signed suspension travel per wheel (m). Positive = compressed, negative = rebound.</summary>
    public readonly float[] SuspensionCompression = new float[WheelCount];

    /// <summary>Whether each wheel currently has ground contact.</summary>
    public readonly bool[] WheelGrounded = new bool[WheelCount];

    // ── Asymmetric damping (bump/rebound) ─────────────────────────────────────

    /// <summary>Per-wheel bump (compression) damping coefficient (N·s/m). Loaded from BeamNG damp_bump_* vars.</summary>
    public readonly float[] BumpDamping = new float[WheelCount];

    /// <summary>Per-wheel rebound (extension) damping coefficient (N·s/m). Loaded from BeamNG damp_rebound_* vars.</summary>
    public readonly float[] ReboundDamping = new float[WheelCount];

    /// <summary>
    /// Per-wheel average damping coefficient (N·s/m) that BEPU's LinearAxisServo constraint uses.
    /// The correction impulse compensates for the difference between this and the direction-aware value.
    /// </summary>
    public readonly float[] BepuAverageDamping = new float[WheelCount];

    /// <summary>Previous frame's suspension compression per wheel, used to compute suspension velocity.</summary>
    private readonly float[] _previousSuspensionCompression = new float[WheelCount];

    /// <summary>Reusable snapshot buffer so the predictor pass can evaluate tyre forces without advancing wheel state twice.</summary>
    private readonly TyreState[] _predictionWheelStates = new TyreState[WheelCount];

    // ── Cached per-frame outputs ─────────────────────────────────────────────

    /// <summary>Longitudinal force per wheel (N). Read by telemetry.</summary>
    public readonly float[] LongitudinalForces = new float[WheelCount];

    /// <summary>Lateral force per wheel (N). Read by telemetry.</summary>
    public readonly float[] LateralForces = new float[WheelCount];

    /// <summary>Self-aligning torque per wheel (N·m). Applied to steering system.</summary>
    public readonly float[] SelfAligningTorques = new float[WheelCount];

    /// <summary>Drive torque delivered to each wheel after differential (N·m).</summary>
    public readonly float[] WheelDriveTorques = new float[WheelCount];

    // ── Force-derived acceleration state (cached for next step) ──────────────
    private Vector3 _forceEstimatedAccelerationWorld;
    private bool _hasForceEstimatedAcceleration;

    /// <summary>Initializes a new dynamics system with default tyre state and tarmac surfaces.</summary>
    public VehicleDynamicsSystem()
    {
        for (var i = 0; i < WheelCount; i++)
        {
            WheelStates[i] = TyreState.CreateDefault();
            WheelSurfaces[i] = SurfaceProperties.ForType(SurfaceType.Tarmac);
        }
    }

    /// <summary>
    /// Main per-frame update. Computes all vehicle dynamics and applies impulses to BEPU bodies.
    ///
    /// Called from <see cref="RallyCarComponent.Update"/> after input processing and motor commands.
    /// </summary>
    /// <param name="chassisBody">BEPU rigid body for the chassis.</param>
    /// <param name="chassisWorld">Chassis world transform matrix.</param>
    /// <param name="wheelPositions">World-space position of each wheel contact point.</param>
    /// <param name="wheelVelocities">World-space linear velocity at each wheel.</param>
    /// <param name="wheelOrientations">World-space orientation frames for each wheel (Right, Up, Forward).</param>
    /// <param name="wheelContactScales">Per-wheel contact confidence/load scales in the range [0, 1].</param>
    /// <param name="wheelGrounded">Whether each wheel has ground contact.</param>
    /// <param name="suspensionCompressions">Signed suspension travel for each wheel (m). Positive = compressed.</param>
    /// <param name="engineTorqueAtWheels">Total engine torque after gearbox at wheel level (N·m).</param>
    /// <param name="brakeTorque">Brake torque per wheel (N·m).</param>
    /// <param name="camberAngles">Camber angle per wheel (rad).</param>
    /// <param name="dt">Physics timestep (s).</param>
    public void Update(
        BodyComponent chassisBody,
        in Matrix chassisWorld,
        ReadOnlySpan<Vector3> wheelPositions,
        ReadOnlySpan<Vector3> wheelVelocities,
        ReadOnlySpan<Matrix> wheelOrientations,
        ReadOnlySpan<float> wheelContactScales,
        ReadOnlySpan<bool> wheelGrounded,
        ReadOnlySpan<float> suspensionCompressions,
        float engineTorqueAtWheels,
        ReadOnlySpan<float> brakeTorque,
        ReadOnlySpan<float> camberAngles,
        float dt)
    {
        if (dt < 1e-6f)
        {
	        return;
        }

        // ── 1. Use force-derived chassis acceleration for load transfer ───────
        // Acceleration is estimated from summed tyre forces at the end of each
        // step and cached for use on the next fixed update.
        // First step fallback: zero acceleration until the first force estimate
        // has been produced.
        // Reference: F = m·a (Newton's second law).
        // Note: chassisWorld.Backward = local +Z = car nose direction (Stride coordinate convention).
        var forwardDir = SafeNormalize(chassisWorld.Backward, Vector3.UnitZ);
        var rightDir = SafeNormalize(chassisWorld.Right, Vector3.UnitX);
        var predictedAccelerationWorld = _hasForceEstimatedAcceleration
            ? _forceEstimatedAccelerationWorld
            : Vector3.Zero;
        var longitudinalAccel = Vector3.Dot(predictedAccelerationWorld, forwardDir);
        var lateralAccel = Vector3.Dot(predictedAccelerationWorld, rightDir);

        // ── 2. First force pass using the cached estimate as a predictor ─────
        var instantaneousAccelerationWorld = RunForcePredictionPass(
            wheelGrounded,
            wheelContactScales,
            suspensionCompressions,
            engineTorqueAtWheels,
            wheelOrientations,
            wheelVelocities,
            brakeTorque,
            camberAngles,
            longitudinalAccel,
            lateralAccel,
            preserveWheelState: true,
            dt);

        // ── 6. Re-evaluate load transfer with the current-step force estimate ─
        // First pass uses the previous force estimate as a predictor. Recomputing within the
        // same step removes the one-frame lag from load transfer and body attitude response.
        longitudinalAccel = Vector3.Dot(instantaneousAccelerationWorld, forwardDir);
        lateralAccel = Vector3.Dot(instantaneousAccelerationWorld, rightDir);

        instantaneousAccelerationWorld = RunForcePredictionPass(
            wheelGrounded,
            wheelContactScales,
            suspensionCompressions,
            engineTorqueAtWheels,
            wheelOrientations,
            wheelVelocities,
            brakeTorque,
            camberAngles,
            longitudinalAccel,
            lateralAccel,
            preserveWheelState: false,
            dt);
        longitudinalAccel = Vector3.Dot(instantaneousAccelerationWorld, forwardDir);
        lateralAccel = Vector3.Dot(instantaneousAccelerationWorld, rightDir);

        // ── 6b. Chassis body attitude torque from suspension compression and sprung-mass acceleration ─
        // Load transfer creates a body moment M = m * a * h about the chassis axes even before
        // the compression-delta term becomes large enough to be visually obvious in the suspension.
        // Feeding that moment into the attitude system makes braking dive and cornering lean show up
        // earlier and more like a sprung body, while still letting suspension compression and damping
        // shape the final response.
        var rollAccelerationTorque = ComputeRollAccelerationTorque(lateralAccel);
        var pitchAccelerationTorque = -longitudinalAccel * VehicleMass
                                      * ComputeEffectivePitchTransferHeight(
                                          CgHeight,
                                          longitudinalAccel,
                                          AntiDiveFactor,
                                          AntiSquatFactor);
        BodyRoll.Apply(
            chassisBody,
            forwardDir,
            rightDir,
            suspensionCompressions,
            rollAccelerationTorque,
            pitchAccelerationTorque,
            dt);

        _forceEstimatedAccelerationWorld = instantaneousAccelerationWorld;
        _hasForceEstimatedAcceleration = true;

        // ── 7. Apply chassis impulses after the force prediction is complete ──
        // The damping correction mutates chassisBody directly, so it must run after the tyre-force
        // prediction passes that used the current wheel-velocity snapshot.
        ApplyAsymmetricDampingCorrection(chassisBody, in chassisWorld, wheelPositions, wheelOrientations, suspensionCompressions, dt);

        // ── 8. Apply all forces as impulses to the chassis at the wheel contact locations ─
        ApplyForces(chassisBody, in chassisWorld, wheelPositions, wheelOrientations, dt);
    }

    /// <summary>
    /// Computes longitudinal and lateral load transfer and modifies per-wheel normal loads.
    ///
    /// <para>Longitudinal load transfer (acceleration/braking):
    /// ΔF = (m · a_x · h_eff) / L where h_eff is reduced by anti-dive / anti-squat geometry.
    /// Positive a_x (acceleration) shifts load rearward.
    /// Reference: Milliken, RCVD §18.2, Eq. 18.1.</para>
    ///
    /// <para>Lateral load transfer (cornering):
    /// Each axle uses ΔF_axle = (m_axle · a_y · (h_cg − h_rc)) / T so higher roll centers
    /// reduce the sprung-mass lever arm. Positive a_y (rightward) shifts load to the left wheels.
    /// Reference: Milliken, RCVD §18.3.</para>
    /// </summary>
    private void ComputeLoadTransfer(
        float longitudinalAccel,
        float lateralAccel,
        ReadOnlySpan<bool> wheelGrounded,
        ReadOnlySpan<float> wheelContactScales)
    {
        var wheelbase = MathF.Max(Wheelbase, 0.1f);
        var trackWidth = MathF.Max(TrackWidth, 0.1f);

        var effectivePitchHeight = ComputeEffectivePitchTransferHeight(
            CgHeight,
            longitudinalAccel,
            AntiDiveFactor,
            AntiSquatFactor);
        var longTransfer = (VehicleMass * longitudinalAccel * effectivePitchHeight) / wheelbase;
        GetAxleLoadFractions(out var frontAxleLoadFraction, out var rearAxleLoadFraction);
        var frontLatTransfer = ComputeAxleLateralTransfer(
            VehicleMass,
            frontAxleLoadFraction,
            lateralAccel,
            CgHeight,
            FrontRollCenterHeight,
            trackWidth);
        var rearLatTransfer = ComputeAxleLateralTransfer(
            VehicleMass,
            rearAxleLoadFraction,
            lateralAccel,
            CgHeight,
            RearRollCenterHeight,
            trackWidth);

        // Start from static loads, then add transfer
        for (var i = 0; i < WheelCount; i++)
        {
            CurrentNormalLoads[i] = StaticNormalLoads[i];
            var contactScale = i < wheelContactScales.Length
                ? Math.Clamp(wheelContactScales[i], 0f, 1f)
                : wheelGrounded[i] ? 1f : 0f;
            WheelGrounded[i] = wheelGrounded[i] || contactScale > 0.05f;
        }

        // Longitudinal: acceleration shifts load rearward (front loses, rear gains)
        CurrentNormalLoads[FL] -= longTransfer * 0.5f;
        CurrentNormalLoads[FR] -= longTransfer * 0.5f;
        CurrentNormalLoads[RL] += longTransfer * 0.5f;
        CurrentNormalLoads[RR] += longTransfer * 0.5f;

        // Lateral: rightward acceleration shifts load to left (left gains, right loses)
        CurrentNormalLoads[FL] += frontLatTransfer * 0.5f;
        CurrentNormalLoads[FR] -= frontLatTransfer * 0.5f;
        CurrentNormalLoads[RL] += rearLatTransfer * 0.5f;
        CurrentNormalLoads[RR] -= rearLatTransfer * 0.5f;

        // Clamp — wheel cannot push up (negative load means wheel has lifted)
        for (var i = 0; i < WheelCount; i++)
        {
            var contactScale = i < wheelContactScales.Length
                ? Math.Clamp(wheelContactScales[i], 0f, 1f)
                : wheelGrounded[i] ? 1f : 0f;

            if (!WheelGrounded[i])
            {
	            CurrentNormalLoads[i] = 0f;
            }
            else
            {
	            CurrentNormalLoads[i] = MathF.Max(CurrentNormalLoads[i], 0f) * contactScale;
            }
        }
    }

    /// <summary>
    /// Anti-roll bar simulation: links left/right suspension to resist body roll.
    ///
    /// <para>For each axle, computes the compression difference between left and right:
    ///   δ = compression_left − compression_right
    ///   F_roll = δ × K_arb
    /// This force is added to the inner wheel and subtracted from the outer,
    /// increasing load on the more-compressed side.</para>
    ///
    /// Anti-roll stiffness should be loaded from JBeam if available.
    ///
    /// Reference: Milliken, RCVD §17.6, anti-roll bar mechanics.
    /// </summary>
    private void ComputeAntiRollForces(ReadOnlySpan<float> suspensionCompressions)
    {
        // Cache suspension compression
        for (var i = 0; i < WheelCount && i < suspensionCompressions.Length; i++)
            SuspensionCompression[i] = suspensionCompressions[i];

        // Front axle anti-roll bar
        if (FrontAntiRollStiffness > 0f)
        {
            var deltaFront = SuspensionCompression[FL] - SuspensionCompression[FR];
            var rollForce = deltaFront * FrontAntiRollStiffness;
            // Opposing forces: push down compressed side, push up extended side
            CurrentNormalLoads[FL] += rollForce;
            CurrentNormalLoads[FR] -= rollForce;
        }

        // Rear axle anti-roll bar
        if (RearAntiRollStiffness > 0f)
        {
            var deltaRear = SuspensionCompression[RL] - SuspensionCompression[RR];
            var rollForce = deltaRear * RearAntiRollStiffness;
            CurrentNormalLoads[RL] += rollForce;
            CurrentNormalLoads[RR] -= rollForce;
        }

        // Re-clamp after anti-roll adjustments
        for (var i = 0; i < WheelCount; i++)
            CurrentNormalLoads[i] = MathF.Max(CurrentNormalLoads[i], 0f);
    }

    /// <summary>
    /// Applies corrective impulses for asymmetric (bump vs rebound) suspension damping.
    ///
    /// <para>BEPU's LinearAxisServo uses a single SpringDampingRatio computed from the
    /// average of bump and rebound damping coefficients. Real rally suspension has
    /// significantly different bump (compression) and rebound (extension) damping —
    /// typically rebound is 2-3× stiffer than bump.</para>
    ///
    /// <para>For each wheel, this method:
    /// <list type="number">
    ///   <item>Computes suspension velocity from compression change: v = (C_now − C_prev) / dt</item>
    ///   <item>Selects the direction-appropriate coefficient: bump if compressing, rebound if extending</item>
    ///   <item>Computes correction force: F = (C_selected − C_average) × v</item>
    ///   <item>Applies the correction as an impulse along the suspension axis at the wheel position</item>
    /// </list></para>
    ///
    /// <para>Reference: Milliken, RCVD §17.2 — Damper characteristics (digressive/progressive).</para>
    /// </summary>
    private void ApplyAsymmetricDampingCorrection(
        BodyComponent chassisBody,
        in Matrix chassisWorld,
        ReadOnlySpan<Vector3> wheelPositions,
        ReadOnlySpan<Matrix> wheelOrientations,
        ReadOnlySpan<float> suspensionCompressions,
        float dt)
    {
        var chassisPosition = chassisWorld.TranslationVector;

        for (var i = 0; i < WheelCount; i++)
        {
            if (!WheelGrounded[i])
            {
                _previousSuspensionCompression[i] = i < suspensionCompressions.Length
                    ? suspensionCompressions[i]
                    : 0f;
                continue;
            }

            var currentCompression = i < suspensionCompressions.Length ? suspensionCompressions[i] : 0f;
            var suspensionVelocity = (currentCompression - _previousSuspensionCompression[i]) / dt;
            _previousSuspensionCompression[i] = currentCompression;

            var averageDamping = BepuAverageDamping[i];
            if (averageDamping <= 0f)
                continue;

            // Positive velocity = compressing (bump), negative = extending (rebound)
            var directionDamping = suspensionVelocity >= 0f
                ? BumpDamping[i]
                : ReboundDamping[i];

            if (directionDamping <= 0f)
                continue;

            var correctionForce = (directionDamping - averageDamping) * suspensionVelocity;

            // Clamp correction to avoid extreme transients (e.g. first frame or teleport)
            const float maxCorrectionForce = 8000f;
            correctionForce = Math.Clamp(correctionForce, -maxCorrectionForce, maxCorrectionForce);

            if (MathF.Abs(correctionForce) < 1f)
                continue;

            // Apply along the suspension axis (approximately wheel up direction)
            var wheelUp = SafeNormalize(wheelOrientations[i].Up, Vector3.UnitY);
            var impulse = wheelUp * (correctionForce * dt);
            var contactOffset = wheelPositions[i] - chassisPosition;

            chassisBody.Awake = true;
            chassisBody.ApplyImpulse(impulse, contactOffset);
        }
    }

    /// <summary>
    /// Distributes engine torque through the drivetrain:
    /// engine → clutch → gearbox → center differential → front/rear differential → wheels.
    ///
    /// The center diff splits torque between front and rear axles.
    /// Each axle diff then splits between left and right wheels.
    ///
    /// Reference: Milliken, RCVD Chapter 6.
    /// </summary>
    private void ComputeDrivetrainTorque(float engineTorqueAtWheels)
    {
        for (var i = 0; i < WheelCount; i++)
        {
            WheelDriveTorques[i] = 0f;
        }

        if (!DriveFrontAxle && !DriveRearAxle)
        {
            return;
        }

        if (DriveFrontAxle && !DriveRearAxle)
        {
            var frontDiffOnly = FrontDiff;
            DifferentialSolver.SplitTorque(in frontDiffOnly, engineTorqueAtWheels,
                WheelStates[FL].AngularVelocity, WheelStates[FR].AngularVelocity,
                out WheelDriveTorques[FL], out WheelDriveTorques[FR]);
            return;
        }

        if (!DriveFrontAxle)
        {
            var rearDiffOnly = RearDiff;
            DifferentialSolver.SplitTorque(in rearDiffOnly, engineTorqueAtWheels,
                WheelStates[RL].AngularVelocity, WheelStates[RR].AngularVelocity,
                out WheelDriveTorques[RL], out WheelDriveTorques[RR]);
            return;
        }

        // Center diff splits to front and rear axles
        var omegaFrontAxle = (WheelStates[FL].AngularVelocity + WheelStates[FR].AngularVelocity) * 0.5f;
        var omegaRearAxle = (WheelStates[RL].AngularVelocity + WheelStates[RR].AngularVelocity) * 0.5f;

        var centerDiff = CenterDiff;
        DifferentialSolver.SplitTorque(in centerDiff, engineTorqueAtWheels,
            omegaFrontAxle, omegaRearAxle,
            out var frontAxleTorque, out var rearAxleTorque);

        // Front diff splits between FL and FR
        var frontDiff = FrontDiff;
        DifferentialSolver.SplitTorque(in frontDiff, frontAxleTorque,
            WheelStates[FL].AngularVelocity, WheelStates[FR].AngularVelocity,
            out var torqueFL, out var torqueFR);
        WheelDriveTorques[FL] = torqueFL;
        WheelDriveTorques[FR] = torqueFR;

        // Rear diff splits between RL and RR
        var rearDiff = RearDiff;
        DifferentialSolver.SplitTorque(in rearDiff, rearAxleTorque,
            WheelStates[RL].AngularVelocity, WheelStates[RR].AngularVelocity,
            out var torqueRL, out var torqueRR);
        WheelDriveTorques[RL] = torqueRL;
        WheelDriveTorques[RR] = torqueRR;
    }

    /// <summary>
    /// Evaluates the tyre model for each wheel, computing longitudinal force (Fx),
    /// lateral force (Fy), and self-aligning torque (Mz).
    /// </summary>
    private void ComputeTyreForces(
        ReadOnlySpan<Matrix> wheelOrientations,
        ReadOnlySpan<Vector3> wheelVelocities,
        ReadOnlySpan<float> brakeTorques,
        ReadOnlySpan<float> camberAngles,
        float dt)
    {
        for (var i = 0; i < WheelCount; i++)
        {
            var tyreModel = TyreModels[i];
            if (tyreModel == null)
            {
                LongitudinalForces[i] = 0f;
                LateralForces[i] = 0f;
                SelfAligningTorques[i] = 0f;
                continue;
            }

            if (!WheelGrounded[i])
            {
                LongitudinalForces[i] = 0f;
                LateralForces[i] = 0f;
                SelfAligningTorques[i] = 0f;
                WheelStates[i].LateralDeflection = 0f;

                // Still call Update with normalLoad=0 so angular velocity integrates
                // from drive/brake torque while airborne (important for AWD diffs and landing).
                tyreModel.Update(
                    ref WheelStates[i],
                    0f, 0f, 0f,
                    WheelDriveTorques[i],
                    brakeTorques[i],
                    0f,
                    in WheelSurfaces[i],
                    dt,
                    out _, out _, out _);
                continue;
            }

            // Decompose wheel velocity into longitudinal and lateral components
            // in the wheel's local frame.
            var wheelRight = SafeNormalize(wheelOrientations[i].Right, Vector3.UnitX);
            var wheelUp = SafeNormalize(wheelOrientations[i].Up, Vector3.UnitY);
            var wheelForward = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), Vector3.UnitZ);

            var vel = wheelVelocities[i];
            var longVel = Vector3.Dot(vel, wheelForward);
            var latVel = Vector3.Dot(vel, wheelRight);

            tyreModel.Update(
                ref WheelStates[i],
                longVel,
                latVel,
                CurrentNormalLoads[i],
                WheelDriveTorques[i],
                brakeTorques[i],
                camberAngles[i],
                in WheelSurfaces[i],
                dt,
                out LongitudinalForces[i],
                out LateralForces[i],
                out SelfAligningTorques[i]);
        }
    }

    private Vector3 RunForcePredictionPass(
        ReadOnlySpan<bool> wheelGrounded,
        ReadOnlySpan<float> wheelContactScales,
        ReadOnlySpan<float> suspensionCompressions,
        float engineTorqueAtWheels,
        ReadOnlySpan<Matrix> wheelOrientations,
        ReadOnlySpan<Vector3> wheelVelocities,
        ReadOnlySpan<float> brakeTorques,
        ReadOnlySpan<float> camberAngles,
        float longitudinalAccel,
        float lateralAccel,
        bool preserveWheelState,
        float dt)
    {
        if (preserveWheelState)
        {
            Array.Copy(WheelStates, _predictionWheelStates, WheelCount);
        }

        ComputeLoadTransfer(longitudinalAccel, lateralAccel, wheelGrounded, wheelContactScales);
        ComputeAntiRollForces(suspensionCompressions);
        ComputeDrivetrainTorque(engineTorqueAtWheels);
        ComputeTyreForces(wheelOrientations, wheelVelocities, brakeTorques, camberAngles, dt);
        var predictedAcceleration = EstimateForceBasedAcceleration(wheelOrientations);

        if (preserveWheelState)
        {
            Array.Copy(_predictionWheelStates, WheelStates, WheelCount);
        }

        return predictedAcceleration;
    }

    /// <summary>
    /// Estimates world-space chassis acceleration from the current modelled tyre forces.
    /// This estimate is cached and projected to chassis axes on the next update.
    /// </summary>
    private Vector3 EstimateForceBasedAcceleration(
        ReadOnlySpan<Matrix> wheelOrientations)
    {
        var netForceWorld = Vector3.Zero;

        for (var i = 0; i < WheelCount; i++)
        {
            if (!WheelGrounded[i])
            {
                continue;
            }

            var wheelRight = SafeNormalize(wheelOrientations[i].Right, Vector3.UnitX);
            var wheelUp = SafeNormalize(wheelOrientations[i].Up, Vector3.UnitY);
            var wheelForward = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), Vector3.UnitZ);
            var wheelForceWorld = wheelForward * LongitudinalForces[i] + wheelRight * LateralForces[i];
            netForceWorld += wheelForceWorld;
        }

        return EstimateWorldAccelerationFromNetForce(netForceWorld, VehicleMass);
    }

    internal static Vector3 EstimateWorldAccelerationFromNetForce(Vector3 netForceWorld, float vehicleMass)
    {
        if (vehicleMass <= 1e-4f)
        {
            return Vector3.Zero;
        }

        return netForceWorld / vehicleMass;
    }

    internal static float ComputeEffectivePitchTransferHeight(
        float cgHeight,
        float longitudinalAccel,
        float antiDiveFactor,
        float antiSquatFactor)
    {
        var clampedCgHeight = MathF.Max(cgHeight, 0.01f);
        var activeFactor = longitudinalAccel >= 0f
            ? Math.Clamp(antiSquatFactor, 0f, 0.95f)
            : Math.Clamp(antiDiveFactor, 0f, 0.95f);
        return MathF.Max(clampedCgHeight * (1f - activeFactor), 0.01f);
    }

    internal static float ComputeLateralRollMomentArm(float cgHeight, float rollCenterHeight)
    {
        var clampedCgHeight = MathF.Max(cgHeight, 0.01f);
        var clampedRollCenter = Math.Clamp(rollCenterHeight, 0f, clampedCgHeight - 0.01f);
        return MathF.Max(clampedCgHeight - clampedRollCenter, 0.01f);
    }

    internal static float ComputeAxleLateralTransfer(
        float vehicleMass,
        float axleLoadFraction,
        float lateralAccel,
        float cgHeight,
        float rollCenterHeight,
        float trackWidth)
    {
        if (vehicleMass <= 1e-4f)
        {
            return 0f;
        }

        var clampedTrackWidth = MathF.Max(trackWidth, 0.1f);
        var clampedLoadFraction = Math.Clamp(axleLoadFraction, 0f, 1f);
        var rollMomentArm = ComputeLateralRollMomentArm(cgHeight, rollCenterHeight);
        return (vehicleMass * clampedLoadFraction * lateralAccel * rollMomentArm) / clampedTrackWidth;
    }

    /// <summary>
    /// Converts net world-space force to chassis-local planar acceleration
    /// components (longitudinal, lateral).
    /// Internal visibility is intentional so deterministic math can be unit-tested
    /// from LibreRally.Tests via InternalsVisibleTo.
    /// </summary>
    /// <param name="netForceWorld">Summed world-space force acting on the chassis (N).</param>
    /// <param name="chassisForward">Normalized chassis forward axis in world space.</param>
    /// <param name="chassisRight">Normalized chassis right axis in world space.</param>
    /// <param name="vehicleMass">Vehicle mass in kilograms.</param>
    /// <returns>Planar acceleration where X = longitudinal and Y = lateral (m/s²).</returns>
    internal static Vector2 EstimatePlanarAccelerationFromNetForce(
        Vector3 netForceWorld,
        Vector3 chassisForward,
        Vector3 chassisRight,
        float vehicleMass)
    {
        var accelerationWorld = EstimateWorldAccelerationFromNetForce(netForceWorld, vehicleMass);
        return new Vector2(
            Vector3.Dot(accelerationWorld, chassisForward),
            Vector3.Dot(accelerationWorld, chassisRight));
    }

    /// <summary>
    /// Applies all computed tyre forces to the chassis at each wheel position.
    /// The wheel rigid bodies provide collision/suspension geometry, while the tyre model
    /// is authoritative for contact-patch grip and therefore pushes the sprung mass directly.
    /// </summary>
    private void ApplyForces(
        BodyComponent chassisBody,
        in Matrix chassisWorld,
        ReadOnlySpan<Vector3> wheelPositions,
        ReadOnlySpan<Matrix> wheelOrientations,
        float dt)
    {
        var chassisPosition = chassisWorld.TranslationVector;

        for (var i = 0; i < WheelCount; i++)
        {
            if (!WheelGrounded[i])
            {
	            continue;
            }

            var fx = LongitudinalForces[i];
            var fy = LateralForces[i];

            if (MathF.Abs(fx) < MinimumWakeForce && MathF.Abs(fy) < MinimumWakeForce)
            {
	            continue;
            }

            // Build force vector in world space using wheel orientation
            var wheelRight = SafeNormalize(wheelOrientations[i].Right, Vector3.UnitX);
            var wheelUp = SafeNormalize(wheelOrientations[i].Up, Vector3.UnitY);
            var wheelForward = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), Vector3.UnitZ);

            var forceWorld = wheelForward * fx + wheelRight * fy;
            var impulse = forceWorld * dt;
            var tyreRadius = TyreModels[i]?.Radius ?? 0.305f;
            var contactPatchPosition = wheelPositions[i] - wheelUp * MathF.Max(tyreRadius, 0.1f);
            var contactOffset = contactPatchPosition - chassisPosition;

            chassisBody.Awake = true;
            chassisBody.ApplyImpulse(impulse, contactOffset);
        }
    }

    /// <summary>
    /// Returns the total self-aligning torque from all wheels.
    /// This should be fed back to the steering system to provide force feedback.
    /// </summary>
    /// <returns>The combined self-aligning torque generated by the steered wheels.</returns>
    public float GetTotalSelfAligningTorque()
    {
        // Only front (steered) wheels contribute to steering feel
        return SelfAligningTorques[FL] + SelfAligningTorques[FR];
    }

    private float ComputeRollAccelerationTorque(float lateralAccel)
    {
        GetAxleLoadFractions(out var frontAxleLoadFraction, out var rearAxleLoadFraction);
        var frontRollMoment = frontAxleLoadFraction
                              * ComputeLateralRollMomentArm(CgHeight, FrontRollCenterHeight);
        var rearRollMoment = rearAxleLoadFraction
                             * ComputeLateralRollMomentArm(CgHeight, RearRollCenterHeight);
        return lateralAccel * VehicleMass * (frontRollMoment + rearRollMoment);
    }

    private void GetAxleLoadFractions(out float frontAxleLoadFraction, out float rearAxleLoadFraction)
    {
        var frontAxleLoad = StaticNormalLoads[FL] + StaticNormalLoads[FR];
        var rearAxleLoad = StaticNormalLoads[RL] + StaticNormalLoads[RR];
        var totalLoad = frontAxleLoad + rearAxleLoad;
        if (totalLoad <= 1e-4f)
        {
            frontAxleLoadFraction = 0.5f;
            rearAxleLoadFraction = 0.5f;
            return;
        }

        frontAxleLoadFraction = frontAxleLoad / totalLoad;
        rearAxleLoadFraction = rearAxleLoad / totalLoad;
    }

    private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
    {
        var lengthSq = value.LengthSquared();
        if (lengthSq < 1e-6f)
        {
	        return fallback;
        }

        return value / MathF.Sqrt(lengthSq);
    }
}
