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
///   <item>Compute longitudinal and lateral load transfer from chassis acceleration.</item>
///   <item>Compute anti-roll bar forces from suspension compression difference.</item>
///   <item>Split engine torque through the drivetrain (center diff → axle diffs → wheels).</item>
///   <item>Evaluate tyre model for each wheel (slip ratio, slip angle → Fx, Fy, Mz).</item>
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
    // Wheel indices — fixed order matching VehiclePhysicsBuilder output.
    public const int FL = 0;
    public const int FR = 1;
    public const int RL = 2;
    public const int RR = 3;
    public const int WheelCount = 4;

    // ── Vehicle-level configuration ──────────────────────────────────────────

    /// <summary>Total vehicle mass including chassis + wheels (kg).</summary>
    public float VehicleMass { get; set; } = 1200f;

    /// <summary>Centre-of-gravity height above ground (m). Affects load transfer magnitude.</summary>
    public float CgHeight { get; set; } = 0.45f;

    /// <summary>Front-to-rear axle distance (m).</summary>
    public float Wheelbase { get; set; } = 2.55f;

    /// <summary>Left-to-right wheel centre distance (m). Used for lateral load transfer.</summary>
    public float TrackWidth { get; set; } = 1.50f;

    /// <summary>Front anti-roll bar stiffness (N·m/rad). 0 = disabled.</summary>
    public float FrontAntiRollStiffness { get; set; } = 8000f;

    /// <summary>Rear anti-roll bar stiffness (N·m/rad). 0 = disabled.</summary>
    public float RearAntiRollStiffness { get; set; } = 5000f;

    /// <summary>Front axle differential configuration.</summary>
    public DifferentialConfig FrontDiff { get; set; } = DifferentialConfig.CreateLimitedSlip(2.0f, 0.25f);

    /// <summary>Rear axle differential configuration.</summary>
    public DifferentialConfig RearDiff { get; set; } = DifferentialConfig.CreateLimitedSlip(3.0f, 0.35f);

    /// <summary>Center differential configuration (AWD torque split front/rear).</summary>
    public DifferentialConfig CenterDiff { get; set; } = DifferentialConfig.CreateLimitedSlip(1.8f, 0.3f);

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

    /// <summary>Suspension compression per wheel (m). Positive = compressed.</summary>
    public readonly float[] SuspensionCompression = new float[WheelCount];

    /// <summary>Whether each wheel currently has ground contact.</summary>
    public readonly bool[] WheelGrounded = new bool[WheelCount];

    // ── Cached per-frame outputs ─────────────────────────────────────────────

    /// <summary>Longitudinal force per wheel (N). Read by telemetry.</summary>
    public readonly float[] LongitudinalForces = new float[WheelCount];

    /// <summary>Lateral force per wheel (N). Read by telemetry.</summary>
    public readonly float[] LateralForces = new float[WheelCount];

    /// <summary>Self-aligning torque per wheel (N·m). Applied to steering system.</summary>
    public readonly float[] SelfAligningTorques = new float[WheelCount];

    /// <summary>Drive torque delivered to each wheel after differential (N·m).</summary>
    public readonly float[] WheelDriveTorques = new float[WheelCount];

    // ── Previous velocity for acceleration estimation ────────────────────────

    private Vector3 _previousVelocity;

    public VehicleDynamicsSystem()
    {
        for (int i = 0; i < WheelCount; i++)
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
    /// <param name="wheelGrounded">Whether each wheel has ground contact.</param>
    /// <param name="suspensionCompressions">Suspension compression for each wheel (m).</param>
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
        ReadOnlySpan<bool> wheelGrounded,
        ReadOnlySpan<float> suspensionCompressions,
        float engineTorqueAtWheels,
        ReadOnlySpan<float> brakeTorque,
        ReadOnlySpan<float> camberAngles,
        float dt)
    {
        if (dt < 1e-6f) return;

        Vector3 chassisPosition = chassisWorld.TranslationVector;
        Vector3 chassisVelocity = chassisBody.LinearVelocity;

        // ── 1. Estimate chassis acceleration for load transfer ───────────────
        // Use finite difference of velocity for acceleration estimation.
        // Reference: Milliken, RCVD §5.6, load transfer from measured acceleration.
        Vector3 acceleration = (chassisVelocity - _previousVelocity) / dt;
        _previousVelocity = chassisVelocity;

        // Project acceleration into chassis-local frame.
        // Note: chassisWorld.Backward = local +Z = car nose direction (Stride coordinate convention).
        Vector3 forwardDir = SafeNormalize(chassisWorld.Backward, Vector3.UnitZ);
        Vector3 rightDir = SafeNormalize(chassisWorld.Right, Vector3.UnitX);
        float longitudinalAccel = Vector3.Dot(acceleration, forwardDir);
        float lateralAccel = Vector3.Dot(acceleration, rightDir);

        // ── 2. Compute load transfer ─────────────────────────────────────────
        ComputeLoadTransfer(longitudinalAccel, lateralAccel, wheelGrounded);

        // ── 3. Compute anti-roll bar forces ──────────────────────────────────
        ComputeAntiRollForces(suspensionCompressions);

        // ── 4. Split engine torque through drivetrain ────────────────────────
        ComputeDrivetrainTorque(engineTorqueAtWheels);

        // ── 5. Evaluate tyre model for each wheel ───────────────────────────
        ComputeTyreForces(wheelOrientations, wheelVelocities, brakeTorque, camberAngles, dt);

        // ── 6. Apply all forces as impulses to BEPU chassis body ─────────────
        ApplyForces(chassisBody, chassisPosition, wheelPositions, wheelOrientations, dt);
    }

    /// <summary>
    /// Computes longitudinal and lateral load transfer and modifies per-wheel normal loads.
    ///
    /// <para>Longitudinal load transfer (acceleration/braking):
    /// ΔF = (m · a_x · h_cg) / L
    /// Positive a_x (acceleration) shifts load rearward.
    /// Reference: Milliken, RCVD §18.2, Eq. 18.1.</para>
    ///
    /// <para>Lateral load transfer (cornering):
    /// ΔF = (m · a_y · h_cg) / T
    /// Positive a_y (rightward) shifts load to the left wheels.
    /// Reference: Milliken, RCVD §18.3.</para>
    /// </summary>
    private void ComputeLoadTransfer(float longitudinalAccel, float lateralAccel,
        ReadOnlySpan<bool> wheelGrounded)
    {
        float wheelbase = MathF.Max(Wheelbase, 0.1f);
        float trackWidth = MathF.Max(TrackWidth, 0.1f);

        // Longitudinal load transfer: ΔF = (m · a_x · h_cg) / L
        float longTransfer = (VehicleMass * longitudinalAccel * CgHeight) / wheelbase;

        // Lateral load transfer: ΔF = (m · a_y · h_cg) / T
        float latTransfer = (VehicleMass * lateralAccel * CgHeight) / trackWidth;

        // Start from static loads, then add transfer
        for (int i = 0; i < WheelCount; i++)
        {
            CurrentNormalLoads[i] = StaticNormalLoads[i];
            WheelGrounded[i] = wheelGrounded[i];
        }

        // Longitudinal: acceleration shifts load rearward (front loses, rear gains)
        CurrentNormalLoads[FL] -= longTransfer * 0.5f;
        CurrentNormalLoads[FR] -= longTransfer * 0.5f;
        CurrentNormalLoads[RL] += longTransfer * 0.5f;
        CurrentNormalLoads[RR] += longTransfer * 0.5f;

        // Lateral: rightward acceleration shifts load to left (left gains, right loses)
        CurrentNormalLoads[FL] += latTransfer * 0.5f;
        CurrentNormalLoads[FR] -= latTransfer * 0.5f;
        CurrentNormalLoads[RL] += latTransfer * 0.5f;
        CurrentNormalLoads[RR] -= latTransfer * 0.5f;

        // Clamp — wheel cannot push up (negative load means wheel has lifted)
        for (int i = 0; i < WheelCount; i++)
        {
            if (!WheelGrounded[i])
                CurrentNormalLoads[i] = 0f;
            else
                CurrentNormalLoads[i] = MathF.Max(CurrentNormalLoads[i], 0f);
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
        for (int i = 0; i < WheelCount && i < suspensionCompressions.Length; i++)
            SuspensionCompression[i] = suspensionCompressions[i];

        // Front axle anti-roll bar
        if (FrontAntiRollStiffness > 0f)
        {
            float deltaFront = SuspensionCompression[FL] - SuspensionCompression[FR];
            float rollForce = deltaFront * FrontAntiRollStiffness;
            // Opposing forces: push down compressed side, push up extended side
            CurrentNormalLoads[FL] += rollForce;
            CurrentNormalLoads[FR] -= rollForce;
        }

        // Rear axle anti-roll bar
        if (RearAntiRollStiffness > 0f)
        {
            float deltaRear = SuspensionCompression[RL] - SuspensionCompression[RR];
            float rollForce = deltaRear * RearAntiRollStiffness;
            CurrentNormalLoads[RL] += rollForce;
            CurrentNormalLoads[RR] -= rollForce;
        }

        // Re-clamp after anti-roll adjustments
        for (int i = 0; i < WheelCount; i++)
            CurrentNormalLoads[i] = MathF.Max(CurrentNormalLoads[i], 0f);
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
        // Center diff splits to front and rear axles
        float omegaFrontAxle = (WheelStates[FL].AngularVelocity + WheelStates[FR].AngularVelocity) * 0.5f;
        float omegaRearAxle = (WheelStates[RL].AngularVelocity + WheelStates[RR].AngularVelocity) * 0.5f;

        var centerDiff = CenterDiff;
        DifferentialSolver.SplitTorque(in centerDiff, engineTorqueAtWheels,
            omegaFrontAxle, omegaRearAxle,
            out float frontAxleTorque, out float rearAxleTorque);

        // Front diff splits between FL and FR
        var frontDiff = FrontDiff;
        DifferentialSolver.SplitTorque(in frontDiff, frontAxleTorque,
            WheelStates[FL].AngularVelocity, WheelStates[FR].AngularVelocity,
            out float torqueFL, out float torqueFR);
        WheelDriveTorques[FL] = torqueFL;
        WheelDriveTorques[FR] = torqueFR;

        // Rear diff splits between RL and RR
        var rearDiff = RearDiff;
        DifferentialSolver.SplitTorque(in rearDiff, rearAxleTorque,
            WheelStates[RL].AngularVelocity, WheelStates[RR].AngularVelocity,
            out float torqueRL, out float torqueRR);
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
        for (int i = 0; i < WheelCount; i++)
        {
            var tyreModel = TyreModels[i];
            if (tyreModel == null || !WheelGrounded[i])
            {
                LongitudinalForces[i] = 0f;
                LateralForces[i] = 0f;
                SelfAligningTorques[i] = 0f;

                if (tyreModel != null)
                    WheelStates[i].LateralDeflection = 0f;
                continue;
            }

            // Decompose wheel velocity into longitudinal and lateral components
            // in the wheel's local frame.
            Vector3 wheelRight = SafeNormalize(wheelOrientations[i].Right, Vector3.UnitX);
            Vector3 wheelUp = SafeNormalize(wheelOrientations[i].Up, Vector3.UnitY);
            Vector3 wheelForward = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), Vector3.UnitZ);

            Vector3 vel = wheelVelocities[i];
            float longVel = Vector3.Dot(vel, wheelForward);
            float latVel = Vector3.Dot(vel, wheelRight);

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

    /// <summary>
    /// Applies all computed tyre forces and self-aligning torques as impulses to the BEPU chassis body.
    /// Forces are applied at each wheel's contact point to generate appropriate moments.
    /// </summary>
    private void ApplyForces(
        BodyComponent chassisBody,
        Vector3 chassisPosition,
        ReadOnlySpan<Vector3> wheelPositions,
        ReadOnlySpan<Matrix> wheelOrientations,
        float dt)
    {
        for (int i = 0; i < WheelCount; i++)
        {
            if (!WheelGrounded[i])
                continue;

            float fx = LongitudinalForces[i];
            float fy = LateralForces[i];

            if (MathF.Abs(fx) < 0.01f && MathF.Abs(fy) < 0.01f)
                continue;

            // Build force vector in world space using wheel orientation
            Vector3 wheelRight = SafeNormalize(wheelOrientations[i].Right, Vector3.UnitX);
            Vector3 wheelUp = SafeNormalize(wheelOrientations[i].Up, Vector3.UnitY);
            Vector3 wheelForward = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), Vector3.UnitZ);

            Vector3 forceWorld = wheelForward * fx + wheelRight * fy;
            Vector3 impulse = forceWorld * dt;

            // Apply at wheel contact point to generate correct yaw/pitch/roll moments
            Vector3 offset = wheelPositions[i] - chassisPosition;
            chassisBody.ApplyImpulse(impulse, offset);
        }

        chassisBody.Awake = true;
    }

    /// <summary>
    /// Returns the total self-aligning torque from all wheels.
    /// This should be fed back to the steering system to provide force feedback.
    /// </summary>
    public float GetTotalSelfAligningTorque()
    {
        // Only front (steered) wheels contribute to steering feel
        return SelfAligningTorques[FL] + SelfAligningTorques[FR];
    }

    private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
    {
        float lengthSq = value.LengthSquared();
        if (lengthSq < 1e-6f)
            return fallback;
        return value / MathF.Sqrt(lengthSq);
    }
}
