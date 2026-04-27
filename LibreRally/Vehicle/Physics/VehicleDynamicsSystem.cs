using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using Stride.BepuPhysics;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;

namespace LibreRally.Vehicle.Physics
{
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
		private const float MinimumWakeForce = 0.01f;
		private const float TorqueConservationTolerance = 0.25f;
		private const int DiagnosticCooldownFrames = 120;
		private const float MaximumSuspensionVelocity = 12f;
		private const float SuspensionForceGuardMultiplier = 4.5f;
		private const float BumpStopStartFraction = 0.8f;
		private const float BumpStopProgressiveStiffnessMultiplier = 6f;
		private const float MinimumBumpStopTravel = 0.01f;
		private static readonly Logger Log = GlobalLogger.GetLogger("VehicleDynamicsSystem");

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
		public DifferentialConfig CenterDiff { get; set; } = DifferentialConfig.CreateLimitedSlip(1.8f);

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

		/// <summary>Suspension compression velocity per wheel (m/s). Positive = compressing into bump.</summary>
		public readonly float[] SuspensionVelocity = new float[WheelCount];

		/// <summary>Hooke-law spring support force per wheel (N). Positive values increase tyre normal load.</summary>
		public readonly float[] SpringForces = new float[WheelCount];

		/// <summary>Directional damper support force per wheel (N). Positive values oppose bump, negative oppose rebound.</summary>
		public readonly float[] DamperForces = new float[WheelCount];

		/// <summary>Progressive bump-stop support force per wheel (N). Only active near maximum compression.</summary>
		public readonly float[] BumpStopForces = new float[WheelCount];

		/// <summary>Whether each wheel currently has ground contact.</summary>
		public readonly bool[] WheelGrounded = new bool[WheelCount];

		/// <summary>Per-wheel spring stiffness (N/m), used when translating suspension compression into tyre load.</summary>
		public readonly float[] SpringStiffness = new float[WheelCount];

		/// <summary>Maximum bump/compression travel per wheel (m).</summary>
		public readonly float[] SuspensionMaximumCompression = new float[WheelCount];

		/// <summary>Maximum droop/rebound travel magnitude per wheel (m).</summary>
		public readonly float[] SuspensionMaximumDroop = new float[WheelCount];

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
		private readonly float[] _averageDamperForces = new float[WheelCount];

		/// <summary>Reusable snapshot buffer so the predictor pass can evaluate tyre forces without advancing wheel state twice.</summary>
		private readonly TyreState[] _predictionWheelStates = new TyreState[WheelCount];

		// ── Cached per-frame outputs ─────────────────────────────────────────────

		/// <summary>Longitudinal force per wheel (N). Read by telemetry.</summary>
		public readonly float[] LongitudinalForces = new float[WheelCount];

		/// <summary>Lateral force per wheel (N). Read by telemetry.</summary>
		public readonly float[] LateralForces = new float[WheelCount];

		/// <summary>Self-aligning torque per wheel (N·m). Applied to steering system.</summary>
		public readonly float[] SelfAligningTorques = new float[WheelCount];

		/// <summary>Overturning couple per wheel (N·m) about the wheel forward axis.</summary>
		public readonly float[] OverturningCouples = new float[WheelCount];

		/// <summary>Rolling-resistance moment per wheel (N·m) about the wheel right axis.</summary>
		public readonly float[] RollingResistanceMoments = new float[WheelCount];

		/// <summary>Effective per-wheel peak friction coefficient after surface/load/temperature modifiers.</summary>
		public readonly float[] EffectivePeakFrictionCoefficients = new float[WheelCount];

		/// <summary>Drive torque delivered to each wheel after differential (N·m).</summary>
		public readonly float[] WheelDriveTorques = new float[WheelCount];
		private readonly float[] _wheelAngularDriveTorques = new float[WheelCount];

		/// <summary>Brake torque applied to each wheel this frame (N·m).</summary>
		public readonly float[] WheelBrakeTorques = new float[WheelCount];

		/// <summary>Tyre reaction torque opposing wheel rotation (N·m).</summary>
		public readonly float[] WheelTyreReactionTorques = new float[WheelCount];

		/// <summary>Total drivetrain torque requested at the wheels before traction limiting (N·m).</summary>
		public float RequestedDrivetrainTorque { get; private set; }

		/// <summary>Total drivetrain torque delivered to the wheel shafts after differential limits (N·m).</summary>
		public float DeliveredDrivetrainTorque { get; private set; }

		/// <summary>Signed undelivered drivetrain torque caused by traction or differential limits (N·m), with the same sign as <see cref="RequestedDrivetrainTorque"/>.</summary>
		public float DrivetrainTorqueShortfall { get; private set; }

		/// <summary>Delivered front-axle drive torque after the center differential (N·m).</summary>
		public float FrontAxleDeliveredTorque { get; private set; }

		/// <summary>Delivered rear-axle drive torque after the center differential (N·m).</summary>
		public float RearAxleDeliveredTorque { get; private set; }

		// ── Force-derived acceleration state (cached for next step) ──────────────
		private Vector3 _forceEstimatedAccelerationWorld;
		private bool _hasForceEstimatedAcceleration;
		private int _drivetrainDiagnosticCooldown;

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
		/// <param name="wheelContactPoints">World-space tyre contact patch position for each wheel.</param>
		/// <param name="suspensionAttachmentPoints">World-space chassis-side suspension attachment point for each wheel.</param>
		/// <param name="suspensionAxes">World-space suspension axis for each wheel. Positive travel points toward bump/compression.</param>
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
			ReadOnlySpan<Vector3> wheelContactPoints,
			ReadOnlySpan<Vector3> suspensionAttachmentPoints,
			ReadOnlySpan<Vector3> suspensionAxes,
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

			PrepareSuspensionState(suspensionCompressions, dt);

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

			// ── Chassis body attitude torque from suspension compression and sprung-mass acceleration ─
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
				SuspensionCompression,
				rollAccelerationTorque,
				pitchAccelerationTorque,
				dt);

			_forceEstimatedAccelerationWorld = instantaneousAccelerationWorld;
			_hasForceEstimatedAcceleration = true;

			// ── 7. Apply chassis impulses after the force prediction is complete ──
			// The damping correction mutates chassisBody directly, so it must run after the tyre-force
			// prediction passes that used the current wheel-velocity snapshot.
			ApplyAsymmetricDampingCorrection(chassisBody, in chassisWorld, suspensionAttachmentPoints, suspensionAxes, dt);

			// ── 8. Apply all forces as impulses to the chassis at the wheel contact locations ─
			ApplyForces(chassisBody, in chassisWorld, wheelContactPoints, wheelOrientations, dt);
		}

		private void PrepareSuspensionState(ReadOnlySpan<float> suspensionCompressions, float dt)
		{
			for (var i = 0; i < WheelCount; i++)
			{
				var maxCompression = MathF.Max(SuspensionMaximumCompression[i], 0f);
				var maxDroop = MathF.Max(SuspensionMaximumDroop[i], 0f);
				var rawCompression = i < suspensionCompressions.Length ? suspensionCompressions[i] : 0f;

				// Compression sign convention used everywhere in vehicle dynamics:
				// positive = wheel moved toward the chassis (bump), negative = droop/rebound.
				var clampedCompression = float.IsFinite(rawCompression)
					? Math.Clamp(rawCompression, -maxDroop, maxCompression)
					: 0f;
				SuspensionCompression[i] = clampedCompression;

				var velocity = dt > 1e-6f
					? (clampedCompression - _previousSuspensionCompression[i]) / dt
					: 0f;
				velocity = float.IsFinite(velocity)
					? Math.Clamp(velocity, -MaximumSuspensionVelocity, MaximumSuspensionVelocity)
					: 0f;
				SuspensionVelocity[i] = velocity;
				_previousSuspensionCompression[i] = clampedCompression;

				var springStiffness = MathF.Max(SpringStiffness[i], 0f);
				var springCompression = Math.Clamp(clampedCompression, 0f, maxCompression);
				var springForceLimit = ComputeSuspensionForceLimit(i, springStiffness, maxCompression);
				var springForce = springStiffness * springCompression;
				var bumpStopForce = ComputeBumpStopForce(i, springCompression, maxCompression, springStiffness, springForceLimit);
				var directionDamping = velocity >= 0f
					? MathF.Max(BumpDamping[i], 0f)
					: MathF.Max(ReboundDamping[i], 0f);
				var averageDamping = MathF.Max(BepuAverageDamping[i], 0f);
				var damperForce = directionDamping * velocity;
				var averageDamperForce = averageDamping * velocity;

				SpringForces[i] = SanitizeForce(springForce, springForceLimit);
				BumpStopForces[i] = SanitizeForce(bumpStopForce, springForceLimit);
				DamperForces[i] = SanitizeSignedForce(damperForce, springForceLimit);
				_averageDamperForces[i] = SanitizeSignedForce(averageDamperForce, springForceLimit);
			}
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
				WheelGrounded[i] = wheelGrounded[i];
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

			if (Vector128.IsHardwareAccelerated)
			{
				ref var curRef = ref MemoryMarshal.GetArrayDataReference(CurrentNormalLoads);
				ref var springRef = ref MemoryMarshal.GetArrayDataReference(SpringForces);
				ref var damperRef = ref MemoryMarshal.GetArrayDataReference(DamperForces);
				ref var bumpRef = ref MemoryMarshal.GetArrayDataReference(BumpStopForces);
				var result = Vector128.LoadUnsafe(ref curRef) +
				             Vector128.LoadUnsafe(ref springRef) +
				             Vector128.LoadUnsafe(ref damperRef) +
				             Vector128.LoadUnsafe(ref bumpRef);
				result.StoreUnsafe(ref curRef);
			}
			else
			{
				for (var i = 0; i < WheelCount; i++)
				{
					CurrentNormalLoads[i] += SpringForces[i] + DamperForces[i] + BumpStopForces[i];
				}
			}

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
					CurrentNormalLoads[i] = Math.Clamp(CurrentNormalLoads[i], 0f, ComputeNormalLoadLimit(i)) * contactScale;
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
		private void ComputeAntiRollForces()
		{
			// Front axle anti-roll bar
			if (FrontAntiRollStiffness > 0f)
			{
				ApplyAntiRollLoadTransfer(FL, FR, FrontAntiRollStiffness);
			}

			// Rear axle anti-roll bar
			if (RearAntiRollStiffness > 0f)
			{
				ApplyAntiRollLoadTransfer(RL, RR, RearAntiRollStiffness);
			}

			// Re-clamp after anti-roll adjustments
			for (var i = 0; i < WheelCount; i++)
				CurrentNormalLoads[i] = Math.Clamp(CurrentNormalLoads[i], 0f, ComputeNormalLoadLimit(i));
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
			ReadOnlySpan<Vector3> suspensionAttachmentPoints,
			ReadOnlySpan<Vector3> suspensionAxes,
			float dt)
		{
			var chassisPosition = chassisWorld.TranslationVector;

			for (var i = 0; i < WheelCount; i++)
			{
				if (!WheelGrounded[i])
				{
					continue;
				}

				var correctionForce = DamperForces[i] - _averageDamperForces[i];

				// Clamp correction to avoid extreme transients (e.g. first frame or teleport)
				var maxCorrectionForce = ComputeSuspensionForceLimit(i, SpringStiffness[i], SuspensionMaximumCompression[i]);
				correctionForce = Math.Clamp(correctionForce, -maxCorrectionForce, maxCorrectionForce);

				if (MathF.Abs(correctionForce) < 1f)
					continue;

				var suspensionAxis = i < suspensionAxes.Length
					? SafeNormalize(suspensionAxes[i], Vector3.UnitY)
					: Vector3.UnitY;
				var impulse = suspensionAxis * (correctionForce * dt);
				var attachmentPoint = i < suspensionAttachmentPoints.Length
					? suspensionAttachmentPoints[i]
					: chassisPosition;
				var contactOffset = attachmentPoint - chassisPosition;

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
				_wheelAngularDriveTorques[i] = 0f;
			}

			RequestedDrivetrainTorque = engineTorqueAtWheels;
			DeliveredDrivetrainTorque = 0f;
			DrivetrainTorqueShortfall = 0f;
			FrontAxleDeliveredTorque = 0f;
			RearAxleDeliveredTorque = 0f;

			Span<float> wheelTractionLimits = stackalloc float[WheelCount];
			for (var i = 0; i < WheelCount; i++)
			{
				wheelTractionLimits[i] = ComputeWheelTractionLimit(i);
			}

			if (!DriveFrontAxle && !DriveRearAxle)
			{
				return;
			}

			ComputeWheelAngularDriveTorque(engineTorqueAtWheels);

			if (DriveFrontAxle && !DriveRearAxle)
			{
				var frontDiffOnly = FrontDiff;
				DifferentialSolver.SplitTorque(in frontDiffOnly, engineTorqueAtWheels,
					WheelStates[FL].AngularVelocity, WheelStates[FR].AngularVelocity,
					wheelTractionLimits[FL], wheelTractionLimits[FR],
					out WheelDriveTorques[FL], out WheelDriveTorques[FR]);
				FrontAxleDeliveredTorque = WheelDriveTorques[FL] + WheelDriveTorques[FR];
				DeliveredDrivetrainTorque = FrontAxleDeliveredTorque;
				DrivetrainTorqueShortfall = ComputeTorqueShortfall(RequestedDrivetrainTorque, DeliveredDrivetrainTorque);
				ValidateDrivetrainTorque("front axle", RequestedDrivetrainTorque, DeliveredDrivetrainTorque);
				return;
			}

			if (!DriveFrontAxle)
			{
				var rearDiffOnly = RearDiff;
				DifferentialSolver.SplitTorque(in rearDiffOnly, engineTorqueAtWheels,
					WheelStates[RL].AngularVelocity, WheelStates[RR].AngularVelocity,
					wheelTractionLimits[RL], wheelTractionLimits[RR],
					out WheelDriveTorques[RL], out WheelDriveTorques[RR]);
				RearAxleDeliveredTorque = WheelDriveTorques[RL] + WheelDriveTorques[RR];
				DeliveredDrivetrainTorque = RearAxleDeliveredTorque;
				DrivetrainTorqueShortfall = ComputeTorqueShortfall(RequestedDrivetrainTorque, DeliveredDrivetrainTorque);
				ValidateDrivetrainTorque("rear axle", RequestedDrivetrainTorque, DeliveredDrivetrainTorque);
				return;
			}

			// Center diff splits to front and rear axles
			var omegaFrontAxle = (WheelStates[FL].AngularVelocity + WheelStates[FR].AngularVelocity) * 0.5f;
			var omegaRearAxle = (WheelStates[RL].AngularVelocity + WheelStates[RR].AngularVelocity) * 0.5f;
			var frontDiffForCapacity = FrontDiff;
			var rearDiffForCapacity = RearDiff;
			var frontAxleTractionLimit = DifferentialSolver.ComputeMaximumDeliveredTorque(
				in frontDiffForCapacity,
				WheelStates[FL].AngularVelocity,
				WheelStates[FR].AngularVelocity,
				wheelTractionLimits[FL],
				wheelTractionLimits[FR]);
			var rearAxleTractionLimit = DifferentialSolver.ComputeMaximumDeliveredTorque(
				in rearDiffForCapacity,
				WheelStates[RL].AngularVelocity,
				WheelStates[RR].AngularVelocity,
				wheelTractionLimits[RL],
				wheelTractionLimits[RR]);

			var centerDiff = CenterDiff;
			DifferentialSolver.SplitTorque(in centerDiff, engineTorqueAtWheels,
				omegaFrontAxle, omegaRearAxle,
				frontAxleTractionLimit, rearAxleTractionLimit,
				out var frontAxleTorque, out var rearAxleTorque);
			FrontAxleDeliveredTorque = frontAxleTorque;
			RearAxleDeliveredTorque = rearAxleTorque;

			// Front diff splits between FL and FR
			var frontDiff = FrontDiff;
			DifferentialSolver.SplitTorque(in frontDiff, frontAxleTorque,
				WheelStates[FL].AngularVelocity, WheelStates[FR].AngularVelocity,
				wheelTractionLimits[FL], wheelTractionLimits[FR],
				out var torqueFL, out var torqueFR);
			WheelDriveTorques[FL] = torqueFL;
			WheelDriveTorques[FR] = torqueFR;

			// Rear diff splits between RL and RR
			var rearDiff = RearDiff;
			DifferentialSolver.SplitTorque(in rearDiff, rearAxleTorque,
				WheelStates[RL].AngularVelocity, WheelStates[RR].AngularVelocity,
				wheelTractionLimits[RL], wheelTractionLimits[RR],
				out var torqueRL, out var torqueRR);
			WheelDriveTorques[RL] = torqueRL;
			WheelDriveTorques[RR] = torqueRR;

			DeliveredDrivetrainTorque = torqueFL + torqueFR + torqueRL + torqueRR;
			DrivetrainTorqueShortfall = ComputeTorqueShortfall(RequestedDrivetrainTorque, DeliveredDrivetrainTorque);
			ValidateDrivetrainTorque("center diff", RequestedDrivetrainTorque, frontAxleTorque + rearAxleTorque);
			ValidateDrivetrainTorque("front diff", frontAxleTorque, torqueFL + torqueFR);
			ValidateDrivetrainTorque("rear diff", rearAxleTorque, torqueRL + torqueRR);
			ValidateDrivetrainTorque("wheel sum", frontAxleTorque + rearAxleTorque, DeliveredDrivetrainTorque);
		}

		/// <summary>
		/// Evaluates the tyre model for each wheel, computing tyre forces plus bounded moment terms.
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
					OverturningCouples[i] = 0f;
					RollingResistanceMoments[i] = 0f;
					EffectivePeakFrictionCoefficients[i] = 0f;
					WheelBrakeTorques[i] = brakeTorques[i];
					WheelTyreReactionTorques[i] = 0f;
					continue;
				}

				WheelBrakeTorques[i] = brakeTorques[i];

				if (!WheelGrounded[i])
				{
					LongitudinalForces[i] = 0f;
					LateralForces[i] = 0f;
					SelfAligningTorques[i] = 0f;
					OverturningCouples[i] = 0f;
					RollingResistanceMoments[i] = 0f;
					EffectivePeakFrictionCoefficients[i] = 0f;
					WheelStates[i].LateralDeflection = 0f;

					tyreModel.Update(
						ref WheelStates[i],
						0f, 0f, 0f,
						_wheelAngularDriveTorques[i],
						brakeTorques[i],
						0f,
						in WheelSurfaces[i],
						dt,
						out _, out _, out _);
					WheelTyreReactionTorques[i] = WheelStates[i].TyreReactionTorque;
					EffectivePeakFrictionCoefficients[i] = 0f;
					continue;
				}

				ResolveWheelBasis(in wheelOrientations[i], out var wheelRight, out _, out var wheelForward);

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
					out SelfAligningTorques[i],
					out OverturningCouples[i],
					out RollingResistanceMoments[i]);
				WheelTyreReactionTorques[i] = WheelStates[i].TyreReactionTorque;
				EffectivePeakFrictionCoefficients[i] = ComputeWheelEffectivePeakFriction(tyreModel, i, MathF.Abs(longVel));
			}
		}

		private float ComputeWheelEffectivePeakFriction(TyreModel tyreModel, int wheelIndex, float absLongitudinalVelocity)
		{
			var normalLoad = CurrentNormalLoads[wheelIndex];
			if (normalLoad <= 1e-3f)
			{
				return 0f;
			}

			var wheelState = WheelStates[wheelIndex];
			var gripTemperatureWeight = Math.Clamp(tyreModel.GripTemperatureSurfaceWeight, 0f, 1f);
			var gripTemperature = gripTemperatureWeight * wheelState.Temperature
			                      + (1f - gripTemperatureWeight) * wheelState.CoreTemperature;
			return tyreModel.ComputeEffectiveFriction(
				normalLoad,
				in WheelSurfaces[wheelIndex],
				gripTemperature,
				wheelState.TreadLife,
				absLongitudinalVelocity);
		}

		private float ComputeWheelTractionLimit(int wheelIndex)
		{
			if (!WheelGrounded[wheelIndex])
			{
				return 0f;
			}

			var tyreModel = TyreModels[wheelIndex];
			if (tyreModel == null)
			{
				return 0f;
			}

			var normalLoad = CurrentNormalLoads[wheelIndex];
			if (normalLoad <= 1e-3f)
			{
				return 0f;
			}

			var wheelState = WheelStates[wheelIndex];
			var gripTemperatureWeight = Math.Clamp(tyreModel.GripTemperatureSurfaceWeight, 0f, 1f);
			var gripTemperature = gripTemperatureWeight * wheelState.Temperature
			                      + (1f - gripTemperatureWeight) * wheelState.CoreTemperature;
			var rollingRadius = tyreModel.ComputeEffectiveRollingRadius(normalLoad);
			var referenceSpeed = MathF.Abs(wheelState.AngularVelocity) * rollingRadius;
			var friction = tyreModel.ComputeEffectiveFriction(
				normalLoad,
				in WheelSurfaces[wheelIndex],
				gripTemperature,
				wheelState.TreadLife,
				referenceSpeed);
			return MathF.Max(0f, friction * normalLoad * rollingRadius);
		}

		private void ComputeWheelAngularDriveTorque(float engineTorqueAtWheels)
		{
			if (DriveFrontAxle && !DriveRearAxle)
			{
				var frontDiffOnly = FrontDiff;
				DifferentialSolver.SplitTorque(
					in frontDiffOnly,
					engineTorqueAtWheels,
					WheelStates[FL].AngularVelocity,
					WheelStates[FR].AngularVelocity,
					out _wheelAngularDriveTorques[FL],
					out _wheelAngularDriveTorques[FR]);
				return;
			}

			if (!DriveFrontAxle)
			{
				var rearDiffOnly = RearDiff;
				DifferentialSolver.SplitTorque(
					in rearDiffOnly,
					engineTorqueAtWheels,
					WheelStates[RL].AngularVelocity,
					WheelStates[RR].AngularVelocity,
					out _wheelAngularDriveTorques[RL],
					out _wheelAngularDriveTorques[RR]);
				return;
			}

			var omegaFrontAxle = (WheelStates[FL].AngularVelocity + WheelStates[FR].AngularVelocity) * 0.5f;
			var omegaRearAxle = (WheelStates[RL].AngularVelocity + WheelStates[RR].AngularVelocity) * 0.5f;

			var centerDiff = CenterDiff;
			DifferentialSolver.SplitTorque(
				in centerDiff,
				engineTorqueAtWheels,
				omegaFrontAxle,
				omegaRearAxle,
				out var frontAxleTorque,
				out var rearAxleTorque);

			var frontDiff = FrontDiff;
			DifferentialSolver.SplitTorque(
				in frontDiff,
				frontAxleTorque,
				WheelStates[FL].AngularVelocity,
				WheelStates[FR].AngularVelocity,
				out _wheelAngularDriveTorques[FL],
				out _wheelAngularDriveTorques[FR]);

			var rearDiff = RearDiff;
			DifferentialSolver.SplitTorque(
				in rearDiff,
				rearAxleTorque,
				WheelStates[RL].AngularVelocity,
				WheelStates[RR].AngularVelocity,
				out _wheelAngularDriveTorques[RL],
				out _wheelAngularDriveTorques[RR]);
		}

		private void ValidateDrivetrainTorque(string stage, float inputTorque, float outputTorque)
		{
			var inputMagnitude = MathF.Abs(inputTorque);
			var outputMagnitude = MathF.Abs(outputTorque);
			var torqueError = outputMagnitude - inputMagnitude;
			if (torqueError <= TorqueConservationTolerance)
			{
				DecrementDrivetrainDiagnosticCooldown();
				return;
			}

			if (_drivetrainDiagnosticCooldown > 0)
			{
				DecrementDrivetrainDiagnosticCooldown();
				return;
			}

			Log.Warning($"[VehicleDynamicsSystem] Drivetrain torque mismatch at {stage}: in={inputTorque:F2}Nm out={outputTorque:F2}Nm err={outputTorque - inputTorque:F2}Nm");
			_drivetrainDiagnosticCooldown = DiagnosticCooldownFrames;
		}

		private static float ComputeTorqueShortfall(float requestedTorque, float deliveredTorque)
		{
			var shortfall = MathF.Max(0f, MathF.Abs(requestedTorque) - MathF.Abs(deliveredTorque));
			return MathF.CopySign(shortfall, requestedTorque);
		}

		private void DecrementDrivetrainDiagnosticCooldown()
		{
			if (_drivetrainDiagnosticCooldown > 0)
			{
				_drivetrainDiagnosticCooldown--;
			}
		}

		private Vector3 RunForcePredictionPass(
			ReadOnlySpan<bool> wheelGrounded,
			ReadOnlySpan<float> wheelContactScales,
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
			ComputeAntiRollForces();
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

				ResolveWheelBasis(in wheelOrientations[i], out var wheelRight, out _, out var wheelForward);
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
			ReadOnlySpan<Vector3> wheelContactPoints,
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
				var overturningCouple = OverturningCouples[i];
				var rollingResistanceMoment = RollingResistanceMoments[i];

				if (MathF.Abs(fx) < MinimumWakeForce
				    && MathF.Abs(fy) < MinimumWakeForce
				    && MathF.Abs(overturningCouple) < 1e-3f
				    && MathF.Abs(rollingResistanceMoment) < 1e-3f)
				{
					continue;
				}

				// Build force vector in world space using wheel orientation
				ResolveWheelBasis(in wheelOrientations[i], out var wheelRight, out _, out var wheelForward);

				var forceWorld = wheelForward * fx + wheelRight * fy;
				var impulse = forceWorld * dt;
				var momentImpulse = ComputeWheelMomentImpulseWorld(
					wheelForward,
					wheelRight,
					overturningCouple,
					rollingResistanceMoment,
					dt);
				var contactPatchPosition = i < wheelContactPoints.Length
					? wheelContactPoints[i]
					: chassisPosition;
				var contactOffset = contactPatchPosition - chassisPosition;

				chassisBody.Awake = true;
				chassisBody.ApplyImpulse(impulse, contactOffset);
				if (momentImpulse.LengthSquared() > 1e-8f)
				{
					chassisBody.ApplyAngularImpulse(momentImpulse);
				}
			}
		}

		internal static Vector3 ComputeWheelMomentImpulseWorld(
			in Vector3 wheelForward,
			in Vector3 wheelRight,
			float overturningCouple,
			float rollingResistanceMoment,
			float dt)
		{
			return wheelForward * (overturningCouple * dt) + wheelRight * (rollingResistanceMoment * dt);
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

		private void ApplyAntiRollLoadTransfer(int leftIndex, int rightIndex, float antiRollStiffness)
		{
			var deltaCompression = SuspensionCompression[leftIndex] - SuspensionCompression[rightIndex];
			var requestedTransfer = deltaCompression * antiRollStiffness;
			var availableToRight = MathF.Max(CurrentNormalLoads[leftIndex], 0f);
			var availableToLeft = MathF.Max(CurrentNormalLoads[rightIndex], 0f);
			var clampedTransfer = Math.Clamp(requestedTransfer, -availableToRight, availableToLeft);

			CurrentNormalLoads[leftIndex] += clampedTransfer;
			CurrentNormalLoads[rightIndex] -= clampedTransfer;
		}

		private float ComputeNormalLoadLimit(int wheelIndex)
		{
			return MathF.Max(StaticNormalLoads[wheelIndex] * SuspensionForceGuardMultiplier,
				StaticNormalLoads[wheelIndex] + ComputeSuspensionForceLimit(wheelIndex, SpringStiffness[wheelIndex], SuspensionMaximumCompression[wheelIndex]));
		}

		private float ComputeSuspensionForceLimit(int wheelIndex, float springStiffness, float maxCompression)
		{
			var effectiveCompression = MathF.Max(maxCompression, 0.05f);
			var springForceAtLimit = MathF.Max(springStiffness, 1f) * effectiveCompression;
			return MathF.Max(StaticNormalLoads[wheelIndex] * SuspensionForceGuardMultiplier, springForceAtLimit * 1.5f);
		}

		private float ComputeBumpStopForce(int wheelIndex, float springCompression, float maxCompression, float springStiffness, float forceLimit)
		{
			if (maxCompression <= 1e-4f || springCompression <= 0f)
			{
				return 0f;
			}

			var bumpStopStart = maxCompression * BumpStopStartFraction;
			if (springCompression <= bumpStopStart)
			{
				return 0f;
			}

			// Keep a small minimum travel so the progressive stop stays well-defined even on
			// short-travel setups or data with nearly coincident limit values.
			var bumpStopTravel = MathF.Max(maxCompression - bumpStopStart, MinimumBumpStopTravel);
			var bumpStopCompression = springCompression - bumpStopStart;
			var normalizedCompression = Math.Clamp(bumpStopCompression / bumpStopTravel, 0f, 1f);
			var baseStiffness = springStiffness > 1f
				? springStiffness
				: MathF.Max(StaticNormalLoads[wheelIndex] / MathF.Max(maxCompression, 0.05f), 1f);
			var progressiveStiffness = baseStiffness * (1f + BumpStopProgressiveStiffnessMultiplier * normalizedCompression * normalizedCompression);
			return Math.Clamp(progressiveStiffness * bumpStopCompression, 0f, forceLimit);
		}

		private static float SanitizeForce(float value, float maxMagnitude)
		{
			if (!float.IsFinite(value))
			{
				return 0f;
			}

			return Math.Clamp(value, 0f, MathF.Max(maxMagnitude, 0f));
		}

		private static float SanitizeSignedForce(float value, float maxMagnitude)
		{
			if (!float.IsFinite(value))
			{
				return 0f;
			}

			var clampedMagnitude = MathF.Max(maxMagnitude, 0f);
			return Math.Clamp(value, -clampedMagnitude, clampedMagnitude);
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

		private static void ResolveWheelBasis(
			in Matrix wheelOrientation,
			out Vector3 wheelRight,
			out Vector3 wheelUp,
			out Vector3 wheelForward)
		{
			// Stride's local +Z axis maps to Matrix.Backward.
			wheelForward = SafeNormalize(wheelOrientation.Backward, Vector3.UnitZ);
			wheelRight = SafeNormalize(wheelOrientation.Right, Vector3.UnitX);

			// Re-orthonormalize to keep force directions stable under small matrix skew.
			wheelRight = SafeNormalize(
				wheelRight - wheelForward * Vector3.Dot(wheelRight, wheelForward),
				Vector3.UnitX);
			wheelUp = SafeNormalize(Vector3.Cross(wheelForward, wheelRight), Vector3.UnitY);
		}
	}
}
