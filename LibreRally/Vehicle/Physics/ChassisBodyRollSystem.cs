using System;
using Stride.BepuPhysics;
using Stride.Core.Mathematics;

namespace LibreRally.Vehicle.Physics
{
	/// <summary>
	/// Computes chassis body attitude torque from suspension compression differences.
	///
	/// <para>For each axle the compression delta between left and right wheels generates a
	/// roll moment about the vehicle's longitudinal axis:
	/// <code>
	///   τ_roll = (compression_left − compression_right) × rollStiffness
	/// </code>
	/// The average front-versus-rear compression difference also generates a pitch moment
	/// about the vehicle's lateral axis:
	/// <code>
	///   τ_pitch = (avgFrontCompression − avgRearCompression) × pitchStiffness
	/// </code>
	/// The resulting torques are applied as angular impulses to the chassis body each
	/// physics step, producing visible body lean and dive/squat from the measured suspension
	/// state rather than from arbitrary visual offsets.</para>
	///
	/// <para>This is a reduced-order, torque-based body attitude model. It approximates the
	/// sprung-mass response from compression and acceleration terms and does not model full
	/// multi-body suspension kinematics (control-arm geometry, instant centers, etc.).</para>
	///
	/// <para>Design constraints:
	/// <list type="bullet">
	///   <item>Zero per-frame allocations — struct with no heap state.</item>
	///   <item>Fixed-timestep safe — torque is converted to an impulse using the caller's dt.</item>
	///   <item>Compatible with <see cref="VehicleDynamicsSystem"/> suspension compression arrays.</item>
	/// </list></para>
	///
	/// <para>Reference: Milliken &amp; Milliken, "Race Car Vehicle Dynamics", §17 — Roll stiffness
	/// and suspension geometry.</para>
	/// </summary>
	public struct ChassisBodyRollSystem
	{
		/// <summary>Front axle roll stiffness (N·m/m). Torque per metre of left-right compression difference.</summary>
		public float FrontRollStiffness;

		/// <summary>Rear axle roll stiffness (N·m/m). Torque per metre of left-right compression difference.</summary>
		public float RearRollStiffness;

		/// <summary>Viscous roll damping coefficient (N·m·s/rad). Resists rate of roll change to prevent oscillation.</summary>
		public float RollDampingCoefficient;

		/// <summary>Pitch stiffness (N·m/m). Torque per metre of front-vs-rear average compression difference.</summary>
		public float PitchStiffness;

		/// <summary>Viscous pitch damping coefficient (N·m·s/rad). Resists dive/squat oscillation.</summary>
		public float PitchDampingCoefficient;

		/// <summary>
		/// Computes and applies chassis body roll torque from suspension compression differences.
		///
		/// <para>Call once per physics step after suspension compressions have been measured
		/// and before tyre force application.</para>
		/// </summary>
		/// <param name="chassisBody">BEPU rigid body for the chassis.</param>
		/// <param name="chassisForward">Unit vector along the chassis longitudinal (forward) axis in world space.</param>
		/// <param name="chassisRight">Unit vector along the chassis lateral (right) axis in world space.</param>
		/// <param name="suspensionCompressions">Per-wheel suspension compression (m). Indexed by <see cref="VehicleDynamicsSystem"/> wheel constants.</param>
		/// <param name="rollAccelerationTorque">Additional roll moment (N·m) from lateral acceleration/load transfer of the sprung mass.</param>
		/// <param name="pitchAccelerationTorque">Additional pitch moment (N·m) from longitudinal acceleration/load transfer of the sprung mass.</param>
		/// <param name="dt">Physics timestep in seconds.</param>
		public void Apply(
			BodyComponent chassisBody,
			Vector3 chassisForward,
			Vector3 chassisRight,
			ReadOnlySpan<float> suspensionCompressions,
			float rollAccelerationTorque,
			float pitchAccelerationTorque,
			float dt)
		{
			if (dt < 1e-6f)
			{
				return;
			}

			// Front axle: FL − FR compression difference
			var deltaFront = suspensionCompressions[VehicleDynamicsSystem.FL]
			                 - suspensionCompressions[VehicleDynamicsSystem.FR];

			// Rear axle: RL − RR compression difference
			var deltaRear = suspensionCompressions[VehicleDynamicsSystem.RL]
			                - suspensionCompressions[VehicleDynamicsSystem.RR];

			// Total roll torque about the longitudinal axis (N·m).
			// Positive delta (left more compressed) → positive torque along +forward (+Z) → chassis rolls left.
			var rollTorque = deltaFront * FrontRollStiffness
			                 + deltaRear * RearRollStiffness
			                 + rollAccelerationTorque;

			// Viscous damping: oppose the current roll-rate component along the forward axis.
			if (RollDampingCoefficient > 0f)
			{
				var angularVelocity = chassisBody.AngularVelocity;
				var rollRate = Vector3.Dot(angularVelocity, chassisForward);
				rollTorque -= rollRate * RollDampingCoefficient;
			}

			var frontAverageCompression = (suspensionCompressions[VehicleDynamicsSystem.FL]
			                               + suspensionCompressions[VehicleDynamicsSystem.FR]) * 0.5f;
			var rearAverageCompression = (suspensionCompressions[VehicleDynamicsSystem.RL]
			                              + suspensionCompressions[VehicleDynamicsSystem.RR]) * 0.5f;
			var pitchTorque = (frontAverageCompression - rearAverageCompression) * PitchStiffness
			                  + pitchAccelerationTorque;

			if (PitchDampingCoefficient > 0f)
			{
				var angularVelocity = chassisBody.AngularVelocity;
				var pitchRate = Vector3.Dot(angularVelocity, chassisRight);
				pitchTorque -= pitchRate * PitchDampingCoefficient;
			}

			if (MathF.Abs(rollTorque) < 1e-4f && MathF.Abs(pitchTorque) < 1e-4f)
			{
				return;
			}

			// Convert torque to angular impulse (L = τ × dt). Roll acts about the chassis
			// forward axis, and positive pitch torque acts about the right axis so that
			// greater front compression produces a nose-down response.
			var rollImpulse = chassisForward * (rollTorque * dt);
			var pitchImpulse = chassisRight * (pitchTorque * dt);

			chassisBody.Awake = true;
			chassisBody.ApplyAngularImpulse(rollImpulse + pitchImpulse);
		}
	}
}
