using System;
using Stride.BepuPhysics;
using Stride.Core.Mathematics;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Computes chassis body roll torque from left/right suspension compression differences.
///
/// <para>For each axle the compression delta between left and right wheels generates a
/// roll moment about the vehicle's longitudinal axis:
/// <code>
///   τ_roll = (compression_left − compression_right) × rollStiffness × trackWidth
/// </code>
/// The resulting torque is applied as an angular impulse to the chassis body each
/// physics step, producing realistic body lean during cornering.</para>
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

    /// <summary>
    /// Computes and applies chassis body roll torque from suspension compression differences.
    ///
    /// <para>Call once per physics step after suspension compressions have been measured
    /// and before tyre force application.</para>
    /// </summary>
    /// <param name="chassisBody">BEPU rigid body for the chassis.</param>
    /// <param name="chassisForward">Unit vector along the chassis longitudinal (forward) axis in world space.</param>
    /// <param name="suspensionCompressions">Per-wheel suspension compression (m). Indexed by <see cref="VehicleDynamicsSystem"/> wheel constants.</param>
    /// <param name="dt">Physics timestep in seconds.</param>
    public void Apply(
        BodyComponent chassisBody,
        Vector3 chassisForward,
        ReadOnlySpan<float> suspensionCompressions,
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
        // Positive delta (left more compressed) → positive torque → chassis rolls right.
        var rollTorque = deltaFront * FrontRollStiffness
                       + deltaRear * RearRollStiffness;

        // Viscous damping: oppose the current roll-rate component along the forward axis.
        if (RollDampingCoefficient > 0f)
        {
            var angularVelocity = chassisBody.AngularVelocity;
            var rollRate = Vector3.Dot(angularVelocity, chassisForward);
            rollTorque -= rollRate * RollDampingCoefficient;
        }

        if (MathF.Abs(rollTorque) < 1e-4f)
        {
            return;
        }

        // Convert torque to angular impulse: Δω = τ × dt, applied about the forward axis.
        var rollImpulse = chassisForward * (rollTorque * dt);

        chassisBody.Awake = true;
        chassisBody.ApplyAngularImpulse(rollImpulse);
    }
}
