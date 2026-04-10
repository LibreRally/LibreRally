using System;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Differential type enumeration.
/// Each type distributes engine torque differently between the two output shafts.
/// Reference: Milliken &amp; Milliken, "Race Car Vehicle Dynamics", §6.4.
/// </summary>
public enum DifferentialType : byte
{
    /// <summary>
    /// Open differential: equal torque split, allows unlimited speed difference.
    /// The faster-spinning wheel limits available traction — poor for rally.
    /// </summary>
    Open,

    /// <summary>
    /// Limited-slip differential: transfers torque based on wheel speed difference
    /// up to a bias ratio. Improves traction on mixed-grip surfaces.
    /// </summary>
    LimitedSlip,

    /// <summary>
    /// Locking differential: both wheels rotate at the same speed.
    /// Maximum traction on loose surfaces but reduces steering agility.
    /// </summary>
    Locking,
}

/// <summary>
/// Configuration for a single differential unit.
/// Used for front, rear, or center differentials in an AWD drivetrain.
/// </summary>
public struct DifferentialConfig
{
    /// <summary>Type of differential behaviour.</summary>
    public DifferentialType Type;

    /// <summary>
    /// Bias ratio for limited-slip differentials (typically 1.5–4.0).
    /// Defines the maximum torque ratio between the two output shafts:
    ///   T_slow / T_fast ≤ biasRatio.
    /// A value of 1.0 = open; higher = more locking tendency.
    /// Reference: Milliken, RCVD §6.4.2.
    /// </summary>
    public float BiasRatio;

    /// <summary>
    /// Locking coefficient for clutch-type LSDs (0.0–1.0).
    /// Controls how aggressively the LSD locks under acceleration.
    /// 0 = pure open behaviour, 1 = immediate full lock.
    /// </summary>
    public float LockingCoefficient;

    /// <summary>Creates a default open differential.</summary>
    public static DifferentialConfig CreateOpen() => new()
    {
        Type = DifferentialType.Open,
        BiasRatio = 1.0f,
        LockingCoefficient = 0f,
    };

    /// <summary>Creates a default limited-slip differential with typical rally bias.</summary>
    public static DifferentialConfig CreateLimitedSlip(float biasRatio = 2.5f, float lockingCoeff = 0.3f) => new()
    {
        Type = DifferentialType.LimitedSlip,
        BiasRatio = biasRatio,
        LockingCoefficient = lockingCoeff,
    };

    /// <summary>Creates a locking differential.</summary>
    public static DifferentialConfig CreateLocking() => new()
    {
        Type = DifferentialType.Locking,
        BiasRatio = float.MaxValue,
        LockingCoefficient = 1.0f,
    };
}

/// <summary>
/// Stateless differential torque-split calculations.
///
/// Torque path: engine → clutch → gearbox → center diff → front/rear diff → wheels.
///
/// Each differential splits its input torque to two outputs based on
/// wheel speed difference and the configured <see cref="DifferentialType"/>.
///
/// All methods are static and allocation-free for performance with 50+ vehicles.
///
/// Reference: Milliken &amp; Milliken, "Race Car Vehicle Dynamics", Chapter 6.
/// </summary>
public static class DifferentialSolver
{
    /// <summary>
    /// Splits <paramref name="inputTorque"/> into two output torques based on the
    /// differential configuration and the angular velocities of the two output shafts.
    /// </summary>
    /// <param name="config">Differential configuration.</param>
    /// <param name="inputTorque">Total torque entering the differential (N·m).</param>
    /// <param name="omegaLeft">Angular velocity of the left output shaft (rad/s).</param>
    /// <param name="omegaRight">Angular velocity of the right output shaft (rad/s).</param>
    /// <param name="torqueLeft">Output: torque delivered to the left shaft (N·m).</param>
    /// <param name="torqueRight">Output: torque delivered to the right shaft (N·m).</param>
    public static void SplitTorque(
        in DifferentialConfig config,
        float inputTorque,
        float omegaLeft,
        float omegaRight,
        out float torqueLeft,
        out float torqueRight)
    {
        switch (config.Type)
        {
            case DifferentialType.Open:
                SplitOpen(inputTorque, out torqueLeft, out torqueRight);
                break;

            case DifferentialType.LimitedSlip:
                SplitLimitedSlip(config, inputTorque, omegaLeft, omegaRight,
                    out torqueLeft, out torqueRight);
                break;

            case DifferentialType.Locking:
                SplitLocking(inputTorque, out torqueLeft, out torqueRight);
                break;

            default:
                SplitOpen(inputTorque, out torqueLeft, out torqueRight);
                break;
        }
    }

    /// <summary>
    /// Open differential: equal torque split (50/50).
    /// Each shaft receives half the input torque regardless of speed difference.
    /// This is the simplest and least effective for rally use.
    /// </summary>
    private static void SplitOpen(float inputTorque, out float torqueLeft, out float torqueRight)
    {
        torqueLeft = inputTorque * 0.5f;
        torqueRight = inputTorque * 0.5f;
    }

    /// <summary>
    /// Limited-slip differential: transfers additional torque to the slower wheel
    /// based on speed difference and bias ratio.
    ///
    /// <para>Model: clutch-type LSD. The friction clutch pack generates a locking torque
    /// proportional to the input torque and the speed difference:
    ///   T_lock = lockingCoeff × |T_input| × tanh(Δω / ω_ref)
    /// The locked torque is transferred from the faster to the slower wheel,
    /// clamped so the torque ratio does not exceed the bias ratio.</para>
    ///
    /// Reference: Milliken, RCVD §6.4.2, clutch-type LSD model.
    /// </summary>
    private static void SplitLimitedSlip(
        in DifferentialConfig config,
        float inputTorque,
        float omegaLeft,
        float omegaRight,
        out float torqueLeft,
        out float torqueRight)
    {
        // Start with 50/50 base split
        var half = inputTorque * 0.5f;
        torqueLeft = half;
        torqueRight = half;

        // Compute speed difference
        var deltaOmega = omegaLeft - omegaRight;
        if (MathF.Abs(deltaOmega) < 0.01f)
        {
	        return; // wheels at same speed — no locking torque needed
        }

        // Locking torque: proportional to input torque magnitude and speed difference.
        // tanh provides smooth onset and saturation.
        const float ReferenceOmega = 5f; // rad/s normalisation for tanh
        var lockingTorque = config.LockingCoefficient * MathF.Abs(inputTorque)
                                                      * MathF.Tanh(MathF.Abs(deltaOmega) / ReferenceOmega);

        // Transfer torque from the faster-spinning wheel to the slower one.
        // Clamp by bias ratio: T_slow / T_fast ≤ biasRatio.
        var maxTransfer = MathF.Abs(half) * MathF.Max(0f, config.BiasRatio - 1f);
        lockingTorque = MathF.Min(lockingTorque, maxTransfer);

        if (deltaOmega > 0f)
        {
            // Left is faster → transfer torque from left to right
            torqueLeft -= lockingTorque;
            torqueRight += lockingTorque;
        }
        else
        {
            // Right is faster → transfer torque from right to left
            torqueLeft += lockingTorque;
            torqueRight -= lockingTorque;
        }
    }

    /// <summary>
    /// Locking differential: equal speed, equal torque.
    /// Both output shafts forced to rotate at the same speed (rigid coupling).
    /// For torque purposes this is equivalent to a 50/50 split since
    /// the wheel speed equalisation is handled by the constraint system.
    /// </summary>
    private static void SplitLocking(float inputTorque, out float torqueLeft, out float torqueRight)
    {
        torqueLeft = inputTorque * 0.5f;
        torqueRight = inputTorque * 0.5f;
    }
}
