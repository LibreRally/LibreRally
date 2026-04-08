using System;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Identifies a driving surface material.
/// Each type defines a unique set of friction, resistance, and deformation properties
/// that modify tyre grip and slip behaviour.
///
/// Reference: Pacejka, "Tire and Vehicle Dynamics", Chapter 4 — road surface effects.
/// </summary>
public enum SurfaceType : byte
{
    Tarmac,
    Gravel,
    Mud,
    Snow,
    Ice,
}

/// <summary>
/// Physics properties for a driving surface.
/// These values scale the tyre model's grip envelope and energy dissipation.
///
/// <list type="bullet">
///   <item><see cref="FrictionCoefficient"/>: peak µ multiplier (1.0 = reference tarmac).</item>
///   <item><see cref="RollingResistance"/>: rolling-resistance force coefficient (N per N of load).</item>
///   <item><see cref="DeformationFactor"/>: how much the surface deforms under load (0 = rigid, 1 = fully deformable).
///         Affects longitudinal slip behaviour — deformable surfaces tolerate higher slip before saturation.</item>
///   <item><see cref="NoiseFactor"/>: random grip perturbation amplitude (0 = smooth, 1 = very rough).
///         Provides micro-variation in grip for more realistic feel.</item>
/// </list>
/// </summary>
public readonly struct SurfaceProperties
{
    public float FrictionCoefficient { get; init; }
    public float RollingResistance { get; init; }
    public float DeformationFactor { get; init; }
    public float NoiseFactor { get; init; }

    /// <summary>
    /// Returns the default <see cref="SurfaceProperties"/> for the given <paramref name="surfaceType"/>.
    /// Values are tuned for rally driving conditions — see Milliken &amp; Milliken,
    /// "Race Car Vehicle Dynamics", Table 2.2, adapted for rally surfaces.
    /// </summary>
    public static SurfaceProperties ForType(SurfaceType surfaceType) => surfaceType switch
    {
        SurfaceType.Tarmac => new SurfaceProperties
        {
            FrictionCoefficient = 1.0f,
            RollingResistance = 0.012f,
            DeformationFactor = 0.0f,
            NoiseFactor = 0.02f,
        },
        SurfaceType.Gravel => new SurfaceProperties
        {
            FrictionCoefficient = 0.65f,
            RollingResistance = 0.025f,
            DeformationFactor = 0.35f,
            NoiseFactor = 0.10f,
        },
        SurfaceType.Mud => new SurfaceProperties
        {
            FrictionCoefficient = 0.45f,
            RollingResistance = 0.045f,
            DeformationFactor = 0.60f,
            NoiseFactor = 0.15f,
        },
        SurfaceType.Snow => new SurfaceProperties
        {
            FrictionCoefficient = 0.35f,
            RollingResistance = 0.030f,
            DeformationFactor = 0.40f,
            NoiseFactor = 0.08f,
        },
        SurfaceType.Ice => new SurfaceProperties
        {
            FrictionCoefficient = 0.15f,
            RollingResistance = 0.008f,
            DeformationFactor = 0.0f,
            NoiseFactor = 0.03f,
        },
        _ => SurfaceProperties.ForType(SurfaceType.Tarmac),
    };
}
