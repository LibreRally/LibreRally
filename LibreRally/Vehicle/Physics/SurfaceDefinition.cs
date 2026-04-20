using System;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Identifies a driving surface material.
/// Each type defines a unique set of friction, resistance, and deformation properties
/// that modify tyre grip and slip behaviour.
///
/// Reference: Pacejka, "Tire and Vehicle Dynamics", Chapter 4 — road surface effects.
/// Reference: The Contact Patch, C1603 — road surface texture and skid resistance.
/// </summary>
public enum SurfaceType : byte
{
    /// <summary>Dry tarmac — high grip and low rolling resistance.</summary>
    Tarmac,

    /// <summary>Loose gravel — medium grip and high deformation.</summary>
    Gravel,

    /// <summary>Wet mud — low grip and high rolling resistance.</summary>
    Mud,

    /// <summary>Hard-packed snow — low grip and moderate deformation.</summary>
    Snow,

    /// <summary>Solid ice — very low grip and minimal rolling resistance.</summary>
    Ice,

    /// <summary>Wet tarmac — reduced grip from surface water film.</summary>
    WetTarmac,
}

/// <summary>
/// Physics properties for a driving surface.
/// These values scale the tyre model's grip envelope and energy dissipation.
///
/// <para>Surface friction is decomposed into two texture-scale contributions following
/// The Contact Patch, C1603:</para>
/// <list type="bullet">
///   <item><see cref="FrictionCoefficient"/>: overall peak µ multiplier (1.0 = reference dry tarmac).
///         This is the combined result of microtexture and macrotexture contributions.</item>
///   <item><see cref="Microtexture"/>: adhesion-dominant grip from sub-0.5 mm asperity peaks (0–1 scale).
///         Provides dry and light-wet grip. Polished surfaces have low microtexture.
///         Reference: The Contact Patch, C1603, §Microtexture.</item>
///   <item><see cref="Macrotexture"/>: hysteresis grip and water-evacuation capacity from 0.5–20 mm
///         scale aggregate texture (0–1 scale). High macrotexture drains water from the
///         contact patch, delaying hydroplaning onset.
///         Reference: The Contact Patch, C1603, §Macro-texture.</item>
///   <item><see cref="WaterDepth"/>: surface water film thickness (m). Zero for dry surfaces.
///         Values above ~0.0025 m (2.5 mm) trigger hydroplaning risk at speed.
///         Reference: The Contact Patch, C1603, §Aqua-planing.</item>
///   <item><see cref="RollingResistance"/>: rolling-resistance force coefficient (N per N of load).</item>
///   <item><see cref="DeformationFactor"/>: how much the surface deforms under load (0 = rigid, 1 = fully deformable).
///         Affects longitudinal slip behaviour — deformable surfaces tolerate higher slip before saturation.</item>
///   <item><see cref="NoiseFactor"/>: road roughness amplitude (0 = smooth, 1 = very rough).
///         Drives deterministic per-frame micro-variation in grip based on road-surface
///         power spectral density, giving the feel of aggregate texture at the contact patch.
///         Reference: The Contact Patch, C1603, §Power spectral density curves.</item>
/// </list>
/// </summary>
public readonly struct SurfaceProperties
{
    /// <summary>Peak friction multiplier (1.0 = reference dry tarmac).</summary>
    public float FrictionCoefficient { get; init; }

    /// <summary>
    /// Microtexture level (0–1). Represents sub-0.5 mm asperity peaks that provide
    /// adhesion grip. Higher values = grittier surface = better dry and light-wet grip.
    /// Fresh calcined bauxite ≈ 1.0; polished stone ≈ 0.3.
    /// Reference: The Contact Patch, C1603, §Microtexture.
    /// </summary>
    public float Microtexture { get; init; }

    /// <summary>
    /// Macrotexture level (0–1). Represents 0.5–20 mm aggregate texture depth.
    /// High macrotexture provides hysteresis grip and drains water from the contact patch.
    /// Positive texture (surface dressing) ≈ 0.9; polished concrete ≈ 0.2.
    /// Reference: The Contact Patch, C1603, §Macro-texture.
    /// </summary>
    public float Macrotexture { get; init; }

    /// <summary>
    /// Surface water film thickness (m). Zero for dry conditions.
    /// Light rain ≈ 0.0005 m, moderate ≈ 0.001 m, heavy ≈ 0.003 m.
    /// Hydroplaning onset ≈ 0.0025 m at highway speeds.
    /// Reference: The Contact Patch, C1603, §Aqua-planing.
    /// </summary>
    public float WaterDepth { get; init; }

    /// <summary>Rolling-resistance force coefficient (N per N of load).</summary>
    public float RollingResistance { get; init; }

    /// <summary>Deformation factor (0 = rigid, 1 = fully deformable).</summary>
    public float DeformationFactor { get; init; }

    /// <summary>Road roughness amplitude (0 = smooth, 1 = very rough).</summary>
    public float NoiseFactor { get; init; }

    /// <summary>
    /// Returns the default <see cref="SurfaceProperties"/> for the given <paramref name="surfaceType"/>.
    /// Values are tuned for rally driving conditions.
    /// Friction coefficients calibrated from The Contact Patch, C1603, Table 1:
    /// dry tarmac 0.8–1.0, wet 0.2–0.65, icy 0.15.
    /// Microtexture/macrotexture values represent typical UK road aggregate.
    /// </summary>
    public static SurfaceProperties ForType(SurfaceType surfaceType) => surfaceType switch
    {
        SurfaceType.Tarmac => new SurfaceProperties
        {
            FrictionCoefficient = 1.0f,
            Microtexture = 0.8f,
            Macrotexture = 0.6f,
            WaterDepth = 0f,
            RollingResistance = 0.012f,
            DeformationFactor = 0.0f,
            NoiseFactor = 0.02f,
        },
        SurfaceType.WetTarmac => new SurfaceProperties
        {
            FrictionCoefficient = 0.55f,
            Microtexture = 0.8f,
            Macrotexture = 0.6f,
            WaterDepth = 0.001f,
            RollingResistance = 0.014f,
            DeformationFactor = 0.0f,
            NoiseFactor = 0.03f,
        },
        SurfaceType.Gravel => new SurfaceProperties
        {
            FrictionCoefficient = 0.65f,
            Microtexture = 0.5f,
            Macrotexture = 0.9f,
            WaterDepth = 0f,
            RollingResistance = 0.025f,
            DeformationFactor = 0.35f,
            NoiseFactor = 0.10f,
        },
        SurfaceType.Mud => new SurfaceProperties
        {
            FrictionCoefficient = 0.45f,
            Microtexture = 0.3f,
            Macrotexture = 0.2f,
            WaterDepth = 0.002f,
            RollingResistance = 0.045f,
            DeformationFactor = 0.60f,
            NoiseFactor = 0.15f,
        },
        SurfaceType.Snow => new SurfaceProperties
        {
            FrictionCoefficient = 0.35f,
            Microtexture = 0.2f,
            Macrotexture = 0.3f,
            WaterDepth = 0f,
            RollingResistance = 0.030f,
            DeformationFactor = 0.40f,
            NoiseFactor = 0.08f,
        },
        SurfaceType.Ice => new SurfaceProperties
        {
            FrictionCoefficient = 0.15f,
            Microtexture = 0.05f,
            Macrotexture = 0.05f,
            WaterDepth = 0f,
            RollingResistance = 0.008f,
            DeformationFactor = 0.0f,
            NoiseFactor = 0.03f,
        },
        _ => SurfaceProperties.ForType(SurfaceType.Tarmac),
    };
}
