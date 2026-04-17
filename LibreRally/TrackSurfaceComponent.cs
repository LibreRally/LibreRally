using LibreRally.Vehicle.Physics;
using Stride.Core;
using Stride.Engine;

namespace LibreRally;

/// <summary>
/// Tags a ground entity with the driving surface type used by the tyre model.
/// </summary>
[DataContract]
[ComponentCategory("LibreRally")]
public sealed class TrackSurfaceComponent : EntityComponent
{
    /// <summary>
    /// Surface type sampled by wheel ground probes for this track section.
    /// </summary>
    public SurfaceType SurfaceType { get; set; } = SurfaceType.Tarmac;
}
