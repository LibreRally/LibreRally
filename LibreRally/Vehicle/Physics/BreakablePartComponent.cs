using Stride.BepuPhysics.Constraints;
using Stride.Engine;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Monitors the weld constraint each frame and breaks it when the measured
/// constraint force exceeds <see cref="BreakStrength"/>.
///
/// Once broken, the part's <see cref="Stride.BepuPhysics.BodyComponent"/> becomes a free rigid body
/// that interacts normally with the physics world.
///
/// Attach this to the same entity as the <see cref="WeldConstraintComponent"/> and
/// the part's <see cref="Stride.BepuPhysics.BodyComponent"/>.
/// </summary>
public class BreakablePartComponent : SyncScript
{
    /// <summary>
    /// Force threshold (N) above which the part detaches.
    /// Sourced from the minimum beamStrength of cross-boundary beams in the jbeam.
    /// </summary>
    public float BreakStrength { get; set; } = 50000f;

    /// <summary>The weld constraint to break. Set by <see cref="VehiclePhysicsBuilder"/>.</summary>
    public WeldConstraintComponent? WeldConstraint { get; set; }

    private bool _broken;

    public override void Update()
    {
        if (_broken || WeldConstraint == null || !WeldConstraint.Attached)
        {
	        return;
        }

        var force = WeldConstraint.GetAccumulatedForceMagnitude();
        if (force > BreakStrength)
        {
	        Break();
        }
    }

    private void Break()
    {
        _broken = true;

        if (WeldConstraint != null)
        {
            Entity.Remove(WeldConstraint);
            WeldConstraint = null;
        }

        // Remove this script — no longer needed
        Entity.Remove(this);
    }
}
