using Stride.BepuPhysics;
using Stride.BepuPhysics.Constraints;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Monitors the velocity change of this part each frame and breaks the weld constraint
/// when the estimated impulse exceeds <see cref="BreakStrength"/>.
///
/// Uses (ΔV × mass) as a proxy for the collision impulse because
/// <see cref="WeldConstraintComponent"/> does not expose its internal constraint force.
///
/// Once broken, the part's <see cref="BodyComponent"/> becomes a free rigid body
/// that interacts normally with the physics world.
///
/// Attach this to the same entity as the <see cref="WeldConstraintComponent"/> and
/// the part's <see cref="BodyComponent"/>.
/// </summary>
public class BreakablePartComponent : SyncScript
{
    /// <summary>
    /// Impulse threshold (N·s) above which the part detaches.
    /// Sourced from the minimum beamStrength of cross-boundary beams in the jbeam.
    /// </summary>
    public float BreakStrength { get; set; } = 50000f;

    /// <summary>The weld constraint to break. Set by <see cref="VehiclePhysicsBuilder"/>.</summary>
    public WeldConstraintComponent? WeldConstraint { get; set; }

    private bool _broken;
    private BodyComponent? _body;

    public override void Start()
    {
        _body = Entity.Get<BodyComponent>();
    }

    public override void Update()
    {
        if (_broken || WeldConstraint == null || _body == null) return;

        // Estimate the impulse that acted on this part this frame:
        //   impulse ≈ ΔV × mass
        var deltaV = _body.LinearVelocity - _body.PreviousLinearVelocity;
        float deltaVMag = deltaV.Length();
        if (deltaVMag < 0.001f) return;

        float inverseMass = _body.BodyInertia.InverseMass;
        float mass = inverseMass > 0f ? 1f / inverseMass : 500f;
        float impulse = deltaVMag * mass;

        if (impulse > BreakStrength)
            Break();
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
