using System;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Definitions;
using Stride.BepuPhysics.Definitions.Colliders;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally.Race;

[ComponentCategory("LibreRally")]
public class CheckpointTrigger : StartupScript
{
    public int CheckpointIndex { get; set; }
    public Vector3 Size { get; set; } = new Vector3(10f, 4f, 2f);
    public event Action<int>? Triggered;

    private bool _triggered;

    public override void Start()
    {
        var trigger = new Trigger();
        trigger.OnEnter += (source, other) =>
        {
            if (!_triggered && other is BodyComponent)
            {
                _triggered = true;
                Triggered?.Invoke(CheckpointIndex);
            }
        };

        var staticComp = new StaticComponent
        {
            Collider = new CompoundCollider
            {
                Colliders = { new BoxCollider { Size = Size, Mass = 1f } }
            },
            ContactEventHandler = trigger,
        };

        Entity.Add(staticComp);
    }
}
