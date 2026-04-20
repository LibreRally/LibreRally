using System;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Definitions;
using Stride.BepuPhysics.Definitions.Colliders;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally.Race;

/// <summary>
/// A trigger component that detects when a vehicle passes through a checkpoint.
/// </summary>
[ComponentCategory("LibreRally")]
public class CheckpointTrigger : StartupScript
{
    /// <summary>
    /// Gets or sets the index of this checkpoint in the race.
    /// </summary>
    public int CheckpointIndex { get; set; }

    /// <summary>
    /// Gets or sets the physical size of the checkpoint trigger volume.
    /// </summary>
    public Vector3 Size { get; set; } = new Vector3(10f, 4f, 2f);

    /// <summary>
    /// Occurs when a vehicle body enters the checkpoint volume. Provides the checkpoint index.
    /// </summary>
    public event Action<int>? Triggered;

    private bool _triggered;

    /// <summary>
    /// Creates and attaches the trigger volume that reports checkpoint crossings.
    /// </summary>
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
