using System.Collections.Generic;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Constraints;
using Stride.Core;
using Stride.Engine;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Holds references to wheel motor constraints so <see cref="Vehicle.RallyCarComponent"/> can
/// drive throttle, brake, and steering each frame.
/// Also stores references to the new tyre model and dynamics system.
/// </summary>
[DataContract]
[ComponentCategory("LibreRally")]
public class WheelSettings : EntityComponent
{
    /// <summary>Drives wheel spin in chassis-local space so the axle can steer without losing drive authority.</summary>
    [DataMemberIgnore]
    public AngularMotorConstraintComponent? DriveMotor { get; set; }

    /// <summary>Steers the wheel around the chassis-up axis (front wheels only).</summary>
    [DataMemberIgnore]
    public AngularAxisMotorConstraintComponent? SteerMotor { get; set; }

    /// <summary>Rate-limited rack command (radians) that the steering servo tracks.</summary>
    [DataMemberIgnore]
    public float CurrentSteerAngle { get; set; }

    /// <summary>Soft-body tyre state and the paper-shaped lateral force curve for this wheel.</summary>
    [DataMemberIgnore]
    public SoftBodyTireModel? TireModel { get; set; }

    /// <summary>Slip-based tyre model with Pacejka/brush physics for this wheel.</summary>
    [DataMemberIgnore]
    public TyreModel? AdvancedTyreModel { get; set; }

    /// <summary>Nominal wheel load for the tyre model, initialized during vehicle build from quarter-mass × gravity.</summary>
    [DataMemberIgnore]
    public float StaticNormalLoad { get; set; }

    /// <summary>Index of this wheel in the <see cref="VehicleDynamicsSystem"/> arrays (0=FL, 1=FR, 2=RL, 3=RR).</summary>
    [DataMemberIgnore]
    public int DynamicsIndex { get; set; } = -1;

    /// <summary>Current surface type under this wheel. Updated by raycast/contact detection.</summary>
    [DataMemberIgnore]
    public SurfaceType CurrentSurface { get; set; } = SurfaceType.Tarmac;

    /// <summary>Reusable hit buffer for the tyre ground-contact probe.</summary>
    [DataMemberIgnore]
    public List<HitInfo> GroundProbeHits { get; } = new();
}
