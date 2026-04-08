using Stride.BepuPhysics.Constraints;
using Stride.Core;
using Stride.Engine;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Holds references to wheel motor constraints so <see cref="Vehicle.RallyCarComponent"/> can
/// drive throttle, brake, and steering each frame.
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
}
