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
    /// <summary>Spins the wheel around its axle (forward/reverse drive and braking).</summary>
    [DataMemberIgnore]
    public AngularAxisMotorConstraintComponent? DriveMotor { get; set; }

    /// <summary>Steers the wheel around the chassis-up axis (front wheels only).</summary>
    [DataMemberIgnore]
    public AngularAxisMotorConstraintComponent? SteerMotor { get; set; }

    /// <summary>Software-tracked steering angle (radians) used to implement max-lock limiting.</summary>
    [DataMemberIgnore]
    public float CurrentSteerAngle { get; set; }
}
