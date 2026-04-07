using System;
using System.Linq;
using LibreRally.Vehicle;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;

namespace LibreRally.Camera;

[ComponentCategory("LibreRally")]
public class RallyCameraScript : SyncScript
{
    public Entity? Target { get; set; }
    public RallyCarComponent? CarComponent { get; set; }

    // Car nose faces Stride +Z (BeamNG -Y → Stride +Z). Camera must be at -Z (behind the tail).
    public Vector3 FollowOffset { get; set; } = new Vector3(0, 3f, -12f);
    public float FollowLerpSpeed { get; set; } = 5f;

    /// <summary>Bonnet/hood camera offset in car-local space (+Z = nose).</summary>
    public Vector3 BonnetOffset { get; set; } = new Vector3(0, 0.8f, 1.5f);

    /// <summary>Front bumper camera offset in car-local space (+Z = nose).</summary>
    public Vector3 BumperOffset { get; set; } = new Vector3(0, 0.3f, 2.8f);

    private enum CameraMode { Follow, Bonnet, Bumper }
    private CameraMode _mode = CameraMode.Follow;

    public override void Start()
    {
        if (Target == null) return;
        var worldPos = ComputeApproximateWorldPos(Target);
        Entity.Transform.Position = worldPos + FollowOffset;
        ApplyCameraRotation(worldPos);
    }

    /// <summary>Sums local positions up the parent chain (valid when all ancestors have identity rotation).</summary>
    private static Vector3 ComputeApproximateWorldPos(Entity entity)
    {
        var pos = entity.Transform.Position;
        var parent = entity.Transform.Parent;
        while (parent != null)
        {
            pos += parent.Entity.Transform.Position;
            parent = parent.Parent;
        }
        return pos;
    }

    public override void Update()
    {
        if (Target == null) return;

        var pad = Input.GamePads.FirstOrDefault();
        if (pad != null && pad.IsButtonPressed(GamePadButton.Y))
        {
            // Cycle: Follow → Bonnet → Bumper → Follow
            _mode = _mode switch
            {
                CameraMode.Follow => CameraMode.Bonnet,
                CameraMode.Bonnet => CameraMode.Bumper,
                _                 => CameraMode.Follow,
            };
        }

        switch (_mode)
        {
            case CameraMode.Follow: UpdateFollowCamera(); break;
            case CameraMode.Bonnet: UpdateCockpitCamera(BonnetOffset); break;
            case CameraMode.Bumper: UpdateCockpitCamera(BumperOffset); break;
        }
    }

    private void UpdateFollowCamera()
    {
        var targetTransform = Target!.Transform;

        // Use only the car's yaw so the camera stays level even when the chassis pitches/rolls.
        // WorldMatrix.Backward = local +Z in world space = car nose direction for this car.
        var carNoseDir = targetTransform.WorldMatrix.Backward;
        carNoseDir.Y = 0f;
        float yaw = carNoseDir.LengthSquared() > 0.001f
            ? MathF.Atan2(carNoseDir.X, carNoseDir.Z)
            : 0f;
        var yawOnly = Quaternion.RotationY(yaw);

        var worldOffset = Vector3.Transform(FollowOffset, yawOnly);
        var desiredPos = targetTransform.WorldMatrix.TranslationVector + worldOffset;

        var dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;
        Entity.Transform.Position = Vector3.Lerp(
            Entity.Transform.Position, desiredPos, MathF.Min(1f, FollowLerpSpeed * dt));

        ApplyCameraRotation(targetTransform.WorldMatrix.TranslationVector);
    }

    /// <summary>
    /// Points the camera toward <paramref name="targetWorldPos"/> using correct Stride conventions.
    /// Stride camera default forward = -Z. After RotationY(yaw), local -Z becomes (-sin yaw, 0, -cos yaw).
    /// </summary>
    private void ApplyCameraRotation(Vector3 targetWorldPos)
    {
        var lookDir = targetWorldPos - Entity.Transform.Position;
        if (lookDir.LengthSquared() < 0.000001f) return;

        lookDir.Normalize();
        float horizontal = MathF.Sqrt(lookDir.X * lookDir.X + lookDir.Z * lookDir.Z);
        float yaw   = MathF.Atan2(-lookDir.X, -lookDir.Z);
        float pitch = MathF.Atan2(lookDir.Y,  horizontal);
        Entity.Transform.Rotation = Quaternion.RotationYawPitchRoll(yaw, pitch, 0f);
    }

    /// <summary>
    /// Positions the camera at <paramref name="localOffset"/> in car-local space, looking forward.
    /// Car nose = local +Z; Stride camera forward = local -Z, so we append a 180° Y rotation
    /// so that the camera's -Z aligns with the car's +Z (nose direction).
    /// </summary>
    private void UpdateCockpitCamera(Vector3 localOffset)
    {
        var targetTransform = Target!.Transform;
        var worldOffset = Vector3.Transform(localOffset, targetTransform.Rotation);
        Entity.Transform.Position = targetTransform.WorldMatrix.TranslationVector + worldOffset;
        // Stride camera looks along -Z; car nose is +Z. Rotate 180° around Y to flip Z.
        Entity.Transform.Rotation = targetTransform.Rotation * Quaternion.RotationY(MathF.PI);
    }
}
