using System;
using System.Linq;
using LibreRally.Vehicle;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;

namespace LibreRally.Camera
{
	/// <summary>
	/// Controls the rally car camera behavior, supporting various views such as follow, bonnet, and bumper camera modes.
	/// </summary>
	[ComponentCategory("LibreRally")]
	public class RallyCameraScript : SyncScript
	{
		/// <summary>
		/// Gets or sets the target entity the camera should follow (typically the car).
		/// </summary>
		public Entity? Target { get; set; }

		/// <summary>
		/// Gets or sets the car component to extract physics information from.
		/// </summary>
		public RallyCarComponent? CarComponent { get; set; }

		/// <summary>
		/// Base offset for the follow camera behind the car.
		/// </summary>
		public Vector3 FollowOffset { get; set; } = new Vector3(0, 3f, -12f);

		/// <summary>
		/// Speed at which the follow camera catches up with the target.
		/// </summary>
		public float FollowLerpSpeed { get; set; } = 5f;

		/// <summary>Bonnet/hood camera offset in car-local space (+Z = nose).</summary>
		// Derived from sunburst2_body.jbeam camerasInternal: ["hood", 0.0, -0.67, 1.13, ...] → Stride (x, z, -y)
		public Vector3 BonnetOffset { get; set; } = new Vector3(0, 1.13f, 0.67f);

		/// <summary>Front bumper camera offset in car-local space (+Z = nose).</summary>
		public Vector3 BumperOffset { get; set; } = new Vector3(0, 0.3f, 2.8f);

		private enum CameraMode { Follow, Bonnet, Bumper }
		private CameraMode _mode = CameraMode.Follow;
		private const float LookDeadZone = 0.15f;
		private const float FollowOrbitYawSpeed = 2.5f;
		private const float FollowOrbitPitchSpeed = 1.75f;
		private const float FollowOrbitRecentreSpeed = 4f;
		private const float CockpitLookYawSpeed = 2.5f;
		private const float CockpitLookPitchSpeed = 1.75f;
		private const float CockpitLookRecentreSpeed = 5f;
		private static readonly float FollowOrbitYawLimit = DegreesToRadians(160f);
		private static readonly float FollowOrbitPitchUpLimit = DegreesToRadians(45f);
		private static readonly float FollowOrbitPitchDownLimit = DegreesToRadians(20f);
		private static readonly float CockpitLookYawLimit = DegreesToRadians(80f);
		private static readonly float CockpitLookPitchUpLimit = DegreesToRadians(40f);
		private static readonly float CockpitLookPitchDownLimit = DegreesToRadians(30f);
		private Vector2 _followLookAngles;
		private Vector2 _cockpitLookAngles;

		public override void Start()
		{
			if (Target == null)
			{
				return;
			}

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
			if (Target == null)
			{
				return;
			}

			var dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;
			if (dt <= 0f || dt > 0.1f)
			{
				dt = 0.016f;
			}

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

			var lookInput = pad != null ? ApplyLookDeadZone(pad.State.RightThumb) : Vector2.Zero;
			_followLookAngles = UpdateLookAngles(
				_followLookAngles,
				lookInput,
				FollowOrbitYawSpeed,
				FollowOrbitPitchSpeed,
				FollowOrbitYawLimit,
				-FollowOrbitPitchDownLimit,
				FollowOrbitPitchUpLimit,
				FollowOrbitRecentreSpeed,
				dt);
			_cockpitLookAngles = UpdateLookAngles(
				_cockpitLookAngles,
				lookInput,
				CockpitLookYawSpeed,
				CockpitLookPitchSpeed,
				CockpitLookYawLimit,
				-CockpitLookPitchDownLimit,
				CockpitLookPitchUpLimit,
				CockpitLookRecentreSpeed,
				dt);

			switch (_mode)
			{
				case CameraMode.Follow: UpdateFollowCamera(dt); break;
				case CameraMode.Bonnet: UpdateCockpitCamera(BonnetOffset); break;
				case CameraMode.Bumper: UpdateCockpitCamera(BumperOffset); break;
			}
		}

		private void UpdateFollowCamera(float dt)
		{
			var targetTransform = Target!.Transform;
			var targetWorldPos = targetTransform.WorldMatrix.TranslationVector;

			// Use only the car's yaw so the camera stays level even when the chassis pitches/rolls.
			// WorldMatrix.Backward = local +Z in world space = car nose direction for this car.
			var carNoseDir = targetTransform.WorldMatrix.Backward;
			carNoseDir.Y = 0f;
			var yaw = carNoseDir.LengthSquared() > 0.001f
				? MathF.Atan2(carNoseDir.X, carNoseDir.Z)
				: 0f;
			var yawOnly = Quaternion.RotationY(yaw);

			var orbitOffset = Vector3.Transform(
				FollowOffset,
				Quaternion.RotationYawPitchRoll(-_followLookAngles.X, _followLookAngles.Y, 0f));
			var worldOffset = Vector3.Transform(orbitOffset, yawOnly);
			var desiredPos = targetWorldPos + worldOffset;

			Entity.Transform.Position = Vector3.Lerp(
				Entity.Transform.Position, desiredPos, MathF.Min(1f, FollowLerpSpeed * dt));

			ApplyCameraRotation(targetWorldPos);
		}

		/// <summary>
		/// Points the camera toward <paramref name="targetWorldPos"/> using correct Stride conventions.
		/// Stride camera default forward = -Z. After RotationY(yaw), local -Z becomes (-sin yaw, 0, -cos yaw).
		/// </summary>
		private void ApplyCameraRotation(Vector3 targetWorldPos)
		{
			var lookDir = targetWorldPos - Entity.Transform.Position;
			if (lookDir.LengthSquared() < 0.000001f)
			{
				return;
			}

			lookDir.Normalize();
			var horizontal = MathF.Sqrt(lookDir.X * lookDir.X + lookDir.Z * lookDir.Z);
			var yaw   = MathF.Atan2(-lookDir.X, -lookDir.Z);
			var pitch = MathF.Atan2(lookDir.Y,  horizontal);
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
			var lookOffset = Quaternion.RotationYawPitchRoll(_cockpitLookAngles.X, -_cockpitLookAngles.Y, 0f);
			// Rotate into the car's +Z-facing frame, then apply the look offset in that car-facing space.
			Entity.Transform.Rotation = targetTransform.Rotation * lookOffset * Quaternion.RotationY(MathF.PI);
		}

		private static Vector2 ApplyLookDeadZone(Vector2 thumb)
		{
			var magnitude = thumb.Length();
			if (magnitude <= LookDeadZone || magnitude <= float.Epsilon)
			{
				return Vector2.Zero;
			}

			var scaledMagnitude = MathF.Min(1f, (magnitude - LookDeadZone) / (1f - LookDeadZone));
			return thumb * (scaledMagnitude / magnitude);
		}

		private static Vector2 UpdateLookAngles(
			Vector2 currentAngles,
			Vector2 lookInput,
			float yawSpeed,
			float pitchSpeed,
			float yawLimit,
			float minPitch,
			float maxPitch,
			float recentreSpeed,
			float dt)
		{
			if (lookInput.LengthSquared() > 0f)
			{
				currentAngles.X = MathUtil.Clamp(currentAngles.X + lookInput.X * yawSpeed * dt, -yawLimit, yawLimit);
				currentAngles.Y = MathUtil.Clamp(currentAngles.Y + lookInput.Y * pitchSpeed * dt, minPitch, maxPitch);
				return currentAngles;
			}

			var t = MathF.Min(1f, recentreSpeed * dt);
			currentAngles.X = MathUtil.Lerp(currentAngles.X, 0f, t);
			currentAngles.Y = MathUtil.Lerp(currentAngles.Y, 0f, t);
			if (MathF.Abs(currentAngles.X) < 0.001f)
			{
				currentAngles.X = 0f;
			}

			if (MathF.Abs(currentAngles.Y) < 0.001f)
			{
				currentAngles.Y = 0f;
			}

			return currentAngles;
		}

		private static float DegreesToRadians(float degrees) => degrees * (MathF.PI / 180f);
	}
}
