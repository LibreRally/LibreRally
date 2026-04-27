using System;
using Stride.Core.Mathematics;
using Stride.Input;

namespace LibreRally.Vehicle
{
	public readonly struct VehicleInputState
	{
		public float Throttle { get; }
		public float Brake { get; }
		public float Steer { get; }
		public bool HandbrakeRequested { get; }
		public bool HasInput => Throttle > 0.02f || Brake > 0.02f || MathF.Abs(Steer) > 0.02f || HandbrakeRequested;

		public VehicleInputState(float throttle, float brake, float steer, bool handbrakeRequested)
		{
			Throttle = throttle;
			Brake = brake;
			Steer = steer;
			HandbrakeRequested = handbrakeRequested;
		}

		public static VehicleInputState Read(
			InputManager input,
			bool playerInputEnabled,
			float gamePadTriggerDeadzone,
			float gamePadSteerDeadzone)
		{
			float throttle = 0f, brake = 0f, steer = 0f;
			var handbrakeRequested = false;

			if (!playerInputEnabled)
			{
				return new VehicleInputState(throttle, brake, steer, handbrakeRequested);
			}

			var pad = input.GamePads.Count > 0 ? input.GamePads[0] : null;
			if (pad != null)
			{
				throttle = ApplyTriggerDeadzone(pad.State.RightTrigger, gamePadTriggerDeadzone);
				brake = ApplyTriggerDeadzone(pad.State.LeftTrigger, gamePadTriggerDeadzone);
				steer = ApplySignedAxisDeadzone(pad.State.LeftThumb.X, gamePadSteerDeadzone);
				handbrakeRequested = pad.IsButtonDown(GamePadButton.A);
			}

			if (input.IsKeyDown(Keys.Up) || input.IsKeyDown(Keys.W))
				throttle = MathF.Max(throttle, 1f);
			if (input.IsKeyDown(Keys.Down) || input.IsKeyDown(Keys.S))
				brake = MathF.Max(brake, 1f);
			if (input.IsKeyDown(Keys.Left) || input.IsKeyDown(Keys.A))
				steer = MathF.Min(steer, -1f);
			if (input.IsKeyDown(Keys.Right) || input.IsKeyDown(Keys.D))
				steer = MathF.Max(steer, 1f);
			if (input.IsKeyDown(Keys.Space))
				handbrakeRequested = true;

			return new VehicleInputState(throttle, brake, steer, handbrakeRequested);
		}

		internal static float ApplyTriggerDeadzone(float value, float deadzone)
		{
			var clampedDeadzone = Math.Clamp(deadzone, 0f, 0.99f);
			var clampedValue = Math.Clamp(value, 0f, 1f);
			if (clampedValue <= clampedDeadzone)
				return 0f;
			return (clampedValue - clampedDeadzone) / (1f - clampedDeadzone);
		}

		internal static float ApplySignedAxisDeadzone(float value, float deadzone)
		{
			var magnitude = ApplyTriggerDeadzone(MathF.Abs(value), deadzone);
			return magnitude == 0f ? 0f : MathF.CopySign(magnitude, value);
		}

		public static bool ShouldKeepAwake(VehicleInputState input, Vector3 linearVelocity, Vector3 angularVelocity)
		{
			if (input.HasInput)
				return true;
			return linearVelocity.LengthSquared() > 0.01f || angularVelocity.LengthSquared() > 0.01f;
		}
	}
}
