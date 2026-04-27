using System;
using Stride.Core.Mathematics;

namespace LibreRally.Vehicle
{
	public struct VehicleEngineState
	{
		public float EngineRpm;
		public float EngineTemp;
		public float OilTemp;
		public float FuelLiters;
		public float TurboBoostBar;
		public float OilPressure;
		public float DrivelineRpm;
		public float CrankTorque;
		public float EngineBrakeTorqueApplied;
		public float DriveTorqueScale;
	}

	public sealed class VehicleEngineSimulator
	{
		public float EngineInertia { get; set; } = 0.25f;
		public float EngineFriction { get; set; } = 11.5f;
		public float EngineDynamicFriction { get; set; } = 0.024f;
		public float EngineBrakeTorque { get; set; } = 38f;
		public float MaxRpm { get; set; } = 7500f;
		public float IdleRpm { get; set; } = 900f;
		public float[] TorqueCurveRpm { get; set; } = [];
		public float[] TorqueCurveNm { get; set; } = [];
		public float AutoClutchLaunchRpm { get; set; } = 4500f;
		public float AutoClutchWheelspinWindowRpm { get; set; } = 1400f;
		public float AutoClutchMinTorqueScale { get; set; } = 0.25f;
		public float ShiftSlipAllowanceRpm { get; set; } = 900f;

		public void StepRpm(ref VehicleEngineState state, float driveInput, float drivelineRpm, float forwardSpeed, float effectiveRatio, float dt)
		{
			var launchRpm = IdleRpm + (AutoClutchLaunchRpm - IdleRpm) * driveInput;
			var torqueRpm = driveInput > 0.05f ? MathF.Max(drivelineRpm, launchRpm) : drivelineRpm;

			var crankTorque = InterpolateTorqueCurve(torqueRpm) * driveInput;
			var applyEngineBraking = driveInput < 0.05f && MathF.Abs(forwardSpeed) > 0.25f;
			var engBrake = applyEngineBraking ? EngineBrakeTorque : 0f;

			var engineOmega = state.EngineRpm * (2f * MathF.PI / 60f);
			var frictionLoss = EngineFriction + EngineDynamicFriction * engineOmega;
			var displayCrank = InterpolateTorqueCurve(state.EngineRpm) * driveInput;
			if (state.EngineRpm >= MaxRpm)
				displayCrank = 0f;

			var netDisplay = displayCrank - frictionLoss - engBrake;
			state.EngineRpm += (netDisplay / EngineInertia) * dt * (60f / (2f * MathF.PI));
			var displayTargetRpm = driveInput > 0.05f ? torqueRpm : drivelineRpm;
			var clutchCoupling = driveInput > 0.05f ? 7f : 5f;
			state.EngineRpm += (displayTargetRpm - state.EngineRpm) * clutchCoupling * dt;

			var slipBlend = Math.Clamp(MathF.Abs(forwardSpeed) * 3.6f / 35f, 0f, 1f);
			var clutchSlipAllowance = driveInput > 0.05f
				? MathUtil.Lerp(AutoClutchLaunchRpm - IdleRpm, 250f, slipBlend)
				: 150f;
			var maxCoupledRpm = drivelineRpm + clutchSlipAllowance;
			state.EngineRpm = MathF.Min(state.EngineRpm, MathF.Max(IdleRpm, maxCoupledRpm));
			state.EngineRpm = Math.Clamp(state.EngineRpm, IdleRpm, MaxRpm + 300f);

			state.CrankTorque = crankTorque;
			state.EngineBrakeTorqueApplied = engBrake;
		}

		public float InterpolateTorqueCurve(float rpm)
		{
			if (TorqueCurveRpm.Length == 0 || TorqueCurveNm.Length == 0)
				return 0f;
			if (rpm <= TorqueCurveRpm[0])
				return TorqueCurveNm[0];
			var last = TorqueCurveRpm.Length - 1;
			if (rpm >= TorqueCurveRpm[last])
				return TorqueCurveNm[last];
			int i;
			for (i = 0; i < last; i++)
			{
				if (rpm <= TorqueCurveRpm[i + 1])
					break;
			}
			var t = (rpm - TorqueCurveRpm[i]) / (TorqueCurveRpm[i + 1] - TorqueCurveRpm[i]);
			return TorqueCurveNm[i] + (TorqueCurveNm[i + 1] - TorqueCurveNm[i]) * t;
		}

		public static float ComputeAutoClutchTorqueScale(float drivenWheelOmega, float slipClampedOmega, float effectiveRatio, float wheelspinWindowRpm, float minTorqueScale)
		{
			var excessRadPerSec = MathF.Max(MathF.Abs(drivenWheelOmega) - MathF.Abs(slipClampedOmega), 0f);
			var excessRpm = excessRadPerSec * effectiveRatio * (60f / (2f * MathF.PI));
			var window = MathF.Max(wheelspinWindowRpm, 1f);
			return MathF.Max(minTorqueScale, 1f - excessRpm / window);
		}

		public static float AdjustAndClampDrivenWheelOmega(float roadWheelOmega, float drivenWheelOmega, float effectiveRatio, float slipAllowanceRpm)
		{
			var maxSlipRadPerSec = slipAllowanceRpm / (effectiveRatio * (60f / (2f * MathF.PI)));
			return Math.Clamp(drivenWheelOmega, roadWheelOmega - maxSlipRadPerSec, roadWheelOmega + maxSlipRadPerSec);
		}
	}
}
