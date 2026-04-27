using System;
using Stride.Core.Mathematics;

namespace LibreRally.Vehicle
{
	public static class VehicleTransmissionController
	{
		internal static float AdvanceControllerScale(float currentScale, float targetScale, float responseRate, float dt)
		{
			var delta = targetScale - currentScale;
			var maxStep = MathF.Abs(responseRate) * dt;
			return currentScale + Math.Clamp(delta, -maxStep, maxStep);
		}

		internal static float ClampDrivenWheelOmegaForSlip(float roadWheelOmega, float drivenWheelOmega, float effectiveRatio, float slipAllowanceRpm)
		{
			var safeRatio = MathF.Max(MathF.Abs(effectiveRatio), 1e-3f);
			var omegaToRpm = safeRatio * (60f / (2f * MathF.PI));
			var clampedRpm = ClampShiftRpmForSlip(MathF.Abs(roadWheelOmega) * omegaToRpm, MathF.Abs(drivenWheelOmega) * omegaToRpm, slipAllowanceRpm);
			var direction = MathF.Abs(drivenWheelOmega) > 0.1f ? MathF.Sign(drivenWheelOmega) :
				MathF.Abs(roadWheelOmega) > 0.1f ? MathF.Sign(roadWheelOmega) : 0f;
			return direction == 0f ? 0f : direction * (clampedRpm / omegaToRpm);
		}

		internal static float ResolveDrivenWheelOmega(float fallbackOmega, float forwardSpeed, ReadOnlySpan<float> sampledWheelOmegas, ReadOnlySpan<bool> sampledWheelGrounded)
		{
			var sampleCount = Math.Min(sampledWheelOmegas.Length, sampledWheelGrounded.Length);
			var groundedOmegaSum = 0f; var groundedCount = 0;
			var sampledOmegaSum = 0f; var sampledCount = 0;
			for (var i = 0; i < sampleCount; i++)
			{
				var omega = sampledWheelOmegas[i];
				sampledOmegaSum += omega; sampledCount++;
				if (!sampledWheelGrounded[i]) continue;
				groundedOmegaSum += omega; groundedCount++;
			}
			if (groundedCount > 0) return groundedOmegaSum / groundedCount;
			if (sampledCount > 0 && MathF.Abs(forwardSpeed) > 0.5f) return sampledOmegaSum / sampledCount;
			return fallbackOmega;
		}

		internal static float ResolveAutoClutchDrivenWheelOmega(float fallbackOmega, ReadOnlySpan<float> sampledWheelOmegas, ReadOnlySpan<bool> sampledWheelGrounded)
		{
			var sampleCount = Math.Min(sampledWheelOmegas.Length, sampledWheelGrounded.Length);
			var groundedCount = 0;
			var selectedOmega = fallbackOmega;
			var selectedAbsOmega = float.PositiveInfinity;
			for (var i = 0; i < sampleCount; i++)
			{
				if (!sampledWheelGrounded[i]) continue;
				var omega = sampledWheelOmegas[i];
				var abs = MathF.Abs(omega);
				if (abs < selectedAbsOmega) { selectedAbsOmega = abs; selectedOmega = omega; }
				groundedCount++;
			}
			return groundedCount > 0 ? selectedOmega : fallbackOmega;
		}

		internal static int ResolveStandingGear(int currentGear, float speedKmh, float forwardSpeed, float throttle, float brake, bool handbrakeRequested, float shiftCooldown, float reverseEngageSpeedKmh)
		{
			if (shiftCooldown > 0f || handbrakeRequested) return currentGear;
			var reverseEngageMs = MathF.Max(reverseEngageSpeedKmh / 3.6f, 0.1f);
			var nearStandstill = speedKmh < reverseEngageSpeedKmh && MathF.Abs(forwardSpeed) < reverseEngageMs;
			var wantsForward = throttle > 0.05f;
			var wantsReverse = brake > 0.05f && throttle <= 0.05f;
			if (currentGear == 0) return wantsForward && nearStandstill ? 1 : 0;
			if (wantsReverse && nearStandstill) return 0;
			if (nearStandstill && currentGear > 1 && !wantsForward) return 1;
			return currentGear;
		}

		internal static int ResolveAutoShift(int currentGear, float shiftRpm, bool isBraking, bool isHandbrake, float driveInput, float shiftCooldown, int numForwardGears, float shiftUpRpm, float shiftDownRpm)
		{
			if (currentGear <= 0) return currentGear;
			if (!isBraking && !isHandbrake && driveInput > 0.05f && shiftCooldown <= 0f)
			{
				if (shiftRpm >= shiftUpRpm && currentGear < numForwardGears) return currentGear + 1;
				if (shiftRpm <= shiftDownRpm && currentGear > 1) return currentGear - 1;
			}
			if (isBraking && currentGear > 1 && shiftCooldown <= 0f && shiftRpm < shiftDownRpm) return currentGear - 1;
			return currentGear;
		}

		internal static float ComputeTractionControlTorqueScale(float drivenWheelSlipRatio, float slipRatioTarget, float slipRatioWindow, float minTorqueScale)
		{
			var excess = MathF.Max(MathF.Abs(drivenWheelSlipRatio) - slipRatioTarget, 0f);
			var window = MathF.Max(slipRatioWindow, 0.001f);
			return MathF.Max(minTorqueScale, 1f - excess / window);
		}

		internal static float ComputeAutoClutchTorqueScale(float drivenWheelOmega, float slipClampedOmega, float effectiveRatio, float wheelspinWindowRpm, float minTorqueScale)
		{
			var excessRadPerSec = MathF.Max(MathF.Abs(drivenWheelOmega) - MathF.Abs(slipClampedOmega), 0f);
			var excessRpm = excessRadPerSec * effectiveRatio * (60f / (2f * MathF.PI));
			var window = MathF.Max(wheelspinWindowRpm, 1f);
			return MathF.Max(minTorqueScale, 1f - excessRpm / window);
		}

		internal static float ResolveRollingDirection(float forwardSpeed, float wheelAngularVelocity)
			=> forwardSpeed > 0.1f ? 1f : forwardSpeed < -0.1f ? -1f : MathF.Sign(wheelAngularVelocity);

		internal static float ComputeLowSpeedYawAssistRate(float steerRack, float forwardSpeed, float driveInput, float driveDirection, float assistGain)
		{
			if (assistGain <= 0f) return 0f;
			var absSpeed = MathF.Abs(forwardSpeed);
			if (absSpeed > 3f) return 0f;
			var speedFactor = 1f - (absSpeed / 3f);
			var steerDemand = MathF.Abs(steerRack) > 0.05f ? steerRack : 0f;
			if (MathF.Abs(steerDemand) <= 0.001f) return 0f;
			var driveFactor = MathF.Max(0f, driveInput) * 0.3f + 0.7f;
			return driveFactor * speedFactor * steerDemand * assistGain * driveDirection;
		}

		internal static float ApplyYawAssistTopUp(float currentYaw, float desiredYawRate, float maxStep)
		{
			var error = desiredYawRate - currentYaw;
			return currentYaw + Math.Clamp(error, -maxStep, maxStep);
		}

		private static float ClampShiftRpmForSlip(float roadRpm, float drivenRpm, float slipAllowanceRpm)
		{
			var minRpm = MathF.Max(0f, roadRpm - slipAllowanceRpm);
			var maxRpm = roadRpm + slipAllowanceRpm;
			return Math.Clamp(drivenRpm, minRpm, maxRpm);
		}

		internal static float ComputeEngineRpmFromWheelOmega(float wheelOmega, float effectiveRatio)
		{
			var wheelToEngineRpm = effectiveRatio * (60f / (2f * MathF.PI));
			return wheelOmega * wheelToEngineRpm;
		}
	}
}
