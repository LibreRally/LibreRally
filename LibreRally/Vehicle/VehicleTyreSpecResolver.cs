using System;
using System.Collections.Generic;
using System.Linq;
using LibreRally.Vehicle.Physics;

namespace LibreRally.Vehicle
{
	internal readonly record struct VehicleTyreSpec(
		float Radius,
		float Width,
		float PressureKpa,
		float PeakFrictionCoefficient,
		float RollingResistanceCoefficient,
		float? BeamNgNoLoadFrictionCoefficient = null,
		float? BeamNgFullLoadFrictionCoefficient = null,
		float? BeamNgLoadSensitivitySlope = null);

	internal static class VehicleTyreSpecResolver
	{
		public const float DefaultRadius = 0.305f;
		public const float DefaultWidth = 0.205f;
		public const float DefaultPressureKpa = 220f;
		public const float DefaultPeakFrictionCoefficient = 1.05f;
		public const float DefaultRollingResistanceCoefficient = 0.012f;

		private const float PsiToKpa = 6.894757f;
		private const float ReferenceLoad = 3000f;

		public static VehicleTyreSpec Resolve(VehicleDefinition definition, bool front)
		{
			var options = definition.PressureWheelOptions
				.Where(option => AppliesToAxle(option, front))
				.Select(option => option.Options)
				.ToList();

			if (options.Count == 0)
			{
				options = definition.PressureWheelOptions
					.Select(option => option.Options)
					.Where(option => option.HasTire == true)
					.ToList();
			}

			if (options.Count == 0)
			{
				options = definition.PressureWheelOptions
					.Select(option => option.Options)
					.ToList();
			}

			var radius = ResolveLastPositive(options.Select(option => option.Radius), DefaultRadius);
			var width = ResolveLastPositive(options.Select(option => option.TireWidth), DefaultWidth);
			var pressurePsi = ResolveLastPositive(options.Select(option => option.PressurePsi), DefaultPressureKpa / PsiToKpa);
			var frictionCoef = ResolveLastPositive(options.Select(option => option.FrictionCoef), 1f);
			var noLoadCoef = ResolveLastPositive(options.Select(option => option.NoLoadCoef));
			var fullLoadCoef = ResolveLastPositive(options.Select(option => option.FullLoadCoef));
			var loadSensitivitySlope = ResolveLastPositive(options.Select(option => option.LoadSensitivitySlope));

			var peakFrictionCoefficient = DefaultPeakFrictionCoefficient * frictionCoef;
			if (noLoadCoef.HasValue && fullLoadCoef.HasValue && loadSensitivitySlope.HasValue)
			{
				peakFrictionCoefficient = TyreModel.ComputeBeamNgLoadCoefficient(
					ReferenceLoad,
					noLoadCoef.Value,
					fullLoadCoef.Value,
					loadSensitivitySlope.Value) * frictionCoef;
			}

			return new VehicleTyreSpec(
				Radius: radius,
				Width: width,
				PressureKpa: pressurePsi * PsiToKpa,
				PeakFrictionCoefficient: Math.Clamp(peakFrictionCoefficient, 0.4f, 2f),
				RollingResistanceCoefficient: DefaultRollingResistanceCoefficient,
				BeamNgNoLoadFrictionCoefficient: noLoadCoef,
				BeamNgFullLoadFrictionCoefficient: fullLoadCoef,
				BeamNgLoadSensitivitySlope: loadSensitivitySlope);
		}

		public static float ResolveDrivenWheelRadius(
			VehiclePowertrainSetup powertrain,
			in VehicleTyreSpec frontSpec,
			in VehicleTyreSpec rearSpec)
		{
			if (powertrain.DriveFrontAxle && powertrain.DriveRearAxle)
			{
				return (frontSpec.Radius + rearSpec.Radius) * 0.5f;
			}

			return powertrain.DriveFrontAxle ? frontSpec.Radius : rearSpec.Radius;
		}

		internal static bool MatchesAxle(string source, bool front)
		{
			if (string.IsNullOrWhiteSpace(source))
			{
				return false;
			}

			var tokens = source.Split(['_', '-', ' '], StringSplitOptions.RemoveEmptyEntries);
			return tokens.Any(token => token.Equals(front ? "F" : "R", StringComparison.OrdinalIgnoreCase) ||
			                           token.Equals(front ? "Front" : "Rear", StringComparison.OrdinalIgnoreCase) ||
			                           token.Equals(front ? "FL" : "RL", StringComparison.OrdinalIgnoreCase) ||
			                           token.Equals(front ? "FR" : "RR", StringComparison.OrdinalIgnoreCase));
		}

		private static bool AppliesToAxle(AssembledPressureWheelOptions option, bool front)
		{
			return MatchesAxle(option.SourceSlotType, front) || MatchesAxle(option.SourcePartName, front);
		}

		private static float ResolveLastPositive(IEnumerable<float?> values, float fallback)
		{
			return ResolveLastPositive(values) ?? fallback;
		}

		private static float? ResolveLastPositive(IEnumerable<float?> values)
		{
			float? resolved = null;
			foreach (var value in values)
			{
				if (value is > 0f and var positiveValue)
				{
					resolved = positiveValue;
				}
			}

			return resolved;
		}
	}
}
