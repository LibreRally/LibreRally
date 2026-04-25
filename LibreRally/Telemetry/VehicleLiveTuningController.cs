using System;
using System.Collections.Generic;
using System.Globalization;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Physics;

namespace LibreRally.Telemetry
{
	internal static class VehicleLiveTuningController
	{
		public static LiveTuningSnapshot CreateSnapshot(
			LoadedVehicle? loadedVehicle,
			string vehicleName,
			string statusText,
			string vehicleFolderPath,
			string configFileName)
		{
			var car = loadedVehicle?.CarComponent;
			var dynamics = car?.Dynamics;
			var frontTyre = dynamics?.TyreModels[VehicleDynamicsSystem.FL];
			if (car == null || dynamics == null || frontTyre == null)
			{
				return new LiveTuningSnapshot
				{
					TimestampUtc = DateTimeOffset.UtcNow,
					VehicleLoaded = false,
					VehicleName = vehicleName,
					StatusText = statusText,
					VehicleFolderPath = vehicleFolderPath,
					ConfigFileName = configFileName,
				};
			}

			var activeVehicle = loadedVehicle!;

			return new LiveTuningSnapshot
			{
				TimestampUtc = DateTimeOffset.UtcNow,
				VehicleLoaded = true,
				VehicleName = vehicleName,
				StatusText = statusText,
				VehicleFolderPath = vehicleFolderPath,
				ConfigFileName = configFileName,
				TyreModelMode = FormatTyreModelMode(frontTyre.ActiveMode),
				PeakFrictionCoefficient = frontTyre.PeakFrictionCoefficient,
				LoadSensitivity = frontTyre.LoadSensitivity,
				SpeedKmh = car.SpeedKmh,
				ForwardSpeedMs = car.ForwardSpeedMs,
				LateralSpeedMs = car.LateralSpeedMs,
				YawRateRad = car.YawRateRad,
				EngineRpm = car.EngineRpm,
				CurrentGear = car.CurrentGear,
				ThrottleInput = car.ThrottleInput,
				BrakeInput = car.BrakeInput,
				DriveInput = car.DriveInput,
				SteeringInput = car.SteeringInput,
				DrivenWheelSlipRatio = car.DrivenWheelSlipRatio,
				TractionLossDetected = car.TractionLossDetected,
				TractionControlEnabled = car.TractionControlEnabled,
				TractionControlActive = car.TractionControlActive,
				TractionControlSlipRatioTarget = car.TractionControlSlipRatioTarget,
				TractionControlSlipRatioWindow = car.TractionControlSlipRatioWindow,
				TractionControlMinimumSpeedKmh = car.TractionControlMinimumSpeedKmh,
				TractionControlApplyRate = car.TractionControlApplyRate,
				TractionControlReleaseRate = car.TractionControlReleaseRate,
				TractionControlMinTorqueScale = car.TractionControlMinTorqueScale,
				TractionControlTorqueScale = car.TractionControlTorqueScale,
				AbsEnabled = car.AbsEnabled,
				AbsActive = car.AbsActive,
				AbsSlipRatioTarget = car.AbsSlipRatioTarget,
				AbsSlipRatioWindow = car.AbsSlipRatioWindow,
				AbsMinBrakeTorqueScale = car.AbsMinBrakeTorqueScale,
				AbsMinimumSpeedKmh = car.AbsMinimumSpeedKmh,
				AbsApplyRate = car.AbsApplyRate,
				AbsReleaseRate = car.AbsReleaseRate,
				AutoClutchLaunchRpm = car.AutoClutchLaunchRpm,
				AutoClutchWheelspinWindowRpm = car.AutoClutchWheelspinWindowRpm,
				AutoClutchMinTorqueScale = car.AutoClutchMinTorqueScale,
				ShiftUpRpm = car.ShiftUpRpm,
				ShiftDownRpm = car.ShiftDownRpm,
				Wheels =
				[
					CreateWheelSnapshot("FL", activeVehicle.WheelFL, dynamics, VehicleDynamicsSystem.FL),
					CreateWheelSnapshot("FR", activeVehicle.WheelFR, dynamics, VehicleDynamicsSystem.FR),
					CreateWheelSnapshot("RL", activeVehicle.WheelRL, dynamics, VehicleDynamicsSystem.RL),
					CreateWheelSnapshot("RR", activeVehicle.WheelRR, dynamics, VehicleDynamicsSystem.RR),
				],
			};
		}

		public static LiveTuningBridgeResponse ApplyPatch(
			LoadedVehicle? loadedVehicle,
			string vehicleName,
			string statusText,
			string vehicleFolderPath,
			string configFileName,
			LiveTuningPatch patch)
		{
			if (loadedVehicle?.CarComponent?.Dynamics == null)
			{
				return CreateFailedResponse("apply_patch", "No active vehicle is loaded.");
			}

			if (patch == null || !patch.HasChanges)
			{
				return CreateFailedResponse("apply_patch", "No live tuning changes were supplied.");
			}

			var car = loadedVehicle.CarComponent;
			var changes = new List<string>();

			if (patch.TyreModelMode != null)
			{
				if (!TryParseTyreModelMode(patch.TyreModelMode, out var mode))
				{
					return CreateFailedResponse("apply_patch", $"Unsupported tyre model mode '{patch.TyreModelMode}'.");
				}

				ApplyToAllTyres(car, tyre => tyre.ActiveMode = mode);
				changes.Add($"tyreModelMode={FormatTyreModelMode(mode)}");
			}

			if (!TryApplyFloat(patch.PeakFrictionCoefficient, 0.4f, 2.0f, value => ApplyToAllTyres(car, tyre => tyre.PeakFrictionCoefficient = value), "peakFrictionCoefficient", changes) ||
			    !TryApplyFloat(patch.LoadSensitivity, 0f, 0.4f, value => ApplyToAllTyres(car, tyre => tyre.LoadSensitivity = value), "loadSensitivity", changes) ||
			    !TryApplyFloat(patch.TractionControlSlipRatioTarget, 0.02f, 0.5f, value => car.TractionControlSlipRatioTarget = value, "tcsSlipRatioTarget", changes) ||
			    !TryApplyFloat(patch.TractionControlSlipRatioWindow, 0.01f, 0.4f, value => car.TractionControlSlipRatioWindow = value, "tcsSlipRatioWindow", changes) ||
			    !TryApplyFloat(patch.TractionControlMinimumSpeedKmh, 0f, 30f, value => car.TractionControlMinimumSpeedKmh = value, "tcsMinimumSpeedKmh", changes) ||
			    !TryApplyFloat(patch.TractionControlApplyRate, 1f, 60f, value => car.TractionControlApplyRate = value, "tcsApplyRate", changes) ||
			    !TryApplyFloat(patch.TractionControlReleaseRate, 0.5f, 30f, value => car.TractionControlReleaseRate = value, "tcsReleaseRate", changes) ||
			    !TryApplyFloat(patch.TractionControlMinTorqueScale, 0f, 0.5f, value => car.TractionControlMinTorqueScale = value, "tcsMinTorqueScale", changes) ||
			    !TryApplyFloat(patch.AbsSlipRatioTarget, 0.02f, 0.5f, value => car.AbsSlipRatioTarget = value, "absSlipRatioTarget", changes) ||
			    !TryApplyFloat(patch.AbsSlipRatioWindow, 0.01f, 0.4f, value => car.AbsSlipRatioWindow = value, "absSlipRatioWindow", changes) ||
			    !TryApplyFloat(patch.AbsMinBrakeTorqueScale, 0f, 0.5f, value => car.AbsMinBrakeTorqueScale = value, "absMinBrakeTorqueScale", changes) ||
			    !TryApplyFloat(patch.AbsMinimumSpeedKmh, 0f, 20f, value => car.AbsMinimumSpeedKmh = value, "absMinimumSpeedKmh", changes) ||
			    !TryApplyFloat(patch.AbsApplyRate, 1f, 80f, value => car.AbsApplyRate = value, "absApplyRate", changes) ||
			    !TryApplyFloat(patch.AbsReleaseRate, 0.5f, 40f, value => car.AbsReleaseRate = value, "absReleaseRate", changes) ||
			    !TryApplyFloat(patch.AutoClutchLaunchRpm, 1000f, 8000f, value => car.AutoClutchLaunchRpm = value, "autoClutchLaunchRpm", changes) ||
			    !TryApplyFloat(patch.AutoClutchWheelspinWindowRpm, 100f, 4000f, value => car.AutoClutchWheelspinWindowRpm = value, "autoClutchWheelspinWindowRpm", changes) ||
			    !TryApplyFloat(patch.AutoClutchMinTorqueScale, 0f, 1f, value => car.AutoClutchMinTorqueScale = value, "autoClutchMinTorqueScale", changes) ||
			    !TryApplyFloat(patch.ShiftUpRpm, 2000f, 9000f, value => car.ShiftUpRpm = value, "shiftUpRpm", changes) ||
			    !TryApplyFloat(patch.ShiftDownRpm, 500f, 6000f, value => car.ShiftDownRpm = value, "shiftDownRpm", changes))
			{
				return CreateFailedResponse("apply_patch", changes[^1]);
			}

			if (patch.TractionControlEnabled.HasValue)
			{
				car.TractionControlEnabled = patch.TractionControlEnabled.Value;
				changes.Add($"tractionControlEnabled={patch.TractionControlEnabled.Value.ToString(CultureInfo.InvariantCulture).ToLowerInvariant()}");
			}

			if (patch.AbsEnabled.HasValue)
			{
				car.AbsEnabled = patch.AbsEnabled.Value;
				changes.Add($"absEnabled={patch.AbsEnabled.Value.ToString(CultureInfo.InvariantCulture).ToLowerInvariant()}");
			}

			var snapshot = CreateSnapshot(loadedVehicle, vehicleName, statusText, vehicleFolderPath, configFileName);
			return new LiveTuningBridgeResponse
			{
				Succeeded = true,
				Command = "apply_patch",
				Summary = $"Applied live tuning changes: {string.Join(", ", changes)}.",
				Snapshot = snapshot,
			};
		}

		private static LiveTuningBridgeResponse CreateFailedResponse(string command, string message)
		{
			return new LiveTuningBridgeResponse
			{
				Succeeded = false,
				Command = command,
				Summary = message,
			};
		}

		private static bool TryApplyFloat(float? value, float min, float max, Action<float> apply, string label, ICollection<string> changes)
		{
			if (!value.HasValue)
			{
				return true;
			}

			if (!float.IsFinite(value.Value) || value.Value < min || value.Value > max)
			{
				changes.Add($"Invalid {label} value '{value.Value.ToString(CultureInfo.InvariantCulture)}' (expected {min.ToString(CultureInfo.InvariantCulture)}-{max.ToString(CultureInfo.InvariantCulture)}).");
				return false;
			}

			apply(value.Value);
			changes.Add($"{label}={value.Value.ToString("0.###", CultureInfo.InvariantCulture)}");
			return true;
		}

		private static void ApplyToAllTyres(RallyCarComponent car, Action<TyreModel> apply)
		{
			if (car.Dynamics == null)
			{
				return;
			}

			for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
			{
				if (car.Dynamics.TyreModels[i] is { } tyre)
				{
					apply(tyre);
				}
			}
		}

		private static LiveTuningWheelSnapshot CreateWheelSnapshot(string wheelId, Stride.Engine.Entity wheelEntity, VehicleDynamicsSystem dynamics, int wheelIndex)
		{
			var wheelSettings = wheelEntity.Get<WheelSettings>();
			var wheelState = dynamics.WheelStates[wheelIndex];
			return new LiveTuningWheelSnapshot
			{
				WheelId = wheelId,
				SurfaceType = (wheelSettings?.CurrentSurface ?? SurfaceType.Tarmac).ToString(),
				SuspensionCompressionMeters = dynamics.SuspensionCompression[wheelIndex],
				NormalLoadNewtons = dynamics.CurrentNormalLoads[wheelIndex],
				SlipRatio = wheelState.SlipRatio,
				AngularVelocityRadPerSecond = wheelState.AngularVelocity,
				DriveTorqueNewtonMeters = wheelState.DriveTorque,
				BrakeTorqueNewtonMeters = wheelState.BrakeTorque,
				TyreReactionTorqueNewtonMeters = wheelState.TyreReactionTorque,
				SurfaceFrictionCoefficient = dynamics.WheelSurfaces[wheelIndex].FrictionCoefficient,
				EffectivePeakFrictionCoefficient = dynamics.EffectivePeakFrictionCoefficients[wheelIndex],
			};
		}

		private static string FormatTyreModelMode(TyreModelMode mode)
		{
			return mode switch
			{
				TyreModelMode.BrushOnly => "brush_only",
				TyreModelMode.PacejkaOnly => "pacejka_only",
				_ => "auto",
			};
		}

		private static bool TryParseTyreModelMode(string value, out TyreModelMode mode)
		{
			switch (value.Trim().ToLowerInvariant())
			{
				case "auto":
				case "blended":
					mode = TyreModelMode.Auto;
					return true;
				case "brush":
				case "brush_only":
					mode = TyreModelMode.BrushOnly;
					return true;
				case "pacejka":
				case "pacejka_only":
					mode = TyreModelMode.PacejkaOnly;
					return true;
				default:
					mode = TyreModelMode.Auto;
					return false;
			}
		}
	}
}
