using System;

namespace LibreRally.Telemetry
{
	internal sealed class LiveTuningWheelSnapshot
	{
		public string WheelId { get; init; } = string.Empty;
		public string SurfaceType { get; init; } = string.Empty;
		public float SuspensionCompressionMeters { get; init; }
		public float NormalLoadNewtons { get; init; }
		public float SlipRatio { get; init; }
		public float AngularVelocityRadPerSecond { get; init; }
		public float DriveTorqueNewtonMeters { get; init; }
		public float BrakeTorqueNewtonMeters { get; init; }
		public float TyreReactionTorqueNewtonMeters { get; init; }
		public float SurfaceFrictionCoefficient { get; init; }
		public float EffectivePeakFrictionCoefficient { get; init; }
	}

	internal sealed class LiveTuningSnapshot
	{
		public DateTimeOffset TimestampUtc { get; init; }
		public bool VehicleLoaded { get; init; }
		public string VehicleName { get; init; } = string.Empty;
		public string StatusText { get; init; } = string.Empty;
		public string VehicleFolderPath { get; init; } = string.Empty;
		public string ConfigFileName { get; init; } = string.Empty;
		public string TyreModelMode { get; init; } = "auto";
		public float PeakFrictionCoefficient { get; init; }
		public float LoadSensitivity { get; init; }
		public float SpeedKmh { get; init; }
		public float ForwardSpeedMs { get; init; }
		public float LateralSpeedMs { get; init; }
		public float YawRateRad { get; init; }
		public float EngineRpm { get; init; }
		public int CurrentGear { get; init; }
		public float ThrottleInput { get; init; }
		public float BrakeInput { get; init; }
		public float DriveInput { get; init; }
		public float SteeringInput { get; init; }
		public float DrivenWheelSlipRatio { get; init; }
		public bool TractionLossDetected { get; init; }
		public bool TractionControlEnabled { get; init; }
		public bool TractionControlActive { get; init; }
		public float TractionControlSlipRatioTarget { get; init; }
		public float TractionControlSlipRatioWindow { get; init; }
		public float TractionControlMinimumSpeedKmh { get; init; }
		public float TractionControlApplyRate { get; init; }
		public float TractionControlReleaseRate { get; init; }
		public float TractionControlMinTorqueScale { get; init; }
		public float TractionControlTorqueScale { get; init; }
		public bool AbsEnabled { get; init; }
		public bool AbsActive { get; init; }
		public float AbsSlipRatioTarget { get; init; }
		public float AbsSlipRatioWindow { get; init; }
		public float AbsMinBrakeTorqueScale { get; init; }
		public float AbsMinimumSpeedKmh { get; init; }
		public float AbsApplyRate { get; init; }
		public float AbsReleaseRate { get; init; }
		public float AutoClutchLaunchRpm { get; init; }
		public float AutoClutchWheelspinWindowRpm { get; init; }
		public float AutoClutchMinTorqueScale { get; init; }
		public float ShiftUpRpm { get; init; }
		public float ShiftDownRpm { get; init; }
		public LiveTuningWheelSnapshot[] Wheels { get; init; } = [];
	}

	internal sealed class LiveTuningPatch
	{
		public string? TyreModelMode { get; init; }
		public float? PeakFrictionCoefficient { get; init; }
		public float? LoadSensitivity { get; init; }
		public bool? TractionControlEnabled { get; init; }
		public float? TractionControlSlipRatioTarget { get; init; }
		public float? TractionControlSlipRatioWindow { get; init; }
		public float? TractionControlMinimumSpeedKmh { get; init; }
		public float? TractionControlApplyRate { get; init; }
		public float? TractionControlReleaseRate { get; init; }
		public float? TractionControlMinTorqueScale { get; init; }
		public bool? AbsEnabled { get; init; }
		public float? AbsSlipRatioTarget { get; init; }
		public float? AbsSlipRatioWindow { get; init; }
		public float? AbsMinBrakeTorqueScale { get; init; }
		public float? AbsMinimumSpeedKmh { get; init; }
		public float? AbsApplyRate { get; init; }
		public float? AbsReleaseRate { get; init; }
		public float? AutoClutchLaunchRpm { get; init; }
		public float? AutoClutchWheelspinWindowRpm { get; init; }
		public float? AutoClutchMinTorqueScale { get; init; }
		public float? ShiftUpRpm { get; init; }
		public float? ShiftDownRpm { get; init; }

		public bool HasChanges =>
			TyreModelMode != null ||
			PeakFrictionCoefficient.HasValue ||
			LoadSensitivity.HasValue ||
			TractionControlEnabled.HasValue ||
			TractionControlSlipRatioTarget.HasValue ||
			TractionControlSlipRatioWindow.HasValue ||
			TractionControlMinimumSpeedKmh.HasValue ||
			TractionControlApplyRate.HasValue ||
			TractionControlReleaseRate.HasValue ||
			TractionControlMinTorqueScale.HasValue ||
			AbsEnabled.HasValue ||
			AbsSlipRatioTarget.HasValue ||
			AbsSlipRatioWindow.HasValue ||
			AbsMinBrakeTorqueScale.HasValue ||
			AbsMinimumSpeedKmh.HasValue ||
			AbsApplyRate.HasValue ||
			AbsReleaseRate.HasValue ||
			AutoClutchLaunchRpm.HasValue ||
			AutoClutchWheelspinWindowRpm.HasValue ||
			AutoClutchMinTorqueScale.HasValue ||
			ShiftUpRpm.HasValue ||
			ShiftDownRpm.HasValue;
	}

	internal sealed class LiveTuningBridgeRequest
	{
		public string Command { get; init; } = string.Empty;
		public LiveTuningPatch? Patch { get; init; }
	}

	internal sealed class LiveTuningBridgeResponse
	{
		public bool Succeeded { get; init; }
		public string Command { get; init; } = string.Empty;
		public string Summary { get; init; } = string.Empty;
		public LiveTuningSnapshot? Snapshot { get; init; }
	}
}
