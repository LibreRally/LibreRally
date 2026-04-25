using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;

namespace LibreRally.Vehicle
{
	public sealed record VehicleTroubleshootingSetupValue(
		string Name,
		float? PreferredValue,
		float? MergedValue,
		float? EffectiveValue,
		string EffectiveSource);

	public sealed record VehicleTroubleshootingPowertrainSnapshot(
		float VehicleMassKg,
		int DrivenWheelCount,
		float LaunchRpm,
		float LaunchEngineTorqueNm,
		float FirstGearRatio,
		float FinalDriveRatio,
		float PerDrivenWheelLaunchTorqueNm);

	public sealed record VehicleTroubleshootingTyreProbe(
		string VariantName,
		TyreModelMode Mode,
		bool NeutralTyreControl,
		float FinalSpeedMs,
		float DistanceTravelledMeters,
		float PeakAccelerationMs2,
		float PeakSlipRatio,
		float PeakLongitudinalForcePerDrivenWheelN);

	public sealed class VehicleTroubleshootingReport
	{
		public string VehicleFolderPath { get; init; } = string.Empty;
		public string? ConfigPath { get; init; }
		public IReadOnlyList<VehicleTroubleshootingSetupValue> SetupValues { get; init; } = [];
		public VehicleTroubleshootingPowertrainSnapshot Powertrain { get; init; } = new(0f, 0, 0f, 0f, 0f, 0f, 0f);
		public IReadOnlyList<VehicleTroubleshootingTyreProbe> TyreProbes { get; init; } = [];
		public IReadOnlyList<string> Observations { get; init; } = [];

		public string Format()
		{
			var builder = new StringBuilder();
			builder.AppendLine($"Vehicle folder: {VehicleFolderPath}");
			builder.AppendLine($"Config: {(string.IsNullOrWhiteSpace(ConfigPath) ? "<none>" : ConfigPath)}");
			builder.AppendLine($"Mass: {Powertrain.VehicleMassKg:F0} kg | Driven wheels: {Powertrain.DrivenWheelCount} | Launch RPM: {Powertrain.LaunchRpm:F0}");
			builder.AppendLine($"Launch torque: engine={Powertrain.LaunchEngineTorqueNm:F0} Nm gear1={Powertrain.FirstGearRatio:F2} final={Powertrain.FinalDriveRatio:F2} per-driven-wheel={Powertrain.PerDrivenWheelLaunchTorqueNm:F0} Nm");
			builder.AppendLine("Setup values:");
			foreach (var setupValue in SetupValues)
			{
				builder.AppendLine(
					$"  {setupValue.Name}: preferred={FormatNullable(setupValue.PreferredValue)} merged={FormatNullable(setupValue.MergedValue)} " +
					$"effective={FormatNullable(setupValue.EffectiveValue)} source={setupValue.EffectiveSource}");
			}

			builder.AppendLine("Tyre probes:");
			foreach (var probe in TyreProbes)
			{
				builder.AppendLine(
					$"  {probe.VariantName}: mode={probe.Mode} neutralTyre={probe.NeutralTyreControl} " +
					$"finalV={probe.FinalSpeedMs:F2}m/s dist={probe.DistanceTravelledMeters:F2}m " +
					$"peakAx={probe.PeakAccelerationMs2:F2}m/s² peakSlip={probe.PeakSlipRatio:F2} peakFx/wheel={probe.PeakLongitudinalForcePerDrivenWheelN:F0}N");
			}

			builder.AppendLine("Observations:");
			foreach (var observation in Observations)
			{
				builder.AppendLine($"  - {observation}");
			}

			return builder.ToString();
		}

		private static string FormatNullable(float? value) => value.HasValue ? value.Value.ToString("F3") : "<none>";
	}

	public static class VehicleTroubleshootingReportBuilder
	{
		private const float DrivelineEfficiency = 0.90f;
		private const float ProbeDurationSeconds = 1.0f;
		private const float ProbeTimeStepSeconds = 0.01f;

		private static readonly (string Name, bool Positive)[] SetupAuditValues =
		[
			("spring_F", true),
			("spring_R", true),
			("damp_bump_F", true),
			("damp_rebound_F", true),
			("damp_bump_R", true),
			("damp_rebound_R", true),
			("camber_F", false),
			("camber_R", false),
			("springheight_F", false),
			("springheight_R", false),
			("trackoffset_F", false),
			("trackoffset_R", false),
			("toe_F", false),
			("toe_R", false),
		];

		public static VehicleTroubleshootingReport Build(string vehicleFolderPath, string? configFileName = null)
		{
			if (!Directory.Exists(vehicleFolderPath))
			{
				throw new DirectoryNotFoundException($"Vehicle folder not found: '{vehicleFolderPath}'.");
			}

			var configPath = PcConfigLoader.FindBestConfig(vehicleFolderPath, configFileName);
			var pcConfig = configPath != null ? PcConfigLoader.Load(configPath) : null;
			var definition = JBeamAssembler.Assemble(vehicleFolderPath, pcConfig);
			var powertrain = VehiclePowertrainResolver.Resolve(definition);
			var vehicleMass = EstimateVehicleMass(definition);
			var setupValues = BuildSetupValues(definition, pcConfig);
			var launchSnapshot = BuildPowertrainSnapshot(powertrain, vehicleMass);
			var frontTyreSpec = VehicleTyreSpecResolver.Resolve(definition, front: true);
			var rearTyreSpec = VehicleTyreSpecResolver.Resolve(definition, front: false);
			var tyreProbes = BuildTyreProbes(frontTyreSpec, rearTyreSpec, powertrain, vehicleMass);
			var observations = BuildObservations(setupValues, tyreProbes, powertrain);

			return new VehicleTroubleshootingReport
			{
				VehicleFolderPath = vehicleFolderPath,
				ConfigPath = configPath,
				SetupValues = setupValues,
				Powertrain = launchSnapshot,
				TyreProbes = tyreProbes,
				Observations = observations,
			};
		}

		private static IReadOnlyList<VehicleTroubleshootingSetupValue> BuildSetupValues(VehicleDefinition definition, PcConfig? pcConfig)
		{
			var values = new List<VehicleTroubleshootingSetupValue>(SetupAuditValues.Length);
			IReadOnlyDictionary<string, float>? preferredVars = pcConfig?.Vars;
			foreach (var (name, positive) in SetupAuditValues)
			{
				float? effectiveValue;
				string effectiveSource;
				if (positive)
				{
					effectiveValue = VehicleSetupValueResolver.TryResolvePositiveValue(preferredVars, definition.Vars, out var resolved, out effectiveSource, name)
						? resolved
						: null;
				}
				else
				{
					effectiveValue = VehicleSetupValueResolver.TryResolveFiniteValue(preferredVars, definition.Vars, out var resolved, out effectiveSource, name)
						? resolved
						: null;
				}

				values.Add(new VehicleTroubleshootingSetupValue(
					Name: name,
					PreferredValue: ResolveConfiguredValue(preferredVars, name),
					MergedValue: ResolveConfiguredValue(definition.Vars, name),
					EffectiveValue: effectiveValue,
					EffectiveSource: effectiveValue.HasValue ? effectiveSource : "unresolved"));
			}

			return values;
		}

		private static VehicleTroubleshootingPowertrainSnapshot BuildPowertrainSnapshot(VehiclePowertrainSetup powertrain, float vehicleMass)
		{
			var drivenWheelCount = Math.Max(powertrain.DrivenWheelKeys.Length, 1);
			var launchRpm = Math.Clamp(
				powertrain.AutoClutchLaunchRpm > 0f ? powertrain.AutoClutchLaunchRpm : powertrain.IdleRpm,
				Math.Max(powertrain.IdleRpm, powertrain.TorqueCurveRpm.FirstOrDefault()),
				Math.Max(powertrain.MaxRpm, powertrain.TorqueCurveRpm.LastOrDefault()));
			var engineTorque = InterpolateTorqueCurve(powertrain, launchRpm);
			var firstGearRatio = ResolveFirstForwardGearRatio(powertrain);
			var perDrivenWheelTorque = engineTorque * firstGearRatio * powertrain.FinalDrive * DrivelineEfficiency / drivenWheelCount;

			return new VehicleTroubleshootingPowertrainSnapshot(
				VehicleMassKg: vehicleMass,
				DrivenWheelCount: drivenWheelCount,
				LaunchRpm: launchRpm,
				LaunchEngineTorqueNm: engineTorque,
				FirstGearRatio: firstGearRatio,
				FinalDriveRatio: powertrain.FinalDrive,
				PerDrivenWheelLaunchTorqueNm: perDrivenWheelTorque);
		}

		private static IReadOnlyList<VehicleTroubleshootingTyreProbe> BuildTyreProbes(
			in VehicleTyreSpec frontTyreSpec,
			in VehicleTyreSpec rearTyreSpec,
			VehiclePowertrainSetup powertrain,
			float vehicleMass)
		{
			var probes = new List<VehicleTroubleshootingTyreProbe>(4)
			{
				RunTyreProbe("Loaded Auto", TyreModelMode.Auto, neutralTyreControl: false, frontTyreSpec, rearTyreSpec, powertrain, vehicleMass),
				RunTyreProbe("Loaded Brush", TyreModelMode.BrushOnly, neutralTyreControl: false, frontTyreSpec, rearTyreSpec, powertrain, vehicleMass),
				RunTyreProbe("Loaded Pacejka", TyreModelMode.PacejkaOnly, neutralTyreControl: false, frontTyreSpec, rearTyreSpec, powertrain, vehicleMass),
				RunTyreProbe("Neutral Tyre Auto", TyreModelMode.Auto, neutralTyreControl: true, CreateNeutralTyreSpec(frontTyreSpec), CreateNeutralTyreSpec(rearTyreSpec), powertrain, vehicleMass),
			};

			return probes;
		}

		private static VehicleTroubleshootingTyreProbe RunTyreProbe(
			string variantName,
			TyreModelMode mode,
			bool neutralTyreControl,
			in VehicleTyreSpec frontTyreSpec,
			in VehicleTyreSpec rearTyreSpec,
			VehiclePowertrainSetup powertrain,
			float vehicleMass)
		{
			var drivenWheelKeys = powertrain.DrivenWheelKeys.Length > 0
				? powertrain.DrivenWheelKeys
				: ["wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR"];
			var launchSnapshot = BuildPowertrainSnapshot(powertrain, vehicleMass);
			var wheelLoad = Math.Max(vehicleMass * 9.81f / 4f, 500f);
			var wheelStates = new Dictionary<string, TyreState>(StringComparer.OrdinalIgnoreCase);
			var tyreModels = new Dictionary<string, TyreModel>(StringComparer.OrdinalIgnoreCase);
			foreach (var wheelKey in drivenWheelKeys)
			{
				var spec = IsFrontWheel(wheelKey) ? frontTyreSpec : rearTyreSpec;
				var tyreModel = VehicleLoader.CreateTyreModel(spec);
				tyreModel.ActiveMode = mode;
				tyreModels[wheelKey] = tyreModel;
				wheelStates[wheelKey] = TyreState.CreateDefault();
			}

			float vehicleSpeed = 0f;
			float distanceTravelled = 0f;
			float peakAcceleration = 0f;
			float peakSlipRatio = 0f;
			float peakLongitudinalForcePerWheel = 0f;
			var surface = SurfaceProperties.ForType(SurfaceType.Tarmac);
			var stepCount = (int)MathF.Ceiling(ProbeDurationSeconds / ProbeTimeStepSeconds);

			for (var step = 0; step < stepCount; step++)
			{
				var totalForce = 0f;
				foreach (var wheelKey in drivenWheelKeys)
				{
					var tyreModel = tyreModels[wheelKey];
					var state = wheelStates[wheelKey];
					tyreModel.Update(
						ref state,
						longitudinalVelocity: vehicleSpeed,
						lateralVelocity: 0f,
						normalLoad: wheelLoad,
						driveTorque: launchSnapshot.PerDrivenWheelLaunchTorqueNm,
						brakeTorque: 0f,
						camberAngle: 0f,
						surface,
						ProbeTimeStepSeconds,
						out var longitudinalForce,
						out _,
						out _,
						out _,
						out _);
					wheelStates[wheelKey] = state;
					totalForce += longitudinalForce;
					peakSlipRatio = MathF.Max(peakSlipRatio, MathF.Abs(state.SlipRatio));
					peakLongitudinalForcePerWheel = MathF.Max(peakLongitudinalForcePerWheel, MathF.Abs(longitudinalForce));
				}

				var acceleration = totalForce / MathF.Max(vehicleMass, 1f);
				peakAcceleration = MathF.Max(peakAcceleration, acceleration);
				vehicleSpeed = MathF.Max(0f, vehicleSpeed + acceleration * ProbeTimeStepSeconds);
				distanceTravelled += vehicleSpeed * ProbeTimeStepSeconds;
			}

			return new VehicleTroubleshootingTyreProbe(
				VariantName: variantName,
				Mode: mode,
				NeutralTyreControl: neutralTyreControl,
				FinalSpeedMs: vehicleSpeed,
				DistanceTravelledMeters: distanceTravelled,
				PeakAccelerationMs2: peakAcceleration,
				PeakSlipRatio: peakSlipRatio,
				PeakLongitudinalForcePerDrivenWheelN: peakLongitudinalForcePerWheel);
		}

		private static IReadOnlyList<string> BuildObservations(
			IReadOnlyList<VehicleTroubleshootingSetupValue> setupValues,
			IReadOnlyList<VehicleTroubleshootingTyreProbe> probes,
			VehiclePowertrainSetup powertrain)
		{
			var observations = new List<string>();
			var overridingSetupValues = setupValues
				.Where(value =>
					value.PreferredValue.HasValue &&
					value.EffectiveValue.HasValue &&
					!string.Equals(value.EffectiveSource, "unresolved", StringComparison.OrdinalIgnoreCase) &&
					MathF.Abs(value.PreferredValue.Value - value.EffectiveValue.Value) < 1e-3f)
				.Select(value => $"{value.Name}<-{value.EffectiveSource}")
				.ToArray();
			if (overridingSetupValues.Length > 0)
			{
				observations.Add($"Active .pc overrides are winning for: {string.Join(", ", overridingSetupValues)}.");
			}

			var loadedAuto = probes.FirstOrDefault(probe => probe.VariantName == "Loaded Auto");
			var loadedBrush = probes.FirstOrDefault(probe => probe.VariantName == "Loaded Brush");
			var loadedPacejka = probes.FirstOrDefault(probe => probe.VariantName == "Loaded Pacejka");
			var neutralAuto = probes.FirstOrDefault(probe => probe.VariantName == "Neutral Tyre Auto");

			if (loadedBrush != null && loadedPacejka != null)
			{
				var speedDelta = MathF.Abs(loadedBrush.FinalSpeedMs - loadedPacejka.FinalSpeedMs);
				if (speedDelta > 0.25f)
				{
					observations.Add($"Brush vs Pacejka materially changes the launch probe ({loadedBrush.FinalSpeedMs:F2} vs {loadedPacejka.FinalSpeedMs:F2} m/s after {ProbeDurationSeconds:F1}s), pointing at tyre-model sensitivity.");
				}
				else
				{
					observations.Add("Brush and Pacejka stay close in the launch probe, which points away from a tyre-mode-specific bug.");
				}
			}

			if (loadedAuto != null && neutralAuto != null)
			{
				if (neutralAuto.FinalSpeedMs > loadedAuto.FinalSpeedMs + 0.25f)
				{
					observations.Add($"Neutral tyre control launches harder than the loaded tyre data ({neutralAuto.FinalSpeedMs:F2} vs {loadedAuto.FinalSpeedMs:F2} m/s), suggesting data tuning or mapping is limiting grip/response more than the core drivetrain.");
				}
				else if (loadedAuto.FinalSpeedMs > 0f && neutralAuto.FinalSpeedMs <= loadedAuto.FinalSpeedMs + 0.25f)
				{
					observations.Add("Neutral tyre control does not dramatically outperform the loaded tyre data, so the slowdown is less likely to be caused solely by tyre-parameter tuning.");
				}
			}

			if (loadedAuto != null &&
			    neutralAuto != null &&
			    loadedBrush != null &&
			    loadedPacejka != null &&
			    loadedAuto.FinalSpeedMs < 1.0f &&
			    neutralAuto.FinalSpeedMs < 1.0f &&
			    loadedBrush.FinalSpeedMs < 1.0f &&
			    loadedPacejka.FinalSpeedMs < 1.0f)
			{
				observations.Add("All probe variants remain slow despite positive launch torque, which points toward a broader drivetrain/vehicle-physics foundation issue rather than a single tyre-mode or setup-value problem.");
			}

			if (powertrain.DrivenWheelKeys.Length == 0)
			{
				observations.Add("Driven wheel resolution fell back to defaults; investigate the assembled powertrain graph before trusting launch results.");
			}

			return observations;
		}

		private static float? ResolveConfiguredValue(IReadOnlyDictionary<string, float>? vars, string baseName)
		{
			if (vars == null)
			{
				return null;
			}

			return VehicleSetupValueResolver.TryResolveFiniteValue(vars, out var value, out _, baseName) ? value : null;
		}

		private static float EstimateVehicleMass(VehicleDefinition definition)
		{
			var mass = definition.Nodes.Values.Sum(node => node.Weight);
			return mass > 100f ? mass : 1200f;
		}

		private static float ResolveFirstForwardGearRatio(VehiclePowertrainSetup powertrain)
		{
			if (powertrain.GearRatios.Length > 1 && MathF.Abs(powertrain.GearRatios[1]) > 1e-4f)
			{
				return MathF.Abs(powertrain.GearRatios[1]);
			}

			return powertrain.GearRatios.Length > 0 ? MathF.Abs(powertrain.GearRatios[0]) : 3.0f;
		}

		private static float InterpolateTorqueCurve(VehiclePowertrainSetup powertrain, float rpm)
		{
			if (powertrain.TorqueCurveRpm.Length < 2 || powertrain.TorqueCurveNm.Length < powertrain.TorqueCurveRpm.Length)
			{
				return 0f;
			}

			rpm = Math.Clamp(rpm, powertrain.TorqueCurveRpm[0], powertrain.TorqueCurveRpm[^1]);
			for (var i = 1; i < powertrain.TorqueCurveRpm.Length; i++)
			{
				if (rpm <= powertrain.TorqueCurveRpm[i])
				{
					var range = powertrain.TorqueCurveRpm[i] - powertrain.TorqueCurveRpm[i - 1];
					if (MathF.Abs(range) < 1e-4f)
					{
						return powertrain.TorqueCurveNm[i];
					}

					var t = (rpm - powertrain.TorqueCurveRpm[i - 1]) / range;
					return powertrain.TorqueCurveNm[i - 1] + t * (powertrain.TorqueCurveNm[i] - powertrain.TorqueCurveNm[i - 1]);
				}
			}

			return powertrain.TorqueCurveNm[^1];
		}

		private static VehicleTyreSpec CreateNeutralTyreSpec(in VehicleTyreSpec source)
		{
			return new VehicleTyreSpec(
				Radius: source.Radius,
				Width: source.Width,
				PressureKpa: source.PressureKpa,
				PeakFrictionCoefficient: VehicleTyreSpecResolver.DefaultPeakFrictionCoefficient,
				RollingResistanceCoefficient: VehicleTyreSpecResolver.DefaultRollingResistanceCoefficient);
		}

		private static bool IsFrontWheel(string wheelKey)
		{
			return wheelKey.Contains("_FL", StringComparison.OrdinalIgnoreCase) ||
			       wheelKey.Contains("_FR", StringComparison.OrdinalIgnoreCase);
		}
	}
}
