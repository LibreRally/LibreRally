using System;
using System.Collections.Generic;
using System.Linq;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using Stride.BepuPhysics.Constraints;
using Stride.Engine;

namespace LibreRally.Vehicle;

public static class VehicleSetupRuntime
{
    private const float SuspensionTargetOffsetLimit = 0.08f;
    private const float Gravity = 9.81f;

    public static void ApplyOverridesToConfig(PcConfig? config, VehicleSetupOverrides? setupOverrides)
    {
        if (config?.Vars == null || setupOverrides == null)
        {
            return;
        }

        foreach (var kv in setupOverrides.VariableOverrides)
        {
            config.Vars[kv.Key] = kv.Value;
        }
    }

    public static void ApplyOverridesToDefinition(VehicleDefinition definition, VehicleSetupOverrides? setupOverrides)
    {
        if (setupOverrides == null)
        {
            return;
        }

        foreach (var kv in setupOverrides.VariableOverrides)
        {
            definition.Vars[kv.Key] = kv.Value;
        }

        foreach (var kv in setupOverrides.PressureWheelOverrides)
        {
            ApplyPressureWheelOverride(definition, kv.Key, kv.Value);
        }

        RefreshSetupValues(definition);
    }

    public static void RefreshSetupValues(VehicleDefinition definition)
    {
        foreach (var variable in definition.Setup.Variables.Values)
        {
            if (variable.IsSynthetic)
            {
                variable.Value = ResolveSyntheticValue(definition, variable);
                continue;
            }

            VehicleSetupSupport.PopulateRuntimeMetadata(variable, definition.Vars);
            if (variable.TargetKind == VehicleSetupTargetKind.AlignmentCamber &&
                !VehicleSetupSupport.IsLiveCamberVariable(variable))
            {
                variable.ApplyMode = VehicleSetupApplyMode.Reload;
            }
        }
    }

    public static VehicleSetupApplyResult Apply(LoadedVehicle vehicle, IReadOnlyDictionary<string, float> requestedValues)
    {
        var result = new VehicleSetupApplyResult();

        foreach (var kv in requestedValues)
        {
            if (!vehicle.Definition.Setup.Variables.TryGetValue(kv.Key, out var variable))
            {
                result.UnknownKeys.Add(kv.Key);
                continue;
            }

            if (variable.IsSynthetic)
            {
                ApplySyntheticValue(vehicle, variable, kv.Value, result);
                continue;
            }

            var effectiveValueName = string.IsNullOrWhiteSpace(variable.EffectiveValueName)
                ? variable.Name
                : variable.EffectiveValueName;
            vehicle.Definition.Vars[effectiveValueName] = kv.Value;
            vehicle.Config?.Vars.TryAdd(effectiveValueName, kv.Value);
            if (vehicle.Config != null)
            {
                vehicle.Config.Vars[effectiveValueName] = kv.Value;
            }

            vehicle.SetupOverrides.VariableOverrides[effectiveValueName] = kv.Value;
            vehicle.LoadRequest.VariableOverrides[effectiveValueName] = kv.Value;
            variable.Value = kv.Value;

            if (variable.ApplyMode == VehicleSetupApplyMode.Live)
            {
                ApplyLiveVariable(vehicle, variable, kv.Value);
                result.LiveAppliedKeys.Add(variable.Name);
            }
            else
            {
                result.ReloadRequiredKeys.Add(variable.Name);
            }
        }

        ApplyCurrentLiveState(vehicle);
        vehicle.CarComponent.RuntimeSetup = VehicleRuntimeSetupState.From(vehicle.Definition.Setup);
        return result;
    }

    private static void ApplySyntheticValue(LoadedVehicle vehicle, VehicleSetupVariable variable, float value, VehicleSetupApplyResult result)
    {
        var axle = variable.Axle == VehicleSetupAxle.Unspecified ? VehicleSetupAxle.Both : variable.Axle;
        if (!vehicle.SetupOverrides.PressureWheelOverrides.TryGetValue(axle, out var overrideState))
        {
            overrideState = new VehiclePressureWheelOverrides();
            vehicle.SetupOverrides.PressureWheelOverrides[axle] = overrideState;
        }

        if (!vehicle.LoadRequest.SetupOverrides.PressureWheelOverrides.TryGetValue(axle, out var loadRequestOverrideState))
        {
            loadRequestOverrideState = new VehiclePressureWheelOverrides();
            vehicle.LoadRequest.SetupOverrides.PressureWheelOverrides[axle] = loadRequestOverrideState;
        }

        switch (variable.TargetKind)
        {
            case VehicleSetupTargetKind.WheelTirePressure:
                overrideState.PressurePsi = value;
                loadRequestOverrideState.PressurePsi = value;
                ApplyPressureWheelOverride(vehicle.Definition, axle, overrideState);
                ApplyLivePressure(vehicle, axle, value);
                result.LiveAppliedKeys.Add(variable.Name);
                break;
            case VehicleSetupTargetKind.WheelOffset:
                overrideState.WheelOffset = value;
                loadRequestOverrideState.WheelOffset = value;
                ApplyPressureWheelOverride(vehicle.Definition, axle, overrideState);
                result.ReloadRequiredKeys.Add(variable.Name);
                break;
            case VehicleSetupTargetKind.WheelRadius:
                overrideState.Radius = value;
                loadRequestOverrideState.Radius = value;
                ApplyPressureWheelOverride(vehicle.Definition, axle, overrideState);
                result.ReloadRequiredKeys.Add(variable.Name);
                break;
            case VehicleSetupTargetKind.WheelWidth:
                overrideState.TireWidth = value;
                loadRequestOverrideState.TireWidth = value;
                ApplyPressureWheelOverride(vehicle.Definition, axle, overrideState);
                result.ReloadRequiredKeys.Add(variable.Name);
                break;
            case VehicleSetupTargetKind.HubRadius:
                overrideState.HubRadius = value;
                loadRequestOverrideState.HubRadius = value;
                ApplyPressureWheelOverride(vehicle.Definition, axle, overrideState);
                result.ReloadRequiredKeys.Add(variable.Name);
                break;
            case VehicleSetupTargetKind.HubWidth:
                overrideState.HubWidth = value;
                loadRequestOverrideState.HubWidth = value;
                ApplyPressureWheelOverride(vehicle.Definition, axle, overrideState);
                result.ReloadRequiredKeys.Add(variable.Name);
                break;
            default:
                result.UnknownKeys.Add(variable.Name);
                return;
        }

        variable.Value = value;
    }

    private static void ApplyLiveVariable(LoadedVehicle vehicle, VehicleSetupVariable variable, float value)
    {
        switch (variable.TargetKind)
        {
            case VehicleSetupTargetKind.SuspensionSpringRate:
            case VehicleSetupTargetKind.SuspensionBumpDamping:
            case VehicleSetupTargetKind.SuspensionReboundDamping:
            case VehicleSetupTargetKind.SuspensionRideHeight:
                ApplyLiveSuspension(vehicle, variable);
                break;
            case VehicleSetupTargetKind.SuspensionAntiRollBar:
                ApplyLiveAntiRollBar(vehicle, variable, value);
                break;
            case VehicleSetupTargetKind.AlignmentCamber:
                ApplyLiveCamber(vehicle, variable);
                break;
            case VehicleSetupTargetKind.BrakeBias:
            case VehicleSetupTargetKind.BrakeStrength:
                ApplyLiveBrakes(vehicle);
                break;
            case VehicleSetupTargetKind.DifferentialFinalDrive:
            case VehicleSetupTargetKind.DifferentialPreload:
            case VehicleSetupTargetKind.DifferentialPowerLock:
            case VehicleSetupTargetKind.DifferentialCoastLock:
            case VehicleSetupTargetKind.DifferentialTorqueSplit:
            case VehicleSetupTargetKind.DifferentialMaxLockingTorque:
            case VehicleSetupTargetKind.DifferentialSlipVsThrottleBias:
            case VehicleSetupTargetKind.DifferentialSlipAllowance:
            case VehicleSetupTargetKind.DifferentialAccelerationLock:
            case VehicleSetupTargetKind.DifferentialBrakeLock:
            case VehicleSetupTargetKind.DifferentialLeftFootBrakeUnlock:
            case VehicleSetupTargetKind.DifferentialBrakeLockInput:
            case VehicleSetupTargetKind.DifferentialTurnInUnlock:
            case VehicleSetupTargetKind.DifferentialTurnInUnlockSteeringAngle:
            case VehicleSetupTargetKind.TransmissionGearRatio:
                ApplyLivePowertrain(vehicle);
                break;
            case VehicleSetupTargetKind.EngineRevLimiterRpm:
            case VehicleSetupTargetKind.EngineRevLimiterCutTime:
            case VehicleSetupTargetKind.EngineWastegatePressure:
                ApplyLiveEngine(vehicle);
                break;
        }
    }

    public static void ApplyCurrentLiveState(LoadedVehicle vehicle)
    {
        ApplyLiveBrakes(vehicle);
        ApplyLivePowertrain(vehicle);
        ApplyLiveEngine(vehicle);
        vehicle.CarComponent.RuntimeSetup = VehicleRuntimeSetupState.From(vehicle.Definition.Setup);
    }

    private static void ApplyLiveSuspension(LoadedVehicle vehicle, VehicleSetupVariable changedVariable)
    {
        foreach (var axle in ResolveAffectedAxles(changedVariable.Axle))
        {
            var springVariable = FindFirstVariable(vehicle.Definition, axle, VehicleSetupTargetKind.SuspensionSpringRate);
            var bumpVariable = FindFirstVariable(vehicle.Definition, axle, VehicleSetupTargetKind.SuspensionBumpDamping);
            var reboundVariable = FindFirstVariable(vehicle.Definition, axle, VehicleSetupTargetKind.SuspensionReboundDamping);
            var rideHeightVariable = FindFirstVariable(vehicle.Definition, axle, VehicleSetupTargetKind.SuspensionRideHeight);

            if (springVariable == null || bumpVariable == null || reboundVariable == null)
            {
                continue;
            }

            var springRate = Math.Max(springVariable.Value, 1f);
            var damping = Math.Max((bumpVariable.Value + reboundVariable.Value) * 0.5f, 1f);
            var targetOffset = Math.Clamp(-(rideHeightVariable?.Value ?? 0f), -SuspensionTargetOffsetLimit, SuspensionTargetOffsetLimit);

            foreach (var wheelSettings in EnumerateWheelSettings(vehicle, axle))
            {
                var quarterMass = Math.Max(wheelSettings.StaticNormalLoad / Gravity, 50f);
                var springFrequency = Math.Clamp((float)(Math.Sqrt(springRate / quarterMass) / (2 * Math.PI)), 1.0f, 6.0f);
                var dampingRatio = Math.Clamp(damping / (2f * (float)Math.Sqrt(springRate * quarterMass)), 0.4f, 2.0f);
                var staticSag = wheelSettings.StaticNormalLoad / springRate;
                var bumpTravel = Math.Clamp(staticSag * 6f, 0.08f, 0.18f);
                var reboundTravel = Math.Clamp(staticSag * 4f, 0.06f, 0.16f);
                var minimumOffset = -Math.Max(reboundTravel, 0f);
                var maximumOffset = Math.Max(bumpTravel, 0f);

                if (wheelSettings.SuspensionSpringServo is LinearAxisServoConstraintComponent springServo)
                {
                    springServo.TargetOffset = targetOffset;
                    springServo.SpringFrequency = springFrequency;
                    springServo.SpringDampingRatio = dampingRatio;
                }

                if (wheelSettings.SuspensionLimit is LinearAxisLimitConstraintComponent suspensionLimit)
                {
                    suspensionLimit.MinimumOffset = minimumOffset;
                    suspensionLimit.MaximumOffset = maximumOffset;
                    suspensionLimit.SpringFrequency = Math.Clamp(springFrequency * 4f, 10f, 20f);
                }

                wheelSettings.SuspensionTargetOffset = targetOffset;
                wheelSettings.SuspensionMinimumOffset = minimumOffset;
                wheelSettings.SuspensionMaximumOffset = maximumOffset;
            }
        }
    }

    private static void ApplyLiveAntiRollBar(LoadedVehicle vehicle, VehicleSetupVariable variable, float value)
    {
        if (vehicle.CarComponent.Dynamics == null)
        {
            return;
        }

        foreach (var axle in ResolveAffectedAxles(variable.Axle))
        {
            if (axle == VehicleSetupAxle.Front)
            {
                vehicle.CarComponent.Dynamics.FrontAntiRollStiffness = value;
            }
            else if (axle == VehicleSetupAxle.Rear)
            {
                vehicle.CarComponent.Dynamics.RearAntiRollStiffness = value;
            }
        }
    }

    private static void ApplyLiveCamber(LoadedVehicle vehicle, VehicleSetupVariable variable)
    {
        var frontCamber = ResolveStaticCamberRadians(vehicle.Definition, VehicleSetupAxle.Front);
        var rearCamber = ResolveStaticCamberRadians(vehicle.Definition, VehicleSetupAxle.Rear);
        vehicle.CarComponent.FrontStaticCamberRadians = frontCamber;
        vehicle.CarComponent.RearStaticCamberRadians = rearCamber;
    }

    private static void ApplyLiveBrakes(LoadedVehicle vehicle)
    {
        var car = vehicle.CarComponent;
        var definition = vehicle.Definition;
        var frontBias = FindFirstVariable(definition, VehicleSetupAxle.Unspecified, VehicleSetupTargetKind.BrakeBias)?.Value ?? 0.5f;
        var brakeStrength = FindFirstVariable(definition, VehicleSetupAxle.Unspecified, VehicleSetupTargetKind.BrakeStrength)?.Value ?? 1f;

        car.BrakeBiasFront = Math.Clamp(frontBias, 0f, 1f);
        car.BrakeForceMultiplier = Math.Max(0f, brakeStrength);
        car.FrontWheelBrakeTorque = ResolveAxleBrakeTorque(definition, VehicleSetupAxle.Front, car.BrakeBiasFront, car.BrakeForceMultiplier);
        car.RearWheelBrakeTorque = ResolveAxleBrakeTorque(definition, VehicleSetupAxle.Rear, car.BrakeBiasFront, car.BrakeForceMultiplier);
        car.RearParkingBrakeTorque = ResolveAxleParkingTorque(definition, VehicleSetupAxle.Rear);
    }

    private static void ApplyLivePowertrain(LoadedVehicle vehicle)
    {
        var car = vehicle.CarComponent;
        var definition = vehicle.Definition;
        var powertrain = VehiclePowertrainResolver.Resolve(definition);

        car.GearRatios = ResolveLiveGearRatios(definition, powertrain.GearRatios);
        car.FinalDrive = ResolveLiveFinalDrive(definition, powertrain);

        if (car.Dynamics == null)
        {
            return;
        }

        car.Dynamics.FrontDiff = ResolveLiveDifferential(definition, VehicleSetupAxle.Front, powertrain.FrontDiff);
        car.Dynamics.RearDiff = ResolveLiveDifferential(definition, VehicleSetupAxle.Rear, powertrain.RearDiff);
        car.Dynamics.CenterDiff = ResolveLiveCenterDifferential(definition, powertrain.CenterDiff);
    }

    private static void ApplyLiveEngine(LoadedVehicle vehicle)
    {
        var car = vehicle.CarComponent;
        var definition = vehicle.Definition;
        var powertrain = VehiclePowertrainResolver.Resolve(definition);

        car.MaxRpm = powertrain.MaxRpm;
        car.RevLimiterRpm = FindFirstVariable(definition, VehicleSetupAxle.Unspecified, VehicleSetupTargetKind.EngineRevLimiterRpm)?.Value
                            ?? powertrain.RevLimiterRpm;
        car.RevLimiterCutTime = FindFirstVariable(definition, VehicleSetupAxle.Unspecified, VehicleSetupTargetKind.EngineRevLimiterCutTime)?.Value
                                ?? powertrain.RevLimiterCutTime;
        car.HasTurbo = powertrain.HasTurbo;
        car.TurboMaxBoostPsi = FindFirstVariable(definition, VehicleSetupAxle.Unspecified, VehicleSetupTargetKind.EngineWastegatePressure)?.Value
                               ?? powertrain.TurboMaxBoostPsi;
        car.StartingFuelLiters = FindFirstVariable(definition, VehicleSetupAxle.Unspecified, VehicleSetupTargetKind.FuelStartingVolume)?.Value
                                 ?? powertrain.StartingFuelLiters;
        car.FuelCapacityLiters = powertrain.FuelCapacityLiters;
    }

    private static void ApplyLivePressure(LoadedVehicle vehicle, VehicleSetupAxle axle, float pressurePsi)
    {
        const float psiToKpa = 6.894757f;
        var pressureKpa = pressurePsi * psiToKpa;

        foreach (var wheelSettings in EnumerateWheelSettings(vehicle, axle))
        {
            if (wheelSettings.TyreModel != null)
            {
                wheelSettings.TyreModel.TyrePressure = pressureKpa;
            }
        }

        if (vehicle.CarComponent.Dynamics == null)
        {
            return;
        }

        for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
        {
            var wheelSettings = FindWheelSettingsByDynamicsIndex(vehicle, i);
            if (wheelSettings == null)
            {
                continue;
            }

            if (axle == VehicleSetupAxle.Front && !wheelSettings.IsFrontAxle ||
                axle == VehicleSetupAxle.Rear && wheelSettings.IsFrontAxle)
            {
                continue;
            }

            if (vehicle.CarComponent.Dynamics.TyreModels[i] != null)
            {
                vehicle.CarComponent.Dynamics.TyreModels[i]!.TyrePressure = pressureKpa;
            }
        }
    }

    private static void ApplyPressureWheelOverride(VehicleDefinition definition, VehicleSetupAxle axle, VehiclePressureWheelOverrides overrides)
    {
        foreach (var option in VehicleSetupSupport.FilterPressureWheelOptionsByAxle(definition.PressureWheelOptions, axle))
        {
            option.Options.PressurePsi = overrides.PressurePsi ?? option.Options.PressurePsi;
            option.Options.WheelOffset = overrides.WheelOffset ?? option.Options.WheelOffset;
            option.Options.Radius = overrides.Radius ?? option.Options.Radius;
            option.Options.TireWidth = overrides.TireWidth ?? option.Options.TireWidth;
            option.Options.HubRadius = overrides.HubRadius ?? option.Options.HubRadius;
            option.Options.HubWidth = overrides.HubWidth ?? option.Options.HubWidth;
        }
    }

    private static float ResolveSyntheticValue(VehicleDefinition definition, VehicleSetupVariable variable)
    {
        var options = VehicleSetupSupport.FilterPressureWheelOptionsByAxle(definition.PressureWheelOptions, variable.Axle);
        return variable.TargetKind switch
        {
            VehicleSetupTargetKind.WheelTirePressure => ResolveLastFinite(options.Select(option => option.Options.PressurePsi), variable.DefaultValue),
            VehicleSetupTargetKind.WheelOffset => ResolveLastFinite(options.Select(option => option.Options.WheelOffset), variable.DefaultValue),
            VehicleSetupTargetKind.WheelRadius => ResolveLastFinite(options.Select(option => option.Options.Radius), variable.DefaultValue),
            VehicleSetupTargetKind.WheelWidth => ResolveLastFinite(options.Select(option => option.Options.TireWidth), variable.DefaultValue),
            VehicleSetupTargetKind.HubRadius => ResolveLastFinite(options.Select(option => option.Options.HubRadius), variable.DefaultValue),
            VehicleSetupTargetKind.HubWidth => ResolveLastFinite(options.Select(option => option.Options.HubWidth), variable.DefaultValue),
            _ => variable.Value,
        };
    }

    private static float ResolveAxleBrakeTorque(
        VehicleDefinition definition,
        VehicleSetupAxle axle,
        float frontBias,
        float brakeStrength)
    {
        var options = VehicleSetupSupport.FilterPressureWheelOptionsByAxle(definition.PressureWheelOptions, axle);
        var parsedTorque = ResolveLastFinite(options.Select(option => option.Options.BrakeTorque), 0f);
        if (parsedTorque > 0f)
        {
            var brakeStrengthVariable = FindFirstVariable(definition, VehicleSetupAxle.Unspecified, VehicleSetupTargetKind.BrakeStrength);
            var brakeBiasVariable = FindFirstVariable(definition, VehicleSetupAxle.Unspecified, VehicleSetupTargetKind.BrakeBias);
            var defaultStrength = brakeStrengthVariable?.DefaultValue ?? 1f;
            var defaultBias = Math.Clamp(brakeBiasVariable?.DefaultValue ?? 0.5f, 0f, 1f);
            var defaultAxleShare = brakeBiasVariable == null
                ? 1f
                : axle == VehicleSetupAxle.Front
                    ? defaultBias
                    : 1f - defaultBias;
            var currentAxleShare = brakeBiasVariable == null
                ? 1f
                : axle == VehicleSetupAxle.Front
                    ? frontBias
                    : 1f - frontBias;
            var normalizedBaseTorque = parsedTorque;

            if (defaultStrength > 1e-4f && defaultAxleShare > 1e-4f)
            {
                normalizedBaseTorque /= defaultStrength * defaultAxleShare;
            }

            return normalizedBaseTorque * Math.Max(0f, brakeStrength) * Math.Max(0f, currentAxleShare);
        }

        var baseTorque = 2100f * brakeStrength;
        return axle == VehicleSetupAxle.Front
            ? baseTorque * frontBias
            : baseTorque * (1f - frontBias);
    }

    private static float ResolveAxleParkingTorque(VehicleDefinition definition, VehicleSetupAxle axle)
    {
        var options = VehicleSetupSupport.FilterPressureWheelOptionsByAxle(definition.PressureWheelOptions, axle);
        return ResolveLastFinite(options.Select(option => option.Options.ParkingTorque), 0f);
    }

    private static float[] ResolveLiveGearRatios(VehicleDefinition definition, float[] baseline)
    {
        var resolved = baseline.ToArray();
        foreach (var variable in definition.Setup.Variables.Values.Where(variable => variable.TargetKind == VehicleSetupTargetKind.TransmissionGearRatio))
        {
            if (TryResolveGearIndex(variable, out var gearIndex) && gearIndex >= 0 && gearIndex < resolved.Length)
            {
                resolved[gearIndex] = Math.Abs(variable.Value);
            }
        }

        return resolved;
    }

    private static float ResolveLiveFinalDrive(VehicleDefinition definition, VehiclePowertrainSetup powertrain)
    {
        var frontValue = FindFirstVariable(definition, VehicleSetupAxle.Front, VehicleSetupTargetKind.DifferentialFinalDrive)?.Value;
        var rearValue = FindFirstVariable(definition, VehicleSetupAxle.Rear, VehicleSetupTargetKind.DifferentialFinalDrive)?.Value;

        if (powertrain.DriveFrontAxle && powertrain.DriveRearAxle)
        {
            if (frontValue.HasValue && rearValue.HasValue)
            {
                return (frontValue.Value + rearValue.Value) * 0.5f;
            }

            return frontValue ?? rearValue ?? powertrain.FinalDrive;
        }

        return powertrain.DriveFrontAxle
            ? frontValue ?? powertrain.FinalDrive
            : rearValue ?? powertrain.FinalDrive;
    }

    private static DifferentialConfig ResolveLiveDifferential(VehicleDefinition definition, VehicleSetupAxle axle, DifferentialConfig fallback)
    {
        fallback.PreloadTorque = FindFirstVariable(definition, axle, VehicleSetupTargetKind.DifferentialPreload)?.Value ?? fallback.PreloadTorque;
        fallback.LockingCoefficient = FindFirstVariable(definition, axle, VehicleSetupTargetKind.DifferentialPowerLock)?.Value ?? fallback.LockingCoefficient;
        fallback.CoastLockingCoefficient = FindFirstVariable(definition, axle, VehicleSetupTargetKind.DifferentialCoastLock)?.Value ?? fallback.CoastLockingCoefficient;
        fallback.PrimaryOutputRatio = FindFirstVariable(definition, axle, VehicleSetupTargetKind.DifferentialTorqueSplit)?.Value ?? fallback.PrimaryOutputRatio;
        return fallback;
    }

    private static DifferentialConfig ResolveLiveCenterDifferential(VehicleDefinition definition, DifferentialConfig fallback)
    {
        var centerVariables = definition.Setup.Variables.Values.Where(variable =>
            !variable.IsSynthetic &&
            variable.TargetKind is (
                VehicleSetupTargetKind.DifferentialPreload or
                VehicleSetupTargetKind.DifferentialPowerLock or
                VehicleSetupTargetKind.DifferentialCoastLock or
                VehicleSetupTargetKind.DifferentialTorqueSplit or
                VehicleSetupTargetKind.DifferentialMaxLockingTorque or
                VehicleSetupTargetKind.DifferentialSlipVsThrottleBias or
                VehicleSetupTargetKind.DifferentialSlipAllowance or
                VehicleSetupTargetKind.DifferentialAccelerationLock or
                VehicleSetupTargetKind.DifferentialBrakeLock or
                VehicleSetupTargetKind.DifferentialLeftFootBrakeUnlock or
                VehicleSetupTargetKind.DifferentialBrakeLockInput or
                VehicleSetupTargetKind.DifferentialTurnInUnlock or
                VehicleSetupTargetKind.DifferentialTurnInUnlockSteeringAngle) &&
            variable.Axle == VehicleSetupAxle.Unspecified &&
            (variable.SourcePartName.Contains("center", StringComparison.OrdinalIgnoreCase) ||
             variable.SourceSlotType.Contains("center", StringComparison.OrdinalIgnoreCase) ||
             variable.SourcePartName.Contains("transfer", StringComparison.OrdinalIgnoreCase) ||
             variable.SourceSlotType.Contains("transfer", StringComparison.OrdinalIgnoreCase)));

        foreach (var variable in centerVariables)
        {
            switch (variable.TargetKind)
            {
                case VehicleSetupTargetKind.DifferentialPreload:
                    fallback.PreloadTorque = variable.Value;
                    break;
                case VehicleSetupTargetKind.DifferentialPowerLock:
                    fallback.LockingCoefficient = variable.Value;
                    break;
                case VehicleSetupTargetKind.DifferentialCoastLock:
                    fallback.CoastLockingCoefficient = variable.Value;
                    break;
                case VehicleSetupTargetKind.DifferentialTorqueSplit:
                    fallback.PrimaryOutputRatio = Math.Clamp(1f - variable.Value, 0f, 1f);
                    break;
                case VehicleSetupTargetKind.DifferentialMaxLockingTorque:
                    fallback.MaxLockingTorque = Math.Max(0f, variable.Value);
                    fallback.UsesActiveCenterControl = true;
                    break;
                case VehicleSetupTargetKind.DifferentialSlipVsThrottleBias:
                    fallback.SlipVsThrottleBias = Math.Clamp(variable.Value, 0f, 1f);
                    fallback.UsesActiveCenterControl = true;
                    break;
                case VehicleSetupTargetKind.DifferentialSlipAllowance:
                    fallback.SlipAllowance = Math.Clamp(variable.Value, 0f, 1f);
                    fallback.UsesActiveCenterControl = true;
                    break;
                case VehicleSetupTargetKind.DifferentialAccelerationLock:
                    fallback.AccelerationLock = Math.Clamp(variable.Value, 0f, 1f);
                    fallback.UsesActiveCenterControl = true;
                    break;
                case VehicleSetupTargetKind.DifferentialBrakeLock:
                    fallback.BrakeLock = Math.Clamp(variable.Value, 0f, 1f);
                    fallback.UsesActiveCenterControl = true;
                    break;
                case VehicleSetupTargetKind.DifferentialLeftFootBrakeUnlock:
                    fallback.LeftFootBrakeUnlock = Math.Clamp(variable.Value, 0f, 1f);
                    fallback.UsesActiveCenterControl = true;
                    break;
                case VehicleSetupTargetKind.DifferentialBrakeLockInput:
                    fallback.BrakeLockInput = Math.Clamp(variable.Value, 0f, 1f);
                    fallback.UsesActiveCenterControl = true;
                    break;
                case VehicleSetupTargetKind.DifferentialTurnInUnlock:
                    fallback.TurnInUnlock = Math.Clamp(variable.Value, 0f, 1f);
                    fallback.UsesActiveCenterControl = true;
                    break;
                case VehicleSetupTargetKind.DifferentialTurnInUnlockSteeringAngle:
                    fallback.TurnInUnlockSteeringAngle = Math.Clamp(variable.Value, 0f, 1f);
                    fallback.UsesActiveCenterControl = true;
                    break;
            }
        }

        return fallback;
    }

    private static bool TryResolveGearIndex(VehicleSetupVariable variable, out int gearIndex)
    {
        if (variable.Name.Equals("gear_R", StringComparison.OrdinalIgnoreCase) ||
            variable.Name.Equals("gear_reverse", StringComparison.OrdinalIgnoreCase))
        {
            gearIndex = 0;
            return true;
        }

        if (variable.Name.StartsWith("gear_", StringComparison.OrdinalIgnoreCase) &&
            int.TryParse(variable.Name[5..], out var parsedIndex))
        {
            gearIndex = parsedIndex;
            return true;
        }

        gearIndex = -1;
        return false;
    }

    private static VehicleSetupVariable? FindFirstVariable(
        VehicleDefinition definition,
        VehicleSetupAxle axle,
        VehicleSetupTargetKind targetKind)
    {
        return definition.Setup.Variables.Values
            .Where(variable => !variable.IsSynthetic && variable.TargetKind == targetKind)
            .Where(variable => axle == VehicleSetupAxle.Unspecified || variable.Axle == axle)
            .OrderByDescending(variable => variable.SourcePartName.Length)
            .FirstOrDefault();
    }

    private static float ResolveStaticCamberRadians(VehicleDefinition definition, VehicleSetupAxle axle)
    {
        var totalRadians = 0f;
        foreach (var variable in definition.Setup.Variables.Values)
        {
            if (variable.TargetKind != VehicleSetupTargetKind.AlignmentCamber ||
                variable.Axle != axle ||
                !VehicleSetupSupport.IsLiveCamberVariable(variable))
            {
                continue;
            }

            totalRadians += RallyCarComponent.ConvertCamberPrecompressionToRadians(variable.Value);
        }

        return totalRadians;
    }

    private static IEnumerable<VehicleSetupAxle> ResolveAffectedAxles(VehicleSetupAxle axle)
    {
        return axle switch
        {
            VehicleSetupAxle.Front => [VehicleSetupAxle.Front],
            VehicleSetupAxle.Rear => [VehicleSetupAxle.Rear],
            VehicleSetupAxle.Both or VehicleSetupAxle.Unspecified => [VehicleSetupAxle.Front, VehicleSetupAxle.Rear],
            _ => [VehicleSetupAxle.Front, VehicleSetupAxle.Rear],
        };
    }

    private static IEnumerable<WheelSettings> EnumerateWheelSettings(LoadedVehicle vehicle, VehicleSetupAxle axle)
    {
        foreach (var wheel in new[] { vehicle.WheelFL, vehicle.WheelFR, vehicle.WheelRL, vehicle.WheelRR })
        {
            var settings = wheel.Get<WheelSettings>();
            if (settings == null)
            {
                continue;
            }

            if (axle == VehicleSetupAxle.Front && !settings.IsFrontAxle ||
                axle == VehicleSetupAxle.Rear && settings.IsFrontAxle)
            {
                continue;
            }

            yield return settings;
        }
    }

    private static WheelSettings? FindWheelSettingsByDynamicsIndex(LoadedVehicle vehicle, int dynamicsIndex)
    {
        return new[] { vehicle.WheelFL, vehicle.WheelFR, vehicle.WheelRL, vehicle.WheelRR }
            .Select(wheel => wheel.Get<WheelSettings>())
            .FirstOrDefault(settings => settings?.DynamicsIndex == dynamicsIndex);
    }

    private static float ResolveLastFinite(IEnumerable<float?> values, float fallback)
    {
        float? resolved = null;
        foreach (var value in values)
        {
            if (value.HasValue && float.IsFinite(value.Value))
            {
                resolved = value.Value;
            }
        }

        return resolved ?? fallback;
    }
}
