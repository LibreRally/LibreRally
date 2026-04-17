using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using LibreRally.Vehicle.JBeam;

namespace LibreRally.Vehicle;

internal static class VehicleSetupSupport
{
    private static readonly string[] EffectiveValueSuffixPriority =
    [
        "_asphalt",
        "_tarmac",
        "_gravel",
        "_dirt",
        "_mud",
        "_snow",
        "_rally",
        "_race",
    ];

    private static readonly IReadOnlyDictionary<string, float> SunburstFallbackDefaults = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase)
    {
        ["brakebias"] = 0.59f,
        ["brakestrength"] = 0.85f,
        ["ffbstrength"] = 0.85f,
        ["fuel2"] = 75f,
        ["acd_1difftorquesplit"] = 0.62f,
        ["acd_1maxTorque"] = 390f,
        ["acd_2slipVsThrottle"] = 0.30f,
        ["acd_3slipAllowed"] = 0.05f,
        ["acd_4accelLock"] = 0.40f,
        ["acd_5brakeLock"] = 0.95f,
        ["acd_6lfbrakeLock"] = 0.20f,
        ["acd_7brakeMax"] = 0.60f,
        ["acd_8steerLock"] = 0.80f,
        ["acd_9steerLockAngle"] = 0.333f,
        ["finaldrive_F"] = 4.55f,
        ["finaldrive_R"] = 4.55f,
        ["lsdlockcoef_F"] = 0.255f,
        ["lsdlockcoefrev_F"] = 0.02f,
        ["lsdpreload_F"] = 90f,
        ["lsdlockcoef_R"] = 0.125f,
        ["lsdlockcoefrev_R"] = 0.125f,
        ["lsdpreload_R"] = 100f,
        ["revLimiterCutTime"] = 0.15f,
        ["revLimiterRPM"] = 7500f,
        ["wastegateTarget"] = 15f,
        ["arb_spring_F"] = 40000f,
        ["bumpstop_bump_F_asphalt"] = 10000f,
        ["damp_bump_F_asphalt"] = 4400f,
        ["damp_bump_F_fast_asphalt"] = 2300f,
        ["damp_rebound_F_asphalt"] = 11500f,
        ["damp_rebound_F_fast_asphalt"] = 5600f,
        ["spring_F_asphalt"] = 60000f,
        ["springheight_F_asphalt"] = -0.025f,
        ["arb_spring_R"] = 25000f,
        ["bumpstop_bump_R_asphalt"] = 10000f,
        ["damp_bump_R_asphalt"] = 3600f,
        ["damp_bump_R_fast_asphalt"] = 1800f,
        ["spring_R_asphalt"] = 50000f,
        ["springheight_R_asphalt"] = -0.025f,
        ["gear_1"] = 3.64f,
        ["gear_2"] = 2.38f,
        ["gear_3"] = 1.76f,
        ["gear_4"] = 1.35f,
        ["gear_5"] = 1.06f,
        ["gear_6"] = 0.84f,
        ["gear_R"] = 3.25f,
        ["camber_F_rally"] = 0.98f,
        ["camber_upper_F_rally"] = -0.01f,
        ["caster_F_rally"] = 1f,
        ["caster_upper_F_rally"] = -0.0152f,
        ["steer_center_F"] = 0f,
        ["toe_F_rally"] = 0.9968f,
        ["camber_R_rally"] = 0.9875f,
        ["camber_upper_R_rally"] = 0f,
        ["toe_R_rally"] = 0.9983f,
        ["tirepressure_F"] = 27f,
        ["trackoffset_F"] = 0.015f,
        ["tirepressure_R"] = 27f,
        ["trackoffset_R"] = 0f,
    };

    public static void PopulateRuntimeMetadata(
        VehicleSetupVariable variable,
        IReadOnlyDictionary<string, float> effectiveValues)
    {
        var (effectiveValueName, value) = ResolveEffectiveValue(variable.Name, variable.DefaultValue, effectiveValues);
        variable.EffectiveValueName = effectiveValueName;
        variable.Value = value;
        variable.DisplayHints["hasResolvedExactValue"] = (
            !string.IsNullOrWhiteSpace(effectiveValueName) &&
            effectiveValues.TryGetValue(effectiveValueName, out var resolvedValue) &&
            float.IsFinite(resolvedValue)).ToString();
        variable.TargetKind = ClassifyTarget(variable);
        variable.Axle = ResolveAxle(variable);
        variable.ApplyMode = ResolveApplyMode(variable.TargetKind);
    }

    public static (string EffectiveValueName, float Value) ResolveEffectiveValue(
        string variableName,
        float fallbackValue,
        IReadOnlyDictionary<string, float> effectiveValues)
    {
        foreach (var suffix in EffectiveValueSuffixPriority)
        {
            var candidate = variableName + suffix;
            if (effectiveValues.TryGetValue(candidate, out var overrideValue) && float.IsFinite(overrideValue))
            {
                return (candidate, overrideValue);
            }
        }

        if (effectiveValues.TryGetValue(variableName, out var exactValue) && float.IsFinite(exactValue))
        {
            return (variableName, exactValue);
        }

        var bestMatch = effectiveValues.Keys
            .Where(key => key.StartsWith(variableName + "_", StringComparison.OrdinalIgnoreCase))
            .OrderBy(key => key, StringComparer.OrdinalIgnoreCase)
            .FirstOrDefault();
        if (bestMatch != null &&
            effectiveValues.TryGetValue(bestMatch, out var matchedValue) &&
            float.IsFinite(matchedValue))
        {
            return (bestMatch, matchedValue);
        }

        return (variableName, fallbackValue);
    }

    public static VehicleSetupTargetKind ClassifyTarget(VehicleSetupVariable variable)
    {
        var name = variable.Name;
        var title = variable.Title;
        var description = variable.Description;
        var category = variable.Category;

        if (ContainsAny(name, title, description, "antiroll", "arb_spring", "swaybar"))
        {
            return VehicleSetupTargetKind.SuspensionAntiRollBar;
        }

        if (ContainsAny(name, title, description, "springheight", "ride height", "ride_height"))
        {
            return VehicleSetupTargetKind.SuspensionRideHeight;
        }

        if (ContainsAny(name, title, description, "damp_bump", "bump damping"))
        {
            return VehicleSetupTargetKind.SuspensionBumpDamping;
        }

        if (ContainsAny(name, title, description, "damp_rebound", "rebound damping"))
        {
            return VehicleSetupTargetKind.SuspensionReboundDamping;
        }

        if (ContainsAny(name, title, description, "spring rate", "spring_") &&
            !ContainsAny(name, title, description, "anti-roll", "anti roll", "arb"))
        {
            return VehicleSetupTargetKind.SuspensionSpringRate;
        }

        if (Contains(category, "alignment") || ContainsAny(name, title, description, "camber", "toe", "caster", "steer_center"))
        {
            if (ContainsAny(name, title, description, "camber"))
            {
                return VehicleSetupTargetKind.AlignmentCamber;
            }

            if (ContainsAny(name, title, description, "toe", "steer_center", "trim"))
            {
                return ContainsAny(name, title, description, "steer_center", "trim")
                    ? VehicleSetupTargetKind.AlignmentSteerTrim
                    : VehicleSetupTargetKind.AlignmentToe;
            }

            if (ContainsAny(name, title, description, "caster"))
            {
                return VehicleSetupTargetKind.AlignmentCaster;
            }
        }

        if (Contains(category, "brake"))
        {
            if (ContainsAny(name, title, description, "bias"))
            {
                return VehicleSetupTargetKind.BrakeBias;
            }

            if (ContainsAny(name, title, description, "force", "strength"))
            {
                return VehicleSetupTargetKind.BrakeStrength;
            }
        }

        if (Contains(category, "differential"))
        {
            if (ContainsAny(name, title, description, "maximum locking torque"))
            {
                return VehicleSetupTargetKind.DifferentialMaxLockingTorque;
            }

            if (ContainsAny(name, title, description, "slip vs throttle"))
            {
                return VehicleSetupTargetKind.DifferentialSlipVsThrottleBias;
            }

            if (ContainsAny(name, title, description, "slip allowance"))
            {
                return VehicleSetupTargetKind.DifferentialSlipAllowance;
            }

            if (ContainsAny(name, title, description, "acceleration lock"))
            {
                return VehicleSetupTargetKind.DifferentialAccelerationLock;
            }

            if (ContainsAny(name, title, description, "left foot brake unlock"))
            {
                return VehicleSetupTargetKind.DifferentialLeftFootBrakeUnlock;
            }

            if (ContainsAny(name, title, description, "brake lock/unlock input"))
            {
                return VehicleSetupTargetKind.DifferentialBrakeLockInput;
            }

            if (ContainsAny(name, title, description, "turn-in unlock steering angle"))
            {
                return VehicleSetupTargetKind.DifferentialTurnInUnlockSteeringAngle;
            }

            if (ContainsAny(name, title, description, "turn-in unlock"))
            {
                return VehicleSetupTargetKind.DifferentialTurnInUnlock;
            }

            if (ContainsAny(name, title, description, "brake lock"))
            {
                return VehicleSetupTargetKind.DifferentialBrakeLock;
            }

            if (ContainsAny(name, title, description, "final drive"))
            {
                return VehicleSetupTargetKind.DifferentialFinalDrive;
            }

            if (ContainsAny(name, title, description, "pre-load", "preload"))
            {
                return VehicleSetupTargetKind.DifferentialPreload;
            }

            if (ContainsAny(name, title, description, "power lock"))
            {
                return VehicleSetupTargetKind.DifferentialPowerLock;
            }

            if (ContainsAny(name, title, description, "coast lock"))
            {
                return VehicleSetupTargetKind.DifferentialCoastLock;
            }

            if (ContainsAny(name, title, description, "torque split"))
            {
                return VehicleSetupTargetKind.DifferentialTorqueSplit;
            }
        }

        if (Contains(category, "engine"))
        {
            if (ContainsAny(name, title, description, "rpm limit cut time", "cut time"))
            {
                return VehicleSetupTargetKind.EngineRevLimiterCutTime;
            }

            if (ContainsAny(name, title, description, "rpm limit", "rev limiter"))
            {
                return VehicleSetupTargetKind.EngineRevLimiterRpm;
            }

            if (ContainsAny(name, title, description, "wastegate", "boost"))
            {
                return VehicleSetupTargetKind.EngineWastegatePressure;
            }
        }

        if (Contains(category, "transmission") && ContainsAny(name, title, description, "gear"))
        {
            return VehicleSetupTargetKind.TransmissionGearRatio;
        }

        if (Contains(category, "chassis") || Contains(category, "fuel"))
        {
            if (ContainsAny(name, title, description, "fuel volume", "starting fuel"))
            {
                return VehicleSetupTargetKind.FuelStartingVolume;
            }

            if (ContainsAny(name, title, description, "fuel capacity"))
            {
                return VehicleSetupTargetKind.FuelCapacity;
            }
        }

        if (Contains(category, "wheel") || Contains(category, "tire") || Contains(category, "tyre"))
        {
            if (ContainsAny(name, title, description, "pressure"))
            {
                return VehicleSetupTargetKind.WheelTirePressure;
            }

            if (ContainsAny(name, title, description, "offset"))
            {
                return VehicleSetupTargetKind.WheelOffset;
            }

            if (ContainsAny(name, title, description, "radius"))
            {
                return ContainsAny(name, title, description, "hub")
                    ? VehicleSetupTargetKind.HubRadius
                    : VehicleSetupTargetKind.WheelRadius;
            }

            if (ContainsAny(name, title, description, "width"))
            {
                return ContainsAny(name, title, description, "hub")
                    ? VehicleSetupTargetKind.HubWidth
                    : VehicleSetupTargetKind.WheelWidth;
            }
        }

        return VehicleSetupTargetKind.Unknown;
    }

    public static VehicleSetupApplyMode ResolveApplyMode(VehicleSetupTargetKind targetKind)
    {
        return targetKind switch
        {
            VehicleSetupTargetKind.SuspensionSpringRate => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.SuspensionBumpDamping => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.SuspensionReboundDamping => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.SuspensionRideHeight => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.SuspensionAntiRollBar => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.AlignmentCamber => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.BrakeBias => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.BrakeStrength => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialFinalDrive => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialPreload => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialPowerLock => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialCoastLock => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialTorqueSplit => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialMaxLockingTorque => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialSlipVsThrottleBias => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialSlipAllowance => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialAccelerationLock => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialBrakeLock => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialLeftFootBrakeUnlock => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialBrakeLockInput => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialTurnInUnlock => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.DifferentialTurnInUnlockSteeringAngle => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.EngineRevLimiterRpm => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.EngineRevLimiterCutTime => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.EngineWastegatePressure => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.TransmissionGearRatio => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.WheelTirePressure => VehicleSetupApplyMode.Live,
            VehicleSetupTargetKind.FuelStartingVolume => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.FuelCapacity => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.AlignmentToe => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.AlignmentCaster => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.AlignmentSteerTrim => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.WheelOffset => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.WheelRadius => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.WheelWidth => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.HubRadius => VehicleSetupApplyMode.Reload,
            VehicleSetupTargetKind.HubWidth => VehicleSetupApplyMode.Reload,
            _ => VehicleSetupApplyMode.Reload,
        };
    }

    public static VehicleSetupAxle ResolveAxle(VehicleSetupVariable variable)
    {
        if (TryResolveAxle(variable.SubCategory, out var subCategoryAxle))
        {
            return subCategoryAxle;
        }

        foreach (var token in new[] { variable.Name, variable.Title, variable.SourceSlotType, variable.SourcePartName })
        {
            if (TryResolveAxle(token, out var tokenAxle))
            {
                return tokenAxle;
            }
        }

        return VehicleSetupAxle.Unspecified;
    }

    public static bool TryResolveAxle(string? raw, out VehicleSetupAxle axle)
    {
        if (!string.IsNullOrWhiteSpace(raw))
        {
            var tokens = raw.Split(['_', '-', ' '], StringSplitOptions.RemoveEmptyEntries);
            foreach (var token in tokens)
            {
                if (token.Equals("F", StringComparison.OrdinalIgnoreCase) ||
                    token.Equals("Front", StringComparison.OrdinalIgnoreCase) ||
                    token.Equals("FL", StringComparison.OrdinalIgnoreCase) ||
                    token.Equals("FR", StringComparison.OrdinalIgnoreCase))
                {
                    axle = VehicleSetupAxle.Front;
                    return true;
                }

                if (token.Equals("R", StringComparison.OrdinalIgnoreCase) ||
                    token.Equals("Rear", StringComparison.OrdinalIgnoreCase) ||
                    token.Equals("RL", StringComparison.OrdinalIgnoreCase) ||
                    token.Equals("RR", StringComparison.OrdinalIgnoreCase))
                {
                    axle = VehicleSetupAxle.Rear;
                    return true;
                }
            }
        }

        axle = VehicleSetupAxle.Unspecified;
        return false;
    }

    public static IEnumerable<AssembledPressureWheelOptions> FilterPressureWheelOptionsByAxle(
        IEnumerable<AssembledPressureWheelOptions> options,
        VehicleSetupAxle axle)
    {
        if (axle == VehicleSetupAxle.Unspecified || axle == VehicleSetupAxle.Both)
        {
            return options;
        }

        return options.Where(option =>
            VehicleTyreSpecResolver.MatchesAxle(option.SourceSlotType, axle == VehicleSetupAxle.Front) ||
            VehicleTyreSpecResolver.MatchesAxle(option.SourcePartName, axle == VehicleSetupAxle.Front));
    }

    public static bool IsLiveCamberVariable(VehicleSetupVariable variable)
    {
        if (variable.TargetKind != VehicleSetupTargetKind.AlignmentCamber)
        {
            return false;
        }

        if (ContainsAny(variable.Name, variable.Title, variable.Description, "upper"))
        {
            return false;
        }

        var defaultValue = variable.DefaultValue;
        var minValue = variable.MinValue ?? defaultValue;
        var maxValue = variable.MaxValue ?? defaultValue;
        return Math.Abs(defaultValue - 1f) <= 0.25f &&
               Math.Abs(minValue - 1f) <= 0.30f &&
               Math.Abs(maxValue - 1f) <= 0.30f;
    }

    public static void ApplyVehicleSpecificFallbacks(string vehicleFolderPath, IDictionary<string, float> effectiveValues)
    {
        if (!Path.GetFileName(vehicleFolderPath).Equals("sunburst2", StringComparison.OrdinalIgnoreCase))
        {
            return;
        }

        foreach (var kv in SunburstFallbackDefaults)
        {
            effectiveValues.TryAdd(kv.Key, kv.Value);
        }
    }

    public static bool ShouldExposeInRuntimeSetup(
        VehicleSetupVariable variable,
        IReadOnlyCollection<VehicleSetupVariable> allVariables)
    {
        if (IsHiddenInUi(variable) || variable.TargetKind == VehicleSetupTargetKind.Unknown)
        {
            return false;
        }

        if (variable.IsSynthetic)
        {
            if (variable.TargetKind == VehicleSetupTargetKind.WheelOffset)
            {
                return !HasVariable(allVariables, variable.Axle, VehicleSetupTargetKind.WheelOffset, synthetic: false);
            }

            return !HasVariable(allVariables, variable.Axle, variable.TargetKind, synthetic: false);
        }

        if (variable.TargetKind == VehicleSetupTargetKind.WheelTirePressure &&
            HasVariable(allVariables, variable.Axle, VehicleSetupTargetKind.WheelTirePressure, synthetic: true))
        {
            return false;
        }

        return true;
    }

    public static IReadOnlyList<VehicleSetupVariable> BuildRuntimeSetupVariables(IReadOnlyCollection<VehicleSetupVariable> allVariables)
    {
        var exposedVariables = allVariables
            .Where(variable => ShouldExposeInRuntimeSetup(variable, allVariables))
            .ToList();
        var deduplicatedVariables = new List<VehicleSetupVariable>(exposedVariables.Count);

        foreach (var group in exposedVariables.GroupBy(
                     variable => ResolveRuntimeSetupIdentity(variable) ?? variable.Name,
                     StringComparer.OrdinalIgnoreCase))
        {
            var selectedVariable = group
                .OrderByDescending(GetRuntimeSetupSelectionPriority)
                .ThenBy(variable => variable.Name, StringComparer.OrdinalIgnoreCase)
                .First();
            deduplicatedVariables.Add(selectedVariable);
        }

        return deduplicatedVariables;
    }

    public static bool IsHiddenInUi(VehicleSetupVariable variable)
    {
        return variable.DisplayHints.TryGetValue("hideInUI", out var raw) &&
               bool.TryParse(raw, out var hidden) &&
               hidden;
    }

    private static bool Contains(string? haystack, string needle)
    {
        return !string.IsNullOrWhiteSpace(haystack) &&
               haystack.Contains(needle, StringComparison.OrdinalIgnoreCase);
    }

    private static bool ContainsAny(string? value, params string[] needles)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return false;
        }

        return needles.Any(needle => value.Contains(needle, StringComparison.OrdinalIgnoreCase));
    }

    private static bool ContainsAny(string? first, string? second, string? third, params string[] needles)
    {
        return ContainsAny(first, needles) || ContainsAny(second, needles) || ContainsAny(third, needles);
    }

    private static bool HasVariable(
        IEnumerable<VehicleSetupVariable> variables,
        VehicleSetupAxle axle,
        VehicleSetupTargetKind targetKind,
        bool synthetic)
    {
        return variables.Any(variable =>
            variable.TargetKind == targetKind &&
            variable.Axle == axle &&
            variable.IsSynthetic == synthetic);
    }

    private static string? ResolveRuntimeSetupIdentity(VehicleSetupVariable variable)
    {
        return variable.TargetKind switch
        {
            VehicleSetupTargetKind.AlignmentCamber or
            VehicleSetupTargetKind.AlignmentCaster or
            VehicleSetupTargetKind.AlignmentToe or
            VehicleSetupTargetKind.AlignmentSteerTrim => $"{variable.Category}|{variable.TargetKind}|{variable.Axle}",
            _ => null,
        };
    }

    private static int GetRuntimeSetupSelectionPriority(VehicleSetupVariable variable)
    {
        var priority = 0;

        if (IsActivePartVariable(variable))
        {
            priority += 300;
        }

        if (HasResolvedExactValue(variable) &&
            variable.Name.Equals(variable.EffectiveValueName, StringComparison.OrdinalIgnoreCase))
        {
            priority += 200;
        }

        if (variable.ApplyMode == VehicleSetupApplyMode.Live)
        {
            priority += 100;
        }

        if (!ContainsAny(variable.Name, variable.Title, variable.Description, "upper", "top"))
        {
            priority += 50;
        }

        if (!string.IsNullOrWhiteSpace(variable.Description))
        {
            priority += 20;
        }

        if (!string.IsNullOrWhiteSpace(variable.EffectiveValueName))
        {
            priority += 10;
        }

        if (!variable.IsSynthetic)
        {
            priority += 5;
        }

        return priority;
    }

    private static bool HasResolvedExactValue(VehicleSetupVariable variable)
    {
        return variable.DisplayHints.TryGetValue("hasResolvedExactValue", out var raw) &&
               bool.TryParse(raw, out var resolvedExactValue) &&
               resolvedExactValue;
    }

    private static bool IsActivePartVariable(VehicleSetupVariable variable)
    {
        return variable.DisplayHints.TryGetValue("activePartVariable", out var raw) &&
               bool.TryParse(raw, out var activePartVariable) &&
               activePartVariable;
    }
}
