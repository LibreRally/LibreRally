using System;
using System.Collections.Generic;
using System.Linq;
using LibreRally.Vehicle.JBeam;

namespace LibreRally.Vehicle;

public enum VehicleSetupApplyMode
{
    Live,
    Reload,
}

public enum VehicleSetupEntryKind
{
    Variable,
    TyrePressure,
}

public sealed record VehicleSetupEntry(
    string Id,
    string ResolvedKey,
    string Label,
    string Description,
    string Category,
    string? SubCategory,
    float Value,
    float DefaultValue,
    float MinimumValue,
    float MaximumValue,
    float Step,
    string Unit,
    float? MinDisplayValue,
    float? MaxDisplayValue,
    VehicleSetupApplyMode ApplyMode,
    VehicleSetupEntryKind Kind,
    VehicleSetupAxle Axle);

public sealed record VehicleSetupCategory(
    string Id,
    string Title,
    string Tagline,
    string Description,
    IReadOnlyList<VehicleSetupEntry> Entries);

public static class VehicleSetupCatalogBuilder
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

    public static IReadOnlyList<VehicleSetupCategory> BuildCategories(VehicleDefinition? definition, VehicleSetupOverrides? overrides)
    {
        if (definition == null)
        {
            return [];
        }

        var entries = BuildVariableEntries(definition, overrides)
            .Concat(BuildPressureEntries(definition, overrides))
            .ToList();

        return entries
            .GroupBy(entry => entry.Category, StringComparer.OrdinalIgnoreCase)
            .OrderBy(group => ResolveCategorySortKey(group.Key))
            .ThenBy(group => group.Key, StringComparer.OrdinalIgnoreCase)
            .Select(group => new VehicleSetupCategory(
                BuildCategoryId(group.Key),
                group.Key,
                BuildCategoryTagline(group.Key),
                BuildCategoryDescription(group.Key),
                group
                    .OrderBy(entry => entry.SubCategory, StringComparer.OrdinalIgnoreCase)
                    .ThenBy(entry => entry.Label, StringComparer.OrdinalIgnoreCase)
                    .ToList()))
            .ToList();
    }

    private static IEnumerable<VehicleSetupEntry> BuildVariableEntries(VehicleDefinition definition, VehicleSetupOverrides? overrides)
    {
        foreach (var variable in definition.SetupVariables)
        {
            if (!float.IsFinite(variable.MinValue) ||
                !float.IsFinite(variable.MaxValue) ||
                variable.MaxValue <= variable.MinValue)
            {
                continue;
            }

            var (resolvedKey, value) = ResolveEffectiveValue(variable.Name, variable.DefaultValue, definition.Vars, overrides);
            yield return new VehicleSetupEntry(
                Id: variable.Name,
                ResolvedKey: resolvedKey,
                Label: ResolveLabel(variable),
                Description: string.IsNullOrWhiteSpace(variable.Description)
                    ? "BeamNG setup variable exposed by the active parts configuration."
                    : variable.Description,
                Category: string.IsNullOrWhiteSpace(variable.Category) ? "General" : variable.Category.Trim(),
                SubCategory: string.IsNullOrWhiteSpace(variable.SubCategory) ? null : variable.SubCategory.Trim(),
                Value: value,
                DefaultValue: variable.DefaultValue,
                MinimumValue: variable.MinValue,
                MaximumValue: variable.MaxValue,
                Step: ResolveStep(variable),
                Unit: NormalizeUnit(variable.Unit),
                MinDisplayValue: variable.MinDisplayValue,
                MaxDisplayValue: variable.MaxDisplayValue,
                ApplyMode: VehicleSetupApplyMode.Reload,
                Kind: VehicleSetupEntryKind.Variable,
                Axle: ResolveAxle(variable.Name, variable.SubCategory, variable.Category));
        }
    }

    private static IEnumerable<VehicleSetupEntry> BuildPressureEntries(VehicleDefinition definition, VehicleSetupOverrides? overrides)
    {
        foreach (var axle in new[] { VehicleSetupAxle.Front, VehicleSetupAxle.Rear })
        {
            var axlePressure = ResolvePressurePsi(definition, overrides, axle);
            var defaultPressure = ResolvePressurePsi(definition, null, axle);
            if (!axlePressure.HasValue)
            {
                continue;
            }

            yield return new VehicleSetupEntry(
                Id: axle == VehicleSetupAxle.Front ? "pressure_F" : "pressure_R",
                ResolvedKey: axle == VehicleSetupAxle.Front ? "pressure_F" : "pressure_R",
                Label: axle == VehicleSetupAxle.Front ? "Front pressure" : "Rear pressure",
                Description: "Current tyre inflation from the active pressure-wheel configuration.",
                Category: "Tyres & Pressures",
                SubCategory: axle == VehicleSetupAxle.Front ? "Front" : "Rear",
                Value: axlePressure.Value,
                DefaultValue: defaultPressure ?? axlePressure.Value,
                MinimumValue: 10f,
                MaximumValue: 45f,
                Step: 0.5f,
                Unit: "psi",
                MinDisplayValue: null,
                MaxDisplayValue: null,
                ApplyMode: VehicleSetupApplyMode.Live,
                Kind: VehicleSetupEntryKind.TyrePressure,
                Axle: axle);
        }
    }

    private static (string ResolvedKey, float Value) ResolveEffectiveValue(
        string variableName,
        float fallbackValue,
        IReadOnlyDictionary<string, float> values,
        VehicleSetupOverrides? overrides)
    {
        foreach (var suffix in EffectiveValueSuffixPriority)
        {
            var candidate = variableName + suffix;
            if (TryResolveOverride(overrides, candidate, out var overrideValue))
            {
                return (candidate, overrideValue);
            }

            if (values.TryGetValue(candidate, out var value) && float.IsFinite(value))
            {
                return (candidate, value);
            }
        }

        if (TryResolveOverride(overrides, variableName, out var exactOverride))
        {
            return (variableName, exactOverride);
        }

        if (values.TryGetValue(variableName, out var exactValue) && float.IsFinite(exactValue))
        {
            return (variableName, exactValue);
        }

        var overriddenMatch = overrides?.VariableOverrides.Keys
            .Where(key => key.StartsWith(variableName + "_", StringComparison.OrdinalIgnoreCase))
            .OrderBy(key => key, StringComparer.OrdinalIgnoreCase)
            .FirstOrDefault();
        if (overriddenMatch != null &&
            overrides != null &&
            overrides.VariableOverrides.TryGetValue(overriddenMatch, out var overrideMatchValue) &&
            float.IsFinite(overrideMatchValue))
        {
            return (overriddenMatch, overrideMatchValue);
        }

        var bestMatch = values.Keys
            .Where(key => key.StartsWith(variableName + "_", StringComparison.OrdinalIgnoreCase))
            .OrderBy(key => key, StringComparer.OrdinalIgnoreCase)
            .FirstOrDefault();
        if (bestMatch != null &&
            values.TryGetValue(bestMatch, out var matchedValue) &&
            float.IsFinite(matchedValue))
        {
            return (bestMatch, matchedValue);
        }

        return (variableName, fallbackValue);
    }

    private static bool TryResolveOverride(VehicleSetupOverrides? overrides, string key, out float value)
    {
        if (overrides != null &&
            overrides.VariableOverrides.TryGetValue(key, out value) &&
            float.IsFinite(value))
        {
            return true;
        }

        value = 0f;
        return false;
    }

    private static float? ResolvePressurePsi(VehicleDefinition definition, VehicleSetupOverrides? overrides, VehicleSetupAxle axle)
    {
        if (overrides?.PressureWheelOverrides.TryGetValue(axle, out var pressureOverride) == true &&
            pressureOverride.PressurePsi is { } overridePressure &&
            float.IsFinite(overridePressure))
        {
            return overridePressure;
        }

        var pressure = definition.PressureWheelOptions
            .Where(option => MatchesAxle(option.SourceSlotType, option.SourcePartName, axle))
            .Select(option => option.Options.PressurePsi)
            .FirstOrDefault(value => value.HasValue && float.IsFinite(value.Value));
        return pressure;
    }

    internal static bool MatchesAxle(string sourceSlotType, string sourcePartName, VehicleSetupAxle axle)
    {
        var combined = $"{sourceSlotType} {sourcePartName}";
        return axle switch
        {
            VehicleSetupAxle.Front => ContainsAny(combined, "_F", " front", "front", "wheeldata_f", "pressurewheel_f"),
            VehicleSetupAxle.Rear => ContainsAny(combined, "_R", " rear", "rear", "wheeldata_r", "pressurewheel_r"),
            _ => false,
        };
    }

    private static VehicleSetupAxle ResolveAxle(string name, string? subCategory, string category)
    {
        if (ContainsAny(subCategory, "front"))
        {
            return VehicleSetupAxle.Front;
        }

        if (ContainsAny(subCategory, "rear"))
        {
            return VehicleSetupAxle.Rear;
        }

        return ContainsAny(name, "_F", "front") || ContainsAny(category, "front")
            ? VehicleSetupAxle.Front
            : ContainsAny(name, "_R", "rear") || ContainsAny(category, "rear")
                ? VehicleSetupAxle.Rear
                : VehicleSetupAxle.Unspecified;
    }

    private static string ResolveLabel(JBeamVariableDefinition variable)
    {
        if (!string.IsNullOrWhiteSpace(variable.Title))
        {
            return variable.Title.Trim();
        }

        return variable.Name.Replace('_', ' ').Replace('-', ' ').Trim();
    }

    private static float ResolveStep(JBeamVariableDefinition variable)
    {
        if (variable.StepDisplayValue is { } explicitStep && explicitStep > 0f)
        {
            return explicitStep;
        }

        var span = Math.Abs(variable.MaxValue - variable.MinValue);
        if (span <= 0.05f)
        {
            return 0.001f;
        }

        if (span <= 0.5f)
        {
            return 0.01f;
        }

        if (span <= 5f)
        {
            return 0.05f;
        }

        if (span <= 25f)
        {
            return 0.1f;
        }

        if (span <= 250f)
        {
            return 1f;
        }

        return 5f;
    }

    private static string NormalizeUnit(string? unit)
    {
        if (string.IsNullOrWhiteSpace(unit))
        {
            return string.Empty;
        }

        return unit!.Trim().TrimStart('+');
    }

    private static int ResolveCategorySortKey(string category)
    {
        var key = category.ToLowerInvariant();
        if (key.Contains("tyre") || key.Contains("tire"))
        {
            return 0;
        }

        if (key.Contains("alignment"))
        {
            return 1;
        }

        if (key.Contains("suspension"))
        {
            return 2;
        }

        if (key.Contains("brake"))
        {
            return 3;
        }

        if (key.Contains("differential"))
        {
            return 4;
        }

        if (key.Contains("transmission"))
        {
            return 5;
        }

        if (key.Contains("engine"))
        {
            return 6;
        }

        if (key.Contains("chassis"))
        {
            return 7;
        }

        if (key.Contains("wheel"))
        {
            return 8;
        }

        return 9;
    }

    private static string BuildCategoryId(string category) =>
        category.ToLowerInvariant().Replace('&', ' ').Replace('/', ' ').Replace(' ', '-');

    private static string BuildCategoryTagline(string category)
    {
        var key = category.ToLowerInvariant();
        if (key.Contains("tyre") || key.Contains("tire"))
        {
            return "Contact patch";
        }

        if (key.Contains("alignment"))
        {
            return "Turn-in platform";
        }

        if (key.Contains("suspension"))
        {
            return "Load support";
        }

        if (key.Contains("brake"))
        {
            return "Stopping balance";
        }

        if (key.Contains("differential"))
        {
            return "Drive & coast";
        }

        if (key.Contains("transmission"))
        {
            return "Ratio stack";
        }

        if (key.Contains("engine"))
        {
            return "Power delivery";
        }

        if (key.Contains("wheel"))
        {
            return "Hub geometry";
        }

        return "Service area";
    }

    private static string BuildCategoryDescription(string category)
    {
        var key = category.ToLowerInvariant();
        if (key.Contains("tyre") || key.Contains("tire"))
        {
            return "Tune tyre-related service settings before heading back to the stage.";
        }

        if (key.Contains("alignment"))
        {
            return "Adjust wheel attitude and stance values exposed by the active BeamNG parts.";
        }

        if (key.Contains("suspension"))
        {
            return "Review spring, damping and ride-height variables from the loaded vehicle definition.";
        }

        if (key.Contains("brake"))
        {
            return "Brake balance and torque multipliers sourced from the active setup.";
        }

        if (key.Contains("differential"))
        {
            return "Differential preload, locking and torque split values from BeamNG metadata.";
        }

        if (key.Contains("transmission"))
        {
            return "Gear and launch settings from the active transmission configuration.";
        }

        if (key.Contains("engine"))
        {
            return "Engine-side calibration values currently exposed through JBeam variables.";
        }

        if (key.Contains("wheel"))
        {
            return "Wheel package geometry that typically needs a quick reload after apply.";
        }

        return "BeamNG setup variables detected for the active vehicle.";
    }

    private static bool ContainsAny(string? source, params string[] values)
    {
        if (string.IsNullOrWhiteSpace(source))
        {
            return false;
        }

        return values.Any(value => source.Contains(value, StringComparison.OrdinalIgnoreCase));
    }
}
