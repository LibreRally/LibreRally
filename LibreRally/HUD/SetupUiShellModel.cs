using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using LibreRally.Vehicle;

namespace LibreRally.HUD;

public enum SetupUiEditorKind
{
    Numeric,
    Choice,
    Toggle,
}

public enum SetupUiApplyMode
{
    Live,
    Reload,
}

public sealed class SetupUiApplyPayload
{
    public Dictionary<string, float> VariableOverrides { get; } = new(StringComparer.OrdinalIgnoreCase);
    public Dictionary<VehicleSetupAxle, float> PressureOverrides { get; } = new();
    public string SummaryText { get; init; } = string.Empty;
    public bool HasChanges => VariableOverrides.Count > 0 || PressureOverrides.Count > 0;
    public bool RequiresReload => VariableOverrides.Count > 0;
}

public sealed class SetupUiShellModel
{
    public string Title { get; }
    public string Subtitle { get; }
    public string VehicleName { get; private set; }
    public string StatusText { get; private set; }
    public IReadOnlyList<SetupUiCategoryModel> Categories { get; }
    public int SelectedCategoryIndex { get; private set; }
    public SetupUiCategoryModel SelectedCategory => Categories[SelectedCategoryIndex];
    public int PendingChangeCount => Categories.Sum(category => category.PendingChangeCount);
    public int PendingLiveChangeCount => Categories.Sum(category => category.PendingLiveChangeCount);
    public int PendingReloadChangeCount => Categories.Sum(category => category.PendingReloadChangeCount);
    public int NonDefaultChangeCount => Categories.Sum(category => category.NonDefaultChangeCount);
    public bool CanApply => PendingChangeCount > 0;
    public bool CanResetToDefaults => NonDefaultChangeCount > 0;

    private SetupUiShellModel(
        string title,
        string subtitle,
        string vehicleName,
        string statusText,
        IReadOnlyList<SetupUiCategoryModel> categories)
    {
        Title = title;
        Subtitle = subtitle;
        VehicleName = vehicleName;
        StatusText = statusText;
        Categories = categories.Count == 0 ? [CreateEmptyCategory()] : categories;
    }

    public void SelectCategory(int index)
    {
        SelectedCategoryIndex = Categories.Count == 0 ? 0 : Math.Clamp(index, 0, Categories.Count - 1);
    }

    public void UpdateVehicleContext(string? vehicleName, string? statusText)
    {
        VehicleName = string.IsNullOrWhiteSpace(vehicleName) ? "Factory Preview Car" : vehicleName;
        StatusText = string.IsNullOrWhiteSpace(statusText) ? "Garage tuning preview ready." : statusText;
    }

    public SetupUiApplyPayload CreateApplyPayload()
    {
        var payload = new SetupUiApplyPayload
        {
            SummaryText = $"{PendingChangeCount} change(s) queued • {PendingLiveChangeCount} live • {PendingReloadChangeCount} reload",
        };

        foreach (var field in Categories.SelectMany(category => category.Fields).Where(field => field.IsBoundNumericField && field.IsDirty))
        {
            switch (field.BindingKind)
            {
                case VehicleSetupEntryKind.Variable:
                    payload.VariableOverrides[field.BoundKey!] = field.RawNumericValue;
                    break;
                case VehicleSetupEntryKind.TyrePressure:
                    payload.PressureOverrides[field.Axle] = field.RawNumericValue;
                    break;
            }
        }

        return payload;
    }

    public void ResetPendingChanges()
    {
        foreach (var field in Categories.SelectMany(category => category.Fields))
        {
            field.ResetToCommittedValue();
        }
    }

    public void AcceptPendingChanges()
    {
        foreach (var field in Categories.SelectMany(category => category.Fields))
        {
            field.AcceptPendingValue();
        }
    }

    public void ResetToDefaults()
    {
        foreach (var field in Categories.SelectMany(category => category.Fields))
        {
            field.ResetToDefaultValue();
        }
    }

    public static SetupUiShellModel CreatePreview(string? vehicleName = null, string? statusText = null)
    {
        var categories = new List<SetupUiCategoryModel>
        {
            new(
                "tyres",
                "Tyres & Pressures",
                "Contact patch",
                "Balance carcass support and pressure before the stage starts.",
                [
                    SetupUiFieldModel.Numeric("front-pressure", "Front pressure", "Cold baseline for front axle balance.", 24.6f, 19f, 32f, 0.1f, "psi", SetupUiApplyMode.Live),
                    SetupUiFieldModel.Numeric("rear-pressure", "Rear pressure", "Helps rotation on entry without destabilising traction.", 23.8f, 19f, 32f, 0.1f, "psi", SetupUiApplyMode.Live),
                    SetupUiFieldModel.Choice("compound", "Compound", "Service-area tyre package preview.", 1, ["Soft gravel", "Medium gravel", "Wet cut"], SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Toggle("tyre-blankets", "Tyre blankets", "Represents service prep helpers.", true, "Prepared", "Cold", SetupUiApplyMode.Reload),
                ]),
            new(
                "alignment",
                "Alignment",
                "Turn-in platform",
                "Dial in the chassis attitude for direction changes and braking stability.",
                [
                    SetupUiFieldModel.Numeric("front-camber", "Front camber", "Sharper response through loaded corners.", -2.6f, -4.5f, -0.5f, 0.1f, "deg", SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Numeric("rear-camber", "Rear camber", "Keeps traction stable when the road drops away.", -1.4f, -3.5f, 0f, 0.1f, "deg", SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Numeric("front-toe", "Front toe", "Fine trim for turn-in and straight-line calm.", 0.08f, -0.30f, 0.30f, 0.01f, "deg", SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Choice("caster-map", "Caster map", "Preset slot for garage metadata.", 0, ["Agile", "Balanced", "High-speed"], SetupUiApplyMode.Reload),
                ]),
            new(
                "dampers",
                "Dampers",
                "Kerb compliance",
                "Control platform movement over cuts, compressions and crests.",
                [
                    SetupUiFieldModel.Numeric("front-bump", "Front bump", "Compression damping for attack under braking.", 7f, 1f, 12f, 0.5f, "clicks", SetupUiApplyMode.Live),
                    SetupUiFieldModel.Numeric("front-rebound", "Front rebound", "Supports platform recovery after crests.", 9f, 1f, 12f, 0.5f, "clicks", SetupUiApplyMode.Live),
                    SetupUiFieldModel.Numeric("rear-bump", "Rear bump", "Rear axle support when loading traction exits.", 6f, 1f, 12f, 0.5f, "clicks", SetupUiApplyMode.Live),
                    SetupUiFieldModel.Numeric("rear-rebound", "Rear rebound", "Settles the car after rotation inputs.", 8f, 1f, 12f, 0.5f, "clicks", SetupUiApplyMode.Live),
                ]),
            new(
                "differential",
                "Differential",
                "Drive & coast",
                "Preview power lock, coast support and preload tuning.",
                [
                    SetupUiFieldModel.Numeric("preload", "Preload", "Base locking force before throttle or brake ramps engage.", 110f, 40f, 200f, 5f, "Nm", SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Numeric("power-ramp", "Power ramp", "Higher values resist inside-wheel spin on exit.", 48f, 20f, 80f, 1f, "%", SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Numeric("coast-ramp", "Coast ramp", "Supports stability during trail braking.", 32f, 10f, 70f, 1f, "%", SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Toggle("centre-lock", "Centre diff lock", "Preview toggle for AWD split logic.", false, "Locked", "Open", SetupUiApplyMode.Reload),
                ]),
            new(
                "transmission",
                "Transmission",
                "Launch & ratio prep",
                "Shape final drive response and start-line behaviour.",
                [
                    SetupUiFieldModel.Numeric("final-drive", "Final drive", "Overall gearing for top speed versus punch.", 4.38f, 3.50f, 5.20f, 0.01f, "ratio", SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Numeric("launch-rpm", "Launch RPM", "Preview target for start-line bite.", 4200f, 2500f, 6500f, 50f, "rpm", SetupUiApplyMode.Reload),
                    SetupUiFieldModel.Choice("shift-map", "Shift map", "Future-ready drive mode preset slot.", 1, ["Smooth", "Stage", "Aggressive"], SetupUiApplyMode.Reload),
                ]),
        };

        var model = new SetupUiShellModel(
            "Garage Setup",
            "Myra pause-menu garage shell restored for code-first startup.",
            "Factory Preview Car",
            "Garage tuning preview ready.",
            categories);

        model.UpdateVehicleContext(vehicleName, statusText);
        return model;
    }

    public static SetupUiShellModel CreateFromVehicleSetup(LoadedVehicle? vehicle, VehicleSetupOverrides? overrides, string? vehicleName = null, string? statusText = null)
    {
        var categories = VehicleSetupCatalogBuilder.BuildCategories(vehicle?.Definition, overrides)
            .Select(CreateRuntimeCategory)
            .ToList();

        var subtitle = categories.Count == 0
            ? "No BeamNG setup metadata was detected for the current vehicle, so the restored garage shell is showing preview content."
            : "BeamNG setup data from the active vehicle. Apply persists pressure overrides live and reloads any BeamNG variable changes.";

        var model = categories.Count == 0
            ? CreatePreview(vehicleName, statusText)
            : new SetupUiShellModel("Garage Setup", subtitle, "Factory Preview Car", "Setup data ready.", categories);

        model.UpdateVehicleContext(vehicleName, statusText ?? model.StatusText);
        return model;
    }

    private static SetupUiCategoryModel CreateRuntimeCategory(VehicleSetupCategory category)
    {
        var duplicateLabels = category.Entries
            .Select(entry => entry.Label)
            .GroupBy(label => label, StringComparer.OrdinalIgnoreCase)
            .Where(group => group.Count() > 1)
            .Select(group => group.Key)
            .ToHashSet(StringComparer.OrdinalIgnoreCase);

        var fields = category.Entries
            .Select(entry => SetupUiFieldModel.FromVehicleSetupEntry(entry, BuildRuntimeFieldLabel(entry, duplicateLabels)))
            .ToList();

        return new SetupUiCategoryModel(category.Id, category.Title, category.Tagline, category.Description, fields);
    }

    private static string BuildRuntimeFieldLabel(VehicleSetupEntry entry, IReadOnlySet<string> duplicateLabels)
    {
        if (!duplicateLabels.Contains(entry.Label))
        {
            return entry.Label;
        }

        if (!string.IsNullOrWhiteSpace(entry.SubCategory) &&
            !entry.Label.StartsWith(entry.SubCategory, StringComparison.OrdinalIgnoreCase))
        {
            return $"{entry.SubCategory} {entry.Label}";
        }

        return entry.Axle switch
        {
            VehicleSetupAxle.Front when !entry.Label.StartsWith("Front", StringComparison.OrdinalIgnoreCase) => $"Front {entry.Label}",
            VehicleSetupAxle.Rear when !entry.Label.StartsWith("Rear", StringComparison.OrdinalIgnoreCase) => $"Rear {entry.Label}",
            _ => entry.Label,
        };
    }

    private static SetupUiCategoryModel CreateEmptyCategory() => new(
        "empty",
        "No Setup Data",
        "Preview shell",
        "This vehicle does not currently expose editable BeamNG setup values.",
        []);
}

public sealed class SetupUiCategoryModel(
    string id,
    string title,
    string tagline,
    string description,
    IReadOnlyList<SetupUiFieldModel> fields)
{
    public string Id { get; } = id;
    public string Title { get; } = title;
    public string Tagline { get; } = tagline;
    public string Description { get; } = description;
    public IReadOnlyList<SetupUiFieldModel> Fields { get; } = fields;
    public int PendingChangeCount => Fields.Count(entry => entry.IsDirty);
    public int PendingLiveChangeCount => Fields.Count(entry => entry.IsDirty && entry.ApplyMode == SetupUiApplyMode.Live);
    public int PendingReloadChangeCount => Fields.Count(entry => entry.IsDirty && entry.ApplyMode == SetupUiApplyMode.Reload);
    public int NonDefaultChangeCount => Fields.Count(entry => entry.CanResetToDefault);
}

public sealed class SetupUiFieldModel
{
    public string Id { get; }
    public string Label { get; }
    public string Description { get; }
    public SetupUiEditorKind EditorKind { get; }
    public SetupUiApplyMode ApplyMode { get; }
    public float NumericValue { get; private set; }
    public float RawNumericValue { get; private set; }
    public float Minimum { get; }
    public float Maximum { get; }
    public float Step { get; }
    public string Unit { get; }
    public bool ToggleValue { get; private set; }
    public string ToggleOnLabel { get; }
    public string ToggleOffLabel { get; }
    public IReadOnlyList<string> ChoiceOptions { get; }
    public int ChoiceIndex { get; private set; }
    public string? BoundKey { get; }
    public VehicleSetupEntryKind? BindingKind { get; }
    public VehicleSetupAxle Axle { get; }
    public bool IsBoundNumericField => EditorKind == SetupUiEditorKind.Numeric && !string.IsNullOrWhiteSpace(BoundKey) && BindingKind.HasValue;
    public string DefaultDisplayValue => EditorKind switch
    {
        SetupUiEditorKind.Numeric => FormatNumericValue(ToDisplay(_defaultRawNumericValue), Unit, Step),
        SetupUiEditorKind.Toggle => _defaultToggleValue ? ToggleOnLabel : ToggleOffLabel,
        SetupUiEditorKind.Choice => ChoiceOptions.Count == 0 ? string.Empty : ChoiceOptions[_defaultChoiceIndex],
        _ => string.Empty,
    };
    public bool CanResetToDefault => EditorKind switch
    {
        SetupUiEditorKind.Numeric => Math.Abs(RawNumericValue - _defaultRawNumericValue) > ResolveDifferenceThreshold(_defaultRawNumericValue),
        SetupUiEditorKind.Toggle => ToggleValue != _defaultToggleValue,
        SetupUiEditorKind.Choice => ChoiceIndex != _defaultChoiceIndex,
        _ => false,
    };
    public bool IsDirty => EditorKind switch
    {
        SetupUiEditorKind.Numeric => Math.Abs(RawNumericValue - _committedRawNumericValue) > ResolveDifferenceThreshold(_committedRawNumericValue),
        SetupUiEditorKind.Toggle => ToggleValue != _committedToggleValue,
        SetupUiEditorKind.Choice => ChoiceIndex != _committedChoiceIndex,
        _ => false,
    };

    private readonly float _displayScale;
    private readonly float _displayOffset;
    private float _committedRawNumericValue;
    private bool _committedToggleValue;
    private int _committedChoiceIndex;
    private readonly float _defaultRawNumericValue;
    private readonly bool _defaultToggleValue;
    private readonly int _defaultChoiceIndex;

    private SetupUiFieldModel(
        string id,
        string label,
        string description,
        SetupUiEditorKind editorKind,
        SetupUiApplyMode applyMode,
        float numericValue,
        float minimum,
        float maximum,
        float step,
        string unit,
        bool toggleValue,
        string toggleOnLabel,
        string toggleOffLabel,
        IReadOnlyList<string>? choiceOptions,
        int choiceIndex,
        string? boundKey = null,
        VehicleSetupEntryKind? bindingKind = null,
        VehicleSetupAxle axle = VehicleSetupAxle.Unspecified,
        float rawNumericValue = 0f,
        float displayScale = 1f,
        float displayOffset = 0f)
    {
        Id = id;
        Label = label;
        Description = description;
        EditorKind = editorKind;
        ApplyMode = applyMode;
        NumericValue = numericValue;
        RawNumericValue = rawNumericValue;
        Minimum = minimum;
        Maximum = maximum;
        Step = step;
        Unit = unit;
        ToggleValue = toggleValue;
        ToggleOnLabel = toggleOnLabel;
        ToggleOffLabel = toggleOffLabel;
        ChoiceOptions = choiceOptions ?? Array.Empty<string>();
        ChoiceIndex = ChoiceOptions.Count == 0 ? 0 : Math.Clamp(choiceIndex, 0, ChoiceOptions.Count - 1);
        BoundKey = boundKey;
        BindingKind = bindingKind;
        Axle = axle;
        _displayScale = Math.Abs(displayScale) < 1e-5f ? 1f : displayScale;
        _displayOffset = displayOffset;
        _committedRawNumericValue = rawNumericValue;
        _committedToggleValue = toggleValue;
        _committedChoiceIndex = ChoiceIndex;
        _defaultRawNumericValue = rawNumericValue;
        _defaultToggleValue = toggleValue;
        _defaultChoiceIndex = ChoiceIndex;
    }

    public string DisplayValue => EditorKind switch
    {
        SetupUiEditorKind.Numeric => FormatNumericValue(NumericValue, Unit, Step),
        SetupUiEditorKind.Toggle => ToggleValue ? ToggleOnLabel : ToggleOffLabel,
        SetupUiEditorKind.Choice => ChoiceOptions.Count == 0 ? string.Empty : ChoiceOptions[ChoiceIndex],
        _ => string.Empty,
    };

    public string ApplyHint => ApplyMode == SetupUiApplyMode.Live ? "Live on apply" : "Reload on apply";

    public void SetNumericValue(float value)
    {
        NumericValue = Math.Clamp(value, Minimum, Maximum);
        RawNumericValue = ToRaw(NumericValue);
    }

    public void SetToggleValue(bool value)
    {
        ToggleValue = value;
    }

    public void CycleChoice(int step = 1)
    {
        if (ChoiceOptions.Count == 0)
        {
            ChoiceIndex = 0;
            return;
        }

        var nextIndex = (ChoiceIndex + step) % ChoiceOptions.Count;
        if (nextIndex < 0)
        {
            nextIndex += ChoiceOptions.Count;
        }

        ChoiceIndex = nextIndex;
    }

    public void ResetToCommittedValue()
    {
        RawNumericValue = _committedRawNumericValue;
        NumericValue = ToDisplay(_committedRawNumericValue);
        ToggleValue = _committedToggleValue;
        ChoiceIndex = _committedChoiceIndex;
    }

    public void AcceptPendingValue()
    {
        _committedRawNumericValue = RawNumericValue;
        _committedToggleValue = ToggleValue;
        _committedChoiceIndex = ChoiceIndex;
    }

    public void ResetToDefaultValue()
    {
        RawNumericValue = _defaultRawNumericValue;
        NumericValue = ToDisplay(_defaultRawNumericValue);
        ToggleValue = _defaultToggleValue;
        ChoiceIndex = _defaultChoiceIndex;
    }

    public static SetupUiFieldModel Numeric(string id, string label, string description, float value, float minimum, float maximum, float step, string unit, SetupUiApplyMode applyMode)
        => new(id, label, description, SetupUiEditorKind.Numeric, applyMode, value, minimum, maximum, step, unit, false, string.Empty, string.Empty, null, 0, null, null, VehicleSetupAxle.Unspecified, value);

    public static SetupUiFieldModel Choice(string id, string label, string description, int choiceIndex, IReadOnlyList<string> options, SetupUiApplyMode applyMode)
        => new(id, label, description, SetupUiEditorKind.Choice, applyMode, 0f, 0f, 0f, 0f, string.Empty, false, string.Empty, string.Empty, options, choiceIndex);

    public static SetupUiFieldModel Toggle(string id, string label, string description, bool value, string onLabel, string offLabel, SetupUiApplyMode applyMode)
        => new(id, label, description, SetupUiEditorKind.Toggle, applyMode, 0f, 0f, 1f, 1f, string.Empty, value, onLabel, offLabel, null, 0);

    public static SetupUiFieldModel FromVehicleSetupEntry(VehicleSetupEntry entry, string label)
    {
        var useDisplayRange = entry.MinDisplayValue.HasValue &&
            entry.MaxDisplayValue.HasValue &&
            entry.MaxDisplayValue > entry.MinDisplayValue &&
            entry.MaximumValue > entry.MinimumValue &&
            entry.Value >= entry.MinimumValue &&
            entry.Value <= entry.MaximumValue;

        var displayScale = useDisplayRange
            ? (entry.MaxDisplayValue!.Value - entry.MinDisplayValue!.Value) / (entry.MaximumValue - entry.MinimumValue)
            : 1f;
        var displayOffset = useDisplayRange
            ? entry.MinDisplayValue!.Value - entry.MinimumValue * displayScale
            : 0f;
        var displayMinimum = useDisplayRange ? entry.MinDisplayValue!.Value : entry.MinimumValue;
        var displayMaximum = useDisplayRange ? entry.MaxDisplayValue!.Value : entry.MaximumValue;
        var displayStep = useDisplayRange ? Math.Max(entry.Step * displayScale, 0.001f) : entry.Step;
        var unit = string.IsNullOrWhiteSpace(entry.Unit) && useDisplayRange && displayMaximum <= 100.5f ? "%" : entry.Unit;

        return new SetupUiFieldModel(
            entry.Id,
            label,
            entry.Description,
            SetupUiEditorKind.Numeric,
            entry.ApplyMode == VehicleSetupApplyMode.Live ? SetupUiApplyMode.Live : SetupUiApplyMode.Reload,
            numericValue: entry.Value * displayScale + displayOffset,
            minimum: displayMinimum,
            maximum: displayMaximum,
            step: displayStep,
            unit: unit,
            toggleValue: false,
            toggleOnLabel: string.Empty,
            toggleOffLabel: string.Empty,
            choiceOptions: null,
            choiceIndex: 0,
            boundKey: entry.ResolvedKey,
            bindingKind: entry.Kind,
            axle: entry.Axle,
            rawNumericValue: entry.Value,
            displayScale: displayScale,
            displayOffset: displayOffset);
    }

    private float ToDisplay(float rawValue) => rawValue * _displayScale + _displayOffset;

    private float ToRaw(float displayValue)
    {
        var rawValue = (displayValue - _displayOffset) / _displayScale;
        var rawMinimum = (Minimum - _displayOffset) / _displayScale;
        var rawMaximum = (Maximum - _displayOffset) / _displayScale;
        return Math.Clamp(rawValue, Math.Min(rawMinimum, rawMaximum), Math.Max(rawMinimum, rawMaximum));
    }

    private static float ResolveDifferenceThreshold(float baseline)
    {
        var scale = Math.Max(Math.Abs(baseline), 1f);
        return scale * 1e-4f;
    }

    private static string FormatNumericValue(float value, string unit, float step)
    {
        var decimals = ResolveDecimalPlaces(step);
        var formatted = value.ToString($"F{decimals}", CultureInfo.InvariantCulture);
        if (decimals > 0)
        {
            formatted = formatted.TrimEnd('0').TrimEnd('.');
        }

        return string.IsNullOrWhiteSpace(unit) ? formatted : $"{formatted} {unit}";
    }

    private static int ResolveDecimalPlaces(float step)
    {
        if (step <= 0f || !float.IsFinite(step))
        {
            return 2;
        }

        var decimals = 0;
        var scaled = step;
        while (decimals < 5 && Math.Abs(scaled - MathF.Round(scaled)) > 0.0001f)
        {
            scaled *= 10f;
            decimals++;
        }

        return decimals;
    }
}
