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

public sealed class SetupUiShellModel
{
    public string Title { get; }
    public string Subtitle { get; }
    public string VehicleName { get; private set; }
    public string StatusText { get; private set; }
    public IReadOnlyList<SetupUiCategoryModel> Categories { get; }
    public int SelectedCategoryIndex { get; private set; }
    public SetupUiCategoryModel SelectedCategory => Categories[SelectedCategoryIndex];
    public bool HasSetupData => Categories.Count > 0;
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
        Categories = categories;
    }

    public void SelectCategory(int index)
    {
        if (Categories.Count == 0)
        {
            SelectedCategoryIndex = 0;
            return;
        }

        SelectedCategoryIndex = Math.Clamp(index, 0, Categories.Count - 1);
    }

    public void UpdateVehicleContext(string? vehicleName, string? statusText)
    {
        VehicleName = string.IsNullOrWhiteSpace(vehicleName) ? "Factory Preview Car" : vehicleName;
        StatusText = string.IsNullOrWhiteSpace(statusText) ? "Setup preview ready." : statusText;
    }

    public IReadOnlyDictionary<string, float> CreateApplyPayload()
    {
        return Categories
            .SelectMany(category => category.Fields)
            .Where(field => field.IsBoundNumericField && field.IsDirty)
            .ToDictionary(field => field.VariableName!, field => field.RawNumericValue, StringComparer.OrdinalIgnoreCase);
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
                "Balance carcass support, pressure and wet-weather readiness before the stage opens up.",
                [
                    SetupUiFieldModel.Numeric("front-pressure", "Front pressure", "Cold baseline for front axle balance.", 24.6f, 19f, 32f, 0.1f, "psi"),
                    SetupUiFieldModel.Numeric("rear-pressure", "Rear pressure", "Helps rotation on entry without destabilising traction.", 23.8f, 19f, 32f, 0.1f, "psi"),
                    SetupUiFieldModel.Choice("compound", "Compound", "Preview-only compound selection for garage metadata wiring.", 1, ["Soft gravel", "Medium gravel", "Wet cut"]),
                    SetupUiFieldModel.Toggle("tyre-blankets", "Tyre blankets", "Represents pre-stage prep helpers for later service logic.", true, "Prepared", "Cold"),
                ]),
            new(
                "alignment",
                "Alignment",
                "Turn-in platform",
                "Dial in the chassis attitude for fast change of direction and confidence under braking.",
                [
                    SetupUiFieldModel.Numeric("front-camber", "Front camber", "Sharper response through loaded medium-speed corners.", -2.6f, -4.5f, -0.5f, 0.1f, "deg"),
                    SetupUiFieldModel.Numeric("rear-camber", "Rear camber", "Keeps traction stable when the road drops away.", -1.4f, -3.5f, 0f, 0.1f, "deg"),
                    SetupUiFieldModel.Numeric("front-toe", "Front toe", "Fine trim for turn-in and straight-line calm.", 0.08f, -0.30f, 0.30f, 0.01f, "deg"),
                    SetupUiFieldModel.Choice("caster-map", "Caster map", "Garage presets that can later map to real suspension metadata.", 0, ["Agile", "Balanced", "High-speed"]),
                ]),
            new(
                "dampers",
                "Dampers",
                "Kerb compliance",
                "Control platform movement over cuts, compressions and fast weight transfer.",
                [
                    SetupUiFieldModel.Numeric("front-bump", "Front bump", "Compression damping for attack over braking zones.", 7f, 1f, 12f, 0.5f, "clicks"),
                    SetupUiFieldModel.Numeric("front-rebound", "Front rebound", "Supports platform recovery after crests.", 9f, 1f, 12f, 0.5f, "clicks"),
                    SetupUiFieldModel.Numeric("rear-bump", "Rear bump", "Rear axle support when loading traction exits.", 6f, 1f, 12f, 0.5f, "clicks"),
                    SetupUiFieldModel.Numeric("rear-rebound", "Rear rebound", "Settles the car after rotation inputs.", 8f, 1f, 12f, 0.5f, "clicks"),
                ]),
            new(
                "differential",
                "Differential",
                "Drive & coast",
                "Preview shell for future diff metadata: power lock, coast support and preload tuning.",
                [
                    SetupUiFieldModel.Numeric("preload", "Preload", "Base locking force before throttle or brake ramps engage.", 110f, 40f, 200f, 5f, "Nm"),
                    SetupUiFieldModel.Numeric("power-ramp", "Power ramp", "Higher values resist inside-wheel spin on exit.", 48f, 20f, 80f, 1f, "%"),
                    SetupUiFieldModel.Numeric("coast-ramp", "Coast ramp", "Supports stability during trail braking.", 32f, 10f, 70f, 1f, "%"),
                    SetupUiFieldModel.Toggle("centre-lock", "Centre diff lock", "Preview toggle for AWD split logic.", false, "Locked", "Open"),
                ]),
            new(
                "transmission",
                "Transmission",
                "Launch & ratio prep",
                "Shape final drive response and quick-launch behaviour before live apply plumbing exists.",
                [
                    SetupUiFieldModel.Numeric("final-drive", "Final drive", "Overall gearing for stage top speed versus punch.", 4.38f, 3.50f, 5.20f, 0.01f, "ratio"),
                    SetupUiFieldModel.Numeric("launch-rpm", "Launch RPM", "Preview target for start-line bite.", 4200f, 2500f, 6500f, 50f, "rpm"),
                    SetupUiFieldModel.Choice("shift-map", "Shift map", "Future-ready drive mode preset slot.", 1, ["Smooth", "Stage", "Aggressive"]),
                ]),
        };

        var model = new SetupUiShellModel(
            title: "Garage Setup",
            subtitle: "Myra preview shell — ready for real metadata and apply-flow plumbing.",
            vehicleName: "Factory Preview Car",
            statusText: "Setup preview ready.",
            categories: categories);

        model.UpdateVehicleContext(vehicleName, statusText);
        return model;
    }

    public static SetupUiShellModel CreateFromRuntimeSetup(
        VehicleRuntimeSetupState? runtimeSetup,
        string? vehicleName = null,
        string? statusText = null)
    {
        if (runtimeSetup == null || runtimeSetup.Categories.Count == 0)
        {
            return CreatePreview(vehicleName, statusText);
        }

        var categories = runtimeSetup.Categories.Values
            .OrderBy(CategorySortKey)
            .ThenBy(category => category.Name, StringComparer.OrdinalIgnoreCase)
            .Select(CreateRuntimeCategory)
            .Where(category => category.Fields.Count > 0)
            .ToList();

        var subtitle = categories.Count == 0
            ? "BeamNG setup metadata is present, but no editable garage variables were exposed for this vehicle."
            : "BeamNG-derived garage setup. Only live or reload-backed fields are shown; apply updates live values immediately and flags geometry rebuilds for reload.";

        var model = new SetupUiShellModel(
            title: "Garage Setup",
            subtitle: subtitle,
            vehicleName: "Factory Preview Car",
            statusText: categories.Count == 0 ? "No editable setup variables were found." : "Setup data ready.",
            categories: categories.Count == 0
                ? [new SetupUiCategoryModel("empty", "No Setup Data", "No editable variables", "This vehicle does not currently expose BeamNG setup metadata.", [])]
                : categories);

        model.UpdateVehicleContext(vehicleName, statusText ?? model.StatusText);
        return model;
    }

    private static SetupUiCategoryModel CreateRuntimeCategory(VehicleRuntimeSetupCategory category)
    {
        var duplicatedLabels = category.Variables
            .Select(ResolveVariableLabel)
            .GroupBy(label => label, StringComparer.OrdinalIgnoreCase)
            .Where(group => group.Count() > 1)
            .Select(group => group.Key)
            .ToHashSet(StringComparer.OrdinalIgnoreCase);
        var fields = category.Variables
            .Select(variable => SetupUiFieldModel.FromVariable(variable, BuildRuntimeFieldLabel(variable, duplicatedLabels)))
            .ToList();

        return new SetupUiCategoryModel(
            category.Name.ToLowerInvariant().Replace(' ', '-'),
            category.Name,
            BuildCategoryTagline(category.Name),
            BuildCategoryDescription(category.Name),
            fields);
    }

    private static string BuildRuntimeFieldLabel(VehicleSetupVariable variable, IReadOnlySet<string> duplicatedLabels)
    {
        var label = ResolveVariableLabel(variable);
        if (!duplicatedLabels.Contains(label))
        {
            return label;
        }

        var prefix = !string.IsNullOrWhiteSpace(variable.SubCategory)
            ? variable.SubCategory.Trim()
            : variable.Axle switch
            {
                VehicleSetupAxle.Front => "Front",
                VehicleSetupAxle.Rear => "Rear",
                VehicleSetupAxle.Both => "Both",
                _ => string.Empty,
            };

        if (string.IsNullOrWhiteSpace(prefix) ||
            label.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
        {
            return label;
        }

        return $"{prefix} {label}";
    }

    private static string ResolveVariableLabel(VehicleSetupVariable variable)
    {
        return string.IsNullOrWhiteSpace(variable.Title) ? variable.Name.Replace('_', ' ').Replace('-', ' ').Trim() : variable.Title;
    }

    private static int CategorySortKey(VehicleRuntimeSetupCategory category)
    {
        return category.Name.ToLowerInvariant() switch
        {
            var name when name.Contains("wheel") || name.Contains("tyre") || name.Contains("tire") => 0,
            var name when name.Contains("alignment") => 1,
            var name when name.Contains("suspension") => 2,
            var name when name.Contains("brake") => 3,
            var name when name.Contains("differential") => 4,
            var name when name.Contains("transmission") => 5,
            var name when name.Contains("engine") => 6,
            var name when name.Contains("chassis") || name.Contains("fuel") => 7,
            _ => 99,
        };
    }

    private static string BuildCategoryTagline(string categoryName)
    {
        return categoryName.ToLowerInvariant() switch
        {
            var name when name.Contains("wheel") || name.Contains("tyre") || name.Contains("tire") => "Contact patch",
            var name when name.Contains("alignment") => "Turn-in platform",
            var name when name.Contains("suspension") => "Platform control",
            var name when name.Contains("brake") => "Stopping balance",
            var name when name.Contains("differential") => "Drive & coast",
            var name when name.Contains("transmission") => "Ratio prep",
            var name when name.Contains("engine") => "Power delivery",
            var name when name.Contains("chassis") || name.Contains("fuel") => "Race trim",
            _ => "Setup controls",
        };
    }

    private static string BuildCategoryDescription(string categoryName)
    {
        return categoryName.ToLowerInvariant() switch
        {
            var name when name.Contains("wheel") || name.Contains("tyre") || name.Contains("tire") => "Tune tyre pressure and wheel package values sourced from the active BeamNG pressure-wheel definitions.",
            var name when name.Contains("alignment") => "Adjust alignment values that shape entry response, mid-corner grip, and high-speed stability.",
            var name when name.Contains("suspension") => "Spring, damping, ride-height, and anti-roll settings feed directly into the current BEPU suspension mapping.",
            var name when name.Contains("brake") => "Dial brake balance and torque multipliers from the active BeamNG brake setup.",
            var name when name.Contains("differential") => "Configure final drive, preload, lock rates, and torque split from the active driveline parts.",
            var name when name.Contains("transmission") => "Edit forward and reverse gear ratios carried through from the active gearbox definition.",
            var name when name.Contains("engine") => "Set rev limiter and boost targets exposed by the active engine configuration.",
            var name when name.Contains("chassis") || name.Contains("fuel") => "Set fuel load and chassis-side service values that may require a full vehicle rebuild.",
            _ => "BeamNG-derived setup values exposed for this vehicle.",
        };
    }
}

public sealed class SetupUiCategoryModel
{
    public string Id { get; }
    public string Title { get; }
    public string Tagline { get; }
    public string Description { get; }
    public IReadOnlyList<SetupUiFieldModel> Fields { get; }
    public int PendingChangeCount => Fields.Count(setupField => setupField.IsDirty);
    public int PendingLiveChangeCount => Fields.Count(setupField => setupField.IsDirty && setupField.ApplyMode == VehicleSetupApplyMode.Live);
    public int PendingReloadChangeCount => Fields.Count(setupField => setupField.IsDirty && setupField.ApplyMode == VehicleSetupApplyMode.Reload);
    public int NonDefaultChangeCount => Fields.Count(setupField => setupField.CanResetToDefault);

    public SetupUiCategoryModel(string id, string title, string tagline, string description, IReadOnlyList<SetupUiFieldModel> fields)
    {
        Id = id;
        Title = title;
        Tagline = tagline;
        Description = description;
        Fields = fields;
    }
}

public sealed class SetupUiFieldModel
{
    public string Id { get; }
    public string Label { get; }
    public string Description { get; }
    public SetupUiEditorKind EditorKind { get; }
    public string? VariableName { get; }
    public VehicleSetupApplyMode ApplyMode { get; }
    public float NumericValue { get; private set; }
    public float Minimum { get; }
    public float Maximum { get; }
    public float Step { get; }
    public string Unit { get; }
    public bool ToggleValue { get; private set; }
    public string ToggleOnLabel { get; }
    public string ToggleOffLabel { get; }
    public IReadOnlyList<string> ChoiceOptions { get; }
    public int ChoiceIndex { get; private set; }
    public bool IsBoundNumericField => EditorKind == SetupUiEditorKind.Numeric && !string.IsNullOrWhiteSpace(VariableName);
    public float RawNumericValue => NumericValue;
    public string DefaultDisplayValue => EditorKind switch
    {
        SetupUiEditorKind.Numeric => FormatNumericValue(_defaultNumericValue, Unit, Step),
        SetupUiEditorKind.Toggle => _defaultToggleValue ? ToggleOnLabel : ToggleOffLabel,
        SetupUiEditorKind.Choice => ChoiceOptions.Count == 0 ? string.Empty : ChoiceOptions[_defaultChoiceIndex],
        _ => string.Empty,
    };
    public bool CanResetToDefault => EditorKind switch
    {
        SetupUiEditorKind.Numeric => Math.Abs(NumericValue - _defaultNumericValue) > ResolveDifferenceThreshold(_defaultNumericValue),
        SetupUiEditorKind.Toggle => ToggleValue != _defaultToggleValue,
        SetupUiEditorKind.Choice => ChoiceIndex != _defaultChoiceIndex,
        _ => false,
    };
    public bool IsDirty => EditorKind switch
    {
        SetupUiEditorKind.Numeric => Math.Abs(NumericValue - _committedNumericValue) > ResolveDirtyThreshold(),
        SetupUiEditorKind.Toggle => ToggleValue != _committedToggleValue,
        SetupUiEditorKind.Choice => ChoiceIndex != _committedChoiceIndex,
        _ => false,
    };

    private float _committedNumericValue;
    private bool _committedToggleValue;
    private int _committedChoiceIndex;
    private readonly float _defaultNumericValue;
    private readonly bool _defaultToggleValue;
    private readonly int _defaultChoiceIndex;

    private SetupUiFieldModel(
        string id,
        string label,
        string description,
        SetupUiEditorKind editorKind,
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
        string? variableName,
        VehicleSetupApplyMode applyMode,
        float? defaultNumericValue = null,
        bool? defaultToggleValue = null,
        int? defaultChoiceIndex = null)
    {
        Id = id;
        Label = label;
        Description = description;
        EditorKind = editorKind;
        NumericValue = numericValue;
        Minimum = minimum;
        Maximum = maximum;
        Step = step;
        Unit = unit;
        ToggleValue = toggleValue;
        ToggleOnLabel = toggleOnLabel;
        ToggleOffLabel = toggleOffLabel;
        ChoiceOptions = choiceOptions ?? Array.Empty<string>();
        ChoiceIndex = ChoiceOptions.Count == 0 ? 0 : Math.Clamp(choiceIndex, 0, ChoiceOptions.Count - 1);
        VariableName = variableName;
        ApplyMode = applyMode;
        _committedNumericValue = numericValue;
        _committedToggleValue = toggleValue;
        _committedChoiceIndex = ChoiceIndex;
        _defaultNumericValue = Math.Clamp(defaultNumericValue ?? numericValue, Minimum, Maximum);
        _defaultToggleValue = defaultToggleValue ?? toggleValue;
        _defaultChoiceIndex = ChoiceOptions.Count == 0
            ? 0
            : Math.Clamp(defaultChoiceIndex ?? ChoiceIndex, 0, ChoiceOptions.Count - 1);
    }

    public string DisplayValue => EditorKind switch
    {
        SetupUiEditorKind.Numeric => FormatNumericValue(NumericValue, Unit, Step),
        SetupUiEditorKind.Toggle => ToggleValue ? ToggleOnLabel : ToggleOffLabel,
        SetupUiEditorKind.Choice => ChoiceOptions.Count == 0 ? string.Empty : ChoiceOptions[ChoiceIndex],
        _ => string.Empty,
    };

    public string ApplyHint => ApplyMode == VehicleSetupApplyMode.Live ? "Live on apply" : "Reload on apply";

    public float NormalizedNumericValue => Maximum <= Minimum
        ? 0f
        : (NumericValue - Minimum) / (Maximum - Minimum);

    public void SetNumericValue(float value)
    {
        NumericValue = Math.Clamp(value, Minimum, Maximum);
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
        NumericValue = _committedNumericValue;
        ToggleValue = _committedToggleValue;
        ChoiceIndex = _committedChoiceIndex;
    }

    public void AcceptPendingValue()
    {
        _committedNumericValue = NumericValue;
        _committedToggleValue = ToggleValue;
        _committedChoiceIndex = ChoiceIndex;
    }

    public void ResetToDefaultValue()
    {
        NumericValue = _defaultNumericValue;
        ToggleValue = _defaultToggleValue;
        ChoiceIndex = _defaultChoiceIndex;
    }

    public static SetupUiFieldModel Numeric(string id, string label, string description, float value, float minimum, float maximum, float step, string unit)
        => new(id, label, description, SetupUiEditorKind.Numeric, value, minimum, maximum, step, unit, false, string.Empty, string.Empty, null, 0, null, VehicleSetupApplyMode.Live);

    public static SetupUiFieldModel Choice(string id, string label, string description, int choiceIndex, IReadOnlyList<string> options)
        => new(id, label, description, SetupUiEditorKind.Choice, 0f, 0f, 0f, 0f, string.Empty, false, string.Empty, string.Empty, options, choiceIndex, null, VehicleSetupApplyMode.Reload);

    public static SetupUiFieldModel Toggle(string id, string label, string description, bool value, string onLabel, string offLabel)
        => new(id, label, description, SetupUiEditorKind.Toggle, 0f, 0f, 1f, 1f, string.Empty, value, onLabel, offLabel, null, 0, null, VehicleSetupApplyMode.Reload);

    public static SetupUiFieldModel FromVariable(VehicleSetupVariable variable, string? labelOverride = null)
    {
        var value = variable.Value;
        var (minimum, maximum, usedDisplayBounds) = ResolveBounds(variable, value);
        var step = ResolveStep(variable, minimum, maximum, usedDisplayBounds);
        var label = string.IsNullOrWhiteSpace(labelOverride) ? ResolveVariableLabel(variable) : labelOverride;
        var description = string.IsNullOrWhiteSpace(variable.Description)
            ? BuildFallbackDescription(variable)
            : variable.Description;

        return new SetupUiFieldModel(
            id: variable.Name,
            label: label,
            description: description,
            editorKind: SetupUiEditorKind.Numeric,
            numericValue: value,
            minimum: minimum,
            maximum: maximum,
            step: step,
            unit: variable.Unit,
            toggleValue: false,
            toggleOnLabel: string.Empty,
            toggleOffLabel: string.Empty,
            choiceOptions: null,
            choiceIndex: 0,
            variableName: variable.Name,
            applyMode: variable.ApplyMode,
            defaultNumericValue: variable.DefaultValue);
    }

    private float ResolveDirtyThreshold()
    {
        return ResolveDifferenceThreshold(_committedNumericValue);
    }

    private float ResolveDifferenceThreshold(float baseline)
    {
        if (Step > 0f)
        {
            return Step * 0.25f;
        }

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

    private static (float Minimum, float Maximum, bool UsedDisplayBounds) ResolveBounds(VehicleSetupVariable variable, float value)
    {
        var rawMinimum = variable.MinValue.HasValue && float.IsFinite(variable.MinValue.Value)
            ? variable.MinValue.Value
            : value - ResolveFallbackSpan(variable, value);
        var rawMaximum = variable.MaxValue.HasValue && float.IsFinite(variable.MaxValue.Value)
            ? variable.MaxValue.Value
            : value + ResolveFallbackSpan(variable, value);
        var hasDisplayMinimum = variable.MinDisplayValue.HasValue && float.IsFinite(variable.MinDisplayValue.Value);
        var hasDisplayMaximum = variable.MaxDisplayValue.HasValue && float.IsFinite(variable.MaxDisplayValue.Value);

        if (rawMaximum <= rawMinimum)
        {
            rawMaximum = rawMinimum + Math.Max(ResolveFallbackSpan(variable, value) * 2f, 1f);
        }

        if (hasDisplayMinimum || hasDisplayMaximum)
        {
            var displayMinimum = hasDisplayMinimum ? variable.MinDisplayValue!.Value : rawMinimum;
            var displayMaximum = hasDisplayMaximum ? variable.MaxDisplayValue!.Value : rawMaximum;
            if (displayMaximum <= displayMinimum)
            {
                displayMaximum = displayMinimum + Math.Max(ResolveFallbackSpan(variable, value) * 2f, 1f);
            }

            if (ContainsValue(displayMinimum, displayMaximum, value) &&
                DisplayBoundsMatchRawScale(rawMinimum, rawMaximum, displayMinimum, displayMaximum))
            {
                return (displayMinimum, displayMaximum, true);
            }
        }

        if (!ContainsValue(rawMinimum, rawMaximum, value))
        {
            rawMinimum = Math.Min(rawMinimum, value);
            rawMaximum = Math.Max(rawMaximum, value);
        }

        return (rawMinimum, rawMaximum, false);
    }

    private static bool ContainsValue(float minimum, float maximum, float value)
    {
        return minimum <= value && value <= maximum;
    }

    private static bool DisplayBoundsMatchRawScale(float rawMinimum, float rawMaximum, float displayMinimum, float displayMaximum)
    {
        var rawSpan = Math.Abs(rawMaximum - rawMinimum);
        var displaySpan = Math.Abs(displayMaximum - displayMinimum);
        if (rawSpan <= 0f || displaySpan <= 0f)
        {
            return true;
        }

        var spanRatio = displaySpan / rawSpan;
        return spanRatio >= 0.25f && spanRatio <= 4f;
    }

    private static float ResolveStep(VehicleSetupVariable variable, float minimum, float maximum, bool usedDisplayBounds)
    {
        var range = Math.Abs(maximum - minimum);
        if (variable.Step.HasValue && variable.Step.Value > 0f && float.IsFinite(variable.Step.Value))
        {
            var step = variable.Step.Value;
            if (usedDisplayBounds || step < range)
            {
                return step;
            }
        }

        if (range <= 0.2f)
        {
            return 0.001f;
        }

        if (range <= 2f)
        {
            return 0.01f;
        }

        if (range <= 20f)
        {
            return 0.1f;
        }

        return 1f;
    }

    private static float ResolveFallbackSpan(VehicleSetupVariable variable, float value)
    {
        if (!string.IsNullOrWhiteSpace(variable.Unit))
        {
            if (variable.Unit.Contains("psi", StringComparison.OrdinalIgnoreCase))
            {
                return 8f;
            }

            if (variable.Unit.Contains("rpm", StringComparison.OrdinalIgnoreCase))
            {
                return 1500f;
            }

            if (variable.Unit.Contains("%", StringComparison.OrdinalIgnoreCase))
            {
                return 25f;
            }

            if (variable.Unit.Contains("deg", StringComparison.OrdinalIgnoreCase))
            {
                return 2f;
            }

            if (variable.Unit.Equals("m", StringComparison.OrdinalIgnoreCase))
            {
                return 0.05f;
            }

            if (variable.Unit.Contains("Nm", StringComparison.OrdinalIgnoreCase))
            {
                return 100f;
            }
        }

        var magnitude = Math.Max(Math.Abs(value), 1f);
        return magnitude * 0.5f;
    }

    private static string ResolveVariableLabel(VehicleSetupVariable variable)
    {
        return string.IsNullOrWhiteSpace(variable.Title) ? Humanize(variable.Name) : variable.Title;
    }

    private static string BuildFallbackDescription(VehicleSetupVariable variable)
    {
        var axle = variable.Axle switch
        {
            VehicleSetupAxle.Front => "Front axle",
            VehicleSetupAxle.Rear => "Rear axle",
            VehicleSetupAxle.Both => "Both axles",
            _ => "Vehicle-wide",
        };

        return $"{axle} BeamNG setup value.";
    }

    private static string Humanize(string raw)
    {
        if (string.IsNullOrWhiteSpace(raw))
        {
            return string.Empty;
        }

        return raw
            .Replace('_', ' ')
            .Replace('-', ' ')
            .Trim();
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
