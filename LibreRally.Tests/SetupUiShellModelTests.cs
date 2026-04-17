using System.Collections.Generic;
using LibreRally.HUD;
using LibreRally.Vehicle;

namespace LibreRally.Tests;

public class SetupUiShellModelTests
{
    [Fact]
    public void CreatePreview_SeedsGarageCategoriesAndVehicleContext()
    {
        SetupUiShellModel shell = SetupUiShellModel.CreatePreview("Sunburst RS", "Loaded: Sunburst RS");

        Assert.Equal("Garage Setup", shell.Title);
        Assert.Equal("Sunburst RS", shell.VehicleName);
        Assert.Equal("Loaded: Sunburst RS", shell.StatusText);
        Assert.True(shell.Categories.Count >= 5);
        Assert.Equal("Tyres & Pressures", shell.SelectedCategory.Title);
    }

    [Fact]
    public void SelectCategory_ClampsToAvailableRange()
    {
        SetupUiShellModel shell = SetupUiShellModel.CreatePreview();

        shell.SelectCategory(-5);
        Assert.Equal(0, shell.SelectedCategoryIndex);

        shell.SelectCategory(999);
        Assert.Equal(shell.Categories.Count - 1, shell.SelectedCategoryIndex);
    }

    [Fact]
    public void NumericField_ClampsAndFormatsDisplayValue()
    {
        SetupUiFieldModel field = SetupUiFieldModel.Numeric("ride-height", "Ride height", "Preview field", 72f, 60f, 90f, 0.5f, "mm");

        field.SetNumericValue(120f);

        Assert.Equal(90f, field.NumericValue);
        Assert.Equal("90 mm", field.DisplayValue);
        Assert.Equal(1f, field.NormalizedNumericValue, 3);
    }

    [Fact]
    public void ChoiceAndToggleFields_CycleAndFormatState()
    {
        SetupUiFieldModel choice = SetupUiFieldModel.Choice("map", "Shift map", "Preview choice", 0, ["Smooth", "Stage", "Aggressive"]);
        SetupUiFieldModel toggle = SetupUiFieldModel.Toggle("lock", "Centre diff lock", "Preview toggle", false, "Locked", "Open");

        choice.CycleChoice();
        choice.CycleChoice();
        toggle.SetToggleValue(true);

        Assert.Equal("Aggressive", choice.DisplayValue);
        Assert.Equal("Locked", toggle.DisplayValue);
    }

    [Fact]
    public void FieldDefaults_CanBeResetWithoutChangingCommittedValue()
    {
        SetupUiFieldModel numeric = SetupUiFieldModel.Numeric("ride-height", "Ride height", "Preview field", 72f, 60f, 90f, 0.5f, "mm");
        SetupUiFieldModel choice = SetupUiFieldModel.Choice("map", "Shift map", "Preview choice", 0, ["Smooth", "Stage", "Aggressive"]);
        SetupUiFieldModel toggle = SetupUiFieldModel.Toggle("lock", "Centre diff lock", "Preview toggle", false, "Locked", "Open");

        numeric.SetNumericValue(80f);
        numeric.AcceptPendingValue();
        choice.CycleChoice();
        toggle.SetToggleValue(true);

        Assert.True(numeric.CanResetToDefault);
        Assert.Equal("72 mm", numeric.DefaultDisplayValue);
        Assert.False(numeric.IsDirty);
        Assert.True(choice.CanResetToDefault);
        Assert.True(toggle.CanResetToDefault);

        numeric.ResetToDefaultValue();
        choice.ResetToDefaultValue();
        toggle.ResetToDefaultValue();

        Assert.Equal(72f, numeric.NumericValue);
        Assert.True(numeric.IsDirty);
        Assert.Equal("Smooth", choice.DisplayValue);
        Assert.Equal("Open", toggle.DisplayValue);
    }

    [Fact]
    public void CreateFromRuntimeSetup_BuildsEditableShellAndTracksPendingChanges()
    {
        var variable = new VehicleSetupVariable
        {
            Name = "spring_F",
            Title = "Front spring rate",
            Description = "Front axle spring stiffness.",
            Category = "Suspension",
            SubCategory = "Front",
            Value = 60000f,
            DefaultValue = 60000f,
            MinValue = 40000f,
            MaxValue = 80000f,
            Step = 500f,
            Unit = "N/m",
            ApplyMode = VehicleSetupApplyMode.Live,
            TargetKind = VehicleSetupTargetKind.SuspensionSpringRate,
            Axle = VehicleSetupAxle.Front,
        };
        var runtimeSetup = VehicleRuntimeSetupState.From(new VehicleSetupSchema
        {
            Variables = new Dictionary<string, VehicleSetupVariable>(System.StringComparer.OrdinalIgnoreCase)
            {
                [variable.Name] = variable,
            },
        });

        var shell = SetupUiShellModel.CreateFromRuntimeSetup(runtimeSetup, "Sunburst RS", "Setup data ready.");
        var field = Assert.Single(shell.SelectedCategory.Fields);

        field.SetNumericValue(62500f);
        var payload = shell.CreateApplyPayload();

        Assert.Equal("Sunburst RS", shell.VehicleName);
        Assert.True(shell.CanApply);
        Assert.Equal(1, shell.PendingChangeCount);
        Assert.Equal(1, shell.PendingLiveChangeCount);
        Assert.Equal(62500f, payload["spring_F"]);

        shell.ResetPendingChanges();

        Assert.False(shell.CanApply);
        Assert.Equal(60000f, field.NumericValue);
    }

    [Fact]
    public void CreateFromRuntimeSetup_IgnoresDisplayBoundsThatDoNotContainRawValue()
    {
        var variable = new VehicleSetupVariable
        {
            Name = "brakebias",
            Title = "Front/Rear Bias",
            Category = "Brakes",
            Value = 0.59f,
            DefaultValue = 0.59f,
            MinValue = 0f,
            MaxValue = 1f,
            MinDisplayValue = 0f,
            MaxDisplayValue = 100f,
            Step = 1f,
            Unit = "%",
            ApplyMode = VehicleSetupApplyMode.Live,
            TargetKind = VehicleSetupTargetKind.BrakeBias,
        };
        var runtimeSetup = VehicleRuntimeSetupState.From(new VehicleSetupSchema
        {
            Variables = new Dictionary<string, VehicleSetupVariable>(System.StringComparer.OrdinalIgnoreCase)
            {
                [variable.Name] = variable,
            },
        });

        var shell = SetupUiShellModel.CreateFromRuntimeSetup(runtimeSetup);
        var field = Assert.Single(shell.SelectedCategory.Fields);

        Assert.Equal(0f, field.Minimum);
        Assert.Equal(1f, field.Maximum);
        Assert.Equal(0.01f, field.Step);
        Assert.InRange(field.NumericValue, field.Minimum, field.Maximum);
    }

    [Fact]
    public void ResetToDefaults_UsesVariableDefaultAndCreatesApplyPayload()
    {
        var variable = new VehicleSetupVariable
        {
            Name = "spring_F",
            Title = "Front spring rate",
            Category = "Suspension",
            Value = 62500f,
            DefaultValue = 60000f,
            MinValue = 40000f,
            MaxValue = 80000f,
            Step = 500f,
            Unit = "N/m",
            ApplyMode = VehicleSetupApplyMode.Live,
            TargetKind = VehicleSetupTargetKind.SuspensionSpringRate,
            Axle = VehicleSetupAxle.Front,
        };
        var runtimeSetup = VehicleRuntimeSetupState.From(new VehicleSetupSchema
        {
            Variables = new Dictionary<string, VehicleSetupVariable>(System.StringComparer.OrdinalIgnoreCase)
            {
                [variable.Name] = variable,
            },
        });

        var shell = SetupUiShellModel.CreateFromRuntimeSetup(runtimeSetup);
        var field = Assert.Single(shell.SelectedCategory.Fields);

        Assert.True(field.CanResetToDefault);
        Assert.True(shell.CanResetToDefaults);

        shell.ResetToDefaults();
        var payload = shell.CreateApplyPayload();

        Assert.Equal(60000f, field.NumericValue);
        Assert.Equal(60000f, field.RawNumericValue);
        Assert.True(shell.CanApply);
        Assert.Single(payload);
        Assert.Equal(60000f, payload["spring_F"]);
    }

    [Fact]
    public void CreateFromRuntimeSetup_PrefixesDuplicateLabelsAndSuppressesDuplicateAlignmentEntries()
    {
        var frontCamber = new VehicleSetupVariable
        {
            Name = "camber_F",
            Title = "Camber Adjust",
            Description = "Adjusts the wheel camber angle at the strut/hub joint.",
            Category = "Wheel Alignment",
            SubCategory = "Front",
            Value = 1f,
            DefaultValue = 1f,
            MinValue = 0.95f,
            MaxValue = 1.05f,
            ApplyMode = VehicleSetupApplyMode.Live,
            TargetKind = VehicleSetupTargetKind.AlignmentCamber,
            Axle = VehicleSetupAxle.Front,
        };
        var frontUpperCamber = new VehicleSetupVariable
        {
            Name = "camber_upper_F",
            Title = "Camber Adjust",
            Description = "Adjusts the wheel camber angle at the strut/hub joint.",
            Category = "Wheel Alignment",
            SubCategory = "Front",
            Value = 0f,
            DefaultValue = 0f,
            MinValue = -0.04f,
            MaxValue = 0.04f,
            ApplyMode = VehicleSetupApplyMode.Reload,
            TargetKind = VehicleSetupTargetKind.AlignmentCamber,
            Axle = VehicleSetupAxle.Front,
        };
        var rearCamber = new VehicleSetupVariable
        {
            Name = "camber_R",
            Title = "Camber Adjust",
            Description = "Adjusts the wheel camber angle at the strut/hub joint.",
            Category = "Wheel Alignment",
            SubCategory = "Rear",
            Value = 0.99f,
            DefaultValue = 0.99f,
            MinValue = 0.95f,
            MaxValue = 1.05f,
            ApplyMode = VehicleSetupApplyMode.Live,
            TargetKind = VehicleSetupTargetKind.AlignmentCamber,
            Axle = VehicleSetupAxle.Rear,
        };

        var runtimeSetup = VehicleRuntimeSetupState.From(new VehicleSetupSchema
        {
            Variables = new Dictionary<string, VehicleSetupVariable>(System.StringComparer.OrdinalIgnoreCase)
            {
                [frontCamber.Name] = frontCamber,
                [frontUpperCamber.Name] = frontUpperCamber,
                [rearCamber.Name] = rearCamber,
            },
        });

        var alignmentCategory = Assert.Single(runtimeSetup.Categories.Values);
        Assert.Equal(["camber_F", "camber_R"], alignmentCategory.Variables.Select(variable => variable.Name).OrderBy(name => name).ToArray());

        var shell = SetupUiShellModel.CreateFromRuntimeSetup(runtimeSetup);
        var labels = shell.SelectedCategory.Fields.Select(field => field.Label).ToArray();

        Assert.Equal(["Front Camber Adjust", "Rear Camber Adjust"], labels);
    }
}
