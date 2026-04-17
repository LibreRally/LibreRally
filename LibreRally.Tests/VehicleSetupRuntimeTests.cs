using System;
using System.Collections.Generic;
using System.IO;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using Stride.Engine;
using Xunit;

namespace LibreRally.Tests;

public sealed class VehicleSetupRuntimeTests
{
    [Fact]
    public void BasicCar_SetupSchema_ShouldClassifyRuntimeSuspensionAlignmentAndWheelEntries()
    {
        var config = PcConfigLoader.Load(Path.Combine(GetBasicCarFolder(), "basic_car.pc"));
        var definition = JBeamAssembler.Assemble(GetBasicCarFolder(), config);

        var spring = definition.Setup.Variables["spring_F"];
        var camber = definition.Setup.Variables["camber_F"];
        var caster = definition.Setup.Variables["caster_F"];
        var toe = definition.Setup.Variables["toe_F"];
        var wheelOffset = definition.Setup.Variables["trackoffset_F"];

        Assert.Equal(VehicleSetupApplyMode.Live, spring.ApplyMode);
        Assert.Equal("spring_F_asphalt", spring.EffectiveValueName);
        Assert.Equal(52000f, spring.Value);
        Assert.Equal(VehicleSetupAxle.Front, spring.Axle);

        Assert.Equal(VehicleSetupApplyMode.Live, camber.ApplyMode);
        Assert.Equal(VehicleSetupTargetKind.AlignmentCamber, camber.TargetKind);

        Assert.Equal(VehicleSetupApplyMode.Reload, caster.ApplyMode);
        Assert.Equal(VehicleSetupTargetKind.AlignmentCaster, caster.TargetKind);

        Assert.Equal(VehicleSetupApplyMode.Reload, toe.ApplyMode);
        Assert.Equal(VehicleSetupTargetKind.AlignmentToe, toe.TargetKind);

        Assert.Equal(VehicleSetupApplyMode.Reload, wheelOffset.ApplyMode);
        Assert.Equal(VehicleSetupTargetKind.WheelOffset, wheelOffset.TargetKind);
    }

    [Fact]
    public void SunburstRally_SetupSchema_ShouldExposeRallyAlignmentEntries()
    {
        var sunburstFolder = GetSunburstFolder();
        var config = PcConfigLoader.Load(Path.Combine(sunburstFolder, "rally_pro_asphalt.pc"));
        var definition = JBeamAssembler.Assemble(sunburstFolder, config);

        Assert.Equal(VehicleSetupApplyMode.Live, definition.Setup.Variables["camber_F_rally"].ApplyMode);
        Assert.Equal(VehicleSetupApplyMode.Reload, definition.Setup.Variables["caster_F_rally"].ApplyMode);
        Assert.Equal(VehicleSetupApplyMode.Reload, definition.Setup.Variables["toe_F_rally"].ApplyMode);
        Assert.Equal(VehicleSetupApplyMode.Reload, definition.Setup.Variables["trackoffset_F"].ApplyMode);
    }

    [Fact]
    public void SunburstRally_RuntimeSetup_ShouldPreferSingleRuntimeBackedAlignmentControlPerAxle()
    {
        var sunburstFolder = GetSunburstFolder();
        var config = PcConfigLoader.Load(Path.Combine(sunburstFolder, "rally_pro_asphalt.pc"));
        var definition = JBeamAssembler.Assemble(sunburstFolder, config);

        var runtimeSetup = VehicleRuntimeSetupState.From(definition.Setup);
        var alignmentCategory = runtimeSetup.Categories["Wheel Alignment"];

        Assert.Contains(alignmentCategory.Variables, variable => variable.Name == "camber_F_rally");
        Assert.Contains(alignmentCategory.Variables, variable => variable.Name == "camber_R_rally");
        Assert.DoesNotContain(alignmentCategory.Variables, variable => variable.Name == "camber_upper_F_rally");
        Assert.DoesNotContain(alignmentCategory.Variables, variable => variable.Name == "camber_upper_R_rally");
        Assert.DoesNotContain(alignmentCategory.Variables, variable => variable.Name == "caster_upper_F_rally");
        Assert.All(
            alignmentCategory.Variables.Where(variable => variable.TargetKind == VehicleSetupTargetKind.AlignmentCamber),
            variable => Assert.True(VehicleSetupSupport.IsLiveCamberVariable(variable)));
    }

    [Fact]
    public void BasicCar_SetupSchema_ShouldClassifyDrivetrainBrakeEngineAndFuelEntries()
    {
        var config = PcConfigLoader.Load(Path.Combine(GetBasicCarFolder(), "basic_car.pc"));
        var definition = JBeamAssembler.Assemble(GetBasicCarFolder(), config);

        Assert.Equal(VehicleSetupTargetKind.BrakeBias, definition.Setup.Variables["brakebias"].TargetKind);
        Assert.Equal(VehicleSetupApplyMode.Live, definition.Setup.Variables["brakebias"].ApplyMode);
        Assert.Equal(VehicleSetupTargetKind.BrakeStrength, definition.Setup.Variables["brakestrength"].TargetKind);
        Assert.Equal(VehicleSetupTargetKind.DifferentialFinalDrive, definition.Setup.Variables["finaldrive_R"].TargetKind);
        Assert.Equal(VehicleSetupTargetKind.DifferentialPreload, definition.Setup.Variables["lsdpreload_R"].TargetKind);
        Assert.Equal(VehicleSetupTargetKind.DifferentialPowerLock, definition.Setup.Variables["lsdlockcoef_R"].TargetKind);
        Assert.Equal(VehicleSetupTargetKind.DifferentialCoastLock, definition.Setup.Variables["lsdlockcoefrev_R"].TargetKind);
        Assert.Equal(VehicleSetupTargetKind.TransmissionGearRatio, definition.Setup.Variables["gear_1"].TargetKind);
        Assert.Equal(VehicleSetupTargetKind.EngineRevLimiterRpm, definition.Setup.Variables["revLimiterRPM"].TargetKind);
        Assert.Equal(VehicleSetupTargetKind.EngineRevLimiterCutTime, definition.Setup.Variables["revLimiterCutTime"].TargetKind);
        Assert.Equal(VehicleSetupTargetKind.FuelStartingVolume, definition.Setup.Variables["fuel"].TargetKind);
        Assert.Equal(VehicleSetupApplyMode.Reload, definition.Setup.Variables["fuel"].ApplyMode);
    }

    [Fact]
    public void Apply_LiveSuspensionAndCamberChanges_ShouldUpdateRuntimeComponents()
    {
        var config = PcConfigLoader.Load(Path.Combine(GetBasicCarFolder(), "basic_car.pc"));
        var definition = JBeamAssembler.Assemble(GetBasicCarFolder(), config);
        var result = VehiclePhysicsBuilder.Build(definition);
        var loadedVehicle = CreateLoadedVehicle(definition, result, config);

        var frontLeft = Assert.IsType<WheelSettings>(loadedVehicle.WheelFL.Get<WheelSettings>());
        var beforeSpringFrequency = Assert.IsType<Stride.BepuPhysics.Constraints.LinearAxisServoConstraintComponent>(frontLeft.SuspensionSpringServo).SpringFrequency;

        var applyResult = VehicleSetupRuntime.Apply(
            loadedVehicle,
            new Dictionary<string, float>
            {
                ["spring_F"] = 61000f,
                ["damp_bump_F"] = 4800f,
                ["damp_rebound_F"] = 9200f,
                ["springheight_F"] = -0.02f,
                ["arb_spring_F"] = 24000f,
                ["camber_F"] = 0.97f,
            });

        Assert.True(applyResult.AppliedLiveChanges);
        Assert.False(applyResult.RequiresReload);
        Assert.Equal("spring_F_asphalt", definition.Setup.Variables["spring_F"].EffectiveValueName);
        Assert.Equal(61000f, definition.Vars["spring_F_asphalt"]);
        Assert.Equal(61000f, config.Vars["spring_F_asphalt"]);
        Assert.Equal(0.02f, frontLeft.SuspensionTargetOffset, 3);
        Assert.NotEqual(beforeSpringFrequency, Assert.IsType<Stride.BepuPhysics.Constraints.LinearAxisServoConstraintComponent>(frontLeft.SuspensionSpringServo).SpringFrequency);
        Assert.Equal(24000f, Assert.IsType<VehicleDynamicsSystem>(loadedVehicle.CarComponent.Dynamics).FrontAntiRollStiffness);
        Assert.Equal(RallyCarComponent.ConvertCamberPrecompressionToRadians(0.97f), loadedVehicle.CarComponent.FrontStaticCamberRadians, 3);
    }

    [Fact]
    public void Apply_WheelPressureAndOffsetChanges_ShouldSplitLiveAndReloadPaths()
    {
        using var workspace = new TempWorkspace();
        var vehicleFolder = Path.Combine(workspace.RootPath, "wheel_setup_car");
        Directory.CreateDirectory(vehicleFolder);
        File.WriteAllText(
            Path.Combine(vehicleFolder, "wheel_setup_car.jbeam"),
            """
            {
              "wheel_setup_car": {
                "slotType": "main",
                "slots": [
                  ["type", "default", "description"],
                  ["wheeldata_F", "wheeldata_front", "Front wheel data"],
                  ["wheeldata_R", "wheeldata_rear", "Rear wheel data"]
                ],
                "nodes": [
                  ["id", "posX", "posY", "posZ"],
                  ["n1", 0, 0, 0]
                ]
              },
              "wheeldata_front": {
                "slotType": "wheeldata_F",
                "pressureWheels": [
                  ["name","hubGroup","group","node1:","node2:","nodeS","nodeArm:","wheelDir"],
                  {"hasTire":true},
                  {"pressurePSI":32},
                  {"wheelOffset":0.015},
                  {"radius":0.34},
                  {"tireWidth":0.23},
                  {"hubRadius":0.10},
                  {"hubWidth":0.18},
                  ["FL", "wheel_FL", "tire_FL", "n1", "n1", 0, "n1", -1],
                  ["FR", "wheel_FR", "tire_FR", "n1", "n1", 0, "n1", 1]
                ]
              },
              "wheeldata_rear": {
                "slotType": "wheeldata_R",
                "pressureWheels": [
                  ["name","hubGroup","group","node1:","node2:","nodeS","nodeArm:","wheelDir"],
                  {"hasTire":true},
                  {"pressurePSI":31},
                  {"wheelOffset":0.010},
                  {"radius":0.35},
                  {"tireWidth":0.24},
                  ["RL", "wheel_RL", "tire_RL", "n1", "n1", 0, "n1", -1],
                  ["RR", "wheel_RR", "tire_RR", "n1", "n1", 0, "n1", 1]
                ]
              }
            }
            """);

        var definition = JBeamAssembler.Assemble(vehicleFolder);
        Assert.Equal(VehicleSetupApplyMode.Live, definition.Setup.Variables["pressure_F"].ApplyMode);
        Assert.Equal(VehicleSetupApplyMode.Reload, definition.Setup.Variables["wheelOffset_F"].ApplyMode);
        Assert.Equal(VehicleSetupApplyMode.Reload, definition.Setup.Variables["tireWidth_F"].ApplyMode);

        var loadedVehicle = CreateManualWheelSetupLoadedVehicle(definition);
        var applyResult = VehicleSetupRuntime.Apply(
            loadedVehicle,
            new Dictionary<string, float>
            {
                ["pressure_F"] = 28f,
                ["wheelOffset_F"] = 0.02f,
                ["tireWidth_F"] = 0.25f,
            });

        Assert.Contains("pressure_F", applyResult.LiveAppliedKeys);
        Assert.Contains("wheelOffset_F", applyResult.ReloadRequiredKeys);
        Assert.Contains("tireWidth_F", applyResult.ReloadRequiredKeys);
        Assert.Equal(28f * 6.894757f, Assert.IsType<WheelSettings>(loadedVehicle.WheelFL.Get<WheelSettings>()).TyreModel!.TyrePressure, 3);
        Assert.Equal(0.02f, definition.Setup.Variables["wheelOffset_F"].Value, 3);
        Assert.Equal(0.25f, definition.Setup.Variables["tireWidth_F"].Value, 3);
        Assert.Equal(0.02f, loadedVehicle.SetupOverrides.PressureWheelOverrides[VehicleSetupAxle.Front].WheelOffset.GetValueOrDefault(), 3);
        Assert.Equal(0.02f, loadedVehicle.LoadRequest.SetupOverrides.PressureWheelOverrides[VehicleSetupAxle.Front].WheelOffset.GetValueOrDefault(), 3);
        Assert.Equal(0.25f, loadedVehicle.LoadRequest.SetupOverrides.PressureWheelOverrides[VehicleSetupAxle.Front].TireWidth.GetValueOrDefault(), 3);
    }

    [Fact]
    public void Apply_BrakeDrivetrainEngineAndFuelChanges_ShouldUpdateLiveAndReloadState()
    {
        var config = PcConfigLoader.Load(Path.Combine(GetBasicCarFolder(), "basic_car.pc"));
        var definition = JBeamAssembler.Assemble(GetBasicCarFolder(), config);
        var result = VehiclePhysicsBuilder.Build(definition);
        var loadedVehicle = CreateLoadedVehicle(definition, result, config);

        var applyResult = VehicleSetupRuntime.Apply(
            loadedVehicle,
            new Dictionary<string, float>
            {
                ["brakebias"] = 0.60f,
                ["brakestrength"] = 0.90f,
                ["finaldrive_R"] = 3.70f,
                ["lsdpreload_R"] = 180f,
                ["lsdlockcoef_R"] = 0.22f,
                ["lsdlockcoefrev_R"] = 0.41f,
                ["gear_1"] = 2.95f,
                ["revLimiterRPM"] = 5600f,
                ["revLimiterCutTime"] = 0.18f,
                ["fuel"] = 12f,
            });

        Assert.Contains("brakebias", applyResult.LiveAppliedKeys);
        Assert.Contains("brakestrength", applyResult.LiveAppliedKeys);
        Assert.Contains("finaldrive_R", applyResult.LiveAppliedKeys);
        Assert.Contains("lsdpreload_R", applyResult.LiveAppliedKeys);
        Assert.Contains("lsdlockcoef_R", applyResult.LiveAppliedKeys);
        Assert.Contains("lsdlockcoefrev_R", applyResult.LiveAppliedKeys);
        Assert.Contains("gear_1", applyResult.LiveAppliedKeys);
        Assert.Contains("revLimiterRPM", applyResult.LiveAppliedKeys);
        Assert.Contains("revLimiterCutTime", applyResult.LiveAppliedKeys);
        Assert.Contains("fuel", applyResult.ReloadRequiredKeys);

        Assert.Equal(0.60f, loadedVehicle.CarComponent.BrakeBiasFront, 3);
        Assert.Equal(0.90f, loadedVehicle.CarComponent.BrakeForceMultiplier, 3);
        Assert.Equal(1134f, loadedVehicle.CarComponent.FrontWheelBrakeTorque, 3);
        Assert.Equal(756f, loadedVehicle.CarComponent.RearWheelBrakeTorque, 3);
        Assert.Equal(3.70f, loadedVehicle.CarComponent.FinalDrive, 3);
        Assert.Equal(2.95f, loadedVehicle.CarComponent.GearRatios[1], 3);
        Assert.Equal(180f, Assert.IsType<VehicleDynamicsSystem>(loadedVehicle.CarComponent.Dynamics).RearDiff.PreloadTorque, 3);
        Assert.Equal(0.22f, loadedVehicle.CarComponent.Dynamics!.RearDiff.LockingCoefficient, 3);
        Assert.Equal(0.41f, loadedVehicle.CarComponent.Dynamics.RearDiff.CoastLockingCoefficient, 3);
        Assert.Equal(5600f, loadedVehicle.CarComponent.RevLimiterRpm, 3);
        Assert.Equal(0.18f, loadedVehicle.CarComponent.RevLimiterCutTime, 3);
        Assert.Equal(12f, loadedVehicle.SetupOverrides.VariableOverrides["fuel"], 3);
        Assert.Equal(12f, loadedVehicle.CarComponent.StartingFuelLiters, 3);
    }

    private static LoadedVehicle CreateLoadedVehicle(VehicleDefinition definition, VehicleBuilderResult result, PcConfig config)
    {
        var dynamics = new VehicleDynamicsSystem();
        foreach (var (wheel, index) in new[] { (result.WheelFL, 0), (result.WheelFR, 1), (result.WheelRL, 2), (result.WheelRR, 3) })
        {
            var wheelSettings = Assert.IsType<WheelSettings>(wheel.Get<WheelSettings>());
            var tyreModel = new TyreModel(0.305f) { Width = 0.205f, TyrePressure = 220f };
            wheelSettings.TyreModel = tyreModel;
            wheelSettings.DynamicsIndex = index;
            dynamics.TyreModels[index] = tyreModel;
            dynamics.StaticNormalLoads[index] = wheelSettings.StaticNormalLoad;
        }

        var car = new RallyCarComponent
        {
            Dynamics = dynamics,
            FrontStaticCamberRadians = 0f,
            RearStaticCamberRadians = 0f,
        };

        return new LoadedVehicle(
            definition,
            result.RootEntity,
            car,
            result.ChassisEntity,
            result.WheelFL,
            result.WheelFR,
            result.WheelRL,
            result.WheelRR,
            new VehicleLoadDiagnostics(definition.FolderPath, null, 1200f),
            config,
            new VehicleSetupOverrides(),
            new LoadedVehicleLoadRequest(definition.FolderPath, null, [definition.FolderPath], [definition.FolderPath], [definition.FolderPath], null, null));
    }

    private static LoadedVehicle CreateManualWheelSetupLoadedVehicle(VehicleDefinition definition)
    {
        var wheelFl = CreateWheelEntity("wheel_FL", isFront: true, dynamicsIndex: 0);
        var wheelFr = CreateWheelEntity("wheel_FR", isFront: true, dynamicsIndex: 1);
        var wheelRl = CreateWheelEntity("wheel_RL", isFront: false, dynamicsIndex: 2);
        var wheelRr = CreateWheelEntity("wheel_RR", isFront: false, dynamicsIndex: 3);

        var dynamics = new VehicleDynamicsSystem();
        foreach (var wheel in new[] { wheelFl, wheelFr, wheelRl, wheelRr })
        {
            var settings = Assert.IsType<WheelSettings>(wheel.Get<WheelSettings>());
            dynamics.TyreModels[settings.DynamicsIndex] = settings.TyreModel;
            dynamics.StaticNormalLoads[settings.DynamicsIndex] = settings.StaticNormalLoad;
        }

        return new LoadedVehicle(
            definition,
            new Entity("root"),
            new RallyCarComponent { Dynamics = dynamics },
            new Entity("chassis"),
            wheelFl,
            wheelFr,
            wheelRl,
            wheelRr,
            new VehicleLoadDiagnostics(definition.FolderPath, null, 1200f),
            new PcConfig(),
            new VehicleSetupOverrides(),
            new LoadedVehicleLoadRequest(definition.FolderPath, null, [definition.FolderPath], [definition.FolderPath], [definition.FolderPath], null, null));
    }

    private static Entity CreateWheelEntity(string name, bool isFront, int dynamicsIndex)
    {
        var entity = new Entity(name);
        entity.Add(new WheelSettings
        {
            IsFrontAxle = isFront,
            DynamicsIndex = dynamicsIndex,
            StaticNormalLoad = 3000f,
            TyreModel = new TyreModel(0.32f) { Width = 0.22f, TyrePressure = 220f },
        });
        return entity;
    }

    private static string GetBasicCarFolder()
    {
        return Path.Combine(GetRepoRoot(), "LibreRally", "Resources", "BeamNG Vehicles", "basic_car");
    }

    private static string GetSunburstFolder()
    {
        return Path.Combine(GetRepoRoot(), "LibreRally", "Resources", "BeamNG Vehicles", "sunburst2");
    }

    private static string GetRepoRoot()
    {
        DirectoryInfo? directory = new(AppContext.BaseDirectory);
        while (directory != null)
        {
            var candidate = Path.Combine(directory.FullName, "LibreRally.sln");
            if (File.Exists(candidate))
            {
                return directory.FullName;
            }

            directory = directory.Parent;
        }

        throw new DirectoryNotFoundException("Could not locate the repository root.");
    }

    private sealed class TempWorkspace : IDisposable
    {
        public TempWorkspace()
        {
            RootPath = Path.Combine(Path.GetTempPath(), "LibreRally.Tests", Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(RootPath);
        }

        public string RootPath { get; }

        public void Dispose()
        {
            if (Directory.Exists(RootPath))
            {
                Directory.Delete(RootPath, recursive: true);
            }
        }
    }
}
