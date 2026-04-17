using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using LibreRally.Vehicle.Content;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using LibreRally.Vehicle.Rendering;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Rendering;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;

namespace LibreRally.Vehicle;

/// <summary>
/// High-level vehicle loader.
/// </summary>
public class VehicleLoader
{
    private static readonly Logger Log = GlobalLogger.GetLogger("VehicleLoader");
    private sealed record ModelSource(string SourcePath, List<ColladaMesh> Meshes, Dictionary<string, string> TextureMap);
    private sealed record SupplementalModelSource(ModelSource Source, IReadOnlyList<string> RequestedMeshNames);
    private readonly record struct TireSpec(float Radius, float Width);
    private readonly GraphicsDevice _graphicsDevice;

    public VehicleLoader(Game game)
    {
        _graphicsDevice = game.GraphicsDevice;
    }

    /// <summary>
    /// Loads the vehicle at <paramref name="vehicleFolderPath"/>, builds physics and mesh entities,
    /// and returns the assembled <see cref="LoadedVehicle"/>.
    /// </summary>
    /// <param name="vehicleFolderPath">Absolute path to the vehicle folder containing .jbeam files.</param>
    /// <param name="configFileName">
    /// Optional .pc config file name (e.g. "rally_pro_asphalt.pc") or base name without extension.
    /// If null, auto-detects: prefers rally_pro_asphalt.pc, then first .pc file found.
    /// </param>
    public LoadedVehicle Load(string vehicleFolderPath, string? configFileName = null)
    {
        var vehiclesRoot = Path.GetDirectoryName(vehicleFolderPath) ?? vehicleFolderPath;
        return LoadInternal(
            vehicleFolderPath,
            configFileName,
            [vehicleFolderPath],
            [vehicleFolderPath],
            [vehiclesRoot],
            null,
            null);
    }

    public LoadedVehicle Load(BeamNgResolvedVehicle vehicleSource, string? configFileName = null)
    {
        return LoadInternal(
            vehicleSource.VehicleFolderPath,
            configFileName,
            vehicleSource.JBeamSearchFolders,
            vehicleSource.JBeamSearchFolders,
            vehicleSource.VehiclesRootDirectories,
            vehicleSource.ResolveVehicleAssetPath,
            vehicleSource);
    }

    private LoadedVehicle LoadInternal(
        string vehicleFolderPath,
        string? configFileName,
        IReadOnlyList<string> jbeamSearchFolders,
        IReadOnlyList<string> materialSearchFolders,
        IReadOnlyList<string> vehiclesRootDirectories,
        Func<string, string?>? virtualAssetResolver,
        BeamNgResolvedVehicle? resolvedVehicle)
    {
        if (!Directory.Exists(vehicleFolderPath))
        {
	        throw new DirectoryNotFoundException($"Vehicle folder not found: '{vehicleFolderPath}'");
        }

        // 0. Load .pc config (parts selection + physics variables)
        PcConfig? pcConfig = null;
        var pcPath = PcConfigLoader.FindBestConfig(vehicleFolderPath, configFileName);
        if (pcPath != null)
        {
            try
            {
                pcConfig = PcConfigLoader.Load(pcPath);
                Log.Info($"[VehicleLoader] PC config: {Path.GetFileName(pcPath)} | " +
                         $"parts={pcConfig.Parts.Count} vars={pcConfig.Vars.Count}");
            }
            catch (Exception ex)
            {
                Log.Warning($"[VehicleLoader] Failed to parse .pc config '{pcPath}': {ex.Message}");
            }
        }
        else
        {
            Log.Warning("[VehicleLoader] No .pc config file found — using jbeam defaults.");
        }

        // 1. Parse + assemble jbeam (with pc config for parts selection + variable substitution)
        var definition = JBeamAssembler.Assemble(jbeamSearchFolders, vehicleFolderPath, pcConfig);

        // 2. Build physics entity hierarchy
        var result = VehiclePhysicsBuilder.Build(definition);
        var rootEntity = result.RootEntity;

        // 3. Attach visual meshes (best-effort — falls back to a visible box)
        TryAttachMeshes(
            vehicleFolderPath,
            result,
            definition,
            pcConfig,
            materialSearchFolders,
            vehiclesRootDirectories,
            virtualAssetResolver,
            resolvedVehicle);

        // 4. Wire up the rally car driving component
        float V(string name, float fallback) =>
            definition.Vars.TryGetValue(name, out var v) && v > 0 ? v : fallback;
        float GetVarWithFallbacks(float fallback, params string[] names)
        {
            foreach (var name in names)
            {
                if (definition.Vars.TryGetValue(name, out var v) && v > 0)
                {
                    return v;
                }
            }

            return fallback;
        }
        float GetNumericVarOrFallback(float fallback, params string[] names)
        {
            foreach (var name in names)
            {
                if (definition.Vars.TryGetValue(name, out var v) && float.IsFinite(v))
                {
                    return v;
                }
            }

            return fallback;
        }
        float ResolveCamberRadians(string axleSuffix)
        {
            var camberPrecompression = GetNumericVarOrFallback(1f, $"camber_{axleSuffix}_asphalt", $"camber_{axleSuffix}");
            return RallyCarComponent.ConvertCamberPrecompressionToRadians(camberPrecompression);
        }

        var powertrain = VehiclePowertrainResolver.Resolve(definition);
        var absEnabled = IsAbsEnabled(definition.BrakeControl);
        var absSlipRatioTarget = ResolveAbsSlipRatioTarget(definition.BrakeControl);
        var tractionControlEnabled = IsTractionControlEnabled(definition.TractionControl);
        var tractionControlSlipRatioTarget = ResolveTractionControlSlipRatioTarget(definition.TractionControl);
        var tractionControlSlipRatioWindow = ResolveTractionControlSlipRatioWindow(definition.TractionControl);
        var frontTyreSpec = VehicleTyreSpecResolver.Resolve(definition, front: true);
        var rearTyreSpec = VehicleTyreSpecResolver.Resolve(definition, front: false);
        var wheelRadius = VehicleTyreSpecResolver.ResolveDrivenWheelRadius(powertrain, frontTyreSpec, rearTyreSpec);

        // ── Vehicle dynamics system ──────────────────────────────────────────
        // Estimate vehicle geometry from JBeam nodes (wheelbase, track width, CG height)
        var wheelbase = EstimateWheelbase(result);
        var trackWidth = EstimateTrackWidth(result);
        var vehicleMass = EstimateVehicleMass(definition);
        var cgHeight = V("cg_height", 0.45f);
        var quarterLoad = vehicleMass * 9.81f / 4f;
        var diagnostics = new VehicleLoadDiagnostics(vehicleFolderPath, pcPath, vehicleMass);

        var dynamics = new VehicleDynamicsSystem
        {
            VehicleMass = vehicleMass,
            CgHeight = cgHeight,
            Wheelbase = wheelbase,
            TrackWidth = trackWidth,
            FrontAntiRollStiffness = GetVarWithFallbacks(8000f, "antiroll_front", "arb_spring_F"),
            RearAntiRollStiffness = GetVarWithFallbacks(5000f, "antiroll_rear", "arb_spring_R"),
            BodyRoll = new ChassisBodyRollSystem
            {
                FrontRollStiffness = GetVarWithFallbacks(5000f, "roll_stiffness_front", "spring_roll_F"),
                RearRollStiffness = GetVarWithFallbacks(4000f, "roll_stiffness_rear", "spring_roll_R"),
                RollDampingCoefficient = GetVarWithFallbacks(800f, "roll_damping"),
            },
            FrontDiff = powertrain.FrontDiff,
            RearDiff = powertrain.RearDiff,
            CenterDiff = powertrain.CenterDiff,
            DriveFrontAxle = powertrain.DriveFrontAxle,
            DriveRearAxle = powertrain.DriveRearAxle,
        };

        // Assign tyre models and static loads to each wheel
        Entity[] wheelEntities = { result.WheelFL, result.WheelFR, result.WheelRL, result.WheelRR };
        var wheelEntityMap = new Dictionary<string, Entity>(StringComparer.OrdinalIgnoreCase)
        {
            ["wheel_FL"] = result.WheelFL,
            ["wheel_FR"] = result.WheelFR,
            ["wheel_RL"] = result.WheelRL,
            ["wheel_RR"] = result.WheelRR,
        };
        VehicleTyreSpec[] wheelTyreSpecs =
        [
            frontTyreSpec,
            frontTyreSpec,
            rearTyreSpec,
            rearTyreSpec,
        ];
        for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
        {
            var tyreModel = CreateTyreModel(wheelTyreSpecs[i]);
            dynamics.TyreModels[i] = tyreModel;
            dynamics.StaticNormalLoads[i] = quarterLoad;

            var ws = wheelEntities[i].Get<WheelSettings>();
            if (ws != null)
            {
                ws.TyreModel = tyreModel;
                ws.StaticNormalLoad = quarterLoad;
                ws.DynamicsIndex = i;
            }
        }

        var car = new RallyCarComponent
        {
            CarBody = result.ChassisEntity,
            WheelRadius = wheelRadius,
            GearRatios = powertrain.GearRatios,
            FinalDrive = powertrain.FinalDrive,
            TorqueCurveRpm = powertrain.TorqueCurveRpm,
            TorqueCurveNm = powertrain.TorqueCurveNm,
            MaxRpm = powertrain.MaxRpm,
            IdleRpm = powertrain.IdleRpm,
            EngineInertia = powertrain.EngineInertia,
            EngineFriction = powertrain.EngineFriction,
            EngineDynamicFriction = powertrain.EngineDynamicFriction,
            EngineBrakeTorque = powertrain.EngineBrakeTorque,
            AutoClutchLaunchRpm = powertrain.AutoClutchLaunchRpm,
            ShiftUpRpm = powertrain.ShiftUpRpm,
            ShiftDownRpm = powertrain.ShiftDownRpm,
            TractionControlEnabled = tractionControlEnabled,
            TractionControlSlipRatioTarget = tractionControlSlipRatioTarget,
            TractionControlSlipRatioWindow = tractionControlSlipRatioWindow,
            TractionLossSlipThreshold = tractionControlEnabled ? tractionControlSlipRatioTarget : 0.18f,
            AbsEnabled = absEnabled,
            AbsSlipRatioTarget = absSlipRatioTarget,
            FrontStaticCamberRadians = ResolveCamberRadians("F"),
            RearStaticCamberRadians = ResolveCamberRadians("R"),
            Dynamics = dynamics,
            // Engine thermal / oil / fuel / turbo
            FuelCapacityLiters = powertrain.FuelCapacityLiters,
            StartingFuelLiters = powertrain.StartingFuelLiters,
            OilVolumeLiters = powertrain.OilVolumeLiters,
            EngineBlockTempDamageThreshold = powertrain.EngineBlockTempDamageThreshold,
            AirRegulatorTemperature = powertrain.AirRegulatorTemperature,
            EngineBlockAirCoolingEfficiency = powertrain.EngineBlockAirCoolingEfficiency,
            BurnEfficiencyThrottle = powertrain.BurnEfficiencyThrottle,
            BurnEfficiencyValues = powertrain.BurnEfficiencyValues,
            HasTurbo = powertrain.HasTurbo,
            TurboMaxBoostPsi = powertrain.TurboMaxBoostPsi,
        };
        car.Wheels.AddRange(wheelEntities);
        car.SteerWheels.AddRange(new[] { result.WheelFL, result.WheelFR });
        car.BreakWheels.AddRange(wheelEntities);
        foreach (var wheelKey in powertrain.DrivenWheelKeys.Where(wheelEntityMap.ContainsKey))
        {
            if (wheelEntityMap.TryGetValue(wheelKey, out var wheelEntity))
            {
                car.DriveWheels.Add(wheelEntity);
            }
        }
        rootEntity.Add(car);

        var reverseGear = powertrain.GearRatios[0];
        var drivenLayout = powertrain.DriveFrontAxle && powertrain.DriveRearAxle
            ? "AWD"
            : powertrain.DriveFrontAxle
                ? "FWD"
                : "RWD";
        Log.Info($"[VehicleLoader] Gears: R={reverseGear:F2} " +
                 string.Join(" ", powertrain.GearRatios.Skip(1).Select((g, i) => $"{i + 1}={g:F2}")) +
                  $" | FD={powertrain.FinalDrive:F2} | MaxRPM={powertrain.MaxRpm:F0} | Layout={drivenLayout} | Driven={string.Join(",", powertrain.DrivenWheelKeys)}" +
                  $" | ABS={(absEnabled ? $"Y@{absSlipRatioTarget:F2}" : "N")}" +
                  $" | TCS={(tractionControlEnabled ? $"Y@{tractionControlSlipRatioTarget:F2}±{tractionControlSlipRatioWindow:F2}" : "N")}");
        Log.Info($"[VehicleLoader] Tyres: F r={frontTyreSpec.Radius:F3}m w={frontTyreSpec.Width:F3}m p={frontTyreSpec.PressureKpa:F0}kPa " +
                 $"| R r={rearTyreSpec.Radius:F3}m w={rearTyreSpec.Width:F3}m p={rearTyreSpec.PressureKpa:F0}kPa");
        Log.Info($"[VehicleLoader] Dynamics: mass={vehicleMass:F0}kg wb={wheelbase:F2}m tw={trackWidth:F2}m cg={cgHeight:F2}m");
        Log.Info($"[VehicleLoader] Engine: fuel={powertrain.FuelCapacityLiters:F1}L start={powertrain.StartingFuelLiters:F1}L " +
                 $"oil={powertrain.OilVolumeLiters:F1}L thermostat={powertrain.AirRegulatorTemperature:F0}°C " +
                 $"turbo={powertrain.HasTurbo} maxBoost={powertrain.TurboMaxBoostPsi:F1}psi");
        Log.Info($"[VehicleLoader] Active vehicle='{definition.VehicleName}' folder='{vehicleFolderPath}' " +
                 $"config='{(pcPath != null ? Path.GetFileName(pcPath) : "<jbeam defaults>")}' nodes={definition.Nodes.Count} mass={vehicleMass:F0}kg");
        if (pcPath != null)
        {
	        Log.Info($"[VehicleLoader] Active config path: {pcPath}");
        }

        return new LoadedVehicle(definition, rootEntity, car, result.ChassisEntity, result.WheelFL, result.WheelFR, result.WheelRL, result.WheelRR, diagnostics);
    }

    internal static bool IsAbsEnabled(JBeamBrakeControlDefinition? brakeControl)
    {
        return brakeControl?.EnableAbs == true || brakeControl?.HasLegacyAbsController == true;
    }

    internal static float ResolveAbsSlipRatioTarget(JBeamBrakeControlDefinition? brakeControl)
    {
        var target = brakeControl?.AbsSlipRatioTarget ?? 0.15f;
        return Math.Clamp(target, 0.05f, 0.35f);
    }

    internal static bool IsTractionControlEnabled(JBeamTractionControlDefinition? tractionControl)
    {
        return tractionControl?.EnableTractionControl == true;
    }

    internal static float ResolveTractionControlSlipRatioTarget(JBeamTractionControlDefinition? tractionControl)
    {
        var target = tractionControl?.SlipThreshold ?? 0.15f;
        return Math.Clamp(target, 0.05f, 0.35f);
    }

    internal static float ResolveTractionControlSlipRatioWindow(JBeamTractionControlDefinition? tractionControl)
    {
        var window = tractionControl?.SlipRangeThreshold ?? 0.10f;
        return Math.Clamp(window, 0.02f, 0.35f);
    }

    internal static TyreModel CreateTyreModel(in VehicleTyreSpec spec)
    {
        bool hasBeamNgLoadCurve =
            spec.BeamNgNoLoadFrictionCoefficient.HasValue &&
            spec.BeamNgFullLoadFrictionCoefficient.HasValue &&
            spec.BeamNgLoadSensitivitySlope.HasValue;

        var tyreModel = new TyreModel(spec.Radius)
        {
            Width = spec.Width,
            TyrePressure = spec.PressureKpa,
            PeakFrictionCoefficient = spec.PeakFrictionCoefficient,
            RollingResistanceCoefficient = spec.RollingResistanceCoefficient,
            LoadSensitivity = hasBeamNgLoadCurve ? 0f : 0.15f,
        };

        if (hasBeamNgLoadCurve)
        {
            tyreModel.BeamNgNoLoadFrictionCoefficient = spec.BeamNgNoLoadFrictionCoefficient.GetValueOrDefault();
            tyreModel.BeamNgFullLoadFrictionCoefficient = spec.BeamNgFullLoadFrictionCoefficient.GetValueOrDefault();
            tyreModel.BeamNgLoadSensitivitySlope = spec.BeamNgLoadSensitivitySlope.GetValueOrDefault();
        }

        return tyreModel;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Mesh attachment
    // ──────────────────────────────────────────────────────────────────────────

    private void TryAttachMeshes(
        string folder,
        VehicleBuilderResult result,
        VehicleDefinition definition,
        PcConfig? pcConfig,
        IReadOnlyList<string> materialSearchFolders,
        IReadOnlyList<string> vehiclesRootDirectories,
        Func<string, string?>? virtualAssetResolver,
        BeamNgResolvedVehicle? resolvedVehicle)
    {
        try
        {
            var baseModelSources = LoadModelSources(folder);
            var modelSources = new List<ModelSource>(baseModelSources);
            var supplementalModelSources = AddSupplementalColladaSources(modelSources, definition, resolvedVehicle);
            if (modelSources.Count == 0)
            {
                AttachFallbackChassis(result.ChassisEntity, definition);
                AttachFallbackTires(result, pcConfig, null);
                return;
            }

            var jsonMaterials = BeamNGMaterialLoader.LoadMaterialTextures(
                materialSearchFolders.Concat(GetSupplementalMaterialSearchFolders(supplementalModelSources.Select(source => source.Source.SourcePath))),
                vehiclesRootDirectories,
                virtualAssetResolver);
            var wheelEntities = new Dictionary<string, Entity>(StringComparer.OrdinalIgnoreCase)
            {
                ["wheel_FL"] = result.WheelFL,
                ["wheel_FR"] = result.WheelFR,
                ["wheel_RL"] = result.WheelRL,
                ["wheel_RR"] = result.WheelRR,
            };

            var wheelMeshNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var tireLikeAttachments = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase)
            {
                ["wheel_FL"] = 0,
                ["wheel_FR"] = 0,
                ["wheel_RL"] = 0,
                ["wheel_RR"] = 0,
            };
            var missingWheelMeshes = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var wheelVisualCount = AttachWheelFlexBodyMeshes(
                definition,
                modelSources,
                wheelEntities,
                wheelMeshNames,
                tireLikeAttachments,
                jsonMaterials,
                folder,
                missingWheelMeshes);

            if (missingWheelMeshes.Count > 0)
            {
                Log.Warning($"Wheel visual meshes missing from local model files: {string.Join(", ", missingWheelMeshes.OrderBy(x => x))}");
            }

            AttachFallbackTires(result, pcConfig, tireLikeAttachments);

            var chassisFlexBodyMeshNames = GetChassisVisualFlexBodyMeshNames(definition);
            var chassisMeshNameSet = new HashSet<string>(chassisFlexBodyMeshNames, StringComparer.OrdinalIgnoreCase);
            var chassisVisualCount = 0;
            var mainSource = SelectMainSourceForChassis(baseModelSources, wheelMeshNames);
            if (mainSource != null)
            {
                var chassisMeshes = SelectChassisMeshes(
                    mainSource.Meshes,
                    mainSource.SourcePath,
                    chassisFlexBodyMeshNames,
                    wheelMeshNames);

                if (TryAttachChassisMeshes(result.ChassisEntity, definition.VehicleName, folder, jsonMaterials, mainSource, chassisMeshes))
                {
                    chassisVisualCount++;
                }
            }

            foreach (var supplementalModelSource in supplementalModelSources)
            {
                var requestedChassisMeshNames = supplementalModelSource.RequestedMeshNames
                    .Where(chassisMeshNameSet.Contains)
                    .ToArray();
                if (requestedChassisMeshNames.Length == 0)
                {
                    continue;
                }

                var chassisMeshes = SelectSupplementalChassisMeshes(
                    supplementalModelSource.Source.Meshes,
                    requestedChassisMeshNames,
                    wheelMeshNames);
                if (TryAttachChassisMeshes(
                        result.ChassisEntity,
                        definition.VehicleName,
                        folder,
                        jsonMaterials,
                        supplementalModelSource.Source,
                        chassisMeshes))
                {
                    chassisVisualCount++;
                }
            }

            if (chassisVisualCount > 0)
            {
                Log.Info($"Wheel visuals: attached {wheelVisualCount} flexbody instances.");
                return;
            }

            Log.Warning("No chassis-like meshes found in model sources.");
            AttachFallbackChassis(result.ChassisEntity, definition);
            return;
        }
        catch (Exception ex)
        {
            Log.Error($"Could not load mesh: {ex.Message}");
        }

        AttachFallbackChassis(result.ChassisEntity, definition);
        AttachFallbackTires(result, pcConfig, null);
    }

    private static ModelSource? SelectMainSourceForChassis(IEnumerable<ModelSource> modelSources, HashSet<string> wheelMeshNames)
    {
        return modelSources
            .Select(source => new
            {
                Source = source,
                ChassisMeshCount = source.Meshes.Count(mesh =>
                    !wheelMeshNames.Any(wheelMeshName => MatchesColladaMesh(mesh, wheelMeshName))),
            })
            .Where(entry => entry.ChassisMeshCount > 0)
            .OrderByDescending(entry => entry.ChassisMeshCount)
            .ThenBy(entry => entry.Source.SourcePath, StringComparer.OrdinalIgnoreCase)
            .Select(entry => entry.Source)
            .FirstOrDefault();
    }

    internal static List<ColladaMesh> SelectChassisMeshes(
        IReadOnlyList<ColladaMesh> sourceMeshes,
        string sourcePath,
        IEnumerable<string> activeChassisMeshNames,
        ISet<string> excludedWheelMeshNames)
    {
        var nonWheelMeshes = sourceMeshes
            .Where(mesh => !excludedWheelMeshNames.Any(wheelMeshName => MatchesColladaMesh(mesh, wheelMeshName)))
            .ToList();

        if (!sourcePath.EndsWith(".dts", StringComparison.OrdinalIgnoreCase))
        {
            return nonWheelMeshes;
        }

        var activeMeshNames = activeChassisMeshNames
            .Where(name => !string.IsNullOrWhiteSpace(name))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToArray();
        if (activeMeshNames.Length == 0)
        {
            return nonWheelMeshes;
        }

        var filteredMeshes = nonWheelMeshes
            .Where(mesh => activeMeshNames.Any(activeMeshName => MatchesColladaMesh(mesh, activeMeshName)))
            .ToList();

        return filteredMeshes.Count > 0 ? filteredMeshes : nonWheelMeshes;
    }

    internal static List<ColladaMesh> SelectSupplementalChassisMeshes(
        IReadOnlyList<ColladaMesh> sourceMeshes,
        IEnumerable<string> requestedMeshNames,
        ISet<string> excludedWheelMeshNames)
    {
        var requestedNames = requestedMeshNames
            .Where(name => !string.IsNullOrWhiteSpace(name))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToArray();
        if (requestedNames.Length == 0)
        {
            return new List<ColladaMesh>();
        }

        return sourceMeshes
            .Where(mesh =>
                !excludedWheelMeshNames.Any(wheelMeshName => MatchesColladaMesh(mesh, wheelMeshName)) &&
                requestedNames.Any(requestedMeshName => MatchesColladaMesh(mesh, requestedMeshName)))
            .ToList();
    }

    private List<ModelSource> LoadModelSources(string folder)
    {
        var modelFiles = Directory.EnumerateFiles(folder, "*.dae", SearchOption.AllDirectories)
            .Concat(Directory.EnumerateFiles(folder, "*.dts", SearchOption.AllDirectories))
            .OrderBy(path => Path.GetDirectoryName(path)?.Equals(folder, StringComparison.OrdinalIgnoreCase) == true ? 0 : 1)
            .ThenBy(path => path, StringComparer.OrdinalIgnoreCase)
            .ToList();

        return LoadModelSourcesFromFiles(modelFiles);
    }

    private List<ModelSource> LoadModelSourcesFromFiles(IEnumerable<string> modelFiles)
    {
        var result = new List<ModelSource>();
        foreach (var modelFile in modelFiles
                     .Where(path => !string.IsNullOrWhiteSpace(path) && File.Exists(path))
                     .Distinct(StringComparer.OrdinalIgnoreCase))
        {
            try
            {
                Dictionary<string, string> textureMap;
                List<ColladaMesh> meshes;
                if (modelFile.EndsWith(".dae", StringComparison.OrdinalIgnoreCase))
                {
                    textureMap = ColladaLoader.LoadTextureMap(modelFile);
                    meshes = ColladaLoader.Load(modelFile);
                    Log.Info($"DAE: {Path.GetFileName(modelFile)} | {meshes.Count} sub-meshes | Collada textures: {textureMap.Count}");
                }
                else
                {
                    textureMap = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                    meshes = DtsLoader.Load(modelFile);
                    Log.Info($"DTS: {Path.GetFileName(modelFile)} | {meshes.Count} sub-meshes");
                }

                result.Add(new ModelSource(modelFile, meshes, textureMap));
            }
            catch (Exception ex)
            {
                Log.Warning($"Could not load model '{Path.GetFileName(modelFile)}': {ex.Message}");
            }
        }

        return result;
    }

    private IReadOnlyList<SupplementalModelSource> AddSupplementalColladaSources(
        List<ModelSource> modelSources,
        VehicleDefinition definition,
        BeamNgResolvedVehicle? resolvedVehicle)
    {
        if (resolvedVehicle == null)
        {
            return Array.Empty<SupplementalModelSource>();
        }

        var missingMeshNames = definition.FlexBodies
            .Select(flexBody => flexBody.MeshName)
            .Where(meshName => !string.IsNullOrWhiteSpace(meshName))
            .Where(meshName => !TryFindGeometry(modelSources, meshName, out _, out _))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToArray();
        if (missingMeshNames.Length == 0)
        {
            return Array.Empty<SupplementalModelSource>();
        }

        var supplementalFiles = resolvedVehicle.ResolveColladaFilesForMeshes(missingMeshNames);
        if (supplementalFiles.Count == 0)
        {
            return Array.Empty<SupplementalModelSource>();
        }

        var supplementalSources = new List<SupplementalModelSource>();
        foreach (var source in LoadModelSourcesFromFiles(supplementalFiles))
        {
            modelSources.Add(source);
            var requestedMeshNames = GetMatchedMeshNames(source.Meshes, missingMeshNames);
            if (requestedMeshNames.Length == 0)
            {
                continue;
            }

            supplementalSources.Add(new SupplementalModelSource(source, requestedMeshNames));
        }

        if (supplementalSources.Count > 0)
        {
            Log.Info("Supplemental model sources: " +
                     string.Join("; ", supplementalSources.Select(source =>
                         $"{Path.GetFileName(source.Source.SourcePath)} [{string.Join(", ", source.RequestedMeshNames.OrderBy(name => name, StringComparer.OrdinalIgnoreCase))}]")));
        }

        return supplementalSources;
    }

    private static IEnumerable<string> GetSupplementalMaterialSearchFolders(IEnumerable<string> colladaFiles)
    {
        var folders = new List<string>();
        var seenFolders = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        foreach (var colladaFile in colladaFiles)
        {
            var directory = Path.GetDirectoryName(colladaFile);
            while (!string.IsNullOrEmpty(directory) && Directory.Exists(directory))
            {
                if (seenFolders.Add(directory))
                {
                    folders.Add(directory);
                }

                var parent = Directory.GetParent(directory)?.FullName;
                if (string.IsNullOrEmpty(parent) ||
                    string.Equals(Path.GetFileName(parent), "vehicles", StringComparison.OrdinalIgnoreCase))
                {
                    break;
                }

                directory = parent;
            }
        }

        return folders;
    }

    private bool TryAttachChassisMeshes(
        Entity chassisEntity,
        string vehicleName,
        string vehicleFolder,
        Dictionary<string, string> jsonMaterials,
        ModelSource source,
        List<ColladaMesh> chassisMeshes)
    {
        var sourceName = Path.GetFileNameWithoutExtension(source.SourcePath);
        var meshEntity = BuildMeshEntity(
            chassisMeshes,
            string.IsNullOrWhiteSpace(sourceName) ? vehicleName : $"{vehicleName}_{sourceName}",
            Path.GetDirectoryName(source.SourcePath) ?? vehicleFolder,
            jsonMaterials,
            source.TextureMap);

        if (meshEntity == null)
        {
            return false;
        }

        // DAE vertices are in BeamNG world space (converted to Stride coords).
        // The chassis entity sits at its node centroid, so counter-offset the mesh
        // entity so its vertex positions map to the correct world locations.
        meshEntity.Transform.Position = -chassisEntity.Transform.Position;
        chassisEntity.AddChild(meshEntity);
        return true;
    }

    private int AttachWheelFlexBodyMeshes(
        VehicleDefinition definition,
        List<ModelSource> modelSources,
        Dictionary<string, Entity> wheelEntities,
        HashSet<string> wheelMeshNames,
        Dictionary<string, int> tireLikeAttachments,
        Dictionary<string, string> jsonMaterials,
        string vehicleFolder,
        HashSet<string> missingWheelMeshes)
    {
        var attached = 0;
        foreach (var flexBody in GetWheelVisualFlexBodies(definition))
        {
            if (!TryResolveWheelKey(flexBody, out string? wheelKey) ||
                wheelKey == null ||
                !wheelEntities.TryGetValue(wheelKey, out var wheelEntity))
            {
                continue;
            }

            if (!TryFindGeometry(modelSources, flexBody.MeshName, out var source, out var sourceMeshes))
            {
                missingWheelMeshes.Add(flexBody.MeshName);
                continue;
            }

            var meshEntity = BuildMeshEntity(
                sourceMeshes,
                $"{flexBody.MeshName}_{wheelKey}",
                Path.GetDirectoryName(source.SourcePath) ?? vehicleFolder,
                jsonMaterials,
                source.TextureMap);

            if (meshEntity == null)
            {
	            continue;
            }

            ApplyWheelFlexBodyTransform(meshEntity, wheelEntity, flexBody, sourceMeshes);
            wheelEntity.AddChild(meshEntity);
            wheelMeshNames.Add(flexBody.MeshName);
            if (IsTireLikeFlexBody(flexBody))
            {
	            tireLikeAttachments[wheelKey]++;
            }

            attached++;
        }

        return attached;
    }

    private static IEnumerable<AssembledFlexBody> GetWheelVisualFlexBodies(VehicleDefinition definition)
    {
        return definition.FlexBodies.Where(IsWheelVisualFlexBody);
    }

    private static IReadOnlyList<string> GetChassisVisualFlexBodyMeshNames(VehicleDefinition definition)
    {
        return definition.FlexBodies
            .Where(flexBody => !IsWheelVisualFlexBody(flexBody))
            .Select(flexBody => flexBody.MeshName)
            .Where(meshName => !string.IsNullOrWhiteSpace(meshName))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToArray();
    }

    private static bool IsWheelVisualFlexBody(AssembledFlexBody flexBody)
    {
        return TryResolveWheelKey(flexBody, out _) &&
               (ContainsWheelVisualToken(flexBody.MeshName) ||
                ContainsWheelVisualToken(flexBody.SourcePartName) ||
                ContainsWheelVisualToken(flexBody.SourceSlotType));
    }

    private static bool TryResolveWheelKey(AssembledFlexBody flexBody, out string? wheelKey)
    {
        foreach (var group in flexBody.NodeGroups)
        {
            if (group.Equals("wheel_FL", StringComparison.OrdinalIgnoreCase) ||
                group.Equals("wheelhub_FL", StringComparison.OrdinalIgnoreCase))
            {
                wheelKey = "wheel_FL";
                return true;
            }

            if (group.Equals("wheel_FR", StringComparison.OrdinalIgnoreCase) ||
                group.Equals("wheelhub_FR", StringComparison.OrdinalIgnoreCase))
            {
                wheelKey = "wheel_FR";
                return true;
            }

            if (group.Equals("wheel_RL", StringComparison.OrdinalIgnoreCase) ||
                group.Equals("wheelhub_RL", StringComparison.OrdinalIgnoreCase))
            {
                wheelKey = "wheel_RL";
                return true;
            }

            if (group.Equals("wheel_RR", StringComparison.OrdinalIgnoreCase) ||
                group.Equals("wheelhub_RR", StringComparison.OrdinalIgnoreCase))
            {
                wheelKey = "wheel_RR";
                return true;
            }
        }

        wheelKey = null;
        return false;
    }

    private static bool TryFindGeometry(
        IEnumerable<ModelSource> modelSources,
        string geometryName,
        out ModelSource source,
        out List<ColladaMesh> meshes)
    {
        foreach (var candidate in modelSources)
        {
            meshes = candidate.Meshes
                .Where(cm => MatchesColladaMesh(cm, geometryName))
                .ToList();
            if (meshes.Count > 0)
            {
                source = candidate;
                return true;
            }
        }

        source = null!;
        meshes = null!;
        return false;
    }

    private static string[] GetMatchedMeshNames(
        IReadOnlyList<ColladaMesh> sourceMeshes,
        IEnumerable<string> lookupNames)
    {
        return lookupNames
            .Where(lookupName => sourceMeshes.Any(mesh => MatchesColladaMesh(mesh, lookupName)))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToArray();
    }

    internal static bool MatchesColladaMesh(ColladaMesh mesh, string lookupName)
    {
        return MatchesColladaMeshName(mesh.GeometryName, lookupName) ||
               MatchesColladaMeshName(mesh.SceneNodeName, lookupName) ||
               MatchesColladaMeshName(mesh.Name, lookupName);
    }

    internal static bool MatchesColladaMeshName(string candidateName, string lookupName)
    {
        if (string.IsNullOrWhiteSpace(candidateName) || string.IsNullOrWhiteSpace(lookupName))
        {
            return false;
        }

        if (candidateName.Equals(lookupName, StringComparison.OrdinalIgnoreCase))
        {
            return true;
        }

        return NormalizeColladaMeshLookupName(candidateName)
            .Equals(NormalizeColladaMeshLookupName(lookupName), StringComparison.OrdinalIgnoreCase);
    }

    internal static string NormalizeColladaMeshLookupName(string value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return string.Empty;
        }

        var normalized = value.Trim().ToLowerInvariant();
        normalized = Regex.Replace(normalized, @"(?:[-_.]?mesh)$", "");
        normalized = Regex.Replace(normalized, @"(?:[-_.]\d{3})+$", "");
        normalized = Regex.Replace(normalized, @"shape$", "");
        normalized = Regex.Replace(normalized, @"[-.\s]+", "_");
        normalized = Regex.Replace(normalized, @"_+", "_");
        return normalized.Trim('_');
    }

    private static bool ContainsWheelVisualToken(string value)
    {
        if (string.IsNullOrEmpty(value))
        {
	        return false;
        }

        return value.Contains("wheel", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("tire", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("tyre", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("hubcap", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("rim", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("brake", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("caliper", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("disc", StringComparison.OrdinalIgnoreCase);
    }

    private static bool IsTireLikeFlexBody(AssembledFlexBody flexBody)
    {
        return flexBody.MeshName.Contains("wheel", StringComparison.OrdinalIgnoreCase) ||
               flexBody.MeshName.Contains("tire", StringComparison.OrdinalIgnoreCase) ||
               flexBody.MeshName.Contains("tyre", StringComparison.OrdinalIgnoreCase) ||
               flexBody.MeshName.Contains("hubcap", StringComparison.OrdinalIgnoreCase) ||
               flexBody.MeshName.Contains("rim", StringComparison.OrdinalIgnoreCase) ||
               flexBody.SourcePartName.Contains("wheel", StringComparison.OrdinalIgnoreCase) ||
               flexBody.SourcePartName.Contains("tire", StringComparison.OrdinalIgnoreCase) ||
               flexBody.SourcePartName.Contains("tyre", StringComparison.OrdinalIgnoreCase) ||
               flexBody.SourcePartName.Contains("hubcap", StringComparison.OrdinalIgnoreCase);
    }

    private void ApplyWheelFlexBodyTransform(
        Entity meshEntity,
        Entity wheelEntity,
        AssembledFlexBody flexBody,
        List<ColladaMesh> sourceMeshes)
    {
        var geometryAlreadyPositioned = ShouldTreatWheelGeometryAsPrePositioned(sourceMeshes, flexBody.Position);

        meshEntity.Transform.Position = geometryAlreadyPositioned || !flexBody.Position.HasValue
            ? -wheelEntity.Transform.Position
            : VehiclePhysicsBuilder.BeamNGToStride(flexBody.Position.Value) - wheelEntity.Transform.Position;

        if (!geometryAlreadyPositioned && flexBody.Rotation.HasValue)
        {
	        meshEntity.Transform.Rotation = BeamNGRotationToStride(flexBody.Rotation.Value);
        }

        if (!geometryAlreadyPositioned && flexBody.Scale.HasValue)
        {
	        meshEntity.Transform.Scale = BeamNGScaleToStride(flexBody.Scale.Value);
        }
    }

    internal static bool ShouldTreatWheelGeometryAsPrePositioned(
        List<ColladaMesh> sourceMeshes,
        System.Numerics.Vector3? expectedPosition)
    {
        return expectedPosition.HasValue &&
               GeometryMatchesPosition(sourceMeshes, expectedPosition.Value);
    }

    private static bool GeometryMatchesPosition(List<ColladaMesh> sourceMeshes, System.Numerics.Vector3 expectedPosition)
    {
        if (sourceMeshes.Count == 0)
        {
	        return false;
        }

        var sum = System.Numerics.Vector3.Zero;
        var count = 0;
        foreach (var mesh in sourceMeshes)
        {
            foreach (var vertex in mesh.Vertices)
            {
                sum += vertex.Position;
                count++;
            }
        }

        if (count == 0)
        {
	        return false;
        }

        var centroid = sum / count;
        return System.Numerics.Vector3.Distance(centroid, expectedPosition) < 0.25f;
    }

    private static Quaternion BeamNGRotationToStride(System.Numerics.Vector3 rotationDegrees)
    {
        var rx = MathUtil.DegreesToRadians(rotationDegrees.X);
        var ry = MathUtil.DegreesToRadians(rotationDegrees.Y);
        var rz = MathUtil.DegreesToRadians(rotationDegrees.Z);

        var rotX = Quaternion.RotationX(rx);
        var rotY = Quaternion.RotationAxis(-Vector3.UnitZ, ry);
        var rotZ = Quaternion.RotationY(rz);
        return Quaternion.Normalize(rotX * rotY * rotZ);
    }

    private static Vector3 BeamNGScaleToStride(System.Numerics.Vector3 scale)
        => new(scale.X, scale.Z, scale.Y);

    private void AttachFallbackChassis(Entity chassisEntity, VehicleDefinition definition)
    {
        Log.Warning("Using fallback orange box mesh.");
        var fallback = BuildFallbackChassisModel(definition);
        if (fallback != null)
        {
            var fallbackEntity = new Entity("chassis_fallback_mesh");
            fallbackEntity.Add(new ModelComponent { Model = fallback });
            chassisEntity.AddChild(fallbackEntity);
        }
    }

    private void AttachFallbackTires(
        VehicleBuilderResult result,
        PcConfig? pcConfig,
        Dictionary<string, int>? tireLikeAttachments)
    {
        var frontSpec = TryGetTireSpec(pcConfig, front: true, out var parsedFront)
            ? parsedFront
            : new TireSpec(0.305f, 0.205f);
        var rearSpec = TryGetTireSpec(pcConfig, front: false, out var parsedRear)
            ? parsedRear
            : frontSpec;

        var wheelSpecs = new Dictionary<string, (Entity Wheel, TireSpec Spec)>
        {
            ["wheel_FL"] = (result.WheelFL, frontSpec),
            ["wheel_FR"] = (result.WheelFR, frontSpec),
            ["wheel_RL"] = (result.WheelRL, rearSpec),
            ["wheel_RR"] = (result.WheelRR, rearSpec),
        };

        foreach (var (wheelKey, data) in wheelSpecs)
        {
            if (tireLikeAttachments != null &&
                tireLikeAttachments.TryGetValue(wheelKey, out var count) &&
                count > 0)
            {
                continue;
            }

            if (data.Wheel.GetChildren().Any(child => child.Name == $"{wheelKey}_fallback_tire"))
            {
	            continue;
            }

            var tireEntity = BuildFallbackTireEntity($"{wheelKey}_fallback_tire", data.Spec);
            data.Wheel.AddChild(tireEntity);
        }
    }

    private bool TryGetTireSpec(PcConfig? pcConfig, bool front, out TireSpec spec)
    {
        spec = default;
        if (pcConfig == null)
        {
	        return false;
        }

        var slotPrefix = front ? "tire_F" : "tire_R";
        var partName = pcConfig.Parts
            .Where(kv => kv.Key.StartsWith(slotPrefix, StringComparison.OrdinalIgnoreCase))
            .Select(kv => kv.Value)
            .FirstOrDefault(v => !string.IsNullOrWhiteSpace(v));

        if (string.IsNullOrWhiteSpace(partName))
        {
	        return false;
        }

        var match = Regex.Match(partName, @"(?<width>\d{3})_(?<aspect>\d{2})_(?<rim>\d{2})");
        if (!match.Success)
        {
	        return false;
        }

        var widthMetres = int.Parse(match.Groups["width"].Value) / 1000f;
        var aspectRatio = int.Parse(match.Groups["aspect"].Value) / 100f;
        var rimDiameterMetres = int.Parse(match.Groups["rim"].Value) * 0.0254f;
        var radius = rimDiameterMetres * 0.5f + (widthMetres * aspectRatio);

        spec = new TireSpec(radius, widthMetres);
        return true;
    }

    private Entity BuildFallbackTireEntity(string name, TireSpec spec)
    {
        var entity = new Entity(name);
        entity.Add(new ModelComponent
        {
            Model = BuildWheelCylinderModel(
                spec.Radius,
                spec.Width,
                new Color4(0.09f, 0.09f, 0.09f, 1f))
        });

        var rimRadius = Math.Max(spec.Radius * 0.62f, 0.12f);
        var rimWidth = spec.Width * 0.68f;
        var rimEntity = new Entity($"{name}_rim");
        rimEntity.Add(new ModelComponent
        {
            Model = BuildWheelCylinderModel(
                rimRadius,
                rimWidth,
                new Color4(0.68f, 0.70f, 0.74f, 1f))
        });
        entity.AddChild(rimEntity);

        var hubRadius = rimRadius * 0.26f;
        var hubWidth = rimWidth * 0.42f;
        var hubEntity = new Entity($"{name}_hub");
        hubEntity.Add(new ModelComponent
        {
            Model = BuildWheelCylinderModel(
                hubRadius,
                hubWidth,
                new Color4(0.26f, 0.28f, 0.31f, 1f))
        });
        entity.AddChild(hubEntity);

        return entity;
    }

    private Model BuildWheelCylinderModel(float radius, float width, Color4 color)
    {
        var halfWidth = width * 0.5f;
        const int Segments = 24;

        var vertices = new List<VertexPositionNormalTexture>();
        var indices = new List<int>();

        void AddVertex(float x, float y, float z, Vector3 normal, Vector2 uv)
            => vertices.Add(new VertexPositionNormalTexture(new Vector3(x, y, z), normal, uv));

        for (var i = 0; i <= Segments; i++)
        {
            var t = i / (float)Segments;
            var angle = MathUtil.TwoPi * t;
            var y = MathF.Cos(angle) * radius;
            var z = MathF.Sin(angle) * radius;
            var normal = Vector3.Normalize(new Vector3(0f, y, z));
            AddVertex(-halfWidth, y, z, normal, new Vector2(t, 1f));
            AddVertex(halfWidth, y, z, normal, new Vector2(t, 0f));
        }

        for (var i = 0; i < Segments; i++)
        {
            var baseIndex = i * 2;
            indices.AddRange(new[]
            {
                baseIndex, baseIndex + 1, baseIndex + 2,
                baseIndex + 1, baseIndex + 3, baseIndex + 2,
            });
        }

        var leftCenter = vertices.Count;
        AddVertex(-halfWidth, 0f, 0f, -Vector3.UnitX, new Vector2(0.5f, 0.5f));
        var rightCenter = vertices.Count;
        AddVertex(halfWidth, 0f, 0f, Vector3.UnitX, new Vector2(0.5f, 0.5f));

        var leftStart = rightCenter + 1;
        for (var i = 0; i <= Segments; i++)
        {
            var t = i / (float)Segments;
            var angle = MathUtil.TwoPi * t;
            var y = MathF.Cos(angle) * radius;
            var z = MathF.Sin(angle) * radius;
            AddVertex(-halfWidth, y, z, -Vector3.UnitX, new Vector2(0.5f + (y / (radius * 2f)), 0.5f + (z / (radius * 2f))));
        }

        var rightStart = vertices.Count;
        for (var i = 0; i <= Segments; i++)
        {
            var t = i / (float)Segments;
            var angle = MathUtil.TwoPi * t;
            var y = MathF.Cos(angle) * radius;
            var z = MathF.Sin(angle) * radius;
            AddVertex(halfWidth, y, z, Vector3.UnitX, new Vector2(0.5f + (y / (radius * 2f)), 0.5f + (z / (radius * 2f))));
        }

        for (var i = 0; i < Segments; i++)
        {
            indices.AddRange(new[] { leftCenter, leftStart + i + 1, leftStart + i });
            indices.AddRange(new[] { rightCenter, rightStart + i, rightStart + i + 1 });
        }

        var mesh = new Mesh
        {
            BoundingBox = new BoundingBox(
                new Vector3(-halfWidth, -radius, -radius),
                new Vector3(halfWidth, radius, radius)),
            Draw = new MeshDraw
            {
                PrimitiveType = PrimitiveType.TriangleList,
                VertexBuffers = new[]
                {
                    new VertexBufferBinding(
                        Stride.Graphics.Buffer.Vertex.New(_graphicsDevice, vertices.ToArray(), GraphicsResourceUsage.Immutable),
                        VertexPositionNormalTexture.Layout,
                        vertices.Count)
                },
                IndexBuffer = new IndexBufferBinding(
                    Stride.Graphics.Buffer.Index.New(_graphicsDevice, indices.ToArray()),
                    true,
                    indices.Count),
                DrawCount = indices.Count,
            }
        };

        return new Model
        {
            mesh,
            BuildMaterial(null, color),
        };
    }

    /// <summary>
    /// Groups sub-meshes by material symbol; for each group merges geometry into one GPU buffer
    /// and resolves a texture via three-level lookup:
    ///   1. BeamNG *.materials.json  (full path, most accurate)
    ///   2. Collada library_images chain  (filename in Textures/)
    ///   3. Grey placeholder  (unresolvable, e.g. sunburst base-car textures not in the mod pack)
    /// </summary>
    private Entity? BuildMeshEntity(
        List<ColladaMesh> colladaMeshes,
        string vehicleName,
        string textureSearchFolder,
        Dictionary<string, string> jsonMaterials,
        Dictionary<string, string> colladaTextureMap)
    {
        if (colladaMeshes.Count == 0)
        {
	        return null;
        }

        var groups = colladaMeshes
            .Where(cm => cm.Vertices.Count > 0 && cm.Indices.Count > 0)
            .GroupBy(cm => cm.MaterialName)
            .ToList();

        if (groups.Count == 0)
        {
	        return null;
        }

        var rootEntity = new Entity($"{vehicleName}_mesh");
        var built = 0;

        foreach (var group in groups)
        {
            var symbol = group.Key;

            // Merge all sub-meshes that share this material symbol
            var allVerts = new List<VertexPositionNormalTexture>();
            var allIndices = new List<int>();
            foreach (var cm in group)
            {
                var baseIndex = allVerts.Count;
                allVerts.AddRange(ConvertToVertexArray(cm.Vertices));
                foreach (var idx in cm.Indices)
                    allIndices.Add(idx + baseIndex);
            }
            if (allVerts.Count == 0)
            {
	            continue;
            }

            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            foreach (var v in allVerts) { min = Vector3.Min(min, v.Position); max = Vector3.Max(max, v.Position); }

            var mesh = new Mesh
            {
                BoundingBox = new BoundingBox(min, max),
                Draw = new MeshDraw
                {
                    PrimitiveType = PrimitiveType.TriangleList,
                    VertexBuffers = new[] { new VertexBufferBinding(
                        Stride.Graphics.Buffer.Vertex.New(_graphicsDevice, allVerts.ToArray(), GraphicsResourceUsage.Immutable),
                        VertexPositionNormalTexture.Layout, allVerts.Count) },
                    IndexBuffer = new IndexBufferBinding(
                        Stride.Graphics.Buffer.Index.New(_graphicsDevice, allIndices.ToArray()), true, allIndices.Count),
                    DrawCount = allIndices.Count,
                }
            };

            // ── Three-level texture lookup ────────────────────────────────────
            // 1. BeamNG JSON: strip "-material" suffix → exact material name
            var matName = StripMaterialSuffix(symbol);
            Texture? texture = null;

            if (jsonMaterials.TryGetValue(matName, out var absolutePath))
            {
	            texture = TryLoadTextureFromPath(absolutePath);
            }

            // 2. Collada library_images chain (filename only, look in Textures/)
            if (texture == null && colladaTextureMap.TryGetValue(symbol, out var colladaFile))
            {
	            texture = TryLoadTexture(textureSearchFolder, colladaFile);
            }

            // 3. Grey placeholder — covers sunburst base-car materials and anything else missing
            var material = BuildMaterial(texture);

            var subEntity = new Entity(symbol);
            subEntity.Add(new ModelComponent { Model = new Model { mesh, material } });
            rootEntity.AddChild(subEntity);
            built++;
        }

        Log.Info($"Mesh: {built} material groups ({colladaMeshes.Count} sub-meshes)");
        return built > 0 ? rootEntity : null;
    }

    private static string StripMaterialSuffix(string symbol)
    {
        var s = symbol;
        if (s.EndsWith("-material", StringComparison.OrdinalIgnoreCase))
        {
	        s = s[..^"-material".Length];
        }
        else if (s.EndsWith("_material", StringComparison.OrdinalIgnoreCase))
        {
	        s = s[..^"_material".Length];
        }

        return s;
    }

    private Material BuildMaterial(Texture? texture, Color4? fallbackColor = null)
    {
        IComputeColor diffuse = texture != null
            ? new ComputeTextureColor(texture)
            : new ComputeColor(fallbackColor ?? new Color4(0.55f, 0.55f, 0.55f, 1f));

        return Material.New(_graphicsDevice, new MaterialDescriptor
        {
            Attributes = new MaterialAttributes
            {
                Diffuse = new MaterialDiffuseMapFeature(diffuse),
                DiffuseModel = new MaterialDiffuseLambertModelFeature(),
            }
        });
    }

    /// <summary>Loads a texture from an absolute file path.</summary>
    private Texture? TryLoadTextureFromPath(string absolutePath)
    {
        if (!File.Exists(absolutePath))
        {
	        return null;
        }

        try
        {
            using var stream = File.OpenRead(absolutePath);
            var image = Image.Load(stream);
            if (image == null)
            {
	            return null;
            }

            var tex = Texture.New(_graphicsDevice, image);
            image.Dispose();
            Log.Info($"Texture loaded: {Path.GetFileName(absolutePath)}");
            return tex;
        }
        catch (Exception ex)
        {
            Log.Warning($"Texture failed '{Path.GetFileName(absolutePath)}': {ex.Message}");
            return null;
        }
    }

    /// <summary>
    /// Looks up a texture by filename in the vehicle folder (root or Textures/ subfolder).
    /// <paramref name="filename"/> may already include an extension (from Collada map) or be a base name.
    /// Handles BeamNG DAE files that reference .color.png names when actual assets are .color.dds.
    /// </summary>
    private Texture? TryLoadTexture(string vehicleFolder, string filename)
    {
        // Search directories: root folder first, then Textures/ subfolder
        var searchDirs = new List<string> { vehicleFolder };
        var texturesDir = Path.Combine(vehicleFolder, "Textures");
        if (Directory.Exists(texturesDir))
        {
	        searchDirs.Add(texturesDir);
        }

        foreach (var dir in searchDirs)
        {
            // 1. Try exact match if it has an extension
            if (!string.IsNullOrEmpty(Path.GetExtension(filename)))
            {
                var direct = Path.Combine(dir, filename);
                if (File.Exists(direct))
                {
	                return TryLoadTextureFromPath(direct);
                }

                // 2. Try replacing the extension — DAE often says .color.png but files are .color.dds
                var noExt = Path.GetFileNameWithoutExtension(filename);
                foreach (var ext in new[] { ".dds", ".png", ".jpg", ".jpeg", ".tga" })
                {
                    var path = Path.Combine(dir, noExt + ext);
                    if (File.Exists(path))
                    {
	                    return TryLoadTextureFromPath(path);
                    }
                }
            }
            else
            {
                // 3. No extension — try appending common extensions
                foreach (var ext in new[] { ".dds", ".png", ".jpg", ".jpeg", ".tga" })
                {
                    var path = Path.Combine(dir, filename + ext);
                    if (!File.Exists(path))
                    {
                        var found = Directory.GetFiles(dir, filename + ext, SearchOption.TopDirectoryOnly);
                        if (found.Length == 0)
                        {
	                        continue;
                        }

                        path = found[0];
                    }
                    var tex = TryLoadTextureFromPath(path);
                    if (tex != null)
                    {
	                    return tex;
                    }
                }
            }
        }
        return null;
    }

    private static VertexPositionNormalTexture[] ConvertToVertexArray(
        System.Collections.Generic.List<ColladaVertex> vertices)
    {
        var result = new VertexPositionNormalTexture[vertices.Count];
        for (var i = 0; i < vertices.Count; i++)
        {
            var v = vertices[i];
            // Convert BeamNG → Stride coordinate space
            var pos = VehiclePhysicsBuilder.BeamNGToStride(v.Position);
            var norm = VehiclePhysicsBuilder.BeamNGToStride(v.Normal);
            result[i] = new VertexPositionNormalTexture(
                new Vector3(pos.X, pos.Y, pos.Z),
                new Vector3(norm.X, norm.Y, norm.Z),
                // Collada uses OpenGL UV convention (V=0 at bottom); DirectX/Stride uses V=0 at top.
                new Vector2(v.TexCoord.X, 1.0f - v.TexCoord.Y));
        }
        return result;
    }

    /// <summary>Creates a bright orange box model sized to the chassis AABB as a fallback mesh.</summary>
    private Model? BuildFallbackChassisModel(VehicleDefinition definition)
    {
        try
        {
            var chassisPart = definition.Parts.FirstOrDefault(p => !p.Detachable);
            if (chassisPart == null)
            {
	            return null;
            }

            var nodes = chassisPart.ExclusiveNodeIds
                .Where(id => definition.Nodes.ContainsKey(id))
                .Select(id => definition.Nodes[id])
                .ToList();
            if (nodes.Count == 0)
            {
	            return null;
            }

            // Size from AABB of chassis nodes (Stride coords)
            var positions = nodes.Select(n => VehiclePhysicsBuilder.BeamNGToStride(n.Position)).ToList();
            var min = positions[0]; var max = positions[0];
            foreach (var p in positions)
            {
                min = Vector3.Min(min, p);
                max = Vector3.Max(max, p);
            }
            var centroid = (min + max) * 0.5f;
            var size = max - min;

            // Build vertices centered at origin (mesh is child of chassis entity which is at centroid)
            var half = size * 0.5f;
            var verts = BuildBoxVertices(half.X, half.Y, half.Z);
            var indices = BuildBoxIndices();

            // Offset all vertex positions by (localMeshOffset = min - centroid_of_mesh_local)
            // The chassis entity is at the centroid of its nodes, so mesh local offset should be zero
            var mesh = new Mesh
            {
                BoundingBox = new BoundingBox(-half, half),
                Draw = new MeshDraw
                {
                    PrimitiveType = PrimitiveType.TriangleList,
                    VertexBuffers = new[] { new VertexBufferBinding(
                        Stride.Graphics.Buffer.Vertex.New(_graphicsDevice, verts, GraphicsResourceUsage.Immutable),
                        VertexPositionNormalTexture.Layout, verts.Length) },
                    IndexBuffer = new IndexBufferBinding(
                        Stride.Graphics.Buffer.Index.New(_graphicsDevice, indices), true, indices.Length),
                    DrawCount = indices.Length,
                }
            };

            var mat = Material.New(_graphicsDevice, new MaterialDescriptor
            {
                Attributes = new MaterialAttributes
                {
                    Diffuse = new MaterialDiffuseMapFeature(new ComputeColor(new Color4(1f, 0.4f, 0f, 1f))),
                    DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                }
            });

            return new Model { mesh, mat };
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"[VehicleLoader] Fallback mesh failed: {ex.Message}");
            return null;
        }
    }

    private static VertexPositionNormalTexture[] BuildBoxVertices(float hx, float hy, float hz)
    {
        var verts = new List<VertexPositionNormalTexture>();
        void Face(Vector3 n, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            verts.Add(new(a, n, new(0, 1)));
            verts.Add(new(b, n, new(1, 1)));
            verts.Add(new(c, n, new(1, 0)));
            verts.Add(new(d, n, new(0, 0)));
        }
        Face(new(0,0,1),  new(-hx,-hy, hz), new( hx,-hy, hz), new( hx, hy, hz), new(-hx, hy, hz));
        Face(new(0,0,-1), new( hx,-hy,-hz), new(-hx,-hy,-hz), new(-hx, hy,-hz), new( hx, hy,-hz));
        Face(new(0,1,0),  new(-hx, hy, hz), new( hx, hy, hz), new( hx, hy,-hz), new(-hx, hy,-hz));
        Face(new(0,-1,0), new(-hx,-hy,-hz), new( hx,-hy,-hz), new( hx,-hy, hz), new(-hx,-hy, hz));
        Face(new(1,0,0),  new( hx,-hy, hz), new( hx,-hy,-hz), new( hx, hy,-hz), new( hx, hy, hz));
        Face(new(-1,0,0), new(-hx,-hy,-hz), new(-hx,-hy, hz), new(-hx, hy, hz), new(-hx, hy,-hz));
        return verts.ToArray();
    }

    private static int[] BuildBoxIndices()
    {
        var idx = new List<int>();
        for (var f = 0; f < 6; f++)
        {
            var b = f * 4;
            idx.AddRange(new[] { b, b+1, b+2, b, b+2, b+3 });
        }
        return idx.ToArray();
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Vehicle geometry estimation for dynamics system
    // ──────────────────────────────────────────────────────────────────────────

    /// <summary>
    /// Estimates the wheelbase (front-to-rear axle distance in metres) from wheel entity positions.
    /// Falls back to a typical rally car wheelbase if positions are not yet available.
    /// </summary>
    private static float EstimateWheelbase(VehicleBuilderResult result)
    {
        var fl = result.WheelFL.Transform.Position;
        var rl = result.WheelRL.Transform.Position;
        // Wheelbase is the Z-distance (forward axis) between front and rear
        var wb = MathF.Abs(fl.Z - rl.Z);
        return wb > 0.5f ? wb : 2.55f; // fallback
    }

    /// <summary>
    /// Estimates the track width (left-to-right wheel distance in metres) from wheel entity positions.
    /// </summary>
    private static float EstimateTrackWidth(VehicleBuilderResult result)
    {
        var fl = result.WheelFL.Transform.Position;
        var fr = result.WheelFR.Transform.Position;
        var tw = MathF.Abs(fl.X - fr.X);
        return tw > 0.5f ? tw : 1.50f; // fallback
    }

    /// <summary>
    /// Estimates vehicle mass from the total node weights in the definition.
    /// </summary>
    private static float EstimateVehicleMass(VehicleDefinition definition)
    {
        var totalMass = 0f;
        foreach (var node in definition.Nodes.Values)
            totalMass += node.Weight;
        return totalMass > 100f ? totalMass : 1200f; // fallback
    }
}

/// <summary>A loaded vehicle with its root entity and assembled definition.</summary>
public record VehicleLoadDiagnostics(string VehicleFolderPath, string? ConfigPath, float EstimatedMassKg);

public record LoadedVehicle(
    VehicleDefinition Definition,
    Entity RootEntity,
    RallyCarComponent CarComponent,
    Entity ChassisEntity,
    Entity WheelFL,
    Entity WheelFR,
    Entity WheelRL,
    Entity WheelRR,
    VehicleLoadDiagnostics Diagnostics);
