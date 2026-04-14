using System;
using System.Collections.Generic;
using System.Linq;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;

namespace LibreRally.Vehicle;

public sealed class VehiclePowertrainSetup
{
    public bool DriveFrontAxle { get; init; } = true;
    public bool DriveRearAxle { get; init; } = true;
    public string[] DrivenWheelKeys { get; init; } = { "wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR" };
    public float[] GearRatios { get; init; } = { 3.25f, 3.64f, 2.38f, 1.76f, 1.35f, 1.06f, 0.84f };
    public float FinalDrive { get; init; } = 4.55f;
    public float[] TorqueCurveRpm { get; init; } = { 0f, 500f, 1000f, 1500f, 2000f, 2500f, 3000f, 3500f, 4000f, 4500f, 5000f, 5500f, 6000f, 6500f, 7000f, 7500f };
    public float[] TorqueCurveNm { get; init; } = { 0f, 62f, 105f, 142f, 176f, 198f, 210f, 216f, 221f, 222f, 221f, 218f, 210f, 199f, 186f, 168f };
    public float MaxRpm { get; init; } = 7500f;
    public float IdleRpm { get; init; } = 900f;
    public float EngineInertia { get; init; } = 0.25f;
    public float EngineFriction { get; init; } = 11.5f;
    public float EngineDynamicFriction { get; init; } = 0.024f;
    public float EngineBrakeTorque { get; init; } = 38f;
    public float AutoClutchLaunchRpm { get; init; } = 4500f;
    public float ShiftUpRpm { get; init; } = 6500f;
    public float ShiftDownRpm { get; init; } = 2200f;
    public DifferentialConfig FrontDiff { get; init; } = DifferentialConfig.CreateLimitedSlip(2.0f, 0.25f);
    public DifferentialConfig RearDiff { get; init; } = DifferentialConfig.CreateLimitedSlip(3.0f, 0.35f);
    public DifferentialConfig CenterDiff { get; init; } = DifferentialConfig.CreateLimitedSlip(1.8f, 0.3f);
}

public static class VehiclePowertrainResolver
{
    private static readonly string[] DefaultDrivenWheelKeys = { "wheel_FL", "wheel_FR", "wheel_RL", "wheel_RR" };
    private static readonly HashSet<string> FrontWheelCodes = new(StringComparer.OrdinalIgnoreCase) { "FL", "FR" };
    private static readonly HashSet<string> RearWheelCodes = new(StringComparer.OrdinalIgnoreCase) { "RL", "RR" };

    public static VehiclePowertrainSetup Resolve(VehicleDefinition definition)
    {
        var deviceMap = definition.PowertrainDevices
            .Where(device => !string.IsNullOrWhiteSpace(device.Name))
            .GroupBy(device => device.Name, StringComparer.OrdinalIgnoreCase)
            .ToDictionary(group => group.Key, group => group.First(), StringComparer.OrdinalIgnoreCase);
        var childMap = BuildChildMap(definition.PowertrainDevices);
        var engineNames = new HashSet<string>(
            definition.PowertrainDevices
                .Where(device => IsEngineType(device.Type))
                .Select(device => device.Name),
            StringComparer.OrdinalIgnoreCase);
        if (engineNames.Count == 0 && definition.Engine != null)
        {
            engineNames.Add("mainEngine");
        }

        var drivenWheelCodes = ResolveDrivenWheelCodes(definition.PowertrainDevices, deviceMap, engineNames);
        var driveFrontAxle = drivenWheelCodes.Overlaps(FrontWheelCodes);
        var driveRearAxle = drivenWheelCodes.Overlaps(RearWheelCodes);
        if (!driveFrontAxle && !driveRearAxle)
        {
            driveFrontAxle = true;
            driveRearAxle = true;
        }

        var finalDriveFallback = driveRearAxle && !driveFrontAxle
            ? GetVar(definition, "finaldrive_R", GetVar(definition, "finaldrive_F", 4.55f))
            : GetVar(definition, "finaldrive_F", GetVar(definition, "finaldrive_R", 4.55f));
        var finalDrive = ResolveFinalDrive(definition.PowertrainDevices, deviceMap, engineNames, drivenWheelCodes, finalDriveFallback);
        var gearRatios = ResolveGearRatios(definition.Gearbox);
        var torqueCurve = ResolveTorqueCurve(definition.Engine);
        var maxRpm = definition.Engine?.MaxRpm > 0f ? definition.Engine.MaxRpm : GetVar(definition, "maxRPM", 7500f);
        var idleRpm = definition.Engine?.IdleRpm > 0f ? definition.Engine.IdleRpm : GetVar(definition, "idleRPM", 900f);
        var controller = definition.VehicleController;
        var shiftUpRpm = ResolveShiftUpRpm(controller, maxRpm);
        var shiftDownRpm = ResolveShiftDownRpm(controller);
        var autoClutchLaunchRpm = controller?.ClutchLaunchTargetRpm
            ?? controller?.ClutchLaunchStartRpm
            ?? MathF.Max(idleRpm + 600f, shiftDownRpm);

        return new VehiclePowertrainSetup
        {
            DriveFrontAxle = driveFrontAxle,
            DriveRearAxle = driveRearAxle,
            DrivenWheelKeys = drivenWheelCodes.Count > 0
                ? drivenWheelCodes.Select(ToWheelKey).OrderBy(key => key, StringComparer.OrdinalIgnoreCase).ToArray()
                : DefaultDrivenWheelKeys,
            GearRatios = gearRatios,
            FinalDrive = finalDrive,
            TorqueCurveRpm = torqueCurve.rpm,
            TorqueCurveNm = torqueCurve.torque,
            MaxRpm = maxRpm,
            IdleRpm = idleRpm,
            EngineInertia = definition.Engine?.Inertia > 0f ? definition.Engine.Inertia : 0.25f,
            EngineFriction = definition.Engine?.Friction > 0f ? definition.Engine.Friction : 11.5f,
            EngineDynamicFriction = definition.Engine?.DynamicFriction > 0f ? definition.Engine.DynamicFriction : 0.024f,
            EngineBrakeTorque = definition.Engine?.EngineBrakeTorque > 0f ? definition.Engine.EngineBrakeTorque : 38f,
            AutoClutchLaunchRpm = autoClutchLaunchRpm,
            ShiftUpRpm = shiftUpRpm,
            ShiftDownRpm = shiftDownRpm,
            FrontDiff = ResolveDifferential(definition.PowertrainDevices, childMap, FrontWheelCodes, DifferentialConfig.CreateLimitedSlip(2.0f, 0.25f)),
            RearDiff = ResolveDifferential(definition.PowertrainDevices, childMap, RearWheelCodes, DifferentialConfig.CreateLimitedSlip(3.0f, 0.35f)),
            CenterDiff = ResolveCenterDifferential(definition.PowertrainDevices, childMap),
        };
    }

    private static Dictionary<string, List<JBeamPowertrainDevice>> BuildChildMap(IEnumerable<JBeamPowertrainDevice> devices)
    {
        var childMap = new Dictionary<string, List<JBeamPowertrainDevice>>(StringComparer.OrdinalIgnoreCase);
        foreach (var device in devices)
        {
            if (string.IsNullOrWhiteSpace(device.InputName))
            {
                continue;
            }

            if (!childMap.TryGetValue(device.InputName, out var children))
            {
                children = new List<JBeamPowertrainDevice>();
                childMap[device.InputName] = children;
            }

            children.Add(device);
        }

        return childMap;
    }

    private static HashSet<string> ResolveDrivenWheelCodes(
        IEnumerable<JBeamPowertrainDevice> devices,
        IReadOnlyDictionary<string, JBeamPowertrainDevice> deviceMap,
        IReadOnlySet<string> engineNames)
    {
        var driven = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        foreach (var device in devices)
        {
            if (string.IsNullOrWhiteSpace(device.ConnectedWheel))
            {
                continue;
            }

            if (IsConnectedToEngine(device, deviceMap, engineNames))
            {
                driven.Add(device.ConnectedWheel);
            }
        }

        return driven;
    }

    private static bool IsConnectedToEngine(
        JBeamPowertrainDevice startDevice,
        IReadOnlyDictionary<string, JBeamPowertrainDevice> deviceMap,
        IReadOnlySet<string> engineNames)
    {
        var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        var currentInput = startDevice.InputName;
        while (!string.IsNullOrWhiteSpace(currentInput))
        {
            if (!visited.Add(currentInput))
            {
                break;
            }

            if (engineNames.Contains(currentInput))
            {
                return true;
            }

            if (!deviceMap.TryGetValue(currentInput, out var upstreamDevice))
            {
                break;
            }

            if (IsEngineType(upstreamDevice.Type))
            {
                return true;
            }

            currentInput = upstreamDevice.InputName;
        }

        return false;
    }

    private static float ResolveFinalDrive(
        IEnumerable<JBeamPowertrainDevice> devices,
        IReadOnlyDictionary<string, JBeamPowertrainDevice> deviceMap,
        IReadOnlySet<string> engineNames,
        IReadOnlySet<string> drivenWheelCodes,
        float fallback)
    {
        var ratios = new List<float>();
        foreach (var device in devices)
        {
            if (string.IsNullOrWhiteSpace(device.ConnectedWheel) ||
                !drivenWheelCodes.Contains(device.ConnectedWheel) ||
                !IsConnectedToEngine(device, deviceMap, engineNames))
            {
                continue;
            }

            var ratio = 1f;
            var foundAny = false;
            var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var currentInput = device.InputName;
            while (!string.IsNullOrWhiteSpace(currentInput))
            {
                if (!visited.Add(currentInput))
                {
                    break;
                }

                if (engineNames.Contains(currentInput))
                {
                    break;
                }

                if (!deviceMap.TryGetValue(currentInput, out var upstreamDevice))
                {
                    break;
                }

                if (!IsGearboxType(upstreamDevice.Type) &&
                    !IsEngineType(upstreamDevice.Type) &&
                    upstreamDevice.GearRatio is { } gearRatio &&
                    MathF.Abs(gearRatio) > 1e-4f)
                {
                    ratio *= MathF.Abs(gearRatio);
                    foundAny = true;
                }

                currentInput = upstreamDevice.InputName;
            }

            if (foundAny)
            {
                ratios.Add(ratio);
            }
        }

        return ratios.Count > 0 ? ratios.Average() : fallback;
    }

    private static float[] ResolveGearRatios(JBeamGearboxDefinition? gearbox)
    {
        if (gearbox?.GearRatios == null || gearbox.GearRatios.Count == 0)
        {
            return new[] { 3.25f, 3.64f, 2.38f, 1.76f, 1.35f, 1.06f, 0.84f };
        }

        var normalized = new List<float>();
        var reverse = gearbox.GearRatios[0];
        normalized.Add(MathF.Abs(reverse) > 1e-4f ? MathF.Abs(reverse) : 3.25f);
        foreach (var ratio in gearbox.GearRatios.Skip(1).Where(ratio => ratio > 1e-4f))
        {
            normalized.Add(MathF.Abs(ratio));
        }

        return normalized.Count > 1 ? normalized.ToArray() : new[] { 3.25f, 3.64f, 2.38f, 1.76f, 1.35f, 1.06f, 0.84f };
    }

    private static (float[] rpm, float[] torque) ResolveTorqueCurve(JBeamEngineDefinition? engine)
    {
        if (engine?.TorqueCurve == null || engine.TorqueCurve.Count < 2)
        {
            return (
                new[] { 0f, 500f, 1000f, 1500f, 2000f, 2500f, 3000f, 3500f, 4000f, 4500f, 5000f, 5500f, 6000f, 6500f, 7000f, 7500f },
                new[] { 0f, 62f, 105f, 142f, 176f, 198f, 210f, 216f, 221f, 222f, 221f, 218f, 210f, 199f, 186f, 168f });
        }

        return (
            engine.TorqueCurve.Select(point => point.Rpm).ToArray(),
            engine.TorqueCurve.Select(point => point.Torque).ToArray());
    }

    private static float ResolveShiftUpRpm(JBeamVehicleControllerDefinition? controller, float maxRpm)
    {
        if (controller?.HighShiftUpRpm is { } highShiftUp && highShiftUp > 0f)
        {
            return highShiftUp;
        }

        var lowShiftUp = GetPositiveValues(controller?.LowShiftUpRpm).DefaultIfEmpty().Max();
        if (lowShiftUp > 0f)
        {
            return lowShiftUp;
        }

        return MathF.Max(2500f, maxRpm * 0.85f);
    }

    private static float ResolveShiftDownRpm(JBeamVehicleControllerDefinition? controller)
    {
        var candidates = GetPositiveValues(controller?.HighShiftDownRpm)
            .Concat(GetPositiveValues(controller?.LowShiftDownRpm))
            .ToArray();
        if (candidates.Length == 0)
        {
            return 2200f;
        }

        return candidates.Average();
    }

    private static IEnumerable<float> GetPositiveValues(IEnumerable<float>? values)
    {
        return values?.Where(value => value > 0f) ?? Enumerable.Empty<float>();
    }

    private static DifferentialConfig ResolveDifferential(
        IEnumerable<JBeamPowertrainDevice> devices,
        IReadOnlyDictionary<string, List<JBeamPowertrainDevice>> childMap,
        IReadOnlySet<string> axleWheelCodes,
        DifferentialConfig fallback)
    {
        foreach (var device in devices)
        {
            if (!IsDifferentialType(device.Type))
            {
                continue;
            }

            var wheels = CollectConnectedWheels(device.Name, childMap);
            if (wheels.SetEquals(axleWheelCodes))
            {
                return MapDifferential(device, fallback);
            }
        }

        return fallback;
    }

    private static DifferentialConfig ResolveCenterDifferential(
        IEnumerable<JBeamPowertrainDevice> devices,
        IReadOnlyDictionary<string, List<JBeamPowertrainDevice>> childMap)
    {
        foreach (var device in devices)
        {
            if (!IsDifferentialType(device.Type))
            {
                continue;
            }

            var wheels = CollectConnectedWheels(device.Name, childMap);
            var hasFront = wheels.Overlaps(FrontWheelCodes);
            var hasRear = wheels.Overlaps(RearWheelCodes);
            if (hasFront && hasRear)
            {
                return MapDifferential(device, DifferentialConfig.CreateLimitedSlip(1.8f, 0.3f));
            }
        }

        return DifferentialConfig.CreateLimitedSlip(1.8f, 0.3f);
    }

    private static HashSet<string> CollectConnectedWheels(
        string rootDeviceName,
        IReadOnlyDictionary<string, List<JBeamPowertrainDevice>> childMap)
    {
        var result = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        var pending = new Stack<string>();
        var visited = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        pending.Push(rootDeviceName);

        while (pending.Count > 0)
        {
            var current = pending.Pop();
            if (!visited.Add(current) || !childMap.TryGetValue(current, out var children))
            {
                continue;
            }

            foreach (var child in children)
            {
                if (!string.IsNullOrWhiteSpace(child.ConnectedWheel))
                {
                    result.Add(child.ConnectedWheel);
                }

                pending.Push(child.Name);
            }
        }

        return result;
    }

    private static DifferentialConfig MapDifferential(JBeamPowertrainDevice device, DifferentialConfig fallback)
    {
        var diffType = device.DiffType?.Trim().ToLowerInvariant();
        return diffType switch
        {
            "open" => DifferentialConfig.CreateOpen(),
            "lsd" or "limitedslip" => DifferentialConfig.CreateLimitedSlip(
                biasRatio: fallback.BiasRatio > 1f && float.IsFinite(fallback.BiasRatio) ? fallback.BiasRatio : 2.5f,
                lockingCoeff: Math.Clamp(device.LsdLockCoef ?? fallback.LockingCoefficient, 0f, 1f),
                coastLockingCoeff: Math.Clamp(device.LsdRevLockCoef ?? device.LsdLockCoef ?? fallback.CoastLockingCoefficient, 0f, 1f),
                preloadTorque: MathF.Max(0f, device.LsdPreload ?? fallback.PreloadTorque)),
            "locked" or "spool" or "welded" => DifferentialConfig.CreateLocking(),
            _ => fallback,
        };
    }

    private static string ToWheelKey(string wheelCode) => $"wheel_{wheelCode.ToUpperInvariant()}";

    private static float GetVar(VehicleDefinition definition, string name, float fallback)
    {
        return definition.Vars.TryGetValue(name, out var value) && value > 0f ? value : fallback;
    }

    private static bool IsEngineType(string type) => type.Contains("engine", StringComparison.OrdinalIgnoreCase);

    private static bool IsGearboxType(string type) => type.Contains("gearbox", StringComparison.OrdinalIgnoreCase);

    private static bool IsDifferentialType(string type) => type.Contains("differential", StringComparison.OrdinalIgnoreCase);
}
