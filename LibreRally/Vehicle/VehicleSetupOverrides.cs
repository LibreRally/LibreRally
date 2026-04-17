using System.Collections.Generic;

namespace LibreRally.Vehicle;

public sealed class VehiclePressureWheelOverrides
{
    public float? PressurePsi { get; set; }
    public float? WheelOffset { get; set; }
    public float? Radius { get; set; }
    public float? TireWidth { get; set; }
    public float? HubRadius { get; set; }
    public float? HubWidth { get; set; }
}

public sealed class VehicleSetupOverrides
{
    public Dictionary<string, float> VariableOverrides { get; } = new(System.StringComparer.OrdinalIgnoreCase);
    public Dictionary<VehicleSetupAxle, VehiclePressureWheelOverrides> PressureWheelOverrides { get; } = new();

    public VehicleSetupOverrides Clone()
    {
        var clone = new VehicleSetupOverrides();
        foreach (var kv in VariableOverrides)
        {
            clone.VariableOverrides[kv.Key] = kv.Value;
        }

        foreach (var kv in PressureWheelOverrides)
        {
            clone.PressureWheelOverrides[kv.Key] = new VehiclePressureWheelOverrides
            {
                PressurePsi = kv.Value.PressurePsi,
                WheelOffset = kv.Value.WheelOffset,
                Radius = kv.Value.Radius,
                TireWidth = kv.Value.TireWidth,
                HubRadius = kv.Value.HubRadius,
                HubWidth = kv.Value.HubWidth,
            };
        }

        return clone;
    }
}

public sealed class VehicleSetupApplyResult
{
    public List<string> LiveAppliedKeys { get; } = new();
    public List<string> ReloadRequiredKeys { get; } = new();
    public List<string> UnknownKeys { get; } = new();

    public bool AppliedLiveChanges => LiveAppliedKeys.Count > 0;
    public bool RequiresReload => ReloadRequiredKeys.Count > 0;
}
