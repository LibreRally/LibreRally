using System.Collections.Generic;

namespace LibreRally.Vehicle;

public enum VehicleSetupAxle
{
    Unspecified,
    Front,
    Rear,
}

public sealed class VehiclePressureWheelOverrides
{
    public float? PressurePsi { get; set; }
}

public sealed class VehicleSetupOverrides
{
    public Dictionary<string, float> VariableOverrides { get; } = new(System.StringComparer.OrdinalIgnoreCase);
    public Dictionary<VehicleSetupAxle, VehiclePressureWheelOverrides> PressureWheelOverrides { get; } = new();

    public void Clear()
    {
        VariableOverrides.Clear();
        PressureWheelOverrides.Clear();
    }

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
            };
        }

        return clone;
    }
}
