using System.Collections.Generic;

namespace LibreRally.Vehicle;

/// <summary>
/// Defines the axles of a vehicle.
/// </summary>
public enum VehicleSetupAxle
{
    /// <summary>Axle not specified.</summary>
    Unspecified,
    /// <summary>Front axle.</summary>
    Front,
    /// <summary>Rear axle.</summary>
    Rear,
}

/// <summary>
/// Contains overrides for pressure-based wheels.
/// </summary>
public sealed class VehiclePressureWheelOverrides
{
    /// <summary>Gets or sets the override pressure in PSI.</summary>
    public float? PressurePsi { get; set; }
}

/// <summary>
/// Represents a set of setup overrides (variables and tyre pressures) for a vehicle.
/// </summary>
public sealed class VehicleSetupOverrides
{
    /// <summary>Gets the dictionary of JBeam variable overrides.</summary>
    public Dictionary<string, float> VariableOverrides { get; } = new(System.StringComparer.OrdinalIgnoreCase);

    /// <summary>Gets the dictionary of pressure overrides per axle.</summary>
    public Dictionary<VehicleSetupAxle, VehiclePressureWheelOverrides> PressureWheelOverrides { get; } = new();

    /// <summary>
    /// Clears all overrides.
    /// </summary>
    public void Clear()
    {
        VariableOverrides.Clear();
        PressureWheelOverrides.Clear();
    }

    /// <summary>
    /// Creates a deep clone of the current overrides.
    /// </summary>
    /// <returns>A new <see cref="VehicleSetupOverrides"/> instance.</returns>
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
