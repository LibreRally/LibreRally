using System.Collections.Generic;
using System.Linq;

namespace LibreRally.Vehicle;

public sealed class VehicleRuntimeSetupCategory
{
    public string Name { get; init; } = "";
    public List<VehicleSetupVariable> Variables { get; init; } = new();
}

public sealed class VehicleRuntimeSetupState
{
    public Dictionary<string, VehicleRuntimeSetupCategory> Categories { get; init; } = new(System.StringComparer.OrdinalIgnoreCase);

    public static VehicleRuntimeSetupState From(VehicleSetupSchema schema)
    {
        var allVariables = schema.Variables.Values.ToArray();
        var categories = VehicleSetupSupport.BuildRuntimeSetupVariables(allVariables)
            .Where(variable =>
                variable.Category.Equals("Suspension", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Equals("Brakes", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Equals("Differentials", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Equals("Engine", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Equals("Transmission", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Equals("Chassis", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Contains("Alignment", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Contains("Wheel", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Contains("Tire", System.StringComparison.OrdinalIgnoreCase) ||
                variable.Category.Contains("Tyre", System.StringComparison.OrdinalIgnoreCase))
            .GroupBy(variable => variable.Category, System.StringComparer.OrdinalIgnoreCase)
            .ToDictionary(
                group => group.Key,
                group => new VehicleRuntimeSetupCategory
                {
                    Name = group.Key,
                    Variables = group
                        .OrderBy(variable => variable.SubCategory)
                        .ThenBy(variable => variable.Title)
                        .ToList(),
                },
                System.StringComparer.OrdinalIgnoreCase);

        return new VehicleRuntimeSetupState { Categories = categories };
    }
}
