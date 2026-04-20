using System.Collections.Generic;
using System.Numerics;
using LibreRally.Vehicle.JBeam;

namespace LibreRally.Vehicle;

/// <summary>
/// A single assembled node: a named point-mass in the vehicle graph.
/// Position is in BeamNG coordinate space (Y-forward, Z-up).
/// </summary>
public record AssembledNode(
    string Id,
    Vector3 Position,
    float Weight,
    List<string> Groups,
    bool Collision);

/// <summary>
/// A single assembled beam connecting two nodes.
/// </summary>
public record AssembledBeam(
    string Id1,
    string Id2,
    float Spring,
    float Damp,
    float DeformThreshold,
    float BreakStrength,
    string BeamType,
    string DeformGroup);

/// <summary>
/// A named group of mesh objects and the node groups they are driven by.
/// Position is the absolute BeamNG-space origin of the mesh (when provided in the flexbody row).
/// </summary>
public record AssembledFlexBody(
    string MeshName,
    List<string> NodeGroups,
    System.Numerics.Vector3? Position = null,
    System.Numerics.Vector3? Rotation = null,
    System.Numerics.Vector3? Scale = null,
    string SourcePartName = "",
    string SourceSlotType = "");

public record AssembledPressureWheelOptions(
    string SourcePartName,
    string SourceSlotType,
    JBeamPressureWheelOptions Options);

public record AssembledPartGearRatio(
    string SourcePartName,
    string SourceSlotType,
    float GearRatio);

/// <summary>
/// A logical car part derived from a jbeam slot, containing all its nodes and mesh info.
/// Parts marked Detachable become separate physics bodies with a breakable weld.
/// </summary>
public class VehiclePart
{
    public string Name { get; init; } = "";

    /// <summary>True when this part should become a separate physics body that can detach.</summary>
    public bool Detachable { get; set; }

    /// <summary>Node IDs that belong exclusively to this part (not shared with chassis).</summary>
    public List<string> ExclusiveNodeIds { get; init; } = new();

    /// <summary>
    /// The minimum beamStrength of all beams connecting this part to the rest of the car.
    /// This is the force threshold at which it detaches.
    /// </summary>
    public float BreakStrength { get; set; } = float.MaxValue;

    /// <summary>Mesh objects driven by this part's node groups.</summary>
    public List<AssembledFlexBody> FlexBodies { get; init; } = new();
}

/// <summary>
/// A fully assembled vehicle definition produced by <see cref="JBeam.JBeamAssembler"/>.
/// All nodes, beams, and parts from the slot hierarchy are merged into flat collections.
/// Coordinates are in BeamNG space: X = left, Y = forward, Z = up.
/// </summary>
public class VehicleDefinition
{
    public string VehicleName { get; init; } = "";

    /// <summary>All nodes keyed by ID.</summary>
    public Dictionary<string, AssembledNode> Nodes { get; init; } = new();

    /// <summary>All beams in the assembled vehicle.</summary>
    public List<AssembledBeam> Beams { get; init; } = new();

    /// <summary>All flex-body mesh mappings.</summary>
    public List<AssembledFlexBody> FlexBodies { get; init; } = new();

    /// <summary>
    /// Logical parts (chassis + detachable pieces).
    /// The first entry (index 0) is always the main chassis.
    /// </summary>
    public List<VehiclePart> Parts { get; init; } = new();

    /// <summary>Path to the vehicle folder (used for loading meshes).</summary>
    public string FolderPath { get; init; } = "";

    /// <summary>
    /// Orientation ref nodes from jbeam refNodes.
    /// Keys: "ref", "back", "left", "up", "leftCorner", "rightCorner".
    /// </summary>
    public Dictionary<string, string> RefNodes { get; init; } = new();

    /// <summary>
    /// Physics variables from the active JBeam variable defaults plus any loaded .pc overrides.
    /// Variable names are stripped of their leading '$' (e.g., "spring_F_asphalt" = 60000).
    /// Used by <see cref="Physics.VehiclePhysicsBuilder"/> to set spring, damping, etc.
    /// </summary>
    public Dictionary<string, float> Vars { get; } = new(System.StringComparer.OrdinalIgnoreCase);

    /// <summary>Setup-variable metadata from the active BeamNG parts.</summary>
    public List<JBeamVariableDefinition> SetupVariables { get; init; } = new();

    /// <summary>Powertrain device graph assembled from active parts.</summary>
    public List<JBeamPowertrainDevice> PowertrainDevices { get; init; } = new();

    /// <summary>Pressure wheel metadata assembled from active parts.</summary>
    public List<JBeamPressureWheel> PressureWheels { get; init; } = new();

    /// <summary>Pressure-wheel option objects assembled from the active parts.</summary>
    public List<AssembledPressureWheelOptions> PressureWheelOptions { get; init; } = new();

    /// <summary>Top-level gear ratios defined by active selected parts such as final drives.</summary>
    public List<AssembledPartGearRatio> PartGearRatios { get; init; } = new();

    /// <summary>Engine metadata parsed from the active JBeam parts, when present.</summary>
    public JBeamEngineDefinition? Engine { get; init; }

    /// <summary>Gearbox metadata parsed from the active JBeam parts, when present.</summary>
    public JBeamGearboxDefinition? Gearbox { get; init; }

    /// <summary>Shift and launch metadata parsed from the active JBeam vehicleController sections, when present.</summary>
    public JBeamVehicleControllerDefinition? VehicleController { get; init; }

    /// <summary>Brake-controller metadata parsed from active parts, including ABS flags and targets.</summary>
    public JBeamBrakeControlDefinition? BrakeControl { get; init; }

    /// <summary>Traction-control metadata parsed from active parts, when present.</summary>
    public JBeamTractionControlDefinition? TractionControl { get; init; }

    /// <summary>Fuel-tank metadata parsed from active parts (capacity, starting fuel).</summary>
    public JBeamFuelTankDefinition? FuelTank { get; init; }
}
