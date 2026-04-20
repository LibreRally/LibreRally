using System.Collections.Generic;
using System.Numerics;
using LibreRally.Vehicle.JBeam;

namespace LibreRally.Vehicle;

/// <summary>
/// A single assembled node: a named point-mass in the vehicle graph.
/// Positions use BeamNG coordinate space (Y-forward, Z-up).
/// </summary>
/// <param name="Id">Unique node identifier.</param>
/// <param name="Position">Node position in BeamNG space.</param>
/// <param name="Weight">Node mass in kilograms.</param>
/// <param name="Groups">Node groups assigned to the node.</param>
/// <param name="Collision">Whether the node participates in collision.</param>
public record AssembledNode(
    string Id,
    Vector3 Position,
    float Weight,
    List<string> Groups,
    bool Collision);

/// <summary>
/// A single assembled beam connecting two nodes.
/// </summary>
/// <param name="Id1">Identifier of the first node.</param>
/// <param name="Id2">Identifier of the second node.</param>
/// <param name="Spring">Resolved beam spring stiffness.</param>
/// <param name="Damp">Resolved beam damping coefficient.</param>
/// <param name="DeformThreshold">Resolved deformation threshold.</param>
/// <param name="BreakStrength">Resolved break strength.</param>
/// <param name="BeamType">BeamNG beam type token.</param>
/// <param name="DeformGroup">Optional deformation group name.</param>
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
/// When present, the transform offset uses the absolute BeamNG-space origin from the flexbody row.
/// </summary>
/// <param name="MeshName">Mesh object name from the imported model.</param>
/// <param name="NodeGroups">Node groups that drive the mesh.</param>
/// <param name="Position">Optional mesh origin override in BeamNG space.</param>
/// <param name="Rotation">Optional mesh rotation override in BeamNG space.</param>
/// <param name="Scale">Optional mesh scale override.</param>
/// <param name="SourcePartName">Name of the source JBeam part.</param>
/// <param name="SourceSlotType">Slot type of the source JBeam part.</param>
public record AssembledFlexBody(
    string MeshName,
    List<string> NodeGroups,
    System.Numerics.Vector3? Position = null,
    System.Numerics.Vector3? Rotation = null,
    System.Numerics.Vector3? Scale = null,
    string SourcePartName = "",
    string SourceSlotType = "");

/// <summary>
/// Assembled <paramref name="Options" /> for a pressure-based wheel.
/// </summary>
/// <param name="SourcePartName">The name of the source JBeam part.</param>
/// <param name="SourceSlotType">The slot type of the source JBeam part.</param>
/// <param name="Options">The resolved pressure wheel options.</param>
public record AssembledPressureWheelOptions(
    string SourcePartName,
    string SourceSlotType,
    JBeamPressureWheelOptions Options);

/// <summary>
/// Assembled gear ratio from a specific part (e.g. final drive).
/// </summary>
/// <param name="SourcePartName">The name of the source JBeam part.</param>
/// <param name="SourceSlotType">The slot type of the source JBeam part.</param>
/// <param name="GearRatio">The resolved gear ratio.</param>
public record AssembledPartGearRatio(
    string SourcePartName,
    string SourceSlotType,
    float GearRatio);

/// <summary>
/// Active BeamNG material-skin selection contributed by a resolved part slot.
/// </summary>
/// <param name="SlotType">The slot type that selected the skin (for example <c>paint_design</c>).</param>
/// <param name="VariantName">The resolved material variant token (for example <c>ngrc</c> or <c>rs</c>).</param>
public readonly record struct ActiveMaterialSkinSelection(
    string SlotType,
    string VariantName);

/// <summary>
/// A logical car part derived from a jbeam slot, containing all its nodes and mesh info.
/// Parts marked Detachable become separate physics bodies with a breakable weld.
/// </summary>
public class VehiclePart
{
    /// <summary>Gets the name of the part.</summary>
    public string Name { get; init; } = "";

	/// <summary>Gets or sets whether this part should become a detachable secondary physics body.</summary>
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
    /// <summary>Gets the name of the vehicle.</summary>
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

    /// <summary>Active material-skin selections from the resolved BeamNG part tree.</summary>
    public List<ActiveMaterialSkinSelection> ActiveMaterialSkinSelections { get; init; } = new();

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
