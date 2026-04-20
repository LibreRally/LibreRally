using System.Collections.Generic;
using System.Numerics;

namespace LibreRally.Vehicle.JBeam;

/// <summary>Raw per-node properties accumulated from preceding property-setter objects.</summary>
/// <param name="Weight">Node mass in kilograms.</param>
/// <param name="Collision">Whether the node participates in collision tests.</param>
/// <param name="SelfCollision">Whether the node can collide with other nodes on the same vehicle.</param>
/// <param name="Material">BeamNG material token applied to the node.</param>
/// <param name="FrictionCoef">Per-node friction coefficient override.</param>
/// <param name="Groups">Node groups assigned to the node.</param>
public record NodeProperties(
    float Weight,
    bool Collision,
    bool SelfCollision,
    string Material,
    float FrictionCoef,
    List<string> Groups);

/// <summary>Default values for <see cref="NodeProperties"/>.</summary>
public static class NodeDefaults
{
    /// <summary>Gets the baseline node properties.</summary>
    public static NodeProperties Default => new(
        Weight: 25f,
        Collision: true,
        SelfCollision: true,
        Material: "|NM_METAL",
        FrictionCoef: 0.5f,
        Groups: new List<string>());
}

/// <summary>A single node (point mass) in the jbeam graph.</summary>
/// <param name="Id">Unique node identifier.</param>
/// <param name="Position">Node position in BeamNG space.</param>
/// <param name="Properties">Resolved per-node properties.</param>
public record JBeamNode(
    string Id,
    Vector3 Position,
    NodeProperties Properties);

/// <summary>Raw per-beam properties accumulated from preceding property-setter objects.</summary>
/// <param name="Spring">Beam spring stiffness.</param>
/// <param name="Damp">Beam damping coefficient.</param>
/// <param name="Deform">Beam deformation threshold.</param>
/// <param name="Strength">Beam break strength.</param>
/// <param name="BeamType">BeamNG beam behaviour token.</param>
/// <param name="DeformGroup">Deformation group name when one is supplied.</param>
/// <param name="DeformationTriggerRatio">Damage ratio that triggers deformation events.</param>
/// <param name="Optional">Whether the beam may be omitted when references are missing.</param>
public record BeamProperties(
    float Spring,
    float Damp,
    float Deform,
    float Strength,
    string BeamType,
    string DeformGroup,
    float DeformationTriggerRatio,
    bool Optional);

/// <summary>Default values for <see cref="BeamProperties"/>.</summary>
public static class BeamDefaults
{
    /// <summary>Gets the baseline beam properties.</summary>
    public static BeamProperties Default => new(
        Spring: 4000000f,
        Damp: 200f,
        Deform: 220000f,
        Strength: float.MaxValue,
        BeamType: "|NORMAL",
        DeformGroup: "",
        DeformationTriggerRatio: 0f,
        Optional: false);
}

/// <summary>A beam (spring/damper) connecting two nodes.</summary>
/// <param name="Id1">Identifier of the first node.</param>
/// <param name="Id2">Identifier of the second node.</param>
/// <param name="Properties">Resolved beam properties.</param>
public record JBeamBeam(
    string Id1,
    string Id2,
    BeamProperties Properties);

/// <summary>Associates a rendered object with the driving node collections.</summary>
/// <param name="Mesh">Mesh object name from the source model.</param>
/// <param name="Groups">Node groups that drive the flexbody.</param>
/// <param name="Position">Optional flexbody position offset in BeamNG space.</param>
/// <param name="Rotation">Optional flexbody rotation offset in BeamNG space.</param>
/// <param name="Scale">Optional flexbody scale override.</param>
public record JBeamFlexBody(
    string Mesh,
    List<string> Groups,
    System.Numerics.Vector3? Position = null,
    System.Numerics.Vector3? Rotation = null,
    System.Numerics.Vector3? Scale = null);

/// <summary>Describes a BeamNG slot, its filler part, and its display text.</summary>
/// <param name="Type">Slot type identifier.</param>
/// <param name="Default">Default part name that fills the slot.</param>
/// <param name="Description">Human-readable slot description.</param>
/// <param name="CoreSlot">Whether the slot must be filled for the vehicle to function.</param>
/// <param name="NodeOffset">Optional node offset applied to the installed part.</param>
public record JBeamSlot(
    string Type,
    string Default,
    string Description,
    bool CoreSlot,
    System.Numerics.Vector3? NodeOffset = null);

/// <summary>Represents a parsed BeamNG powertrain device and its upstream connection metadata.</summary>
/// <param name="Type">BeamNG powertrain device type.</param>
/// <param name="Name">Device name used for graph lookups.</param>
/// <param name="InputName">Upstream device identifier feeding this one.</param>
/// <param name="InputIndex">Input slot index used by the device.</param>
/// <param name="ConnectedWheel">Wheel code connected to the device, when applicable.</param>
/// <param name="GearRatio">Optional gear ratio contributed by the device.</param>
/// <param name="DiffType">Differential subtype token reported by BeamNG.</param>
/// <param name="LsdPreload">Optional LSD preload torque.</param>
/// <param name="LsdLockCoef">Optional locking coefficient under power.</param>
/// <param name="LsdRevLockCoef">Optional locking coefficient on coast.</param>
public record JBeamPowertrainDevice(
    string Type,
    string Name,
    string InputName,
    int InputIndex,
    string ConnectedWheel,
    float? GearRatio,
    string DiffType,
    float? LsdPreload,
    float? LsdLockCoef,
    float? LsdRevLockCoef);

/// <summary>Properties for a pressure-based wheel (BeamNG-style wheel/tyre combo).</summary>
/// <param name="Name">Internal name of the wheel.</param>
/// <param name="WheelKey">Lookup key for the wheel.</param>
/// <param name="HubGroup">Node <paramref name="Group" /> for the hub.</param>
/// <param name="Group">Node group for the tyre.</param>
/// <param name="Node1">Primary node ID.</param>
/// <param name="Node2">Secondary node ID.</param>
/// <param name="NodeArm">Arm node ID.</param>
/// <param name="WheelDir">Wheel direction (1.0 or -1.0).</param>
/// <param name="SteerAxisUp">Upper steering axis node ID.</param>
/// <param name="SteerAxisDown">Lower steering axis node ID.</param>
public record JBeamPressureWheel(
    string Name,
    string WheelKey,
    string HubGroup,
    string Group,
    string Node1,
    string Node2,
    string NodeArm,
    float WheelDir,
    string SteerAxisUp = "",
    string SteerAxisDown = "");

/// <summary>Options for pressure-based wheels, influence tyre physics and dimensions.</summary>
public class JBeamPressureWheelOptions
{
    /// <summary>Whether the wheel has a tyre attached.</summary>
    public bool? HasTire { get; init; }

    /// <summary>Tyre radius in metres.</summary>
    public float? Radius { get; init; }

    /// <summary>Tyre width in metres.</summary>
    public float? TireWidth { get; init; }

    /// <summary>Tyre inflation pressure in PSI.</summary>
    public float? PressurePsi { get; init; }

    /// <summary>Friction coefficient multiplier.</summary>
    public float? FrictionCoef { get; init; }

    /// <summary>Sliding friction coefficient multiplier.</summary>
    public float? SlidingFrictionCoef { get; init; }

    /// <summary>Tread pattern influence coefficient.</summary>
    public float? TreadCoef { get; init; }

    /// <summary>Zero-load friction coefficient.</summary>
    public float? NoLoadCoef { get; init; }

    /// <summary>Load-sensitivity slope for friction.</summary>
    public float? LoadSensitivitySlope { get; init; }

    /// <summary>Full-load asymptotic friction coefficient.</summary>
    public float? FullLoadCoef { get; init; }

    /// <summary>Tyre softness coefficient.</summary>
    public float? SoftnessCoef { get; init; }

    /// <summary>Hub radius in metres.</summary>
    public float? HubRadius { get; init; }

    /// <summary>Hub width in metres.</summary>
    public float? HubWidth { get; init; }

    /// <summary>Wheel lateral offset.</summary>
    public float? WheelOffset { get; init; }
}

/// <summary>Definition for a tuneable variable in a jbeam part.</summary>
public sealed class JBeamVariableDefinition
{
    /// <summary>Variable name (e.g., "$spring").</summary>
    public string Name { get; init; } = "";

    /// <summary>Data type.</summary>
    public string Type { get; init; } = "";

    /// <summary>Display unit.</summary>
    public string Unit { get; init; } = "";

    /// <summary>Category for grouping in the UI.</summary>
    public string Category { get; init; } = "";

    /// <summary>Default value.</summary>
    public float DefaultValue { get; init; }

    /// <summary>Minimum value allowed.</summary>
    public float MinValue { get; init; }

    /// <summary>Maximum value allowed.</summary>
    public float MaxValue { get; init; }

    /// <summary>Display title.</summary>
    public string Title { get; init; } = "";

    /// <summary>Detailed description.</summary>
    public string Description { get; init; } = "";

    /// <summary>Subcategory for UI grouping.</summary>
    public string SubCategory { get; init; } = "";

    /// <summary>Minimum display value for the UI slider.</summary>
    public float? MinDisplayValue { get; init; }

    /// <summary>Maximum display value for the UI slider.</summary>
    public float? MaxDisplayValue { get; init; }

    /// <summary>Step size for the UI slider.</summary>
    public float? StepDisplayValue { get; init; }
}

/// <summary>A point on a <paramref name="Torque" /> curve.</summary>
/// <param name="Rpm">Engine RPM.</param>
/// <param name="Torque">Torque in Nm.</param>
public record JBeamTorquePoint(float Rpm, float Torque);

/// <summary>Definition of an internal combustion engine's physical and thermal properties.</summary>
public class JBeamEngineDefinition
{
    /// <summary>Torque curve mapping RPM to peak torque.</summary>
    public List<JBeamTorquePoint> TorqueCurve { get; init; } = new();

    /// <summary>Target idle RPM.</summary>
    public float IdleRpm { get; init; }

    /// <summary>Maximum safe engine RPM.</summary>
    public float MaxRpm { get; init; }

    /// <summary>Rotational inertia of the engine block and flywheel (kg·m²).</summary>
    public float Inertia { get; init; }

    /// <summary>Static engine friction (Nm).</summary>
    public float Friction { get; init; }

    /// <summary>Dynamic engine friction (Nm/(rad/s)).</summary>
    public float DynamicFriction { get; init; }

    /// <summary>Additional torque applied during engine braking.</summary>
    public float EngineBrakeTorque { get; init; }

    /// <summary>Whether thermal simulation is enabled for this engine.</summary>
    public bool ThermalsEnabled { get; init; }

    /// <summary>Total oil volume in litres.</summary>
    public float OilVolume { get; init; }

    /// <summary>Engine block material (e.g., "iron", "aluminum").</summary>
    public string EngineBlockMaterial { get; init; } = "iron";

    /// <summary>Temperature at which the engine block begins taking damage (°C).</summary>
    public float EngineBlockTemperatureDamageThreshold { get; init; }

    /// <summary>Temperature at which the cylinder walls begin taking damage (°C).</summary>
    public float CylinderWallTemperatureDamageThreshold { get; init; }

    /// <summary>Burn efficiency curve mapping throttle to thermal efficiency.</summary>
    public List<JBeamBurnEfficiencyPoint> BurnEfficiency { get; init; } = new();

    /// <summary>Whether the engine is strictly air-cooled.</summary>
    public bool IsAirCooledOnly { get; init; }

    /// <summary>Target coolant temperature regulated by air flow (°C).</summary>
    public float AirRegulatorTemperature { get; init; }

    /// <summary>Efficiency of air cooling for the engine block.</summary>
    public float EngineBlockAirCoolingEfficiency { get; init; }

    /// <summary>Turbocharger definition if present.</summary>
    public JBeamTurboDefinition? Turbo { get; init; }
}

/// <summary>Represents one point on the engine burn curve.</summary>
/// <param name="Throttle">Throttle fraction in the range 0-1.</param>
/// <param name="Efficiency">Resulting thermal conversion value for the point.</param>
public record JBeamBurnEfficiencyPoint(float Throttle, float Efficiency);

/// <summary>Turbocharger definition parsed from a turbocharger section.</summary>
public class JBeamTurboDefinition
{
    /// <summary>Wastegate pressure in PSI.</summary>
    public float WastegatePressure { get; init; }

    /// <summary>Turbo friction coefficient.</summary>
    public float FrictionCoef { get; init; }

    /// <summary>Turbo rotational inertia.</summary>
    public float Inertia { get; init; }

    /// <summary>Input gas expansion ratio.</summary>
    public float GammaIn { get; init; }

    /// <summary>Output gas expansion ratio.</summary>
    public float GammaOut { get; init; }

    /// <summary>Pressure rate in PSI per unit of flow.</summary>
    public float PressureRatePsi { get; init; }
}

/// <summary>Fuel tank / energy storage parsed from an energyStorage section and its named block.</summary>
public class JBeamFuelTankDefinition
{
    /// <summary>Type of energy stored (e.g., "gasoline").</summary>
    public string EnergyType { get; init; } = "gasoline";

    /// <summary>Total fuel capacity in litres.</summary>
    public float FuelCapacity { get; init; }

    /// <summary>Starting fuel amount in litres.</summary>
    public float StartingFuelCapacity { get; init; }
}

/// <summary>Definition for a manual or automatic gearbox.</summary>
public class JBeamGearboxDefinition
{
    /// <summary>List of forward gear ratios.</summary>
    public List<float> GearRatios { get; init; } = new();
}

/// <summary>Definition for shift logic and clutch control parameters.</summary>
public class JBeamVehicleControllerDefinition
{
    /// <summary>RPM at which the clutch begins to engage during launch.</summary>
    public float? ClutchLaunchStartRpm { get; init; }

    /// <summary>Target RPM to maintain during a clutch launch.</summary>
    public float? ClutchLaunchTargetRpm { get; init; }

    /// <summary>Upshift RPM for high-performance driving.</summary>
    public float? HighShiftUpRpm { get; init; }

    /// <summary>Downshift RPMs for each gear in high-performance mode.</summary>
    public List<float> HighShiftDownRpm { get; init; } = new();

    /// <summary>Upshift RPMs for each gear in economy mode.</summary>
    public List<float> LowShiftUpRpm { get; init; } = new();

    /// <summary>Downshift RPMs for each gear in economy mode.</summary>
    public List<float> LowShiftDownRpm { get; init; } = new();
}

/// <summary>Definition for ABS and brake distribution logic.</summary>
public class JBeamBrakeControlDefinition
{
    /// <summary>Whether ABS is enabled.</summary>
    public bool? EnableAbs { get; init; }

    /// <summary>Target slip ratio for the ABS controller.</summary>
    public float? AbsSlipRatioTarget { get; init; }

    /// <summary>Whether the vehicle uses a legacy ABS controller implementation.</summary>
    public bool HasLegacyAbsController { get; init; }
}

/// <summary>Definition for traction control logic.</summary>
public class JBeamTractionControlDefinition
{
    /// <summary>Whether traction control is enabled.</summary>
    public bool? EnableTractionControl { get; init; }

    /// <summary>Slip ratio threshold for TC intervention.</summary>
    public float? SlipThreshold { get; init; }

    /// <summary>Range of slip above the threshold for progressive intervention.</summary>
    public float? SlipRangeThreshold { get; init; }

    /// <summary>Maximum velocity where TC remains active.</summary>
    public float? MaxVelocity { get; init; }
}

/// <summary>All raw data parsed from a single .jbeam section (one named object within a file).</summary>
public class JBeamPart
{
    /// <summary>Internal name of the part.</summary>
    public string Name { get; init; } = "";

    /// <summary>Type of slot this part occupies.</summary>
    public string SlotType { get; init; } = "";

    /// <summary>Skin token used for material variants when the part declares one explicitly.</summary>
    public string SkinName { get; init; } = "";

    /// <summary>Global skin token applied by the part when BeamNG declares one.</summary>
    public string GlobalSkin { get; init; } = "";

    /// <summary>The material variant token contributed by this part, preferring <see cref="SkinName"/>.</summary>
    public string ActiveMaterialVariantName => !string.IsNullOrWhiteSpace(SkinName)
        ? SkinName
        : GlobalSkin;

    /// <summary>Gets whether this is the vehicle's root "main" part.</summary>
    public bool IsMain => SlotType == "main";

    /// <summary>List of slots defined by this part.</summary>
    public List<JBeamSlot> Slots { get; init; } = new();

    /// <summary>List of nodes defined by this part.</summary>
    public List<JBeamNode> Nodes { get; init; } = new();

    /// <summary>List of beams defined by this part.</summary>
    public List<JBeamBeam> Beams { get; init; } = new();

    /// <summary>List of flexbodies (meshes) attached to this part's node groups.</summary>
    public List<JBeamFlexBody> FlexBodies { get; init; } = new();

    /// <summary>Map of variable names to their values for this part.</summary>
    public Dictionary<string, float> Variables { get; init; } = new(System.StringComparer.OrdinalIgnoreCase);

    /// <summary>Definitions for variables that can be tuned for this part.</summary>
    public List<JBeamVariableDefinition> VariableDefinitions { get; init; } = new();

    /// <summary>List of powertrain devices (engines, diffs) defined by this part.</summary>
    public List<JBeamPowertrainDevice> PowertrainDevices { get; init; } = new();

    /// <summary>List of pressure wheels defined by this part.</summary>
    public List<JBeamPressureWheel> PressureWheels { get; init; } = new();

    /// <summary>Default pressure wheel options for wheels in this part.</summary>
    public JBeamPressureWheelOptions? PressureWheelOptions { get; init; }

    /// <summary>Specific gear ratio provided by this part (e.g., final drive).</summary>
    public float? GearRatio { get; init; }

    /// <summary>Engine definition if this part is an engine.</summary>
    public JBeamEngineDefinition? Engine { get; init; }

    /// <summary>Gearbox definition if this part is a gearbox.</summary>
    public JBeamGearboxDefinition? Gearbox { get; init; }

    /// <summary>Vehicle controller settings if this part provides them.</summary>
    public JBeamVehicleControllerDefinition? VehicleController { get; init; }

    /// <summary>Brake control settings if this part provides them.</summary>
    public JBeamBrakeControlDefinition? BrakeControl { get; init; }

    /// <summary>Traction control settings if this part provides them.</summary>
    public JBeamTractionControlDefinition? TractionControl { get; init; }

    /// <summary>Fuel tank definition if this part is a fuel tank.</summary>
    public JBeamFuelTankDefinition? FuelTank { get; init; }

    /// <summary>Ref nodes: ref, back, left, up positions used for orientation.</summary>
    public Dictionary<string, string> RefNodes { get; init; } = new();
}
