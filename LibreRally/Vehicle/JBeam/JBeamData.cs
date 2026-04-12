using System.Collections.Generic;
using System.Numerics;

namespace LibreRally.Vehicle.JBeam;

/// <summary>Raw per-node properties accumulated from preceding property-setter objects.</summary>
public record NodeProperties(
    float Weight,
    bool Collision,
    bool SelfCollision,
    string Material,
    float FrictionCoef,
    List<string> Groups);

public static class NodeDefaults
{
    public static NodeProperties Default => new(
        Weight: 25f,
        Collision: true,
        SelfCollision: true,
        Material: "|NM_METAL",
        FrictionCoef: 0.5f,
        Groups: new List<string>());
}

/// <summary>A single node (point mass) in the jbeam graph.</summary>
public record JBeamNode(
    string Id,
    Vector3 Position,
    NodeProperties Properties);

/// <summary>Raw per-beam properties accumulated from preceding property-setter objects.</summary>
public record BeamProperties(
    float Spring,
    float Damp,
    float Deform,
    float Strength,
    string BeamType,
    string DeformGroup,
    float DeformationTriggerRatio,
    bool Optional);

public static class BeamDefaults
{
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
public record JBeamBeam(
    string Id1,
    string Id2,
    BeamProperties Properties);

/// <summary>Maps a mesh object name to one or more node groups that drive it.</summary>
public record JBeamFlexBody(
    string Mesh,
    List<string> Groups,
    System.Numerics.Vector3? Position = null,
    System.Numerics.Vector3? Rotation = null,
    System.Numerics.Vector3? Scale = null);

/// <summary>A slot definition: type, default filler, description.</summary>
public record JBeamSlot(
    string Type,
    string Default,
    string Description,
    bool CoreSlot,
    System.Numerics.Vector3? NodeOffset = null);

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

public record JBeamTorquePoint(float Rpm, float Torque);

public class JBeamEngineDefinition
{
    public List<JBeamTorquePoint> TorqueCurve { get; init; } = new();
    public float IdleRpm { get; init; }
    public float MaxRpm { get; init; }
    public float Inertia { get; init; }
    public float Friction { get; init; }
    public float DynamicFriction { get; init; }
    public float EngineBrakeTorque { get; init; }
}

public class JBeamGearboxDefinition
{
    public List<float> GearRatios { get; init; } = new();
}

public class JBeamVehicleControllerDefinition
{
    public float? ClutchLaunchStartRpm { get; init; }
    public float? ClutchLaunchTargetRpm { get; init; }
    public float? HighShiftUpRpm { get; init; }
    public List<float> HighShiftDownRpm { get; init; } = new();
    public List<float> LowShiftUpRpm { get; init; } = new();
    public List<float> LowShiftDownRpm { get; init; } = new();
}

/// <summary>All raw data parsed from a single .jbeam section (one named object within a file).</summary>
public class JBeamPart
{
    public string Name { get; init; } = "";
    public string SlotType { get; init; } = "";
    public bool IsMain => SlotType == "main";

    public List<JBeamSlot> Slots { get; init; } = new();
    public List<JBeamNode> Nodes { get; init; } = new();
    public List<JBeamBeam> Beams { get; init; } = new();
    public List<JBeamFlexBody> FlexBodies { get; init; } = new();
    public Dictionary<string, float> Variables { get; init; } = new(System.StringComparer.OrdinalIgnoreCase);
    public List<JBeamPowertrainDevice> PowertrainDevices { get; init; } = new();
    public JBeamEngineDefinition? Engine { get; init; }
    public JBeamGearboxDefinition? Gearbox { get; init; }
    public JBeamVehicleControllerDefinition? VehicleController { get; init; }

    /// <summary>Ref nodes: ref, back, left, up positions used for orientation.</summary>
    public Dictionary<string, string> RefNodes { get; init; } = new();
}
