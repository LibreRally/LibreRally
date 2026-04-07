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
    System.Numerics.Vector3? Position = null);

/// <summary>A slot definition: type, default filler, description.</summary>
public record JBeamSlot(
    string Type,
    string Default,
    string Description,
    bool CoreSlot);

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

    /// <summary>Ref nodes: ref, back, left, up positions used for orientation.</summary>
    public Dictionary<string, string> RefNodes { get; init; } = new();
}
