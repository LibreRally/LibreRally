using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;

namespace LibreRally.Vehicle.JBeam;

/// <summary>
/// Loads all .jbeam files from a vehicle folder, resolves the slot hierarchy
/// starting from the "main" slot, and produces a flat <see cref="VehicleDefinition"/>.
///
/// BeamNG jbeam coordinate space: X = right, Y = forward, Z = up.
/// We convert to Stride space (X = right, Y = up, Z = backward) in
/// <see cref="VehiclePhysicsBuilder"/> rather than here, to preserve raw data fidelity.
/// </summary>
public static class JBeamAssembler
{
    /// <summary>Loads a vehicle from a folder and all sub-folders (for RLA_Evo Jbeams/ layout).</summary>
    public static VehicleDefinition Assemble(string vehicleFolder)
    {
        // Collect all jbeam files in the folder (and one level down for Jbeams/ subdirectory)
        var jbeamFiles = Directory
            .EnumerateFiles(vehicleFolder, "*.jbeam", SearchOption.AllDirectories)
            .ToList();

        // Parse all files → flat dictionary: partName → JBeamPart
        var partLibrary = new Dictionary<string, JBeamPart>(StringComparer.OrdinalIgnoreCase);
        foreach (string file in jbeamFiles)
        {
            List<JBeamPart> parts;
            try { parts = JBeamParser.ParseFile(file); }
            catch (Exception ex)
            {
                // Log but continue — a single bad file shouldn't prevent loading
                Console.Error.WriteLine($"[JBeamAssembler] Failed to parse '{file}': {ex.Message}");
                continue;
            }

            foreach (var p in parts)
            {
                if (!partLibrary.ContainsKey(p.Name))
                    partLibrary[p.Name] = p;
            }
        }

        // Find the main part (slotType == "main")
        JBeamPart? mainPart = partLibrary.Values.FirstOrDefault(p => p.IsMain);
        if (mainPart == null)
            throw new InvalidOperationException($"No jbeam part with slotType 'main' found in '{vehicleFolder}'.");

        // Resolve slot hierarchy depth-first, collecting all active parts in order
        var activeParts = new List<JBeamPart>();
        ResolveSlots(mainPart, partLibrary, activeParts, new HashSet<string>(StringComparer.OrdinalIgnoreCase));

        // Merge into VehicleDefinition
        return Merge(mainPart.Name, vehicleFolder, activeParts);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Slot resolution
    // ──────────────────────────────────────────────────────────────────────────

    private static void ResolveSlots(
        JBeamPart part,
        Dictionary<string, JBeamPart> library,
        List<JBeamPart> result,
        HashSet<string> visited)
    {
        if (!visited.Add(part.Name)) return;
        result.Add(part);

        foreach (var slot in part.Slots)
        {
            if (string.IsNullOrEmpty(slot.Default)) continue;

            if (library.TryGetValue(slot.Default, out var child))
                ResolveSlots(child, library, result, visited);
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Merge
    // ──────────────────────────────────────────────────────────────────────────

    private static VehicleDefinition Merge(
        string vehicleName,
        string folderPath,
        List<JBeamPart> parts)
    {
        var allNodes = new Dictionary<string, AssembledNode>(StringComparer.OrdinalIgnoreCase);
        var allBeams = new List<AssembledBeam>();
        var allFlexBodies = new List<AssembledFlexBody>();

        // Build a map: slotType → part name, for identifying detachable vs chassis
        var slotTypeToPartName = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (var p in parts)
            if (!string.IsNullOrEmpty(p.SlotType))
                slotTypeToPartName[p.SlotType] = p.Name;

        // Merge nodes and beams from all resolved parts
        var nodeToPartName = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

        foreach (var part in parts)
        {
            foreach (var node in part.Nodes)
            {
                if (allNodes.ContainsKey(node.Id)) continue;
                allNodes[node.Id] = new AssembledNode(
                    node.Id,
                    node.Position,
                    node.Properties.Weight,
                    node.Properties.Groups,
                    node.Properties.Collision);
                nodeToPartName[node.Id] = part.Name;
            }

            foreach (var beam in part.Beams)
                allBeams.Add(new AssembledBeam(
                    beam.Id1, beam.Id2,
                    beam.Properties.Spring,
                    beam.Properties.Damp,
                    beam.Properties.Deform,
                    beam.Properties.Strength,
                    beam.Properties.BeamType,
                    beam.Properties.DeformGroup));

            foreach (var fb in part.FlexBodies)
                allFlexBodies.Add(new AssembledFlexBody(fb.Mesh, fb.Groups, fb.Position));
        }

        // Build logical parts:
        // - Part 0 = chassis (all nodes not exclusively owned by a detachable slot)
        // - Parts 1..N = detachable pieces identified by slot type patterns
        var logicalParts = BuildLogicalParts(parts, allNodes, allBeams, nodeToPartName, allFlexBodies);

        var refNodes = parts
            .Where(p => p.RefNodes.Count > 0)
            .Select(p => p.RefNodes)
            .FirstOrDefault() ?? new Dictionary<string, string>();

        return new VehicleDefinition
        {
            VehicleName = vehicleName,
            Nodes = allNodes,
            Beams = allBeams,
            FlexBodies = allFlexBodies,
            Parts = logicalParts,
            FolderPath = folderPath,
            RefNodes = refNodes,
        };
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Logical part identification
    // ──────────────────────────────────────────────────────────────────────────

    // Slot type patterns that map to detachable body parts
    private static readonly HashSet<string> DetachableSlotPatterns = new(StringComparer.OrdinalIgnoreCase)
    {
        "hood", "trunk", "door", "bumper", "fender", "quarterpanel",
        "sideskirt", "spoiler", "grille", "headlight", "taillight",
        "glass", "windshield", "backlight", "mirror", "roof"
    };

    private static List<VehiclePart> BuildLogicalParts(
        List<JBeamPart> activeParts,
        Dictionary<string, AssembledNode> allNodes,
        List<AssembledBeam> allBeams,
        Dictionary<string, string> nodeToPartName,
        List<AssembledFlexBody> allFlexBodies)
    {
        var result = new List<VehiclePart>();

        // Identify detachable parts based on slot type name containing a detachable keyword
        var detachablePartNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
        foreach (var part in activeParts)
        {
            if (IsDetachable(part.SlotType))
                detachablePartNames.Add(part.Name);
        }

        // For each detachable part: find its exclusive nodes and break strength
        var usedNodeIds = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        foreach (var part in activeParts)
        {
            if (!detachablePartNames.Contains(part.Name)) continue;

            var exclusiveNodes = part.Nodes
                .Select(n => n.Id)
                .ToList();

            foreach (var id in exclusiveNodes)
                usedNodeIds.Add(id);

            // Break strength = min beamStrength of beams in any deformGroup that crosses
            // from this part's nodes to chassis nodes
            float breakStrength = ComputeBreakStrength(
                exclusiveNodes,
                allBeams,
                nodeToPartName,
                part.Name);

            // Flex bodies for this part
            var exclusiveNodeSet = new HashSet<string>(exclusiveNodes, StringComparer.OrdinalIgnoreCase);
            var partGroups = part.Nodes
                .SelectMany(n => n.Properties.Groups)
                .Distinct(StringComparer.OrdinalIgnoreCase)
                .ToHashSet(StringComparer.OrdinalIgnoreCase);

            var partFlexBodies = allFlexBodies
                .Where(fb => fb.NodeGroups.Any(g => partGroups.Contains(g)))
                .ToList();

            result.Add(new VehiclePart
            {
                Name = part.Name,
                Detachable = true,
                ExclusiveNodeIds = exclusiveNodes,
                BreakStrength = breakStrength,
                FlexBodies = partFlexBodies,
            });
        }

        // Chassis = everything not owned by a detachable part
        var chassisNodes = allNodes.Keys
            .Where(id => !usedNodeIds.Contains(id))
            .ToList();

        var chassisGroups = chassisNodes
            .SelectMany(id => allNodes[id].Groups)
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToHashSet(StringComparer.OrdinalIgnoreCase);

        var chassisFlexBodies = allFlexBodies
            .Where(fb => fb.NodeGroups.Any(g => chassisGroups.Contains(g)))
            .ToList();

        var chassis = new VehiclePart
        {
            Name = "chassis",
            Detachable = false,
            ExclusiveNodeIds = chassisNodes,
            BreakStrength = float.MaxValue,
            FlexBodies = chassisFlexBodies,
        };

        // Chassis goes first
        result.Insert(0, chassis);
        return result;
    }

    private static bool IsDetachable(string slotType)
    {
        if (string.IsNullOrEmpty(slotType)) return false;
        foreach (string pattern in DetachableSlotPatterns)
        {
            if (slotType.Contains(pattern, StringComparison.OrdinalIgnoreCase))
                return true;
        }
        return false;
    }

    private static float ComputeBreakStrength(
        List<string> partNodeIds,
        List<AssembledBeam> allBeams,
        Dictionary<string, string> nodeToPartName,
        string partName)
    {
        var partSet = new HashSet<string>(partNodeIds, StringComparer.OrdinalIgnoreCase);
        float minStrength = float.MaxValue;

        foreach (var beam in allBeams)
        {
            bool id1InPart = partSet.Contains(beam.Id1);
            bool id2InPart = partSet.Contains(beam.Id2);

            // A beam that crosses the boundary (one node in, one node out)
            if (id1InPart ^ id2InPart)
            {
                if (beam.BreakStrength < minStrength)
                    minStrength = beam.BreakStrength;
            }
        }

        // Use a sensible default if no cross-boundary beam was found
        return minStrength == float.MaxValue ? 50000f : minStrength;
    }
}
