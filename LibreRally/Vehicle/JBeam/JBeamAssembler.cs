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
    private sealed record ResolvedPartInstance(JBeamPart Part, Vector3 VisualOffset);

    /// <summary>Loads a vehicle from a folder and all sub-folders (for RLA_Evo Jbeams/ layout).</summary>
    public static VehicleDefinition Assemble(string vehicleFolder, PcConfig? pcConfig = null)
    {
        return Assemble([vehicleFolder], vehicleFolder, pcConfig);
    }

    /// <summary>
    /// Loads a vehicle from one or more search folders, using <paramref name="vehicleFolder"/>
    /// as the primary folder for diagnostics and config lookup.
    /// </summary>
    public static VehicleDefinition Assemble(
        IEnumerable<string> searchFolders,
        string vehicleFolder,
        PcConfig? pcConfig = null)
    {
        var allSearchFolders = searchFolders
            .Append(vehicleFolder)
            .Where(Directory.Exists)
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToList();

        // Collect all jbeam files across the active search folders.
        var jbeamFiles = allSearchFolders
            .SelectMany(folder => Directory.EnumerateFiles(folder, "*.jbeam", SearchOption.AllDirectories))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .OrderBy(path => path, StringComparer.OrdinalIgnoreCase)
            .ToList();

        var vars = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);
        foreach (var file in jbeamFiles)
        {
            try
            {
                vars = JBeamParser.ParseVariableDefaultsFile(file, vars);
            }
            catch (Exception ex)
            {
                Console.Error.WriteLine($"[JBeamAssembler] Failed to parse variable defaults from '{file}': {ex.Message}");
            }
        }

        if (pcConfig?.Vars != null)
        {
            foreach (var kv in pcConfig.Vars)
            {
                vars[kv.Key] = kv.Value;
            }
        }

        // Parse all files → flat dictionary: partName → JBeamPart
        var partLibrary = new Dictionary<string, JBeamPart>(StringComparer.OrdinalIgnoreCase);
        foreach (var file in jbeamFiles)
        {
            List<JBeamPart> parts;
            try { parts = JBeamParser.ParseFile(file, vars); }
            catch (Exception ex)
            {
                // Log but continue — a single bad file shouldn't prevent loading
                Console.Error.WriteLine($"[JBeamAssembler] Failed to parse '{file}': {ex.Message}");
                continue;
            }

            foreach (var p in parts)
            {
                if (!partLibrary.ContainsKey(p.Name))
                {
	                partLibrary[p.Name] = p;
                }
            }
        }

        // Find the main part (slotType == "main")
        JBeamPart? mainPart = null;
        if (pcConfig != null && !string.IsNullOrEmpty(pcConfig.MainPartName))
        {
	        partLibrary.TryGetValue(pcConfig.MainPartName, out mainPart);
        }

        mainPart ??= partLibrary.Values.FirstOrDefault(p => p.IsMain);
        if (mainPart == null)
        {
	        throw new InvalidOperationException($"No jbeam part with slotType 'main' found in '{vehicleFolder}'.");
        }

        // Resolve slot hierarchy depth-first, collecting all active parts in order
        var activeParts = new List<ResolvedPartInstance>();
        ResolveSlots(mainPart, partLibrary, activeParts,
            new HashSet<string>(StringComparer.OrdinalIgnoreCase),
            pcConfig?.Parts,
            Vector3.Zero);

        // Merge into VehicleDefinition
        var def = Merge(mainPart.Name, vehicleFolder, activeParts);
        // Attach vars so physics builder can use them
        if (vars != null)
        {
	        foreach (var kv in vars)
		        def.Vars[kv.Key] = kv.Value;
        }

        return def;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Slot resolution
    // ──────────────────────────────────────────────────────────────────────────

    private static void ResolveSlots(
        JBeamPart part,
        Dictionary<string, JBeamPart> library,
        List<ResolvedPartInstance> result,
        HashSet<string> visited,
        Dictionary<string, string>? pcParts = null,
        Vector3 visualOffset = default)
    {
        if (!visited.Add(part.Name))
        {
	        return;
        }

        result.Add(new ResolvedPartInstance(part, visualOffset));

        foreach (var slot in part.Slots)
        {
            var partName = slot.Default;
            if (pcParts != null && pcParts.TryGetValue(slot.Type, out var pcOverride))
            {
	            partName = string.IsNullOrWhiteSpace(pcOverride)
                    ? slot.Default
                    : pcOverride;
            }

            if (string.IsNullOrEmpty(partName))
            {
	            continue;
            }

            if (library.TryGetValue(partName, out var child))
            {
	            ResolveSlots(
		            child,
		            library,
		            result,
		            visited,
		            pcParts,
		            visualOffset + (slot.NodeOffset ?? Vector3.Zero));
            }
            // Missing parts (e.g. common.zip parts not shipped with the mod) are silently ignored.
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Merge
    // ──────────────────────────────────────────────────────────────────────────

    private static VehicleDefinition Merge(
        string vehicleName,
        string folderPath,
        List<ResolvedPartInstance> parts)
    {
        var allNodes = new Dictionary<string, AssembledNode>(StringComparer.OrdinalIgnoreCase);
        var allBeams = new List<AssembledBeam>();
        var allFlexBodies = new List<AssembledFlexBody>();
        var allPowertrainDevices = new List<JBeamPowertrainDevice>();
        var allPressureWheels = new List<JBeamPressureWheel>();
        JBeamEngineDefinition? engine = null;
        JBeamGearboxDefinition? gearbox = null;
        JBeamVehicleControllerDefinition? controller = null;
        JBeamBrakeControlDefinition? brakeControl = null;
        var resolvedParts = parts.Select(p => p.Part).ToList();

        // Build a map: slotType → part name, for identifying detachable vs chassis
        var slotTypeToPartName = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (var p in resolvedParts)
            if (!string.IsNullOrEmpty(p.SlotType))
            {
	            slotTypeToPartName[p.SlotType] = p.Name;
            }

        // Merge nodes and beams from all resolved parts
        var nodeToPartName = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

        foreach (var resolved in parts)
        {
            var part = resolved.Part;
            foreach (var node in part.Nodes)
            {
                if (allNodes.ContainsKey(node.Id))
                {
	                continue;
                }

                allNodes[node.Id] = new AssembledNode(
                    node.Id,
                    ApplySlotOffset(node.Position, resolved.VisualOffset, node.Properties.Groups),
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
                allFlexBodies.Add(new AssembledFlexBody(
                    fb.Mesh,
                    fb.Groups,
                    CombineFlexBodyPosition(fb.Position, resolved.VisualOffset, fb.Groups),
                    fb.Rotation,
                    fb.Scale,
                    part.Name,
                    part.SlotType));

            allPowertrainDevices.AddRange(part.PowertrainDevices);
            allPressureWheels.AddRange(part.PressureWheels);
            engine ??= part.Engine;
            gearbox ??= part.Gearbox;
            controller = MergeVehicleController(controller, part.VehicleController);
            brakeControl = MergeBrakeControl(brakeControl, part.BrakeControl);
        }

        // Build logical parts:
        // - Part 0 = chassis (all nodes not exclusively owned by a detachable slot)
        // - Parts 1..N = detachable pieces identified by slot type patterns
        var logicalParts = BuildLogicalParts(resolvedParts, allNodes, allBeams, nodeToPartName, allFlexBodies);

        var refNodes = resolvedParts
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
            PowertrainDevices = allPowertrainDevices,
            PressureWheels = allPressureWheels,
            Engine = engine,
            Gearbox = gearbox,
            VehicleController = controller,
            BrakeControl = brakeControl,
        };
    }

    private static JBeamVehicleControllerDefinition? MergeVehicleController(
        JBeamVehicleControllerDefinition? current,
        JBeamVehicleControllerDefinition? next)
    {
        if (next == null)
        {
            return current;
        }

        if (current == null)
        {
            return next;
        }

        return new JBeamVehicleControllerDefinition
        {
            ClutchLaunchStartRpm = current.ClutchLaunchStartRpm ?? next.ClutchLaunchStartRpm,
            ClutchLaunchTargetRpm = current.ClutchLaunchTargetRpm ?? next.ClutchLaunchTargetRpm,
            HighShiftUpRpm = current.HighShiftUpRpm ?? next.HighShiftUpRpm,
            HighShiftDownRpm = current.HighShiftDownRpm.Count > 0 ? current.HighShiftDownRpm : next.HighShiftDownRpm,
            LowShiftUpRpm = current.LowShiftUpRpm.Count > 0 ? current.LowShiftUpRpm : next.LowShiftUpRpm,
            LowShiftDownRpm = current.LowShiftDownRpm.Count > 0 ? current.LowShiftDownRpm : next.LowShiftDownRpm,
        };
    }

    private static JBeamBrakeControlDefinition? MergeBrakeControl(
        JBeamBrakeControlDefinition? current,
        JBeamBrakeControlDefinition? next)
    {
        if (next == null)
        {
            return current;
        }

        if (current == null)
        {
            return next;
        }

        return new JBeamBrakeControlDefinition
        {
            EnableAbs = next.EnableAbs ?? current.EnableAbs,
            AbsSlipRatioTarget = next.AbsSlipRatioTarget ?? current.AbsSlipRatioTarget,
            HasLegacyAbsController = current.HasLegacyAbsController || next.HasLegacyAbsController,
        };
    }

    private static Vector3? CombineFlexBodyPosition(Vector3? basePosition, Vector3 slotOffset, IReadOnlyCollection<string> groups)
    {
        if (basePosition == null && slotOffset == Vector3.Zero)
        {
	        return null;
        }

        return ApplySlotOffset(basePosition ?? Vector3.Zero, slotOffset, groups);
    }

    private static Vector3 ApplySlotOffset(Vector3 basePosition, Vector3 slotOffset, IReadOnlyCollection<string> groups)
    {
        if (slotOffset == Vector3.Zero)
        {
            return basePosition;
        }

        var adjustedOffset = slotOffset;
        if (slotOffset.X != 0f && TryResolveLateralSideSign(groups, basePosition.X, out var sideSign))
        {
            adjustedOffset.X *= sideSign;
        }

        return basePosition + adjustedOffset;
    }

    private static bool TryResolveLateralSideSign(IReadOnlyCollection<string> groups, float fallbackX, out float sideSign)
    {
        foreach (var group in groups)
        {
            if (group.EndsWith("_FR", StringComparison.OrdinalIgnoreCase) ||
                group.EndsWith("_RR", StringComparison.OrdinalIgnoreCase))
            {
                sideSign = -1f;
                return true;
            }

            if (group.EndsWith("_FL", StringComparison.OrdinalIgnoreCase) ||
                group.EndsWith("_RL", StringComparison.OrdinalIgnoreCase))
            {
                sideSign = 1f;
                return true;
            }
        }

        if (Math.Abs(fallbackX) > 1e-4f)
        {
            sideSign = MathF.Sign(fallbackX);
            return true;
        }

        sideSign = 0f;
        return false;
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
            {
	            detachablePartNames.Add(part.Name);
            }
        }

        // For each detachable part: find its exclusive nodes and break strength
        var usedNodeIds = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        foreach (var part in activeParts)
        {
            if (!detachablePartNames.Contains(part.Name))
            {
	            continue;
            }

            var exclusiveNodes = part.Nodes
                .Select(n => n.Id)
                .ToList();

            foreach (var id in exclusiveNodes)
                usedNodeIds.Add(id);

            // Break strength = min beamStrength of beams in any deformGroup that crosses
            // from this part's nodes to chassis nodes
            var breakStrength = ComputeBreakStrength(
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
        if (string.IsNullOrEmpty(slotType))
        {
	        return false;
        }

        foreach (var pattern in DetachableSlotPatterns)
        {
            if (slotType.Contains(pattern, StringComparison.OrdinalIgnoreCase))
            {
	            return true;
            }
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
        var minStrength = float.MaxValue;

        foreach (var beam in allBeams)
        {
            var id1InPart = partSet.Contains(beam.Id1);
            var id2InPart = partSet.Contains(beam.Id2);

            // A beam that crosses the boundary (one node in, one node out)
            if (id1InPart ^ id2InPart)
            {
                if (beam.BreakStrength < minStrength)
                {
	                minStrength = beam.BreakStrength;
                }
            }
        }

        // Use a sensible default if no cross-boundary beam was found
        return minStrength == float.MaxValue ? 50000f : minStrength;
    }
}
