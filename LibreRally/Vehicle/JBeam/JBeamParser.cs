using System;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Text.Json;

namespace LibreRally.Vehicle.JBeam;

/// <summary>
/// Parses .jbeam files into a list of <see cref="JBeamPart"/> objects.
///
/// jbeam extends JSON with:
///   - C-style // line comments and /* */ block comments
///   - Optional commas between array/object elements (any adjacent values)
///   - Trailing commas before } or ]
///
/// <see cref="PreprocessJBeam"/> normalises the text to valid JSON before parsing.
/// </summary>
public static class JBeamParser
{
    private static readonly JsonDocumentOptions ParseOptions = new()
    {
        CommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true,
    };

    /// <summary>Per-parse variable substitution table. Set before calling Parse*, cleared after.</summary>
    [ThreadStatic]
    private static Dictionary<string, float>? _vars;

    /// <summary>
    /// Sets the active variable table for the current thread. Any $varName values
    /// encountered during parsing will be resolved from this dictionary.
    /// </summary>
    public static void SetVars(Dictionary<string, float>? vars) => _vars = vars;

    public static List<JBeamPart> ParseFile(string path, Dictionary<string, float>? vars = null)
    {
        _vars = vars;
        var raw = System.IO.File.ReadAllText(path);
        return Parse(raw);
    }

    public static Dictionary<string, float> ParseVariableDefaultsFile(string path, Dictionary<string, float>? inheritedVars = null)
    {
        var previousVars = _vars;
        try
        {
            var raw = System.IO.File.ReadAllText(path);
            return ParseVariableDefaults(raw, inheritedVars);
        }
        finally
        {
            _vars = previousVars;
        }
    }

    public static List<JBeamPart> Parse(string jbeamText)
    {
        // Some jbeam files contain multiple top-level objects — merge them before preprocessing.
        var merged = MergeRootObjects(jbeamText);
        var preprocessed = PreprocessJBeam(merged);

        JsonDocument doc;
        try
        {
            doc = JsonDocument.Parse(preprocessed, ParseOptions);
        }
        catch (JsonException ex)
        {
            throw new FormatException($"Failed to parse jbeam as JSON: {ex.Message}", ex);
        }

        var parts = new List<JBeamPart>();
        if (doc.RootElement.ValueKind != JsonValueKind.Object)
        {
	        return parts;
        }

        foreach (var partProp in doc.RootElement.EnumerateObject())
        {
            if (partProp.Value.ValueKind != JsonValueKind.Object)
            {
	            continue;
            }

            var part = ParsePart(partProp.Name, partProp.Value);
            if (part != null)
            {
	            parts.Add(part);
            }
        }

        return parts;
    }

    private static Dictionary<string, float> ParseVariableDefaults(string jbeamText, Dictionary<string, float>? inheritedVars)
    {
        var merged = MergeRootObjects(jbeamText);
        var preprocessed = PreprocessJBeam(merged);

        using var doc = JsonDocument.Parse(preprocessed, ParseOptions);
        var vars = inheritedVars != null
            ? new Dictionary<string, float>(inheritedVars, StringComparer.OrdinalIgnoreCase)
            : new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);

        var previousVars = _vars;
        try
        {
            _vars = vars;
            if (doc.RootElement.ValueKind != JsonValueKind.Object)
            {
                return vars;
            }

            foreach (var partProp in doc.RootElement.EnumerateObject())
            {
                if (partProp.Value.ValueKind != JsonValueKind.Object)
                {
                    continue;
                }

                ParseVariables(partProp.Value, vars);
            }

            return vars;
        }
        finally
        {
            _vars = previousVars;
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Preprocessing: jbeam → valid JSON
    // ──────────────────────────────────────────────────────────────────────────

    /// <summary>
    /// Converts jbeam pseudo-JSON to valid JSON by:
    /// 1. Stripping // and /* */ comments
    /// 2. Inserting commas between adjacent values that have no separator
    /// </summary>
    private static string PreprocessJBeam(string input)
    {
        var sb = new StringBuilder(input.Length + input.Length / 8);
        var i = 0;
        var afterValue = false; // true when last emitted token was a complete JSON value

        void MaybeComma()
        {
            if (!afterValue)
            {
	            return;
            }

            sb.Append(',');
            afterValue = false;
        }

        while (i < input.Length)
        {
            var c = input[i];

            // ── Whitespace ─────────────────────────────────────────────────
            if (char.IsWhiteSpace(c)) { sb.Append(c); i++; continue; }

            // ── Block comment /* … */ ──────────────────────────────────────
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '*')
            {
                i += 2;
                while (i < input.Length - 1 && !(input[i] == '*' && input[i + 1] == '/'))
                    i++;
                if (i < input.Length - 1)
                {
	                i += 2;
                }

                continue;
            }

            // ── Line comment // ────────────────────────────────────────────
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '/')
            {
                while (i < input.Length && input[i] != '\n') i++;
                if (i < input.Length) { sb.Append('\n'); i++; }
                continue;
            }

            // ── String ─────────────────────────────────────────────────────
            if (c == '"')
            {
                MaybeComma();
                sb.Append(c); i++;
                var esc = false;
                while (i < input.Length)
                {
                    var sc = input[i]; sb.Append(sc); i++;
                    if (esc) { esc = false; continue; }
                    if (sc == '\\') { esc = true; continue; }
                    if (sc == '"')
                    {
	                    break;
                    }
                }
                afterValue = true;
                continue;
            }

            // ── Structural ─────────────────────────────────────────────────
            if (c == '[' || c == '{') { MaybeComma(); sb.Append(c); i++; afterValue = false; continue; }
            if (c == ']' || c == '}') { sb.Append(c); i++; afterValue = true;  continue; }
            // Only emit a comma if there's a preceding value; skip redundant commas (e.g. "val,,key")
            if (c == ',') { if (afterValue) { sb.Append(','); } afterValue = false; i++; continue; }
            if (c == ':')              { sb.Append(c); i++; afterValue = false; continue; }

            // ── Number ─────────────────────────────────────────────────────
            if (char.IsDigit(c) || c == '-')
            {
                MaybeComma();
                while (i < input.Length)
                {
                    var nc = input[i];
                    if (char.IsDigit(nc) || nc == '.' || nc == 'e' || nc == 'E' || nc == '+' || nc == '-')
                    { sb.Append(nc); i++; }
                    else
                    {
	                    break;
                    }
                }
                afterValue = true;
                continue;
            }

            // ── Keyword (true / false / null / identifiers) ────────────────
            if (char.IsLetter(c))
            {
                MaybeComma();
                while (i < input.Length && (char.IsLetterOrDigit(input[i]) || input[i] == '_'))
                { sb.Append(input[i]); i++; }
                afterValue = true;
                continue;
            }

            // ── Fallback ───────────────────────────────────────────────────
            sb.Append(c); i++;
        }

        return sb.ToString();
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Part parsing
    // ──────────────────────────────────────────────────────────────────────────

    private static JBeamPart? ParsePart(string name, JsonElement obj)
    {
        var slotType = "";
        if (obj.TryGetProperty("slotType", out var st))
        {
            if (st.ValueKind == JsonValueKind.String)
            {
	            slotType = st.GetString() ?? "";
            }
            else if (st.ValueKind == JsonValueKind.Array)
            {
	            slotType = string.Join(",", st.EnumerateArray().ToList()
		            .Where(e => e.ValueKind == JsonValueKind.String)
		            .Select(e => e.GetString() ?? "")
		            .Where(s => s.Length > 0));
            }
        }

        var part = new JBeamPart
        {
            Name = name,
            SlotType = slotType,
            Slots = ParseSlots(obj),
            Nodes = ParseNodes(obj),
            Beams = ParseBeams(obj),
            FlexBodies = ParseFlexBodies(obj),
            Variables = ParseVariables(obj),
            PowertrainDevices = ParsePowertrain(obj),
            Engine = ParseEngineDefinition(obj),
            Gearbox = ParseGearboxDefinition(obj),
            VehicleController = ParseVehicleControllerDefinition(obj),
            RefNodes = ParseRefNodes(obj),
        };

        return part;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Slots
    // ──────────────────────────────────────────────────────────────────────────

    private static List<JBeamSlot> ParseSlots(JsonElement obj)
    {
        var slots = new List<JBeamSlot>();

        // Legacy "slots" format: [type, default, description, {options}?]
        if (obj.TryGetProperty("slots", out var arr) && arr.ValueKind == JsonValueKind.Array)
        {
	        ParseSlotsArray(arr, slots, legacy: true);
        }

        // Modern "slots2" format: [name, allowTypes[], denyTypes[], default, description, {options}?]
        if (obj.TryGetProperty("slots2", out var arr2) && arr2.ValueKind == JsonValueKind.Array)
        {
	        ParseSlotsArray(arr2, slots, legacy: false);
        }

        return slots;
    }

    private static void ParseSlotsArray(JsonElement arr, List<JBeamSlot> slots, bool legacy)
    {
        var headerSeen = false;
        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind != JsonValueKind.Array)
            {
	            continue;
            }

            var items = elem.EnumerateArray().ToList();

            if (!headerSeen) { headerSeen = true; continue; }

            // Legacy:  [type(0), default(1), description(2), {options}(3)?]
            // Slots2:  [name(0), allowTypes(1), denyTypes(2), default(3), description(4), {options}(5)?]
            var defaultIdx  = legacy ? 1 : 3;
            var descIdx     = legacy ? 2 : 4;
            var optionsIdx  = legacy ? 3 : 5;

            if (items.Count <= defaultIdx)
            {
	            continue;
            }

            // Skip entries where default is not a string (e.g. empty array or missing)
            var defaultVal = items[defaultIdx].ValueKind == JsonValueKind.String
                ? items[defaultIdx].GetString()
                : null;
            if (string.IsNullOrEmpty(defaultVal))
            {
	            continue;
            }

            var type = items[0].ValueKind == JsonValueKind.String ? items[0].GetString() ?? "" : "";

            var core = false;
            Vector3? nodeOffset = null;
            if (items.Count > optionsIdx && items[optionsIdx].ValueKind == JsonValueKind.Object)
            {
                if (items[optionsIdx].TryGetProperty("coreSlot", out var cs))
                {
	                core = cs.ValueKind == JsonValueKind.True;
                }

                if (items[optionsIdx].TryGetProperty("nodeOffset", out var no) &&
                    no.ValueKind == JsonValueKind.Object)
                {
                    nodeOffset = ParseVector3(no);
                }
            }

            slots.Add(new JBeamSlot(
                Type: type,
                Default: defaultVal,
                Description: items.Count > descIdx && items[descIdx].ValueKind == JsonValueKind.String
                    ? items[descIdx].GetString() ?? ""
                    : "",
                CoreSlot: core,
                NodeOffset: nodeOffset));
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Nodes
    // ──────────────────────────────────────────────────────────────────────────

    private static List<JBeamNode> ParseNodes(JsonElement obj)
    {
        var nodes = new List<JBeamNode>();
        if (!obj.TryGetProperty("nodes", out var arr) || arr.ValueKind != JsonValueKind.Array)
        {
	        return nodes;
        }

        var headerSeen = false;
        var props = NodeDefaults.Default with { Groups = new List<string>() };

        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind == JsonValueKind.Object)
            {
                props = ApplyNodeProps(props, elem);
                continue;
            }

            if (elem.ValueKind == JsonValueKind.Array)
            {
                var items = elem.EnumerateArray().ToList();
                if (!headerSeen)
                {
                    headerSeen = true;
                    continue;
                }
                if (items.Count < 4)
                {
	                continue;
                }

                var id = items[0].ValueKind == JsonValueKind.String ? items[0].GetString() ?? "" : "";
                var x = GetFloat(items[1]);
                var y = GetFloat(items[2]);
                var z = GetFloat(items[3]);

                // Inline property override
                var rowProps = props with { Groups = new List<string>(props.Groups) };
                for (var i = 4; i < items.Count; i++)
                {
                    if (items[i].ValueKind == JsonValueKind.Object)
                    {
	                    rowProps = ApplyNodeProps(rowProps, items[i]);
                    }
                }

                nodes.Add(new JBeamNode(id, new Vector3(x, y, z), rowProps));
            }
        }
        return nodes;
    }

    private static NodeProperties ApplyNodeProps(NodeProperties current, JsonElement obj)
    {
        var weight = current.Weight;
        var collision = current.Collision;
        var selfCollision = current.SelfCollision;
        var material = current.Material;
        var friction = current.FrictionCoef;
        var groups = new List<string>(current.Groups);

        if (obj.TryGetProperty("nodeWeight", out var nw))
        {
	        weight = GetFloat(nw);
        }

        if (obj.TryGetProperty("collision", out var col))
        {
	        collision = col.GetBoolean();
        }

        if (obj.TryGetProperty("selfCollision", out var sc))
        {
	        selfCollision = sc.GetBoolean();
        }

        if (obj.TryGetProperty("nodeMaterial", out var nm))
        {
	        material = SafeGetString(nm).Length > 0 ? SafeGetString(nm) : material;
        }

        if (obj.TryGetProperty("frictionCoef", out var fc))
        {
	        friction = GetFloat(fc);
        }

        if (obj.TryGetProperty("group", out var grp))
        {
            groups = new List<string>();
            if (grp.ValueKind == JsonValueKind.String)
            {
                var s = grp.GetString() ?? "";
                if (s.Length > 0)
                {
	                groups.Add(s);
                }
            }
            else if (grp.ValueKind == JsonValueKind.Array)
            {
                foreach (var g in grp.EnumerateArray())
                {
                    var s = g.GetString() ?? "";
                    if (s.Length > 0)
                    {
	                    groups.Add(s);
                    }
                }
            }
        }

        return new NodeProperties(weight, collision, selfCollision, material, friction, groups);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Beams
    // ──────────────────────────────────────────────────────────────────────────

    private static List<JBeamBeam> ParseBeams(JsonElement obj)
    {
        var beams = new List<JBeamBeam>();
        if (!obj.TryGetProperty("beams", out var arr) || arr.ValueKind != JsonValueKind.Array)
        {
	        return beams;
        }

        var headerSeen = false;
        var props = BeamDefaults.Default;

        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind == JsonValueKind.Object)
            {
                props = ApplyBeamProps(props, elem);
                continue;
            }

            if (elem.ValueKind == JsonValueKind.Array)
            {
                var items = elem.EnumerateArray().ToList();
                if (!headerSeen)
                {
                    headerSeen = true;
                    continue;
                }
                if (items.Count < 2)
                {
	                continue;
                }

                var id1 = items[0].ValueKind == JsonValueKind.String ? items[0].GetString() ?? "" : "";
                var id2 = items[1].ValueKind == JsonValueKind.String ? items[1].GetString() ?? "" : "";
                if (string.IsNullOrEmpty(id1) || string.IsNullOrEmpty(id2))
                {
	                continue;
                }

                var rowProps = props;
                for (var i = 2; i < items.Count; i++)
                {
                    if (items[i].ValueKind == JsonValueKind.Object)
                    {
	                    rowProps = ApplyBeamProps(rowProps, items[i]);
                    }
                }

                beams.Add(new JBeamBeam(id1, id2, rowProps));
            }
        }
        return beams;
    }

    private static BeamProperties ApplyBeamProps(BeamProperties current, JsonElement obj)
    {
        var spring = current.Spring;
        var damp = current.Damp;
        var deform = current.Deform;
        var strength = current.Strength;
        var type = current.BeamType;
        var deformGroup = current.DeformGroup;
        var triggerRatio = current.DeformationTriggerRatio;
        var optional = current.Optional;

        if (obj.TryGetProperty("beamSpring", out var bs))
        {
	        spring = GetFloat(bs);
        }

        if (obj.TryGetProperty("beamDamp", out var bd))
        {
	        damp = GetFloat(bd);
        }

        if (obj.TryGetProperty("beamDeform", out var bdf))
        {
	        deform = GetFloat(bdf);
        }

        if (obj.TryGetProperty("beamStrength", out var bst))
        {
	        strength = GetFloatOrMax(bst);
        }

        if (obj.TryGetProperty("beamType", out var bt))
        {
	        type = SafeGetString(bt).Length > 0 ? SafeGetString(bt) : type;
        }

        if (obj.TryGetProperty("deformGroup", out var dg))
        {
	        deformGroup = SafeGetString(dg);
        }

        if (obj.TryGetProperty("deformationTriggerRatio", out var dtr))
        {
	        triggerRatio = GetFloat(dtr);
        }

        if (obj.TryGetProperty("optional", out var opt))
        {
	        optional = opt.ValueKind == JsonValueKind.True || (opt.ValueKind == JsonValueKind.Number && opt.GetInt32() != 0);
        }

        return new BeamProperties(spring, damp, deform, strength, type, deformGroup, triggerRatio, optional);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // FlexBodies
    // ──────────────────────────────────────────────────────────────────────────

    private static List<JBeamFlexBody> ParseFlexBodies(JsonElement obj)
    {
        var flexBodies = new List<JBeamFlexBody>();
        if (!obj.TryGetProperty("flexbodies", out var arr) || arr.ValueKind != JsonValueKind.Array)
        {
	        return flexBodies;
        }

        var headerSeen = false;
        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind == JsonValueKind.Object)
            {
	            continue;
            }

            if (elem.ValueKind == JsonValueKind.Array)
            {
                var items = elem.EnumerateArray().ToList();
                if (!headerSeen) { headerSeen = true; continue; }
                if (items.Count < 2)
                {
	                continue;
                }

                var mesh = items[0].ValueKind == JsonValueKind.String ? items[0].GetString() ?? "" : "";
                var groups = new List<string>();
                System.Numerics.Vector3? flexPos = null;
                System.Numerics.Vector3? flexRot = null;
                System.Numerics.Vector3? flexScale = null;

                if (items[1].ValueKind == JsonValueKind.Array)
                {
                    foreach (var g in items[1].EnumerateArray())
                    {
                        var s = g.GetString() ?? "";
                        if (s.Length > 0)
                        {
	                        groups.Add(s);
                        }
                    }
                }

                // Items[3] (or later) may be an object with a "pos" sub-object giving the
                // absolute BeamNG-space position of the mesh — exactly what we need for wheel centres.
                for (var i = 3; i < items.Count; i++)
                {
                    if (items[i].ValueKind != JsonValueKind.Object)
                    {
	                    continue;
                    }

                    if (items[i].TryGetProperty("pos", out var posObj)
                        && posObj.ValueKind == JsonValueKind.Object)
                    {
                        flexPos = ParseVector3(posObj);
                    }

                    if (items[i].TryGetProperty("rot", out var rotObj)
                        && rotObj.ValueKind == JsonValueKind.Object)
                    {
                        flexRot = ParseVector3(rotObj);
                    }

                    if (items[i].TryGetProperty("scale", out var scaleObj)
                        && scaleObj.ValueKind == JsonValueKind.Object)
                    {
                        flexScale = ParseVector3(scaleObj);
                    }
                }

                if (!string.IsNullOrEmpty(mesh))
                {
	                flexBodies.Add(new JBeamFlexBody(mesh, groups, flexPos, flexRot, flexScale));
                }
            }
        }
        return flexBodies;
    }

    private static Dictionary<string, float> ParseVariables(JsonElement obj)
    {
        var variables = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);
        ParseVariables(obj, variables);
        return variables;
    }

    private static void ParseVariables(JsonElement obj, Dictionary<string, float> variables)
    {
        if (!obj.TryGetProperty("variables", out var arr) || arr.ValueKind != JsonValueKind.Array)
        {
            return;
        }

        var headerSeen = false;
        var previousVars = _vars;
        try
        {
            _vars = variables;
            foreach (var elem in arr.EnumerateArray())
            {
                if (elem.ValueKind != JsonValueKind.Array)
                {
                    continue;
                }

                var items = elem.EnumerateArray().ToList();
                if (!headerSeen)
                {
                    headerSeen = true;
                    continue;
                }

                if (items.Count < 5)
                {
                    continue;
                }

                var rawName = SafeGetString(items[0]);
                if (string.IsNullOrWhiteSpace(rawName))
                {
                    continue;
                }

                var name = rawName.TrimStart('$');
                variables[name] = GetFloat(items[4]);
            }
        }
        finally
        {
            _vars = previousVars;
        }
    }

    private static List<JBeamPowertrainDevice> ParsePowertrain(JsonElement obj)
    {
        var devices = new List<JBeamPowertrainDevice>();
        if (!obj.TryGetProperty("powertrain", out var arr) || arr.ValueKind != JsonValueKind.Array)
        {
            return devices;
        }

        var headerSeen = false;
        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind != JsonValueKind.Array)
            {
                continue;
            }

            var items = elem.EnumerateArray().ToList();
            if (!headerSeen)
            {
                headerSeen = true;
                continue;
            }

            if (items.Count < 4)
            {
                continue;
            }

            var options = items.Skip(4).FirstOrDefault(item => item.ValueKind == JsonValueKind.Object);
            var connectedWheel = "";
            float? gearRatio = null;
            var diffType = "";
            float? lsdPreload = null;
            float? lsdLockCoef = null;
            float? lsdRevLockCoef = null;

            if (options.ValueKind == JsonValueKind.Object)
            {
                if (options.TryGetProperty("connectedWheel", out var connectedWheelElement))
                {
                    connectedWheel = SafeGetString(connectedWheelElement);
                }

                if (options.TryGetProperty("gearRatio", out var gearRatioElement))
                {
                    gearRatio = GetFloat(gearRatioElement);
                }

                if (options.TryGetProperty("diffType", out var diffTypeElement))
                {
                    diffType = SafeGetString(diffTypeElement);
                }

                if (options.TryGetProperty("lsdPreload", out var lsdPreloadElement))
                {
                    lsdPreload = GetFloat(lsdPreloadElement);
                }

                if (options.TryGetProperty("lsdLockCoef", out var lsdLockCoefElement))
                {
                    lsdLockCoef = GetFloat(lsdLockCoefElement);
                }

                if (options.TryGetProperty("lsdRevLockCoef", out var lsdRevLockCoefElement))
                {
                    lsdRevLockCoef = GetFloat(lsdRevLockCoefElement);
                }
            }

            devices.Add(new JBeamPowertrainDevice(
                Type: SafeGetString(items[0]),
                Name: SafeGetString(items[1]),
                InputName: SafeGetString(items[2]),
                InputIndex: (int)GetFloat(items[3]),
                ConnectedWheel: connectedWheel,
                GearRatio: gearRatio,
                DiffType: diffType,
                LsdPreload: lsdPreload,
                LsdLockCoef: lsdLockCoef,
                LsdRevLockCoef: lsdRevLockCoef));
        }

        return devices;
    }

    private static JBeamEngineDefinition? ParseEngineDefinition(JsonElement obj)
    {
        if (!TryGetNamedSection(obj, section =>
                section.TryGetProperty("torque", out _) &&
                section.TryGetProperty("maxRPM", out _),
            out var engineSection))
        {
            return null;
        }

        return new JBeamEngineDefinition
        {
            TorqueCurve = ParseTorqueCurve(engineSection),
            IdleRpm = GetOptionalFloat(engineSection, "idleRPM"),
            MaxRpm = GetOptionalFloat(engineSection, "maxRPM"),
            Inertia = GetOptionalFloat(engineSection, "inertia"),
            Friction = GetOptionalFloat(engineSection, "friction"),
            DynamicFriction = GetOptionalFloat(engineSection, "dynamicFriction"),
            EngineBrakeTorque = GetOptionalFloat(engineSection, "engineBrakeTorque"),
        };
    }

    private static JBeamGearboxDefinition? ParseGearboxDefinition(JsonElement obj)
    {
        if (!TryGetNamedSection(obj, section => section.TryGetProperty("gearRatios", out _), out var gearboxSection))
        {
            return null;
        }

        return new JBeamGearboxDefinition
        {
            GearRatios = ParseFloatList(gearboxSection, "gearRatios"),
        };
    }

    private static JBeamVehicleControllerDefinition? ParseVehicleControllerDefinition(JsonElement obj)
    {
        if (!obj.TryGetProperty("vehicleController", out var controllerSection) ||
            controllerSection.ValueKind != JsonValueKind.Object)
        {
            return null;
        }

        return new JBeamVehicleControllerDefinition
        {
            ClutchLaunchStartRpm = TryGetOptionalFloat(controllerSection, "clutchLaunchStartRPM"),
            ClutchLaunchTargetRpm = TryGetOptionalFloat(controllerSection, "clutchLaunchTargetRPM"),
            HighShiftUpRpm = TryGetOptionalFloat(controllerSection, "highShiftUpRPM"),
            HighShiftDownRpm = ParseFloatList(controllerSection, "highShiftDownRPM"),
            LowShiftUpRpm = ParseFloatList(controllerSection, "lowShiftUpRPM"),
            LowShiftDownRpm = ParseFloatList(controllerSection, "lowShiftDownRPM"),
        };
    }

    // ──────────────────────────────────────────────────────────────────────────
    // RefNodes
    // ──────────────────────────────────────────────────────────────────────────

    private static Dictionary<string, string> ParseRefNodes(JsonElement obj)
    {
        var dict = new Dictionary<string, string>();
        if (!obj.TryGetProperty("refNodes", out var arr) || arr.ValueKind != JsonValueKind.Array)
        {
	        return dict;
        }

        string[]? headers = null;
        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind != JsonValueKind.Array)
            {
	            continue;
            }

            var items = elem.EnumerateArray().ToList();

            if (headers == null)
            {
                headers = new string[items.Count];
                for (var i = 0; i < items.Count; i++)
                    headers[i] = items[i].ValueKind == JsonValueKind.String
                        ? (items[i].GetString() ?? "").TrimEnd(':')
                        : "";
                continue;
            }

            for (var i = 0; i < Math.Min(headers.Length, items.Count); i++)
            {
                var val = items[i].ValueKind == JsonValueKind.String ? items[i].GetString() ?? "" : "";
                if (!string.IsNullOrEmpty(val))
                {
	                dict[headers[i]] = val;
                }
            }
        }
        return dict;
    }

    private static bool TryGetNamedSection(
        JsonElement obj,
        Func<JsonElement, bool> predicate,
        out JsonElement section)
    {
        foreach (var property in obj.EnumerateObject())
        {
            if (property.Value.ValueKind != JsonValueKind.Object)
            {
                continue;
            }

            if (predicate(property.Value))
            {
                section = property.Value;
                return true;
            }
        }

        section = default;
        return false;
    }

    private static List<JBeamTorquePoint> ParseTorqueCurve(JsonElement engineSection)
    {
        var points = new List<JBeamTorquePoint>();
        if (!engineSection.TryGetProperty("torque", out var torqueElement) ||
            torqueElement.ValueKind != JsonValueKind.Array)
        {
            return points;
        }

        var headerSeen = false;
        foreach (var elem in torqueElement.EnumerateArray())
        {
            if (elem.ValueKind != JsonValueKind.Array)
            {
                continue;
            }

            var items = elem.EnumerateArray().ToList();
            if (!headerSeen)
            {
                headerSeen = true;
                continue;
            }

            if (items.Count < 2)
            {
                continue;
            }

            points.Add(new JBeamTorquePoint(GetFloat(items[0]), GetFloat(items[1])));
        }

        return points;
    }

    private static List<float> ParseFloatList(JsonElement obj, string propertyName)
    {
        var values = new List<float>();
        if (!obj.TryGetProperty(propertyName, out var arrayElement) ||
            arrayElement.ValueKind != JsonValueKind.Array)
        {
            return values;
        }

        foreach (var item in arrayElement.EnumerateArray())
        {
            values.Add(GetFloat(item));
        }

        return values;
    }

    private static float GetOptionalFloat(JsonElement obj, string propertyName, float fallback = 0f)
    {
        return obj.TryGetProperty(propertyName, out var element) ? GetFloat(element) : fallback;
    }

    private static float? TryGetOptionalFloat(JsonElement obj, string propertyName)
    {
        return obj.TryGetProperty(propertyName, out var element) ? GetFloat(element) : null;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Helpers
    // ──────────────────────────────────────────────────────────────────────────

    private static float GetFloat(JsonElement e)
    {
        if (e.ValueKind == JsonValueKind.Number)
        {
	        return e.GetSingle();
        }

        if (e.ValueKind == JsonValueKind.String)
        {
            var s = (e.GetString() ?? "").Trim();
            // Resolve $variable references from the active vars table
            if (s.StartsWith("$") && !s.StartsWith("$=") && _vars != null)
            {
                var varName = s[1..]; // strip leading $
                if (_vars.TryGetValue(varName, out var resolved))
                {
	                return resolved;
                }

                return 0f;
            }

            if (TryEvaluateArithmeticExpression(s, out var expr))
            {
	            return expr;
            }

            return float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : 0f;
        }
        return 0f;
    }

    private static float GetFloatOrMax(JsonElement e)
    {
        if (e.ValueKind == JsonValueKind.String)
        {
            var s = e.GetString() ?? "";
            if (s == "FLT_MAX" || s.Contains("MAX"))
            {
	            return float.MaxValue;
            }
        }
        return GetFloat(e);
    }

    /// <summary>
    /// Returns the string value if the element is a string, otherwise empty string.
    /// Avoids InvalidOperationException when a field unexpectedly contains an array or number.
    /// </summary>
    private static string SafeGetString(JsonElement e)
        => e.ValueKind == JsonValueKind.String ? e.GetString() ?? "" : "";

    private static Vector3 ParseVector3(JsonElement obj)
    {
        obj.TryGetProperty("x", out var x);
        obj.TryGetProperty("y", out var y);
        obj.TryGetProperty("z", out var z);
        return new Vector3(GetFloat(x), GetFloat(y), GetFloat(z));
    }

    private static bool TryEvaluateArithmeticExpression(string raw, out float value)
    {
        value = 0f;
        if (!raw.StartsWith("$=", StringComparison.Ordinal))
        {
	        return false;
        }

        var expr = ReplaceVariablesWithValues(raw[2..].Trim());
        if (string.IsNullOrWhiteSpace(expr))
        {
	        return false;
        }

        try
        {
            var parser = new ArithmeticParser(expr);
            value = parser.ParseExpression();
            parser.SkipWhitespace();
            return parser.AtEnd;
        }
        catch
        {
            value = 0f;
            return false;
        }
    }

    private static string ReplaceVariablesWithValues(string expr)
    {
        if (string.IsNullOrEmpty(expr))
        {
	        return expr;
        }

        var sb = new StringBuilder(expr.Length + 16);
        for (var i = 0; i < expr.Length; i++)
        {
            if (expr[i] != '$')
            {
                sb.Append(expr[i]);
                continue;
            }

            var start = i + 1;
            var end = start;
            while (end < expr.Length &&
                   (char.IsLetterOrDigit(expr[end]) || expr[end] == '_'))
            {
                end++;
            }

            var varName = expr[start..end];
            var resolved = 0f;
            if (!string.IsNullOrEmpty(varName) && _vars != null)
            {
	            _vars.TryGetValue(varName, out resolved);
            }

            sb.Append(resolved.ToString(CultureInfo.InvariantCulture));
            i = end - 1;
        }

        return sb.ToString();
    }

    private sealed class ArithmeticParser
    {
        private readonly string _text;
        private int _index;

        public ArithmeticParser(string text) => _text = text;

        public bool AtEnd => _index >= _text.Length;

        public void SkipWhitespace()
        {
            while (_index < _text.Length && char.IsWhiteSpace(_text[_index]))
                _index++;
        }

        public float ParseExpression()
        {
            var value = ParseTerm();
            while (true)
            {
                SkipWhitespace();
                if (Match('+'))
                {
	                value += ParseTerm();
                }
                else if (Match('-'))
                {
	                value -= ParseTerm();
                }
                else
                {
	                break;
                }
            }

            return value;
        }

        private float ParseTerm()
        {
            var value = ParseFactor();
            while (true)
            {
                SkipWhitespace();
                if (Match('*'))
                {
	                value *= ParseFactor();
                }
                else if (Match('/'))
                {
	                value /= ParseFactor();
                }
                else
                {
	                break;
                }
            }

            return value;
        }

        private float ParseFactor()
        {
            SkipWhitespace();
            if (Match('+'))
            {
	            return ParseFactor();
            }

            if (Match('-'))
            {
	            return -ParseFactor();
            }

            if (Match('('))
            {
                var value = ParseExpression();
                SkipWhitespace();
                Expect(')');
                return value;
            }

            return ParseNumber();
        }

        private float ParseNumber()
        {
            SkipWhitespace();
            var start = _index;
            while (_index < _text.Length &&
                   (char.IsDigit(_text[_index]) || _text[_index] == '.'))
            {
                _index++;
            }

            if (start == _index)
            {
	            throw new FormatException("Expected number.");
            }

            return float.Parse(_text[start.._index], NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        private bool Match(char ch)
        {
            if (_index < _text.Length && _text[_index] == ch)
            {
                _index++;
                return true;
            }

            return false;
        }

        private void Expect(char ch)
        {
            if (!Match(ch))
            {
	            throw new FormatException($"Expected '{ch}'.");
            }
        }
    }

    /// <summary>
    /// Some jbeam files contain multiple adjacent top-level JSON objects, e.g.:
    ///   { "part_a": { ... } }
    ///   { "part_b": { ... } }
    /// This method merges them into a single object by removing the inter-object
    /// boundary <c>} {</c> (ignoring comments and strings).
    /// </summary>
    private static string MergeRootObjects(string input)
    {
        // ── Fix 1: file starts with "key": instead of { "key": ──────────────
        // Trim leading whitespace/comments and check if first non-whitespace is a string literal.
        var k = 0;
        while (k < input.Length && char.IsWhiteSpace(input[k])) k++;
        if (k < input.Length && input[k] == '"')
        {
            // No leading '{' — wrap the whole file in {}
            input = "{\n" + input + "\n}";
        }

        // ── Quick check: count depth-0 close-brace events ────────────────────
        var depth = 0;
        var rootCloseCount = 0;
        var i = 0;
        while (i < input.Length)
        {
            var c = input[i];
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '*')
            { i += 2; while (i < input.Length - 1 && !(input[i] == '*' && input[i + 1] == '/')) i++; i += 2; continue; }
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '/')
            { while (i < input.Length && input[i] != '\n') i++; continue; }
            if (c == '"')
            { i++; while (i < input.Length) { if (input[i] == '\\') { i += 2; continue; } if (input[i++] == '"')
	            {
		            break;
	            }
            } continue; }
            if (c == '{')
            {
	            depth++;
            }
            else if (c == '}') { depth--; if (depth <= 0)
	            {
		            rootCloseCount++;
	            }
            }
            i++;
        }

        if (rootCloseCount <= 1)
        {
	        return input; // single root, nothing to do
        }

        // ── Fix 2 / Fix 3: multiple root close events ─────────────────────────
        // Either there are multiple `{...} {...}` blocks or a trailing stray `}`.
        // Walk the text:
        //   - When depth reaches 0 at '}' and the NEXT token is '{', merge them (insert ',')
        //   - When depth would go negative at '}', drop it (stray brace)
        var sb = new StringBuilder(input.Length);
        depth = 0;
        i = 0;
        while (i < input.Length)
        {
            var c = input[i];

            // Block comment — copy verbatim
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '*')
            {
                var start = i; i += 2;
                while (i < input.Length - 1 && !(input[i] == '*' && input[i + 1] == '/')) i++;
                i += 2;
                sb.Append(input, start, i - start);
                continue;
            }
            // Line comment — copy verbatim
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '/')
            {
                var start = i;
                while (i < input.Length && input[i] != '\n') i++;
                sb.Append(input, start, i - start);
                continue;
            }
            // String — copy verbatim
            if (c == '"')
            {
                sb.Append(c); i++;
                while (i < input.Length)
                {
                    var sc = input[i]; sb.Append(sc); i++;
                    if (sc == '\\') { if (i < input.Length) { sb.Append(input[i]); i++; } continue; }
                    if (sc == '"')
                    {
	                    break;
                    }
                }
                continue;
            }

            if (c == '{') { depth++; sb.Append(c); i++; continue; }

            if (c == '}')
            {
                depth--;
                if (depth < 0)
                {
                    // Stray closing brace — drop it
                    i++;
                    continue;
                }
                if (depth == 0)
                {
                    // Look ahead past whitespace/comments to see if another '{' follows
                    var j = i + 1;
                    while (j < input.Length)
                    {
                        var jc = input[j];
                        if (char.IsWhiteSpace(jc)) { j++; continue; }
                        if (jc == '/' && j + 1 < input.Length && input[j + 1] == '*')
                        { j += 2; while (j < input.Length - 1 && !(input[j] == '*' && input[j + 1] == '/')) j++; j += 2; continue; }
                        if (jc == '/' && j + 1 < input.Length && input[j + 1] == '/')
                        { while (j < input.Length && input[j] != '\n') j++; continue; }
                        break;
                    }
                    if (j < input.Length && input[j] == '{')
                    {
                        // Next root object found — insert ',' and skip the '{' boundary
                        sb.Append(',');
                        i++;          // consume '}'
                        while (i < input.Length && i != j) i++;
                        i++;          // consume '{'
                        depth = 1;
                        continue;
                    }
                }
                sb.Append(c); i++;
                continue;
            }

            sb.Append(c); i++;
        }
        return sb.ToString();
    }
}

file static class JsonElementExtensions
{
    public static List<JsonElement> ToList(this JsonElement.ArrayEnumerator enumerator)
    {
        var list = new List<JsonElement>();
        foreach (var item in enumerator) list.Add(item);
        return list;
    }
}
