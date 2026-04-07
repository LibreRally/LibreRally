using System;
using System.Collections.Generic;
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

    public static List<JBeamPart> ParseFile(string path)
    {
        string raw = System.IO.File.ReadAllText(path);
        return Parse(raw);
    }

    public static List<JBeamPart> Parse(string jbeamText)
    {
        // Some jbeam files contain multiple top-level objects — merge them before preprocessing.
        string merged = MergeRootObjects(jbeamText);
        string preprocessed = PreprocessJBeam(merged);

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
            return parts;

        foreach (JsonProperty partProp in doc.RootElement.EnumerateObject())
        {
            if (partProp.Value.ValueKind != JsonValueKind.Object)
                continue;

            var part = ParsePart(partProp.Name, partProp.Value);
            if (part != null)
                parts.Add(part);
        }

        return parts;
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
        int i = 0;
        bool afterValue = false; // true when last emitted token was a complete JSON value

        void MaybeComma()
        {
            if (!afterValue) return;
            sb.Append(',');
            afterValue = false;
        }

        while (i < input.Length)
        {
            char c = input[i];

            // ── Whitespace ─────────────────────────────────────────────────
            if (char.IsWhiteSpace(c)) { sb.Append(c); i++; continue; }

            // ── Block comment /* … */ ──────────────────────────────────────
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '*')
            {
                i += 2;
                while (i < input.Length - 1 && !(input[i] == '*' && input[i + 1] == '/'))
                    i++;
                if (i < input.Length - 1) i += 2;
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
                bool esc = false;
                while (i < input.Length)
                {
                    char sc = input[i]; sb.Append(sc); i++;
                    if (esc) { esc = false; continue; }
                    if (sc == '\\') { esc = true; continue; }
                    if (sc == '"') break;
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
                    char nc = input[i];
                    if (char.IsDigit(nc) || nc == '.' || nc == 'e' || nc == 'E' || nc == '+' || nc == '-')
                    { sb.Append(nc); i++; }
                    else break;
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
        string slotType = "";
        if (obj.TryGetProperty("slotType", out var st))
        {
            if (st.ValueKind == JsonValueKind.String)
                slotType = st.GetString() ?? "";
            else if (st.ValueKind == JsonValueKind.Array)
                slotType = string.Join(",", st.EnumerateArray().ToList()
                    .Where(e => e.ValueKind == JsonValueKind.String)
                    .Select(e => e.GetString() ?? "")
                    .Where(s => s.Length > 0));
        }

        var part = new JBeamPart
        {
            Name = name,
            SlotType = slotType,
            Slots = ParseSlots(obj),
            Nodes = ParseNodes(obj),
            Beams = ParseBeams(obj),
            FlexBodies = ParseFlexBodies(obj),
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
            ParseSlotsArray(arr, slots, legacy: true);

        // Modern "slots2" format: [name, allowTypes[], denyTypes[], default, description, {options}?]
        if (obj.TryGetProperty("slots2", out var arr2) && arr2.ValueKind == JsonValueKind.Array)
            ParseSlotsArray(arr2, slots, legacy: false);

        return slots;
    }

    private static void ParseSlotsArray(JsonElement arr, List<JBeamSlot> slots, bool legacy)
    {
        bool headerSeen = false;
        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind != JsonValueKind.Array) continue;
            var items = elem.EnumerateArray().ToList();

            if (!headerSeen) { headerSeen = true; continue; }

            // Legacy:  [type(0), default(1), description(2), {options}(3)?]
            // Slots2:  [name(0), allowTypes(1), denyTypes(2), default(3), description(4), {options}(5)?]
            int defaultIdx  = legacy ? 1 : 3;
            int descIdx     = legacy ? 2 : 4;
            int optionsIdx  = legacy ? 3 : 5;

            if (items.Count <= defaultIdx) continue;

            // Skip entries where default is not a string (e.g. empty array or missing)
            string? defaultVal = items[defaultIdx].ValueKind == JsonValueKind.String
                ? items[defaultIdx].GetString()
                : null;
            if (string.IsNullOrEmpty(defaultVal)) continue;

            string type = items[0].ValueKind == JsonValueKind.String ? items[0].GetString() ?? "" : "";

            bool core = false;
            if (items.Count > optionsIdx && items[optionsIdx].ValueKind == JsonValueKind.Object)
            {
                if (items[optionsIdx].TryGetProperty("coreSlot", out var cs))
                    core = cs.ValueKind == JsonValueKind.True;
            }

            slots.Add(new JBeamSlot(
                Type: type,
                Default: defaultVal,
                Description: items.Count > descIdx && items[descIdx].ValueKind == JsonValueKind.String
                    ? items[descIdx].GetString() ?? ""
                    : "",
                CoreSlot: core));
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Nodes
    // ──────────────────────────────────────────────────────────────────────────

    private static List<JBeamNode> ParseNodes(JsonElement obj)
    {
        var nodes = new List<JBeamNode>();
        if (!obj.TryGetProperty("nodes", out var arr) || arr.ValueKind != JsonValueKind.Array)
            return nodes;

        bool headerSeen = false;
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
                if (items.Count < 4) continue;

                string id = items[0].ValueKind == JsonValueKind.String ? items[0].GetString() ?? "" : "";
                float x = GetFloat(items[1]);
                float y = GetFloat(items[2]);
                float z = GetFloat(items[3]);

                // Inline property override
                var rowProps = props with { Groups = new List<string>(props.Groups) };
                for (int i = 4; i < items.Count; i++)
                {
                    if (items[i].ValueKind == JsonValueKind.Object)
                        rowProps = ApplyNodeProps(rowProps, items[i]);
                }

                nodes.Add(new JBeamNode(id, new Vector3(x, y, z), rowProps));
            }
        }
        return nodes;
    }

    private static NodeProperties ApplyNodeProps(NodeProperties current, JsonElement obj)
    {
        float weight = current.Weight;
        bool collision = current.Collision;
        bool selfCollision = current.SelfCollision;
        string material = current.Material;
        float friction = current.FrictionCoef;
        var groups = new List<string>(current.Groups);

        if (obj.TryGetProperty("nodeWeight", out var nw)) weight = GetFloat(nw);
        if (obj.TryGetProperty("collision", out var col)) collision = col.GetBoolean();
        if (obj.TryGetProperty("selfCollision", out var sc)) selfCollision = sc.GetBoolean();
        if (obj.TryGetProperty("nodeMaterial", out var nm)) material = SafeGetString(nm).Length > 0 ? SafeGetString(nm) : material;
        if (obj.TryGetProperty("frictionCoef", out var fc)) friction = GetFloat(fc);
        if (obj.TryGetProperty("group", out var grp))
        {
            groups = new List<string>();
            if (grp.ValueKind == JsonValueKind.String)
            {
                var s = grp.GetString() ?? "";
                if (s.Length > 0) groups.Add(s);
            }
            else if (grp.ValueKind == JsonValueKind.Array)
            {
                foreach (var g in grp.EnumerateArray())
                {
                    var s = g.GetString() ?? "";
                    if (s.Length > 0) groups.Add(s);
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
            return beams;

        bool headerSeen = false;
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
                if (items.Count < 2) continue;

                string id1 = items[0].ValueKind == JsonValueKind.String ? items[0].GetString() ?? "" : "";
                string id2 = items[1].ValueKind == JsonValueKind.String ? items[1].GetString() ?? "" : "";
                if (string.IsNullOrEmpty(id1) || string.IsNullOrEmpty(id2)) continue;

                var rowProps = props;
                for (int i = 2; i < items.Count; i++)
                {
                    if (items[i].ValueKind == JsonValueKind.Object)
                        rowProps = ApplyBeamProps(rowProps, items[i]);
                }

                beams.Add(new JBeamBeam(id1, id2, rowProps));
            }
        }
        return beams;
    }

    private static BeamProperties ApplyBeamProps(BeamProperties current, JsonElement obj)
    {
        float spring = current.Spring;
        float damp = current.Damp;
        float deform = current.Deform;
        float strength = current.Strength;
        string type = current.BeamType;
        string deformGroup = current.DeformGroup;
        float triggerRatio = current.DeformationTriggerRatio;
        bool optional = current.Optional;

        if (obj.TryGetProperty("beamSpring", out var bs)) spring = GetFloat(bs);
        if (obj.TryGetProperty("beamDamp", out var bd)) damp = GetFloat(bd);
        if (obj.TryGetProperty("beamDeform", out var bdf)) deform = GetFloat(bdf);
        if (obj.TryGetProperty("beamStrength", out var bst)) strength = GetFloatOrMax(bst);
        if (obj.TryGetProperty("beamType", out var bt)) type = SafeGetString(bt).Length > 0 ? SafeGetString(bt) : type;
        if (obj.TryGetProperty("deformGroup", out var dg)) deformGroup = SafeGetString(dg);
        if (obj.TryGetProperty("deformationTriggerRatio", out var dtr)) triggerRatio = GetFloat(dtr);
        if (obj.TryGetProperty("optional", out var opt)) optional = opt.ValueKind == JsonValueKind.True || (opt.ValueKind == JsonValueKind.Number && opt.GetInt32() != 0);

        return new BeamProperties(spring, damp, deform, strength, type, deformGroup, triggerRatio, optional);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // FlexBodies
    // ──────────────────────────────────────────────────────────────────────────

    private static List<JBeamFlexBody> ParseFlexBodies(JsonElement obj)
    {
        var flexBodies = new List<JBeamFlexBody>();
        if (!obj.TryGetProperty("flexbodies", out var arr) || arr.ValueKind != JsonValueKind.Array)
            return flexBodies;

        bool headerSeen = false;
        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind == JsonValueKind.Object) continue;

            if (elem.ValueKind == JsonValueKind.Array)
            {
                var items = elem.EnumerateArray().ToList();
                if (!headerSeen) { headerSeen = true; continue; }
                if (items.Count < 2) continue;

                string mesh = items[0].ValueKind == JsonValueKind.String ? items[0].GetString() ?? "" : "";
                var groups = new List<string>();
                System.Numerics.Vector3? flexPos = null;

                if (items[1].ValueKind == JsonValueKind.Array)
                {
                    foreach (var g in items[1].EnumerateArray())
                    {
                        var s = g.GetString() ?? "";
                        if (s.Length > 0) groups.Add(s);
                    }
                }

                // Items[3] (or later) may be an object with a "pos" sub-object giving the
                // absolute BeamNG-space position of the mesh — exactly what we need for wheel centres.
                for (int i = 3; i < items.Count; i++)
                {
                    if (items[i].ValueKind != JsonValueKind.Object) continue;
                    if (items[i].TryGetProperty("pos", out var posObj)
                        && posObj.ValueKind == JsonValueKind.Object)
                    {
                        posObj.TryGetProperty("x", out var px);
                        posObj.TryGetProperty("y", out var py);
                        posObj.TryGetProperty("z", out var pz);
                        flexPos = new System.Numerics.Vector3(
                            (float)(px.ValueKind == JsonValueKind.Number ? px.GetDouble() : 0),
                            (float)(py.ValueKind == JsonValueKind.Number ? py.GetDouble() : 0),
                            (float)(pz.ValueKind == JsonValueKind.Number ? pz.GetDouble() : 0));
                        break;
                    }
                }

                if (!string.IsNullOrEmpty(mesh))
                    flexBodies.Add(new JBeamFlexBody(mesh, groups, flexPos));
            }
        }
        return flexBodies;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // RefNodes
    // ──────────────────────────────────────────────────────────────────────────

    private static Dictionary<string, string> ParseRefNodes(JsonElement obj)
    {
        var dict = new Dictionary<string, string>();
        if (!obj.TryGetProperty("refNodes", out var arr) || arr.ValueKind != JsonValueKind.Array)
            return dict;

        string[]? headers = null;
        foreach (var elem in arr.EnumerateArray())
        {
            if (elem.ValueKind != JsonValueKind.Array) continue;
            var items = elem.EnumerateArray().ToList();

            if (headers == null)
            {
                headers = new string[items.Count];
                for (int i = 0; i < items.Count; i++)
                    headers[i] = items[i].ValueKind == JsonValueKind.String
                        ? (items[i].GetString() ?? "").TrimEnd(':')
                        : "";
                continue;
            }

            for (int i = 0; i < Math.Min(headers.Length, items.Count); i++)
            {
                string val = items[i].ValueKind == JsonValueKind.String ? items[i].GetString() ?? "" : "";
                if (!string.IsNullOrEmpty(val))
                    dict[headers[i]] = val;
            }
        }
        return dict;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Helpers
    // ──────────────────────────────────────────────────────────────────────────

    private static float GetFloat(JsonElement e)
    {
        return e.ValueKind switch
        {
            JsonValueKind.Number => e.GetSingle(),
            JsonValueKind.String => float.TryParse(e.GetString(), out float v) ? v : 0f,
            _ => 0f,
        };
    }

    private static float GetFloatOrMax(JsonElement e)
    {
        if (e.ValueKind == JsonValueKind.String)
        {
            var s = e.GetString() ?? "";
            if (s == "FLT_MAX" || s.Contains("MAX")) return float.MaxValue;
        }
        return GetFloat(e);
    }

    /// <summary>
    /// Returns the string value if the element is a string, otherwise empty string.
    /// Avoids InvalidOperationException when a field unexpectedly contains an array or number.
    /// </summary>
    private static string SafeGetString(JsonElement e)
        => e.ValueKind == JsonValueKind.String ? e.GetString() ?? "" : "";

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
        int k = 0;
        while (k < input.Length && char.IsWhiteSpace(input[k])) k++;
        if (k < input.Length && input[k] == '"')
        {
            // No leading '{' — wrap the whole file in {}
            input = "{\n" + input + "\n}";
        }

        // ── Quick check: count depth-0 close-brace events ────────────────────
        int depth = 0;
        int rootCloseCount = 0;
        int i = 0;
        while (i < input.Length)
        {
            char c = input[i];
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '*')
            { i += 2; while (i < input.Length - 1 && !(input[i] == '*' && input[i + 1] == '/')) i++; i += 2; continue; }
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '/')
            { while (i < input.Length && input[i] != '\n') i++; continue; }
            if (c == '"')
            { i++; while (i < input.Length) { if (input[i] == '\\') { i += 2; continue; } if (input[i++] == '"') break; } continue; }
            if (c == '{') depth++;
            else if (c == '}') { depth--; if (depth <= 0) rootCloseCount++; }
            i++;
        }

        if (rootCloseCount <= 1) return input; // single root, nothing to do

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
            char c = input[i];

            // Block comment — copy verbatim
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '*')
            {
                int start = i; i += 2;
                while (i < input.Length - 1 && !(input[i] == '*' && input[i + 1] == '/')) i++;
                i += 2;
                sb.Append(input, start, i - start);
                continue;
            }
            // Line comment — copy verbatim
            if (c == '/' && i + 1 < input.Length && input[i + 1] == '/')
            {
                int start = i;
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
                    char sc = input[i]; sb.Append(sc); i++;
                    if (sc == '\\') { if (i < input.Length) { sb.Append(input[i]); i++; } continue; }
                    if (sc == '"') break;
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
                    int j = i + 1;
                    while (j < input.Length)
                    {
                        char jc = input[j];
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
