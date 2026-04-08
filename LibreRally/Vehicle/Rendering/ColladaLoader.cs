using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Xml.Linq;

namespace LibreRally.Vehicle.Rendering;

/// <summary>Vertex with position, normal, and UV coordinates.</summary>
public record ColladaVertex(Vector3 Position, Vector3 Normal, Vector2 TexCoord);

/// <summary>A single mesh sub-object from a Collada file, identified by geometry name.</summary>
public class ColladaMesh
{
    public string Name { get; init; } = "";
    public string GeometryName { get; init; } = "";
    public string MaterialName { get; init; } = "";
    public List<ColladaVertex> Vertices { get; init; } = new();
    public List<int> Indices { get; init; } = new();
}

/// <summary>
/// Pure managed Collada (.dae) loader using System.Xml.Linq.
/// No native dependencies required.
/// Handles the subset of Collada produced by BeamNG's mesh export.
/// </summary>
public static class ColladaLoader
{
    private static readonly XNamespace Ns = "http://www.collada.org/2005/11/COLLADASchema";

    public static List<ColladaMesh> Load(string daeFilePath)
    {
        var doc = XDocument.Load(daeFilePath);
        var root = doc.Root ?? throw new InvalidDataException("Empty Collada file.");

        var meshes = new List<ColladaMesh>();
        var geometryLib = root.Element(Ns + "library_geometries");
        if (geometryLib == null) return meshes;

        foreach (var geometry in geometryLib.Elements(Ns + "geometry"))
        {
            string geoName = geometry.Attribute("name")?.Value
                ?? geometry.Attribute("id")?.Value
                ?? "unknown";

            var mesh = geometry.Element(Ns + "mesh");
            if (mesh == null) continue;

            meshes.AddRange(ParseMesh(geoName, mesh));
        }

        return meshes;
    }

    // BeamNG flex-body deformation meshes (steering rack morph targets etc.) have vertices
    // spanning tens of metres.  Any axis extent above this threshold flags them for exclusion.
    private const float MaxGeomExtentMetres = 7f;

    private static IEnumerable<ColladaMesh> ParseMesh(string geometryName, XElement mesh)
    {
        // Build source arrays keyed by #id
        var sources = new Dictionary<string, float[]>(StringComparer.OrdinalIgnoreCase);
        foreach (var source in mesh.Elements(Ns + "source"))
        {
            string id = "#" + (source.Attribute("id")?.Value ?? "");
            var floatArray = source.Element(Ns + "float_array");
            if (floatArray != null)
            {
                sources[id] = ParseFloatArray(floatArray.Value);
            }
        }

        // Resolve <vertices> indirection
        var verticesElem = mesh.Element(Ns + "vertices");
        string verticesId = "#" + (verticesElem?.Attribute("id")?.Value ?? "");
        string positionSourceId = "";
        if (verticesElem != null)
        {
            foreach (var input in verticesElem.Elements(Ns + "input"))
            {
                if (input.Attribute("semantic")?.Value == "POSITION")
                    positionSourceId = input.Attribute("source")?.Value ?? "";
            }
        }
        sources[verticesId + "_POSITION"] = sources.GetValueOrDefault(positionSourceId, Array.Empty<float>());

        // Skip BeamNG flex-body / morph-target geometry whose vertices span huge distances.
        // (e.g. steer_01a…steer_05a span ±19 m — they are deformation targets, not real meshes.)
        if (sources.TryGetValue(positionSourceId, out float[]? rawPos) && rawPos.Length >= 3)
        {
            float minX = float.MaxValue, maxX = float.MinValue;
            float minY = float.MaxValue, maxY = float.MinValue;
            float minZ = float.MaxValue, maxZ = float.MinValue;
            for (int i = 0; i + 2 < rawPos.Length; i += 3)
            {
                if (rawPos[i    ] < minX) minX = rawPos[i    ];
                if (rawPos[i    ] > maxX) maxX = rawPos[i    ];
                if (rawPos[i + 1] < minY) minY = rawPos[i + 1];
                if (rawPos[i + 1] > maxY) maxY = rawPos[i + 1];
                if (rawPos[i + 2] < minZ) minZ = rawPos[i + 2];
                if (rawPos[i + 2] > maxZ) maxZ = rawPos[i + 2];
            }
            if ((maxX - minX) > MaxGeomExtentMetres ||
                (maxY - minY) > MaxGeomExtentMetres ||
                (maxZ - minZ) > MaxGeomExtentMetres)
                yield break;
        }

        // Process <triangles> and <polylist> primitives
        foreach (var prim in mesh.Elements(Ns + "triangles").Concat(mesh.Elements(Ns + "polylist")))
        {
            string material = prim.Attribute("material")?.Value ?? "";
            var colladaMesh = BuildPrimitiveMesh(
                geometryName,
                $"{geometryName}_{material}",
                material,
                prim,
                sources,
                verticesId);

            if (colladaMesh != null)
                yield return colladaMesh;
        }
    }

    private static ColladaMesh? BuildPrimitiveMesh(
        string geometryName,
        string name,
        string material,
        XElement prim,
        Dictionary<string, float[]> sources,
        string verticesId)
    {
        // Gather inputs: semantic → (source, offset)
        var inputs = new Dictionary<string, (string Source, int Offset)>(StringComparer.OrdinalIgnoreCase);
        int maxOffset = 0;
        foreach (var input in prim.Elements(Ns + "input"))
        {
            string semantic = input.Attribute("semantic")?.Value ?? "";
            string source = input.Attribute("source")?.Value ?? "";
            int offset = int.TryParse(input.Attribute("offset")?.Value, out int o) ? o : 0;
            inputs[semantic] = (source, offset);
            if (offset > maxOffset) maxOffset = offset;
        }

        int stride = maxOffset + 1;

        float[] positions = GetSourceByRole(sources, inputs, verticesId, "VERTEX", "POSITION");
        float[] normals = GetSourceByRole(sources, inputs, null, "NORMAL", null);
        float[] uvs = GetSourceByRole(sources, inputs, null, "TEXCOORD", null);

        // Parse index list
        var pElem = prim.Element(Ns + "p");
        if (pElem == null) return null;

        int[] rawIndices = ParseIntArray(pElem.Value);
        if (rawIndices.Length == 0) return null;

        int posOffset = inputs.TryGetValue("VERTEX", out var vIn) ? vIn.Offset : 0;
        int normOffset = inputs.TryGetValue("NORMAL", out var nIn) ? nIn.Offset : -1;
        int uvOffset = inputs.TryGetValue("TEXCOORD", out var uIn) ? uIn.Offset : -1;

        var vertices = new List<ColladaVertex>();
        var indices = new List<int>();
        var vertexMap = new Dictionary<(int, int, int), int>();

        int GetOrAddVertex(int baseIdx)
        {
            int pi = rawIndices[baseIdx + posOffset];
            int ni = normOffset >= 0 ? rawIndices[baseIdx + normOffset] : -1;
            int ui = uvOffset >= 0 ? rawIndices[baseIdx + uvOffset] : -1;

            var key = (pi, ni, ui);
            if (!vertexMap.TryGetValue(key, out int existingIdx))
            {
                existingIdx = vertices.Count;
                vertexMap[key] = existingIdx;

                Vector3 pos = positions.Length >= (pi + 1) * 3
                    ? new Vector3(positions[pi * 3], positions[pi * 3 + 1], positions[pi * 3 + 2])
                    : Vector3.Zero;

                Vector3 norm = ni >= 0 && normals.Length >= (ni + 1) * 3
                    ? new Vector3(normals[ni * 3], normals[ni * 3 + 1], normals[ni * 3 + 2])
                    : Vector3.UnitY;

                Vector2 uv = ui >= 0 && uvs.Length >= (ui + 1) * 2
                    ? new Vector2(uvs[ui * 2], uvs[ui * 2 + 1])
                    : Vector2.Zero;

                vertices.Add(new ColladaVertex(pos, norm, uv));
            }

            return existingIdx;
        }

        if (prim.Name.LocalName.Equals("polylist", StringComparison.OrdinalIgnoreCase))
        {
            var vcountElem = prim.Element(Ns + "vcount");
            if (vcountElem == null) return null;

            int[] polyVertexCounts = ParseIntArray(vcountElem.Value);
            int cursor = 0;

            foreach (int polyVertCount in polyVertexCounts)
            {
                if (polyVertCount < 3)
                {
                    cursor += polyVertCount * stride;
                    continue;
                }

                int polygonIndexCount = polyVertCount * stride;
                if (cursor + polygonIndexCount > rawIndices.Length)
                    break;

                int first = GetOrAddVertex(cursor);
                int prev = GetOrAddVertex(cursor + stride);

                for (int pv = 2; pv < polyVertCount; pv++)
                {
                    int current = GetOrAddVertex(cursor + pv * stride);
                    indices.Add(first);
                    indices.Add(prev);
                    indices.Add(current);
                    prev = current;
                }

                cursor += polygonIndexCount;
            }
        }
        else
        {
            int vertCount = rawIndices.Length / stride;
            for (int vi = 0; vi < vertCount; vi++)
            {
                int baseIdx = vi * stride;
                indices.Add(GetOrAddVertex(baseIdx));
            }
        }

        return new ColladaMesh
        {
            Name = name,
            GeometryName = geometryName,
            MaterialName = material,
            Vertices = vertices,
            Indices = indices,
        };
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Helpers
    // ──────────────────────────────────────────────────────────────────────────

    private static float[] GetSourceByRole(
        Dictionary<string, float[]> sources,
        Dictionary<string, (string Source, int Offset)> inputs,
        string? alternateId,
        string semantic,
        string? fallbackKey)
    {
        if (!inputs.TryGetValue(semantic, out var inputDef))
            return Array.Empty<float>();

        // For VERTEX semantic, resolve through the vertices element
        string key = inputDef.Source;
        if (alternateId != null && key == alternateId && fallbackKey != null)
            key = alternateId + "_" + fallbackKey;

        return sources.GetValueOrDefault(key, Array.Empty<float>());
    }

    /// <summary>
    /// Parses the Collada material chain and returns a dictionary mapping each geometry
    /// primitive's material binding symbol to the texture image filename (e.g. "sunburst_body.png").
    /// The file path stored in the DAE may be absolute from another machine — only the filename
    /// component is returned so callers can do a local lookup.
    /// </summary>
    public static Dictionary<string, string> LoadTextureMap(string daeFilePath)
    {
        var result = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        try
        {
            var doc = XDocument.Load(daeFilePath);
            var root = doc.Root;
            if (root == null) return result;

            // Step 1 — image id → just the filename (absolute paths from other machines)
            var imageFiles = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (var img in root.Descendants(Ns + "image"))
            {
                string id = img.Attribute("id")?.Value ?? "";
                var initFrom = img.Element(Ns + "init_from");
                if (initFrom == null || string.IsNullOrEmpty(id)) continue;

                string rawPath = Uri.UnescapeDataString(initFrom.Value.Trim());
                // Normalise path separators before extracting filename
                rawPath = rawPath.Replace('/', System.IO.Path.DirectorySeparatorChar);
                string filename = System.IO.Path.GetFileName(rawPath);
                if (!string.IsNullOrEmpty(filename))
                    imageFiles[id] = filename;
            }

            // Step 2 — for each effect: resolve sampler→surface→image chain and find diffuse texture
            var effectToFilename = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (var effect in root.Descendants(Ns + "effect"))
            {
                string effectId = effect.Attribute("id")?.Value ?? "";
                if (string.IsNullOrEmpty(effectId)) continue;

                // Collect newparam sid → surface init_from image id
                var surfaceToImage = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                var samplerToSurface = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

                foreach (var newparam in effect.Descendants(Ns + "newparam"))
                {
                    string sid = newparam.Attribute("sid")?.Value ?? "";
                    if (string.IsNullOrEmpty(sid)) continue;

                    var surface = newparam.Element(Ns + "surface");
                    if (surface != null)
                    {
                        string imgRef = surface.Element(Ns + "init_from")?.Value.Trim() ?? "";
                        if (!string.IsNullOrEmpty(imgRef)) surfaceToImage[sid] = imgRef;
                    }

                    var sampler = newparam.Element(Ns + "sampler2D");
                    if (sampler != null)
                    {
                        string srcRef = sampler.Element(Ns + "source")?.Value.Trim() ?? "";
                        if (!string.IsNullOrEmpty(srcRef)) samplerToSurface[sid] = srcRef;
                    }
                }

                // Find the diffuse texture reference first; fall back to any texture
                XElement? texElem = effect.Descendants(Ns + "texture")
                    .FirstOrDefault(t => t.Ancestors().Any(a => a.Name.LocalName == "diffuse"));
                texElem ??= effect.Descendants(Ns + "texture").FirstOrDefault();

                if (texElem == null) continue;
                string texRef = texElem.Attribute("texture")?.Value ?? "";

                string? filename = null;
                // sampler → surface → image
                if (samplerToSurface.TryGetValue(texRef, out string? surfaceId) &&
                    surfaceToImage.TryGetValue(surfaceId, out string? imageId) &&
                    imageFiles.TryGetValue(imageId, out string? fn))
                {
                    filename = fn;
                }
                // direct image reference
                else if (imageFiles.TryGetValue(texRef, out string? directFn))
                {
                    filename = directFn;
                }

                if (filename != null)
                    effectToFilename[effectId] = filename;
            }

            // Step 3 — material id → effect id
            var materialToEffect = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (var mat in root.Descendants(Ns + "material"))
            {
                string matId = mat.Attribute("id")?.Value ?? "";
                string effectUrl = mat.Element(Ns + "instance_effect")?
                    .Attribute("url")?.Value?.TrimStart('#') ?? "";
                if (!string.IsNullOrEmpty(matId) && !string.IsNullOrEmpty(effectUrl))
                    materialToEffect[matId] = effectUrl;
            }

            // Step 4 — visual_scene bind_material: symbol → material id → filename
            foreach (var instMat in root.Descendants(Ns + "instance_material"))
            {
                string symbol = instMat.Attribute("symbol")?.Value ?? "";
                string target = instMat.Attribute("target")?.Value?.TrimStart('#') ?? "";
                if (string.IsNullOrEmpty(symbol) || string.IsNullOrEmpty(target)) continue;

                if (materialToEffect.TryGetValue(target, out string? effectId) &&
                    effectToFilename.TryGetValue(effectId, out string? filename))
                {
                    result.TryAdd(symbol, filename);
                }
            }
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"[ColladaLoader] LoadTextureMap error: {ex.Message}");
        }

        return result;
    }

    private static float[] ParseFloatArray(string value)
    {
        var parts = value.Split((char[])null!, StringSplitOptions.RemoveEmptyEntries);
        var result = new float[parts.Length];
        for (int i = 0; i < parts.Length; i++)
            float.TryParse(parts[i], System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out result[i]);
        return result;
    }

    private static int[] ParseIntArray(string value)
    {
        var parts = value.Split((char[])null!, StringSplitOptions.RemoveEmptyEntries);
        var result = new int[parts.Length];
        for (int i = 0; i < parts.Length; i++)
            int.TryParse(parts[i], out result[i]);
        return result;
    }
}
