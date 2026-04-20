using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Xml.Linq;

namespace LibreRally.Vehicle.Rendering;

/// <summary>Vertex with <paramref name="Position" />, <paramref name="Normal" />, and UV coordinates.</summary>
public record ColladaVertex(Vector3 Position, Vector3 Normal, Vector2 TexCoord);

/// <summary>A single mesh sub-object from a Collada file, identified by geometry name.</summary>
public class ColladaMesh
{
    public string Name { get; init; } = "";
    public string GeometryName { get; init; } = "";
    public string MaterialName { get; init; } = "";
    public string SceneNodeName { get; init; } = "";
    public bool HasBakedTransform { get; init; }
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
    private sealed record ColladaGeometry(string Id, string Name, List<ColladaMesh> Meshes);

    public static List<ColladaMesh> Load(string daeFilePath)
    {
        var doc = XDocument.Load(daeFilePath);
        var root = doc.Root ?? throw new InvalidDataException("Empty Collada file.");

        var geometryLibrary = LoadGeometryLibrary(root);
        if (geometryLibrary.Count == 0)
        {
	        return new List<ColladaMesh>();
        }

        var sceneMeshes = LoadSceneMeshes(root, geometryLibrary);
        var filteredSceneMeshes = FilterByExtent(sceneMeshes);
        if (filteredSceneMeshes.Count > 0)
        {
	        return filteredSceneMeshes;
        }

        return FilterByExtent(geometryLibrary.Values.SelectMany(geometry => geometry.Meshes));
    }

    // BeamNG morph-target/helper geometry can still span many metres after scene transforms.
    // Filter it after baking node transforms so authored wheel/brake meshes with large raw
    // coordinates are not discarded before their scene scale/offset is applied.
    private const float MaxGeomExtentMetres = 7f;

    private static Dictionary<string, ColladaGeometry> LoadGeometryLibrary(XElement root)
    {
        var result = new Dictionary<string, ColladaGeometry>(StringComparer.OrdinalIgnoreCase);
        var geometryLib = root.Element(Ns + "library_geometries");
        if (geometryLib == null)
        {
	        return result;
        }

        foreach (var geometry in geometryLib.Elements(Ns + "geometry"))
        {
            var geometryId = geometry.Attribute("id")?.Value
                             ?? geometry.Attribute("name")?.Value
                             ?? "";
            if (string.IsNullOrWhiteSpace(geometryId))
            {
	            continue;
            }

            var geometryName = geometry.Attribute("name")?.Value
                               ?? geometryId;

            var mesh = geometry.Element(Ns + "mesh");
            if (mesh == null)
            {
	            continue;
            }

            result[geometryId] = new ColladaGeometry(
                geometryId,
                geometryName,
                ParseMesh(geometryName, mesh).ToList());
        }

        return result;
    }

    private static List<ColladaMesh> LoadSceneMeshes(
        XElement root,
        IReadOnlyDictionary<string, ColladaGeometry> geometryLibrary)
    {
        var sceneLibrary = root.Element(Ns + "library_visual_scenes");
        if (sceneLibrary == null)
        {
	        return new List<ColladaMesh>();
        }

        var visualScenes = sceneLibrary.Elements(Ns + "visual_scene");
        var activeSceneId = root.Element(Ns + "scene")?
            .Element(Ns + "instance_visual_scene")?
            .Attribute("url")?
            .Value?
            .TrimStart('#');

        if (!string.IsNullOrWhiteSpace(activeSceneId))
        {
            var activeVisualScenes = visualScenes
                .Where(visualScene => string.Equals(
                    visualScene.Attribute("id")?.Value,
                    activeSceneId,
                    StringComparison.OrdinalIgnoreCase))
                .ToList();

            if (activeVisualScenes.Count > 0)
            {
	            visualScenes = activeVisualScenes;
            }
        }

        var result = new List<ColladaMesh>();
        foreach (var visualScene in visualScenes)
        {
            foreach (var node in visualScene.Elements(Ns + "node"))
                AddSceneMeshes(node, Matrix4x4.Identity, geometryLibrary, result);
        }

        return result;
    }

    private static void AddSceneMeshes(
        XElement node,
        Matrix4x4 parentTransform,
        IReadOnlyDictionary<string, ColladaGeometry> geometryLibrary,
        List<ColladaMesh> result)
    {
        var localTransform = ParseNodeTransform(node);
        var worldTransform = Matrix4x4.Multiply(localTransform, parentTransform);
        var nodeName = node.Attribute("name")?.Value
                       ?? node.Attribute("id")?.Value
                       ?? "";

        foreach (var instanceGeometry in node.Elements(Ns + "instance_geometry"))
        {
            var geometryId = instanceGeometry.Attribute("url")?.Value?.TrimStart('#') ?? "";
            if (string.IsNullOrWhiteSpace(geometryId) ||
                !geometryLibrary.TryGetValue(geometryId, out var geometry) ||
                ShouldSkipSceneGeometry(nodeName, geometryId, geometry.Name))
            {
                continue;
            }

            foreach (var mesh in geometry.Meshes)
                result.Add(BakeSceneTransform(mesh, worldTransform, nodeName));
        }

        foreach (var childNode in node.Elements(Ns + "node"))
            AddSceneMeshes(childNode, worldTransform, geometryLibrary, result);
    }

    private static Matrix4x4 ParseNodeTransform(XElement node)
    {
        var matrixElement = node.Element(Ns + "matrix");
        if (matrixElement == null)
        {
	        return Matrix4x4.Identity;
        }

        var values = ParseFloatArray(matrixElement.Value);
        if (values.Length != 16)
        {
	        return Matrix4x4.Identity;
        }

        // BeamNG serializes Collada node transforms with translation in the last column.
        // Transpose into System.Numerics row-vector layout so Vector3.Transform applies them correctly.
        return new Matrix4x4(
            values[0], values[4], values[8], values[12],
            values[1], values[5], values[9], values[13],
            values[2], values[6], values[10], values[14],
            values[3], values[7], values[11], values[15]);
    }

    private static ColladaMesh BakeSceneTransform(
        ColladaMesh sourceMesh,
        Matrix4x4 transform,
        string sceneNodeName)
    {
        var hasBakedTransform = !IsApproximatelyIdentity(transform);
        var normalTransform = transform;
        if (hasBakedTransform && Matrix4x4.Invert(transform, out var inverseTransform))
        {
	        normalTransform = Matrix4x4.Transpose(inverseTransform);
        }

        var vertices = new List<ColladaVertex>(sourceMesh.Vertices.Count);
        foreach (var vertex in sourceMesh.Vertices)
        {
            var position = hasBakedTransform
                ? Vector3.Transform(vertex.Position, transform)
                : vertex.Position;

            var normal = hasBakedTransform
                ? Vector3.TransformNormal(vertex.Normal, normalTransform)
                : vertex.Normal;
            if (normal.LengthSquared() > 1e-8f)
            {
	            normal = Vector3.Normalize(normal);
            }
            else
            {
	            normal = Vector3.UnitY;
            }

            vertices.Add(new ColladaVertex(position, normal, vertex.TexCoord));
        }

        return new ColladaMesh
        {
            Name = string.IsNullOrWhiteSpace(sceneNodeName)
                ? sourceMesh.Name
                : $"{sceneNodeName}_{sourceMesh.Name}",
            GeometryName = sourceMesh.GeometryName,
            MaterialName = sourceMesh.MaterialName,
            SceneNodeName = sceneNodeName,
            HasBakedTransform = hasBakedTransform,
            Vertices = vertices,
            Indices = ShouldFlipTriangleWinding(transform)
                ? FlipTriangleWinding(sourceMesh.Indices)
                : new List<int>(sourceMesh.Indices),
        };
    }

    private static bool ShouldSkipSceneGeometry(string nodeName, string geometryId, string geometryName)
        => IsHelperGeometryName(nodeName) ||
           IsHelperGeometryName(geometryId) ||
           IsHelperGeometryName(geometryName);

    private static bool IsHelperGeometryName(string value)
        => !string.IsNullOrWhiteSpace(value) &&
           (value.Contains("_empty", StringComparison.OrdinalIgnoreCase) ||
            value.Contains("_helper", StringComparison.OrdinalIgnoreCase));

    private static bool ShouldFlipTriangleWinding(Matrix4x4 transform)
        => !IsApproximatelyIdentity(transform) && transform.GetDeterminant() < 0f;

    private static List<int> FlipTriangleWinding(List<int> indices)
    {
        var result = new List<int>(indices.Count);
        var triangleCount = indices.Count / 3;
        for (var i = 0; i < triangleCount; i++)
        {
            var baseIndex = i * 3;
            result.Add(indices[baseIndex]);
            result.Add(indices[baseIndex + 2]);
            result.Add(indices[baseIndex + 1]);
        }

        for (var i = triangleCount * 3; i < indices.Count; i++)
            result.Add(indices[i]);

        return result;
    }

    private static bool IsApproximatelyIdentity(Matrix4x4 matrix)
    {
        const float Epsilon = 1e-4f;
        return MathF.Abs(matrix.M11 - 1f) < Epsilon &&
               MathF.Abs(matrix.M12) < Epsilon &&
               MathF.Abs(matrix.M13) < Epsilon &&
               MathF.Abs(matrix.M14) < Epsilon &&
               MathF.Abs(matrix.M21) < Epsilon &&
               MathF.Abs(matrix.M22 - 1f) < Epsilon &&
               MathF.Abs(matrix.M23) < Epsilon &&
               MathF.Abs(matrix.M24) < Epsilon &&
               MathF.Abs(matrix.M31) < Epsilon &&
               MathF.Abs(matrix.M32) < Epsilon &&
               MathF.Abs(matrix.M33 - 1f) < Epsilon &&
               MathF.Abs(matrix.M34) < Epsilon &&
               MathF.Abs(matrix.M41) < Epsilon &&
               MathF.Abs(matrix.M42) < Epsilon &&
               MathF.Abs(matrix.M43) < Epsilon &&
               MathF.Abs(matrix.M44 - 1f) < Epsilon;
    }

    private static List<ColladaMesh> FilterByExtent(IEnumerable<ColladaMesh> meshes)
        => meshes.Where(IsWithinExtentLimit).ToList();

    private static bool IsWithinExtentLimit(ColladaMesh mesh)
    {
        if (mesh.Vertices.Count == 0)
        {
            return false;
        }

        float minX = float.MaxValue, maxX = float.MinValue;
        float minY = float.MaxValue, maxY = float.MinValue;
        float minZ = float.MaxValue, maxZ = float.MinValue;
        foreach (var vertex in mesh.Vertices)
        {
            if (vertex.Position.X < minX)
            {
                minX = vertex.Position.X;
            }

            if (vertex.Position.X > maxX)
            {
                maxX = vertex.Position.X;
            }

            if (vertex.Position.Y < minY)
            {
                minY = vertex.Position.Y;
            }

            if (vertex.Position.Y > maxY)
            {
                maxY = vertex.Position.Y;
            }

            if (vertex.Position.Z < minZ)
            {
                minZ = vertex.Position.Z;
            }

            if (vertex.Position.Z > maxZ)
            {
                maxZ = vertex.Position.Z;
            }
        }

        return (maxX - minX) <= MaxGeomExtentMetres &&
               (maxY - minY) <= MaxGeomExtentMetres &&
               (maxZ - minZ) <= MaxGeomExtentMetres;
    }

    private static IEnumerable<ColladaMesh> ParseMesh(string geometryName, XElement mesh)
    {
        // Build source arrays keyed by #id
        var sources = new Dictionary<string, float[]>(StringComparer.OrdinalIgnoreCase);
        foreach (var source in mesh.Elements(Ns + "source"))
        {
            var id = "#" + (source.Attribute("id")?.Value ?? "");
            var floatArray = source.Element(Ns + "float_array");
            if (floatArray != null)
            {
                sources[id] = ParseFloatArray(floatArray.Value);
            }
        }

        // Resolve <vertices> indirection
        var verticesElem = mesh.Element(Ns + "vertices");
        var verticesId = "#" + (verticesElem?.Attribute("id")?.Value ?? "");
        var positionSourceId = "";
        if (verticesElem != null)
        {
            foreach (var input in verticesElem.Elements(Ns + "input"))
            {
                if (input.Attribute("semantic")?.Value == "POSITION")
                {
	                positionSourceId = input.Attribute("source")?.Value ?? "";
                }
            }
        }
        sources[verticesId + "_POSITION"] = sources.GetValueOrDefault(positionSourceId, Array.Empty<float>());

        // Process <triangles> and <polylist> primitives
        foreach (var prim in mesh.Elements(Ns + "triangles").Concat(mesh.Elements(Ns + "polylist")))
        {
            var material = prim.Attribute("material")?.Value ?? "";
            var colladaMesh = BuildPrimitiveMesh(
                geometryName,
                $"{geometryName}_{material}",
                material,
                prim,
                sources,
                verticesId);

            if (colladaMesh != null)
            {
	            yield return colladaMesh;
            }
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
        var maxOffset = 0;
        foreach (var input in prim.Elements(Ns + "input"))
        {
            var semantic = input.Attribute("semantic")?.Value ?? "";
            var source = input.Attribute("source")?.Value ?? "";
            var offset = int.TryParse(input.Attribute("offset")?.Value, out var o) ? o : 0;
            inputs[semantic] = (source, offset);
            if (offset > maxOffset)
            {
	            maxOffset = offset;
            }
        }

        var stride = maxOffset + 1;

        var positions = GetSourceByRole(sources, inputs, verticesId, "VERTEX", "POSITION");
        var normals = GetSourceByRole(sources, inputs, null, "NORMAL", null);
        var uvs = GetSourceByRole(sources, inputs, null, "TEXCOORD", null);

        // Parse index list
        var pElem = prim.Element(Ns + "p");
        if (pElem == null)
        {
	        return null;
        }

        var rawIndices = ParseIntArray(pElem.Value);
        if (rawIndices.Length == 0)
        {
	        return null;
        }

        var posOffset = inputs.TryGetValue("VERTEX", out var vIn) ? vIn.Offset : 0;
        var normOffset = inputs.TryGetValue("NORMAL", out var nIn) ? nIn.Offset : -1;
        var uvOffset = inputs.TryGetValue("TEXCOORD", out var uIn) ? uIn.Offset : -1;

        var vertices = new List<ColladaVertex>();
        var indices = new List<int>();
        var vertexMap = new Dictionary<(int, int, int), int>();

        int GetOrAddVertex(int baseIdx)
        {
            var pi = rawIndices[baseIdx + posOffset];
            var ni = normOffset >= 0 ? rawIndices[baseIdx + normOffset] : -1;
            var ui = uvOffset >= 0 ? rawIndices[baseIdx + uvOffset] : -1;

            var key = (pi, ni, ui);
            if (!vertexMap.TryGetValue(key, out var existingIdx))
            {
                existingIdx = vertices.Count;
                vertexMap[key] = existingIdx;

                var pos = positions.Length >= (pi + 1) * 3
                    ? new Vector3(positions[pi * 3], positions[pi * 3 + 1], positions[pi * 3 + 2])
                    : Vector3.Zero;

                var norm = ni >= 0 && normals.Length >= (ni + 1) * 3
                    ? new Vector3(normals[ni * 3], normals[ni * 3 + 1], normals[ni * 3 + 2])
                    : Vector3.UnitY;

                var uv = ui >= 0 && uvs.Length >= (ui + 1) * 2
                    ? new Vector2(uvs[ui * 2], uvs[ui * 2 + 1])
                    : Vector2.Zero;

                vertices.Add(new ColladaVertex(pos, norm, uv));
            }

            return existingIdx;
        }

        if (prim.Name.LocalName.Equals("polylist", StringComparison.OrdinalIgnoreCase))
        {
            var vcountElem = prim.Element(Ns + "vcount");
            if (vcountElem == null)
            {
	            return null;
            }

            var polyVertexCounts = ParseIntArray(vcountElem.Value);
            var cursor = 0;

            foreach (var polyVertCount in polyVertexCounts)
            {
                if (polyVertCount < 3)
                {
                    cursor += polyVertCount * stride;
                    continue;
                }

                var polygonIndexCount = polyVertCount * stride;
                if (cursor + polygonIndexCount > rawIndices.Length)
                {
	                break;
                }

                var first = GetOrAddVertex(cursor);
                var prev = GetOrAddVertex(cursor + stride);

                for (var pv = 2; pv < polyVertCount; pv++)
                {
                    var current = GetOrAddVertex(cursor + pv * stride);
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
            var vertCount = rawIndices.Length / stride;
            for (var vi = 0; vi < vertCount; vi++)
            {
                var baseIdx = vi * stride;
                indices.Add(GetOrAddVertex(baseIdx));
            }
        }

        return new ColladaMesh
        {
            Name = name,
            GeometryName = geometryName,
            MaterialName = material,
            Vertices = vertices,
            // BeamNG's Collada exports use the opposite front-face winding to Stride's default
            // rasterizer, so flip imported primitives once here. If a baked scene transform
            // reflects the mesh, BakeSceneTransform flips again to preserve the visible outside.
            Indices = FlipTriangleWinding(indices),
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
        {
	        return Array.Empty<float>();
        }

        // For VERTEX semantic, resolve through the vertices element
        var key = inputDef.Source;
        if (alternateId != null && key == alternateId && fallbackKey != null)
        {
	        key = alternateId + "_" + fallbackKey;
        }

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
            if (root == null)
            {
	            return result;
            }

            // Step 1 — image id → just the filename (absolute paths from other machines)
            var imageFiles = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (var img in root.Descendants(Ns + "image"))
            {
                var id = img.Attribute("id")?.Value ?? "";
                var initFrom = img.Element(Ns + "init_from");
                if (initFrom == null || string.IsNullOrEmpty(id))
                {
	                continue;
                }

                var rawPath = Uri.UnescapeDataString(initFrom.Value.Trim());
                // Normalise path separators before extracting filename
                rawPath = rawPath.Replace('/', System.IO.Path.DirectorySeparatorChar);
                var filename = System.IO.Path.GetFileName(rawPath);
                if (!string.IsNullOrEmpty(filename))
                {
	                imageFiles[id] = filename;
                }
            }

            // Step 2 — for each effect: resolve sampler→surface→image chain and find diffuse texture
            var effectToFilename = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (var effect in root.Descendants(Ns + "effect"))
            {
                var effectId = effect.Attribute("id")?.Value ?? "";
                if (string.IsNullOrEmpty(effectId))
                {
	                continue;
                }

                // Collect newparam sid → surface init_from image id
                var surfaceToImage = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
                var samplerToSurface = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

                foreach (var newparam in effect.Descendants(Ns + "newparam"))
                {
                    var sid = newparam.Attribute("sid")?.Value ?? "";
                    if (string.IsNullOrEmpty(sid))
                    {
	                    continue;
                    }

                    var surface = newparam.Element(Ns + "surface");
                    if (surface != null)
                    {
                        var imgRef = surface.Element(Ns + "init_from")?.Value.Trim() ?? "";
                        if (!string.IsNullOrEmpty(imgRef))
                        {
	                        surfaceToImage[sid] = imgRef;
                        }
                    }

                    var sampler = newparam.Element(Ns + "sampler2D");
                    if (sampler != null)
                    {
                        var srcRef = sampler.Element(Ns + "source")?.Value.Trim() ?? "";
                        if (!string.IsNullOrEmpty(srcRef))
                        {
	                        samplerToSurface[sid] = srcRef;
                        }
                    }
                }

                // Find the diffuse texture reference first; fall back to any texture
                var texElem = effect.Descendants(Ns + "texture")
                    .FirstOrDefault(t => t.Ancestors().Any(a => a.Name.LocalName == "diffuse"));
                texElem ??= effect.Descendants(Ns + "texture").FirstOrDefault();

                if (texElem == null)
                {
	                continue;
                }

                var texRef = texElem.Attribute("texture")?.Value ?? "";

                string? filename = null;
                // sampler → surface → image
                if (samplerToSurface.TryGetValue(texRef, out var surfaceId) &&
                    surfaceToImage.TryGetValue(surfaceId, out var imageId) &&
                    imageFiles.TryGetValue(imageId, out var fn))
                {
                    filename = fn;
                }
                // direct image reference
                else if (imageFiles.TryGetValue(texRef, out var directFn))
                {
                    filename = directFn;
                }

                if (filename != null)
                {
	                effectToFilename[effectId] = filename;
                }
            }

            // Step 3 — material id → effect id
            var materialToEffect = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (var mat in root.Descendants(Ns + "material"))
            {
                var matId = mat.Attribute("id")?.Value ?? "";
                var effectUrl = mat.Element(Ns + "instance_effect")?
                    .Attribute("url")?.Value?.TrimStart('#') ?? "";
                if (!string.IsNullOrEmpty(matId) && !string.IsNullOrEmpty(effectUrl))
                {
	                materialToEffect[matId] = effectUrl;
                }
            }

            // Step 4 — visual_scene bind_material: symbol → material id → filename
            foreach (var instMat in root.Descendants(Ns + "instance_material"))
            {
                var symbol = instMat.Attribute("symbol")?.Value ?? "";
                var target = instMat.Attribute("target")?.Value?.TrimStart('#') ?? "";
                if (string.IsNullOrEmpty(symbol) || string.IsNullOrEmpty(target))
                {
	                continue;
                }

                if (materialToEffect.TryGetValue(target, out var effectId) &&
                    effectToFilename.TryGetValue(effectId, out var filename))
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
        for (var i = 0; i < parts.Length; i++)
            float.TryParse(parts[i], System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out result[i]);
        return result;
    }

    private static int[] ParseIntArray(string value)
    {
        var parts = value.Split((char[])null!, StringSplitOptions.RemoveEmptyEntries);
        var result = new int[parts.Length];
        for (var i = 0; i < parts.Length; i++)
            int.TryParse(parts[i], out result[i]);
        return result;
    }
}
