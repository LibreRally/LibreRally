using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using LibreRally.Vehicle.Rendering;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Rendering;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;

namespace LibreRally.Vehicle;

/// <summary>
/// High-level vehicle loader.
/// </summary>
public class VehicleLoader
{
    private static readonly Logger Log = GlobalLogger.GetLogger("VehicleLoader");
    private readonly GraphicsDevice _graphicsDevice;

    public VehicleLoader(Game game)
    {
        _graphicsDevice = game.GraphicsDevice;
    }

    /// <summary>
    /// Loads the vehicle at <paramref name="vehicleFolderPath"/>, builds physics and mesh entities,
    /// and returns the assembled <see cref="LoadedVehicle"/>.
    /// </summary>
    /// <param name="vehicleFolderPath">Absolute path to the vehicle folder containing .jbeam files.</param>
    /// <param name="configFileName">
    /// Optional .pc config file name (e.g. "rally_pro_asphalt.pc") or base name without extension.
    /// If null, auto-detects: prefers rally_pro_asphalt.pc, then first .pc file found.
    /// </param>
    public LoadedVehicle Load(string vehicleFolderPath, string? configFileName = null)
    {
        if (!Directory.Exists(vehicleFolderPath))
            throw new DirectoryNotFoundException($"Vehicle folder not found: '{vehicleFolderPath}'");

        // 0. Load .pc config (parts selection + physics variables)
        PcConfig? pcConfig = null;
        string? pcPath = PcConfigLoader.FindBestConfig(vehicleFolderPath, configFileName);
        if (pcPath != null)
        {
            try
            {
                pcConfig = PcConfigLoader.Load(pcPath);
                Log.Info($"[VehicleLoader] PC config: {Path.GetFileName(pcPath)} | " +
                         $"parts={pcConfig.Parts.Count} vars={pcConfig.Vars.Count}");
            }
            catch (Exception ex)
            {
                Log.Warning($"[VehicleLoader] Failed to parse .pc config '{pcPath}': {ex.Message}");
            }
        }
        else
        {
            Log.Warning("[VehicleLoader] No .pc config file found — using jbeam defaults.");
        }

        // 1. Parse + assemble jbeam (with pc config for parts selection + variable substitution)
        VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolderPath, pcConfig);

        // 2. Build physics entity hierarchy
        VehicleBuilderResult result = VehiclePhysicsBuilder.Build(definition);
        Entity rootEntity = result.RootEntity;

        // 3. Attach visual meshes (best-effort — falls back to a visible box)
        TryAttachMeshes(vehicleFolderPath, result.ChassisEntity, definition);

        // 4. Wire up the rally car driving component
        float V(string name, float fallback) =>
            definition.Vars.TryGetValue(name, out float v) && v > 0 ? v : fallback;

        float finalDrive  = V("finaldrive_F", 4.55f);
        float wheelRadius = 0.305f;
        float maxRpm      = V("maxRPM",  7500f);
        float idleRpm     = V("idleRPM",  900f);
        float peakTorque  = 222f; // sunburst2 2.0T rally engine peak Nm (crank)

        // Build gear ratio array: index 0 = reverse, 1-6 = forward gears
        float gearR = V("gear_R", 3.25f);
        float[] gears = {
            gearR,
            V("gear_1", 3.64f), V("gear_2", 2.38f), V("gear_3", 1.76f),
            V("gear_4", 1.35f), V("gear_5", 1.06f), V("gear_6", 0.84f),
        };

        var car = new RallyCarComponent
        {
            CarBody = result.ChassisEntity,
            Wheels       = { result.WheelFL, result.WheelFR, result.WheelRL, result.WheelRR },
            SteerWheels  = { result.WheelFL, result.WheelFR },
            DriveWheels  = { result.WheelFL, result.WheelFR, result.WheelRL, result.WheelRR },
            BreakWheels  = { result.WheelFL, result.WheelFR, result.WheelRL, result.WheelRR },
            WheelRadius  = wheelRadius,
            GearRatios   = gears,
            FinalDrive   = finalDrive,
            MaxRpm       = maxRpm,
            IdleRpm      = idleRpm,
        };
        rootEntity.Add(car);

        Log.Info($"[VehicleLoader] Gears: R={gearR:F2} " +
                 string.Join(" ", gears.Skip(1).Select((g, i) => $"{i+1}={g:F2}")) +
                 $" | FD={finalDrive:F2} | MaxRPM={maxRpm:F0} | PeakTorque={peakTorque:F0}Nm");

        return new LoadedVehicle(definition, rootEntity, car, result.ChassisEntity, result.WheelFL, result.WheelFR, result.WheelRL, result.WheelRR);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Mesh attachment
    // ──────────────────────────────────────────────────────────────────────────

    private void TryAttachMeshes(string folder, Entity chassisEntity, VehicleDefinition definition)
    {
        var daeFiles = Directory.GetFiles(folder, "*.dae", SearchOption.TopDirectoryOnly);
        if (daeFiles.Length > 0)
        {
            string daeFile = daeFiles[0];
            try
            {
                // Level 1: BeamNG *.materials.json — most accurate, full path resolution
                string vehiclesRoot = Path.GetDirectoryName(folder)!;
                var jsonMaterials = BeamNGMaterialLoader.LoadMaterialTextures(folder, vehiclesRoot);

                // Level 2: Collada library_images chain — filename-based fallback
                var colladaTextureMap = ColladaLoader.LoadTextureMap(daeFile);

                var colladaMeshes = ColladaLoader.Load(daeFile);
                Log.Info($"DAE: {colladaMeshes.Count} sub-meshes | " +
                         $"JSON materials: {jsonMaterials.Count} | " +
                         $"Collada textures: {colladaTextureMap.Count}");

                var meshEntity = BuildMeshEntity(
                    colladaMeshes, definition.VehicleName,
                    folder, vehiclesRoot,
                    jsonMaterials, colladaTextureMap);

                if (meshEntity != null)
                {
                    // DAE vertices are in BeamNG world space (converted to Stride coords).
                    // The chassis entity sits at its node centroid, so counter-offset the mesh
                    // entity so its vertex positions map to the correct world locations.
                    meshEntity.Transform.Position = -chassisEntity.Transform.Position;
                    chassisEntity.AddChild(meshEntity);
                    return;
                }
            }
            catch (Exception ex)
            {
                Log.Error($"Could not load mesh: {ex.Message}");
            }
        }

        Log.Warning("Using fallback orange box mesh.");
        var fallback = BuildFallbackChassisModel(definition);
        if (fallback != null)
        {
            var fallbackEntity = new Entity("chassis_fallback_mesh");
            fallbackEntity.Add(new ModelComponent { Model = fallback });
            chassisEntity.AddChild(fallbackEntity);
        }
    }

    /// <summary>
    /// Groups sub-meshes by material symbol; for each group merges geometry into one GPU buffer
    /// and resolves a texture via three-level lookup:
    ///   1. BeamNG *.materials.json  (full path, most accurate)
    ///   2. Collada library_images chain  (filename in Textures/)
    ///   3. Grey placeholder  (unresolvable, e.g. sunburst base-car textures not in the mod pack)
    /// </summary>
    private Entity? BuildMeshEntity(
        List<ColladaMesh> colladaMeshes,
        string vehicleName,
        string vehicleFolder,
        string vehiclesRoot,
        Dictionary<string, string> jsonMaterials,
        Dictionary<string, string> colladaTextureMap)
    {
        if (colladaMeshes.Count == 0) return null;

        var groups = colladaMeshes
            .Where(cm => cm.Vertices.Count > 0 && cm.Indices.Count > 0)
            .GroupBy(cm => cm.MaterialName)
            .ToList();

        if (groups.Count == 0) return null;

        var rootEntity = new Entity($"{vehicleName}_mesh");
        int built = 0;

        foreach (var group in groups)
        {
            string symbol = group.Key;

            // Merge all sub-meshes that share this material symbol
            var allVerts = new List<VertexPositionNormalTexture>();
            var allIndices = new List<int>();
            foreach (var cm in group)
            {
                int baseIndex = allVerts.Count;
                allVerts.AddRange(ConvertToVertexArray(cm.Vertices));
                foreach (int idx in cm.Indices)
                    allIndices.Add(idx + baseIndex);
            }
            if (allVerts.Count == 0) continue;

            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            foreach (var v in allVerts) { min = Vector3.Min(min, v.Position); max = Vector3.Max(max, v.Position); }

            var mesh = new Mesh
            {
                BoundingBox = new BoundingBox(min, max),
                Draw = new MeshDraw
                {
                    PrimitiveType = PrimitiveType.TriangleList,
                    VertexBuffers = new[] { new VertexBufferBinding(
                        Stride.Graphics.Buffer.Vertex.New(_graphicsDevice, allVerts.ToArray(), GraphicsResourceUsage.Immutable),
                        VertexPositionNormalTexture.Layout, allVerts.Count) },
                    IndexBuffer = new IndexBufferBinding(
                        Stride.Graphics.Buffer.Index.New(_graphicsDevice, allIndices.ToArray()), true, allIndices.Count),
                    DrawCount = allIndices.Count,
                }
            };

            // ── Three-level texture lookup ────────────────────────────────────
            // 1. BeamNG JSON: strip "-material" suffix → exact material name
            string matName = StripMaterialSuffix(symbol);
            Texture? texture = null;

            if (jsonMaterials.TryGetValue(matName, out string? absolutePath))
                texture = TryLoadTextureFromPath(absolutePath);

            // 2. Collada library_images chain (filename only, look in Textures/)
            if (texture == null && colladaTextureMap.TryGetValue(symbol, out string? colladaFile))
                texture = TryLoadTexture(vehicleFolder, colladaFile);

            // 3. Grey placeholder — covers sunburst base-car materials and anything else missing
            var material = BuildMaterial(texture);

            var subEntity = new Entity(symbol);
            subEntity.Add(new ModelComponent { Model = new Model { mesh, material } });
            rootEntity.AddChild(subEntity);
            built++;
        }

        Log.Info($"Mesh: {built} material groups ({colladaMeshes.Count} sub-meshes)");
        return built > 0 ? rootEntity : null;
    }

    private static string StripMaterialSuffix(string symbol)
    {
        string s = symbol;
        if (s.EndsWith("-material", StringComparison.OrdinalIgnoreCase))
            s = s[..^"-material".Length];
        else if (s.EndsWith("_material", StringComparison.OrdinalIgnoreCase))
            s = s[..^"_material".Length];
        return s;
    }

    private Material BuildMaterial(Texture? texture)
    {
        IComputeColor diffuse = texture != null
            ? new ComputeTextureColor(texture)
            : new ComputeColor(new Color4(0.55f, 0.55f, 0.55f, 1f));

        return Material.New(_graphicsDevice, new MaterialDescriptor
        {
            Attributes = new MaterialAttributes
            {
                Diffuse = new MaterialDiffuseMapFeature(diffuse),
                DiffuseModel = new MaterialDiffuseLambertModelFeature(),
            }
        });
    }

    /// <summary>Loads a texture from an absolute file path.</summary>
    private Texture? TryLoadTextureFromPath(string absolutePath)
    {
        if (!File.Exists(absolutePath)) return null;
        try
        {
            using var stream = File.OpenRead(absolutePath);
            var image = Image.Load(stream);
            if (image == null) return null;
            var tex = Texture.New(_graphicsDevice, image);
            image.Dispose();
            Log.Info($"Texture loaded: {Path.GetFileName(absolutePath)}");
            return tex;
        }
        catch (Exception ex)
        {
            Log.Warning($"Texture failed '{Path.GetFileName(absolutePath)}': {ex.Message}");
            return null;
        }
    }

    /// <summary>
    /// Looks up a texture by filename in the vehicle folder (root or Textures/ subfolder).
    /// <paramref name="filename"/> may already include an extension (from Collada map) or be a base name.
    /// Handles BeamNG DAE files that reference .color.png names when actual assets are .color.dds.
    /// </summary>
    private Texture? TryLoadTexture(string vehicleFolder, string filename)
    {
        // Search directories: root folder first, then Textures/ subfolder
        var searchDirs = new List<string> { vehicleFolder };
        string texturesDir = Path.Combine(vehicleFolder, "Textures");
        if (Directory.Exists(texturesDir)) searchDirs.Add(texturesDir);

        foreach (string dir in searchDirs)
        {
            // 1. Try exact match if it has an extension
            if (!string.IsNullOrEmpty(Path.GetExtension(filename)))
            {
                string direct = Path.Combine(dir, filename);
                if (File.Exists(direct)) return TryLoadTextureFromPath(direct);

                // 2. Try replacing the extension — DAE often says .color.png but files are .color.dds
                string noExt = Path.GetFileNameWithoutExtension(filename);
                foreach (string ext in new[] { ".dds", ".png", ".jpg", ".jpeg", ".tga" })
                {
                    string path = Path.Combine(dir, noExt + ext);
                    if (File.Exists(path)) return TryLoadTextureFromPath(path);
                }
            }
            else
            {
                // 3. No extension — try appending common extensions
                foreach (string ext in new[] { ".dds", ".png", ".jpg", ".jpeg", ".tga" })
                {
                    string path = Path.Combine(dir, filename + ext);
                    if (!File.Exists(path))
                    {
                        var found = Directory.GetFiles(dir, filename + ext, SearchOption.TopDirectoryOnly);
                        if (found.Length == 0) continue;
                        path = found[0];
                    }
                    var tex = TryLoadTextureFromPath(path);
                    if (tex != null) return tex;
                }
            }
        }
        return null;
    }

    private static VertexPositionNormalTexture[] ConvertToVertexArray(
        System.Collections.Generic.List<ColladaVertex> vertices)
    {
        var result = new VertexPositionNormalTexture[vertices.Count];
        for (int i = 0; i < vertices.Count; i++)
        {
            var v = vertices[i];
            // Convert BeamNG → Stride coordinate space
            var pos = VehiclePhysicsBuilder.BeamNGToStride(v.Position);
            var norm = VehiclePhysicsBuilder.BeamNGToStride(v.Normal);
            result[i] = new VertexPositionNormalTexture(
                new Vector3(pos.X, pos.Y, pos.Z),
                new Vector3(norm.X, norm.Y, norm.Z),
                // Collada uses OpenGL UV convention (V=0 at bottom); DirectX/Stride uses V=0 at top.
                new Vector2(v.TexCoord.X, 1.0f - v.TexCoord.Y));
        }
        return result;
    }

    /// <summary>Creates a bright orange box model sized to the chassis AABB as a fallback mesh.</summary>
    private Model? BuildFallbackChassisModel(VehicleDefinition definition)
    {
        try
        {
            var chassisPart = definition.Parts.FirstOrDefault(p => !p.Detachable);
            if (chassisPart == null) return null;

            var nodes = chassisPart.ExclusiveNodeIds
                .Where(id => definition.Nodes.ContainsKey(id))
                .Select(id => definition.Nodes[id])
                .ToList();
            if (nodes.Count == 0) return null;

            // Size from AABB of chassis nodes (Stride coords)
            var positions = nodes.Select(n => VehiclePhysicsBuilder.BeamNGToStride(n.Position)).ToList();
            var min = positions[0]; var max = positions[0];
            foreach (var p in positions)
            {
                min = Vector3.Min(min, p);
                max = Vector3.Max(max, p);
            }
            var centroid = (min + max) * 0.5f;
            var size = max - min;

            // Build vertices centered at origin (mesh is child of chassis entity which is at centroid)
            var half = size * 0.5f;
            var verts = BuildBoxVertices(half.X, half.Y, half.Z);
            var indices = BuildBoxIndices();

            // Offset all vertex positions by (localMeshOffset = min - centroid_of_mesh_local)
            // The chassis entity is at the centroid of its nodes, so mesh local offset should be zero
            var mesh = new Mesh
            {
                BoundingBox = new BoundingBox(-half, half),
                Draw = new MeshDraw
                {
                    PrimitiveType = PrimitiveType.TriangleList,
                    VertexBuffers = new[] { new VertexBufferBinding(
                        Stride.Graphics.Buffer.Vertex.New(_graphicsDevice, verts, GraphicsResourceUsage.Immutable),
                        VertexPositionNormalTexture.Layout, verts.Length) },
                    IndexBuffer = new IndexBufferBinding(
                        Stride.Graphics.Buffer.Index.New(_graphicsDevice, indices), true, indices.Length),
                    DrawCount = indices.Length,
                }
            };

            var mat = Material.New(_graphicsDevice, new MaterialDescriptor
            {
                Attributes = new MaterialAttributes
                {
                    Diffuse = new MaterialDiffuseMapFeature(new ComputeColor(new Color4(1f, 0.4f, 0f, 1f))),
                    DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                }
            });

            return new Model { mesh, mat };
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine($"[VehicleLoader] Fallback mesh failed: {ex.Message}");
            return null;
        }
    }

    private static VertexPositionNormalTexture[] BuildBoxVertices(float hx, float hy, float hz)
    {
        var verts = new List<VertexPositionNormalTexture>();
        void Face(Vector3 n, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            verts.Add(new(a, n, new(0, 1)));
            verts.Add(new(b, n, new(1, 1)));
            verts.Add(new(c, n, new(1, 0)));
            verts.Add(new(d, n, new(0, 0)));
        }
        Face(new(0,0,1),  new(-hx,-hy, hz), new( hx,-hy, hz), new( hx, hy, hz), new(-hx, hy, hz));
        Face(new(0,0,-1), new( hx,-hy,-hz), new(-hx,-hy,-hz), new(-hx, hy,-hz), new( hx, hy,-hz));
        Face(new(0,1,0),  new(-hx, hy, hz), new( hx, hy, hz), new( hx, hy,-hz), new(-hx, hy,-hz));
        Face(new(0,-1,0), new(-hx,-hy,-hz), new( hx,-hy,-hz), new( hx,-hy, hz), new(-hx,-hy, hz));
        Face(new(1,0,0),  new( hx,-hy, hz), new( hx,-hy,-hz), new( hx, hy,-hz), new( hx, hy, hz));
        Face(new(-1,0,0), new(-hx,-hy,-hz), new(-hx,-hy, hz), new(-hx, hy, hz), new(-hx, hy,-hz));
        return verts.ToArray();
    }

    private static int[] BuildBoxIndices()
    {
        var idx = new List<int>();
        for (int f = 0; f < 6; f++)
        {
            int b = f * 4;
            idx.AddRange(new[] { b, b+1, b+2, b, b+2, b+3 });
        }
        return idx.ToArray();
    }
}

/// <summary>A loaded vehicle with its root entity and assembled definition.</summary>
public record LoadedVehicle(VehicleDefinition Definition, Entity RootEntity, RallyCarComponent CarComponent, Entity ChassisEntity, Entity WheelFL, Entity WheelFR, Entity WheelRL, Entity WheelRR);
