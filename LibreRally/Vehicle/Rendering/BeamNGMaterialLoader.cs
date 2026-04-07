using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;

namespace LibreRally.Vehicle.Rendering;

/// <summary>
/// Parses BeamNG *.materials.json files and resolves material names to absolute texture paths.
///
/// BeamNG material convention:
///   - Material name (JSON key) matches the DAE binding symbol without the "-material" suffix
///   - Texture paths use the form  "/vehicles/&lt;name&gt;/..." or "vehicles/&lt;name&gt;/..."
///     where &lt;name&gt; is relative to the BeamNG vehicles root directory
///   - "null.dds" / "null2.dds" etc. are placeholder entries meaning "no texture"
///   - Each material has "Stages" array; Stage 0 (or first non-empty stage) holds the base colour
/// </summary>
public static class BeamNGMaterialLoader
{
    private static readonly JsonDocumentOptions ParseOptions = new()
    {
        AllowTrailingCommas = true,
        CommentHandling = JsonCommentHandling.Skip,
    };

    /// <summary>
    /// Scans every *.materials.json in <paramref name="vehicleFolder"/> and builds a map
    /// from material name → absolute path of the best diffuse texture.
    /// <para>
    /// <paramref name="vehiclesRootDir"/> is the parent directory of all vehicle folders,
    /// e.g. the "BeamNG Vehicles" directory.  It is used to resolve /vehicles/… paths.
    /// </para>
    /// </summary>
    public static Dictionary<string, string> LoadMaterialTextures(
        string vehicleFolder,
        string vehiclesRootDir)
    {
        var result = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);

        foreach (var jsonFile in Directory.GetFiles(vehicleFolder, "*.materials.json",
                     SearchOption.TopDirectoryOnly))
        {
            try
            {
                ParseFile(jsonFile, vehiclesRootDir, result);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[BeamNGMaterialLoader] Could not parse '{Path.GetFileName(jsonFile)}': {ex.Message}");
            }
        }

        return result;
    }

    // ──────────────────────────────────────────────────────────────────────────

    private static void ParseFile(
        string jsonFile,
        string vehiclesRootDir,
        Dictionary<string, string> result)
    {
        var text = File.ReadAllText(jsonFile);
        using var doc = JsonDocument.Parse(text, ParseOptions);

        foreach (var matEntry in doc.RootElement.EnumerateObject())
        {
            string matName = matEntry.Name;
            // Skip metadata keys (not material definitions)
            if (matName is "version" or "persistentId") continue;

            var matElem = matEntry.Value;
            if (matElem.ValueKind != JsonValueKind.Object) continue;

            string? texPath = FindBestTexturePath(matElem);
            if (texPath == null) continue;

            string? resolved = ResolveVehiclePath(texPath, vehiclesRootDir);
            if (!string.IsNullOrEmpty(resolved))
                result.TryAdd(matName, resolved);
        }
    }

    /// <summary>
    /// Returns the first non-null diffuse/baseColor texture path found in any stage,
    /// skipping null placeholder textures. Returns null if none found.
    /// </summary>
    private static string? FindBestTexturePath(JsonElement material)
    {
        if (!material.TryGetProperty("Stages", out var stages)) return null;

        foreach (var stage in stages.EnumerateArray())
        {
            if (stage.ValueKind != JsonValueKind.Object) continue;

            // PBR format (version 1.5+): baseColorMap
            if (TryGetNonNullPath(stage, "baseColorMap", out string? path)) return path;
            // Legacy format: colorMap
            if (TryGetNonNullPath(stage, "colorMap", out path)) return path;
            // Diffuse colour only — skip; no texture file to load
        }

        return null;
    }

    private static bool TryGetNonNullPath(JsonElement stage, string key, out string? path)
    {
        path = null;
        if (!stage.TryGetProperty(key, out var elem)) return false;
        if (elem.ValueKind != JsonValueKind.String) return false;

        string raw = elem.GetString()!;
        // Ignore BeamNG placeholder textures
        if (IsNullTexture(raw)) return false;

        path = raw;
        return true;
    }

    private static bool IsNullTexture(string path)
    {
        string filename = Path.GetFileNameWithoutExtension(path).ToLowerInvariant();
        return filename.StartsWith("null") || filename == "desktop";
    }

    /// <summary>
    /// Converts a BeamNG virtual path like "/vehicles/RLA_Evo/Textures/carpet.dds"
    /// to an absolute file system path using <paramref name="vehiclesRootDir"/> as the
    /// mount point for the /vehicles/ prefix.
    /// Returns null if the resolved file does not exist.
    /// </summary>
    private static string? ResolveVehiclePath(string vehiclePath, string vehiclesRootDir)
    {
        if (string.IsNullOrWhiteSpace(vehiclePath)) return null;

        // Normalise: remove leading slashes, convert forward-slashes
        string normalised = vehiclePath.TrimStart('/', '\\');

        // Strip the "vehicles/" prefix that BeamNG uses as a virtual root
        const string vehiclesPrefix = "vehicles/";
        if (normalised.StartsWith(vehiclesPrefix, StringComparison.OrdinalIgnoreCase))
            normalised = normalised[vehiclesPrefix.Length..];

        string fullPath = Path.Combine(vehiclesRootDir,
            normalised.Replace('/', Path.DirectorySeparatorChar));

        if (File.Exists(fullPath)) return fullPath;

        // Fallback: if the original reference was a .dds, try .png (for copyright-free placeholders)
        if (Path.GetExtension(fullPath).Equals(".dds", StringComparison.OrdinalIgnoreCase))
        {
            string pngPath = Path.ChangeExtension(fullPath, ".png");
            if (File.Exists(pngPath)) return pngPath;
        }

        return null;
    }
}
