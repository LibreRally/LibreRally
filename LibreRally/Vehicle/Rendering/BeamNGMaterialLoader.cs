using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace LibreRally.Vehicle.Rendering;

/// <summary>
/// Loads BeamNG <c>*.materials.json</c> files and resolves material identifiers to concrete diffuse texture
/// file paths so imported meshes can bind textures at runtime.
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
    /// <param name="vehicleFolder">Folder containing the target vehicle's material definition files.</param>
    /// <param name="vehiclesRootDir">Root vehicles directory used to resolve BeamNG virtual texture paths.</param>
    /// <returns>
    /// A dictionary keyed by material name, where each value is the absolute path of the resolved base-color texture.
    /// </returns>
    public static Dictionary<string, string> LoadMaterialTextures(
        string vehicleFolder,
        string vehiclesRootDir)
    {
        return LoadMaterialTextures([vehicleFolder], [vehiclesRootDir], null);
    }

    /// <summary>
    /// Loads material texture mappings from one or more search folders.
    /// </summary>
    /// <param name="materialSearchFolders">Candidate directories to scan for <c>*.materials.json</c> files.</param>
    /// <param name="vehiclesRootDirs">
    /// One or more BeamNG vehicles root directories used to resolve virtual <c>/vehicles/...</c> texture paths.
    /// </param>
    /// <param name="virtualPathResolver">
    /// Optional callback that resolves virtual BeamNG paths when the texture is not found directly on disk.
    /// </param>
    /// <returns>
    /// A dictionary keyed by material name, where each value is the first resolved base-color texture path.
    /// When duplicate material names are discovered, the existing mapping is kept and later matches are ignored.
    /// </returns>
    public static Dictionary<string, string> LoadMaterialTextures(
        IEnumerable<string> materialSearchFolders,
        IEnumerable<string> vehiclesRootDirs,
        Func<string, string?>? virtualPathResolver)
    {
        var result = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        var distinctRootDirs = vehiclesRootDirs
            .Where(path => !string.IsNullOrWhiteSpace(path))
            .Distinct(StringComparer.OrdinalIgnoreCase)
            .ToArray();

        foreach (var searchFolder in materialSearchFolders
                     .Where(path => !string.IsNullOrWhiteSpace(path) && Directory.Exists(path))
                     .Distinct(StringComparer.OrdinalIgnoreCase))
        {
            foreach (var jsonFile in Directory.GetFiles(searchFolder, "*.materials.json",
                         SearchOption.TopDirectoryOnly))
            {
                try
                {
                    ParseFile(jsonFile, distinctRootDirs, virtualPathResolver, result);
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"[BeamNGMaterialLoader] Could not parse '{Path.GetFileName(jsonFile)}': {ex.Message}");
                }
            }
        }

        return result;
    }

    // ──────────────────────────────────────────────────────────────────────────

    private static void ParseFile(
        string jsonFile,
        IReadOnlyList<string> vehiclesRootDirs,
        Func<string, string?>? virtualPathResolver,
        Dictionary<string, string> result)
    {
        var text = File.ReadAllText(jsonFile);
        using var doc = JsonDocument.Parse(text, ParseOptions);

        foreach (var matEntry in doc.RootElement.EnumerateObject())
        {
            var matName = matEntry.Name;
            // Skip metadata keys (not material definitions)
            if (matName is "version" or "persistentId")
            {
	            continue;
            }

            var matElem = matEntry.Value;
            if (matElem.ValueKind != JsonValueKind.Object)
            {
	            continue;
            }

            var texPath = FindBestTexturePath(matElem);
            if (texPath == null)
            {
	            continue;
            }

            var resolved = ResolveVehiclePath(texPath, vehiclesRootDirs, virtualPathResolver);
            if (!string.IsNullOrEmpty(resolved))
            {
	            result.TryAdd(matName, resolved);
            }
        }
    }

    /// <summary>
    /// Returns the first non-null diffuse/baseColor texture path found in any stage,
    /// skipping <see langword="null" /> placeholder textures. Returns <see langword="null" /> if none are found.
    /// </summary>
    private static string? FindBestTexturePath(JsonElement material)
    {
        if (!material.TryGetProperty("Stages", out var stages))
        {
	        return null;
        }

        foreach (var stage in stages.EnumerateArray())
        {
            if (stage.ValueKind != JsonValueKind.Object)
            {
	            continue;
            }

            // PBR format (version 1.5+): baseColorMap
            if (TryGetNonNullPath(stage, "baseColorMap", out var path))
            {
	            return path;
            }

            // Legacy format: colorMap
            if (TryGetNonNullPath(stage, "colorMap", out path))
            {
	            return path;
            }
            // Diffuse colour only — skip; no texture file to load
        }

        return null;
    }

    private static bool TryGetNonNullPath(JsonElement stage, string key, out string? path)
    {
        path = null;
        if (!stage.TryGetProperty(key, out var elem))
        {
	        return false;
        }

        if (elem.ValueKind != JsonValueKind.String)
        {
	        return false;
        }

        var raw = elem.GetString()!;
        // Ignore BeamNG placeholder textures
        if (IsNullTexture(raw))
        {
	        return false;
        }

        path = raw;
        return true;
    }

    private static bool IsNullTexture(string path)
    {
        var filename = Path.GetFileNameWithoutExtension(path).ToLowerInvariant();
        return filename.StartsWith("null") || filename == "desktop";
    }

    /// <summary>
    /// Converts a BeamNG virtual path like "/vehicles/RLA_Evo/Textures/carpet.dds"
    /// to an absolute file system path using <paramref name="vehiclesRootDirs"/> as the
    /// mount point(s) for the /vehicles/ prefix.
    /// Returns null if the resolved file does not exist.
    /// </summary>
    private static string? ResolveVehiclePath(
        string vehiclePath,
        IReadOnlyList<string> vehiclesRootDirs,
        Func<string, string?>? virtualPathResolver)
    {
        if (string.IsNullOrWhiteSpace(vehiclePath))
        {
	        return null;
        }

        // Normalise: remove leading slashes, convert forward-slashes
        var normalised = vehiclePath.TrimStart('/', '\\');

        // Strip the "vehicles/" prefix that BeamNG uses as a virtual root
        const string vehiclesPrefix = "vehicles/";
        if (normalised.StartsWith(vehiclesPrefix, StringComparison.OrdinalIgnoreCase))
        {
	        normalised = normalised[vehiclesPrefix.Length..];
        }

        foreach (var vehiclesRootDir in vehiclesRootDirs)
        {
            var fullPath = Path.Combine(vehiclesRootDir,
                normalised.Replace('/', Path.DirectorySeparatorChar));

            if (File.Exists(fullPath))
            {
	            return fullPath;
            }

            // Fallback: if the original reference was a .dds, try .png (for copyright-free placeholders)
            if (Path.GetExtension(fullPath).Equals(".dds", StringComparison.OrdinalIgnoreCase))
            {
                var pngPath = Path.ChangeExtension(fullPath, ".png");
                if (File.Exists(pngPath))
                {
	                return pngPath;
                }
            }
        }

        var virtualPath = "vehicles/" + normalised.Replace('\\', '/');
        var resolved = virtualPathResolver?.Invoke(virtualPath);
        if (!string.IsNullOrEmpty(resolved))
        {
            return resolved;
        }

        if (Path.GetExtension(virtualPath).Equals(".dds", StringComparison.OrdinalIgnoreCase))
        {
            var pngPath = Path.ChangeExtension(virtualPath, ".png");
            if (!string.IsNullOrEmpty(pngPath))
            {
                resolved = virtualPathResolver?.Invoke(pngPath.Replace('\\', '/'));
                if (!string.IsNullOrEmpty(resolved))
                {
                    return resolved;
                }
            }
        }

        return null;
    }
}
