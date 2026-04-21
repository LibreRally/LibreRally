using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;

namespace LibreRally.Vehicle.Rendering
{
	/// <summary>Resolved BeamNG texture set for a material entry.</summary>
	/// <param name="BaseColorPath">Resolved file-system path to the base-color texture.</param>
	/// <param name="ColorPalettePath">Optional resolved file-system path to the palette texture used for instance colour overrides.</param>
	/// <param name="UsesInstanceDiffuse">Whether the material expects instance-driven diffuse colours.</param>
	public readonly record struct BeamNgMaterialTextureSet(
		string BaseColorPath,
		string? ColorPalettePath,
		bool UsesInstanceDiffuse);

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
		private static readonly string[] TextureExtensions = [".dds", ".png", ".jpg", ".jpeg", ".tga"];

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
		/// <param name="activeMaterialSkins">Optional active material-skin selections that override base textures.</param>
		/// <returns>
		/// A dictionary keyed by material name, where each value is the absolute path of the resolved base-color texture.
		/// </returns>
		public static Dictionary<string, string> LoadMaterialTextures(
			string vehicleFolder,
			string vehiclesRootDir,
			IReadOnlyList<ActiveMaterialSkinSelection>? activeMaterialSkins = null)
		{
			return LoadMaterialTextures([vehicleFolder], [vehiclesRootDir], null, activeMaterialSkins);
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
		/// <param name="activeMaterialSkins">Optional active material-skin selections that override base textures.</param>
		/// <returns>
		/// A dictionary keyed by material name, where each value is the first resolved base-color texture path.
		/// When duplicate material names are discovered, the existing mapping is kept and later matches are ignored.
		/// </returns>
		public static Dictionary<string, string> LoadMaterialTextures(
			IEnumerable<string> materialSearchFolders,
			IEnumerable<string> vehiclesRootDirs,
			Func<string, string?>? virtualPathResolver,
			IReadOnlyList<ActiveMaterialSkinSelection>? activeMaterialSkins = null)
		{
			return LoadMaterialTextureSets(materialSearchFolders, vehiclesRootDirs, virtualPathResolver, activeMaterialSkins)
				.ToDictionary(entry => entry.Key, entry => entry.Value.BaseColorPath, StringComparer.OrdinalIgnoreCase);
		}

		/// <summary>Loads BeamNG material texture sets from a single vehicle folder.</summary>
		/// <param name="vehicleFolder">Folder containing the target vehicle's material definition files.</param>
		/// <param name="vehiclesRootDir">Root vehicles directory used to resolve BeamNG virtual texture paths.</param>
		/// <param name="activeMaterialSkins">Optional active material-skin selections that override base textures.</param>
		/// <returns>Resolved texture sets keyed by material name.</returns>
		public static Dictionary<string, BeamNgMaterialTextureSet> LoadMaterialTextureSets(
			string vehicleFolder,
			string vehiclesRootDir,
			IReadOnlyList<ActiveMaterialSkinSelection>? activeMaterialSkins = null)
		{
			return LoadMaterialTextureSets([vehicleFolder], [vehiclesRootDir], null, activeMaterialSkins);
		}

		/// <summary>Loads BeamNG material texture sets from one or more search folders.</summary>
		/// <param name="materialSearchFolders">Candidate directories to scan for <c>*.materials.json</c> files.</param>
		/// <param name="vehiclesRootDirs">BeamNG vehicles root directories used to resolve virtual paths.</param>
		/// <param name="virtualPathResolver">Optional callback for resolving virtual BeamNG asset paths.</param>
		/// <param name="activeMaterialSkins">Optional active material-skin selections that override base textures.</param>
		/// <returns>Resolved texture sets keyed by material name.</returns>
		public static Dictionary<string, BeamNgMaterialTextureSet> LoadMaterialTextureSets(
			IEnumerable<string> materialSearchFolders,
			IEnumerable<string> vehiclesRootDirs,
			Func<string, string?>? virtualPathResolver,
			IReadOnlyList<ActiveMaterialSkinSelection>? activeMaterialSkins = null)
		{
			var result = new Dictionary<string, BeamNgMaterialTextureSet>(StringComparer.OrdinalIgnoreCase);
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

			ApplyActiveSkinOverrides(result, activeMaterialSkins);
			return result;
		}

		// ──────────────────────────────────────────────────────────────────────────

		private static void ParseFile(
			string jsonFile,
			IReadOnlyList<string> vehiclesRootDirs,
			Func<string, string?>? virtualPathResolver,
			Dictionary<string, BeamNgMaterialTextureSet> result)
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

				if (!TryFindBestTextureSet(matElem, vehiclesRootDirs, virtualPathResolver, out var textureSet))
				{
					continue;
				}

				RegisterMaterialTextureSet(matName, matElem, textureSet, result);
			}
		}

		private static void RegisterMaterialTextureSet(
			string materialName,
			JsonElement material,
			BeamNgMaterialTextureSet textureSet,
			Dictionary<string, BeamNgMaterialTextureSet> result)
		{
			result.TryAdd(materialName, textureSet);

			if (!material.TryGetProperty("mapTo", out var mapToElement) ||
			    mapToElement.ValueKind != JsonValueKind.String)
			{
				return;
			}

			var mapTo = mapToElement.GetString();
			if (string.IsNullOrWhiteSpace(mapTo))
			{
				return;
			}

			result.TryAdd(mapTo, textureSet);
		}

		private static bool TryFindBestTextureSet(
			JsonElement material,
			IReadOnlyList<string> vehiclesRootDirs,
			Func<string, string?>? virtualPathResolver,
			out BeamNgMaterialTextureSet textureSet)
		{
			textureSet = default;
			if (!material.TryGetProperty("Stages", out var stages))
			{
				return false;
			}

			BeamNgMaterialTextureSet? fallback = null;
			foreach (var stage in stages.EnumerateArray())
			{
				if (stage.ValueKind != JsonValueKind.Object)
				{
					continue;
				}

				if (TryGetStageTextureSet(stage, vehiclesRootDirs, virtualPathResolver, requireInstanceDiffuse: true, out textureSet))
				{
					return true;
				}

				if (fallback == null &&
				    TryGetStageTextureSet(stage, vehiclesRootDirs, virtualPathResolver, requireInstanceDiffuse: false, out var fallbackSet))
				{
					fallback = fallbackSet;
				}
			}

			if (fallback.HasValue)
			{
				textureSet = fallback.Value;
				return true;
			}

			return false;
		}

		private static bool TryGetStageTextureSet(
			JsonElement stage,
			IReadOnlyList<string> vehiclesRootDirs,
			Func<string, string?>? virtualPathResolver,
			bool requireInstanceDiffuse,
			out BeamNgMaterialTextureSet textureSet)
		{
			textureSet = default;
			if (requireInstanceDiffuse &&
			    (!stage.TryGetProperty("instanceDiffuse", out var instanceDiffuse) || instanceDiffuse.ValueKind != JsonValueKind.True))
			{
				return false;
			}

			if (!TryGetStageBaseTexturePath(stage, out var baseTexturePath))
			{
				return false;
			}

			if (string.IsNullOrWhiteSpace(baseTexturePath))
			{
				return false;
			}

			var resolvedBaseColorPath = ResolveVehiclePath(baseTexturePath, vehiclesRootDirs, virtualPathResolver);
			if (string.IsNullOrEmpty(resolvedBaseColorPath))
			{
				return false;
			}

			string? resolvedColorPalettePath = null;
			if (TryGetNonNullPath(stage, "colorPaletteMap", out var colorPalettePath))
			{
				resolvedColorPalettePath = string.IsNullOrWhiteSpace(colorPalettePath)
					? null
					: ResolveVehiclePath(colorPalettePath, vehiclesRootDirs, virtualPathResolver);
			}

			textureSet = new BeamNgMaterialTextureSet(
				resolvedBaseColorPath,
				resolvedColorPalettePath,
				requireInstanceDiffuse);
			return true;
		}

		private static bool TryGetStageBaseTexturePath(JsonElement stage, out string? path)
		{
			path = null;
			if (TryGetNonNullPath(stage, "baseColorMap", out path))
			{
				return true;
			}

			if (TryGetNonNullPath(stage, "colorMap", out path))
			{
				return true;
			}

			return false;
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

				var resolvedLocalPath = ResolveExistingTexturePath(fullPath);
				if (!string.IsNullOrEmpty(resolvedLocalPath))
				{
					return resolvedLocalPath;
				}
			}

			var virtualPath = "vehicles/" + normalised.Replace('\\', '/');
			return ResolveVirtualTexturePath(virtualPath, virtualPathResolver);
		}

		private static string? ResolveExistingTexturePath(string fullPath)
		{
			if (File.Exists(fullPath))
			{
				return fullPath;
			}

			var extension = Path.GetExtension(fullPath);
			var basePath = string.IsNullOrEmpty(extension)
				? fullPath
				: fullPath[..^extension.Length];

			foreach (var candidatePath in TextureExtensions
				         .Select(ext => basePath + ext)
				         .Distinct(StringComparer.OrdinalIgnoreCase))
			{
				if (File.Exists(candidatePath))
				{
					return candidatePath;
				}
			}

			return null;
		}

		private static string? ResolveVirtualTexturePath(string virtualPath, Func<string, string?>? virtualPathResolver)
		{
			if (virtualPathResolver == null)
			{
				return null;
			}

			var resolved = virtualPathResolver(virtualPath);
			if (!string.IsNullOrEmpty(resolved))
			{
				return resolved;
			}

			var extension = Path.GetExtension(virtualPath);
			var basePath = string.IsNullOrEmpty(extension)
				? virtualPath
				: virtualPath[..^extension.Length];

			foreach (var candidatePath in TextureExtensions
				         .Select(ext => (basePath + ext).Replace('\\', '/'))
				         .Distinct(StringComparer.OrdinalIgnoreCase))
			{
				resolved = virtualPathResolver(candidatePath);
				if (!string.IsNullOrEmpty(resolved))
				{
					return resolved;
				}
			}

			return null;
		}

		private static void ApplyActiveSkinOverrides(
			Dictionary<string, BeamNgMaterialTextureSet> result,
			IReadOnlyList<ActiveMaterialSkinSelection>? activeMaterialSkins)
		{
			if (activeMaterialSkins == null || activeMaterialSkins.Count == 0)
			{
				return;
			}

			var materialEntries = result.ToArray();
			foreach (var selection in activeMaterialSkins)
			{
				foreach (var suffix in EnumerateMaterialSkinSuffixes(selection))
				{
					foreach (var (materialKey, textureSet) in materialEntries)
					{
						if (!materialKey.EndsWith(suffix, StringComparison.OrdinalIgnoreCase))
						{
							continue;
						}

						var baseMaterialKey = materialKey[..^suffix.Length];
						if (string.IsNullOrWhiteSpace(baseMaterialKey))
						{
							continue;
						}

						result[baseMaterialKey] = textureSet;
					}
				}
			}
		}

		private static IEnumerable<string> EnumerateMaterialSkinSuffixes(ActiveMaterialSkinSelection selection)
		{
			if (string.IsNullOrWhiteSpace(selection.VariantName))
			{
				yield break;
			}

			if (selection.SlotType.Equals("paint_design", StringComparison.OrdinalIgnoreCase))
			{
				yield return ".skin." + selection.VariantName;
			}

			yield return "." + selection.SlotType + "." + selection.VariantName;
		}
	}
}
