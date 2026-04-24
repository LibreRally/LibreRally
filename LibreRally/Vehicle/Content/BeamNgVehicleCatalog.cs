using System;
using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
using System.Linq;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Stride.Core.Diagnostics;

namespace LibreRally.Vehicle.Content
{
	/// <summary>Identifies where a BeamNG vehicle package was discovered.</summary>
	public enum BeamNgVehicleSourceKind
	{
		/// <summary>Vehicle content stored as a directory on disk.</summary>
		Folder,

		/// <summary>Vehicle content stored inside a BeamNG zip archive.</summary>
		ZipArchive,
	}

	/// <summary>Describes a discovered BeamNG vehicle package.</summary>
	/// <param name="VehicleId">Resolved BeamNG vehicle identifier.</param>
	/// <param name="SourcePath">Directory or archive path that contains the vehicle.</param>
	/// <param name="SourceKind">Origin of the discovered package.</param>
	/// <param name="SourceLabel">Short human-readable source label.</param>
	public sealed record BeamNgVehicleDescriptor(
		string VehicleId,
		string SourcePath,
		BeamNgVehicleSourceKind SourceKind,
		string SourceLabel)
	{
		/// <summary>Gets a user-facing display name for the discovered vehicle package.</summary>
		public string DisplayName => SourceKind == BeamNgVehicleSourceKind.ZipArchive
			? $"{VehicleId} [zip]"
			: VehicleId;
	}

	/// <summary>Describes one selectable BeamNG <paramref name="Vehicle" /> variant backed by an optional <c>.pc</c> configuration.</summary>
	/// <param name="Vehicle">The parent discovered BeamNG vehicle package.</param>
	/// <param name="ConfigFileName">The configuration file name to load, or <see langword="null" /> for JBeam defaults.</param>
	/// <param name="VariantId">Stable identifier for the variant within the package.</param>
	/// <param name="VehicleDisplayName">The user-facing parent package name shown in menus.</param>
	/// <param name="VariantDisplayName">User-facing variant/configuration name shown in menus.</param>
	/// <param name="Description">Optional descriptive copy for the variant.</param>
	/// <param name="ThumbnailPath">Absolute path to the preview thumbnail image when available.</param>
	/// <param name="ConfigType">Optional classification such as Factory or Rally.</param>
	/// <param name="IsDefaultVariant">Indicates whether the variant matches the package default configuration.</param>
	public sealed record BeamNgVehicleVariantDescriptor(
		BeamNgVehicleDescriptor Vehicle,
		string? ConfigFileName,
		string VariantId,
		string VehicleDisplayName,
		string VariantDisplayName,
		string Description,
		string? ThumbnailPath,
		string? ConfigType,
		bool IsDefaultVariant)
	{
		/// <summary>Gets the BeamNG identifier for the parent package.</summary>
		public string VehicleId => Vehicle.VehicleId;

		/// <summary>Gets the directory or archive path that contains the parent package.</summary>
		public string SourcePath => Vehicle.SourcePath;

		/// <summary>Gets the origin of the parent package.</summary>
		public BeamNgVehicleSourceKind SourceKind => Vehicle.SourceKind;

		/// <summary>Gets the source label for the parent package.</summary>
		public string SourceLabel => Vehicle.SourceLabel;

		/// <summary>Gets a combined user-facing label for the vehicle and variant.</summary>
		public string DisplayName => string.IsNullOrWhiteSpace(VehicleDisplayName)
			? VariantDisplayName
			: $"{VehicleDisplayName} — {VariantDisplayName}";
	}

	/// <summary>Represents a resolved BeamNG vehicle together with its asset lookup context.</summary>
	public sealed class BeamNgResolvedVehicle
	{
		private readonly BeamNgVehicleCatalog _catalog;

		internal BeamNgResolvedVehicle(
			BeamNgVehicleCatalog catalog,
			string vehicleId,
			string vehicleFolderPath,
			IReadOnlyList<string> jbeamSearchFolders,
			IReadOnlyList<string> vehiclesRootDirectories,
			string sourceDescription)
		{
			_catalog = catalog;
			VehicleId = vehicleId;
			VehicleFolderPath = vehicleFolderPath;
			JBeamSearchFolders = jbeamSearchFolders;
			VehiclesRootDirectories = vehiclesRootDirectories;
			SourceDescription = sourceDescription;
		}

		/// <summary>Gets the BeamNG vehicle identifier.</summary>
		public string VehicleId { get; }

		/// <summary>Gets the extracted or source folder that contains the active vehicle files.</summary>
		public string VehicleFolderPath { get; }

		/// <summary>Gets the folders searched for JBeam and related vehicle content.</summary>
		public IReadOnlyList<string> JBeamSearchFolders { get; }

		/// <summary>Gets the root directories used to resolve BeamNG virtual vehicle paths.</summary>
		public IReadOnlyList<string> VehiclesRootDirectories { get; }

		/// <summary>Gets a human-readable description of the source package.</summary>
		public string SourceDescription { get; }

		/// <summary>Resolves a BeamNG virtual vehicle asset path to a concrete file on disk.</summary>
		/// <param name="vehiclePath">Virtual BeamNG asset path.</param>
		/// <returns>The resolved absolute file path when found; otherwise <see langword="null"/>.</returns>
		public string? ResolveVehicleAssetPath(string vehiclePath)
		{
			return _catalog.ResolveVehicleAssetPath(vehiclePath, VehiclesRootDirectories);
		}

		/// <summary>Finds Collada mesh files that contain the requested mesh names.</summary>
		/// <param name="meshNames">Mesh names to look up.</param>
		/// <returns>Absolute file paths for matching Collada files.</returns>
		public IReadOnlyList<string> ResolveColladaFilesForMeshes(IEnumerable<string> meshNames)
		{
			return _catalog.ResolveColladaFilesForMeshes(VehicleId, meshNames);
		}
	}

	/// <summary>Discovers bundled BeamNG vehicle content and resolves files from folders or archives.</summary>
	public sealed class BeamNgVehicleCatalog : IVehicleCatalog
	{
		private static readonly Logger Log = GlobalLogger.GetLogger("BeamNgVehicleCatalog");
		private static readonly string[] PngFallbackExtensions = [".png", ".jpg", ".jpeg", ".tga"];
		private static readonly string[] ThumbnailExtensions = [".jpg", ".jpeg", ".png"];
		private static readonly string[] CommonMetadataExtensions = [".jbeam", ".json", ".pc"];
		private static readonly JsonDocumentOptions JsonOptions = new() { AllowTrailingCommas = true, CommentHandling = JsonCommentHandling.Skip, };
		private readonly string _bundledVehiclesRoot;
		private readonly string? _beamNgContentVehiclesRoot;
		private readonly string _cacheRoot;
		private readonly Dictionary<string, BeamNgVehicleDescriptor> _bundledVehiclesById = new(StringComparer.OrdinalIgnoreCase);
		private readonly Dictionary<string, IReadOnlyList<string>> _colladaFilesByPackageAndMeshSet = new(StringComparer.OrdinalIgnoreCase);
		private IReadOnlyList<BeamNgVehicleDescriptor>? _discoveredVehicles;
		private IReadOnlyList<BeamNgVehicleVariantDescriptor>? _discoveredVehicleVariants;

		/// <summary>Initializes a new vehicle catalog rooted at the given bundled and BeamNG content directories.</summary>
		/// <param name="bundledVehiclesRoot">Root directory that contains bundled vehicle folders or archives.</param>
		/// <param name="beamNgContentVehiclesRoot">Optional BeamNG installation content directory.</param>
		/// <param name="cacheRoot">Optional cache directory used for extracted archive content.</param>
		public BeamNgVehicleCatalog(string bundledVehiclesRoot, string? beamNgContentVehiclesRoot = null, string? cacheRoot = null)
		{
			_bundledVehiclesRoot = bundledVehiclesRoot;
			_beamNgContentVehiclesRoot = beamNgContentVehiclesRoot;
			_cacheRoot = cacheRoot ?? Path.Combine(
				Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
				"LibreRally",
				"BeamNGCache");
		}

		/// <summary>Attempts to locate BeamNG's installed <c>content/vehicles</c> directory on Windows.</summary>
		/// <returns>The detected vehicles directory when present; otherwise <see langword="null"/>.</returns>
		public static string? DetectBeamNgContentVehiclesRoot()
		{
			string[] candidates =
			[
				@"C:\Program Files (x86)\Steam\steamapps\common\BeamNG.drive\content\vehicles",
				@"C:\Program Files\Steam\steamapps\common\BeamNG.drive\content\vehicles",
			];

			return Array.Find(candidates, Directory.Exists);
		}

		/// <summary>Discovers all bundled vehicles that can be loaded by LibreRally.</summary>
		/// <returns>Cached descriptors for every discovered folder or archive vehicle.</returns>
		public IReadOnlyList<BeamNgVehicleDescriptor> DiscoverBundledVehicles()
		{
			if (_discoveredVehicles != null)
			{
				return _discoveredVehicles;
			}

			var vehicles = new List<BeamNgVehicleDescriptor>();
			_bundledVehiclesById.Clear();
			_colladaFilesByPackageAndMeshSet.Clear();
			_discoveredVehicleVariants = null;

			if (Directory.Exists(_bundledVehiclesRoot))
			{
				foreach (var folder in Directory.EnumerateDirectories(_bundledVehiclesRoot)
					         .OrderBy(path => path, StringComparer.OrdinalIgnoreCase))
				{
					if (!ContainsVehicleContent(folder))
					{
						continue;
					}

					var vehicleId = Path.GetFileName(folder.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
					var descriptor = new BeamNgVehicleDescriptor(vehicleId, folder, BeamNgVehicleSourceKind.Folder, "folder");
					vehicles.Add(descriptor);
					_bundledVehiclesById.TryAdd(vehicleId, descriptor);
				}

				foreach (var zipPath in Directory.EnumerateFiles(_bundledVehiclesRoot, "*.zip", SearchOption.TopDirectoryOnly)
					         .OrderBy(path => path, StringComparer.OrdinalIgnoreCase))
				{
					if (!TryReadPrimaryVehicleId(zipPath, out var vehicleId))
					{
						continue;
					}

					var descriptor = new BeamNgVehicleDescriptor(vehicleId, zipPath, BeamNgVehicleSourceKind.ZipArchive, "zip");
					vehicles.Add(descriptor);
					_bundledVehiclesById.TryAdd(vehicleId, descriptor);
				}
			}

			_discoveredVehicles = vehicles
				.OrderBy(vehicle => vehicle.VehicleId, StringComparer.OrdinalIgnoreCase)
				.ThenBy(vehicle => vehicle.SourceKind)
				.ToArray();
			return _discoveredVehicles;
		}

		/// <summary>Discovers every selectable bundled vehicle variant, including per-configuration metadata and thumbnails.</summary>
		/// <returns>Cached descriptors for every discovered variant across bundled folders and archives.</returns>
		public IReadOnlyList<BeamNgVehicleVariantDescriptor> DiscoverBundledVehicleVariants()
		{
			if (_discoveredVehicleVariants != null)
			{
				return _discoveredVehicleVariants;
			}

			var variants = new List<BeamNgVehicleVariantDescriptor>();
			foreach (var vehicle in DiscoverBundledVehicles())
			{
				variants.AddRange(DiscoverVehicleVariants(vehicle));
			}

			_discoveredVehicleVariants = variants
				.OrderBy(variant => variant.VehicleDisplayName, StringComparer.OrdinalIgnoreCase)
				.ThenByDescending(variant => variant.IsDefaultVariant)
				.ThenBy(variant => variant.VariantDisplayName, StringComparer.OrdinalIgnoreCase)
				.ThenBy(variant => variant.Vehicle.SourceKind)
				.ToArray();
			return _discoveredVehicleVariants;
		}

		/// <summary>Resolves a requested vehicle identifier or path into a loadable BeamNG vehicle source.</summary>
		/// <param name="requestedPathOrId">Vehicle id, folder path, or zip path supplied by the caller.</param>
		/// <returns>A resolved vehicle source that can be consumed by the loader.</returns>
		/// <exception cref="ArgumentException">Thrown when <paramref name="requestedPathOrId" /> is empty or whitespace.</exception>
		/// <exception cref="FileNotFoundException">Thrown when the requested vehicle cannot be resolved from the configured sources.</exception>
		public BeamNgResolvedVehicle ResolveVehicle(string requestedPathOrId)
		{
			if (string.IsNullOrWhiteSpace(requestedPathOrId))
			{
				throw new ArgumentException("Vehicle path or id must not be empty.", nameof(requestedPathOrId));
			}

			var discoveredVehicles = DiscoverBundledVehicles();
			var absoluteCandidate = Path.IsPathRooted(requestedPathOrId)
				? requestedPathOrId
				: Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, requestedPathOrId));

			if (Directory.Exists(absoluteCandidate))
			{
				return ResolveDirectoryVehicle(absoluteCandidate, Path.GetFileName(absoluteCandidate), absoluteCandidate);
			}

			if (File.Exists(absoluteCandidate) &&
			    Path.GetExtension(absoluteCandidate).Equals(".zip", StringComparison.OrdinalIgnoreCase))
			{
				return ResolveZipVehicle(absoluteCandidate, Path.GetFileName(absoluteCandidate));
			}

			var bundledCandidate = Path.Combine(_bundledVehiclesRoot, requestedPathOrId);
			if (Directory.Exists(bundledCandidate))
			{
				return ResolveDirectoryVehicle(bundledCandidate, Path.GetFileName(bundledCandidate), bundledCandidate);
			}

			if (File.Exists(bundledCandidate) &&
			    Path.GetExtension(bundledCandidate).Equals(".zip", StringComparison.OrdinalIgnoreCase))
			{
				return ResolveZipVehicle(bundledCandidate, Path.GetFileName(bundledCandidate));
			}

			var descriptor = discoveredVehicles.FirstOrDefault(vehicle =>
				vehicle.VehicleId.Equals(requestedPathOrId, StringComparison.OrdinalIgnoreCase) ||
				vehicle.SourcePath.Equals(requestedPathOrId, StringComparison.OrdinalIgnoreCase));

			if (descriptor == null)
			{
				throw new FileNotFoundException($"Could not resolve vehicle '{requestedPathOrId}' from '{_bundledVehiclesRoot}'.");
			}

			return descriptor.SourceKind == BeamNgVehicleSourceKind.ZipArchive
				? ResolveZipVehicle(descriptor.SourcePath, descriptor.DisplayName)
				: ResolveDirectoryVehicle(descriptor.SourcePath, descriptor.DisplayName, descriptor.SourcePath);
		}

		internal string? ResolveVehicleAssetPath(string vehiclePath, IReadOnlyList<string> vehiclesRootDirectories)
		{
			var normalizedPath = NormalizeVehiclePath(vehiclePath);
			if (normalizedPath == null)
			{
				return null;
			}

			var relativePath = normalizedPath["vehicles/".Length..];
			if (TryResolveFromVehicleRoots(relativePath, vehiclesRootDirectories, out var resolvedPath))
			{
				return resolvedPath;
			}

			var packageName = relativePath
				.Split('/', '\\', StringSplitOptions.RemoveEmptyEntries)
				.FirstOrDefault();
			if (string.IsNullOrEmpty(packageName))
			{
				return null;
			}

			var bundledFolder = Path.Combine(_bundledVehiclesRoot, relativePath.Replace('/', Path.DirectorySeparatorChar));
			if (File.Exists(bundledFolder))
			{
				return bundledFolder;
			}

			if (_bundledVehiclesById.TryGetValue(packageName, out var bundledVehicle))
			{
				if (bundledVehicle.SourceKind == BeamNgVehicleSourceKind.Folder)
				{
					var bundledPath = Path.Combine(_bundledVehiclesRoot, relativePath.Replace('/', Path.DirectorySeparatorChar));
					if (File.Exists(bundledPath))
					{
						return bundledPath;
					}
				}
				else
				{
					var extractedVehiclesRoot = EnsureZipExtracted(bundledVehicle.SourcePath);
					var extractedPath = Path.Combine(extractedVehiclesRoot, relativePath.Replace('/', Path.DirectorySeparatorChar));
					if (File.Exists(extractedPath))
					{
						return extractedPath;
					}
				}
			}

			if (!string.IsNullOrEmpty(_beamNgContentVehiclesRoot))
			{
				var packageZip = Path.Combine(_beamNgContentVehiclesRoot, packageName + ".zip");
				if (File.Exists(packageZip))
				{
					var extracted = MaterializeZipEntry(packageZip, normalizedPath);
					if (extracted != null)
					{
						return extracted;
					}
				}
			}

			if (Path.GetExtension(normalizedPath).Equals(".dds", StringComparison.OrdinalIgnoreCase))
			{
				foreach (var altExtension in PngFallbackExtensions)
				{
					var altPath = ReplaceExtension(normalizedPath, altExtension);
					if (altPath == null)
					{
						continue;
					}

					if (TryResolveFromVehicleRoots(altPath["vehicles/".Length..], vehiclesRootDirectories, out resolvedPath))
					{
						return resolvedPath;
					}

					if (!string.IsNullOrEmpty(_beamNgContentVehiclesRoot))
					{
						var packageZip = Path.Combine(_beamNgContentVehiclesRoot, packageName + ".zip");
						if (File.Exists(packageZip))
						{
							var extracted = MaterializeZipEntry(packageZip, altPath);
							if (extracted != null)
							{
								return extracted;
							}
						}
					}
				}
			}

			return null;
		}

		internal IReadOnlyList<string> ResolveColladaFilesForMeshes(string vehicleId, IEnumerable<string> meshNames)
		{
			DiscoverBundledVehicles();
			var normalizedMeshNames = meshNames
				.Where(name => !string.IsNullOrWhiteSpace(name))
				.Distinct(StringComparer.OrdinalIgnoreCase)
				.OrderBy(name => name, StringComparer.OrdinalIgnoreCase)
				.ToArray();
			if (normalizedMeshNames.Length == 0)
			{
				return [];
			}

			var packageNames = new[] { vehicleId, "common" }
				.Where(name => !string.IsNullOrWhiteSpace(name))
				.Distinct(StringComparer.OrdinalIgnoreCase)
				.ToArray();
			var colladaFiles = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
			foreach (var packageName in packageNames)
			{
				foreach (var file in ResolveColladaFilesForPackageMeshes(packageName, normalizedMeshNames))
				{
					colladaFiles.Add(file);
				}
			}

			var unresolvedMeshNames = GetUnresolvedMeshNames(colladaFiles, normalizedMeshNames);
			if (unresolvedMeshNames.Length > 0)
			{
				foreach (var packageName in EnumerateAdditionalPackageNames(packageNames))
				{
					foreach (var file in ResolveColladaFilesForPackageMeshes(packageName, unresolvedMeshNames))
					{
						colladaFiles.Add(file);
					}

					unresolvedMeshNames = GetUnresolvedMeshNames(colladaFiles, unresolvedMeshNames);
					if (unresolvedMeshNames.Length == 0)
					{
						break;
					}
				}
			}

			return colladaFiles
				.OrderBy(path => path, StringComparer.OrdinalIgnoreCase)
				.ToArray();
		}

		private BeamNgResolvedVehicle ResolveDirectoryVehicle(string vehicleFolderPath, string sourceDescription, string sourcePath)
		{
			var vehicleId = Path.GetFileName(vehicleFolderPath.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
			var jbeamSearchFolders = new List<string> { vehicleFolderPath };
			var vehiclesRootDirectories = new List<string> { Path.GetDirectoryName(vehicleFolderPath)! };

			var commonMetadataFolder = EnsureCommonMetadataExtracted();
			if (commonMetadataFolder != null)
			{
				jbeamSearchFolders.Add(commonMetadataFolder);
				vehiclesRootDirectories.Add(Path.GetDirectoryName(commonMetadataFolder)!);
			}

			return new BeamNgResolvedVehicle(
				this,
				vehicleId,
				vehicleFolderPath,
				DistinctExistingPaths(jbeamSearchFolders),
				DistinctExistingPaths(vehiclesRootDirectories),
				$"folder:{sourceDescription}");
		}

		private IReadOnlyList<BeamNgVehicleVariantDescriptor> DiscoverVehicleVariants(BeamNgVehicleDescriptor vehicle)
		{
			if (vehicle.SourceKind == BeamNgVehicleSourceKind.ZipArchive)
			{
				var extractedVehiclesRoot = EnsureZipExtracted(vehicle.SourcePath);
				var extractedVehicleFolder = Path.Combine(extractedVehiclesRoot, vehicle.VehicleId);
				return DiscoverVehicleVariantsFromFolder(vehicle, extractedVehicleFolder);
			}

			return DiscoverVehicleVariantsFromFolder(vehicle, vehicle.SourcePath);
		}

		private static IReadOnlyList<BeamNgVehicleVariantDescriptor> DiscoverVehicleVariantsFromFolder(
			BeamNgVehicleDescriptor vehicle,
			string vehicleFolderPath)
		{
			if (!Directory.Exists(vehicleFolderPath))
			{
				return [];
			}

			var baseInfo = ReadVehicleInfo(Path.Combine(vehicleFolderPath, "info.json"));
			var vehicleDisplayName = BuildVehicleDisplayName(vehicle.VehicleId, baseInfo.Brand, baseInfo.Name);
			var configFiles = Directory
				.EnumerateFiles(vehicleFolderPath, "*.pc", SearchOption.TopDirectoryOnly)
				.Select(Path.GetFileName)
				.Where(fileName => !string.IsNullOrWhiteSpace(fileName))
				.OrderBy(fileName => fileName, StringComparer.OrdinalIgnoreCase)
				.ToArray()!;
			if (configFiles.Length == 0)
			{
				return
				[
					CreateVariantDescriptor(
						vehicle,
						vehicleFolderPath,
						baseInfo,
						vehicleDisplayName,
						configFileName: null),
				];
			}

			return configFiles
				.Select(configFileName => CreateVariantDescriptor(
					vehicle,
					vehicleFolderPath,
					baseInfo,
					vehicleDisplayName,
					configFileName))
				.OrderByDescending(variant => variant.IsDefaultVariant)
				.ThenBy(variant => variant.VariantDisplayName, StringComparer.OrdinalIgnoreCase)
				.ToArray();
		}

		private BeamNgResolvedVehicle ResolveZipVehicle(string zipPath, string sourceDescription)
		{
			var extractedVehiclesRoot = EnsureZipExtracted(zipPath);
			if (!TryReadPrimaryVehicleId(zipPath, out var vehicleId))
			{
				vehicleId = Path.GetFileNameWithoutExtension(zipPath);
			}

			var vehicleFolderPath = Path.Combine(extractedVehiclesRoot, vehicleId);
			if (!Directory.Exists(vehicleFolderPath))
			{
				throw new DirectoryNotFoundException($"Zip vehicle '{zipPath}' did not extract to '{vehicleFolderPath}'.");
			}

			var jbeamSearchFolders = new List<string> { vehicleFolderPath };
			var vehiclesRootDirectories = new List<string> { extractedVehiclesRoot };

			var commonMetadataFolder = EnsureCommonMetadataExtracted();
			if (commonMetadataFolder != null)
			{
				jbeamSearchFolders.Add(commonMetadataFolder);
				vehiclesRootDirectories.Add(Path.GetDirectoryName(commonMetadataFolder)!);
			}

			return new BeamNgResolvedVehicle(
				this,
				vehicleId,
				vehicleFolderPath,
				DistinctExistingPaths(jbeamSearchFolders),
				DistinctExistingPaths(vehiclesRootDirectories),
				$"zip:{sourceDescription}");
		}

		private string EnsureZipExtracted(string zipPath)
		{
			var cacheRoot = GetCacheRoot(zipPath, "full");
			var completionMarker = Path.Combine(cacheRoot, ".extract.complete");
			var vehiclesRoot = Path.Combine(cacheRoot, "vehicles");
			if (File.Exists(completionMarker) && Directory.Exists(vehiclesRoot))
			{
				return vehiclesRoot;
			}

			if (Directory.Exists(cacheRoot))
			{
				Directory.Delete(cacheRoot, true);
			}

			Directory.CreateDirectory(cacheRoot);
			using var archive = ZipFile.OpenRead(zipPath);
			foreach (var entry in archive.Entries)
			{
				ExtractEntry(entry, cacheRoot);
			}

			File.WriteAllText(completionMarker, zipPath);
			return vehiclesRoot;
		}

		private string? EnsureCommonMetadataExtracted()
		{
			if (string.IsNullOrEmpty(_beamNgContentVehiclesRoot))
			{
				return null;
			}

			var commonZipPath = Path.Combine(_beamNgContentVehiclesRoot, "common.zip");
			if (!File.Exists(commonZipPath))
			{
				return null;
			}

			var cacheRoot = GetCacheRoot(commonZipPath, "metadata");
			var completionMarker = Path.Combine(cacheRoot, ".metadata.complete");
			var commonFolder = Path.Combine(cacheRoot, "vehicles", "common");
			if (File.Exists(completionMarker) && Directory.Exists(commonFolder))
			{
				return commonFolder;
			}

			Directory.CreateDirectory(cacheRoot);
			using var archive = ZipFile.OpenRead(commonZipPath);
			foreach (var entry in archive.Entries)
			{
				if (entry.Length <= 0 ||
				    !entry.FullName.StartsWith("vehicles/common/", StringComparison.OrdinalIgnoreCase) ||
				    !ShouldExtractCommonMetadata(entry.FullName))
				{
					continue;
				}

				ExtractEntry(entry, cacheRoot);
			}

			File.WriteAllText(completionMarker, commonZipPath);
			return Directory.Exists(commonFolder) ? commonFolder : null;
		}

		private string? MaterializeZipEntry(string zipPath, string normalizedVehiclePath)
		{
			var cacheRoot = GetCacheRoot(zipPath, "ondemand");
			var destination = Path.Combine(cacheRoot, normalizedVehiclePath.Replace('/', Path.DirectorySeparatorChar));
			if (File.Exists(destination))
			{
				return destination;
			}

			using var archive = ZipFile.OpenRead(zipPath);
			var entry = archive.GetEntry(normalizedVehiclePath.Replace('\\', '/'));
			if (entry == null || entry.Length <= 0)
			{
				return null;
			}

			ExtractEntry(entry, cacheRoot);
			return File.Exists(destination) ? destination : null;
		}

		private IReadOnlyList<string> ResolveColladaFilesForPackageMeshes(string packageName, IReadOnlyList<string> meshNames)
		{
			var cacheKey = packageName + "|" + string.Join(";", meshNames);
			if (_colladaFilesByPackageAndMeshSet.TryGetValue(cacheKey, out var cached))
			{
				return cached;
			}

			var colladaFiles = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
			foreach (var file in FindColladaFilesInBundledFolder(packageName, meshNames))
			{
				colladaFiles.Add(file);
			}

			foreach (var file in FindColladaFilesInInstalledOrBundledZip(packageName, meshNames))
			{
				colladaFiles.Add(file);
			}

			cached = colladaFiles
				.OrderBy(path => path, StringComparer.OrdinalIgnoreCase)
				.ToArray();
			_colladaFilesByPackageAndMeshSet[cacheKey] = cached;
			return cached;
		}

		private IEnumerable<string> EnumerateAdditionalPackageNames(IReadOnlyCollection<string> excludedPackageNames)
		{
			var yielded = new HashSet<string>(excludedPackageNames, StringComparer.OrdinalIgnoreCase);

			if (Directory.Exists(_bundledVehiclesRoot))
			{
				foreach (var folder in Directory.EnumerateDirectories(_bundledVehiclesRoot)
					         .OrderBy(path => path, StringComparer.OrdinalIgnoreCase))
				{
					var packageName = Path.GetFileName(folder);
					if (string.IsNullOrWhiteSpace(packageName) || !yielded.Add(packageName))
					{
						continue;
					}

					yield return packageName;
				}
			}

			foreach (var bundledVehicle in _bundledVehiclesById.Values
				         .OrderBy(vehicle => vehicle.VehicleId, StringComparer.OrdinalIgnoreCase))
			{
				if (yielded.Add(bundledVehicle.VehicleId))
				{
					yield return bundledVehicle.VehicleId;
				}
			}

			if (string.IsNullOrEmpty(_beamNgContentVehiclesRoot) || !Directory.Exists(_beamNgContentVehiclesRoot))
			{
				yield break;
			}

			foreach (var zipPath in Directory.EnumerateFiles(_beamNgContentVehiclesRoot, "*.zip", SearchOption.TopDirectoryOnly)
				         .OrderBy(path => path, StringComparer.OrdinalIgnoreCase))
			{
				var packageName = Path.GetFileNameWithoutExtension(zipPath);
				if (!string.IsNullOrWhiteSpace(packageName) && yielded.Add(packageName))
				{
					yield return packageName;
				}
			}
		}

		private IEnumerable<string> FindColladaFilesInBundledFolder(string packageName, IReadOnlyList<string> meshNames)
		{
			var bundledFolder = Path.Combine(_bundledVehiclesRoot, packageName);
			if (!Directory.Exists(bundledFolder))
			{
				yield break;
			}

			var unresolvedMeshes = new HashSet<string>(meshNames, StringComparer.OrdinalIgnoreCase);
			foreach (var colladaFile in EnumerateColladaFiles(bundledFolder)
				         .OrderBy(GetColladaPathPreference)
				         .ThenBy(path => path, StringComparer.OrdinalIgnoreCase))
			{
				if (unresolvedMeshes.Count == 0)
				{
					yield break;
				}

				var matchedMeshes = GetMatchedMeshNames(colladaFile, unresolvedMeshes);
				if (matchedMeshes.Length == 0)
				{
					continue;
				}

				yield return colladaFile;
				foreach (var meshName in matchedMeshes)
				{
					unresolvedMeshes.Remove(meshName);
				}
			}
		}

		private IEnumerable<string> FindColladaFilesInInstalledOrBundledZip(string packageName, IReadOnlyList<string> meshNames)
		{
			var zipCandidates = new List<string>();
			if (_bundledVehiclesById.TryGetValue(packageName, out var bundledVehicle) &&
			    bundledVehicle.SourceKind == BeamNgVehicleSourceKind.ZipArchive)
			{
				zipCandidates.Add(bundledVehicle.SourcePath);
			}

			if (!string.IsNullOrEmpty(_beamNgContentVehiclesRoot))
			{
				var installedZip = Path.Combine(_beamNgContentVehiclesRoot, packageName + ".zip");
				if (File.Exists(installedZip))
				{
					zipCandidates.Add(installedZip);
				}
			}

			foreach (var zipPath in zipCandidates
				         .Distinct(StringComparer.OrdinalIgnoreCase))
			{
				foreach (var file in MaterializeMatchingColladaEntries(zipPath, packageName, meshNames))
				{
					yield return file;
				}
			}
		}

		private IEnumerable<string> MaterializeMatchingColladaEntries(string zipPath, string packageName, IReadOnlyList<string> meshNames)
		{
			using var archive = ZipFile.OpenRead(zipPath);
			var packagePrefix = $"vehicles/{packageName}/";
			var unresolvedMeshes = new HashSet<string>(meshNames, StringComparer.OrdinalIgnoreCase);
			var colladaEntries = archive.Entries
				.Where(entry =>
					entry.Length > 0 &&
					entry.FullName.StartsWith(packagePrefix, StringComparison.OrdinalIgnoreCase) &&
					IsColladaEntry(entry.FullName))
				.OrderBy(entry => GetColladaPathPreference(entry.FullName))
				.ThenBy(entry => entry.FullName, StringComparer.OrdinalIgnoreCase);
			foreach (var entry in colladaEntries)
			{
				if (unresolvedMeshes.Count == 0)
				{
					yield break;
				}

				using var reader = new StreamReader(entry.Open());
				var text = reader.ReadToEnd();
				var matchedMeshes = unresolvedMeshes
					.Where(meshName => text.IndexOf(meshName, StringComparison.OrdinalIgnoreCase) >= 0)
					.ToArray();
				if (matchedMeshes.Length == 0)
				{
					continue;
				}

				var materialized = MaterializeZipEntry(zipPath, entry.FullName.Replace('\\', '/'));
				if (!string.IsNullOrEmpty(materialized))
				{
					MaterializeAncestorMaterialFiles(archive, zipPath, entry.FullName);
					yield return materialized;
				}

				foreach (var meshName in matchedMeshes)
				{
					unresolvedMeshes.Remove(meshName);
				}
			}
		}

		private static IEnumerable<string> EnumerateColladaFiles(string rootDirectory)
		{
			return Directory
				.EnumerateFiles(rootDirectory, "*", SearchOption.AllDirectories)
				.Where(path => IsColladaEntry(path))
				.Distinct(StringComparer.OrdinalIgnoreCase);
		}

		private void MaterializeAncestorMaterialFiles(ZipArchive archive, string zipPath, string colladaEntryPath)
		{
			var cacheRoot = GetCacheRoot(zipPath, "ondemand");
			var directory = NormalizeEntryDirectory(colladaEntryPath);
			while (!string.IsNullOrEmpty(directory))
			{
				foreach (var entry in archive.Entries.Where(entry =>
					         entry.Length > 0 &&
					         entry.FullName.EndsWith(".materials.json", StringComparison.OrdinalIgnoreCase) &&
					         string.Equals(NormalizeEntryDirectory(entry.FullName), directory, StringComparison.OrdinalIgnoreCase)))
				{
					ExtractEntry(entry, cacheRoot);
				}

				var parentSeparator = directory.LastIndexOf('/');
				if (parentSeparator <= 0)
				{
					break;
				}

				var parentDirectory = directory[..parentSeparator];
				if (string.Equals(parentDirectory, "vehicles", StringComparison.OrdinalIgnoreCase))
				{
					break;
				}

				directory = parentDirectory;
			}
		}

		private static string NormalizeEntryDirectory(string entryPath)
		{
			var normalizedPath = entryPath.Replace('\\', '/');
			var separatorIndex = normalizedPath.LastIndexOf('/');
			return separatorIndex >= 0
				? normalizedPath[..separatorIndex]
				: string.Empty;
		}

		private static bool ColladaFileContainsAnyMesh(string colladaFile, IReadOnlyList<string> meshNames)
		{
			var text = File.ReadAllText(colladaFile);
			return meshNames.Any(meshName => text.IndexOf(meshName, StringComparison.OrdinalIgnoreCase) >= 0);
		}

		private static string[] GetMatchedMeshNames(string colladaFile, IEnumerable<string> meshNames)
		{
			var text = File.ReadAllText(colladaFile);
			return meshNames
				.Where(meshName => text.IndexOf(meshName, StringComparison.OrdinalIgnoreCase) >= 0)
				.ToArray();
		}

		private static string[] GetUnresolvedMeshNames(IEnumerable<string> colladaFiles, IReadOnlyList<string> meshNames)
		{
			var unresolved = new HashSet<string>(meshNames, StringComparer.OrdinalIgnoreCase);
			foreach (var colladaFile in colladaFiles
				         .Where(File.Exists)
				         .Distinct(StringComparer.OrdinalIgnoreCase))
			{
				if (unresolved.Count == 0)
				{
					break;
				}

				var text = File.ReadAllText(colladaFile);
				foreach (var meshName in unresolved.ToArray())
				{
					if (text.IndexOf(meshName, StringComparison.OrdinalIgnoreCase) >= 0)
					{
						unresolved.Remove(meshName);
					}
				}
			}

			return unresolved
				.OrderBy(name => name, StringComparer.OrdinalIgnoreCase)
				.ToArray();
		}

		private static bool IsColladaEntry(string path)
		{
			var extension = Path.GetExtension(path);
			return extension.Equals(".dae", StringComparison.OrdinalIgnoreCase) ||
			       extension.Equals(".cdae", StringComparison.OrdinalIgnoreCase);
		}

		private static int GetColladaPathPreference(string path)
		{
			var extension = Path.GetExtension(path);
			return extension.Equals(".dae", StringComparison.OrdinalIgnoreCase) ? 0 : 1;
		}

		private string GetCacheRoot(string sourcePath, string mode)
		{
			var sourceKey = ComputeSourceKey(sourcePath);
			var safeName = SanitizeFileName(Path.GetFileNameWithoutExtension(sourcePath));
			return Path.Combine(_cacheRoot, mode, $"{safeName}-{sourceKey}");
		}

		private static bool ContainsVehicleContent(string folder)
		{
			return Directory.EnumerateFiles(folder, "*.jbeam", SearchOption.AllDirectories).Any() ||
			       Directory.EnumerateFiles(folder, "*.pc", SearchOption.TopDirectoryOnly).Any();
		}

		private static BeamNgVehicleVariantDescriptor CreateVariantDescriptor(
			BeamNgVehicleDescriptor vehicle,
			string vehicleFolderPath,
			VehicleInfo baseInfo,
			string vehicleDisplayName,
			string? configFileName)
		{
			var variantId = string.IsNullOrWhiteSpace(configFileName)
				? "default"
				: Path.GetFileNameWithoutExtension(configFileName);
			var variantInfo = string.IsNullOrWhiteSpace(configFileName)
				? default
				: ReadVariantInfo(Path.Combine(vehicleFolderPath, $"info_{variantId}.json"));
			var variantDisplayName = !string.IsNullOrWhiteSpace(variantInfo.Configuration)
				? variantInfo.Configuration
				: !string.IsNullOrWhiteSpace(baseInfo.Name) && string.IsNullOrWhiteSpace(configFileName)
					? baseInfo.Name!
					: FormatVariantFallbackLabel(variantId, configFileName);
			var description = !string.IsNullOrWhiteSpace(variantInfo.Description)
				? variantInfo.Description!
				: baseInfo.Description ?? string.Empty;
			var defaultConfigFileName = NormalizeConfigFileName(baseInfo.DefaultConfigFileName);
			var normalizedConfigFileName = NormalizeConfigFileName(configFileName);
			var thumbnailPath = ResolveThumbnailPath(vehicleFolderPath, variantId);
			return new BeamNgVehicleVariantDescriptor(
				vehicle,
				normalizedConfigFileName,
				variantId,
				vehicleDisplayName,
				variantDisplayName,
				description,
				thumbnailPath,
				variantInfo.ConfigType,
				!string.IsNullOrWhiteSpace(defaultConfigFileName) &&
				string.Equals(defaultConfigFileName, normalizedConfigFileName, StringComparison.OrdinalIgnoreCase));
		}

		private static bool TryReadPrimaryVehicleId(string zipPath, out string vehicleId)
		{
			vehicleId = string.Empty;

			try
			{
				using var archive = ZipFile.OpenRead(zipPath);
				vehicleId = archive.Entries
					.Select(entry => entry.FullName.Replace('\\', '/'))
					.Select(path => path.Split('/', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries))
					.Where(parts =>
						parts.Length >= 2 &&
						parts[0].Equals("vehicles", StringComparison.OrdinalIgnoreCase) &&
						!parts.Any(segment => segment is "." or ".."))
					.Select(parts => parts[1])
					.FirstOrDefault(name => !string.IsNullOrWhiteSpace(name)) ?? string.Empty;
				return !string.IsNullOrWhiteSpace(vehicleId);
			}
			catch
			{
				return false;
			}
		}

		private static bool TryResolveFromVehicleRoots(
			string relativeVehiclePath,
			IReadOnlyList<string> vehiclesRootDirectories,
			out string? resolvedPath)
		{
			foreach (var root in vehiclesRootDirectories)
			{
				var candidate = Path.Combine(root, relativeVehiclePath.Replace('/', Path.DirectorySeparatorChar));
				if (File.Exists(candidate))
				{
					resolvedPath = candidate;
					return true;
				}
			}

			resolvedPath = null;
			return false;
		}

		private static string BuildVehicleDisplayName(string vehicleId, string? brand, string? name)
		{
			if (!string.IsNullOrWhiteSpace(brand) && !string.IsNullOrWhiteSpace(name))
			{
				return $"{brand} {name}";
			}

			if (!string.IsNullOrWhiteSpace(name))
			{
				return name!;
			}

			return vehicleId;
		}

		private static string FormatVariantFallbackLabel(string variantId, string? configFileName)
		{
			if (string.IsNullOrWhiteSpace(configFileName))
			{
				return "Default setup";
			}

			return variantId.Replace('_', ' ');
		}

		internal static string? NormalizeConfigFileName(string? configFileName)
		{
			if (string.IsNullOrWhiteSpace(configFileName))
			{
				return null;
			}

			var trimmed = configFileName.Trim();
			return Path.GetExtension(trimmed).Equals(".pc", StringComparison.OrdinalIgnoreCase)
				? Path.GetFileName(trimmed)
				: Path.GetFileName(trimmed) + ".pc";
		}

		private static string? ResolveThumbnailPath(string vehicleFolderPath, string variantId)
		{
			foreach (var extension in ThumbnailExtensions)
			{
				var exactPath = Path.Combine(vehicleFolderPath, variantId + extension);
				if (File.Exists(exactPath))
				{
					return exactPath;
				}
			}

			foreach (var extension in ThumbnailExtensions)
			{
				var fallbackPath = Path.Combine(vehicleFolderPath, "default" + extension);
				if (File.Exists(fallbackPath))
				{
					return fallbackPath;
				}
			}

			return null;
		}

		private static VehicleInfo ReadVehicleInfo(string infoFilePath)
		{
			if (!TryLoadJsonDocument(infoFilePath, out var document))
			{
				return default;
			}

			using (document)
			{
				var root = document.RootElement;
				return new VehicleInfo(
					GetOptionalStringProperty(root, "Brand"),
					GetOptionalStringProperty(root, "Name"),
					GetOptionalStringProperty(root, "Description"),
					GetOptionalStringProperty(root, "default_pc"));
			}
		}

		private static VariantInfo ReadVariantInfo(string infoFilePath)
		{
			if (!TryLoadJsonDocument(infoFilePath, out var document))
			{
				return default;
			}

			using (document)
			{
				var root = document.RootElement;
				return new VariantInfo(
					GetOptionalStringProperty(root, "Configuration"),
					GetOptionalStringProperty(root, "Description"),
					GetOptionalStringProperty(root, "Config Type"));
			}
		}

		private static bool TryLoadJsonDocument(string filePath, out JsonDocument? document)
		{
			document = null;
			if (!File.Exists(filePath))
			{
				return false;
			}

			try
			{
				document = JsonDocument.Parse(File.ReadAllText(filePath), JsonOptions);
				return true;
			}
			catch (IOException ex)
			{
				Log.Debug($"Could not read '{filePath}': {ex.Message}");
				return false;
			}
			catch (UnauthorizedAccessException ex)
			{
				Log.Debug($"Access denied reading '{filePath}': {ex.Message}");
				return false;
			}
			catch (JsonException ex)
			{
				Log.Debug($"Malformed JSON in '{filePath}': {ex.Message}");
				return false;
			}
		}

		private static string? GetOptionalStringProperty(JsonElement element, string propertyName)
		{
			return element.TryGetProperty(propertyName, out var property) &&
			       property.ValueKind == JsonValueKind.String
				? property.GetString()
				: null;
		}

		private static string? NormalizeVehiclePath(string vehiclePath)
		{
			if (string.IsNullOrWhiteSpace(vehiclePath))
			{
				return null;
			}

			var normalized = vehiclePath
				.Trim()
				.TrimStart('/', '\\')
				.Replace('\\', '/');
			return normalized.StartsWith("vehicles/", StringComparison.OrdinalIgnoreCase)
				? normalized
				: null;
		}

		private static string? ReplaceExtension(string normalizedVehiclePath, string extension)
		{
			var directory = Path.GetDirectoryName(normalizedVehiclePath.Replace('/', Path.DirectorySeparatorChar));
			var filename = Path.GetFileNameWithoutExtension(normalizedVehiclePath);
			if (string.IsNullOrEmpty(filename))
			{
				return null;
			}

			var combined = directory == null
				? filename + extension
				: Path.Combine(directory, filename + extension);
			return combined.Replace('\\', '/');
		}

		private static bool ShouldExtractCommonMetadata(string entryPath)
		{
			var extension = Path.GetExtension(entryPath).ToLowerInvariant();
			return CommonMetadataExtensions.Contains(extension, StringComparer.OrdinalIgnoreCase);
		}

		private static void ExtractEntry(ZipArchiveEntry entry, string cacheRoot)
		{
			if (!TryNormalizeArchiveEntryPath(entry.FullName, out var normalizedEntryPath))
			{
				return;
			}

			var fullCacheRoot = Path.GetFullPath(cacheRoot);
			if (!fullCacheRoot.EndsWith(Path.DirectorySeparatorChar))
			{
				fullCacheRoot += Path.DirectorySeparatorChar;
			}

			var fullDestination = Path.GetFullPath(
				Path.Combine(cacheRoot, normalizedEntryPath.Replace('/', Path.DirectorySeparatorChar)));
			if (!fullDestination.StartsWith(fullCacheRoot, StringComparison.OrdinalIgnoreCase))
			{
				throw new InvalidDataException(
					$"Archive entry '{entry.FullName}' attempts path traversal outside cache root (possible Zip Slip attack).");
			}

			var destinationDirectory = Path.GetDirectoryName(fullDestination);
			if (destinationDirectory != null)
			{
				Directory.CreateDirectory(destinationDirectory);
			}

			if (string.IsNullOrEmpty(entry.Name))
			{
				Directory.CreateDirectory(fullDestination);
				return;
			}

			entry.ExtractToFile(fullDestination, overwrite: true);
		}

		private static bool TryNormalizeArchiveEntryPath(string entryPath, out string normalizedPath)
		{
			normalizedPath = string.Empty;
			if (string.IsNullOrWhiteSpace(entryPath))
			{
				return false;
			}

			var normalizedSeparators = entryPath.Replace('\\', '/');
			if (Path.IsPathRooted(normalizedSeparators))
			{
				return false;
			}

			var segments = normalizedSeparators
				.Split('/', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
			if (segments.Length == 0)
			{
				return false;
			}

			if (segments.Any(segment => segment is "." or ".."))
			{
				return false;
			}

			normalizedPath = string.Join('/', segments);
			return true;
		}

		private static string ComputeSourceKey(string sourcePath)
		{
			var info = new FileInfo(sourcePath);
			var fingerprint = $"{Path.GetFullPath(sourcePath)}|{info.Length}|{info.LastWriteTimeUtc.Ticks}";
			var hashBytes = SHA256.HashData(Encoding.UTF8.GetBytes(fingerprint));
			return Convert.ToHexString(hashBytes.AsSpan(0, 6)).ToLowerInvariant();
		}

		private static string SanitizeFileName(string value)
		{
			var invalid = Path.GetInvalidFileNameChars();
			var builder = new StringBuilder(value.Length);
			foreach (var ch in value)
			{
				builder.Append(invalid.Contains(ch) ? '_' : ch);
			}

			return builder.ToString();
		}

		private static IReadOnlyList<string> DistinctExistingPaths(IEnumerable<string> paths)
		{
			return paths
				.Where(path => !string.IsNullOrWhiteSpace(path))
				.Where(path => Directory.Exists(path))
				.Distinct(StringComparer.OrdinalIgnoreCase)
				.ToArray();
		}

		private readonly record struct VehicleInfo(
			string? Brand,
			string? Name,
			string? Description,
			string? DefaultConfigFileName);

		private readonly record struct VariantInfo(
			string? Configuration,
			string? Description,
			string? ConfigType);
	}
}
