using System;
using System.IO;
using System.IO.Compression;
using System.Linq;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Content;
using LibreRally.Vehicle.JBeam;
using Xunit;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the beam ng vehicle catalog behavior.
	/// </summary>
	public sealed class BeamNgVehicleCatalogTests
	{
		/// <summary>
		/// Verifies that discover bundled vehicles finds loose folders and zip packages.
		/// </summary>
		[Fact]
		public void DiscoverBundledVehicles_FindsLooseFoldersAndZipPackages()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			Directory.CreateDirectory(bundledRoot);

			string looseVehicle = Path.Combine(bundledRoot, "loose_car");
			Directory.CreateDirectory(looseVehicle);
			File.WriteAllText(Path.Combine(looseVehicle, "loose_car.jbeam"), "{}");

			string zipPath = Path.Combine(bundledRoot, "modpack.zip");
			CreateZip(zipPath, ("vehicles/zip_car/base.pc", "{}"));

			BeamNgVehicleCatalog catalog = new(bundledRoot, cacheRoot: Path.Combine(workspace.RootPath, "cache"));

			BeamNgVehicleDescriptor[] vehicles = catalog.DiscoverBundledVehicles().ToArray();

			Assert.Contains(vehicles, vehicle =>
				vehicle.VehicleId == "loose_car" &&
				vehicle.SourceKind == BeamNgVehicleSourceKind.Folder);
			Assert.Contains(vehicles, vehicle =>
				vehicle.VehicleId == "zip_car" &&
				vehicle.SourceKind == BeamNgVehicleSourceKind.ZipArchive);
		}

		/// <summary>
		/// Verifies that discover bundled vehicle variants reads config metadata and thumbnails.
		/// </summary>
		[Fact]
		public void DiscoverBundledVehicleVariants_ReadsConfigMetadataAndThumbnails()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			string vehicleFolder = Path.Combine(bundledRoot, "sunburst2");
			Directory.CreateDirectory(vehicleFolder);
			File.WriteAllText(Path.Combine(vehicleFolder, "sunburst2.jbeam"), "{}");
			File.WriteAllText(
				Path.Combine(vehicleFolder, "info.json"),
				"""
				{
				  "Brand": "Hirochi",
				  "Name": "Sunburst",
				  "default_pc": "rally_pro_asphalt"
				}
				""");
			File.WriteAllText(Path.Combine(vehicleFolder, "rally_pro_asphalt.pc"), "{}");
			File.WriteAllText(Path.Combine(vehicleFolder, "base_CVT.pc"), "{}");
			File.WriteAllText(
				Path.Combine(vehicleFolder, "info_rally_pro_asphalt.json"),
				"""
				{
				  "Configuration": "NGRC1 - Asphalt Rally (Sequential)",
				  "Description": "Full championship-spec rally car for asphalt stages",
				  "Config Type": "Rally"
				}
				""");
			File.WriteAllText(Path.Combine(vehicleFolder, "rally_pro_asphalt.jpg"), "thumbnail");
			File.WriteAllText(Path.Combine(vehicleFolder, "default.jpg"), "fallback");

			BeamNgVehicleCatalog catalog = new(bundledRoot, cacheRoot: Path.Combine(workspace.RootPath, "cache"));

			BeamNgVehicleVariantDescriptor[] variants = catalog.DiscoverBundledVehicleVariants().ToArray();
			BeamNgVehicleVariantDescriptor rallyVariant = Assert.Single(variants, variant =>
				variant.ConfigFileName == "rally_pro_asphalt.pc");
			BeamNgVehicleVariantDescriptor baseVariant = Assert.Single(variants, variant =>
				variant.ConfigFileName == "base_CVT.pc");

			Assert.Equal("Hirochi Sunburst", rallyVariant.VehicleDisplayName);
			Assert.Equal("NGRC1 - Asphalt Rally (Sequential)", rallyVariant.VariantDisplayName);
			Assert.Equal("Rally", rallyVariant.ConfigType);
			Assert.True(rallyVariant.IsDefaultVariant);
			Assert.EndsWith("rally_pro_asphalt.jpg", rallyVariant.ThumbnailPath, StringComparison.OrdinalIgnoreCase);
			Assert.EndsWith("default.jpg", baseVariant.ThumbnailPath, StringComparison.OrdinalIgnoreCase);
		}

		/// <summary>
		/// Verifies that discover bundled vehicle variants extracts zip variants before reading metadata.
		/// </summary>
		[Fact]
		public void DiscoverBundledVehicleVariants_ExtractsZipVariantsBeforeReadingMetadata()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			Directory.CreateDirectory(bundledRoot);

			string zipPath = Path.Combine(bundledRoot, "sunburst-pack.zip");
			CreateZip(
				zipPath,
				("vehicles/sunburst2/info.json", "{ \"Brand\": \"Hirochi\", \"Name\": \"Sunburst\", \"default_pc\": \"track_attack\" }"),
				("vehicles/sunburst2/track_attack.pc", "{}"),
				("vehicles/sunburst2/info_track_attack.json", "{ \"Configuration\": \"Track Attack\", \"Config Type\": \"Race\" }"),
				("vehicles/sunburst2/track_attack.jpg", "thumbnail"));

			BeamNgVehicleCatalog catalog = new(bundledRoot, cacheRoot: Path.Combine(workspace.RootPath, "cache"));

			BeamNgVehicleVariantDescriptor variant = Assert.Single(catalog.DiscoverBundledVehicleVariants());

			Assert.Equal("sunburst2", variant.VehicleId);
			Assert.Equal("track_attack.pc", variant.ConfigFileName);
			Assert.Equal("Track Attack", variant.VariantDisplayName);
			Assert.True(variant.IsDefaultVariant);
			Assert.Equal(BeamNgVehicleSourceKind.ZipArchive, variant.SourceKind);
			Assert.True(File.Exists(variant.ThumbnailPath));
		}

		/// <summary>
		/// Verifies that resolve vehicle extracts zip packages and common metadata.
		/// </summary>
		[Fact]
		public void ResolveVehicle_ExtractsZipPackagesAndCommonMetadata()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			string beamNgContentRoot = Path.Combine(workspace.RootPath, "beamng");
			Directory.CreateDirectory(bundledRoot);
			Directory.CreateDirectory(beamNgContentRoot);

			string zipPath = Path.Combine(bundledRoot, "modpack.zip");
			CreateZip(zipPath,
				("vehicles/zip_car/base.pc", "{ \"parts\": {}, \"vars\": {} }"),
				("vehicles/zip_car/zip_car.jbeam", "{}"));
			CreateZip(Path.Combine(beamNgContentRoot, "common.zip"),
				("vehicles/common/shared/common_part.jbeam", "{}"));

			BeamNgVehicleCatalog catalog = new(
				bundledRoot,
				beamNgContentRoot,
				Path.Combine(workspace.RootPath, "cache"));

			BeamNgResolvedVehicle resolved = catalog.ResolveVehicle("zip_car");

			Assert.True(Directory.Exists(resolved.VehicleFolderPath));
			Assert.True(File.Exists(Path.Combine(resolved.VehicleFolderPath, "base.pc")));
			string commonSearchFolder = Assert.Single(resolved.JBeamSearchFolders.Where(path =>
				path.EndsWith(Path.Combine("vehicles", "common"), StringComparison.OrdinalIgnoreCase)));
			Assert.True(File.Exists(Path.Combine(commonSearchFolder, "shared", "common_part.jbeam")));
		}

		/// <summary>
		/// Verifies that resolve vehicle does not extract unsafe zip paths outside cache.
		/// </summary>
		[Fact]
		public void ResolveVehicle_DoesNotExtractUnsafeZipPathsOutsideCache()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			string cacheRoot = Path.Combine(workspace.RootPath, "cache");
			Directory.CreateDirectory(bundledRoot);

			string zipPath = Path.Combine(bundledRoot, "modpack.zip");
			CreateZip(zipPath,
				("vehicles/zip_car/base.pc", "{ \"parts\": {}, \"vars\": {} }"),
				("vehicles/zip_car/zip_car.jbeam", "{}"),
				("../outside.txt", "should-not-write"));

			BeamNgVehicleCatalog catalog = new(bundledRoot, cacheRoot: cacheRoot);

			BeamNgResolvedVehicle resolved = catalog.ResolveVehicle("zip_car");

			Assert.True(File.Exists(Path.Combine(resolved.VehicleFolderPath, "base.pc")));
			Assert.False(File.Exists(Path.Combine(workspace.RootPath, "outside.txt")));
		}

		/// <summary>
		/// Verifies that resolve vehicle asset path materializes beam ng content assets on demand.
		/// </summary>
		[Fact]
		public void ResolveVehicleAssetPath_MaterializesBeamNgContentAssetsOnDemand()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			string beamNgContentRoot = Path.Combine(workspace.RootPath, "beamng");
			Directory.CreateDirectory(bundledRoot);
			Directory.CreateDirectory(beamNgContentRoot);

			string looseVehicle = Path.Combine(bundledRoot, "loose_car");
			Directory.CreateDirectory(looseVehicle);
			File.WriteAllText(Path.Combine(looseVehicle, "loose_car.jbeam"), "{}");

			CreateZip(Path.Combine(beamNgContentRoot, "common.zip"),
				("vehicles/common/nullcolormaskR.color.png", "png-data"));

			BeamNgVehicleCatalog catalog = new(
				bundledRoot,
				beamNgContentRoot,
				Path.Combine(workspace.RootPath, "cache"));

			BeamNgResolvedVehicle resolved = catalog.ResolveVehicle("loose_car");
			string materialized = Assert.IsType<string>(resolved.ResolveVehicleAssetPath("/vehicles/common/nullcolormaskR.color.dds"));

			Assert.True(File.Exists(materialized));
			Assert.Equal("png-data", File.ReadAllText(materialized));
		}

		/// <summary>
		/// Verifies that resolve collada files for meshes materializes shared common dae on demand.
		/// </summary>
		[Fact]
		public void ResolveColladaFilesForMeshes_MaterializesSharedCommonDaeOnDemand()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			string beamNgContentRoot = Path.Combine(workspace.RootPath, "beamng");
			Directory.CreateDirectory(bundledRoot);
			Directory.CreateDirectory(beamNgContentRoot);

			string looseVehicle = Path.Combine(bundledRoot, "loose_car");
			Directory.CreateDirectory(looseVehicle);
			File.WriteAllText(Path.Combine(looseVehicle, "loose_car.jbeam"), "{}");

			CreateZip(
				Path.Combine(beamNgContentRoot, "common.zip"),
				("vehicles/common/shared/common_part.jbeam", "{}"),
				("vehicles/common/wheels/wheels_collection/generic_wheels.dae", "<COLLADA><geometry id=\"wheel_11a_18x8\" /></COLLADA>"),
				("vehicles/common/wheels/main.materials.json", "{ }"),
				("vehicles/common/main.materials.json", "{ }"));

			BeamNgVehicleCatalog catalog = new(
				bundledRoot,
				beamNgContentRoot,
				Path.Combine(workspace.RootPath, "cache"));

			BeamNgResolvedVehicle resolved = catalog.ResolveVehicle("loose_car");
			string colladaFile = Assert.Single(resolved.ResolveColladaFilesForMeshes(["wheel_11a_18x8"]));
			string wheelsMaterialFile = Path.Combine(
				Path.GetDirectoryName(Path.GetDirectoryName(colladaFile)!)!,
				"main.materials.json");
			string commonMaterialFile = Path.Combine(
				Path.GetDirectoryName(Path.GetDirectoryName(Path.GetDirectoryName(colladaFile)!)!)!,
				"main.materials.json");

			Assert.True(File.Exists(colladaFile));
			Assert.Contains("generic_wheels.dae", colladaFile, StringComparison.OrdinalIgnoreCase);
			Assert.True(File.Exists(wheelsMaterialFile));
			Assert.True(File.Exists(commonMaterialFile));
		}

		/// <summary>
		/// Verifies that resolve collada files for meshes searches other installed vehicle packages.
		/// </summary>
		[Fact]
		public void ResolveColladaFilesForMeshes_SearchesOtherInstalledVehiclePackages()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			string beamNgContentRoot = Path.Combine(workspace.RootPath, "beamng");
			Directory.CreateDirectory(bundledRoot);
			Directory.CreateDirectory(beamNgContentRoot);

			string looseVehicle = Path.Combine(bundledRoot, "basic_car");
			Directory.CreateDirectory(looseVehicle);
			File.WriteAllText(Path.Combine(looseVehicle, "basic_car.jbeam"), "{}");

			CreateZip(
				Path.Combine(beamNgContentRoot, "autobello.zip"),
				("vehicles/autobello/wheels/autobello_wheels.dae", "<COLLADA><geometry id=\"steelwheel_11a_13x5\" /><geometry id=\"tire_03c_13x6_24\" /></COLLADA>"),
				("vehicles/autobello/wheels/main.materials.json", "{ }"),
				("vehicles/autobello/main.materials.json", "{ }"));

			BeamNgVehicleCatalog catalog = new(
				bundledRoot,
				beamNgContentRoot,
				Path.Combine(workspace.RootPath, "cache"));

			BeamNgResolvedVehicle resolved = catalog.ResolveVehicle("basic_car");
			string colladaFile = Assert.Single(resolved.ResolveColladaFilesForMeshes(["steelwheel_11a_13x5", "tire_03c_13x6_24"]));
			string packageMaterialFile = Path.Combine(Path.GetDirectoryName(colladaFile)!, "main.materials.json");
			string rootMaterialFile = Path.Combine(Path.GetDirectoryName(Path.GetDirectoryName(colladaFile)!)!, "main.materials.json");

			Assert.True(File.Exists(colladaFile));
			Assert.Contains("autobello_wheels.dae", colladaFile, StringComparison.OrdinalIgnoreCase);
			Assert.True(File.Exists(packageMaterialFile));
			Assert.True(File.Exists(rootMaterialFile));
		}

		/// <summary>
		/// Verifies that resolve collada files for meshes prefers plain dae over compressed cdae.
		/// </summary>
		[Fact]
		public void ResolveColladaFilesForMeshes_PrefersPlainDaeOverCompressedCdae()
		{
			using var workspace = new TempWorkspace();
			string bundledRoot = Path.Combine(workspace.RootPath, "bundled");
			string beamNgContentRoot = Path.Combine(workspace.RootPath, "beamng");
			Directory.CreateDirectory(bundledRoot);
			Directory.CreateDirectory(beamNgContentRoot);

			string looseVehicle = Path.Combine(bundledRoot, "basic_car");
			Directory.CreateDirectory(looseVehicle);
			File.WriteAllText(Path.Combine(looseVehicle, "basic_car.jbeam"), "{}");

			CreateZip(
				Path.Combine(beamNgContentRoot, "common.zip"),
				("vehicles/common/wheels/wheels_collection/steelwheel.cdae", "\u001Fbinary-placeholder"),
				("vehicles/common/wheels/wheels_collection/steelwheel.dae", "<COLLADA><geometry id=\"steelwheel_11a_13x5\" /></COLLADA>"),
				("vehicles/common/wheels/main.materials.json", "{ }"),
				("vehicles/common/main.materials.json", "{ }"));

			BeamNgVehicleCatalog catalog = new(
				bundledRoot,
				beamNgContentRoot,
				Path.Combine(workspace.RootPath, "cache"));

			BeamNgResolvedVehicle resolved = catalog.ResolveVehicle("basic_car");
			string colladaFile = Assert.Single(resolved.ResolveColladaFilesForMeshes(["steelwheel_11a_13x5"]));

			Assert.EndsWith(".dae", colladaFile, StringComparison.OrdinalIgnoreCase);
			Assert.False(colladaFile.EndsWith(".cdae", StringComparison.OrdinalIgnoreCase));
		}

		/// <summary>
		/// Verifies that assemble search folders resolves supplemental common parts.
		/// </summary>
		[Fact]
		public void Assemble_SearchFolders_ResolvesSupplementalCommonParts()
		{
			using var workspace = new TempWorkspace();
			string vehicleFolder = Path.Combine(workspace.RootPath, "vehicle");
			string commonFolder = Path.Combine(workspace.RootPath, "common");
			Directory.CreateDirectory(vehicleFolder);
			Directory.CreateDirectory(commonFolder);

			File.WriteAllText(
				Path.Combine(vehicleFolder, "main.jbeam"),
				"""
				{
				  "test_car": {
				    "slotType":"main",
				    "slots":[
				      ["type","default","description"],
				      ["shared_slot","shared_part","Shared slot"]
				    ]
				  }
				}
				""");

			File.WriteAllText(
				Path.Combine(commonFolder, "shared.jbeam"),
				"""
				{
				  "shared_part": {
				    "slotType":"shared_slot",
				    "nodes":[
				      ["id","posX","posY","posZ"],
				      ["w1", 1, 2, 3]
				    ]
				  }
				}
				""");

			VehicleDefinition definition = JBeamAssembler.Assemble([vehicleFolder, commonFolder], vehicleFolder);

			Assert.Contains("w1", definition.Nodes.Keys);
		}

		/// <summary>
		/// Verifies that assemble blank pc override falls back to slot default.
		/// </summary>
		[Fact]
		public void Assemble_BlankPcOverride_FallsBackToSlotDefault()
		{
			using var workspace = new TempWorkspace();
			string vehicleFolder = Path.Combine(workspace.RootPath, "vehicle");
			Directory.CreateDirectory(vehicleFolder);

			File.WriteAllText(
				Path.Combine(vehicleFolder, "main.jbeam"),
				"""
				{
				  "test_car": {
				    "slotType":"main",
				    "slots":[
				      ["type","default","description"],
				      ["shared_slot","shared_part","Shared slot"]
				    ]
				  },
				  "shared_part": {
				    "slotType":"shared_slot",
				    "nodes":[
				      ["id","posX","posY","posZ"],
				      ["w1", 1, 2, 3]
				    ]
				  }
				}
				""");

			PcConfig pcConfig = new() { MainPartName = "test_car", Parts = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase) { ["shared_slot"] = "", }, Vars = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase), };

			VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolder, pcConfig);

			Assert.Contains("w1", definition.Nodes.Keys);
		}

		/// <summary>
		/// Verifies that assemble wheel slot offset applies outward per side.
		/// </summary>
		[Fact]
		public void Assemble_WheelSlotOffset_AppliesOutwardPerSide()
		{
			using var workspace = new TempWorkspace();
			string vehicleFolder = Path.Combine(workspace.RootPath, "vehicle");
			Directory.CreateDirectory(vehicleFolder);

			File.WriteAllText(
				Path.Combine(vehicleFolder, "main.jbeam"),
				"""
				{
				  "test_car": {
				    "slotType":"main",
				    "slots":[
				      ["type","default","description"],
				      ["wheel_F_4","wheel_pair","Front wheels", {"nodeOffset":{"x":0.175,"y":-1.12738,"z":0.29476}}]
				    ]
				  },
				  "wheel_pair": {
				    "slotType":"wheel_F_4",
				    "flexbodies":[
				      ["mesh","[group]:","nonFlexMaterials"],
				      ["steelwheel", ["wheel_FR","wheelhub_FR"], [], {"pos":{"x":-0.48, "y":0.0, "z":0.0}}],
				      ["steelwheel", ["wheel_FL","wheelhub_FL"], [], {"pos":{"x":0.48, "y":0.0, "z":0.0}}]
				    ],
				    "nodes":[
				      ["id", "posX", "posY", "posZ"],
				      {"group":"wheelhub_FR"},
				      ["fw1r", -0.33, 0.0, 0.0],
				      {"group":"wheelhub_FL"},
				      ["fw1l", 0.33, 0.0, 0.0]
				    ]
				  }
				}
				""");

			VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolder);
			AssembledFlexBody rightWheel = Assert.Single(definition.FlexBodies, flexBody => flexBody.NodeGroups.Contains("wheel_FR"));
			AssembledFlexBody leftWheel = Assert.Single(definition.FlexBodies, flexBody => flexBody.NodeGroups.Contains("wheel_FL"));

			Assert.Equal(-0.655f, rightWheel.Position!.Value.X, 3);
			Assert.Equal(0.655f, leftWheel.Position!.Value.X, 3);
			Assert.Equal(-0.505f, definition.Nodes["fw1r"].Position.X, 3);
			Assert.Equal(0.505f, definition.Nodes["fw1l"].Position.X, 3);
			Assert.Equal(-1.12738f, rightWheel.Position.Value.Y, 5);
			Assert.Equal(0.29476f, rightWheel.Position.Value.Z, 5);
		}

		private static void CreateZip(string zipPath, params (string EntryPath, string Content)[] entries)
		{
			using ZipArchive archive = ZipFile.Open(zipPath, ZipArchiveMode.Create);
			foreach ((string entryPath, string content) in entries)
			{
				ZipArchiveEntry entry = archive.CreateEntry(entryPath);
				using StreamWriter writer = new(entry.Open());
				writer.Write(content);
			}
		}

		private sealed class TempWorkspace : IDisposable
		{
			public TempWorkspace()
			{
				RootPath = Path.Combine(Path.GetTempPath(), "LibreRally.Tests", Guid.NewGuid().ToString("N"));
				Directory.CreateDirectory(RootPath);
			}

			public string RootPath { get; }

			public void Dispose()
			{
				if (Directory.Exists(RootPath))
				{
					Directory.Delete(RootPath, recursive: true);
				}
			}
		}
	}
}
