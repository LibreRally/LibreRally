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
	public sealed class BeamNgVehicleCatalogTests
	{
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

			PcConfig pcConfig = new()
			{
				MainPartName = "test_car",
				Parts = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase)
				{
					["shared_slot"] = "",
				},
				Vars = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase),
			};

			VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolder, pcConfig);

			Assert.Contains("w1", definition.Nodes.Keys);
		}

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
