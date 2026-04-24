using System;
using System.IO;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Rendering;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the beam ng material loader behavior.
	/// </summary>
	public class BeamNgMaterialLoaderTests
	{
		private static string CombineRelativePath(string basePath, string relativePath)
		{
			if (Path.IsPathRooted(relativePath))
			{
				throw new ArgumentException("Path must be relative.", nameof(relativePath));
			}

			return Path.Combine(basePath, relativePath);
		}

		private static string GetVehicleFolder(string vehicleFolderName)
		{
			DirectoryInfo? directory = new(AppContext.BaseDirectory);
			while (directory != null)
			{
				string candidate = Path.Combine(directory.FullName, "LibreRally.sln");
				if (File.Exists(candidate))
				{
					return Path.Combine(directory.FullName, "LibreRally", "Resources", "BeamNG Vehicles", vehicleFolderName);
				}

				directory = directory.Parent;
			}

			throw new DirectoryNotFoundException($"Could not locate the repository root for '{vehicleFolderName}' material tests.");
		}

		/// <summary>
		/// Verifies that rally pro gravel should expose selected material skin variants.
		/// </summary>
		[Fact]
		public void RallyProGravel_ShouldExposeSelectedMaterialSkinVariants()
		{
			string vehicleFolder = GetVehicleFolder("sunburst2");
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(vehicleFolder, "rally_pro_gravel.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolder, config);

			Assert.Contains(definition.ActiveMaterialSkinSelections,
				selection => selection.SlotType == "paint_design" && selection.VariantName == "ngrc");
			Assert.Contains(definition.ActiveMaterialSkinSelections,
				selection => selection.SlotType == "skin_interior" && selection.VariantName == "rs");
		}

		/// <summary>
		/// Verifies that rally pro gravel should resolve skin material textures over base maps.
		/// </summary>
		[Fact]
		public void RallyProGravel_ShouldResolveSkinMaterialTexturesOverBaseMaps()
		{
			string vehicleFolder = GetVehicleFolder("sunburst2");
			string vehiclesRoot = Directory.GetParent(vehicleFolder)!.FullName;
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(vehicleFolder, "rally_pro_gravel.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolder, config);

			var textures = BeamNGMaterialLoader.LoadMaterialTextures(
				[vehicleFolder],
				[vehiclesRoot],
				null,
				definition.ActiveMaterialSkinSelections);

			Assert.EndsWith("sunburst2_skin_ngrc_b.color.dds", textures["sunburst2_main"], StringComparison.OrdinalIgnoreCase);
			Assert.EndsWith("sunburst2_interior_rs_b.color.dds", textures["sunburst2_interior"], StringComparison.OrdinalIgnoreCase);
			Assert.EndsWith("sunburst2_skin_ngrc_b.color.dds", textures["sunburst2_main.skin.ngrc"], StringComparison.OrdinalIgnoreCase);
		}

		/// <summary>
		/// Verifies that rally pro gravel should expose palette texture for instance diffuse materials.
		/// </summary>
		[Fact]
		public void RallyProGravel_ShouldExposePaletteTextureForInstanceDiffuseMaterials()
		{
			string vehicleFolder = GetVehicleFolder("sunburst2");
			string vehiclesRoot = Directory.GetParent(vehicleFolder)!.FullName;
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(vehicleFolder, "rally_pro_gravel.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolder, config);

			var textures = BeamNGMaterialLoader.LoadMaterialTextureSets(
				[vehicleFolder],
				[vehiclesRoot],
				null,
				definition.ActiveMaterialSkinSelections);

			Assert.True(textures["sunburst2_main"].UsesInstanceDiffuse);
			Assert.NotNull(textures["sunburst2_main"].ColorPalettePath);
			Assert.EndsWith(
				"sunburst2_skin_ngrc_p.color.dds",
				textures["sunburst2_main"].ColorPalettePath,
				StringComparison.OrdinalIgnoreCase);
		}
	}
}
