using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies effective setup-variable resolution across surface-specific suffixes.
	/// </summary>
	public class VehicleSetupValueResolverTests
	{
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

			throw new DirectoryNotFoundException($"Could not locate the repository root for '{vehicleFolderName}' tests.");
		}

		/// <summary>
		/// Verifies that active pc suspension vars override assembled definition defaults.
		/// </summary>
		[Fact]
		public void SunburstRallyAsphalt_PrefersPcSuspensionValuesOverDefinitionDefaults()
		{
			string vehicleFolder = GetVehicleFolder("sunburst2");
			PcConfig config = PcConfigLoader.Load(Path.Combine(vehicleFolder, "rally_am_asphalt.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolder, config);

			Assert.Equal(30000f, VehicleSetupValueResolver.GetPositiveValue(config.Vars, definition.Vars, 0f, "spring_F"));
			Assert.Equal(22000f, VehicleSetupValueResolver.GetPositiveValue(config.Vars, definition.Vars, 0f, "spring_R"));
			Assert.Equal(4200f, VehicleSetupValueResolver.GetPositiveValue(config.Vars, definition.Vars, 0f, "damp_bump_F"));
			Assert.Equal(3600f, VehicleSetupValueResolver.GetPositiveValue(config.Vars, definition.Vars, 0f, "damp_bump_R"));
		}

		/// <summary>
		/// Verifies that asphalt-specific values still win over rally values when both are present.
		/// </summary>
		[Fact]
		public void GetPositiveValue_PrefersAsphaltSuffixOverRally()
		{
			Dictionary<string, float> vars = new(StringComparer.OrdinalIgnoreCase)
			{
				["spring_F_asphalt"] = 60000f,
				["spring_F_rally"] = 30000f,
				["spring_F"] = 25000f,
			};

			Assert.Equal(60000f, VehicleSetupValueResolver.GetPositiveValue(vars, 0f, "spring_F"));
		}

		/// <summary>
		/// Verifies that signed setup values can resolve from rally-specific vars.
		/// </summary>
		[Fact]
		public void GetFiniteValue_UsesRallySuffixForSignedValues()
		{
			Dictionary<string, float> vars = new(StringComparer.OrdinalIgnoreCase)
			{
				["springheight_F_rally"] = -0.025f,
				["springheight_F"] = 0f,
			};

			Assert.Equal(-0.025f, VehicleSetupValueResolver.GetFiniteValue(vars, 0f, "springheight_F"));
		}
	}
}
