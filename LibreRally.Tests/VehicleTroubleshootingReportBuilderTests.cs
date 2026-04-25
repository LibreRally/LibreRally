using LibreRally.Vehicle;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the reusable troubleshooting report used for A/B vehicle-physics investigations.
	/// </summary>
	public class VehicleTroubleshootingReportBuilderTests
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
		/// Verifies that the troubleshooting report prefers the active pc setup values and produces comparable probe variants.
		/// </summary>
		[Fact]
		public void SunburstTroubleshootingReport_PrefersActivePcValuesAndBuildsProbeVariants()
		{
			var report = VehicleTroubleshootingReportBuilder.Build(GetVehicleFolder("sunburst2"), "rally_am_asphalt.pc");

			var springFront = Assert.Single(report.SetupValues, value => value.Name == "spring_F");
			var springRear = Assert.Single(report.SetupValues, value => value.Name == "spring_R");
			Assert.Equal(30000f, springFront.EffectiveValue);
			Assert.Equal("spring_F_rally", springFront.EffectiveSource);
			Assert.Equal(22000f, springRear.EffectiveValue);
			Assert.Equal("spring_R_rally", springRear.EffectiveSource);

			Assert.Contains(report.TyreProbes, probe => probe.VariantName == "Loaded Auto");
			Assert.Contains(report.TyreProbes, probe => probe.VariantName == "Loaded Brush");
			Assert.Contains(report.TyreProbes, probe => probe.VariantName == "Loaded Pacejka");
			Assert.Contains(report.TyreProbes, probe => probe.VariantName == "Neutral Tyre Auto");
			Assert.All(report.TyreProbes, probe =>
			{
				Assert.True(float.IsFinite(probe.FinalSpeedMs));
				Assert.True(float.IsFinite(probe.PeakAccelerationMs2));
				Assert.True(float.IsFinite(probe.PeakSlipRatio));
				Assert.True(float.IsFinite(probe.PeakLongitudinalForcePerDrivenWheelN));
			});

			var loadedBrush = Assert.Single(report.TyreProbes, probe => probe.VariantName == "Loaded Brush");
			var loadedPacejka = Assert.Single(report.TyreProbes, probe => probe.VariantName == "Loaded Pacejka");
			Assert.NotEqual(loadedBrush.FinalSpeedMs, loadedPacejka.FinalSpeedMs);
		}

		/// <summary>
		/// Verifies that the troubleshooting report formats a readable summary for human comparison.
		/// </summary>
		[Fact]
		public void BasicCarTroubleshootingReport_FormatsReadableSummary()
		{
			var report = VehicleTroubleshootingReportBuilder.Build(GetVehicleFolder("basic_car"), "basic_car.pc");
			var text = report.Format();

			Assert.Contains("Vehicle folder:", text);
			Assert.Contains("Setup values:", text);
			Assert.Contains("Tyre probes:", text);
			Assert.Contains("Observations:", text);
		}
	}
}
