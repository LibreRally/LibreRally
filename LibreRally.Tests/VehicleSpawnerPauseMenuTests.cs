using LibreRally;

namespace LibreRally.Tests
{
	public class VehicleSpawnerPauseMenuTests
	{
		[Fact]
		public void PauseMenuEntries_ExposeRespawnSetupAndVehicleSelectActions()
		{
			var entries = VehicleSpawner.GetPauseMenuEntries();

			Assert.Collection(
				entries,
				entry =>
				{
					Assert.Equal(PauseMenuAction.ResumeDriving, entry.Action);
					Assert.Equal("Resume Driving", entry.Item.Title);
				},
				entry =>
				{
					Assert.Equal(PauseMenuAction.ResetVehicle, entry.Action);
					Assert.Equal("Reset Vehicle", entry.Item.Title);
					Assert.Contains("spawn point", entry.Item.Description, StringComparison.OrdinalIgnoreCase);
				},
				entry => Assert.Equal(PauseMenuAction.GarageSetup, entry.Action),
				entry => Assert.Equal(PauseMenuAction.VehicleSelect, entry.Action),
				entry => Assert.Equal(PauseMenuAction.PhysicsCalibration, entry.Action),
				entry =>
				{
					Assert.Equal(PauseMenuAction.Telemetry, entry.Action);
					Assert.Equal("Telemetry", entry.Item.Title);
				});
		}

		[Fact]
		public void ResolvePauseMenuAction_ReturnsGarageSetupForThirdSlot()
		{
			var action = VehicleSpawner.ResolvePauseMenuAction(2);

			Assert.Equal(PauseMenuAction.GarageSetup, action);
		}

		[Fact]
		public void ResolvePauseMenuAction_ReturnsTelemetryForSixthSlot()
		{
			var action = VehicleSpawner.ResolvePauseMenuAction(5);

			Assert.Equal(PauseMenuAction.Telemetry, action);
		}
	}
}
