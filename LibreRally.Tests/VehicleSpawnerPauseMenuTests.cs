namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the vehicle spawner pause menu behavior.
	/// </summary>
	public class VehicleSpawnerPauseMenuTests
	{
		/// <summary>
		/// Verifies that pause menu entries expose respawn setup and vehicle select actions.
		/// </summary>
		[Fact]
		public void PauseMenuEntries_ExposeRespawnSetupAndVehicleSelectActions()
		{
			var entries = VehicleSpawner.GetPauseMenuEntries();

			Assert.Collection(
				entries,
				entry => AssertEntry(entry, PauseMenuAction.ResumeDriving, "Resume Driving"),
				entry => AssertEntry(entry, PauseMenuAction.ResetVehicle, "Reset Vehicle", "spawn point"),
				entry => AssertEntry(entry, PauseMenuAction.GarageSetup, "Garage Setup"),
				entry => AssertEntry(entry, PauseMenuAction.VehicleSelect, "Vehicle Select"),
				entry => AssertEntry(entry, PauseMenuAction.PhysicsCalibration, "Physics Calibration"),
				entry => AssertEntry(entry, PauseMenuAction.Telemetry, "Telemetry"));

			static void AssertEntry(PauseMenuEntry entry, PauseMenuAction action, string title, string? descriptionPart = null)
			{
				Assert.Equal(action, entry.Action);
				Assert.Equal(title, entry.Item.Title);

				if (descriptionPart != null)
				{
					Assert.Contains(descriptionPart, entry.Item.Description, StringComparison.OrdinalIgnoreCase);
				}
			}
		}

		/// <summary>
		/// Verifies that resolve pause menu action returns garage setup for third slot.
		/// </summary>
		[Fact]
		public void ResolvePauseMenuAction_ReturnsGarageSetupForThirdSlot()
		{
			var action = VehicleSpawner.ResolvePauseMenuAction(2);

			Assert.Equal(PauseMenuAction.GarageSetup, action);
		}

		/// <summary>
		/// Verifies that resolve pause menu action returns telemetry for sixth slot.
		/// </summary>
		[Fact]
		public void ResolvePauseMenuAction_ReturnsTelemetryForSixthSlot()
		{
			var action = VehicleSpawner.ResolvePauseMenuAction(5);

			Assert.Equal(PauseMenuAction.Telemetry, action);
		}
	}
}
