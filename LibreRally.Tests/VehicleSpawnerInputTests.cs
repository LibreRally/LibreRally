namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the vehicle spawner input behavior.
	/// </summary>
	public class VehicleSpawnerInputTests
	{
		/// <summary>
		/// Verifies that is pause menu toggle requested accepts controller start.
		/// </summary>
		[Fact]
		public void IsPauseMenuToggleRequested_AcceptsControllerStart()
		{
			bool requested = VehicleSpawner.IsPauseMenuToggleRequested(
				keyboardPausePressed: false,
				controllerStartPressed: true);

			Assert.True(requested);
		}

		/// <summary>
		/// Verifies that is vehicle menu shortcut requested only accepts keyboard shortcut.
		/// </summary>
		[Fact]
		public void IsVehicleMenuShortcutRequested_OnlyAcceptsKeyboardShortcut()
		{
			Assert.True(VehicleSpawner.IsVehicleMenuShortcutRequested(keyboardTogglePressed: true));
			Assert.False(VehicleSpawner.IsVehicleMenuShortcutRequested(keyboardTogglePressed: false));
		}

		/// <summary>
		/// Verifies that resolve pause menu toggle routes start to pause flow.
		/// </summary>
		[Fact]
		public void ResolvePauseMenuToggle_RoutesStartToPauseFlow()
		{
			Assert.Equal(VehicleSpawner.MenuScreen.Pause, VehicleSpawner.ResolvePauseMenuToggle(VehicleSpawner.MenuScreen.None));
			Assert.Equal(VehicleSpawner.MenuScreen.None, VehicleSpawner.ResolvePauseMenuToggle(VehicleSpawner.MenuScreen.Pause));
			Assert.Equal(VehicleSpawner.MenuScreen.Pause, VehicleSpawner.ResolvePauseMenuToggle(VehicleSpawner.MenuScreen.VehicleSelection));
		}

		/// <summary>
		/// Verifies that resolve vehicle selection close target returns pause only when opened from pause.
		/// </summary>
		[Fact]
		public void ResolveVehicleSelectionCloseTarget_ReturnsPauseOnlyWhenOpenedFromPause()
		{
			Assert.Equal(VehicleSpawner.MenuScreen.Pause, VehicleSpawner.ResolveVehicleSelectionCloseTarget(openedFromPauseMenu: true));
			Assert.Equal(VehicleSpawner.MenuScreen.None, VehicleSpawner.ResolveVehicleSelectionCloseTarget(openedFromPauseMenu: false));
		}

		/// <summary>
		/// Verifies that resolve telemetry menu action maps indexes to expected actions.
		/// </summary>
		[Fact]
		public void ResolveTelemetryMenuAction_MapsIndexesToExpectedActions()
		{
			Assert.Equal(TelemetryMenuAction.ToggleOutGauge, VehicleSpawner.ResolveTelemetryMenuAction(0));
			Assert.Equal(TelemetryMenuAction.ReconnectOutGauge, VehicleSpawner.ResolveTelemetryMenuAction(3));
			Assert.Equal(TelemetryMenuAction.ToggleOutSim, VehicleSpawner.ResolveTelemetryMenuAction(4));
			Assert.Equal(TelemetryMenuAction.ReconnectOutSim, VehicleSpawner.ResolveTelemetryMenuAction(7));
		}

		/// <summary>
		/// Verifies that is vehicle menu confirm requested accepts controller a.
		/// </summary>
		[Fact]
		public void IsVehicleMenuConfirmRequested_AcceptsControllerA()
		{
			bool requested = VehicleSpawner.IsVehicleMenuConfirmRequested(
				keyboardConfirmPressed: false,
				controllerConfirmPressed: true);

			Assert.True(requested);
		}

		/// <summary>
		/// Verifies that is vehicle menu cancel requested accepts controller b.
		/// </summary>
		[Fact]
		public void IsVehicleMenuCancelRequested_AcceptsControllerB()
		{
			bool requested = VehicleSpawner.IsVehicleMenuCancelRequested(
				keyboardCancelPressed: false,
				controllerCancelPressed: true);

			Assert.True(requested);
		}

		/// <summary>
		/// Verifies that is vehicle menu move down requested accepts controller d pad down.
		/// </summary>
		[Fact]
		public void IsVehicleMenuMoveDownRequested_AcceptsControllerDPadDown()
		{
			bool requested = VehicleSpawner.IsVehicleMenuMoveDownRequested(
				keyboardDownPressed: false,
				controllerDownPressed: true);

			Assert.True(requested);
		}

		/// <summary>
		/// Verifies that is vehicle menu move up requested accepts controller d pad up.
		/// </summary>
		[Fact]
		public void IsVehicleMenuMoveUpRequested_AcceptsControllerDPadUp()
		{
			bool requested = VehicleSpawner.IsVehicleMenuMoveUpRequested(
				keyboardUpPressed: false,
				controllerUpPressed: true);

			Assert.True(requested);
		}
	}
}
