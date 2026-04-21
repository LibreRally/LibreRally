using LibreRally;

namespace LibreRally.Tests
{
	public class VehicleSpawnerInputTests
	{
		[Fact]
		public void IsPauseMenuToggleRequested_AcceptsControllerStart()
		{
			bool requested = VehicleSpawner.IsPauseMenuToggleRequested(
				keyboardPausePressed: false,
				controllerStartPressed: true);

			Assert.True(requested);
		}

		[Fact]
		public void IsVehicleMenuShortcutRequested_OnlyAcceptsKeyboardShortcut()
		{
			Assert.True(VehicleSpawner.IsVehicleMenuShortcutRequested(keyboardTogglePressed: true));
			Assert.False(VehicleSpawner.IsVehicleMenuShortcutRequested(keyboardTogglePressed: false));
		}

		[Fact]
		public void ResolvePauseMenuToggle_RoutesStartToPauseFlow()
		{
			Assert.Equal(VehicleSpawner.MenuScreen.Pause, VehicleSpawner.ResolvePauseMenuToggle(VehicleSpawner.MenuScreen.None));
			Assert.Equal(VehicleSpawner.MenuScreen.None, VehicleSpawner.ResolvePauseMenuToggle(VehicleSpawner.MenuScreen.Pause));
			Assert.Equal(VehicleSpawner.MenuScreen.Pause, VehicleSpawner.ResolvePauseMenuToggle(VehicleSpawner.MenuScreen.VehicleSelection));
		}

		[Fact]
		public void ResolveVehicleSelectionCloseTarget_ReturnsPauseOnlyWhenOpenedFromPause()
		{
			Assert.Equal(VehicleSpawner.MenuScreen.Pause, VehicleSpawner.ResolveVehicleSelectionCloseTarget(openedFromPauseMenu: true));
			Assert.Equal(VehicleSpawner.MenuScreen.None, VehicleSpawner.ResolveVehicleSelectionCloseTarget(openedFromPauseMenu: false));
		}

		[Fact]
		public void IsVehicleMenuConfirmRequested_AcceptsControllerA()
		{
			bool requested = VehicleSpawner.IsVehicleMenuConfirmRequested(
				keyboardConfirmPressed: false,
				controllerConfirmPressed: true);

			Assert.True(requested);
		}

		[Fact]
		public void IsVehicleMenuCancelRequested_AcceptsControllerB()
		{
			bool requested = VehicleSpawner.IsVehicleMenuCancelRequested(
				keyboardCancelPressed: false,
				controllerCancelPressed: true);

			Assert.True(requested);
		}

		[Fact]
		public void IsVehicleMenuMoveDownRequested_AcceptsControllerDPadDown()
		{
			bool requested = VehicleSpawner.IsVehicleMenuMoveDownRequested(
				keyboardDownPressed: false,
				controllerDownPressed: true);

			Assert.True(requested);
		}

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
