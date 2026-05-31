using Stride.BepuPhysics;
using Stride.BepuPhysics.Definitions.Colliders;
using Stride.Core.Mathematics;
using Stride.Engine;

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
		/// Verifies that track selection always returns to the pause menu when closed.
		/// </summary>
		[Fact]
		public void ResolveTrackSelectionCloseTarget_ReturnsPauseMenu()
		{
			Assert.Equal(VehicleSpawner.MenuScreen.Pause, VehicleSpawner.ResolveTrackSelectionCloseTarget());
		}

		/// <summary>
		/// Verifies that Cesium spawns wait for ground support when a spawn anchor is configured.
		/// </summary>
		[Fact]
		public void ShouldWaitForSpawnSupport_IsTrueForCesiumAnchors()
		{
			Assert.True(VehicleSpawner.ShouldWaitForSpawnSupport(StartupSceneMode.Cesium, "spawn_anchor"));
			Assert.False(VehicleSpawner.ShouldWaitForSpawnSupport(StartupSceneMode.Default, "spawn_anchor"));
			Assert.False(VehicleSpawner.ShouldWaitForSpawnSupport(StartupSceneMode.Cesium, null));
		}

		/// <summary>
		/// Verifies that Cesium track identity compares the underlying stage rather than only the scene mode.
		/// </summary>
		[Fact]
		public void AreEquivalentTrackSelections_MatchesCuratedIonStagesByAssetId()
		{
			var current = new StartupSceneSelection(
				StartupSceneMode.Cesium,
				CesiumStageBootstrapConfiguration.ResolvePreset("ion") with
				{
					IonAssetId = 2275207,
				});
			var curatedGoogle = new StartupSceneSelection(
				StartupSceneMode.Cesium,
				CesiumStageBootstrapConfiguration.ResolvePreset("google-photorealistic"));
			var curatedOsm = new StartupSceneSelection(
				StartupSceneMode.Cesium,
				CesiumStageBootstrapConfiguration.ResolvePreset("cesium-osm-buildings"));

			Assert.True(VehicleSpawner.AreEquivalentTrackSelections(current, curatedGoogle));
			Assert.False(VehicleSpawner.AreEquivalentTrackSelections(current, curatedOsm));
		}

		/// <summary>
		/// Verifies that the waiting status names the active track while spawn support is pending.
		/// </summary>
		[Fact]
		public void BuildSpawnWaitingStatus_IncludesTrackName()
		{
			var status = VehicleSpawner.BuildSpawnWaitingStatus("Cesium Ion");

			Assert.Contains("Cesium Ion", status);
			Assert.Contains("spawn", status, StringComparison.OrdinalIgnoreCase);
		}

		/// <summary>
		/// Verifies that spawn support only becomes ready for tagged driveable surfaces inside the accepted range.
		/// </summary>
		[Fact]
		public void IsSpawnSupportHitReady_RequiresTaggedSurfaceWithinRange()
		{
			var entity = new Entity("surface");
			var collider = new StaticComponent
			{
				Collider = new CompoundCollider
				{
					Colliders = { new BoxCollider { Size = new Vector3(1f) } },
				},
			};
			entity.Add(collider);
			entity.Add(new TrackSurfaceComponent());

			Assert.True(VehicleSpawner.IsSpawnSupportHitReady(collider, 4f, 12f));
			Assert.True(VehicleSpawner.IsSpawnSupportHitReady(collider, 40f, 48f));
			Assert.False(VehicleSpawner.IsSpawnSupportHitReady(collider, 14f, 12f));
			Assert.False(VehicleSpawner.IsSpawnSupportHitReady(
				new StaticComponent
				{
					Collider = new CompoundCollider
					{
						Colliders = { new BoxCollider { Size = new Vector3(1f) } },
					},
				},
				4f,
				12f));
		}

		/// <summary>
		/// Verifies that the fallback support pad activates after the configured wait threshold.
		/// </summary>
		[Fact]
		public void ShouldActivateSpawnFallbackSupport_ActivatesAtTimeout()
		{
			Assert.False(VehicleSpawner.ShouldActivateSpawnFallbackSupport(7.9f, 8f));
			Assert.True(VehicleSpawner.ShouldActivateSpawnFallbackSupport(8f, 8f));
		}

		/// <summary>
		/// Verifies that the fallback support center lands beneath the spawn point with the requested clearance.
		/// </summary>
		[Fact]
		public void BuildSpawnFallbackSupportCenter_PlacesPadBelowSpawn()
		{
			var center = VehicleSpawner.BuildSpawnFallbackSupportCenter(new Vector3(10f, 6f, -2f), 2.5f, 1f);

			Assert.Equal(new Vector3(10f, 3f, -2f), center);
		}

		/// <summary>
		/// Verifies that a resolved support contact is converted into a spawn point above the surface.
		/// </summary>
		[Fact]
		public void BuildSpawnPositionFromSupportPoint_PlacesSpawnAboveSurface()
		{
			var spawn = VehicleSpawner.BuildSpawnPositionFromSupportPoint(new Vector3(4f, 1f, -3f), 2.5f);

			Assert.Equal(new Vector3(4f, 3.5f, -3f), spawn);
		}

		/// <summary>
		/// Verifies that the spawn-loading camera sits above the anchor and points back toward it.
		/// </summary>
		[Fact]
		public void BuildSpawnLoadingCameraPose_LooksAtSpawnAnchor()
		{
			var spawnPosition = new Vector3(10f, 2.5f, -6f);
			var pose = VehicleSpawner.BuildSpawnLoadingCameraPose(spawnPosition);
			var forward = Vector3.Normalize(Matrix.RotationQuaternion(pose.Rotation).Forward);
			var expectedForward = Vector3.Normalize(spawnPosition - pose.Position);

			Assert.True(pose.Position.Y > spawnPosition.Y);
			Assert.InRange(Vector3.Distance(forward, expectedForward), 0f, 0.001f);
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
