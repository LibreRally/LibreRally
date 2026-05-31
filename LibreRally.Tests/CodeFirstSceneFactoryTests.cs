using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Rendering.Lights;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the code first scene factory behavior.
	/// </summary>
	public class CodeFirstSceneFactoryTests
	{
		/// <summary>
		/// Verifies that create main scene creates expected root entities.
		/// </summary>
		[Fact]
		public void CreateMainScene_CreatesExpectedRootEntities()
		{
			var sceneSetup = CodeFirstSceneFactory.CreateMainSceneSetup();
			var scene = sceneSetup.Scene;
			var entities = scene.Entities.ToList();

			var ground = Assert.Single(entities, entity => entity.Name == "Ground");
			Assert.NotNull(ground.Get<ModelComponent>());
			Assert.NotNull(ground.Get<VehicleSpawner>());

			var camera = Assert.Single(entities, entity => entity.Name == "Camera");
			var cameraComponent = Assert.IsType<CameraComponent>(camera.Get<CameraComponent>());
			Assert.Same(sceneSetup.MainCamera, cameraComponent);
			Assert.NotNull(camera.Get<BasicCameraController>());
			Assert.False(cameraComponent.Slot.IsEmpty);

			var directionalLight = Assert.Single(entities, entity => entity.Name == "Directional light");
			var lightComponent = directionalLight.Get<LightComponent>();
			Assert.NotNull(lightComponent);
			Assert.IsType<LightDirectional>(lightComponent.Type);

			var skyboxEntity = Assert.Single(entities, entity => entity.Name == "Skybox");
			Assert.NotNull(skyboxEntity.Get<BackgroundComponent>());
			var skyboxLightComponent = skyboxEntity.Get<LightComponent>();
			Assert.NotNull(skyboxLightComponent);
			Assert.IsType<LightSkybox>(skyboxLightComponent.Type);
		}

		/// <summary>
		/// Verifies that create main scene setup binds main camera to graphics compositor slot.
		/// </summary>
		[Fact]
		public void CreateMainSceneSetup_BindsMainCameraToGraphicsCompositorSlot()
		{
			var sceneSetup = CodeFirstSceneFactory.CreateMainSceneSetup();

			var cameraSlot = Assert.Single(sceneSetup.GraphicsCompositor.Cameras);
			Assert.Equal("Main", cameraSlot.Name);
			Assert.Equal(cameraSlot.ToSlotId(), sceneSetup.MainCamera.Slot);
		}

		/// <summary>
		/// Verifies that the Cesium sample scene exposes the expected root entities.
		/// </summary>
		[Fact]
		public void CreateCesiumSampleSceneSetup_CreatesExpectedRootEntities()
		{
			var sceneSetup = CodeFirstSceneFactory.CreateCesiumSampleSceneSetup(null);
			var entities = sceneSetup.Scene.Entities.ToList();

			var camera = Assert.Single(entities, entity => entity.Name == "Camera");
			var cameraComponent = Assert.IsType<CameraComponent>(camera.Get<CameraComponent>());
			Assert.NotNull(camera.Get<BasicCameraController>());
			Assert.Equal(0.5f, cameraComponent.NearClipPlane);
			Assert.Equal(20000f, cameraComponent.FarClipPlane);

			var directionalLight = Assert.Single(entities, entity => entity.Name == "Directional light");
			Assert.NotNull(directionalLight.Get<LightComponent>());

			var skyboxEntity = Assert.Single(entities, entity => entity.Name == "Skybox");
			Assert.NotNull(skyboxEntity.Get<BackgroundComponent>());

			var cesiumEntity = Assert.Single(entities, entity => entity.Name == "Cesium Sample Stage");
			var georeference = Assert.IsType<CesiumGeoreferenceComponent>(cesiumEntity.Get<CesiumGeoreferenceComponent>());
			Assert.Equal(-75.612094d, georeference.OriginLongitudeDegrees);
			Assert.Equal(40.042531d, georeference.OriginLatitudeDegrees);
			Assert.Equal(20d, georeference.OriginHeightMeters);
			var cesiumScript = Assert.IsType<CesiumTilesetScript>(cesiumEntity.Get<CesiumTilesetScript>());
			Assert.Equal(CesiumTilesetDataSource.Url, cesiumScript.DataSource);
			Assert.False(string.IsNullOrWhiteSpace(cesiumScript.Url));
			Assert.Equal(268435456L, cesiumScript.MaximumCachedBytes);
			Assert.True(cesiumScript.AutoFrameOnInitialTile);
			Assert.True(cesiumScript.EnablePhysicsProxyColliders);
			Assert.True(cesiumScript.UseAccuratePhysicsMeshes);
			Assert.Equal(250f, cesiumScript.PhysicsProxyActivationDistance);
			Assert.Equal(8, cesiumScript.MaximumPhysicsProxyBoxesPerTile);

			var spawnAnchor = Assert.Single(entities, entity => entity.Name == "sample_spawn_anchor");
			var anchor = Assert.IsType<CesiumGlobeAnchorComponent>(spawnAnchor.Get<CesiumGlobeAnchorComponent>());
			Assert.Equal(-75.612094d, anchor.LongitudeDegrees);
			Assert.Equal(40.042531d, anchor.LatitudeDegrees);
			Assert.Equal(20d, anchor.HeightMeters);
			Assert.Equal(new Vector3(0f, 2.5f, 0f), anchor.LocalOffset);
			Assert.NotNull(spawnAnchor.Get<CesiumGlobeAnchorScript>());

			var gameplayRoot = Assert.Single(entities, entity => entity.Name == "GameplayRoot");
			var vehicleSpawner = Assert.IsType<VehicleSpawner>(gameplayRoot.Get<VehicleSpawner>());
			Assert.Equal(Vector3.Zero, vehicleSpawner.SpawnPosition);
			Assert.Equal("sample_spawn_anchor", vehicleSpawner.SpawnAnchorEntityName);
		}

		/// <summary>
		/// Verifies that Cesium stage configuration is applied to the runtime bootstrap entity.
		/// </summary>
		[Fact]
		public void CreateCesiumStageSceneSetup_AppliesStageConfiguration()
		{
			var stage = CesiumStageBootstrapConfiguration.ResolvePreset("ion") with
			{
				IonAssetId = 12345,
				IonAccessToken = "test-token",
				OriginLongitudeDegrees = 12.34,
				OriginLatitudeDegrees = 56.78,
				OriginHeightMeters = 125d,
				SpawnLongitudeDegrees = 12.341,
				SpawnLatitudeDegrees = 56.781,
				SpawnHeightMeters = 132d,
				SpawnEastUpNorthOffset = new Vector3(4f, 2f, -3f),
				MaximumScreenSpaceError = 9.5f,
				MaximumSimultaneousTileLoads = 4,
				AutoFrameOnInitialTile = false,
			};

			var sceneSetup = CodeFirstSceneFactory.CreateCesiumStageSceneSetup(null, stage);
			var entities = sceneSetup.Scene.Entities.ToList();

			var cesiumEntity = Assert.Single(entities, entity => entity.Name == "Cesium Ion Stage");
			var georeference = Assert.IsType<CesiumGeoreferenceComponent>(cesiumEntity.Get<CesiumGeoreferenceComponent>());
			Assert.Equal(12.34, georeference.OriginLongitudeDegrees);
			Assert.Equal(56.78, georeference.OriginLatitudeDegrees);
			Assert.Equal(125d, georeference.OriginHeightMeters);
			var cesiumScript = Assert.IsType<CesiumTilesetScript>(cesiumEntity.Get<CesiumTilesetScript>());
			Assert.Equal(CesiumTilesetDataSource.CesiumIon, cesiumScript.DataSource);
			Assert.Equal(12345, cesiumScript.IonAssetId);
			Assert.Equal("test-token", cesiumScript.IonAccessToken);
			Assert.Equal(string.Empty, cesiumScript.LocalFilePath);
			Assert.Equal(12.34, cesiumScript.OriginLongitudeDegrees);
			Assert.Equal(56.78, cesiumScript.OriginLatitudeDegrees);
			Assert.Equal(125d, cesiumScript.OriginHeightMeters);
			Assert.Equal(9.5f, cesiumScript.MaximumScreenSpaceError);
			Assert.Equal((uint)4, cesiumScript.MaximumSimultaneousTileLoads);
			Assert.Equal(268435456L, cesiumScript.MaximumCachedBytes);
			Assert.False(cesiumScript.AutoFrameOnInitialTile);
			Assert.Equal(0.5f, sceneSetup.MainCamera.NearClipPlane);
			Assert.Equal(20000f, sceneSetup.MainCamera.FarClipPlane);
			Assert.Equal(Vector3.Zero, stage.Camera.LookAtTarget);

			var spawnAnchor = Assert.Single(entities, entity => entity.Name == stage.SpawnAnchorEntityName);
			var anchor = Assert.IsType<CesiumGlobeAnchorComponent>(spawnAnchor.Get<CesiumGlobeAnchorComponent>());
			Assert.Equal(12.341, anchor.LongitudeDegrees);
			Assert.Equal(56.781, anchor.LatitudeDegrees);
			Assert.Equal(132d, anchor.HeightMeters);
			Assert.Equal(new Vector3(4f, 2f, -3f), anchor.LocalOffset);

			var gameplayRoot = Assert.Single(entities, entity => entity.Name == "GameplayRoot");
			var vehicleSpawner = Assert.IsType<VehicleSpawner>(gameplayRoot.Get<VehicleSpawner>());
			Assert.Equal(stage.SpawnAnchorEntityName, vehicleSpawner.SpawnAnchorEntityName);
		}

		/// <summary>
		/// Verifies that Cesium stage camera look-at uses the camera's visible forward axis.
		/// </summary>
		[Fact]
		public void CreateCesiumStageSceneSetup_OrientsCameraTowardLookAtTarget()
		{
			var baseStage = CesiumStageBootstrapConfiguration.ResolvePreset("ion");
			var stage = baseStage with
			{
				Camera = baseStage.Camera with
				{
					Position = new Vector3(18f, 12f, -24f),
					LookAtTarget = new Vector3(-6f, 9f, 14f),
					Rotation = Quaternion.Identity,
				},
			};

			var sceneSetup = CodeFirstSceneFactory.CreateCesiumStageSceneSetup(null, stage);
			var camera = Assert.Single(sceneSetup.Scene.Entities, entity => entity.Name == "Camera");
			var cameraRotation = Matrix.RotationQuaternion(camera.Transform.Rotation);
			var expectedForward = Vector3.Normalize(stage.Camera.LookAtTarget!.Value - stage.Camera.Position);
			var actualForward = Vector3.Normalize(cameraRotation.Forward);

			Assert.InRange(Vector3.Distance(actualForward, expectedForward), 0f, 0.001f);
		}

		/// <summary>
		/// Verifies that startup scene mode parsing recognizes the Cesium scene selector.
		/// </summary>
		[Theory]
		[InlineData("cesium", "Cesium")]
		[InlineData("CESIUM", "Cesium")]
		[InlineData("default", "Default")]
		[InlineData(null, "Default")]
		public void ResolveStartupSceneMode_ParsesExpectedModes(string? value, string expectedMode)
		{
			var mode = CodeFirstSceneFactory.ResolveStartupSceneMode(value);
			Assert.Equal(expectedMode, mode.ToString());
		}

		/// <summary>
		/// Verifies that startup scene selection resolves the default scene and Cesium sample stage.
		/// </summary>
		[Fact]
		public void ResolveStartupSceneSelection_UsesDefaultModeAndSampleStage()
		{
			var selection = CodeFirstSceneFactory.ResolveStartupSceneSelection(_ => null);

			Assert.Equal(StartupSceneMode.Default, selection.SceneMode);
			Assert.Equal("sample", selection.CesiumStage.StageKey);
			Assert.Equal(CesiumTilesetDataSource.Url, selection.CesiumStage.DataSource);
		}

		/// <summary>
		/// Verifies that the track selector exposes the curated Cesium ion stages with shared runtime overrides.
		/// </summary>
		[Fact]
		public void BuildTrackSelectionStages_UsesCuratedIonPresets()
		{
			var runtimeStage = CesiumStageBootstrapConfiguration.ResolvePreset("ion") with
			{
				IonAssetId = 2275207,
				IonAccessToken = "secret-token",
				OriginLongitudeDegrees = 144.9673305,
				OriginLatitudeDegrees = -37.8175087,
				OriginHeightMeters = 25d,
				SpawnLongitudeDegrees = 144.9673305,
				SpawnLatitudeDegrees = -37.8175087,
				SpawnHeightMeters = 25d,
				AutoFrameOnInitialTile = false,
			};

			var stages = CesiumStageBootstrapConfiguration.BuildTrackSelectionStages(runtimeStage);

			Assert.Collection(
				stages,
				stage =>
				{
					Assert.Equal("melbourne-photogrammetry", stage.StageKey);
					Assert.Equal(69380, stage.IonAssetId);
				},
				stage =>
				{
					Assert.Equal("google-photorealistic-3d-tiles", stage.StageKey);
					Assert.Equal(2275207, stage.IonAssetId);
				},
				stage =>
				{
					Assert.Equal("cesium-osm-buildings", stage.StageKey);
					Assert.Equal(96188, stage.IonAssetId);
				});
			Assert.All(stages, stage => Assert.Equal("secret-token", stage.IonAccessToken));
			Assert.All(stages, stage => Assert.Equal(144.9673305, stage.OriginLongitudeDegrees));
			Assert.All(stages, stage => Assert.Equal(-37.8175087, stage.OriginLatitudeDegrees));
			Assert.All(stages, stage => Assert.Equal(25d, stage.SpawnHeightMeters));
			Assert.All(stages, stage => Assert.False(stage.AutoFrameOnInitialTile));
		}

		/// <summary>
		/// Verifies that environment-driven Cesium stage overrides flow into startup selection.
		/// </summary>
		[Fact]
		public void ResolveStartupSceneSelection_AppliesCesiumEnvironmentOverrides()
		{
			var environment = new Dictionary<string, string?>
			{
				[LibreRallyEnvironmentVariables.SceneMode] = "cesium",
				[LibreRallyEnvironmentVariables.CesiumStage] = "ion",
				[LibreRallyEnvironmentVariables.CesiumIonAssetId] = "98765",
				[LibreRallyEnvironmentVariables.CesiumIonAccessToken] = "secret-token",
				[LibreRallyEnvironmentVariables.CesiumOriginLongitude] = "8.125",
				[LibreRallyEnvironmentVariables.CesiumOriginLatitude] = "46.804",
				[LibreRallyEnvironmentVariables.CesiumOriginHeight] = "1430.5",
				[LibreRallyEnvironmentVariables.CesiumMaximumScreenSpaceError] = "7.25",
				[LibreRallyEnvironmentVariables.CesiumMaximumTileLoads] = "6",
			};
			var selection = CodeFirstSceneFactory.ResolveStartupSceneSelection(key => environment.TryGetValue(key, out var value) ? value : null);

			Assert.Equal(StartupSceneMode.Cesium, selection.SceneMode);
			Assert.Equal("ion", selection.CesiumStage.StageKey);
			Assert.Equal(CesiumTilesetDataSource.CesiumIon, selection.CesiumStage.DataSource);
			Assert.Equal(98765, selection.CesiumStage.IonAssetId);
			Assert.Equal("secret-token", selection.CesiumStage.IonAccessToken);
			Assert.Equal(8.125, selection.CesiumStage.OriginLongitudeDegrees);
			Assert.Equal(46.804, selection.CesiumStage.OriginLatitudeDegrees);
			Assert.Equal(1430.5, selection.CesiumStage.OriginHeightMeters);
			Assert.Equal(7.25f, selection.CesiumStage.MaximumScreenSpaceError);
			Assert.Equal((uint)6, selection.CesiumStage.MaximumSimultaneousTileLoads);
			Assert.False(selection.CesiumStage.AutoFrameOnInitialTile);
		}

		/// <summary>
		/// Verifies that curated Cesium presets keep their built-in asset IDs even when generic ion overrides are present.
		/// </summary>
		[Fact]
		public void ResolveStartupSceneSelection_PreservesCuratedStageAssetId()
		{
			var environment = new Dictionary<string, string?>
			{
				[LibreRallyEnvironmentVariables.SceneMode] = "cesium",
				[LibreRallyEnvironmentVariables.CesiumStage] = "google-photorealistic",
				[LibreRallyEnvironmentVariables.CesiumIonAssetId] = "69380",
				[LibreRallyEnvironmentVariables.CesiumIonAccessToken] = "secret-token",
				[LibreRallyEnvironmentVariables.CesiumOriginLongitude] = "144.9673305",
				[LibreRallyEnvironmentVariables.CesiumOriginLatitude] = "-37.8175087",
			};
			var selection = CodeFirstSceneFactory.ResolveStartupSceneSelection(key => environment.TryGetValue(key, out var value) ? value : null);

			Assert.Equal(StartupSceneMode.Cesium, selection.SceneMode);
			Assert.Equal("google-photorealistic-3d-tiles", selection.CesiumStage.StageKey);
			Assert.Equal(2275207, selection.CesiumStage.IonAssetId);
			Assert.Equal("secret-token", selection.CesiumStage.IonAccessToken);
			Assert.Equal(144.9673305, selection.CesiumStage.OriginLongitudeDegrees);
			Assert.Equal(-37.8175087, selection.CesiumStage.OriginLatitudeDegrees);
		}

		/// <summary>
		/// Verifies that local-file Cesium stage overrides flow into startup selection.
		/// </summary>
		[Fact]
		public void ResolveStartupSceneSelection_AppliesLocalFileOverrides()
		{
			var environment = new Dictionary<string, string?>
			{
				[LibreRallyEnvironmentVariables.SceneMode] = "cesium",
				[LibreRallyEnvironmentVariables.CesiumStage] = "file",
				[LibreRallyEnvironmentVariables.CesiumLocalFile] = @"Resources\Cesium\tileset.json",
				[LibreRallyEnvironmentVariables.CesiumMaximumCachedBytes] = "1048576",
			};
			var selection = CodeFirstSceneFactory.ResolveStartupSceneSelection(key => environment.TryGetValue(key, out var value) ? value : null);

			Assert.Equal(StartupSceneMode.Cesium, selection.SceneMode);
			Assert.Equal("file", selection.CesiumStage.StageKey);
			Assert.Equal(CesiumTilesetDataSource.LocalFile, selection.CesiumStage.DataSource);
			Assert.Equal(@"Resources\Cesium\tileset.json", selection.CesiumStage.LocalFilePath);
			Assert.Equal(1048576L, selection.CesiumStage.MaximumCachedBytes);
			Assert.True(selection.CesiumStage.AutoFrameOnInitialTile);
		}

		/// <summary>
		/// Verifies that invalid Cesium latitude values fail fast with a swap hint.
		/// </summary>
		[Fact]
		public void ResolveStartupSceneSelection_InvalidLatitudeThrowsHelpfulError()
		{
			var environment = new Dictionary<string, string?>
			{
				[LibreRallyEnvironmentVariables.SceneMode] = "cesium",
				[LibreRallyEnvironmentVariables.CesiumStage] = "ion",
				[LibreRallyEnvironmentVariables.CesiumOriginLongitude] = "-37.8175087",
				[LibreRallyEnvironmentVariables.CesiumOriginLatitude] = "144.9673305",
			};

			var exception = Assert.Throws<InvalidOperationException>(
				() => CodeFirstSceneFactory.ResolveStartupSceneSelection(key => environment.TryGetValue(key, out var value) ? value : null));
			Assert.Contains(LibreRallyEnvironmentVariables.CesiumOriginLatitude, exception.Message);
			Assert.Contains("swapped", exception.Message);
		}
	}
}
