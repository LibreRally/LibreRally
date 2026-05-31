using System;
using System.Linq;
using System.Threading.Tasks;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Particles.Rendering;
using Stride.Rendering;
using Stride.Rendering.Compositing;
using Stride.Rendering.Lights;
using Stride.Rendering.Skyboxes;

namespace LibreRally
{
	/// <summary>
	/// Factory for creating the main game scene and its associated setup using code-first approach.
	/// </summary>
	public static class CodeFirstSceneFactory
	{
		private const float CesiumCameraNearClipPlane = 0.5f;
		private const float CesiumCameraFarClipPlane = 20_000f;

		/// <summary>
		/// Creates the startup scene selected for the current runtime session.
		/// </summary>
		/// <param name="game">The game instance to use for asset loading.</param>
		/// <returns>A <see cref="CodeFirstSceneSetup"/> representing the startup scene.</returns>
		public static CodeFirstSceneSetup CreateStartupSceneSetup(Game? game)
		{
			return CreateStartupSceneSetup(game, ResolveStartupSceneSelection(Environment.GetEnvironmentVariable));
		}

		internal static CodeFirstSceneSetup CreateStartupSceneSetup(Game? game, StartupSceneSelection selection) =>
			CreateDrivingSceneSetup(game, selection);

		/// <summary>
		/// Creates the main scene with default settings.
		/// </summary>
		/// <returns>A new <see cref="Scene"/> instance.</returns>
		public static Scene CreateMainScene() => CreateMainSceneSetup().Scene;

		/// <summary>
		/// Creates the main scene with an optional game instance for asset loading.
		/// </summary>
		/// <param name="game">The game instance to use for loading assets.</param>
		/// <returns>A new <see cref="Scene"/> instance.</returns>
		public static Scene CreateMainScene(Game? game) => CreateMainSceneSetup(game).Scene;

		/// <summary>
		/// Creates the main scene setup with default settings.
		/// </summary>
		/// <returns>A <see cref="CodeFirstSceneSetup"/> containing the scene, camera, and compositor.</returns>
		public static CodeFirstSceneSetup CreateMainSceneSetup() => CreateMainSceneSetup(null);

		/// <summary>
		/// Creates the main scene setup with an optional game instance for asset loading.
		/// </summary>
		/// <param name="game">The game instance to use for loading assets.</param>
		/// <returns>A <see cref="CodeFirstSceneSetup"/> containing the scene, camera, and compositor.</returns>
		public static CodeFirstSceneSetup CreateMainSceneSetup(Game? game) =>
			CreateDrivingSceneSetup(
				game,
				new StartupSceneSelection(
					StartupSceneMode.Default,
					CesiumStageBootstrapConfiguration.CreateSampleStage()));

		/// <summary>
		/// Creates a code-first Cesium sample scene that streams a public 3D Tiles dataset.
		/// </summary>
		/// <param name="game">The active game instance used for asset loading.</param>
		/// <returns>A configured Cesium sample scene setup.</returns>
		public static CodeFirstSceneSetup CreateCesiumSampleSceneSetup(Game? game) =>
			CreateCesiumStageSceneSetup(game, CesiumStageBootstrapConfiguration.CreateSampleStage());

		/// <summary>
		/// Creates a code-first Cesium scene using the supplied stage configuration.
		/// </summary>
		/// <param name="game">The active game instance used for asset loading.</param>
		/// <param name="stage">Resolved stage bootstrap configuration.</param>
		/// <returns>A configured Cesium scene setup.</returns>
		internal static CodeFirstSceneSetup CreateCesiumStageSceneSetup(Game? game, CesiumStageBootstrapConfiguration stage)
		{
			return CreateDrivingSceneSetup(game, new StartupSceneSelection(StartupSceneMode.Cesium, stage));
		}

		private static CodeFirstSceneSetup CreateDrivingSceneSetup(Game? game, StartupSceneSelection selection)
		{
			var scene = new Scene();
			var cameraComponent = AddCamera(
				scene,
				selection.SceneMode == StartupSceneMode.Cesium
					? selection.CesiumStage.Camera
					: CreateDefaultDrivingCameraConfiguration());
			if (selection.SceneMode == StartupSceneMode.Cesium)
			{
				ConfigureCesiumCamera(cameraComponent);
				AddDefaultLighting(scene, game, 18f);
				var cesiumEntity = CreateCesiumStageEntity(selection.CesiumStage);
				scene.Entities.Add(cesiumEntity);
				scene.Entities.Add(CreateCesiumSpawnAnchorEntity(selection.CesiumStage, cesiumEntity));
			}
			else
			{
				AddDefaultLighting(scene, game, 20f);
			}

			scene.Entities.Add(CreateGameplayRootEntity(game, selection));
			return CreateSceneSetup(scene, cameraComponent);
		}

		private static Entity CreateGameplayRootEntity(Game? game, StartupSceneSelection selection)
		{
			var isCesiumStage = selection.SceneMode == StartupSceneMode.Cesium;
			var entity = new Entity(isCesiumStage ? "GameplayRoot" : "Ground");
			if (!isCesiumStage)
			{
				var groundModel = LoadAsset<Model>(game, "Ground");
				entity.Add(new ModelComponent { Model = groundModel });
			}

			entity.Add(new VehicleSpawner
			{
				VehicleFolderPath = "Resources/BeamNG Vehicles/sunburst2",
				ConfigFileName = "rally_pro_gravel.pc",
				SpawnPosition = isCesiumStage ? Vector3.Zero : new Vector3(0f, 2f, 0f),
				SpawnAnchorEntityName = isCesiumStage ? selection.CesiumStage.SpawnAnchorEntityName : null,
				StartupSelection = selection,
			});
			return entity;
		}

		private static Entity CreateCesiumStageEntity(CesiumStageBootstrapConfiguration stage)
		{
			var cesiumEntity = new Entity(stage.StageEntityName);
			cesiumEntity.Add(new CesiumGeoreferenceComponent
			{
				OriginLongitudeDegrees = stage.OriginLongitudeDegrees,
				OriginLatitudeDegrees = stage.OriginLatitudeDegrees,
				OriginHeightMeters = stage.OriginHeightMeters,
			});
			cesiumEntity.Add(new CesiumTilesetScript
			{
				DataSource = stage.DataSource,
				Url = stage.Url,
				LocalFilePath = stage.LocalFilePath,
				IonAssetId = stage.IonAssetId,
				IonAccessToken = stage.IonAccessToken,
				OriginLongitudeDegrees = stage.OriginLongitudeDegrees,
				OriginLatitudeDegrees = stage.OriginLatitudeDegrees,
				OriginHeightMeters = stage.OriginHeightMeters,
				MaximumScreenSpaceError = stage.MaximumScreenSpaceError,
				MaximumSimultaneousTileLoads = stage.MaximumSimultaneousTileLoads,
				MaximumCachedBytes = stage.MaximumCachedBytes,
				AutoFrameOnInitialTile = stage.AutoFrameOnInitialTile,
			});
			return cesiumEntity;
		}

		private static Entity CreateCesiumSpawnAnchorEntity(CesiumStageBootstrapConfiguration stage, Entity cesiumEntity)
		{
			var anchorEntity = new Entity(stage.SpawnAnchorEntityName);
			anchorEntity.Add(new CesiumGlobeAnchorComponent
			{
				LongitudeDegrees = stage.SpawnLongitudeDegrees,
				LatitudeDegrees = stage.SpawnLatitudeDegrees,
				HeightMeters = stage.SpawnHeightMeters,
				LocalOffset = stage.SpawnEastUpNorthOffset,
				AlignToEastUpNorth = false,
			});
			anchorEntity.Add(new CesiumGlobeAnchorScript
			{
				GeoreferenceEntity = cesiumEntity,
			});
			return anchorEntity;
		}

		private static void ConfigureCesiumCamera(CameraComponent cameraComponent)
		{
			cameraComponent.NearClipPlane = CesiumCameraNearClipPlane;
			cameraComponent.FarClipPlane = CesiumCameraFarClipPlane;
		}

		internal static StartupSceneSelection ResolveStartupSceneSelection(Func<string, string?> getEnvironmentVariable) =>
			StartupSceneSelection.FromEnvironment(getEnvironmentVariable);

		private static void EnsureParticleRenderFeature(GraphicsCompositor graphicsCompositor)
		{
			if (graphicsCompositor.RenderFeatures.OfType<ParticleEmitterRenderFeature>().Any())
			{
				return;
			}

			var opaqueStage = graphicsCompositor.RenderStages.FirstOrDefault(stage => stage.Name == "Opaque");
			var transparentStage = graphicsCompositor.RenderStages.FirstOrDefault(stage => stage.Name == "Transparent");
			if (opaqueStage == null || transparentStage == null)
			{
				return;
			}

			var particleFeature = new ParticleEmitterRenderFeature();
			particleFeature.RenderStageSelectors.Add(new ParticleEmitterTransparentRenderStageSelector
			{
				OpaqueRenderStage = opaqueStage,
				TransparentRenderStage = transparentStage,
			});
			graphicsCompositor.RenderFeatures.Add(particleFeature);
		}

		private static T? LoadAsset<T>(Game? game, string assetName)
			where T : class
		{
			return game?.Content.Load<T>(assetName);
		}

		private static SceneCameraBootstrapConfiguration CreateDefaultDrivingCameraConfiguration() =>
			new(
				Position: new Vector3(2.6f, 0.6f, -1f),
				Rotation: new Quaternion(0f, 0.82903755f, 0f, 0.5591929f),
				LookAtTarget: null,
				KeyboardMovementSpeed: new Vector3(5f),
				TouchMovementSpeed: new Vector3(0.7f, 0.7f, 0.3f),
				SpeedFactor: 5f,
				KeyboardRotationSpeed: new Vector2(3f),
				MouseRotationSpeed: new Vector2(1f, 1f),
				TouchRotationSpeed: new Vector2(1f, 0.7f));

		private static CameraComponent AddCamera(Scene scene, SceneCameraBootstrapConfiguration configuration)
		{
			var camera = new Entity("Camera");
			camera.Transform.Position = configuration.Position;
			camera.Transform.Rotation = configuration.LookAtTarget.HasValue
				? CreateCameraLookAtRotation(configuration.Position, configuration.LookAtTarget.Value)
				: configuration.Rotation;
			var cameraComponent = new CameraComponent();
			camera.Add(cameraComponent);
			camera.Add(new BasicCameraController
			{
				Gamepad = false,
				KeyboardMovementSpeed = configuration.KeyboardMovementSpeed,
				TouchMovementSpeed = configuration.TouchMovementSpeed,
				SpeedFactor = configuration.SpeedFactor,
				KeyboardRotationSpeed = configuration.KeyboardRotationSpeed,
				MouseRotationSpeed = configuration.MouseRotationSpeed,
				TouchRotationSpeed = configuration.TouchRotationSpeed,
			});
			scene.Entities.Add(camera);
			return cameraComponent;
		}

		private static Quaternion CreateCameraLookAtRotation(Vector3 position, Vector3 target)
		{
			var forward = target - position;
			return forward.LengthSquared() <= 1e-6f
				? Quaternion.Identity
				: CesiumRotationUtilities.CreateCameraRotationFromUpAndForward(Vector3.UnitY, forward);
		}

		private static void AddDefaultLighting(Scene scene, Game? game, float directionalLightIntensity)
		{
			var directionalLight = new Entity("Directional light");
			directionalLight.Transform.Position = new Vector3(0f, 2f, 0f);
			directionalLight.Transform.Rotation = new Quaternion(1.131334e-08f, -0.9659258f, -0.25881904f, -4.222196e-08f);
			directionalLight.Add(new LightComponent
			{
				Type = new LightDirectional(),
				Intensity = directionalLightIntensity,
			});
			scene.Entities.Add(directionalLight);

			var skyboxTexture = LoadAsset<Texture>(game, "Skybox texture");
			var skybox = LoadAsset<Skybox>(game, "Skybox");
			var skyboxEntity = new Entity("Skybox");
			var background = new BackgroundComponent();
			if (skyboxTexture != null)
			{
				background.Texture = skyboxTexture;
			}

			skyboxEntity.Add(background);
			var skyboxLight = new LightSkybox();
			if (skybox != null)
			{
				skyboxLight.Skybox = skybox;
			}

			skyboxEntity.Add(new LightComponent { Type = skyboxLight });
			scene.Entities.Add(skyboxEntity);
		}

		private static CodeFirstSceneSetup CreateSceneSetup(Scene scene, CameraComponent cameraComponent)
		{
			var graphicsCompositor = GraphicsCompositorHelper.CreateDefault(
				enablePostEffects: false,
				camera: cameraComponent,
				graphicsProfile: GraphicsProfile.Level_11_0);
			EnsureParticleRenderFeature(graphicsCompositor);

			if (graphicsCompositor.Cameras.Count > 0)
			{
				graphicsCompositor.Cameras[0].Name = "Main";
				cameraComponent.Slot = graphicsCompositor.Cameras[0].ToSlotId();
			}

			return new CodeFirstSceneSetup(scene, cameraComponent, graphicsCompositor);
		}

		internal static StartupSceneMode ResolveStartupSceneMode(string? value)
		{
			if (string.Equals(value, "cesium", StringComparison.OrdinalIgnoreCase))
			{
				return StartupSceneMode.Cesium;
			}

			return StartupSceneMode.Default;
		}
	}

	/// <summary>
	/// Main game class for LibreRally, responsible for initializing the scene and content.
	/// </summary>
	public sealed class LibreRallyGame : Game
	{
		/// <summary>
	/// Initializes a new instance of the <see cref="LibreRallyGame"/> class with a 60 Hz fixed timestep.
	/// </summary>
	public LibreRallyGame()
		{
			IsFixedTimeStep = true;
			TargetElapsedTime = TimeSpan.FromSeconds(1.0 / 60.0);
			IsDrawDesynchronized = false;
		}

		/// <summary>
		/// Loads the initial scene content and wires up the startup game state.
		/// </summary>
		/// <exception cref="InvalidOperationException">Thrown when required Stride scene services are unavailable during startup.</exception>
		protected override async Task LoadContent()
		{
			await base.LoadContent();

			var sceneSystem = Services.GetService<SceneSystem>()
			                  ?? throw new InvalidOperationException("Stride SceneSystem service is unavailable during startup.");
			
			// If SceneInstance is not yet initialized, create it now.
			// This can occur in Release mode or in certain initialization orders.
			if (sceneSystem.SceneInstance == null)
			{
				sceneSystem.SceneInstance = new SceneInstance(Services);
			}

			var sceneSetup = CodeFirstSceneFactory.CreateStartupSceneSetup(this);
			sceneSystem.GraphicsCompositor = sceneSetup.GraphicsCompositor;
			sceneSystem.SceneInstance.RootScene = sceneSetup.Scene;
		}
	}

	internal enum StartupSceneMode
	{
		Default,
		Cesium,
	}

	/// <summary>
	/// Holds the results of a code-first <paramref name="Scene" /> setup.
	/// </summary>
	/// <param name="Scene">The created scene.</param>
	/// <param name="MainCamera">The main camera component.</param>
	/// <param name="GraphicsCompositor">The graphics compositor used by the <paramref name="Scene" />.</param>
	public sealed record CodeFirstSceneSetup(
		Scene Scene,
		CameraComponent MainCamera,
		GraphicsCompositor GraphicsCompositor);
}
