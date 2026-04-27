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
		public static CodeFirstSceneSetup CreateMainSceneSetup(Game? game)
		{
			var groundModel = LoadAsset<Model>(game, "Ground");
			var skyboxTexture = LoadAsset<Texture>(game, "Skybox texture");
			var skybox = LoadAsset<Skybox>(game, "Skybox");

			var scene = new Scene();

			var ground = new Entity("Ground");
			ground.Add(new ModelComponent { Model = groundModel });
			ground.Add(new VehicleSpawner
			{
				VehicleFolderPath = "Resources/BeamNG Vehicles/sunburst2",
				ConfigFileName = "rally_pro_gravel.pc",
				SpawnPosition = new Vector3(0f, 2f, 0f),
			});
			scene.Entities.Add(ground);

			var camera = new Entity("Camera");
			camera.Transform.Position = new Vector3(2.6f, 0.6f, -1f);
			camera.Transform.Rotation = new Quaternion(0f, 0.82903755f, 0f, 0.5591929f);
			var cameraComponent = new CameraComponent();
			camera.Add(cameraComponent);
			camera.Add(new BasicCameraController
			{
				Gamepad = false,
				KeyboardMovementSpeed = new Vector3(5f),
				TouchMovementSpeed = new Vector3(0.7f, 0.7f, 0.3f),
				SpeedFactor = 5f,
				KeyboardRotationSpeed = new Vector2(3f),
				MouseRotationSpeed = new Vector2(1f, 1f),
				TouchRotationSpeed = new Vector2(1f, 0.7f),
			});
			scene.Entities.Add(camera);

			var directionalLight = new Entity("Directional light");
			directionalLight.Transform.Position = new Vector3(0f, 2f, 0f);
			directionalLight.Transform.Rotation = new Quaternion(1.131334e-08f, -0.9659258f, -0.25881904f, -4.222196e-08f);
			directionalLight.Add(new LightComponent
			{
				Type = new LightDirectional(),
				Intensity = 20f,
			});
			scene.Entities.Add(directionalLight);

			var skyboxEntity = new Entity("Skybox");
			skyboxEntity.Transform.Position = new Vector3(0f, 2f, -2f);

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

			var sceneSetup = CodeFirstSceneFactory.CreateMainSceneSetup(this);
			sceneSystem.GraphicsCompositor = sceneSetup.GraphicsCompositor;
			sceneSystem.SceneInstance.RootScene = sceneSetup.Scene;
		}
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
