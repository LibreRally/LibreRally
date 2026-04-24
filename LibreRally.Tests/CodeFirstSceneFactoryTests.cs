using System.Linq;
using LibreRally;
using Stride.Engine;
using Stride.Rendering;
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

			var ground = Assert.Single(entities.Where(entity => entity.Name == "Ground"));
			Assert.NotNull(ground.Get<ModelComponent>());
			Assert.NotNull(ground.Get<VehicleSpawner>());

			var camera = Assert.Single(entities.Where(entity => entity.Name == "Camera"));
			var cameraComponent = Assert.IsType<CameraComponent>(camera.Get<CameraComponent>());
			Assert.Same(sceneSetup.MainCamera, cameraComponent);
			Assert.NotNull(camera.Get<BasicCameraController>());
			Assert.False(cameraComponent.Slot.IsEmpty);

			var directionalLight = Assert.Single(entities.Where(entity => entity.Name == "Directional light"));
			var lightComponent = directionalLight.Get<LightComponent>();
			Assert.NotNull(lightComponent);
			Assert.IsType<LightDirectional>(lightComponent!.Type);

			var skyboxEntity = Assert.Single(entities.Where(entity => entity.Name == "Skybox"));
			Assert.NotNull(skyboxEntity.Get<BackgroundComponent>());
			var skyboxLightComponent = skyboxEntity.Get<LightComponent>();
			Assert.NotNull(skyboxLightComponent);
			Assert.IsType<LightSkybox>(skyboxLightComponent!.Type);
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
	}
}
