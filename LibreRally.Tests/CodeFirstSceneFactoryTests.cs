using System.Linq;
using LibreRally;
using Stride.Engine;
using Stride.Rendering;
using Stride.Rendering.Lights;

namespace LibreRally.Tests;

public class CodeFirstSceneFactoryTests
{
    [Fact]
    public void CreateMainScene_CreatesExpectedRootEntities()
    {
        var scene = CodeFirstSceneFactory.CreateMainScene();
        var entities = scene.Entities.ToList();

        var ground = Assert.Single(entities.Where(entity => entity.Name == "Ground"));
        Assert.NotNull(ground.Get<ModelComponent>());
        Assert.NotNull(ground.Get<VehicleSpawner>());

        var camera = Assert.Single(entities.Where(entity => entity.Name == "Camera"));
        Assert.NotNull(camera.Get<CameraComponent>());
        Assert.NotNull(camera.Get<BasicCameraController>());

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
}
