using System;
using System.Threading.Tasks;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Rendering;
using Stride.Rendering.Lights;
using Stride.Rendering.Skyboxes;

namespace LibreRally;

public static class CodeFirstSceneFactory
{
    public static Scene CreateMainScene() => CreateMainScene(null);

    public static Scene CreateMainScene(Game? game)
    {
        var groundModel = LoadAsset<Model>(game, "Ground");
        var skyboxTexture = LoadAsset<Texture>(game, "Skybox texture");
        var skybox = LoadAsset<Skybox>(game, "Skybox");

        var scene = new Scene();

        var ground = new Entity("Ground");
        ground.Add(new ModelComponent { Model = groundModel });
        ground.Add(new VehicleSpawner
        {
            VehicleFolderPath = "Resources/BeamNG Vehicles/basic_car",
            ConfigFileName = "basic_car.pc",
            SpawnPosition = new Vector3(0f, 2f, 0f),
        });
        scene.Entities.Add(ground);

        var camera = new Entity("Camera");
        camera.Transform.Position = new Vector3(2.6f, 0.6f, -1f);
        camera.Transform.Rotation = new Quaternion(0f, 0.82903755f, 0f, 0.5591929f);
        camera.Add(new CameraComponent());
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

        return scene;
    }

    private static T? LoadAsset<T>(Game? game, string assetName)
        where T : class
    {
        return game?.Content.Load<T>(assetName);
    }
}

public sealed class LibreRallyGame : Game
{
    protected override async Task LoadContent()
    {
        await base.LoadContent();

        var sceneSystem = Services.GetService<SceneSystem>()
            ?? throw new InvalidOperationException("Stride SceneSystem service is unavailable during startup.");
        if (sceneSystem.SceneInstance == null)
        {
            throw new InvalidOperationException("Stride SceneInstance is unavailable during startup.");
        }

        sceneSystem.SceneInstance.RootScene = CodeFirstSceneFactory.CreateMainScene(this);
    }
}
