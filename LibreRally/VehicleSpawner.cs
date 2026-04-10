using System;
using System.IO;
using System.Linq;
using LibreRally.Camera;
using LibreRally.HUD;
using LibreRally.Vehicle;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Definitions.Colliders;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Input;
using Stride.Rendering;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;

namespace LibreRally;

[ComponentCategory("LibreRally")]
public class VehicleSpawner : SyncScript
{
    public string VehicleFolderPath { get; set; } = @"Resources\BeamNG Vehicles\sunburst2";

    /// <summary>
    /// Name of the .pc config file to load (with or without .pc extension).
    /// E.g. "rally_pro_asphalt" or "rally_pro_asphalt.pc".
    /// Leave empty to auto-detect (prefers rally_pro_asphalt.pc if present).
    /// </summary>
    public string ConfigFileName { get; set; } = "rally_pro_asphalt.pc";

    public Vector3 SpawnPosition { get; set; } = new Vector3(0, 0.15f, 0);

    private string _status = "Loading...";
    private bool _showDebug = true;
    private VehicleTelemetryOverlay? _telemetryOverlay;

    public override void Start()
    {
        AddGroundPhysics();
        EnsureTelemetryOverlay();

        try
        {
            LoadVehicle();
        }
        catch (Exception ex)
        {
            Log.Error($"Failed to load vehicle: {ex}");
            _status = $"Load error: {ex.Message}";
            if (_telemetryOverlay != null)
            {
	            _telemetryOverlay.StatusText = _status;
            }
        }
    }

    private void LoadVehicle()
    {
        var requestedConfig = string.IsNullOrWhiteSpace(ConfigFileName) ? "<auto>" : ConfigFileName;
        var basePath = Path.IsPathRooted(VehicleFolderPath)
            ? VehicleFolderPath
            : Path.Combine(AppContext.BaseDirectory, VehicleFolderPath);
        Log.Info($"[VehicleSpawner] Load request: folder='{VehicleFolderPath}' resolved='{basePath}' config='{requestedConfig}'");

        var loader = new VehicleLoader((Game)Game);
        var vehicle = loader.Load(basePath, string.IsNullOrWhiteSpace(ConfigFileName) ? null : ConfigFileName);

        // CRITICAL: apply SpawnPosition to each physics entity directly.
        // They must be at scene root level (no offset parent) so BEPU writes correct world positions back.
        foreach (var child in vehicle.RootEntity.GetChildren().ToList())
            child.Transform.Position += SpawnPosition;

        // Keep root at origin (it holds the RallyCarComponent SyncScript)
        vehicle.RootEntity.Transform.Position = Vector3.Zero;
        SceneSystem.SceneInstance.RootScene.Entities.Add(vehicle.RootEntity);

        var activeConfig = vehicle.Diagnostics.ConfigPath != null
            ? Path.GetFileName(vehicle.Diagnostics.ConfigPath)
            : "<jbeam defaults>";
        _status = $"Loaded: {vehicle.Definition.VehicleName} cfg={activeConfig} mass={vehicle.Diagnostics.EstimatedMassKg:F0}kg";
        Log.Info($"[VehicleSpawner] {_status} folder='{vehicle.Diagnostics.VehicleFolderPath}'");

        AttachCamera(vehicle.ChassisEntity, vehicle.CarComponent);
        AttachHud(vehicle.CarComponent);
        AttachSpeedoGauge(vehicle.CarComponent);
        AttachTelemetryOverlay(vehicle.CarComponent);
    }

    private SpeedoGauge? _speedoGauge;

    private void AttachSpeedoGauge(RallyCarComponent car)
    {
        // Remove old instance if respawning
        if (_speedoGauge != null)
        {
            ((Game)Game).GameSystems.Remove(_speedoGauge);
            _speedoGauge.Dispose();
        }
        _speedoGauge = new SpeedoGauge(Services);
        ((Game)Game).GameSystems.Add(_speedoGauge);

        // The gauge reads from _speedoGauge properties we update in Update()
        // Store reference so Update() can push telemetry
        _car = car;
    }

    private RallyCarComponent? _car;

    private void AttachTelemetryOverlay(RallyCarComponent car)
    {
        EnsureTelemetryOverlay();
        if (_telemetryOverlay == null)
        {
	        return;
        }

        _telemetryOverlay.Car = car;
        _telemetryOverlay.StatusText = _status;
        _telemetryOverlay.OverlayVisible = _showDebug;
    }

    private void EnsureTelemetryOverlay()
    {
        if (_telemetryOverlay != null)
        {
	        return;
        }

        _telemetryOverlay = new VehicleTelemetryOverlay(Services)
        {
            Car = _car,
            StatusText = _status,
            OverlayVisible = _showDebug,
        };

        ((Game)Game).GameSystems.Add(_telemetryOverlay);
    }

    private void AttachHud(RallyCarComponent car)
    {
        // Find an existing HUD entity or create one
        var hudEntity = SceneSystem.SceneInstance.RootScene.Entities
            .FirstOrDefault(e => e.Name == "HUD");
        if (hudEntity == null)
        {
            hudEntity = new Entity("HUD");
            SceneSystem.SceneInstance.RootScene.Entities.Add(hudEntity);
        }

        // Remove any existing RallyHud component then re-add with the current car
        foreach (var h in hudEntity.GetAll<LibreRally.HUD.RallyHud>().ToList())
            hudEntity.Remove(h);
        hudEntity.Add(new LibreRally.HUD.RallyHud { Car = car });
    }

    private void AttachCamera(Entity chassis, RallyCarComponent car)
    {
        var cameraEntity = SceneSystem.SceneInstance.RootScene.Entities
            .FirstOrDefault(e => e.Get<CameraComponent>() != null);
        if (cameraEntity == null)
        {
	        return;
        }

        foreach (var s in cameraEntity.GetAll<SyncScript>().ToList())
            cameraEntity.Remove(s);

        cameraEntity.Add(new RallyCameraScript
        {
            Target = chassis,
            CarComponent = car,
        });
    }

    private void AddGroundPhysics()
    {
        Entity.Transform.Position = new Vector3(Entity.Transform.Position.X, -0.5f, Entity.Transform.Position.Z);
        Entity.Add(new StaticComponent
        {
            FrictionCoefficient = 1.5f,
            Collider = new CompoundCollider
            {
                Colliders = { new BoxCollider { Size = new Vector3(500f, 1f, 500f) } }
            }
        });

        // Add checkerboard visual so there's a visual reference for movement
        AddCheckerboardFloor();
    }

    /// <summary>
    /// Creates a large tessellated quad mesh with a procedural checkerboard texture
    /// so the player has a visual reference while driving.
    /// </summary>
    private void AddCheckerboardFloor()
    {
        try
        {
            var gd = ((Game)Game).GraphicsDevice;

            // ── Procedural 2×2 checkerboard texture ──────────────────────────
            // Two shades of grey; tiled heavily via UV scaling so tiles are ~8m each.
            byte dark  = 55;
            byte light = 115;
            var pixelBytes = new byte[]
            {
                dark,  dark,  dark,  255,   // top-left: dark
                light, light, light, 255,   // top-right: light
                light, light, light, 255,   // bottom-left: light
                dark,  dark,  dark,  255,   // bottom-right: dark
            };
            var image = Image.New2D(2, 2, 1, PixelFormat.R8G8B8A8_UNorm);
            var imageData = image.GetPixelBuffer(0, 0).GetPixels<byte>();
            for (var i = 0; i < pixelBytes.Length; i++) imageData[i] = pixelBytes[i];
            var checkerTex = Texture.New(gd, image);
            image.Dispose();

            // ── Large flat quad (500×500 m), Y = +0.5 so it sits on the top of the box ──
            // UV 0→125 means each 2×2 pixel tile = 4m. With nearest-neighbour wrap that
            // creates a clean 4m checkerboard across the whole ground plane.
            const float Half    = 250f;
            const float UvScale = 62.5f;   // 500m / 8m per tile
            const float SurfY   = 0.5f;    // top of the 1m-thick box

            var verts = new VertexPositionNormalTexture[]
            {
                new(new Vector3(-Half, SurfY, -Half), Vector3.UnitY, new Vector2(0,       0)),
                new(new Vector3( Half, SurfY, -Half), Vector3.UnitY, new Vector2(UvScale, 0)),
                new(new Vector3( Half, SurfY,  Half), Vector3.UnitY, new Vector2(UvScale, UvScale)),
                new(new Vector3(-Half, SurfY,  Half), Vector3.UnitY, new Vector2(0,       UvScale)),
            };
            var indices = new int[] { 0, 1, 2, 0, 2, 3 };

            var mesh = new Mesh
            {
                BoundingBox = new BoundingBox(new Vector3(-Half, SurfY - 0.01f, -Half),
                                              new Vector3( Half, SurfY + 0.01f,  Half)),
                Draw = new MeshDraw
                {
                    PrimitiveType = PrimitiveType.TriangleList,
                    VertexBuffers = new[]
                    {
                        new VertexBufferBinding(
                            Stride.Graphics.Buffer.Vertex.New(gd, verts, GraphicsResourceUsage.Immutable),
                            VertexPositionNormalTexture.Layout, verts.Length)
                    },
                    IndexBuffer = new IndexBufferBinding(
                        Stride.Graphics.Buffer.Index.New(gd, indices), true, indices.Length),
                    DrawCount = indices.Length,
                }
            };

            // Nearest-neighbour sampler so checker tiles have crisp edges
            var samplerDesc = new SamplerStateDescription(TextureFilter.Point, TextureAddressMode.Wrap);
            var sampler = SamplerState.New(gd, samplerDesc);

            var diffuseMap = new ComputeTextureColor(checkerTex)
            {
                AddressModeU = TextureAddressMode.Wrap,
                AddressModeV = TextureAddressMode.Wrap,
                Filtering    = TextureFilter.Point,
            };

            var material = Material.New(gd, new MaterialDescriptor
            {
                Attributes = new MaterialAttributes
                {
                    Diffuse      = new MaterialDiffuseMapFeature(diffuseMap),
                    DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                }
            });

            var floorEntity = new Entity("ground_checker");
            floorEntity.Add(new ModelComponent { Model = new Model { mesh, material } });
            Entity.AddChild(floorEntity);
        }
        catch (Exception ex)
        {
            Log.Warning($"Could not build checkerboard floor: {ex.Message}");
        }
    }

    public override void Update()
    {
        // Push telemetry to gauge each frame
        if (_speedoGauge != null && _car != null)
        {
            _speedoGauge.SpeedKmh      = _car.SpeedKmh;
            _speedoGauge.EngineRpm     = _car.EngineRpm;
            _speedoGauge.MaxRpm        = _car.MaxRpm;
            _speedoGauge.CurrentGear   = _car.CurrentGear;
            _speedoGauge.ThrottleInput = _car.ThrottleInput;
            _speedoGauge.BrakeInput    = _car.BrakeInput;
            _speedoGauge.HandbrakeEngaged = _car.HandbrakeEngaged;
        }

        // Toggle debug info with F3
        if (Input.IsKeyPressed(Keys.F3))
        {
	        _showDebug = !_showDebug;
        }

        if (_telemetryOverlay != null)
        {
            _telemetryOverlay.Car = _car;
            _telemetryOverlay.StatusText = _status;
            _telemetryOverlay.OverlayVisible = _showDebug;
        }

        if (_car == null)
        {
            DebugText.Print(_status, new Int2(10, 10));
        }
    }
}
