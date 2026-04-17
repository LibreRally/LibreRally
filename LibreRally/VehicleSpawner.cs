using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net.Sockets;
using LibreRally.Camera;
using LibreRally.HUD;
using LibreRally.Telemetry;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Content;
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
    public string VehicleFolderPath { get; set; } = @"Resources\BeamNG Vehicles\basic_car";

    /// <summary>
    /// Name of the .pc config file to load (with or without .pc extension).
    /// E.g. "rally_pro_asphalt" or "rally_pro_asphalt.pc".
    /// Leave empty to auto-detect (prefers rally_pro_asphalt.pc if present).
    /// </summary>
    public string ConfigFileName { get; set; } = "rally_pro_asphalt.pc";

    public Vector3 SpawnPosition { get; set; } = new Vector3(0, 0.15f, 0);
    public bool OutGaugeEnabled { get; set; }
    public int OutGaugeDelayCentiseconds { get; set; } = 1;
    public string OutGaugeIp { get; set; } = "127.0.0.1";
    public int OutGaugePort { get; set; } = 4444;
    public int OutGaugeId { get; set; }

    private string _status = "Loading...";
    private bool _showDebug = true;
    private VehicleTelemetryOverlay? _telemetryOverlay;
    private VehicleSelectionOverlay? _vehicleSelectionOverlay;
    private BeamNgVehicleCatalog? _vehicleCatalog;
    private List<BeamNgVehicleDescriptor> _availableVehicles = [];
    private int _selectedVehicleIndex;
    private bool _showVehicleMenu;
    private LoadedVehicle? _loadedVehicle;
    private UdpClient? _outGaugeClient;
    private string? _outGaugeTargetHost;
    private int _outGaugeTargetPort;
    private const double FailureLogIntervalSeconds = 5d;
    private bool _outGaugeSendFailed;
    private double _outGaugeNextFailureLogTimeSeconds;
    private float _outGaugeElapsed;
    private const float MinTrackUvScale = 0.25f;
    private const float TrackMeshBoundsHalfHeight = 0.02f;
    private const float BankedSectionRollAngleRadians = -0.35f;
    private const float InclineSectionPitchAngleRadians = 0.22f;
    private const float TrackSurfaceGlossiness = 0.2f;
    private const float TrackSurfaceMetalness = 0f;

    /// <summary>
    /// Lightweight configuration for one procedural test-track section.
    /// </summary>
    private readonly struct TrackSegmentDefinition(
        string name,
        Vector3 localPosition,
        Quaternion localRotation,
        Vector3 colliderSize,
        Color4 albedo,
        float uvScale,
        float frictionCoefficient)
    {
        public string Name { get; } = name;
        public Vector3 LocalPosition { get; } = localPosition;
        public Quaternion LocalRotation { get; } = localRotation;
        public Vector3 ColliderSize { get; } = colliderSize;
        public Color4 Albedo { get; } = albedo;
        public float UvScale { get; } = uvScale;
        public float FrictionCoefficient { get; } = frictionCoefficient;
    }

    public override void Start()
    {
        AddGroundPhysics();
        EnsureTelemetryOverlay();
        InitializeVehicleCatalog();
        EnsureVehicleSelectionOverlay();

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

    private void LoadVehicle(BeamNgVehicleDescriptor? selectedVehicle = null)
    {
        var requestedConfig = string.IsNullOrWhiteSpace(ConfigFileName) ? "<auto>" : ConfigFileName;
        var requestedSource = selectedVehicle?.SourcePath ?? VehicleFolderPath;
        BeamNgResolvedVehicle? resolvedVehicle = null;
        if (_vehicleCatalog != null)
        {
            resolvedVehicle = _vehicleCatalog.ResolveVehicle(requestedSource);
        }

        var basePath = resolvedVehicle?.VehicleFolderPath ?? (Path.IsPathRooted(requestedSource)
            ? requestedSource
            : Path.Combine(AppContext.BaseDirectory, requestedSource));
        Log.Info($"[VehicleSpawner] Load request: source='{requestedSource}' resolved='{basePath}' config='{requestedConfig}'");

        var loader = new VehicleLoader((Game)Game);
        var vehicle = resolvedVehicle != null
            ? loader.Load(resolvedVehicle, string.IsNullOrWhiteSpace(ConfigFileName) ? null : ConfigFileName)
            : loader.Load(basePath, string.IsNullOrWhiteSpace(ConfigFileName) ? null : ConfigFileName);

        if (selectedVehicle != null)
        {
            VehicleFolderPath = selectedVehicle.SourcePath;
            _selectedVehicleIndex = _availableVehicles.FindIndex(vehicleDescriptor =>
                vehicleDescriptor.SourcePath.Equals(selectedVehicle.SourcePath, StringComparison.OrdinalIgnoreCase));
        }

        UnloadVehicle();
        _loadedVehicle = vehicle;

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
        _status = $"Loaded: {vehicle.Definition.VehicleName} src={(resolvedVehicle?.SourceDescription ?? "folder")} cfg={activeConfig} mass={vehicle.Diagnostics.EstimatedMassKg:F0}kg";
        Log.Info($"[VehicleSpawner] {_status} folder='{vehicle.Diagnostics.VehicleFolderPath}'");

        AttachCamera(vehicle.ChassisEntity, vehicle.CarComponent);
        AttachHud(vehicle.CarComponent);
        AttachSpeedoGauge(vehicle.CarComponent);
        AttachTelemetryOverlay(vehicle.CarComponent);
    }

    private void UnloadVehicle()
    {
        if (_loadedVehicle?.RootEntity != null)
        {
            SceneSystem.SceneInstance.RootScene.Entities.Remove(_loadedVehicle.RootEntity);
        }

        _loadedVehicle = null;
        _car = null;
        _outGaugeElapsed = 0f;
    }

    private SpeedoGauge? _speedoGauge;
    private EngineGaugeStrip? _engineGaugeStrip;

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

        // Attach the engine auxiliary gauge strip below the speedo cluster
        if (_engineGaugeStrip != null)
        {
            ((Game)Game).GameSystems.Remove(_engineGaugeStrip);
            _engineGaugeStrip.Dispose();
        }
        _engineGaugeStrip = new EngineGaugeStrip(Services);
        ((Game)Game).GameSystems.Add(_engineGaugeStrip);

        // The gauge reads from _speedoGauge/_engineGaugeStrip properties we update in Update()
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

    private void EnsureVehicleSelectionOverlay()
    {
        if (_vehicleSelectionOverlay != null)
        {
            return;
        }

        _vehicleSelectionOverlay = new VehicleSelectionOverlay(Services)
        {
            Vehicles = _availableVehicles,
            SelectedIndex = _selectedVehicleIndex,
            OverlayVisible = _showVehicleMenu,
            StatusText = _status,
        };

        ((Game)Game).GameSystems.Add(_vehicleSelectionOverlay);
    }

    private void InitializeVehicleCatalog()
    {
        var bundledVehiclesRoot = Path.Combine(AppContext.BaseDirectory, "Resources", "BeamNG Vehicles");
        var beamNgContentVehiclesRoot = BeamNgVehicleCatalog.DetectBeamNgContentVehiclesRoot();
        _vehicleCatalog = new BeamNgVehicleCatalog(bundledVehiclesRoot, beamNgContentVehiclesRoot);
        _availableVehicles = _vehicleCatalog.DiscoverBundledVehicles().ToList();
        _selectedVehicleIndex = ResolveVehicleSelectionIndex();
    }

    private int ResolveVehicleSelectionIndex()
    {
        if (_availableVehicles.Count == 0)
        {
            return -1;
        }

        var absoluteVehiclePath = Path.IsPathRooted(VehicleFolderPath)
            ? VehicleFolderPath
            : Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, VehicleFolderPath));

        var exactPathIndex = _availableVehicles.FindIndex(vehicle =>
            vehicle.SourcePath.Equals(absoluteVehiclePath, StringComparison.OrdinalIgnoreCase));
        if (exactPathIndex >= 0)
        {
            return exactPathIndex;
        }

        var idIndex = _availableVehicles.FindIndex(vehicle =>
            vehicle.VehicleId.Equals(VehicleFolderPath, StringComparison.OrdinalIgnoreCase) ||
            absoluteVehiclePath.EndsWith(Path.DirectorySeparatorChar + vehicle.VehicleId, StringComparison.OrdinalIgnoreCase));
        return idIndex >= 0 ? idIndex : 0;
    }

    private void HandleVehicleSelectionInput()
    {
        if (Input.IsKeyPressed(Keys.F2))
        {
            _showVehicleMenu = !_showVehicleMenu;
        }

        if (_car != null)
        {
            _car.PlayerInputEnabled = !_showVehicleMenu;
        }

        if (!_showVehicleMenu)
        {
            return;
        }

        if (_availableVehicles.Count == 0)
        {
            if (Input.IsKeyPressed(Keys.Escape))
            {
                _showVehicleMenu = false;
            }

            return;
        }

        if (_selectedVehicleIndex < 0)
        {
            _selectedVehicleIndex = 0;
        }

        if (Input.IsKeyPressed(Keys.Up))
        {
            _selectedVehicleIndex = (_selectedVehicleIndex - 1 + _availableVehicles.Count) % _availableVehicles.Count;
        }

        if (Input.IsKeyPressed(Keys.Down))
        {
            _selectedVehicleIndex = (_selectedVehicleIndex + 1) % _availableVehicles.Count;
        }

        if (Input.IsKeyPressed(Keys.Enter))
        {
            var selectedVehicle = _availableVehicles[_selectedVehicleIndex];
            _showVehicleMenu = false;
            LoadVehicle(selectedVehicle);
            return;
        }

        if (Input.IsKeyPressed(Keys.Escape))
        {
            _showVehicleMenu = false;
        }
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

        AddRoadTestSections();
    }

    private void AddRoadTestSections()
    {
        try
        {
            var gd = ((Game)Game).GraphicsDevice;
            var segments = new[]
            {
                new TrackSegmentDefinition(
                    "test_track_tarmac",
                    new Vector3(0f, 0.5f, 0f),
                    Quaternion.Identity,
                    new Vector3(60f, 0.2f, 16f),
                    new Color4(0.21f, 0.21f, 0.22f, 1f),
                    7.5f,
                    1.35f),
                new TrackSegmentDefinition(
                    "test_track_gravel",
                    new Vector3(-22f, 0.5f, 26f),
                    Quaternion.Identity,
                    new Vector3(28f, 0.25f, 14f),
                    new Color4(0.47f, 0.40f, 0.29f, 1f),
                    5f,
                    1.05f),
                new TrackSegmentDefinition(
                    "test_track_snow",
                    new Vector3(22f, 0.5f, 26f),
                    Quaternion.Identity,
                    new Vector3(28f, 0.25f, 14f),
                    new Color4(0.88f, 0.9f, 0.92f, 1f),
                    5f,
                    0.85f),
                new TrackSegmentDefinition(
                    "test_track_banked",
                    new Vector3(38f, 1.8f, -6f),
                    Quaternion.RotationZ(BankedSectionRollAngleRadians),
                    new Vector3(20f, 0.25f, 28f),
                    new Color4(0.24f, 0.24f, 0.25f, 1f),
                    4f,
                    1.2f),
                new TrackSegmentDefinition(
                    "test_track_incline",
                    new Vector3(0f, 1.7f, 40f),
                    Quaternion.RotationX(InclineSectionPitchAngleRadians),
                    new Vector3(12f, 0.25f, 42f),
                    new Color4(0.23f, 0.23f, 0.24f, 1f),
                    5.25f,
                    1.25f),
            };

            foreach (var segment in segments)
            {
                Entity.AddChild(CreateTrackSegment(gd, segment));
            }
        }
        catch (Exception ex)
        {
            Log.Warning($"Could not build test track sections: {ex.Message}");
        }
    }

    private Entity CreateTrackSegment(GraphicsDevice graphicsDevice, in TrackSegmentDefinition segment)
    {
        var halfWidth = segment.ColliderSize.X * 0.5f;
        var halfLength = segment.ColliderSize.Z * 0.5f;
        // Clamp UV tiling to a sane minimum to avoid near-zero texture repetition artifacts.
        var uvScale = MathF.Max(MinTrackUvScale, segment.UvScale);

        var vertices = new VertexPositionNormalTexture[]
        {
            new(new Vector3(-halfWidth, 0f, -halfLength), Vector3.UnitY, new Vector2(0f, 0f)),
            new(new Vector3(halfWidth, 0f, -halfLength), Vector3.UnitY, new Vector2(uvScale, 0f)),
            new(new Vector3(halfWidth, 0f, halfLength), Vector3.UnitY, new Vector2(uvScale, uvScale)),
            new(new Vector3(-halfWidth, 0f, halfLength), Vector3.UnitY, new Vector2(0f, uvScale)),
        };
        var indices = new[] { 0, 1, 2, 0, 2, 3 };

        var mesh = new Mesh
        {
            BoundingBox = new BoundingBox(
                new Vector3(-halfWidth, -TrackMeshBoundsHalfHeight, -halfLength),
                new Vector3(halfWidth, TrackMeshBoundsHalfHeight, halfLength)),
            Draw = new MeshDraw
            {
                PrimitiveType = PrimitiveType.TriangleList,
                VertexBuffers =
                [
                    new VertexBufferBinding(
                        Stride.Graphics.Buffer.Vertex.New(graphicsDevice, vertices, GraphicsResourceUsage.Immutable),
                        VertexPositionNormalTexture.Layout,
                        vertices.Length),
                ],
                IndexBuffer = new IndexBufferBinding(
                    Stride.Graphics.Buffer.Index.New(graphicsDevice, indices),
                    true,
                    indices.Length),
                DrawCount = indices.Length,
            },
        };

        var material = Material.New(graphicsDevice, new MaterialDescriptor
        {
            Attributes = new MaterialAttributes
            {
                Diffuse = new MaterialDiffuseMapFeature(new ComputeColor { Value = segment.Albedo }),
                DiffuseModel = new MaterialDiffuseLambertModelFeature(),
                MicroSurface = new MaterialGlossinessMapFeature(new ComputeFloat { Value = TrackSurfaceGlossiness }),
                Specular = new MaterialMetalnessMapFeature(new ComputeFloat { Value = TrackSurfaceMetalness }),
            },
        });

        var trackEntity = new Entity(segment.Name);
        trackEntity.Transform.Position = segment.LocalPosition;
        trackEntity.Transform.Rotation = segment.LocalRotation;
        trackEntity.Add(new StaticComponent
        {
            FrictionCoefficient = segment.FrictionCoefficient,
            Collider = new CompoundCollider
            {
                Colliders =
                {
                    new BoxCollider { Size = segment.ColliderSize },
                },
            },
        });
        trackEntity.Add(new ModelComponent { Model = new Model { mesh, material } });

        return trackEntity;
    }

    public override void Update()
    {
        HandleVehicleSelectionInput();
        var dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;

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
            _speedoGauge.TractionLossDetected = _car.TractionLossDetected;
            _speedoGauge.TractionControlActive = _car.TractionControlActive;
            _speedoGauge.DrivenWheelSlipRatio = _car.DrivenWheelSlipRatio;
        }

        // Push engine auxiliary telemetry to the gauge strip
        if (_engineGaugeStrip != null && _car != null)
        {
            _engineGaugeStrip.TurboBoostBar      = _car.TurboBoostBar;
            _engineGaugeStrip.TurboMaxBoostBar    = _car.TurboMaxBoostPsi * RallyCarComponent.PsiToBar;
            _engineGaugeStrip.HasTurbo            = _car.HasTurbo;
            _engineGaugeStrip.EngineTempC          = _car.EngineTemp;
            _engineGaugeStrip.EngineTempDamageC    = _car.EngineBlockTempDamageThreshold;
            _engineGaugeStrip.FuelLiters           = _car.FuelLiters;
            _engineGaugeStrip.FuelCapacityLiters   = _car.FuelCapacityLiters;
            _engineGaugeStrip.HasFuel              = _car.FuelCapacityLiters > 0f;
            _engineGaugeStrip.OilPressureBar       = _car.OilPressure;
            _engineGaugeStrip.HasOil               = _car.OilVolumeLiters > 0f;
        }

        SendOutGaugeTelemetry(dt);

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

        if (_vehicleSelectionOverlay != null)
        {
            _vehicleSelectionOverlay.Vehicles = _availableVehicles;
            _vehicleSelectionOverlay.SelectedIndex = _selectedVehicleIndex;
            _vehicleSelectionOverlay.OverlayVisible = _showVehicleMenu;
            _vehicleSelectionOverlay.StatusText = _status;
        }

        if (_car == null)
        {
            DebugText.Print(_status, new Int2(10, 10));
        }
    }

    private void SendOutGaugeTelemetry(float deltaTime)
    {
        if (!OutGaugeEnabled || _car == null)
        {
            _outGaugeElapsed = 0f;
            if (!OutGaugeEnabled)
            {
                DisposeOutGaugeClient();
            }
            return;
        }

        var sendIntervalSeconds = Math.Max(0, OutGaugeDelayCentiseconds) * 0.01f;
        _outGaugeElapsed += Math.Max(0f, deltaTime);
        if (sendIntervalSeconds > 0f && _outGaugeElapsed < sendIntervalSeconds)
        {
            return;
        }

        _outGaugeElapsed = 0f;
        EnsureOutGaugeClient();
        if (_outGaugeClient == null || string.IsNullOrWhiteSpace(_outGaugeTargetHost))
        {
            return;
        }

        try
        {
            var sessionMilliseconds = Math.Max(0d, Game.UpdateTime.Total.TotalMilliseconds);
            var snapshot = OutGaugeProtocol.FromCar(_car, unchecked((uint)sessionMilliseconds));
            var payload = OutGaugeProtocol.Encode(snapshot, OutGaugeId);
            _outGaugeClient.Send(payload, payload.Length, _outGaugeTargetHost, _outGaugeTargetPort);
            _outGaugeSendFailed = false;
        }
        catch (SocketException ex)
        {
            HandleOutGaugeSendFailure(ex);
        }
        catch (ObjectDisposedException ex)
        {
            HandleOutGaugeSendFailure(ex);
        }
        catch (InvalidOperationException ex)
        {
            HandleOutGaugeSendFailure(ex);
        }
    }

    private void EnsureOutGaugeClient()
    {
        var targetHost = OutGaugeIp?.Trim();
        var targetPort = OutGaugePort;
        if (string.IsNullOrWhiteSpace(targetHost) || targetPort is < 1 or > 65535)
        {
            DisposeOutGaugeClient();
            return;
        }

        if (_outGaugeClient != null &&
            string.Equals(_outGaugeTargetHost, targetHost, StringComparison.OrdinalIgnoreCase) &&
            _outGaugeTargetPort == targetPort)
        {
            return;
        }

        DisposeOutGaugeClient();
        _outGaugeClient = new UdpClient();
        _outGaugeTargetHost = targetHost;
        _outGaugeTargetPort = targetPort;
        _outGaugeSendFailed = false;
        _outGaugeNextFailureLogTimeSeconds = 0d;
    }

    private void DisposeOutGaugeClient()
    {
        _outGaugeClient?.Dispose();
        _outGaugeClient = null;
        _outGaugeTargetHost = null;
        _outGaugeTargetPort = 0;
        _outGaugeSendFailed = false;
        _outGaugeNextFailureLogTimeSeconds = 0d;
    }

    private void HandleOutGaugeSendFailure(Exception ex)
    {
        var sessionSeconds = Math.Max(0d, Game.UpdateTime.Total.TotalSeconds);
        if (!_outGaugeSendFailed || sessionSeconds >= _outGaugeNextFailureLogTimeSeconds)
        {
            Log.Warning($"OutGauge send failed ({_outGaugeTargetHost}:{_outGaugeTargetPort}): {ex.Message}");
            _outGaugeNextFailureLogTimeSeconds = sessionSeconds + FailureLogIntervalSeconds;
        }

        _outGaugeSendFailed = true;
    }
}
