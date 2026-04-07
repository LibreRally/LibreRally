using System;
using System.IO;
using System.Linq;
using LibreRally.Camera;
using LibreRally.Vehicle;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Definitions.Colliders;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally;

[ComponentCategory("LibreRally")]
public class VehicleSpawner : SyncScript
{
    public string VehicleFolderPath { get; set; } = @"Resources\BeamNG Vehicles\sunburst\sunburst2";

    /// <summary>
    /// Name of the .pc config file to load (with or without .pc extension).
    /// E.g. "rally_pro_asphalt" or "rally_pro_asphalt.pc".
    /// Leave empty to auto-detect (prefers rally_pro_asphalt.pc if present).
    /// </summary>
    public string ConfigFileName { get; set; } = "rally_pro_asphalt";

    public Vector3 SpawnPosition { get; set; } = new Vector3(0, 0.15f, 0);

    private string _status = "Loading...";
    private Entity? _chassis;
    private Entity[] _wheels = Array.Empty<Entity>();

    public override void Start()
    {
        AddGroundPhysics();

        try
        {
            LoadVehicle();
        }
        catch (Exception ex)
        {
            Log.Error($"Failed to load vehicle: {ex}");
            _status = $"Load error: {ex.Message}";
        }
    }

    private void LoadVehicle()
    {
        var basePath = Path.IsPathRooted(VehicleFolderPath)
            ? VehicleFolderPath
            : Path.Combine(AppContext.BaseDirectory, VehicleFolderPath);

        var loader = new VehicleLoader((Game)Game);
        LoadedVehicle vehicle = loader.Load(basePath, string.IsNullOrEmpty(ConfigFileName) ? null : ConfigFileName);

        // CRITICAL: apply SpawnPosition to each physics entity directly.
        // They must be at scene root level (no offset parent) so BEPU writes correct world positions back.
        foreach (var child in vehicle.RootEntity.GetChildren().ToList())
            child.Transform.Position += SpawnPosition;

        // Keep root at origin (it holds the RallyCarComponent SyncScript)
        vehicle.RootEntity.Transform.Position = Vector3.Zero;
        SceneSystem.SceneInstance.RootScene.Entities.Add(vehicle.RootEntity);

        _chassis = vehicle.ChassisEntity;
        _wheels  = new[] { vehicle.WheelFL, vehicle.WheelFR, vehicle.WheelRL, vehicle.WheelRR };
        _status = $"Loaded: {vehicle.Definition.VehicleName}  nodes={vehicle.Definition.Nodes.Count}";
        Log.Info(_status);

        AttachCamera(vehicle.ChassisEntity, vehicle.CarComponent);
    }

    private void AttachCamera(Entity chassis, RallyCarComponent car)
    {
        var cameraEntity = SceneSystem.SceneInstance.RootScene.Entities
            .FirstOrDefault(e => e.Get<CameraComponent>() != null);
        if (cameraEntity == null) return;

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
    }

    public override void Update()
    {
        if (_chassis != null)
        {
            var p = _chassis.Transform.WorldMatrix.TranslationVector;
            DebugText.Print($"Car: {p.X:F1}, {p.Y:F1}, {p.Z:F1}  | {_status}", new Int2(10, 10));
            for (int i = 0; i < _wheels.Length; i++)
            {
                var wp = _wheels[i].Transform.WorldMatrix.TranslationVector;
                DebugText.Print($"W{i}({_wheels[i].Name}): {wp.X:F2},{wp.Y:F2},{wp.Z:F2}", new Int2(10, 50 + i * 16));
            }
        }
        else
        {
            DebugText.Print(_status, new Int2(10, 10));
        }
    }}
