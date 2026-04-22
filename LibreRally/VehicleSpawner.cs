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
using LibreRally.Vehicle.Physics;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Definitions.Colliders;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Input;
using Stride.Rendering;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;

namespace LibreRally
{
	/// <summary>
	/// Defines the available actions in the pause menu.
	/// </summary>
	public enum PauseMenuAction
	{
		/// <summary>Resume the current driving session.</summary>
		ResumeDriving,
		/// <summary>Reset the vehicle to the spawn position.</summary>
		ResetVehicle,
		/// <summary>Open the garage setup/tuning menu.</summary>
		GarageSetup,
		/// <summary>Open the vehicle selection menu.</summary>
		VehicleSelect,
		/// <summary>Open the physics calibration menu.</summary>
		PhysicsCalibration,
		/// <summary>Open the telemetry controls menu.</summary>
		Telemetry,
	}

	/// <summary>
	/// Represents an entry in the pause menu, associating a UI <paramref name="Item" /> with an <paramref name="Action" />.
	/// </summary>
	/// <param name="Item">The UI item descriptor.</param>
	/// <param name="Action">The action to execute when selected.</param>
	public readonly record struct PauseMenuEntry(PauseMenuItem Item, PauseMenuAction Action);

	/// <summary>
	/// Defines the available actions in the telemetry menu.
	/// </summary>
	internal enum TelemetryMenuAction
	{
		ToggleOutGauge,
		ConnectOutGauge,
		DisconnectOutGauge,
		ReconnectOutGauge,
		ToggleOutSim,
		ConnectOutSim,
		DisconnectOutSim,
		ReconnectOutSim,
	}

	/// <summary>
	/// Main script responsible for spawning vehicles, managing UI overlays, and handling telemetry.
	/// </summary>
	[ComponentCategory("LibreRally")]
	public class VehicleSpawner : SyncScript
	{
		/// <summary>
		/// Gets or sets the path to the folder containing BeamNG vehicles.
		/// </summary>
		public string VehicleFolderPath { get; set; } = @"Resources\BeamNG Vehicles\basic_car";

		/// <summary>
		/// Name of the .pc config file to load (with or without .pc extension).
		/// E.g. "rally_pro_asphalt" or "rally_pro_asphalt.pc".
		/// Leave empty to auto-detect (prefers rally_pro_asphalt.pc if present).
		/// </summary>
		public string ConfigFileName { get; set; } = "rally_pro_asphalt.pc";

		/// <summary>
		/// Gets or sets the initial spawn position for the vehicle.
		/// </summary>
		public Vector3 SpawnPosition { get; set; } = new Vector3(0, 0.15f, 0);

		/// <summary>
		/// Gets or sets a value indicating whether OutGauge telemetry is enabled.
		/// </summary>
		public bool OutGaugeEnabled { get; set; } = true;

		/// <summary>
		/// Gets or sets the delay between OutGauge packets in centiseconds.
		/// </summary>
		public int OutGaugeDelayCentiseconds { get; set; } = 1;

		/// <summary>
		/// Gets or sets the target IP address for OutGauge telemetry.
		/// </summary>
		public string OutGaugeIp { get; set; } = "127.0.0.1";

		/// <summary>
		/// Gets or sets the target port for OutGauge telemetry.
		/// </summary>
		public int OutGaugePort { get; set; } = 4444;

		/// <summary>
		/// Gets or sets the unique ID for OutGauge telemetry.
		/// </summary>
		public int OutGaugeId { get; set; }

		/// <summary>
		/// Gets or sets a value indicating whether OutSim telemetry is enabled.
		/// </summary>
		public bool OutSimEnabled { get; set; }

		/// <summary>
		/// Gets or sets the delay between OutSim packets in centiseconds.
		/// </summary>
		public int OutSimDelayCentiseconds { get; set; } = 1;

		/// <summary>
		/// Gets or sets the target IP address for OutSim telemetry.
		/// </summary>
		public string OutSimIp { get; set; } = "127.0.0.1";

		/// <summary>
		/// Gets or sets the target port for OutSim telemetry.
		/// </summary>
		public int OutSimPort { get; set; } = 4123;

		/// <summary>
		/// Gets or sets the unique ID for OutSim telemetry.
		/// </summary>
		public int OutSimId { get; set; }

		private string _status = "Loading...";
		private bool _showDebug = true;
		private DrivingHudOverlay? _drivingHudOverlay;
		private PauseMenuOverlay? _pauseMenuOverlay;
		private SetupUiShellOverlay? _setupUiShellOverlay;
		private VehicleSelectionOverlay? _vehicleSelectionOverlay;
		private PhysicsCalibrationOverlay? _physicsCalibrationOverlay;
		private TelemetryOverlay? _telemetryOverlay;
		private BeamNgVehicleCatalog? _vehicleCatalog;
		private List<BeamNgVehicleDescriptor> _availableVehicles = [];
		private readonly VehicleSetupOverrides _setupOverrides = new();
		private int _selectedVehicleIndex;
		private int _pauseMenuSelectedIndex;
		private int _telemetryMenuSelectedIndex;
		private bool _vehicleSelectionOpenedFromPauseMenu;
		private MenuScreen _activeMenuScreen;
		private LoadedVehicle? _loadedVehicle;
		private UdpClient? _outGaugeClient;
		private bool _outGaugeConnectionRequested = true;
		private string? _outGaugeTargetHost;
		private int _outGaugeTargetPort;
		private const double FailureLogIntervalSeconds = 5d;
		private bool _outGaugeSendFailed;
		private double _outGaugeNextFailureLogTimeSeconds;
		private float _outGaugeElapsed;
		private UdpClient? _outSimClient;
		private bool _outSimConnectionRequested = true;
		private string? _outSimTargetHost;
		private int _outSimTargetPort;
		private bool _outSimSendFailed;
		private double _outSimNextFailureLogTimeSeconds;
		private float _outSimElapsed;
		private bool _outSimHasPreviousLinearVelocity;
		private Vector3 _outSimPreviousLinearVelocity;
		private TrackSurfaceMaterialLibrary? _trackSurfaceMaterialLibrary;
		private const float MinTrackUvScale = 0.25f;
		private const float TrackMeshBoundsHalfHeight = 0.02f;
		private const float TrackSurfaceLift = 0.01f;
		private const float BankedSectionRollAngleRadians = -0.35f;
		private const float InclineSectionPitchAngleRadians = 0.22f;
		private const float TrackSurfaceGlossiness = 0.2f;
		private const float TrackSurfaceMetalness = 0f;
		private const float PsiToKpa = 6.894757f;
		private const int PauseMenuResumeIndex = 0;
		private const int PauseMenuResetVehicleIndex = 1;
		private const int PauseMenuGarageSetupIndex = 2;
		private const int PauseMenuVehicleSelectIndex = 3;
		private const int PauseMenuPhysicsCalibrationIndex = 4;
		private const int PauseMenuTelemetryIndex = 5;
		private const int PauseMenuItemCount = 6;
		private const int TelemetryMenuItemCount = 8;
		private static readonly IReadOnlyList<PauseMenuEntry> PauseMenuEntries =
		[
			new(new PauseMenuItem("Resume Driving", "Return to the stage and hand control back to the driver."), PauseMenuAction.ResumeDriving),
			new(new PauseMenuItem("Reset Vehicle", "Respawn the current car back at the spawn point."), PauseMenuAction.ResetVehicle),
			new(new PauseMenuItem("Garage Setup", "Open the restored Myra tuning shell for stage prep changes."), PauseMenuAction.GarageSetup),
			new(new PauseMenuItem("Vehicle Select", "Load a different bundled vehicle from the current catalog."), PauseMenuAction.VehicleSelect),
			new(new PauseMenuItem("Physics Calibration", "Isolate tyre model contributions and tune anti-slip parameters live."), PauseMenuAction.PhysicsCalibration),
			new(new PauseMenuItem("Telemetry", "Enable or disable OutGauge/OutSim, then connect, disconnect, or reconnect their UDP sockets."), PauseMenuAction.Telemetry),
		];

		internal enum MenuScreen
		{
			None,
			Pause,
			GarageSetup,
			VehicleSelection,
			PhysicsCalibration,
			Telemetry,
		}

		/// <summary>
		/// Lightweight configuration for one procedural test-track section.
		/// </summary>
		private readonly struct TrackSegmentDefinition(
			string name,
			Vector3 localPosition,
			Quaternion localRotation,
			Vector3 colliderSize,
			SurfaceType surfaceType,
			Color4 albedo,
			float uvScale,
			float frictionCoefficient)
		{
			public string Name { get; } = name;
			public Vector3 LocalPosition { get; } = localPosition;
			public Quaternion LocalRotation { get; } = localRotation;
			public Vector3 ColliderSize { get; } = colliderSize;
			public SurfaceType SurfaceType { get; } = surfaceType;
			public Color4 Albedo { get; } = albedo;
			public float UvScale { get; } = uvScale;
			public float FrictionCoefficient { get; } = frictionCoefficient;
		}

		/// <summary>
		/// Initializes the track, overlays, and vehicle catalog at scene startup.
		/// </summary>
		public override void Start()
		{
			AddGroundPhysics();
			EnsureDrivingHudOverlay();
			InitializeVehicleCatalog();
			EnsurePauseMenuOverlay();
			EnsureSetupUiShellOverlay();
			EnsureVehicleSelectionOverlay();
			EnsurePhysicsCalibrationOverlay();
			EnsureTelemetryOverlay();

			try
			{
				LoadVehicle();
			}
			catch (Exception ex)
			{
				Log.Error($"Failed to load vehicle: {ex}");
				_status = $"Load error: {ex.Message}";
				if (_drivingHudOverlay != null)
				{
					_drivingHudOverlay.StatusText = _status;
				}
			}
		}

		private void LoadVehicle(BeamNgVehicleDescriptor? selectedVehicle = null)
		{
			var requestedConfig = string.IsNullOrWhiteSpace(ConfigFileName) ? "<auto>" : ConfigFileName;
			var requestedSource = selectedVehicle?.SourcePath ?? VehicleFolderPath;
			if (selectedVehicle != null &&
			    !selectedVehicle.SourcePath.Equals(VehicleFolderPath, StringComparison.OrdinalIgnoreCase))
			{
				_setupOverrides.Clear();
			}

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
				? loader.Load(resolvedVehicle, string.IsNullOrWhiteSpace(ConfigFileName) ? null : ConfigFileName, _setupOverrides)
				: loader.Load(basePath, string.IsNullOrWhiteSpace(ConfigFileName) ? null : ConfigFileName, _setupOverrides);

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
			AttachDrivingHud(vehicle.CarComponent);
			BindGarageSetupOverlay();
			BindPhysicsCalibrationOverlay();
			RefreshTelemetryOverlay();
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
			_outSimElapsed = 0f;
			_outSimHasPreviousLinearVelocity = false;
		}

		private RallyCarComponent? _car;

		private void AttachDrivingHud(RallyCarComponent car)
		{
			EnsureDrivingHudOverlay();
			if (_drivingHudOverlay == null)
			{
				return;
			}

			_drivingHudOverlay.Car = car;
			_drivingHudOverlay.StatusText = _status;
			_drivingHudOverlay.DebugOverlayVisible = _showDebug;
			_car = car;
		}

		private void EnsureDrivingHudOverlay()
		{
			if (_drivingHudOverlay != null)
			{
				return;
			}

			_drivingHudOverlay = new DrivingHudOverlay(Services)
			{
				Car = _car,
				StatusText = _status,
				DebugOverlayVisible = _showDebug,
			};

			((Game)Game).GameSystems.Add(_drivingHudOverlay);
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
				OverlayVisible = _activeMenuScreen == MenuScreen.VehicleSelection,
				StatusText = _status,
			};
			_vehicleSelectionOverlay.ItemActivated = selectedIndex =>
			{
				if (selectedIndex < 0 || selectedIndex >= _availableVehicles.Count)
				{
					return;
				}

				_selectedVehicleIndex = selectedIndex;
				_activeMenuScreen = MenuScreen.None;
				_vehicleSelectionOpenedFromPauseMenu = false;
				LoadVehicle(_availableVehicles[selectedIndex]);
			};

			((Game)Game).GameSystems.Add(_vehicleSelectionOverlay);
		}

		private void EnsurePauseMenuOverlay()
		{
			if (_pauseMenuOverlay != null)
			{
				return;
			}

			_pauseMenuOverlay = new PauseMenuOverlay(Services)
			{
				Items = PauseMenuEntries.Select(entry => entry.Item).ToArray(),
				SelectedIndex = _pauseMenuSelectedIndex,
				OverlayVisible = _activeMenuScreen == MenuScreen.Pause,
				VehicleName = GetCurrentVehicleName(),
				StatusText = _status,
			};
			_pauseMenuOverlay.ItemActivated = selectedIndex =>
			{
				_pauseMenuSelectedIndex = selectedIndex;
				ExecutePauseMenuAction(ResolvePauseMenuAction(selectedIndex));
			};

			((Game)Game).GameSystems.Add(_pauseMenuOverlay);
		}

		private void EnsureSetupUiShellOverlay()
		{
			if (_setupUiShellOverlay != null)
			{
				return;
			}

			_setupUiShellOverlay = new SetupUiShellOverlay(Services)
			{
				OverlayVisible = _activeMenuScreen == MenuScreen.GarageSetup,
				VehicleName = GetCurrentVehicleName(),
				StatusText = _status,
			};
			_setupUiShellOverlay.ApplyRequested = ApplyGarageSetupChanges;
			_setupUiShellOverlay.CloseRequested = CloseGarageSetup;

			((Game)Game).GameSystems.Add(_setupUiShellOverlay);
		}

		private void EnsurePhysicsCalibrationOverlay()
		{
			if (_physicsCalibrationOverlay != null)
			{
				return;
			}

			_physicsCalibrationOverlay = new PhysicsCalibrationOverlay(Services)
			{
				OverlayVisible = _activeMenuScreen == MenuScreen.PhysicsCalibration,
				VehicleName = GetCurrentVehicleName(),
				StatusText = _status,
			};
			_physicsCalibrationOverlay.CloseRequested = ClosePhysicsCalibration;

			((Game)Game).GameSystems.Add(_physicsCalibrationOverlay);
		}

		private void EnsureTelemetryOverlay()
		{
			if (_telemetryOverlay != null)
			{
				return;
			}

			_telemetryOverlay = new TelemetryOverlay(Services)
			{
				Items = BuildTelemetryMenuItems(),
				SelectedIndex = _telemetryMenuSelectedIndex,
				OverlayVisible = _activeMenuScreen == MenuScreen.Telemetry,
				VehicleName = GetCurrentVehicleName(),
				StatusText = _status,
				OutGaugeSummary = BuildOutGaugeSummary(),
				OutSimSummary = BuildOutSimSummary(),
			};
			_telemetryOverlay.ItemActivated = selectedIndex =>
			{
				_telemetryMenuSelectedIndex = Math.Clamp(selectedIndex, 0, TelemetryMenuItemCount - 1);
				ExecuteTelemetryMenuAction(ResolveTelemetryMenuAction(_telemetryMenuSelectedIndex));
			};

			((Game)Game).GameSystems.Add(_telemetryOverlay);
		}

		private void BindPhysicsCalibrationOverlay()
		{
			if (_physicsCalibrationOverlay == null)
			{
				return;
			}

			_physicsCalibrationOverlay.BindVehicle(_loadedVehicle?.CarComponent);
		}

		private void BindGarageSetupOverlay()
		{
			if (_setupUiShellOverlay == null)
			{
				return;
			}

			_setupUiShellOverlay.BindVehicle(_loadedVehicle, _setupOverrides, _status);
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

		internal static bool IsPauseMenuToggleRequested(bool keyboardPausePressed, bool controllerStartPressed) =>
			keyboardPausePressed || controllerStartPressed;

		internal static bool IsVehicleMenuShortcutRequested(bool keyboardTogglePressed) =>
			keyboardTogglePressed;

		internal static IReadOnlyList<PauseMenuEntry> GetPauseMenuEntries() => PauseMenuEntries;

		internal static PauseMenuAction ResolvePauseMenuAction(int selectedIndex) =>
			PauseMenuEntries[Math.Clamp(selectedIndex, 0, PauseMenuEntries.Count - 1)].Action;

		internal static MenuScreen ResolvePauseMenuToggle(MenuScreen currentScreen) =>
			currentScreen == MenuScreen.Pause ? MenuScreen.None : MenuScreen.Pause;

		internal static MenuScreen ResolveVehicleSelectionCloseTarget(bool openedFromPauseMenu) =>
			openedFromPauseMenu ? MenuScreen.Pause : MenuScreen.None;

		internal static bool IsVehicleMenuMoveUpRequested(bool keyboardUpPressed, bool controllerUpPressed) =>
			keyboardUpPressed || controllerUpPressed;

		internal static bool IsVehicleMenuMoveDownRequested(bool keyboardDownPressed, bool controllerDownPressed) =>
			keyboardDownPressed || controllerDownPressed;

		internal static bool IsVehicleMenuConfirmRequested(bool keyboardConfirmPressed, bool controllerConfirmPressed) =>
			keyboardConfirmPressed || controllerConfirmPressed;

		internal static bool IsVehicleMenuCancelRequested(bool keyboardCancelPressed, bool controllerCancelPressed) =>
			keyboardCancelPressed || controllerCancelPressed;

		internal static TelemetryMenuAction ResolveTelemetryMenuAction(int selectedIndex) =>
			(TelemetryMenuAction)Math.Clamp(selectedIndex, 0, TelemetryMenuItemCount - 1);

		private void HandlePauseAndVehicleSelectionInput()
		{
			var pad = Input.GamePads.FirstOrDefault();
			var pauseMenuToggleRequested = IsPauseMenuToggleRequested(
				Input.IsKeyPressed(Keys.Escape),
				pad?.IsButtonPressed(GamePadButton.Start) ?? false);
			var vehicleMenuShortcutRequested = IsVehicleMenuShortcutRequested(Input.IsKeyPressed(Keys.F2));

			if (_activeMenuScreen == MenuScreen.GarageSetup)
			{
				if (_car != null)
				{
					_car.PlayerInputEnabled = false;
				}

				return;
			}

			if (_activeMenuScreen == MenuScreen.PhysicsCalibration)
			{
				if (_car != null)
				{
					_car.PlayerInputEnabled = false;
				}

				return;
			}

			if (_activeMenuScreen == MenuScreen.Telemetry)
			{
				if (IsVehicleMenuCancelRequested(
					    Input.IsKeyPressed(Keys.Escape) || pauseMenuToggleRequested,
					    (pad?.IsButtonPressed(GamePadButton.B) ?? false) || (pad?.IsButtonPressed(GamePadButton.Start) ?? false)))
				{
					CloseTelemetryMenu();
				}
				else
				{
					HandleTelemetryMenuInput(pad);
				}

				if (_car != null)
				{
					_car.PlayerInputEnabled = false;
				}

				return;
			}

			if (_activeMenuScreen == MenuScreen.VehicleSelection)
			{
				if (vehicleMenuShortcutRequested)
				{
					CloseVehicleSelection();
				}
				else if (pauseMenuToggleRequested)
				{
					_activeMenuScreen = MenuScreen.Pause;
					_vehicleSelectionOpenedFromPauseMenu = false;
					_pauseMenuSelectedIndex = PauseMenuResumeIndex;
				}
				else
				{
					HandleVehicleSelectionInput(pad);
				}
			}
			else
			{
				if (vehicleMenuShortcutRequested)
				{
					OpenVehicleSelection(fromPauseMenu: _activeMenuScreen == MenuScreen.Pause);
				}
				else if (pauseMenuToggleRequested)
				{
					_activeMenuScreen = ResolvePauseMenuToggle(_activeMenuScreen);
					if (_activeMenuScreen == MenuScreen.Pause)
					{
						_pauseMenuSelectedIndex = PauseMenuResumeIndex;
					}
				}

				if (_activeMenuScreen == MenuScreen.Pause)
				{
					HandlePauseMenuInput(pad);
				}
			}

			if (_car != null)
			{
				_car.PlayerInputEnabled = _activeMenuScreen == MenuScreen.None;
			}
		}

		private void HandleVehicleSelectionInput(IGamePadDevice? pad)
		{
			if (_availableVehicles.Count == 0)
			{
				var closeEmptyMenuRequested = IsVehicleMenuCancelRequested(
					Input.IsKeyPressed(Keys.Escape),
					pad?.IsButtonPressed(GamePadButton.B) ?? false);
				if (closeEmptyMenuRequested)
				{
					CloseVehicleSelection();
				}

				return;
			}

			if (_selectedVehicleIndex < 0)
			{
				_selectedVehicleIndex = 0;
			}

			if (IsVehicleMenuMoveUpRequested(
				    Input.IsKeyPressed(Keys.Up),
				    pad?.IsButtonPressed(GamePadButton.PadUp) ?? false))
			{
				_selectedVehicleIndex = (_selectedVehicleIndex - 1 + _availableVehicles.Count) % _availableVehicles.Count;
			}

			if (IsVehicleMenuMoveDownRequested(
				    Input.IsKeyPressed(Keys.Down),
				    pad?.IsButtonPressed(GamePadButton.PadDown) ?? false))
			{
				_selectedVehicleIndex = (_selectedVehicleIndex + 1) % _availableVehicles.Count;
			}

			if (IsVehicleMenuConfirmRequested(
				    Input.IsKeyPressed(Keys.Enter),
				    pad?.IsButtonPressed(GamePadButton.A) ?? false))
			{
				var selectedVehicle = _availableVehicles[_selectedVehicleIndex];
				_activeMenuScreen = MenuScreen.None;
				_vehicleSelectionOpenedFromPauseMenu = false;
				LoadVehicle(selectedVehicle);
				return;
			}

			if (IsVehicleMenuCancelRequested(
				    Input.IsKeyPressed(Keys.Escape),
				    pad?.IsButtonPressed(GamePadButton.B) ?? false))
			{
				CloseVehicleSelection();
			}
		}

		private void HandlePauseMenuInput(IGamePadDevice? pad)
		{
			if (IsVehicleMenuMoveUpRequested(
				    Input.IsKeyPressed(Keys.Up),
				    pad?.IsButtonPressed(GamePadButton.PadUp) ?? false))
			{
				_pauseMenuSelectedIndex = (_pauseMenuSelectedIndex - 1 + PauseMenuItemCount) % PauseMenuItemCount;
			}

			if (IsVehicleMenuMoveDownRequested(
				    Input.IsKeyPressed(Keys.Down),
				    pad?.IsButtonPressed(GamePadButton.PadDown) ?? false))
			{
				_pauseMenuSelectedIndex = (_pauseMenuSelectedIndex + 1) % PauseMenuItemCount;
			}

			if (!IsVehicleMenuConfirmRequested(
				    Input.IsKeyPressed(Keys.Enter),
				    pad?.IsButtonPressed(GamePadButton.A) ?? false))
			{
				return;
			}

			switch (_pauseMenuSelectedIndex)
			{
				case PauseMenuResumeIndex:
					ExecutePauseMenuAction(PauseMenuAction.ResumeDriving);
					break;
				case PauseMenuResetVehicleIndex:
					ExecutePauseMenuAction(PauseMenuAction.ResetVehicle);
					break;
				case PauseMenuGarageSetupIndex:
					ExecutePauseMenuAction(PauseMenuAction.GarageSetup);
					break;
				case PauseMenuVehicleSelectIndex:
					ExecutePauseMenuAction(PauseMenuAction.VehicleSelect);
					break;
				case PauseMenuPhysicsCalibrationIndex:
					ExecutePauseMenuAction(PauseMenuAction.PhysicsCalibration);
					break;
				case PauseMenuTelemetryIndex:
					ExecutePauseMenuAction(PauseMenuAction.Telemetry);
					break;
			}
		}

		private void HandleTelemetryMenuInput(IGamePadDevice? pad)
		{
			if (IsVehicleMenuMoveUpRequested(
				    Input.IsKeyPressed(Keys.Up),
				    pad?.IsButtonPressed(GamePadButton.PadUp) ?? false))
			{
				_telemetryMenuSelectedIndex = (_telemetryMenuSelectedIndex - 1 + TelemetryMenuItemCount) % TelemetryMenuItemCount;
				RefreshTelemetryOverlay();
			}

			if (IsVehicleMenuMoveDownRequested(
				    Input.IsKeyPressed(Keys.Down),
				    pad?.IsButtonPressed(GamePadButton.PadDown) ?? false))
			{
				_telemetryMenuSelectedIndex = (_telemetryMenuSelectedIndex + 1) % TelemetryMenuItemCount;
				RefreshTelemetryOverlay();
			}

			if (!IsVehicleMenuConfirmRequested(
				    Input.IsKeyPressed(Keys.Enter),
				    pad?.IsButtonPressed(GamePadButton.A) ?? false))
			{
				return;
			}

			ExecuteTelemetryMenuAction(ResolveTelemetryMenuAction(_telemetryMenuSelectedIndex));
		}

		private void ExecutePauseMenuAction(PauseMenuAction action)
		{
			switch (action)
			{
				case PauseMenuAction.ResumeDriving:
					_activeMenuScreen = MenuScreen.None;
					break;
				case PauseMenuAction.ResetVehicle:
					RespawnCurrentVehicle();
					break;
				case PauseMenuAction.GarageSetup:
					OpenGarageSetup();
					break;
				case PauseMenuAction.VehicleSelect:
					OpenVehicleSelection(fromPauseMenu: true);
					break;
				case PauseMenuAction.PhysicsCalibration:
					OpenPhysicsCalibration();
					break;
				case PauseMenuAction.Telemetry:
					OpenTelemetryMenu();
					break;
			}
		}

		private void ExecuteTelemetryMenuAction(TelemetryMenuAction action)
		{
			switch (action)
			{
				case TelemetryMenuAction.ToggleOutGauge:
					SetOutGaugeEnabled(!OutGaugeEnabled);
					SetTelemetryStatus($"OutGauge telemetry {(OutGaugeEnabled ? "enabled" : "disabled")}.");
					break;
				case TelemetryMenuAction.ConnectOutGauge:
					ConnectOutGauge();
					break;
				case TelemetryMenuAction.DisconnectOutGauge:
					DisconnectOutGauge();
					break;
				case TelemetryMenuAction.ReconnectOutGauge:
					ReconnectOutGauge();
					break;
				case TelemetryMenuAction.ToggleOutSim:
					SetOutSimEnabled(!OutSimEnabled);
					SetTelemetryStatus($"OutSim telemetry {(OutSimEnabled ? "enabled" : "disabled")}.");
					break;
				case TelemetryMenuAction.ConnectOutSim:
					ConnectOutSim();
					break;
				case TelemetryMenuAction.DisconnectOutSim:
					DisconnectOutSim();
					break;
				case TelemetryMenuAction.ReconnectOutSim:
					ReconnectOutSim();
					break;
			}
		}

		private void RespawnCurrentVehicle()
		{
			_activeMenuScreen = MenuScreen.None;
			var selectedVehicle = ResolveSelectedVehicleDescriptor();
			if (selectedVehicle != null)
			{
				LoadVehicle(selectedVehicle);
				return;
			}

			LoadVehicle();
		}

		private void OpenVehicleSelection(bool fromPauseMenu)
		{
			_vehicleSelectionOpenedFromPauseMenu = fromPauseMenu;
			_activeMenuScreen = MenuScreen.VehicleSelection;
		}

		private void CloseVehicleSelection()
		{
			_activeMenuScreen = ResolveVehicleSelectionCloseTarget(_vehicleSelectionOpenedFromPauseMenu);
			_vehicleSelectionOpenedFromPauseMenu = false;
			if (_activeMenuScreen == MenuScreen.Pause)
			{
				_pauseMenuSelectedIndex = PauseMenuVehicleSelectIndex;
			}
		}

		private void OpenGarageSetup()
		{
			BindGarageSetupOverlay();
			_activeMenuScreen = MenuScreen.GarageSetup;
		}

		private void CloseGarageSetup()
		{
			_activeMenuScreen = MenuScreen.Pause;
			_pauseMenuSelectedIndex = PauseMenuGarageSetupIndex;
		}

		private void OpenPhysicsCalibration()
		{
			BindPhysicsCalibrationOverlay();
			_activeMenuScreen = MenuScreen.PhysicsCalibration;
		}

		private void ClosePhysicsCalibration()
		{
			_activeMenuScreen = MenuScreen.Pause;
			_pauseMenuSelectedIndex = PauseMenuPhysicsCalibrationIndex;
		}

		private void OpenTelemetryMenu()
		{
			RefreshTelemetryOverlay();
			_activeMenuScreen = MenuScreen.Telemetry;
		}

		private void CloseTelemetryMenu()
		{
			_activeMenuScreen = MenuScreen.Pause;
			_pauseMenuSelectedIndex = PauseMenuTelemetryIndex;
			RefreshTelemetryOverlay();
		}

		private void ApplyGarageSetupChanges(SetupUiApplyPayload payload)
		{
			if (!payload.HasChanges)
			{
				return;
			}

			foreach (var kv in payload.VariableOverrides)
			{
				_setupOverrides.VariableOverrides[kv.Key] = kv.Value;
			}

			foreach (var kv in payload.PressureOverrides)
			{
				if (!_setupOverrides.PressureWheelOverrides.TryGetValue(kv.Key, out var pressureOverride))
				{
					pressureOverride = new VehiclePressureWheelOverrides();
					_setupOverrides.PressureWheelOverrides[kv.Key] = pressureOverride;
				}

				pressureOverride.PressurePsi = kv.Value;
			}

			ApplyLivePressureOverrides(payload.PressureOverrides);
			_status = $"Applied garage setup for {GetCurrentVehicleName()}: {payload.SummaryText}";

			if (!payload.RequiresReload)
			{
				BindGarageSetupOverlay();
				return;
			}

			var selectedVehicle = ResolveSelectedVehicleDescriptor();
			LoadVehicle(selectedVehicle);
		}

		private void ApplyLivePressureOverrides(IReadOnlyDictionary<VehicleSetupAxle, float> pressureOverrides)
		{
			if (_loadedVehicle == null || pressureOverrides.Count == 0)
			{
				return;
			}

			foreach (var (axle, pressurePsi) in pressureOverrides)
			{
				if (!float.IsFinite(pressurePsi))
				{
					continue;
				}

				foreach (var wheel in EnumerateAxleWheels(axle))
				{
					var wheelSettings = wheel.Get<WheelSettings>();
					if (wheelSettings?.TyreModel == null)
					{
						continue;
					}

					wheelSettings.TyreModel.TyrePressure = pressurePsi * PsiToKpa;
				}
			}
		}

		private IEnumerable<Entity> EnumerateAxleWheels(VehicleSetupAxle axle)
		{
			if (_loadedVehicle == null)
			{
				yield break;
			}

			if (axle == VehicleSetupAxle.Front)
			{
				yield return _loadedVehicle.WheelFL;
				yield return _loadedVehicle.WheelFR;
				yield break;
			}

			if (axle == VehicleSetupAxle.Rear)
			{
				yield return _loadedVehicle.WheelRL;
				yield return _loadedVehicle.WheelRR;
			}
		}

		private BeamNgVehicleDescriptor? ResolveSelectedVehicleDescriptor()
		{
			return _selectedVehicleIndex >= 0 && _selectedVehicleIndex < _availableVehicles.Count
				? _availableVehicles[_selectedVehicleIndex]
				: null;
		}

		private string GetCurrentVehicleName()
		{
			if (!string.IsNullOrWhiteSpace(_loadedVehicle?.Definition.VehicleName))
			{
				return _loadedVehicle.Definition.VehicleName;
			}

			return Path.GetFileName(VehicleFolderPath.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
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
				_trackSurfaceMaterialLibrary ??= new TrackSurfaceMaterialLibrary(
					gd,
					Path.Combine(AppContext.BaseDirectory, "Resources", "Track Materials"));
				var segments = new[]
				{
					new TrackSegmentDefinition(
						"test_track_tarmac",
						new Vector3(0f, 0.5f, 0f),
						Quaternion.Identity,
						new Vector3(60f, 0.2f, 16f),
						SurfaceType.Tarmac,
						new Color4(0.21f, 0.21f, 0.22f, 1f),
						7.5f,
						1.35f),
					new TrackSegmentDefinition(
						"test_track_gravel",
						new Vector3(-22f, 0.5f, 26f),
						Quaternion.Identity,
						new Vector3(28f, 0.25f, 14f),
						SurfaceType.Gravel,
						new Color4(0.47f, 0.40f, 0.29f, 1f),
						5f,
						1.05f),
					new TrackSegmentDefinition(
						"test_track_snow",
						new Vector3(22f, 0.5f, 26f),
						Quaternion.Identity,
						new Vector3(28f, 0.25f, 14f),
						SurfaceType.Snow,
						new Color4(0.88f, 0.9f, 0.92f, 1f),
						5f,
						0.85f),
					new TrackSegmentDefinition(
						"test_track_banked",
						new Vector3(38f, 1.8f, -6f),
						Quaternion.RotationZ(BankedSectionRollAngleRadians),
						new Vector3(20f, 0.25f, 28f),
						SurfaceType.Tarmac,
						new Color4(0.24f, 0.24f, 0.25f, 1f),
						4f,
						1.2f),
					new TrackSegmentDefinition(
						"test_track_incline",
						new Vector3(0f, 1.7f, 40f),
						Quaternion.RotationX(InclineSectionPitchAngleRadians),
						new Vector3(12f, 0.25f, 42f),
						SurfaceType.Tarmac,
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
				Log.Warning($"Could not build test track sections: {ex}");
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
				new(new Vector3(halfWidth, 0f, -halfLength), Vector3.UnitY, new Vector2(1f, 0f)),
				new(new Vector3(halfWidth, 0f, halfLength), Vector3.UnitY, new Vector2(1f, 1f)),
				new(new Vector3(-halfWidth, 0f, halfLength), Vector3.UnitY, new Vector2(0f, 1f)),
				new(new Vector3(-halfWidth, 0f, -halfLength), -Vector3.UnitY, new Vector2(0f, 0f)),
				new(new Vector3(halfWidth, 0f, -halfLength), -Vector3.UnitY, new Vector2(1f, 0f)),
				new(new Vector3(halfWidth, 0f, halfLength), -Vector3.UnitY, new Vector2(1f, 1f)),
				new(new Vector3(-halfWidth, 0f, halfLength), -Vector3.UnitY, new Vector2(0f, 1f)),
			};
			var indices = new[] { 0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6 };

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

			var material = _trackSurfaceMaterialLibrary?.CreateMaterial(
				               segment.SurfaceType,
				               uvScale,
				               segment.Albedo,
				               TrackSurfaceGlossiness,
				               TrackSurfaceMetalness)
			               ?? Material.New(graphicsDevice, new MaterialDescriptor
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
			trackEntity.Transform.Position = segment.LocalPosition + new Vector3(0f, TrackSurfaceLift, 0f);
			trackEntity.Transform.Rotation = segment.LocalRotation;
			trackEntity.Add(new StaticComponent
			{
				FrictionCoefficient = segment.FrictionCoefficient,
				Collider = new CompoundCollider
				{
					Colliders =
					{
						new BoxCollider
						{
							Size = segment.ColliderSize,
							PositionLocal = new Vector3(0f, -segment.ColliderSize.Y * 0.5f, 0f),
						},
					},
				},
			});
			trackEntity.Add(new TrackSurfaceComponent { SurfaceType = segment.SurfaceType });
			trackEntity.Add(new ModelComponent { Model = new Model { mesh, material } });

			return trackEntity;
		}

		/// <summary>
		/// Handles per-frame input, telemetry, and debug updates for spawned vehicles.
		/// </summary>
		public override void Update()
		{
			HandlePauseAndVehicleSelectionInput();
			var dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;

			SendOutGaugeTelemetry(dt);
			SendOutSimTelemetry(dt);

			// Toggle debug info with F3
			if (Input.IsKeyPressed(Keys.F3))
			{
				_showDebug = !_showDebug;
			}

			if (_drivingHudOverlay != null)
			{
				_drivingHudOverlay.Car = _car;
				_drivingHudOverlay.StatusText = _status;
				_drivingHudOverlay.DebugOverlayVisible = _showDebug;
			}

			if (_vehicleSelectionOverlay != null)
			{
				_vehicleSelectionOverlay.Vehicles = _availableVehicles;
				_vehicleSelectionOverlay.SelectedIndex = _selectedVehicleIndex;
				_vehicleSelectionOverlay.OverlayVisible = _activeMenuScreen == MenuScreen.VehicleSelection;
				_vehicleSelectionOverlay.StatusText = _status;
			}

			if (_pauseMenuOverlay != null)
			{
				_pauseMenuOverlay.SelectedIndex = _pauseMenuSelectedIndex;
				_pauseMenuOverlay.OverlayVisible = _activeMenuScreen == MenuScreen.Pause;
				_pauseMenuOverlay.VehicleName = GetCurrentVehicleName();
				_pauseMenuOverlay.StatusText = _status;
			}

			if (_telemetryOverlay != null)
			{
				_telemetryOverlay.SelectedIndex = _telemetryMenuSelectedIndex;
				_telemetryOverlay.OverlayVisible = _activeMenuScreen == MenuScreen.Telemetry;
				_telemetryOverlay.VehicleName = GetCurrentVehicleName();
				_telemetryOverlay.StatusText = _status;
				_telemetryOverlay.OutGaugeSummary = BuildOutGaugeSummary();
				_telemetryOverlay.OutSimSummary = BuildOutSimSummary();
			}

			if (_setupUiShellOverlay != null)
			{
				_setupUiShellOverlay.OverlayVisible = _activeMenuScreen == MenuScreen.GarageSetup;
				_setupUiShellOverlay.VehicleName = GetCurrentVehicleName();
				_setupUiShellOverlay.StatusText = _status;
			}

			if (_physicsCalibrationOverlay != null)
			{
				_physicsCalibrationOverlay.OverlayVisible = _activeMenuScreen == MenuScreen.PhysicsCalibration;
				_physicsCalibrationOverlay.VehicleName = GetCurrentVehicleName();
				_physicsCalibrationOverlay.StatusText = _status;
			}

			if (_car == null)
			{
				DebugText.Print(_status, new Int2(10, 10));
			}
		}

		private void SendOutGaugeTelemetry(float deltaTime)
		{
			if (!OutGaugeEnabled || !_outGaugeConnectionRequested || _car == null)
			{
				_outGaugeElapsed = 0f;
				if (!OutGaugeEnabled || !_outGaugeConnectionRequested)
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

		private void SendOutSimTelemetry(float deltaTime)
		{
			if (!OutSimEnabled || !_outSimConnectionRequested || _car == null)
			{
				_outSimElapsed = 0f;
				_outSimHasPreviousLinearVelocity = false;
				if (!OutSimEnabled || !_outSimConnectionRequested)
				{
					DisposeOutSimClient();
				}
				return;
			}

			var sendIntervalSeconds = Math.Max(0, OutSimDelayCentiseconds) * 0.01f;
			_outSimElapsed += Math.Max(0f, deltaTime);
			if (sendIntervalSeconds > 0f && _outSimElapsed < sendIntervalSeconds)
			{
				return;
			}

			var sampleDelta = Math.Max(_outSimElapsed, 1e-4f);
			_outSimElapsed = 0f;
			EnsureOutSimClient();
			if (_outSimClient == null || string.IsNullOrWhiteSpace(_outSimTargetHost))
			{
				return;
			}

			var chassisBody = _car.CarBody.Get<BodyComponent>();
			if (chassisBody == null)
			{
				_outSimHasPreviousLinearVelocity = false;
				return;
			}

			var linearVelocity = chassisBody.LinearVelocity;
			var acceleration = _outSimHasPreviousLinearVelocity
				? (linearVelocity - _outSimPreviousLinearVelocity) / sampleDelta
				: Vector3.Zero;
			_outSimPreviousLinearVelocity = linearVelocity;
			_outSimHasPreviousLinearVelocity = true;

			try
			{
				var sessionMilliseconds = Math.Max(0d, Game.UpdateTime.Total.TotalMilliseconds);
				var snapshot = OutSimProtocol.FromCar(_car, unchecked((uint)sessionMilliseconds), acceleration);
				var payload = OutSimProtocol.Encode(snapshot, OutSimId);
				_outSimClient.Send(payload, payload.Length);
				_outSimSendFailed = false;
			}
			catch (SocketException ex)
			{
				HandleOutSimTelemetryFailure(ex);
			}
			catch (ObjectDisposedException ex)
			{
				HandleOutSimTelemetryFailure(ex);
			}
			catch (InvalidOperationException ex)
			{
				HandleOutSimTelemetryFailure(ex);
			}
		}

		private void EnsureOutSimClient()
		{
			var targetHost = OutSimIp?.Trim();
			var targetPort = OutSimPort;
			if (string.IsNullOrWhiteSpace(targetHost) || targetPort is < 1 or > 65535)
			{
				DisposeOutSimClient();
				return;
			}

			if (_outSimClient != null &&
			    string.Equals(_outSimTargetHost, targetHost, StringComparison.OrdinalIgnoreCase) &&
			    _outSimTargetPort == targetPort)
			{
				return;
			}

			DisposeOutSimClient();
			_outSimClient = new UdpClient();
			_outSimTargetHost = targetHost;
			_outSimTargetPort = targetPort;
			try
			{
				_outSimClient.Connect(targetHost, targetPort);
			}
			catch (SocketException ex)
			{
				HandleOutSimTelemetryFailure(ex);
				DisposeOutSimClient();
				return;
			}
			_outSimSendFailed = false;
			_outSimNextFailureLogTimeSeconds = 0d;
		}

		private void DisposeOutSimClient()
		{
			_outSimClient?.Dispose();
			_outSimClient = null;
			_outSimTargetHost = null;
			_outSimTargetPort = 0;
			_outSimSendFailed = false;
			_outSimNextFailureLogTimeSeconds = 0d;
			_outSimHasPreviousLinearVelocity = false;
		}

		private void HandleOutSimTelemetryFailure(Exception ex)
		{
			var sessionSeconds = Math.Max(0d, Game.UpdateTime.Total.TotalSeconds);
			if (!_outSimSendFailed || sessionSeconds >= _outSimNextFailureLogTimeSeconds)
			{
				Log.Warning($"OutSim telemetry failed ({_outSimTargetHost}:{_outSimTargetPort}): {ex.Message}");
				_outSimNextFailureLogTimeSeconds = sessionSeconds + FailureLogIntervalSeconds;
			}

			_outSimSendFailed = true;
		}

		private IReadOnlyList<PauseMenuItem> BuildTelemetryMenuItems() =>
		[
			new(OutGaugeEnabled ? "Disable OutGauge" : "Enable OutGauge", "Toggle Live for Speed OutGauge speed/RPM packets on or off."),
			new("Connect OutGauge", "Open the OutGauge UDP socket without changing the enable toggle."),
			new("Disconnect OutGauge", "Close the OutGauge UDP socket while keeping the configured target intact."),
			new("Reconnect OutGauge", "Dispose and recreate the OutGauge UDP socket for the current endpoint."),
			new(OutSimEnabled ? "Disable OutSim" : "Enable OutSim", "Toggle Live for Speed OutSim motion packets on or off."),
			new("Connect OutSim", "Open the OutSim UDP socket without changing the enable toggle."),
			new("Disconnect OutSim", "Close the OutSim UDP socket while keeping the configured target intact."),
			new("Reconnect OutSim", "Dispose and recreate the OutSim UDP socket for the current endpoint."),
		];

		private void RefreshTelemetryOverlay()
		{
			if (_telemetryOverlay == null)
			{
				return;
			}

			_telemetryOverlay.Items = BuildTelemetryMenuItems();
			_telemetryOverlay.SelectedIndex = _telemetryMenuSelectedIndex;
			_telemetryOverlay.VehicleName = GetCurrentVehicleName();
			_telemetryOverlay.StatusText = _status;
			_telemetryOverlay.OutGaugeSummary = BuildOutGaugeSummary();
			_telemetryOverlay.OutSimSummary = BuildOutSimSummary();
		}

		private string BuildOutGaugeSummary()
		{
			var endpoint = DescribeTelemetryEndpoint(OutGaugeIp, OutGaugePort);
			var enabledState = OutGaugeEnabled ? "enabled" : "disabled";
			var connectionState = DescribeSocketState(_outGaugeConnectionRequested, _outGaugeClient != null, _outGaugeSendFailed);
			return $"OutGauge // {enabledState} // {endpoint} // {connectionState} // {(OutGaugeId == 0 ? "id=none" : $"id={OutGaugeId}")}";
		}

		private string BuildOutSimSummary()
		{
			var endpoint = DescribeTelemetryEndpoint(OutSimIp, OutSimPort);
			var enabledState = OutSimEnabled ? "enabled" : "disabled";
			var connectionState = DescribeSocketState(_outSimConnectionRequested, _outSimClient != null, _outSimSendFailed);
			return $"OutSim // {enabledState} // {endpoint} // {connectionState} // {(OutSimId == 0 ? "id=none" : $"id={OutSimId}")}";
		}

		private static string DescribeTelemetryEndpoint(string? host, int port)
		{
			var trimmedHost = host?.Trim();
			return string.IsNullOrWhiteSpace(trimmedHost) || port is < 1 or > 65535
				? "endpoint not configured"
				: $"{trimmedHost}:{port}";
		}

		private static string DescribeSocketState(bool connectionRequested, bool socketOpen, bool sendFailed)
		{
			if (!connectionRequested)
			{
				return "socket disconnected";
			}

			if (!socketOpen)
			{
				return "socket closed";
			}

			return sendFailed ? "socket open, last send failed" : "socket open";
		}

		private void SetOutGaugeEnabled(bool enabled)
		{
			OutGaugeEnabled = enabled;
			_outGaugeElapsed = 0f;
			if (!enabled)
			{
				DisposeOutGaugeClient();
				return;
			}

			if (_outGaugeConnectionRequested)
			{
				EnsureOutGaugeClient();
			}
		}

		private void SetOutSimEnabled(bool enabled)
		{
			OutSimEnabled = enabled;
			_outSimElapsed = 0f;
			_outSimHasPreviousLinearVelocity = false;
			if (!enabled)
			{
				DisposeOutSimClient();
				return;
			}

			if (_outSimConnectionRequested)
			{
				EnsureOutSimClient();
			}
		}

		private void ConnectOutGauge()
		{
			_outGaugeConnectionRequested = true;
			EnsureOutGaugeClient();
			SetTelemetryStatus($"OutGauge socket connect requested ({DescribeSocketState(_outGaugeConnectionRequested, _outGaugeClient != null, _outGaugeSendFailed)}).");
		}

		private void DisconnectOutGauge()
		{
			_outGaugeConnectionRequested = false;
			DisposeOutGaugeClient();
			SetTelemetryStatus("OutGauge socket disconnected.");
		}

		private void ReconnectOutGauge()
		{
			_outGaugeConnectionRequested = true;
			DisposeOutGaugeClient();
			EnsureOutGaugeClient();
			SetTelemetryStatus($"OutGauge socket reconnected ({DescribeSocketState(_outGaugeConnectionRequested, _outGaugeClient != null, _outGaugeSendFailed)}).");
		}

		private void ConnectOutSim()
		{
			_outSimConnectionRequested = true;
			EnsureOutSimClient();
			SetTelemetryStatus($"OutSim socket connect requested ({DescribeSocketState(_outSimConnectionRequested, _outSimClient != null, _outSimSendFailed)}).");
		}

		private void DisconnectOutSim()
		{
			_outSimConnectionRequested = false;
			DisposeOutSimClient();
			SetTelemetryStatus("OutSim socket disconnected.");
		}

		private void ReconnectOutSim()
		{
			_outSimConnectionRequested = true;
			DisposeOutSimClient();
			EnsureOutSimClient();
			SetTelemetryStatus($"OutSim socket reconnected ({DescribeSocketState(_outSimConnectionRequested, _outSimClient != null, _outSimSendFailed)}).");
		}

		private void SetTelemetryStatus(string message)
		{
			_status = $"Telemetry: {message}";
			RefreshTelemetryOverlay();
		}
	}
}
