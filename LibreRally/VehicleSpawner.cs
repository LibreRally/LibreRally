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
using Stride.Games;
using Stride.Input;

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

		/// <summary>Open the track selection menu.</summary>
		TrackSelect,

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
	/// Represents a playable track or world entry available in the track-selection menu.
	/// </summary>
	/// <param name="Item">The UI descriptor shown in the overlay.</param>
	/// <param name="Selection">The startup scene selection to load when chosen.</param>
	internal readonly record struct TrackSelectionEntry(PauseMenuItem Item, StartupSceneSelection Selection);

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
		CyclePhysicsTickRate,
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
		/// Gets or sets the entity name of the optional geospatial spawn anchor.
		/// </summary>
		internal string? SpawnAnchorEntityName { get; set; }

		/// <summary>
		/// Gets or sets the startup scene selection that should be preserved for track switching.
		/// </summary>
		internal StartupSceneSelection StartupSelection { get; set; } =
			new(StartupSceneMode.Default, CesiumStageBootstrapConfiguration.CreateSampleStage());

		/// <summary>
		/// Gets or sets a value indicating whether OutGauge telemetry is enabled.
		/// </summary>
		public bool OutGaugeEnabled
		{
			get => _telemetrySession.OutGaugeEnabled;
			set => _telemetrySession.OutGaugeEnabled = value;
		}

		/// <summary>
		/// Gets or sets the delay between OutGauge packets in centiseconds.
		/// </summary>
		public int OutGaugeDelayCentiseconds
		{
			get => _telemetrySession.OutGaugeDelayCentiseconds;
			set => _telemetrySession.OutGaugeDelayCentiseconds = value;
		}

		/// <summary>
		/// Gets or sets the target IP address for OutGauge telemetry.
		/// </summary>
		public string OutGaugeIp
		{
			get => _telemetrySession.OutGaugeIp;
			set => _telemetrySession.OutGaugeIp = value;
		}

		/// <summary>
		/// Gets or sets the target port for OutGauge telemetry.
		/// </summary>
		public int OutGaugePort
		{
			get => _telemetrySession.OutGaugePort;
			set => _telemetrySession.OutGaugePort = value;
		}

		/// <summary>
		/// Gets or sets the unique ID for OutGauge telemetry.
		/// </summary>
		public int OutGaugeId
		{
			get => _telemetrySession.OutGaugeId;
			set => _telemetrySession.OutGaugeId = value;
		}

		/// <summary>
		/// Gets or sets a value indicating whether OutSim telemetry is enabled.
		/// </summary>
		public bool OutSimEnabled
		{
			get => _telemetrySession.OutSimEnabled;
			set => _telemetrySession.OutSimEnabled = value;
		}

		/// <summary>
		/// Gets or sets the delay between OutSim packets in centiseconds.
		/// </summary>
		public int OutSimDelayCentiseconds
		{
			get => _telemetrySession.OutSimDelayCentiseconds;
			set => _telemetrySession.OutSimDelayCentiseconds = value;
		}

		/// <summary>
		/// Gets or sets the target IP address for OutSim telemetry.
		/// </summary>
		public string OutSimIp
		{
			get => _telemetrySession.OutSimIp;
			set => _telemetrySession.OutSimIp = value;
		}

		/// <summary>
		/// Gets or sets the target port for OutSim telemetry.
		/// </summary>
		public int OutSimPort
		{
			get => _telemetrySession.OutSimPort;
			set => _telemetrySession.OutSimPort = value;
		}

		/// <summary>
		/// Gets or sets the unique ID for OutSim telemetry.
		/// </summary>
		public int OutSimId
		{
			get => _telemetrySession.OutSimId;
			set => _telemetrySession.OutSimId = value;
		}

		/// <summary>
		/// Gets or sets a value indicating whether the localhost live tuning bridge is enabled.
		/// </summary>
		public bool LiveTuningBridgeEnabled { get; set; } = true;

		/// <summary>
		/// Gets or sets the localhost TCP port used by the live tuning bridge.
		/// </summary>
		public int LiveTuningBridgePort { get; set; } = 18765;

		private string _status = "Loading...";
		private bool _showDebug = true;
		private DrivingHudOverlay? _drivingHudOverlay;
		private PauseMenuOverlay? _pauseMenuOverlay;
		private SetupUiShellOverlay? _setupUiShellOverlay;
		private VehicleSelectionOverlay? _vehicleSelectionOverlay;
		private TrackSelectionOverlay? _trackSelectionOverlay;
		private PhysicsCalibrationOverlay? _physicsCalibrationOverlay;
		private TelemetryOverlay? _telemetryOverlay;
		private LoadingOverlay? _loadingOverlay;
		private readonly VehicleSetupOverrides _setupOverrides = new();
		private readonly VehicleTelemetrySession _telemetrySession = new();
		private readonly VehicleSpawnerTrackBuilder _trackBuilder = new();
		private readonly List<HitInfo> _spawnSupportHits = [];
		private VehicleSpawnerVehicleSession? _vehicleSession;
		private VehicleLiveTuningBridgeServer? _liveTuningBridgeServer;
		private PendingVehicleLoadRequest? _pendingVehicleLoad;
		private Entity? _spawnFallbackSupportEntity;
		private float _pendingVehicleLoadWaitSeconds;
		private int _selectedVehicleIndex;
		private int _selectedTrackIndex;
		private int _pauseMenuSelectedIndex;
		private int _telemetryMenuSelectedIndex;
		private bool _vehicleSelectionOpenedFromPauseMenu;
		private StartupSceneSelection? _pendingSceneSelection;
		private MenuScreen _activeMenuScreen;
		private const int PauseMenuResumeIndex = 0;
		private const int PauseMenuResetVehicleIndex = 1;
		private const int PauseMenuGarageSetupIndex = 2;
		private const int PauseMenuVehicleSelectIndex = 3;
		private const int PauseMenuTrackSelectIndex = 4;
		private const int PauseMenuPhysicsCalibrationIndex = 5;
		private const int PauseMenuTelemetryIndex = 6;
		private const int PauseMenuItemCount = 7;
		private const int TelemetryMenuItemCount = 9;
		private const float SpawnSupportProbeHeight = 5f;
		private const float SpawnSupportProbeLength = 48f;
		private const float SpawnSupportSurfaceClearance = 2.5f;
		private const float SpawnSupportFallbackDelaySeconds = 8f;
		private const float SpawnSupportFallbackTopOffset = 2.5f;
		private const float SpawnLoadingCameraHeight = 96f;
		private const float SpawnLoadingCameraBackOffset = 28f;
		private static readonly Vector3 SpawnSupportFallbackPadSize = new(8f, 1f, 8f);

		private static readonly IReadOnlyList<PauseMenuEntry> PauseMenuEntries =
		[
			new(new PauseMenuItem("Resume Driving", "Return to the stage and hand control back to the driver."), PauseMenuAction.ResumeDriving),
			new(new PauseMenuItem("Reset Vehicle", "Respawn the current car back at the spawn point."), PauseMenuAction.ResetVehicle),
			new(new PauseMenuItem("Garage Setup", "Open the restored Myra tuning shell for stage prep changes."), PauseMenuAction.GarageSetup),
			new(new PauseMenuItem("Vehicle Select", "Load a different bundled vehicle from the current catalog."), PauseMenuAction.VehicleSelect),
			new(new PauseMenuItem("Track Select", "Switch between the local test track and the available Cesium track presets."), PauseMenuAction.TrackSelect),
			new(new PauseMenuItem("Physics Calibration", "Isolate tyre model contributions and tune anti-slip parameters live."), PauseMenuAction.PhysicsCalibration),
			new(new PauseMenuItem("Telemetry", "Enable or disable OutGauge/OutSim, then connect, disconnect, or reconnect their UDP sockets."), PauseMenuAction.Telemetry),
		];

		internal enum MenuScreen
		{
			None,
			Pause,
			GarageSetup,
			VehicleSelection,
			TrackSelection,
			PhysicsCalibration,
			Telemetry,
		}

		private sealed record PendingVehicleLoadRequest(
			BeamNgVehicleVariantDescriptor? SelectedVehicle,
			IProgress<VehicleLoadProgress>? Progress);

		/// <summary>
		/// Initializes the track, overlays, and vehicle catalog at scene startup.
		/// </summary>
		public override void Start()
		{
			_vehicleSession = new VehicleSpawnerVehicleSession((Game)Game, SceneSystem, _setupOverrides);
			AddGroundPhysics();
			EnsureDrivingHudOverlay();
			InitializeVehicleCatalog();
			_selectedTrackIndex = ResolveCurrentTrackIndex();
			EnsurePauseMenuOverlay();
			EnsureSetupUiShellOverlay();
			EnsureVehicleSelectionOverlay();
			EnsureTrackSelectionOverlay();
			EnsurePhysicsCalibrationOverlay();
			EnsureTelemetryOverlay();
			InitializeLiveTuningBridge();
			EnsureLoadingOverlay();
			if (_loadingOverlay != null)
			{
				_loadingOverlay.Visible = true;
				_loadingOverlay.Enabled = true;
			}

			try
			{
				var progress = new Progress<VehicleLoadProgress>(p =>
				{
					_status = p.Stage;
					if (_loadingOverlay != null)
					{
						_loadingOverlay.StatusText = p.Stage;
						_loadingOverlay.TitleText = $"Loading Vehicle... ({p.Progress * 100f:F0}%)";
						_loadingOverlay.SetProgress(p.Progress);
					}
				});
				RequestVehicleLoad(progress: progress);
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
			finally
			{
				if (_loadingOverlay != null && _pendingVehicleLoad == null)
				{
					_loadingOverlay.Visible = false;
					_loadingOverlay.Enabled = false;
				}
			}
		}

		/// <summary>
		/// Releases transient overlays and live systems when the scene is torn down.
		/// </summary>
		public override void Cancel()
		{
			DisposeRuntimeSystems();
			base.Cancel();
		}

		private void RequestVehicleLoad(BeamNgVehicleVariantDescriptor? selectedVehicle = null, IProgress<VehicleLoadProgress>? progress = null)
		{
			if (ShouldWaitForSpawnSupport(StartupSelection.SceneMode, SpawnAnchorEntityName) && !IsSpawnSupportReady())
			{
				_pendingVehicleLoad = new PendingVehicleLoadRequest(selectedVehicle, progress);
				_pendingVehicleLoadWaitSeconds = 0f;
				UnloadVehicle();
				PrepareSpawnLoadingCamera();
				_status = BuildSpawnWaitingStatus(GetCurrentTrackName());
				ShowLoadingOverlay(_status, progress: 0f);
				return;
			}

			_pendingVehicleLoad = null;
			_pendingVehicleLoadWaitSeconds = 0f;
			LoadVehicleNow(selectedVehicle, progress);
		}

		private void LoadVehicleNow(BeamNgVehicleVariantDescriptor? selectedVehicle = null, IProgress<VehicleLoadProgress>? progress = null)
		{
			if (_vehicleSession == null)
			{
				throw new InvalidOperationException("Vehicle session was not initialized.");
			}

			var spawnPosition = ResolveSpawnPosition();
			var loadResult = _vehicleSession.LoadVehicle(
				VehicleFolderPath,
				ConfigFileName,
				spawnPosition,
				_selectedVehicleIndex,
				selectedVehicle,
				progress);
			VehicleFolderPath = loadResult.VehicleFolderPath;
			ConfigFileName = loadResult.ConfigFileName;
			_selectedVehicleIndex = loadResult.SelectedVehicleIndex;
			_status = loadResult.StatusText;

			AttachCamera(loadResult.LoadedVehicle.ChassisEntity, loadResult.LoadedVehicle.CarComponent);
			AttachDrivingHud(loadResult.LoadedVehicle.CarComponent);
			BindGarageSetupOverlay();
			BindPhysicsCalibrationOverlay();
			RefreshTelemetryOverlay();
		}

		private void UnloadVehicle()
		{
			_vehicleSession?.UnloadVehicle();
		}

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
		}

		private void EnsureDrivingHudOverlay()
		{
			if (_drivingHudOverlay != null)
			{
				return;
			}

			_drivingHudOverlay = new DrivingHudOverlay(Services) { Car = _vehicleSession?.Car, StatusText = _status, DebugOverlayVisible = _showDebug, };

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
				Vehicles = GetAvailableVehicles(), SelectedIndex = _selectedVehicleIndex, OverlayVisible = _activeMenuScreen == MenuScreen.VehicleSelection, StatusText = _status,
			};
			_vehicleSelectionOverlay.ItemActivated = selectedIndex =>
			{
				var vehicles = GetAvailableVehicles();
				if (selectedIndex < 0 || selectedIndex >= vehicles.Count)
				{
					return;
				}

				_selectedVehicleIndex = selectedIndex;
				_activeMenuScreen = MenuScreen.None;
				_vehicleSelectionOpenedFromPauseMenu = false;
				RequestVehicleLoad(vehicles[selectedIndex]);
			};

			((Game)Game).GameSystems.Add(_vehicleSelectionOverlay);
		}

		private void EnsureTrackSelectionOverlay()
		{
			if (_trackSelectionOverlay != null)
			{
				return;
			}

			_trackSelectionOverlay = new TrackSelectionOverlay(Services)
			{
				Items = GetAvailableTrackItems(),
				SelectedIndex = _selectedTrackIndex,
				OverlayVisible = _activeMenuScreen == MenuScreen.TrackSelection,
				CurrentTrackName = GetCurrentTrackName(),
				StatusText = _status,
			};
			_trackSelectionOverlay.ItemActivated = selectedIndex =>
			{
				var tracks = GetAvailableTrackSelections();
				if (selectedIndex < 0 || selectedIndex >= tracks.Count)
				{
					return;
				}

				QueueTrackSelection(tracks[selectedIndex].Selection);
			};

			((Game)Game).GameSystems.Add(_trackSelectionOverlay);
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

			_setupUiShellOverlay = new SetupUiShellOverlay(Services) { OverlayVisible = _activeMenuScreen == MenuScreen.GarageSetup, VehicleName = GetCurrentVehicleName(), StatusText = _status, };
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

			_physicsCalibrationOverlay = new PhysicsCalibrationOverlay(Services) { OverlayVisible = _activeMenuScreen == MenuScreen.PhysicsCalibration, VehicleName = GetCurrentVehicleName(), StatusText = _status, };
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

		private void EnsureLoadingOverlay()
		{
			if (_loadingOverlay != null) return;
			_loadingOverlay = new LoadingOverlay(Services)
			{
				Visible = false,
				Enabled = false,
			};
			((Game)Game).GameSystems.Add(_loadingOverlay);
		}

		private void BindPhysicsCalibrationOverlay()
		{
			if (_physicsCalibrationOverlay == null)
			{
				return;
			}

			_physicsCalibrationOverlay.BindVehicle(_vehicleSession?.LoadedVehicle?.CarComponent);
		}

		private void BindGarageSetupOverlay()
		{
			if (_setupUiShellOverlay == null)
			{
				return;
			}

			_setupUiShellOverlay.BindVehicle(_vehicleSession?.LoadedVehicle, _setupOverrides, _status);
		}

		private void InitializeVehicleCatalog()
		{
			if (_vehicleSession == null)
			{
				throw new InvalidOperationException("Vehicle session was not initialized.");
			}

			_selectedVehicleIndex = _vehicleSession.InitializeCatalog(VehicleFolderPath, ConfigFileName);
		}

		private IReadOnlyList<TrackSelectionEntry> GetAvailableTrackSelections()
		{
			var tracks = new List<TrackSelectionEntry>
			{
				new(
					new PauseMenuItem(
						"Local Test Track",
						"Load the existing procedural test environment with the familiar local ground sections."),
					StartupSelection with { SceneMode = StartupSceneMode.Default }),
			};
			foreach (var stage in CesiumStageBootstrapConfiguration.BuildTrackSelectionStages(StartupSelection.CesiumStage))
			{
				tracks.Add(new TrackSelectionEntry(
					new PauseMenuItem(
						stage.DisplayName,
						$"Load {stage.Description}"),
					new StartupSceneSelection(StartupSceneMode.Cesium, stage)));
			}

			return tracks;
		}

		private IReadOnlyList<PauseMenuItem> GetAvailableTrackItems() =>
			GetAvailableTrackSelections().Select(entry => entry.Item).ToArray();

		private int ResolveCurrentTrackIndex()
		{
			var tracks = GetAvailableTrackSelections();
			for (var index = 0; index < tracks.Count; index++)
			{
				if (AreEquivalentTrackSelections(tracks[index].Selection, StartupSelection))
				{
					return index;
				}
			}

			return 0;
		}

		private string GetCurrentTrackName()
		{
			var tracks = GetAvailableTrackSelections();
			for (var index = 0; index < tracks.Count; index++)
			{
				if (AreEquivalentTrackSelections(tracks[index].Selection, StartupSelection))
				{
					return tracks[index].Item.Title;
				}
			}

			return ResolveTrackSelectionLabel(StartupSelection);
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

		internal static bool AreEquivalentTrackSelections(StartupSceneSelection left, StartupSceneSelection right) =>
			left.SceneMode == right.SceneMode
			// Default mode always maps to the single local test track regardless of any stale Cesium stage value.
			&& (left.SceneMode != StartupSceneMode.Cesium
			    || CesiumStageBootstrapConfiguration.MatchesTrackIdentity(left.CesiumStage, right.CesiumStage));

		internal static MenuScreen ResolveVehicleSelectionCloseTarget(bool openedFromPauseMenu) =>
			openedFromPauseMenu ? MenuScreen.Pause : MenuScreen.None;

		internal static MenuScreen ResolveTrackSelectionCloseTarget() => MenuScreen.Pause;

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
				if (_vehicleSession?.Car != null)
				{
					_vehicleSession.Car.PlayerInputEnabled = false;
				}

				return;
			}

			if (_activeMenuScreen == MenuScreen.PhysicsCalibration)
			{
				if (_vehicleSession?.Car != null)
				{
					_vehicleSession.Car.PlayerInputEnabled = false;
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

				if (_vehicleSession?.Car != null)
				{
					_vehicleSession.Car.PlayerInputEnabled = false;
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
			else if (_activeMenuScreen == MenuScreen.TrackSelection)
			{
				if (pauseMenuToggleRequested)
				{
					CloseTrackSelection();
				}
				else
				{
					HandleTrackSelectionInput(pad);
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

			if (_vehicleSession?.Car != null)
			{
				_vehicleSession.Car.PlayerInputEnabled = _activeMenuScreen == MenuScreen.None;
			}
		}

		private void HandleVehicleSelectionInput(IGamePadDevice? pad)
		{
			var vehicles = GetAvailableVehicles();
			if (vehicles.Count == 0)
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
				_selectedVehicleIndex = (_selectedVehicleIndex - 1 + vehicles.Count) % vehicles.Count;
			}

			if (IsVehicleMenuMoveDownRequested(
				    Input.IsKeyPressed(Keys.Down),
				    pad?.IsButtonPressed(GamePadButton.PadDown) ?? false))
			{
				_selectedVehicleIndex = (_selectedVehicleIndex + 1) % vehicles.Count;
			}

			if (IsVehicleMenuConfirmRequested(
				    Input.IsKeyPressed(Keys.Enter),
				    pad?.IsButtonPressed(GamePadButton.A) ?? false))
			{
				var selectedVehicle = vehicles[_selectedVehicleIndex];
				_activeMenuScreen = MenuScreen.None;
				_vehicleSelectionOpenedFromPauseMenu = false;
				RequestVehicleLoad(selectedVehicle);
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
				case PauseMenuTrackSelectIndex:
					ExecutePauseMenuAction(PauseMenuAction.TrackSelect);
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
				case PauseMenuAction.TrackSelect:
					OpenTrackSelection();
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
				case TelemetryMenuAction.CyclePhysicsTickRate:
					var label = RallyCarComponent.CyclePhysicsTickRate();
					SetTelemetryStatus($"Physics tick rate: {label}");
					RefreshTelemetryOverlay();
					break;
			}
		}

		private void RespawnCurrentVehicle()
		{
			_activeMenuScreen = MenuScreen.None;
			var selectedVehicle = ResolveSelectedVehicleDescriptor();
			if (selectedVehicle != null)
			{
				RequestVehicleLoad(selectedVehicle);
				return;
			}

			RequestVehicleLoad();
		}

		private void HandleTrackSelectionInput(IGamePadDevice? pad)
		{
			var tracks = GetAvailableTrackSelections();
			if (tracks.Count == 0)
			{
				if (IsVehicleMenuCancelRequested(
					    Input.IsKeyPressed(Keys.Escape),
					    pad?.IsButtonPressed(GamePadButton.B) ?? false))
				{
					CloseTrackSelection();
				}

				return;
			}

			_selectedTrackIndex = Math.Clamp(_selectedTrackIndex, 0, tracks.Count - 1);
			if (IsVehicleMenuMoveUpRequested(
				    Input.IsKeyPressed(Keys.Up),
				    pad?.IsButtonPressed(GamePadButton.PadUp) ?? false))
			{
				_selectedTrackIndex = (_selectedTrackIndex - 1 + tracks.Count) % tracks.Count;
			}

			if (IsVehicleMenuMoveDownRequested(
				    Input.IsKeyPressed(Keys.Down),
				    pad?.IsButtonPressed(GamePadButton.PadDown) ?? false))
			{
				_selectedTrackIndex = (_selectedTrackIndex + 1) % tracks.Count;
			}

			if (IsVehicleMenuConfirmRequested(
				    Input.IsKeyPressed(Keys.Enter),
				    pad?.IsButtonPressed(GamePadButton.A) ?? false))
			{
				QueueTrackSelection(tracks[_selectedTrackIndex].Selection);
				return;
			}

			if (IsVehicleMenuCancelRequested(
				    Input.IsKeyPressed(Keys.Escape),
				    pad?.IsButtonPressed(GamePadButton.B) ?? false))
			{
				CloseTrackSelection();
			}
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

		private void OpenTrackSelection()
		{
			_selectedTrackIndex = ResolveCurrentTrackIndex();
			_activeMenuScreen = MenuScreen.TrackSelection;
		}

		private void CloseTrackSelection()
		{
			_activeMenuScreen = ResolveTrackSelectionCloseTarget();
			_pauseMenuSelectedIndex = PauseMenuTrackSelectIndex;
		}

		private void QueueTrackSelection(StartupSceneSelection selection)
		{
			if (AreEquivalentTrackSelections(selection, StartupSelection))
			{
				CloseTrackSelection();
				return;
			}

			_pendingSceneSelection = selection;
			_activeMenuScreen = MenuScreen.None;
			_status = $"Loading {ResolveTrackSelectionLabel(selection)}...";
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

			_vehicleSession?.ApplyLivePressureOverrides(payload.PressureOverrides);
			_status = $"Applied garage setup for {GetCurrentVehicleName()}: {payload.SummaryText}";

			if (!payload.RequiresReload)
			{
				BindGarageSetupOverlay();
				return;
			}

			var selectedVehicle = ResolveSelectedVehicleDescriptor();
			RequestVehicleLoad(selectedVehicle);
		}

		private BeamNgVehicleVariantDescriptor? ResolveSelectedVehicleDescriptor()
		{
			return _vehicleSession?.ResolveSelectedVehicleDescriptor(_selectedVehicleIndex);
		}

		private string GetCurrentVehicleName()
		{
			return _vehicleSession?.GetCurrentVehicleName(VehicleFolderPath)
			       ?? Path.GetFileName(VehicleFolderPath.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
		}

		private IReadOnlyList<BeamNgVehicleVariantDescriptor> GetAvailableVehicles() =>
			_vehicleSession?.AvailableVehicles ?? Array.Empty<BeamNgVehicleVariantDescriptor>();

		private void AttachCamera(Entity chassis, RallyCarComponent car)
		{
			var cameraEntity = ResolveCameraEntity();
			if (cameraEntity == null)
			{
				return;
			}

			RemoveCameraScripts(cameraEntity);
			cameraEntity.Add(new RallyCameraScript { Target = chassis, CarComponent = car, });
		}

		private void AddGroundPhysics()
		{
			_trackBuilder.ConfigureGround(Entity, (Game)Game, StartupSelection.SceneMode == StartupSceneMode.Default);
		}

		internal static bool ShouldWaitForSpawnSupport(StartupSceneMode sceneMode, string? spawnAnchorEntityName) =>
			sceneMode == StartupSceneMode.Cesium && !string.IsNullOrWhiteSpace(spawnAnchorEntityName);

		internal static string BuildSpawnWaitingStatus(string trackName) =>
			$"Waiting for {trackName} collision near the spawn point...";

		private Vector3 ResolveSpawnPosition()
		{
			if (TryResolveSpawnSupportPoint(out var supportPoint))
			{
				return BuildSpawnPositionFromSupportPoint(supportPoint, SpawnSupportSurfaceClearance);
			}

			if (TryResolveSpawnAnchorPosition(out var anchorPosition))
			{
				return anchorPosition;
			}

			return SpawnPosition;
		}

		private bool TryResolveSpawnAnchorPosition(out Vector3 anchorPosition)
		{
			anchorPosition = Vector3.Zero;
			if (string.IsNullOrWhiteSpace(SpawnAnchorEntityName) || SceneSystem.SceneInstance.RootScene == null)
			{
				return false;
			}

			var rootScene = SceneSystem.SceneInstance.RootScene;
			var anchorEntity = FindEntityByName(rootScene.Entities, SpawnAnchorEntityName);
			if (anchorEntity == null)
			{
				return false;
			}

			var anchor = anchorEntity.Get<CesiumGlobeAnchorComponent>();
			var georeference = FindComponentInScene<CesiumGeoreferenceComponent>(rootScene.Entities);
			if (anchor != null && georeference != null)
			{
				anchorPosition = anchor.ResolveLocalPosition(georeference);
				return true;
			}

			anchorPosition = anchorEntity.Transform.Position;
			return true;
		}

		private bool TryProcessPendingVehicleLoad()
		{
			if (_pendingVehicleLoad == null)
			{
				return false;
			}

			_pendingVehicleLoadWaitSeconds += (float)Game.UpdateTime.Elapsed.TotalSeconds;
			PrepareSpawnLoadingCamera();
			var spawnSupportReady = !ShouldWaitForSpawnSupport(StartupSelection.SceneMode, SpawnAnchorEntityName) || IsSpawnSupportReady();
			if (!spawnSupportReady && TryActivateSpawnFallbackSupport())
			{
				spawnSupportReady = IsSpawnSupportReady();
			}

			if (spawnSupportReady)
			{
				var request = _pendingVehicleLoad;
				_pendingVehicleLoad = null;
				_pendingVehicleLoadWaitSeconds = 0f;
				try
				{
					ShowLoadingOverlay("Spawn support ready.", progress: 1f);
					LoadVehicleNow(request.SelectedVehicle, request.Progress);
				}
				catch (Exception ex)
				{
					Log.Error($"Failed to load vehicle after waiting for spawn support: {ex}");
					_status = $"Load error: {ex.Message}";
				}
				finally
				{
					HideLoadingOverlay();
				}

				return false;
			}

			_status = BuildSpawnWaitingStatus(GetCurrentTrackName());
			ShowLoadingOverlay(_status, progress: 0f);
			return true;
		}

		private bool IsSpawnSupportReady()
		{
			return TryResolveSpawnSupportPoint(out _);
		}

		private bool TryResolveSpawnSupportPoint(out Vector3 supportPoint)
		{
			supportPoint = Vector3.Zero;
			if (!TryResolveSpawnAnchorPosition(out var spawnPosition))
			{
				return false;
			}

			var simulation = ResolveSceneSimulation();
			if (simulation == null)
			{
				return false;
			}

			_spawnSupportHits.Clear();
			var rayOrigin = spawnPosition + Vector3.UnitY * SpawnSupportProbeHeight;
			var rayDirection = -Vector3.UnitY;
			simulation.RayCastPenetrating(in rayOrigin, in rayDirection, SpawnSupportProbeLength, _spawnSupportHits);
			var nearestDistance = float.PositiveInfinity;
			foreach (var hit in _spawnSupportHits)
			{
				if (IsSpawnSupportHitReady(hit.Collidable, hit.Distance, SpawnSupportProbeLength))
				{
					if (hit.Distance < nearestDistance)
					{
						nearestDistance = hit.Distance;
					}
				}
			}

			if (!float.IsFinite(nearestDistance))
			{
				return false;
			}

			supportPoint = rayOrigin + rayDirection * nearestDistance;
			return true;
		}

		internal static bool IsSpawnSupportHitReady(object? collidable, float hitDistance, float maximumAcceptedDistance)
		{
			if (collidable is not EntityComponent entityComponent
			    || entityComponent.Entity?.Get<TrackSurfaceComponent>() == null)
			{
				return false;
			}

			return float.IsFinite(hitDistance)
			       && hitDistance >= 0f
			       && hitDistance <= maximumAcceptedDistance;
		}

		internal static bool ShouldActivateSpawnFallbackSupport(float elapsedSeconds, float fallbackDelaySeconds) =>
			float.IsFinite(elapsedSeconds) && elapsedSeconds >= Math.Max(fallbackDelaySeconds, 0f);

		internal static Vector3 BuildSpawnFallbackSupportCenter(Vector3 spawnPosition, float topOffset, float padThickness)
		{
			var safeTopOffset = Math.Max(topOffset, 0f);
			var safeThickness = Math.Max(padThickness, 0f);
			return spawnPosition - Vector3.UnitY * (safeTopOffset + safeThickness * 0.5f);
		}

		internal static Vector3 BuildSpawnPositionFromSupportPoint(Vector3 supportPoint, float clearance)
		{
			return supportPoint + Vector3.UnitY * Math.Max(clearance, 0f);
		}

		internal static (Vector3 Position, Quaternion Rotation) BuildSpawnLoadingCameraPose(Vector3 spawnPosition)
		{
			var cameraPosition = spawnPosition + new Vector3(0f, SpawnLoadingCameraHeight, SpawnLoadingCameraBackOffset);
			var forward = spawnPosition - cameraPosition;
			return (cameraPosition, CesiumRotationUtilities.CreateCameraRotationFromUpAndForward(Vector3.UnitY, forward));
		}

		private void PrepareSpawnLoadingCamera()
		{
			if (!TryResolveSpawnAnchorPosition(out var spawnPosition))
			{
				return;
			}

			var cameraEntity = ResolveCameraEntity();
			if (cameraEntity == null)
			{
				return;
			}

			RemoveCameraScripts(cameraEntity);
			var pose = BuildSpawnLoadingCameraPose(spawnPosition);
			cameraEntity.Transform.Position = pose.Position;
			cameraEntity.Transform.Rotation = pose.Rotation;
		}

		private bool TryActivateSpawnFallbackSupport()
		{
			if (_spawnFallbackSupportEntity != null)
			{
				return true;
			}

			if (!ShouldActivateSpawnFallbackSupport(_pendingVehicleLoadWaitSeconds, SpawnSupportFallbackDelaySeconds)
			    || !TryResolveSpawnAnchorPosition(out var spawnPosition))
			{
				return false;
			}

			var supportEntity = new Entity("CesiumSpawnFallbackSupport");
			supportEntity.Transform.Position = BuildSpawnFallbackSupportCenter(
				spawnPosition,
				SpawnSupportFallbackTopOffset,
				SpawnSupportFallbackPadSize.Y);
			supportEntity.Add(new StaticComponent
			{
				FrictionCoefficient = 1.1f,
				Collider = new CompoundCollider
				{
					Colliders = { new BoxCollider { Size = SpawnSupportFallbackPadSize } },
				},
			});
			supportEntity.Add(new TrackSurfaceComponent
			{
				SurfaceType = LibreRally.Vehicle.Physics.SurfaceType.Tarmac,
				SurfaceFrictionCoefficient = 1.1f,
			});
			Entity.AddChild(supportEntity);
			_spawnFallbackSupportEntity = supportEntity;
			_status = $"Using temporary spawn support for {GetCurrentTrackName()}.";
			return true;
		}

		private BepuSimulation? ResolveSceneSimulation()
		{
			var rootScene = SceneSystem.SceneInstance.RootScene;
			if (rootScene == null)
			{
				return null;
			}

			return FindComponentInScene<StaticComponent>(rootScene.Entities)?.Simulation
			       ?? FindComponentInScene<BodyComponent>(rootScene.Entities)?.Simulation;
		}

		private void ShowLoadingOverlay(string statusText, float progress)
		{
			if (_loadingOverlay == null)
			{
				return;
			}

			_loadingOverlay.TitleText = "Preparing Stage...";
			_loadingOverlay.StatusText = statusText;
			_loadingOverlay.SetProgress(progress);
			_loadingOverlay.Visible = true;
			_loadingOverlay.Enabled = true;
		}

		private void HideLoadingOverlay()
		{
			if (_loadingOverlay == null)
			{
				return;
			}

			_loadingOverlay.Visible = false;
			_loadingOverlay.Enabled = false;
		}

		private void InitializeLiveTuningBridge()
		{
			if (!LiveTuningBridgeEnabled || _liveTuningBridgeServer != null)
			{
				return;
			}

			try
			{
				_liveTuningBridgeServer = new VehicleLiveTuningBridgeServer(LiveTuningBridgePort);
				_liveTuningBridgeServer.Start();
				Log.Info($"[VehicleSpawner] Live tuning bridge listening on 127.0.0.1:{_liveTuningBridgeServer.Port}");
			}
			catch (SocketException ex)
			{
				Log.Error($"Failed to start live tuning bridge on port {LiveTuningBridgePort}: {ex.Message}");
				_status = $"Live bridge error: {ex.Message}";
			}
			catch (ArgumentOutOfRangeException ex)
			{
				Log.Error($"Invalid live tuning bridge port '{LiveTuningBridgePort}': {ex.Message}");
				_status = $"Live bridge error: {ex.Message}";
			}
		}

		private LiveTuningSnapshot CreateLiveTuningSnapshot()
		{
			return VehicleLiveTuningController.CreateSnapshot(
				_vehicleSession?.LoadedVehicle,
				GetCurrentVehicleName(),
				_status,
				VehicleFolderPath,
				ConfigFileName);
		}

		private LiveTuningBridgeResponse ApplyLiveTuningPatch(LiveTuningPatch patch)
		{
			var response = VehicleLiveTuningController.ApplyPatch(
				_vehicleSession?.LoadedVehicle,
				GetCurrentVehicleName(),
				_status,
				VehicleFolderPath,
				ConfigFileName,
				patch);
			if (response.Succeeded)
			{
				_status = response.Summary;
			}

			return response;
		}

		private LiveTuningBridgeResponse ReloadLiveTuningVehicle()
		{
			try
			{
				var selectedVehicle = ResolveSelectedVehicleDescriptor();
				RequestVehicleLoad(selectedVehicle);
				var isQueued = _pendingVehicleLoad != null;
				return new LiveTuningBridgeResponse
				{
					Succeeded = true,
					Command = "reload_vehicle",
					Summary = isQueued
						? $"Queued reload for {GetCurrentVehicleName()} while Cesium spawn support becomes ready."
						: $"Reloaded {GetCurrentVehicleName()} with the current config.",
					Snapshot = CreateLiveTuningSnapshot(),
				};
			}
			catch (InvalidOperationException ex)
			{
				return new LiveTuningBridgeResponse
				{
					Succeeded = false,
					Command = "reload_vehicle",
					Summary = ex.Message,
				};
			}
			catch (IOException ex)
			{
				return new LiveTuningBridgeResponse
				{
					Succeeded = false,
					Command = "reload_vehicle",
					Summary = ex.Message,
				};
			}
			catch (UnauthorizedAccessException ex)
			{
				return new LiveTuningBridgeResponse
				{
					Succeeded = false,
					Command = "reload_vehicle",
					Summary = ex.Message,
				};
			}
		}

		/// <summary>
		/// Handles per-frame input, telemetry, and debug updates for spawned vehicles.
		/// </summary>
		public override void Update()
		{
			HandlePauseAndVehicleSelectionInput();
			if (_pendingSceneSelection != null)
			{
				ReloadScene(_pendingSceneSelection);
				return;
			}

			if (TryProcessPendingVehicleLoad())
			{
				return;
			}

			var dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;
			_telemetrySession.Update((Game)Game, dt, _vehicleSession?.Car);
			_liveTuningBridgeServer?.Pump(CreateLiveTuningSnapshot, ApplyLiveTuningPatch, ReloadLiveTuningVehicle);

			// Toggle debug info with F3
			if (Input.IsKeyPressed(Keys.F3))
			{
				_showDebug = !_showDebug;
			}

			if (_drivingHudOverlay != null)
			{
				_drivingHudOverlay.Car = _vehicleSession?.Car;
				_drivingHudOverlay.StatusText = _status;
				_drivingHudOverlay.DebugOverlayVisible = _showDebug;
			}

			if (_vehicleSelectionOverlay != null)
			{
				_vehicleSelectionOverlay.Vehicles = GetAvailableVehicles();
				_vehicleSelectionOverlay.SelectedIndex = _selectedVehicleIndex;
				_vehicleSelectionOverlay.OverlayVisible = _activeMenuScreen == MenuScreen.VehicleSelection;
				_vehicleSelectionOverlay.StatusText = _status;
			}

			if (_trackSelectionOverlay != null)
			{
				_trackSelectionOverlay.Items = GetAvailableTrackItems();
				_trackSelectionOverlay.SelectedIndex = _selectedTrackIndex;
				_trackSelectionOverlay.OverlayVisible = _activeMenuScreen == MenuScreen.TrackSelection;
				_trackSelectionOverlay.CurrentTrackName = GetCurrentTrackName();
				_trackSelectionOverlay.StatusText = _status;
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

			if (_vehicleSession?.Car == null)
			{
				DebugText.Print(_status, new Int2(10, 10));
			}
		}

		private void ReloadScene(StartupSceneSelection selection)
		{
			_pendingSceneSelection = null;
			DisposeRuntimeSystems();
			var game = (Game)Game;
			var sceneSetup = CodeFirstSceneFactory.CreateStartupSceneSetup(game, selection);
			SceneSystem.GraphicsCompositor = sceneSetup.GraphicsCompositor;
			SceneSystem.SceneInstance.RootScene = sceneSetup.Scene;
		}

		private void DisposeRuntimeSystems()
		{
			_pendingVehicleLoad = null;
			_pendingVehicleLoadWaitSeconds = 0f;
			_spawnFallbackSupportEntity = null;
			UnloadVehicle();
			DisposeOverlay(ref _trackSelectionOverlay);
			DisposeOverlay(ref _vehicleSelectionOverlay);
			DisposeOverlay(ref _pauseMenuOverlay);
			DisposeOverlay(ref _setupUiShellOverlay);
			DisposeOverlay(ref _physicsCalibrationOverlay);
			DisposeOverlay(ref _telemetryOverlay);
			DisposeOverlay(ref _loadingOverlay);
			DisposeOverlay(ref _drivingHudOverlay);
			_liveTuningBridgeServer?.Dispose();
			_liveTuningBridgeServer = null;
		}

		private void DisposeOverlay<T>(ref T? overlay)
			where T : GameSystemBase
		{
			if (overlay == null)
			{
				return;
			}

			((Game)Game).GameSystems.Remove(overlay);
			overlay = null;
		}

		private static Entity? FindEntityByName(IEnumerable<Entity> entities, string name)
		{
			foreach (var entity in entities)
			{
				if (string.Equals(entity.Name, name, StringComparison.Ordinal))
				{
					return entity;
				}

				var childMatch = FindEntityByName(entity.GetChildren(), name);
				if (childMatch != null)
				{
					return childMatch;
				}
			}

			return null;
		}

		private Entity? ResolveCameraEntity()
		{
			return SceneSystem.SceneInstance.RootScene?.Entities
				.FirstOrDefault(entity => entity.Get<CameraComponent>() != null);
		}

		private static void RemoveCameraScripts(Entity cameraEntity)
		{
			foreach (var script in cameraEntity.GetAll<SyncScript>().ToList())
			{
				cameraEntity.Remove(script);
			}
		}

		private static T? FindComponentInScene<T>(IEnumerable<Entity> entities)
			where T : EntityComponent
		{
			foreach (var entity in entities)
			{
				var component = entity.Get<T>();
				if (component != null)
				{
					return component;
				}

				var childComponent = FindComponentInScene<T>(entity.GetChildren());
				if (childComponent != null)
				{
					return childComponent;
				}
			}

			return null;
		}

		private static string ResolveTrackSelectionLabel(StartupSceneSelection selection) =>
			selection.SceneMode == StartupSceneMode.Cesium
				? selection.CesiumStage.DisplayName
				: "Local Test Track";

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
			new($"Physics Rate: {RallyCarComponent.PhysicsTickRateLabel}", "Cycle through physics tick rates (30/60/100/120/240/360 Hz). Higher rates improve handling at low FPS."),
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
			=> _telemetrySession.BuildOutGaugeSummary();

		private string BuildOutSimSummary()
			=> _telemetrySession.BuildOutSimSummary();

		private void SetOutGaugeEnabled(bool enabled)
		{
			_telemetrySession.SetOutGaugeEnabled(enabled);
		}

		private void SetOutSimEnabled(bool enabled)
		{
			_telemetrySession.SetOutSimEnabled(enabled);
		}

		private void ConnectOutGauge()
		{
			_telemetrySession.ConnectOutGauge();
			SetTelemetryStatus($"OutGauge socket connect requested ({_telemetrySession.DescribeOutGaugeSocketState()}).");
		}

		private void DisconnectOutGauge()
		{
			_telemetrySession.DisconnectOutGauge();
			SetTelemetryStatus("OutGauge socket disconnected.");
		}

		private void ReconnectOutGauge()
		{
			_telemetrySession.ReconnectOutGauge();
			SetTelemetryStatus($"OutGauge socket reconnected ({_telemetrySession.DescribeOutGaugeSocketState()}).");
		}

		private void ConnectOutSim()
		{
			_telemetrySession.ConnectOutSim();
			SetTelemetryStatus($"OutSim socket connect requested ({_telemetrySession.DescribeOutSimSocketState()}).");
		}

		private void DisconnectOutSim()
		{
			_telemetrySession.DisconnectOutSim();
			SetTelemetryStatus("OutSim socket disconnected.");
		}

		private void ReconnectOutSim()
		{
			_telemetrySession.ReconnectOutSim();
			SetTelemetryStatus($"OutSim socket reconnected ({_telemetrySession.DescribeOutSimSocketState()}).");
		}

		private void SetTelemetryStatus(string message)
		{
			_status = $"Telemetry: {message}";
			RefreshTelemetryOverlay();
		}
	}
}
