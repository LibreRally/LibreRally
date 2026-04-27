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
using Stride.Core.Mathematics;
using Stride.Engine;
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
		private PhysicsCalibrationOverlay? _physicsCalibrationOverlay;
		private TelemetryOverlay? _telemetryOverlay;
		private LoadingOverlay? _loadingOverlay;
		private readonly VehicleSetupOverrides _setupOverrides = new();
		private readonly VehicleTelemetrySession _telemetrySession = new();
		private readonly VehicleSpawnerTrackBuilder _trackBuilder = new();
		private VehicleSpawnerVehicleSession? _vehicleSession;
		private VehicleLiveTuningBridgeServer? _liveTuningBridgeServer;
		private int _selectedVehicleIndex;
		private int _pauseMenuSelectedIndex;
		private int _telemetryMenuSelectedIndex;
		private bool _vehicleSelectionOpenedFromPauseMenu;
		private MenuScreen _activeMenuScreen;
		private const int PauseMenuResumeIndex = 0;
		private const int PauseMenuResetVehicleIndex = 1;
		private const int PauseMenuGarageSetupIndex = 2;
		private const int PauseMenuVehicleSelectIndex = 3;
		private const int PauseMenuPhysicsCalibrationIndex = 4;
		private const int PauseMenuTelemetryIndex = 5;
		private const int PauseMenuItemCount = 6;
		private const int TelemetryMenuItemCount = 9;

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
		/// Initializes the track, overlays, and vehicle catalog at scene startup.
		/// </summary>
		public override void Start()
		{
			_vehicleSession = new VehicleSpawnerVehicleSession((Game)Game, SceneSystem, _setupOverrides);
			AddGroundPhysics();
			EnsureDrivingHudOverlay();
			InitializeVehicleCatalog();
			EnsurePauseMenuOverlay();
			EnsureSetupUiShellOverlay();
			EnsureVehicleSelectionOverlay();
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
				LoadVehicle(progress: progress);
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
				if (_loadingOverlay != null)
				{
					_loadingOverlay.Visible = false;
					_loadingOverlay.Enabled = false;
				}
			}
		}

		private void LoadVehicle(BeamNgVehicleVariantDescriptor? selectedVehicle = null, IProgress<VehicleLoadProgress>? progress = null)
		{
			if (_vehicleSession == null)
			{
				throw new InvalidOperationException("Vehicle session was not initialized.");
			}

			var loadResult = _vehicleSession.LoadVehicle(
				VehicleFolderPath,
				ConfigFileName,
				SpawnPosition,
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
				LoadVehicle(vehicles[selectedIndex]);
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

			_vehicleSession?.ApplyLivePressureOverrides(payload.PressureOverrides);
			_status = $"Applied garage setup for {GetCurrentVehicleName()}: {payload.SummaryText}";

			if (!payload.RequiresReload)
			{
				BindGarageSetupOverlay();
				return;
			}

			var selectedVehicle = ResolveSelectedVehicleDescriptor();
			LoadVehicle(selectedVehicle);
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
			var cameraEntity = SceneSystem.SceneInstance.RootScene.Entities
				.FirstOrDefault(e => e.Get<CameraComponent>() != null);
			if (cameraEntity == null)
			{
				return;
			}

			foreach (var s in cameraEntity.GetAll<SyncScript>().ToList())
			{
				cameraEntity.Remove(s);
			}

			cameraEntity.Add(new RallyCameraScript { Target = chassis, CarComponent = car, });
		}

		private void AddGroundPhysics()
		{
			_trackBuilder.ConfigureGround(Entity, (Game)Game);
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
				LoadVehicle(selectedVehicle);
				return new LiveTuningBridgeResponse
				{
					Succeeded = true,
					Command = "reload_vehicle",
					Summary = $"Reloaded {GetCurrentVehicleName()} with the current config.",
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
