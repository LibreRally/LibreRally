using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Content;
using LibreRally.Vehicle.Physics;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally
{
	/// <summary>
	/// Coordinates vehicle catalog discovery, load/unload flow, and live setup application for <see cref="VehicleSpawner"/>.
	/// </summary>
	internal sealed class VehicleSpawnerVehicleSession
	{
		private static readonly Logger Log = GlobalLogger.GetLogger("VehicleSpawnerVehicleSession");
		private readonly Game _game;
		private readonly SceneSystem _sceneSystem;
		private readonly VehicleSetupOverrides _setupOverrides;
		private readonly Func<Game, IVehicleLoader> _loaderFactory;
		private IVehicleCatalog? _vehicleCatalog;

		/// <summary>
		/// Initializes a new instance of the <see cref="VehicleSpawnerVehicleSession"/> class.
		/// </summary>
		/// <param name="game">The active game instance.</param>
		/// <param name="sceneSystem">The active scene system.</param>
		/// <param name="setupOverrides">The mutable setup overrides shared with the garage UI.</param>
		/// <param name="loaderFactory">The factory used to create vehicle loaders.</param>
		public VehicleSpawnerVehicleSession(
			Game game,
			SceneSystem sceneSystem,
			VehicleSetupOverrides setupOverrides,
			Func<Game, IVehicleLoader>? loaderFactory = null)
		{
			_game = game ?? throw new ArgumentNullException(nameof(game));
			_sceneSystem = sceneSystem ?? throw new ArgumentNullException(nameof(sceneSystem));
			_setupOverrides = setupOverrides ?? throw new ArgumentNullException(nameof(setupOverrides));
			_loaderFactory = loaderFactory ?? (gameInstance => new VehicleLoader(gameInstance));
		}

		/// <summary>
		/// Gets the currently discovered selectable vehicle variants.
		/// </summary>
		public IReadOnlyList<BeamNgVehicleVariantDescriptor> AvailableVehicles { get; private set; } = [];

		/// <summary>
		/// Gets the currently loaded runtime vehicle.
		/// </summary>
		public LoadedVehicle? LoadedVehicle { get; private set; }

		/// <summary>
		/// Gets the active rally car component when a vehicle is loaded.
		/// </summary>
		public RallyCarComponent? Car => LoadedVehicle?.CarComponent;

		/// <summary>
		/// Initializes the BeamNG vehicle catalog and discovers selectable variants.
		/// </summary>
		/// <param name="vehicleFolderPath">The currently configured vehicle folder path.</param>
		/// <param name="configFileName">The currently configured vehicle config name.</param>
		/// <returns>The resolved selection index within <see cref="AvailableVehicles"/>.</returns>
		public int InitializeCatalog(string vehicleFolderPath, string configFileName)
		{
			var bundledVehiclesRoot = Path.Combine(AppContext.BaseDirectory, "Resources", "BeamNG Vehicles");
			var beamNgContentVehiclesRoot = BeamNgVehicleCatalog.DetectBeamNgContentVehiclesRoot();
			_vehicleCatalog = new BeamNgVehicleCatalog(bundledVehiclesRoot, beamNgContentVehiclesRoot);
			AvailableVehicles = _vehicleCatalog.DiscoverBundledVehicleVariants();
			return ResolveVehicleSelectionIndex(vehicleFolderPath, configFileName);
		}

		/// <summary>
		/// Loads the requested vehicle into the current scene and returns the updated session state.
		/// </summary>
		/// <param name="vehicleFolderPath">The currently configured vehicle folder path.</param>
		/// <param name="configFileName">The currently configured config file name.</param>
		/// <param name="spawnPosition">The world-space spawn offset applied to the loaded vehicle.</param>
		/// <param name="selectedVehicleIndex">The previously selected vehicle index.</param>
		/// <param name="selectedVehicle">The optional explicit vehicle variant to load.</param>
		/// <returns>The updated vehicle load result.</returns>
		public VehicleSpawnerVehicleLoadResult LoadVehicle(
			string vehicleFolderPath,
			string configFileName,
			Vector3 spawnPosition,
			int selectedVehicleIndex,
			BeamNgVehicleVariantDescriptor? selectedVehicle = null)
		{
			var selectedConfig = selectedVehicle?.ConfigFileName;
			var requestedConfig = selectedVehicle != null
				? selectedConfig ?? "<jbeam defaults>"
				: string.IsNullOrWhiteSpace(configFileName) ? "<auto>" : configFileName;
			var requestedSource = selectedVehicle?.SourcePath ?? vehicleFolderPath;
			if (selectedVehicle != null &&
			    (!selectedVehicle.SourcePath.Equals(vehicleFolderPath, StringComparison.OrdinalIgnoreCase) ||
			     !string.Equals(selectedVehicle.ConfigFileName, configFileName, StringComparison.OrdinalIgnoreCase)))
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

			var loader = _loaderFactory(_game);
			var configToLoad = selectedVehicle != null
				? selectedVehicle.ConfigFileName
				: string.IsNullOrWhiteSpace(configFileName) ? null : configFileName;
			var vehicle = resolvedVehicle != null
				? loader.Load(resolvedVehicle, configToLoad, _setupOverrides)
				: loader.Load(basePath, configToLoad, _setupOverrides);

			if (selectedVehicle != null)
			{
				vehicleFolderPath = selectedVehicle.SourcePath;
				configFileName = selectedVehicle.ConfigFileName ?? string.Empty;
				selectedVehicleIndex = AvailableVehicles.ToList().FindIndex(vehicleDescriptor =>
					vehicleDescriptor.SourcePath.Equals(selectedVehicle.SourcePath, StringComparison.OrdinalIgnoreCase) &&
					string.Equals(vehicleDescriptor.ConfigFileName, selectedVehicle.ConfigFileName, StringComparison.OrdinalIgnoreCase));
			}

			UnloadVehicle();
			LoadedVehicle = vehicle;

			foreach (var child in vehicle.RootEntity.GetChildren().ToList())
			{
				child.Transform.Position += spawnPosition;
			}

			vehicle.RootEntity.Transform.Position = Vector3.Zero;
			_sceneSystem.SceneInstance.RootScene.Entities.Add(vehicle.RootEntity);

			var activeConfig = vehicle.Diagnostics.ConfigPath != null
				? Path.GetFileName(vehicle.Diagnostics.ConfigPath)
				: "<jbeam defaults>";
			var status = $"Loaded: {vehicle.Definition.VehicleName} src={(resolvedVehicle?.SourceDescription ?? "folder")} cfg={activeConfig} mass={vehicle.Diagnostics.EstimatedMassKg:F0}kg";
			Log.Info($"[VehicleSpawner] {status} folder='{vehicle.Diagnostics.VehicleFolderPath}'");

			return new VehicleSpawnerVehicleLoadResult(
				vehicle,
				vehicleFolderPath,
				configFileName,
				selectedVehicleIndex,
				status);
		}

		/// <summary>
		/// Unloads the current vehicle from the active scene.
		/// </summary>
		public void UnloadVehicle()
		{
			if (LoadedVehicle?.RootEntity != null)
			{
				_sceneSystem.SceneInstance.RootScene.Entities.Remove(LoadedVehicle.RootEntity);
			}

			LoadedVehicle = null;
		}

		/// <summary>
		/// Applies live tyre-pressure overrides to the currently loaded vehicle.
		/// </summary>
		/// <param name="pressureOverrides">The pressure overrides to apply.</param>
		public void ApplyLivePressureOverrides(IReadOnlyDictionary<VehicleSetupAxle, float> pressureOverrides)
		{
			if (LoadedVehicle == null || pressureOverrides.Count == 0)
			{
				return;
			}

			const float PsiToKpa = 6.894757f;
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

		/// <summary>
		/// Resolves the currently selected vehicle descriptor.
		/// </summary>
		/// <param name="selectedVehicleIndex">The selected vehicle index.</param>
		/// <returns>The selected vehicle descriptor when available; otherwise, <see langword="null" />.</returns>
		public BeamNgVehicleVariantDescriptor? ResolveSelectedVehicleDescriptor(int selectedVehicleIndex)
		{
			return selectedVehicleIndex >= 0 && selectedVehicleIndex < AvailableVehicles.Count
				? AvailableVehicles[selectedVehicleIndex]
				: null;
		}

		/// <summary>
		/// Gets the best user-facing name for the active vehicle.
		/// </summary>
		/// <param name="vehicleFolderPath">The configured vehicle folder path used as a fallback.</param>
		/// <returns>The active vehicle name.</returns>
		public string GetCurrentVehicleName(string vehicleFolderPath)
		{
			if (!string.IsNullOrWhiteSpace(LoadedVehicle?.Definition.VehicleName))
			{
				return LoadedVehicle.Definition.VehicleName;
			}

			return Path.GetFileName(vehicleFolderPath.TrimEnd(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar));
		}

		private int ResolveVehicleSelectionIndex(string vehicleFolderPath, string configFileName)
		{
			if (AvailableVehicles.Count == 0)
			{
				return -1;
			}

			var absoluteVehiclePath = Path.IsPathRooted(vehicleFolderPath)
				? vehicleFolderPath
				: Path.GetFullPath(Path.Combine(AppContext.BaseDirectory, vehicleFolderPath));

			var exactPathIndex = AvailableVehicles.ToList().FindIndex(vehicle =>
				vehicle.SourcePath.Equals(absoluteVehiclePath, StringComparison.OrdinalIgnoreCase) &&
				string.Equals(vehicle.ConfigFileName, NormalizeConfigSelection(configFileName), StringComparison.OrdinalIgnoreCase));
			if (exactPathIndex >= 0)
			{
				return exactPathIndex;
			}

			var idIndex = AvailableVehicles.ToList().FindIndex(vehicle =>
				vehicle.VehicleId.Equals(vehicleFolderPath, StringComparison.OrdinalIgnoreCase) ||
				absoluteVehiclePath.EndsWith(Path.DirectorySeparatorChar + vehicle.VehicleId, StringComparison.OrdinalIgnoreCase));
			if (idIndex < 0)
			{
				return 0;
			}

			var preferredConfig = NormalizeConfigSelection(configFileName);
			if (string.IsNullOrWhiteSpace(preferredConfig))
			{
				return idIndex;
			}

			var matchingVariantIndex = AvailableVehicles.ToList().FindIndex(vehicle =>
				(vehicle.VehicleId.Equals(vehicleFolderPath, StringComparison.OrdinalIgnoreCase) ||
				 absoluteVehiclePath.EndsWith(Path.DirectorySeparatorChar + vehicle.VehicleId, StringComparison.OrdinalIgnoreCase)) &&
				string.Equals(vehicle.ConfigFileName, preferredConfig, StringComparison.OrdinalIgnoreCase));
			return matchingVariantIndex >= 0 ? matchingVariantIndex : idIndex;
		}

		private IEnumerable<Entity> EnumerateAxleWheels(VehicleSetupAxle axle)
		{
			if (LoadedVehicle == null)
			{
				yield break;
			}

			if (axle == VehicleSetupAxle.Front)
			{
				yield return LoadedVehicle.WheelFL;
				yield return LoadedVehicle.WheelFR;
				yield break;
			}

			if (axle == VehicleSetupAxle.Rear)
			{
				yield return LoadedVehicle.WheelRL;
				yield return LoadedVehicle.WheelRR;
			}
		}

		private static string? NormalizeConfigSelection(string? configFileName)
			=> BeamNgVehicleCatalog.NormalizeConfigFileName(configFileName);
	}

	/// <summary>
	/// Represents the updated state produced by a vehicle load operation.
	/// </summary>
	/// <param name="LoadedVehicle">The loaded runtime vehicle.</param>
	/// <param name="VehicleFolderPath">The updated configured vehicle folder path.</param>
	/// <param name="ConfigFileName">The updated configured config file name.</param>
	/// <param name="SelectedVehicleIndex">The updated selected vehicle index.</param>
	/// <param name="StatusText">The resulting user-facing status text.</param>
	internal sealed record VehicleSpawnerVehicleLoadResult(
		LoadedVehicle LoadedVehicle,
		string VehicleFolderPath,
		string ConfigFileName,
		int SelectedVehicleIndex,
		string StatusText);
}
