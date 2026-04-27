using System;
using LibreRally.Vehicle.Content;

namespace LibreRally.Vehicle
{
	/// <summary>
	/// Loads resolved BeamNG vehicle content into runtime LibreRally entities.
	/// </summary>
	internal interface IVehicleLoader
	{
		/// <summary>
		/// Loads a vehicle from a folder on disk.
		/// </summary>
		/// <param name="vehicleFolderPath">The absolute path to the vehicle folder.</param>
		/// <param name="configFileName">The optional configuration file name.</param>
		/// <param name="setupOverrides">The optional live setup overrides.</param>
		/// <param name="progress">Optional progress reporter for loading UI updates.</param>
		/// <returns>The loaded vehicle.</returns>
		LoadedVehicle Load(string vehicleFolderPath, string? configFileName = null, VehicleSetupOverrides? setupOverrides = null, IProgress<VehicleLoadProgress>? progress = null);

		/// <summary>
		/// Loads a vehicle from a resolved BeamNG content source.
		/// </summary>
		/// <param name="vehicleSource">The resolved BeamNG source.</param>
		/// <param name="configFileName">The optional configuration file name.</param>
		/// <param name="setupOverrides">The optional live setup overrides.</param>
		/// <param name="progress">Optional progress reporter for loading UI updates.</param>
		/// <returns>The loaded vehicle.</returns>
		LoadedVehicle Load(BeamNgResolvedVehicle vehicleSource, string? configFileName = null, VehicleSetupOverrides? setupOverrides = null, IProgress<VehicleLoadProgress>? progress = null);
	}
}
