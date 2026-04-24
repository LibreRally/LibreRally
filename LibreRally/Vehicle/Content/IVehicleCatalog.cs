using System.Collections.Generic;

namespace LibreRally.Vehicle.Content
{
	/// <summary>
	/// Discovers and resolves BeamNG vehicle content sources.
	/// </summary>
	internal interface IVehicleCatalog
	{
		/// <summary>
		/// Discovers every bundled vehicle variant that can be selected in LibreRally.
		/// </summary>
		/// <returns>The discovered vehicle variants.</returns>
		IReadOnlyList<BeamNgVehicleVariantDescriptor> DiscoverBundledVehicleVariants();

		/// <summary>
		/// Resolves a requested vehicle identifier or path into a loadable source.
		/// </summary>
		/// <param name="requestedPathOrId">The requested vehicle identifier or path.</param>
		/// <returns>The resolved vehicle source.</returns>
		BeamNgResolvedVehicle ResolveVehicle(string requestedPathOrId);
	}
}
