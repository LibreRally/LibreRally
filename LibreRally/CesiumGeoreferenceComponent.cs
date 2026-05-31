using System;
using Evergine.Bindings.CesiumNative;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally
{
	/// <summary>
	/// Stores a reusable local-to-globe georeference for Stride scene entities.
	/// </summary>
	/// <remarks>
	/// The component keeps the WGS84 origin and a configurable local origin separate so anchored entities can be
	/// reprojected if the local frame is rebased in the future.
	/// </remarks>
	[DataContract]
	[ComponentCategory("LibreRally")]
	public sealed class CesiumGeoreferenceComponent : EntityComponent
	{
		private double _originLongitudeDegrees = -75.612094d;
		private double _originLatitudeDegrees = 40.042531d;
		private double _originHeightMeters = 20d;
		private Vector3 _localOrigin = Vector3.Zero;
		private int _revision;
		private CesiumLocalGeoreference? _cachedGeoreference;

		/// <summary>
		/// Gets or sets the WGS84 longitude in degrees for the local frame origin.
		/// </summary>
		public double OriginLongitudeDegrees
		{
			get => _originLongitudeDegrees;
			set => SetDoubleField(ref _originLongitudeDegrees, value);
		}

		/// <summary>
		/// Gets or sets the WGS84 latitude in degrees for the local frame origin.
		/// </summary>
		public double OriginLatitudeDegrees
		{
			get => _originLatitudeDegrees;
			set => SetDoubleField(ref _originLatitudeDegrees, value);
		}

		/// <summary>
		/// Gets or sets the ellipsoid height in meters for the local frame origin.
		/// </summary>
		public double OriginHeightMeters
		{
			get => _originHeightMeters;
			set => SetDoubleField(ref _originHeightMeters, value);
		}

		/// <summary>
		/// Gets or sets the local Stride position that represents the configured globe origin.
		/// </summary>
		public Vector3 LocalOrigin
		{
			get => _localOrigin;
			set => SetVectorField(ref _localOrigin, value);
		}

		/// <summary>
		/// Gets the monotonically increasing revision used by runtime scripts to detect rebasing or origin changes.
		/// </summary>
		public int Revision => _revision;

		/// <summary>
		/// Resolves the current georeference math instance for this component.
		/// </summary>
		/// <returns>The cached georeference for the current configuration.</returns>
		internal CesiumLocalGeoreference ResolveGeoreference()
		{
			_cachedGeoreference ??= new CesiumLocalGeoreference(
				OriginLongitudeDegrees,
				OriginLatitudeDegrees,
				OriginHeightMeters,
				LocalOrigin);
			return _cachedGeoreference;
		}

		/// <summary>
		/// Converts a WGS84 position into local Stride coordinates using the current georeference.
		/// </summary>
		/// <param name="longitudeDegrees">Longitude in degrees.</param>
		/// <param name="latitudeDegrees">Latitude in degrees.</param>
		/// <param name="heightMeters">Height above the ellipsoid in meters.</param>
		/// <returns>The local Stride position.</returns>
		public Vector3 ToLocalPosition(double longitudeDegrees, double latitudeDegrees, double heightMeters)
		{
			return ResolveGeoreference().ToLocalPosition(CesiumCartographicUtilities.ToEarthCenteredEarthFixed(longitudeDegrees, latitudeDegrees, heightMeters));
		}

		/// <summary>
		/// Converts a local Stride position into ECEF meters using the current georeference.
		/// </summary>
		/// <param name="localPosition">The local Stride position.</param>
		/// <returns>The corresponding ECEF position in meters.</returns>
		internal CesiumDouble3 ToEarthCenteredEarthFixed(Vector3 localPosition)
		{
			return ResolveGeoreference().ToEarthCenteredEarthFixed(localPosition);
		}

		/// <summary>
		/// Resolves east, up, and north axes for a WGS84 location in local Stride coordinates.
		/// </summary>
		/// <param name="longitudeDegrees">Longitude in degrees.</param>
		/// <param name="latitudeDegrees">Latitude in degrees.</param>
		/// <param name="east">The east axis expressed in local Stride coordinates.</param>
		/// <param name="up">The up axis expressed in local Stride coordinates.</param>
		/// <param name="north">The north axis expressed in local Stride coordinates.</param>
		public void ResolveEastUpNorthAxes(double longitudeDegrees, double latitudeDegrees, out Vector3 east, out Vector3 up, out Vector3 north)
		{
			ResolveGeoreference().ResolveEastUpNorthAxes(longitudeDegrees, latitudeDegrees, out east, out up, out north);
		}

		private void SetDoubleField(ref double field, double value)
		{
			if (Math.Abs(field - value) <= 1e-12d)
			{
				return;
			}

			field = value;
			MarkDirty();
		}

		private void SetVectorField(ref Vector3 field, Vector3 value)
		{
			if ((field - value).LengthSquared() <= 1e-12f)
			{
				return;
			}

			field = value;
			MarkDirty();
		}

		private void MarkDirty()
		{
			_cachedGeoreference = null;
			unchecked
			{
				_revision++;
				if (_revision == int.MinValue)
				{
					_revision = 0;
				}
			}
		}
	}

	internal static class CesiumCartographicUtilities
	{
		public static CesiumDouble3 ToEarthCenteredEarthFixed(double longitudeDegrees, double latitudeDegrees, double heightMeters)
		{
			using var ellipsoid = Ellipsoid.Wgs84();
			var ecef = ellipsoid.CartographicToCartesian(Cartographic.FromDegrees(longitudeDegrees, latitudeDegrees, heightMeters));
			return new CesiumDouble3(ecef.X, ecef.Y, ecef.Z);
		}
	}
}
