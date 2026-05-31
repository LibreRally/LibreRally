using System;
using Evergine.Bindings.CesiumNative;
using Stride.Core.Mathematics;

namespace LibreRally
{
	/// <summary>
	/// Maps a fixed local Stride coordinate system to a WGS84 Earth-centered frame for Cesium tileset rendering.
	/// </summary>
	internal sealed class CesiumLocalGeoreference
	{
		private readonly Vector3 _localOrigin;
		private readonly CesiumDouble3 _originEcef;
		private readonly CesiumDouble3 _eastAxis;
		private readonly CesiumDouble3 _upAxis;
		private readonly CesiumDouble3 _northAxis;

		/// <summary>
		/// Initializes a new georeference centered on the provided geodetic origin.
		/// </summary>
		/// <param name="longitudeDegrees">Longitude in degrees.</param>
		/// <param name="latitudeDegrees">Latitude in degrees.</param>
		/// <param name="heightMeters">Height above the ellipsoid in meters.</param>
		public CesiumLocalGeoreference(double longitudeDegrees, double latitudeDegrees, double heightMeters)
			: this(longitudeDegrees, latitudeDegrees, heightMeters, Vector3.Zero)
		{
		}

		/// <summary>
		/// Initializes a new georeference centered on the provided geodetic origin and mapped to a configurable local origin.
		/// </summary>
		/// <param name="longitudeDegrees">Longitude in degrees.</param>
		/// <param name="latitudeDegrees">Latitude in degrees.</param>
		/// <param name="heightMeters">Height above the ellipsoid in meters.</param>
		/// <param name="localOrigin">The local Stride position that should represent the geodetic origin.</param>
		public CesiumLocalGeoreference(double longitudeDegrees, double latitudeDegrees, double heightMeters, Vector3 localOrigin)
		{
			_localOrigin = localOrigin;
			using var ellipsoid = Ellipsoid.Wgs84();
			var origin = ellipsoid.CartographicToCartesian(Cartographic.FromDegrees(longitudeDegrees, latitudeDegrees, heightMeters));
			_originEcef = new CesiumDouble3(origin.X, origin.Y, origin.Z);
			ResolveEastUpNorthAxes(longitudeDegrees, latitudeDegrees, out _eastAxis, out _upAxis, out _northAxis);
		}

		/// <summary>
		/// Converts an ECEF position into local Stride coordinates.
		/// </summary>
		/// <param name="ecefPosition">The Earth-centered position in meters.</param>
		/// <returns>The local position in meters.</returns>
		public Vector3 ToLocalPosition(CesiumDouble3 ecefPosition)
		{
			var relative = ecefPosition - _originEcef;
			return _localOrigin + new Vector3(
				(float)CesiumDouble3.Dot(relative, _eastAxis),
				(float)CesiumDouble3.Dot(relative, _upAxis),
				(float)-CesiumDouble3.Dot(relative, _northAxis));
		}

		/// <summary>
		/// Converts a local Stride position into ECEF meters.
		/// </summary>
		/// <param name="localPosition">The local position in meters.</param>
		/// <returns>The Earth-centered position in meters.</returns>
		public CesiumDouble3 ToEarthCenteredEarthFixed(Vector3 localPosition)
		{
			var relativePosition = localPosition - _localOrigin;
			var east = _eastAxis * relativePosition.X;
			var up = _upAxis * relativePosition.Y;
			var south = _northAxis * -relativePosition.Z;
			return _originEcef + east + up + south;
		}

		/// <summary>
		/// Converts a local Stride direction into an ECEF direction.
		/// </summary>
		/// <param name="localDirection">The local direction vector.</param>
		/// <returns>The equivalent ECEF direction vector.</returns>
		public CesiumDouble3 ToEarthCenteredEarthFixedDirection(Vector3 localDirection)
		{
			var ecefDirection = (_eastAxis * localDirection.X)
			                  + (_upAxis * localDirection.Y)
			                  + (_northAxis * -localDirection.Z);
			return CesiumDouble3.Normalize(ecefDirection);
		}

		/// <summary>
		/// Converts an ECEF direction into a local Stride direction.
		/// </summary>
		/// <param name="ecefDirection">The ECEF direction vector.</param>
		/// <returns>The equivalent local direction vector.</returns>
		public Vector3 ToLocalDirection(CesiumDouble3 ecefDirection)
		{
			var normalizedDirection = CesiumDouble3.Normalize(ecefDirection);
			return new Vector3(
				(float)CesiumDouble3.Dot(normalizedDirection, _eastAxis),
				(float)CesiumDouble3.Dot(normalizedDirection, _upAxis),
				(float)-CesiumDouble3.Dot(normalizedDirection, _northAxis));
		}

		/// <summary>
		/// Resolves east, up, and north axes for a geodetic location and returns them in local Stride space.
		/// </summary>
		/// <param name="longitudeDegrees">Longitude in degrees.</param>
		/// <param name="latitudeDegrees">Latitude in degrees.</param>
		/// <param name="east">The east axis expressed in local Stride coordinates.</param>
		/// <param name="up">The up axis expressed in local Stride coordinates.</param>
		/// <param name="north">The north axis expressed in local Stride coordinates.</param>
		public void ResolveEastUpNorthAxes(double longitudeDegrees, double latitudeDegrees, out Vector3 east, out Vector3 up, out Vector3 north)
		{
			CesiumLocalGeoreference.ResolveEastUpNorthAxes(longitudeDegrees, latitudeDegrees, out CesiumDouble3 eastEcef, out CesiumDouble3 upEcef, out CesiumDouble3 northEcef);
			east = ToLocalDirection(eastEcef);
			up = ToLocalDirection(upEcef);
			north = ToLocalDirection(northEcef);
		}

		internal static void ResolveEastUpNorthAxes(double longitudeDegrees, double latitudeDegrees, out CesiumDouble3 east, out CesiumDouble3 up, out CesiumDouble3 north)
		{
			var longitudeRadians = MathUtil.DegreesToRadians((float)longitudeDegrees);
			var latitudeRadians = MathUtil.DegreesToRadians((float)latitudeDegrees);
			var cosLatitude = Math.Cos(latitudeRadians);
			var sinLatitude = Math.Sin(latitudeRadians);
			var cosLongitude = Math.Cos(longitudeRadians);
			var sinLongitude = Math.Sin(longitudeRadians);

			east = CesiumDouble3.Normalize(new CesiumDouble3(-sinLongitude, cosLongitude, 0d));
			up = CesiumDouble3.Normalize(new CesiumDouble3(cosLatitude * cosLongitude, cosLatitude * sinLongitude, sinLatitude));
			north = CesiumDouble3.Normalize(CesiumDouble3.Cross(up, east));
		}
	}

	/// <summary>
	/// Double-precision vector helpers used for georeferenced Cesium transforms.
	/// </summary>
	internal readonly record struct CesiumDouble3(double X, double Y, double Z)
	{
		/// <summary>
		/// Adds two vectors.
		/// </summary>
		public static CesiumDouble3 operator +(CesiumDouble3 left, CesiumDouble3 right) =>
			new(left.X + right.X, left.Y + right.Y, left.Z + right.Z);

		/// <summary>
		/// Subtracts two vectors.
		/// </summary>
		public static CesiumDouble3 operator -(CesiumDouble3 left, CesiumDouble3 right) =>
			new(left.X - right.X, left.Y - right.Y, left.Z - right.Z);

		/// <summary>
		/// Scales the vector by a scalar value.
		/// </summary>
		public static CesiumDouble3 operator *(CesiumDouble3 value, double scale) =>
			new(value.X * scale, value.Y * scale, value.Z * scale);

		/// <summary>
		/// Computes the dot product for two vectors.
		/// </summary>
		public static double Dot(CesiumDouble3 left, CesiumDouble3 right) =>
			(left.X * right.X) + (left.Y * right.Y) + (left.Z * right.Z);

		/// <summary>
		/// Computes the cross product for two vectors.
		/// </summary>
		public static CesiumDouble3 Cross(CesiumDouble3 left, CesiumDouble3 right) =>
			new(
				(left.Y * right.Z) - (left.Z * right.Y),
				(left.Z * right.X) - (left.X * right.Z),
				(left.X * right.Y) - (left.Y * right.X));

		/// <summary>
		/// Normalizes the provided vector.
		/// </summary>
		public static CesiumDouble3 Normalize(CesiumDouble3 value)
		{
			var lengthSquared = Dot(value, value);
			if (lengthSquared <= double.Epsilon)
			{
				return new CesiumDouble3(0d, 1d, 0d);
			}

			var inverseLength = 1d / Math.Sqrt(lengthSquared);
			return value * inverseLength;
		}
	}
}
