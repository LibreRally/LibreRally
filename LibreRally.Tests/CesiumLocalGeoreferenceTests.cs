using Stride.Core.Mathematics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the fixed local-to-ECEF georeference used by the Cesium runtime bridge.
	/// </summary>
	public sealed class CesiumLocalGeoreferenceTests
	{
		/// <summary>
		/// Verifies that the configured origin maps to the local scene origin.
		/// </summary>
		[Fact]
		public void ToEarthCenteredEarthFixed_OriginRoundTripsToLocalZero()
		{
			var georeference = new CesiumLocalGeoreference(-75.612094, 40.042531, 20d);

			var ecef = georeference.ToEarthCenteredEarthFixed(Vector3.Zero);
			var local = georeference.ToLocalPosition(ecef);

			Assert.True(local.Length() < 0.001f);
		}

		/// <summary>
		/// Verifies that cardinal local axes survive a local-to-ECEF-to-local round trip.
		/// </summary>
		[Theory]
		[InlineData(25f, 0f, 0f)]
		[InlineData(0f, 15f, 0f)]
		[InlineData(0f, 0f, 40f)]
		[InlineData(12f, 8f, -22f)]
		public void ToEarthCenteredEarthFixed_RoundTripsLocalOffsets(float x, float y, float z)
		{
			var georeference = new CesiumLocalGeoreference(-75.612094, 40.042531, 20d);
			var input = new Vector3(x, y, z);

			var ecef = georeference.ToEarthCenteredEarthFixed(input);
			var local = georeference.ToLocalPosition(ecef);

			Assert.InRange(MathF.Abs(local.X - input.X), 0f, 0.01f);
			Assert.InRange(MathF.Abs(local.Y - input.Y), 0f, 0.01f);
			Assert.InRange(MathF.Abs(local.Z - input.Z), 0f, 0.01f);
		}

		/// <summary>
		/// Verifies that the local Cesium frame stays right-handed in Stride's Y-up coordinates.
		/// </summary>
		[Fact]
		public void ResolveEastUpNorthAxes_MapsNorthToNegativeZ()
		{
			var georeference = new CesiumLocalGeoreference(-75.612094, 40.042531, 20d);

			georeference.ResolveEastUpNorthAxes(-75.612094, 40.042531, out var east, out var up, out var north);

			Assert.InRange(Vector3.Distance(east, Vector3.UnitX), 0f, 0.001f);
			Assert.InRange(Vector3.Distance(up, Vector3.UnitY), 0f, 0.001f);
			Assert.InRange(Vector3.Distance(north, -Vector3.UnitZ), 0f, 0.001f);
		}
	}
}
