using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies reusable Cesium georeference and globe-anchor component behavior.
	/// </summary>
	public sealed class CesiumAnchorComponentTests
	{
		/// <summary>
		/// Verifies that rebasing the local origin is reflected in georeference conversions.
		/// </summary>
		[Fact]
		public void CesiumGeoreferenceComponent_LocalOriginControlsRoundTrip()
		{
			var georeference = new CesiumGeoreferenceComponent
			{
				OriginLongitudeDegrees = -75.612094d,
				OriginLatitudeDegrees = 40.042531d,
				OriginHeightMeters = 20d,
				LocalOrigin = new Vector3(125f, 8f, -42f),
			};

			var ecef = georeference.ToEarthCenteredEarthFixed(georeference.LocalOrigin);
			var local = georeference.ResolveGeoreference().ToLocalPosition(ecef);

			Assert.InRange((local - georeference.LocalOrigin).Length(), 0f, 0.001f);
			Assert.True(georeference.Revision > 0);
		}

		/// <summary>
		/// Verifies that an anchor resolves tangent-frame offsets against the configured georeference.
		/// </summary>
		[Fact]
		public void CesiumGlobeAnchorComponent_ApplyToPositionsEntity()
		{
			var georeference = new CesiumGeoreferenceComponent
			{
				OriginLongitudeDegrees = -75.612094d,
				OriginLatitudeDegrees = 40.042531d,
				OriginHeightMeters = 20d,
			};
			var anchor = new CesiumGlobeAnchorComponent
			{
				LongitudeDegrees = georeference.OriginLongitudeDegrees,
				LatitudeDegrees = georeference.OriginLatitudeDegrees,
				HeightMeters = georeference.OriginHeightMeters,
				LocalOffset = new Vector3(12f, 3f, -7f),
			};
			var entity = new Entity("Anchor");

			anchor.ApplyTo(entity, georeference);

			georeference.ResolveEastUpNorthAxes(
				georeference.OriginLongitudeDegrees,
				georeference.OriginLatitudeDegrees,
				out var east,
				out var up,
				out var north);
			var expectedPosition = (east * 12f) + (up * 3f) + (north * -7f);
			var actualUp = Vector3.Normalize(Vector3.Transform(Vector3.UnitY, entity.Transform.Rotation));
			var actualForward = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, entity.Transform.Rotation));

			Assert.InRange((entity.Transform.Position - expectedPosition).Length(), 0f, 0.01f);
			Assert.InRange(Vector3.Distance(actualUp, up), 0f, 0.001f);
			Assert.InRange(Vector3.Distance(actualForward, north), 0f, 0.001f);
		}

		/// <summary>
		/// Verifies that anchor orientation follows the local east/up/north frame.
		/// </summary>
		[Fact]
		public void CesiumGlobeAnchorComponent_ResolveLocalRotationAlignsUpAxis()
		{
			var georeference = new CesiumGeoreferenceComponent
			{
				OriginLongitudeDegrees = -75.612094d,
				OriginLatitudeDegrees = 40.042531d,
				OriginHeightMeters = 20d,
			};
			var anchor = new CesiumGlobeAnchorComponent
			{
				LongitudeDegrees = -75.592094d,
				LatitudeDegrees = 40.052531d,
				HeightMeters = 45d,
			};

			var rotation = anchor.ResolveLocalRotation(georeference);
			georeference.ResolveEastUpNorthAxes(anchor.LongitudeDegrees, anchor.LatitudeDegrees, out _, out var expectedUp, out var expectedNorth);
			var actualUp = Vector3.Normalize(Vector3.Transform(Vector3.UnitY, rotation));
			var actualForward = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, rotation));

			Assert.InRange(Vector3.Distance(actualUp, expectedUp), 0f, 0.001f);
			Assert.InRange(Vector3.Distance(actualForward, expectedNorth), 0f, 0.001f);
		}
	}
}
