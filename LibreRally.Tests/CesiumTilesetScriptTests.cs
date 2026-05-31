using Stride.Core.Mathematics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies Cesium tileset runtime configuration helpers and matrix utilities.
	/// </summary>
	public sealed class CesiumTilesetScriptTests
	{
		/// <summary>
		/// Verifies that URL datasets fall back to the public sample when no URL is supplied.
		/// </summary>
		[Fact]
		public void ResolveRuntimeConfiguration_UsesSampleUrlFallback()
		{
			var configuration = CesiumTilesetScript.ResolveRuntimeConfiguration(
				CesiumTilesetDataSource.Url,
				null,
				null,
				0,
				null,
				18f,
				8,
				1024);

			Assert.Equal(CesiumTilesetDataSource.Url, configuration.DataSource);
			Assert.Contains("tileset.json", configuration.SourceIdentifier, StringComparison.OrdinalIgnoreCase);
		}

		/// <summary>
		/// Verifies that local file datasets are normalized into absolute file URIs.
		/// </summary>
		[Fact]
		public void ResolveRuntimeConfiguration_NormalizesLocalFilePath()
		{
			var configuration = CesiumTilesetScript.ResolveRuntimeConfiguration(
				CesiumTilesetDataSource.LocalFile,
				null,
				@".\Resources\Cesium\tileset.json",
				0,
				null,
				18f,
				8,
				1024);

			Assert.Equal(CesiumTilesetDataSource.LocalFile, configuration.DataSource);
			Assert.StartsWith("file:///", configuration.SourceIdentifier, StringComparison.OrdinalIgnoreCase);
			Assert.Contains("tileset.json", configuration.SourceIdentifier, StringComparison.OrdinalIgnoreCase);
		}

		/// <summary>
		/// Verifies that ion datasets reject missing credentials.
		/// </summary>
		[Fact]
		public void ResolveRuntimeConfiguration_RejectsMissingIonToken()
		{
			var error = Assert.Throws<InvalidOperationException>(() => CesiumTilesetScript.ResolveRuntimeConfiguration(
				CesiumTilesetDataSource.CesiumIon,
				null,
				null,
				123,
				"",
				18f,
				8,
				1024));

			Assert.Contains("access token", error.Message, StringComparison.OrdinalIgnoreCase);
		}

		/// <summary>
		/// Verifies that composed node transforms apply translation and scale in column-major order.
		/// </summary>
		[Fact]
		public void ComposeTransform_AppliesScaleAndTranslation()
		{
			var transform = CesiumMatrixUtilities.ComposeTransform(
				translationX: 10d,
				translationY: 5d,
				translationZ: -3d,
				rotationX: 0d,
				rotationY: 0d,
				rotationZ: 0d,
				rotationW: 1d,
				scaleX: 2d,
				scaleY: 3d,
				scaleZ: 4d);

			var point = CesiumMatrixUtilities.TransformPoint(transform, new CesiumDouble3(1d, 2d, 3d));

			Assert.Equal(new Vector3(12f, 11f, 9f), new Vector3((float)point.X, (float)point.Y, (float)point.Z));
		}

		/// <summary>
		/// Verifies that the 3D Tiles y-up to z-up bridge is composed after glTF node transforms.
		/// </summary>
		[Fact]
		public void YUpToZUp_ComposesAfterNodeTransform()
		{
			var nodeTransform = CesiumMatrixUtilities.ComposeTransform(
				translationX: 0d,
				translationY: 1d,
				translationZ: 0d,
				rotationX: 0d,
				rotationY: 0d,
				rotationZ: 0d,
				rotationW: 1d,
				scaleX: 1d,
				scaleY: 1d,
				scaleZ: 1d);
			var correctTransform = CesiumMatrixUtilities.Multiply(
				CesiumMatrixUtilities.YUpToZUp(),
				nodeTransform);
			var incorrectTransform = CesiumMatrixUtilities.Multiply(
				nodeTransform,
				CesiumMatrixUtilities.YUpToZUp());

			var correctPoint = CesiumMatrixUtilities.TransformPoint(correctTransform, new CesiumDouble3(0d, 0d, 0d));
			var incorrectPoint = CesiumMatrixUtilities.TransformPoint(incorrectTransform, new CesiumDouble3(0d, 0d, 0d));

			Assert.Equal(new Vector3(0f, 0f, 1f), new Vector3((float)correctPoint.X, (float)correctPoint.Y, (float)correctPoint.Z));
			Assert.Equal(new Vector3(0f, 1f, 0f), new Vector3((float)incorrectPoint.X, (float)incorrectPoint.Y, (float)incorrectPoint.Z));
		}

		/// <summary>
		/// Verifies that physics proxy bounds collapse to one merged tile box when the primitive budget is exceeded.
		/// </summary>
		[Fact]
		public void SelectPhysicsProxyBounds_CollapsesWhenPrimitiveBudgetIsExceeded()
		{
			var primitiveBounds = new[]
			{
				new BoundingBox(new Vector3(-2f, 0f, -1f), new Vector3(-1f, 1f, 1f)),
				new BoundingBox(new Vector3(2f, 0f, -3f), new Vector3(4f, 2f, -1f)),
			};

			var selectedBounds = CesiumTilesetScript.SelectPhysicsProxyBounds(primitiveBounds, 1);

			var mergedBounds = Assert.Single(selectedBounds);
			Assert.Equal(new Vector3(-2f, 0f, -3f), mergedBounds.Minimum);
			Assert.Equal(new Vector3(4f, 2f, 1f), mergedBounds.Maximum);
		}

		/// <summary>
		/// Verifies that tile physics proxies activate based on the closest point on the tile bounds.
		/// </summary>
		[Fact]
		public void ShouldEnablePhysicsProxy_UsesDistanceToTileBounds()
		{
			var tileBounds = new BoundingBox(new Vector3(10f, 0f, 10f), new Vector3(20f, 10f, 20f));

			Assert.True(CesiumTilesetScript.ShouldEnablePhysicsProxy(new Vector3(22f, 5f, 15f), tileBounds, 2.5f));
			Assert.False(CesiumTilesetScript.ShouldEnablePhysicsProxy(new Vector3(23.1f, 5f, 15f), tileBounds, 2.5f));
		}
	}
}
