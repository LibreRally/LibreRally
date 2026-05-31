using Stride.Core.Mathematics;
using Stride.Graphics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies Cesium triangle expansion helpers used by the runtime importer.
	/// </summary>
	public class CesiumTilesetGeometryTests
	{
		/// <summary>
		/// Verifies that invalid source indices are discarded during triangle expansion.
		/// </summary>
		[Fact]
		public void ExpandTriangleVertices_InvalidTriangle_IsSkipped()
		{
			var vertices = new[]
			{
				new VertexPositionNormalTexture(new Vector3(0f, 0f, 0f), Vector3.UnitY, Vector2.Zero),
				new VertexPositionNormalTexture(new Vector3(1f, 0f, 0f), Vector3.UnitY, Vector2.Zero),
				new VertexPositionNormalTexture(new Vector3(0f, 1f, 0f), Vector3.UnitY, Vector2.Zero),
			};

			var expanded = CesiumTilesetScript.ExpandTriangleVertices(vertices, [0, 1, 2, 0, 2, 99]);

			Assert.Equal(3, expanded.Length);
			Assert.Equal(vertices[0].Position, expanded[0].Position);
			Assert.Equal(vertices[2].Position, expanded[1].Position);
			Assert.Equal(vertices[1].Position, expanded[2].Position);
		}

		/// <summary>
		/// Verifies that expanded Cesium triangles flip winding to match Stride front faces.
		/// </summary>
		[Fact]
		public void ExpandTriangleVertices_FlipsTriangleWinding()
		{
			var vertices = new[]
			{
				new VertexPositionNormalTexture(new Vector3(0f, 0f, 0f), Vector3.UnitZ, Vector2.Zero),
				new VertexPositionNormalTexture(new Vector3(1f, 0f, 0f), Vector3.UnitZ, Vector2.Zero),
				new VertexPositionNormalTexture(new Vector3(0f, 1f, 0f), Vector3.UnitZ, Vector2.Zero),
			};

			var expanded = CesiumTilesetScript.ExpandTriangleVertices(vertices, [0, 1, 2]);

			Assert.Equal(vertices[0].Position, expanded[0].Position);
			Assert.Equal(vertices[2].Position, expanded[1].Position);
			Assert.Equal(vertices[1].Position, expanded[2].Position);
		}

		/// <summary>
		/// Verifies that sequential indices cover every expanded vertex exactly once.
		/// </summary>
		[Fact]
		public void CreateSequentialIndices_ReturnsZeroBasedSequence()
		{
			var indices = CesiumTilesetScript.CreateSequentialIndices(4);

			Assert.Equal([0, 1, 2, 3], indices);
		}

		/// <summary>
		/// Verifies that the 3D Tiles y-up to z-up matrix matches the spec transform.
		/// </summary>
		[Fact]
		public void YUpToZUp_RotatesGltfVectorIntoTileSpace()
		{
			var converted = CesiumMatrixUtilities.TransformPoint(
				CesiumMatrixUtilities.YUpToZUp(),
				new CesiumDouble3(10d, 20d, 30d));

			Assert.Equal(new CesiumDouble3(10d, -30d, 20d), converted);
		}

		/// <summary>
		/// Verifies that normalized unsigned short UVs (common in Cesium photogrammetry) are correctly denormalized to [0, 1].
		/// </summary>
		[Fact]
		public void ReadVector2_DenormalizesUnsignedShortUVs()
		{
			var ushortMax = 65535u;
			var halfMax = ushortMax / 2u;
			var buffer = new ushort[] { 0, (ushort)ushortMax, (ushort)halfMax, (ushort)(ushortMax * 3 / 4) };
			var handle = System.Runtime.InteropServices.GCHandle.Alloc(buffer, System.Runtime.InteropServices.GCHandleType.Pinned);
			try
			{
				unsafe
				{
					var accessor = new Evergine.Bindings.CesiumNative.AccessorData
					{
						Data = (void*)handle.AddrOfPinnedObject(),
						Stride = (nuint)(sizeof(ushort) * 2),
						Count = 2,
						ComponentType = 5123, // UnsignedShort
						NumberOfComponents = 2,
						ByteLength = (nuint)(sizeof(ushort) * 4),
					};

					Assert.Equal(new Vector2(0f, 1f), CesiumAccessorReader.ReadVector2(accessor, 0));
					Assert.Equal(new Vector2(halfMax / 65535f, (ushort)(ushortMax * 3 / 4) / 65535f), CesiumAccessorReader.ReadVector2(accessor, 1));
				}
			}
			finally
			{
				handle.Free();
			}
		}

		/// <summary>
		/// Verifies that float UV accessors pass through unchanged.
		/// </summary>
		[Fact]
		public void ReadVector2_KeepsFloatUvsUnchanged()
		{
			var buffer = new[] { 0.700121f, 0.467925f, 0.176998f, 0.856657f };
			var handle = System.Runtime.InteropServices.GCHandle.Alloc(buffer, System.Runtime.InteropServices.GCHandleType.Pinned);
			try
			{
				unsafe
				{
					var accessor = new Evergine.Bindings.CesiumNative.AccessorData
					{
						Data = (void*)handle.AddrOfPinnedObject(),
						Stride = (nuint)(sizeof(float) * 2),
						Count = 2,
						ComponentType = 5126,
						NumberOfComponents = 2,
						ByteLength = (nuint)(sizeof(float) * 4),
					};

					Assert.Equal(new Vector2(0.700121f, 0.467925f), CesiumAccessorReader.ReadVector2(accessor, 0));
					Assert.Equal(new Vector2(0.176998f, 0.856657f), CesiumAccessorReader.ReadVector2(accessor, 1));
				}
			}
			finally
			{
				handle.Free();
			}
		}

		/// <summary>
		/// Verifies that requested texture-coordinate sets resolve to the matching glTF semantic.
		/// </summary>
		[Theory]
		[InlineData(-1, "TEXCOORD_0")]
		[InlineData(0, "TEXCOORD_0")]
		[InlineData(1, "TEXCOORD_1")]
		[InlineData(2, "TEXCOORD_2")]
		public void ResolveTextureCoordinateSemantic_UsesExpectedSemantic(int texCoordSet, string expectedSemantic)
		{
			Assert.Equal(expectedSemantic, CesiumTilesetScript.ResolveTextureCoordinateSemantic(texCoordSet));
		}
	}
}
