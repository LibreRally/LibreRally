using System.IO;
using System.Linq;
using System.Numerics;
using LibreRally.Vehicle.Rendering;

namespace LibreRally.Tests
{
	public class ColladaLoaderTests
	{
		[Fact]
		public void Load_UsesVisualSceneInstancesAndSkipsUninstancedGeometry()
		{
			string path = WriteTempDae(
				$$"""
				  <?xml version="1.0" encoding="utf-8"?>
				  <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
				    <library_geometries>
				      {{BuildTriangleGeometry("instanced-mesh", "instanced_part")}}
				      {{BuildTriangleGeometry("unused-mesh", "unused_part")}}
				    </library_geometries>
				    <library_visual_scenes>
				      <visual_scene id="Scene" name="Scene">
				        <node id="instanced_node" name="instanced_node" type="NODE">
				          <matrix sid="transform">1 0 0 5 0 1 0 6 0 0 1 7 0 0 0 1</matrix>
				          <instance_geometry url="#instanced-mesh" />
				        </node>
				      </visual_scene>
				    </library_visual_scenes>
				    <scene>
				      <instance_visual_scene url="#Scene" />
				    </scene>
				  </COLLADA>
				  """);

			try
			{
				List<ColladaMesh> meshes = ColladaLoader.Load(path);

				ColladaMesh mesh = Assert.Single(meshes);
				Assert.Equal("instanced_part", mesh.GeometryName);
				Assert.Equal("instanced_node", mesh.SceneNodeName);
				Assert.True(mesh.HasBakedTransform);
				Assert.Equal(new Vector3(5f, 6f, 7f), mesh.Vertices[0].Position);
				Assert.DoesNotContain(meshes, candidate => candidate.GeometryName == "unused_part");
			}
			finally
			{
				File.Delete(path);
			}
		}

		[Fact]
		public void Load_ComposesParentAndChildNodeTransforms()
		{
			string path = WriteTempDae(
				$$"""
				  <?xml version="1.0" encoding="utf-8"?>
				  <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
				    <library_geometries>
				      {{BuildTriangleGeometry("nested-mesh", "nested_part")}}
				    </library_geometries>
				    <library_visual_scenes>
				      <visual_scene id="Scene" name="Scene">
				        <node id="parent_node" name="parent_node" type="NODE">
				          <matrix sid="transform">1 0 0 1 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
				          <node id="child_node" name="child_node" type="NODE">
				            <matrix sid="transform">1 0 0 0 0 1 0 2 0 0 1 0 0 0 0 1</matrix>
				            <instance_geometry url="#nested-mesh" />
				          </node>
				        </node>
				      </visual_scene>
				    </library_visual_scenes>
				    <scene>
				      <instance_visual_scene url="#Scene" />
				    </scene>
				  </COLLADA>
				  """);

			try
			{
				ColladaMesh mesh = Assert.Single(ColladaLoader.Load(path));

				Assert.Equal("child_node", mesh.SceneNodeName);
				Assert.Equal(new Vector3(1f, 2f, 0f), mesh.Vertices[0].Position);
			}
			finally
			{
				File.Delete(path);
			}
		}

		[Fact]
		public void Load_FallsBackToRawGeometry_WhenVisualSceneIsMissing()
		{
			string path = WriteTempDae(
				$$"""
				  <?xml version="1.0" encoding="utf-8"?>
				  <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
				    <library_geometries>
				      {{BuildTriangleGeometry("raw-mesh", "raw_part")}}
				    </library_geometries>
				  </COLLADA>
				  """);

			try
			{
				ColladaMesh mesh = Assert.Single(ColladaLoader.Load(path));

				Assert.Equal("raw_part", mesh.GeometryName);
				Assert.False(mesh.HasBakedTransform);
				Assert.Equal(Vector3.Zero, mesh.Vertices[0].Position);
			}
			finally
			{
				File.Delete(path);
			}
		}

		[Fact]
		public void Load_FlipsRawGeometryTriangleWindingForStrideFrontFaces()
		{
			string path = WriteTempDae(
				$$"""
				  <?xml version="1.0" encoding="utf-8"?>
				  <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
				    <library_geometries>
				      {{BuildTriangleGeometry("raw-winding-mesh", "raw_winding_part")}}
				    </library_geometries>
				  </COLLADA>
				  """);

			try
			{
				ColladaMesh mesh = Assert.Single(ColladaLoader.Load(path));

				Assert.Equal(new[] { 0, 2, 1 }, mesh.Indices);
			}
			finally
			{
				File.Delete(path);
			}
		}

		[Fact]
		public void Load_NegativeDeterminantSceneTransform_FlipsWindingBack()
		{
			string path = WriteTempDae(
				$$"""
				  <?xml version="1.0" encoding="utf-8"?>
				  <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
				    <library_geometries>
				      {{BuildTriangleGeometry("mirrored-mesh", "mirrored_part")}}
				    </library_geometries>
				    <library_visual_scenes>
				      <visual_scene id="Scene" name="Scene">
				        <node id="mirrored_node" name="mirrored_node" type="NODE">
				          <matrix sid="transform">-1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1</matrix>
				          <instance_geometry url="#mirrored-mesh" />
				        </node>
				      </visual_scene>
				    </library_visual_scenes>
				    <scene>
				      <instance_visual_scene url="#Scene" />
				    </scene>
				  </COLLADA>
				  """);

			try
			{
				ColladaMesh mesh = Assert.Single(ColladaLoader.Load(path));

				Assert.Equal(new[] { 0, 1, 2 }, mesh.Indices);
				Assert.Equal(new Vector3(-1f, 0f, 0f), mesh.Vertices[1].Position);
			}
			finally
			{
				File.Delete(path);
			}
		}

		private static string WriteTempDae(string content)
		{
			string path = Path.Combine(Path.GetTempPath(), $"{Path.GetRandomFileName()}.dae");
			File.WriteAllText(path, content);
			return path;
		}

		private static string BuildTriangleGeometry(string geometryId, string geometryName)
			=> $@"<geometry id=""{geometryId}"" name=""{geometryName}"">
  <mesh>
    <source id=""{geometryId}-positions"">
      <float_array id=""{geometryId}-positions-array"" count=""9"">0 0 0 1 0 0 0 1 0</float_array>
    </source>
    <source id=""{geometryId}-normals"">
      <float_array id=""{geometryId}-normals-array"" count=""9"">0 0 1 0 0 1 0 0 1</float_array>
    </source>
    <source id=""{geometryId}-uvs"">
      <float_array id=""{geometryId}-uvs-array"" count=""6"">0 0 1 0 0 1</float_array>
    </source>
    <vertices id=""{geometryId}-vertices"">
      <input semantic=""POSITION"" source=""#{geometryId}-positions"" />
    </vertices>
    <triangles count=""1"" material=""body"">
      <input semantic=""VERTEX"" source=""#{geometryId}-vertices"" offset=""0"" />
      <input semantic=""NORMAL"" source=""#{geometryId}-normals"" offset=""1"" />
      <input semantic=""TEXCOORD"" source=""#{geometryId}-uvs"" offset=""2"" />
      <p>0 0 0 1 1 1 2 2 2</p>
    </triangles>
  </mesh>
</geometry>";
	}
}
