using System.Collections.Generic;
using System.IO;
using System.Numerics;
using LibreRally.Vehicle.Rendering;

namespace LibreRally.Tests;

public class DtsLoaderTests
{
    [Fact]
    public void Load_ReadsBasicV24StaticMesh()
    {
        string path = Path.Combine(Path.GetTempPath(), $"{Path.GetRandomFileName()}.dts");
        File.WriteAllBytes(path, BuildMinimalDts(version: 24, useStripPrimitive: false));

        try
        {
            ColladaMesh mesh = Assert.Single(DtsLoader.Load(path));
            Assert.Equal("body", mesh.GeometryName);
            Assert.Equal("body", mesh.SceneNodeName);
            Assert.Equal(3, mesh.Vertices.Count);
            Assert.Equal(3, mesh.Indices.Count);
            Assert.Equal(new Vector3(0f, 1f, 0f), mesh.Vertices[2].Position);
            Assert.Equal(new Vector2(0f, 1f), mesh.Vertices[2].TexCoord);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void Load_ReadsBasicV26StaticMesh()
    {
        string path = Path.Combine(Path.GetTempPath(), $"{Path.GetRandomFileName()}.dts");
        File.WriteAllBytes(path, BuildMinimalDts(version: 26, useStripPrimitive: false));

        try
        {
            ColladaMesh mesh = Assert.Single(DtsLoader.Load(path));
            Assert.Equal("body", mesh.GeometryName);
            Assert.Equal(3, mesh.Vertices.Count);
            Assert.Equal(new Vector3(1f, 0f, 0f), mesh.Vertices[1].Position);
            Assert.Equal(new Vector2(1f, 0f), mesh.Vertices[1].TexCoord);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void Load_ReadsStripPrimitiveWithoutDuplicatingVertices()
    {
        string path = Path.Combine(Path.GetTempPath(), $"{Path.GetRandomFileName()}.dts");
        File.WriteAllBytes(path, BuildMinimalDts(version: 26, useStripPrimitive: true));

        try
        {
            ColladaMesh mesh = Assert.Single(DtsLoader.Load(path));
            Assert.Equal(4, mesh.Vertices.Count);
            Assert.Equal(6, mesh.Indices.Count);
            Assert.Equal([0, 1, 2, 2, 1, 3], mesh.Indices);
        }
        finally
        {
            File.Delete(path);
        }
    }

    private static byte[] BuildMinimalDts(short version, bool useStripPrimitive)
    {
        var data32 = new List<uint>();
        var data16 = new List<ushort>();
        var data8 = new List<byte>();
        int guard = 0;

        static void AddS32(List<uint> list, int value) => list.Add(unchecked((uint)value));
        static void AddF32(List<uint> list, float value) => list.Add(unchecked((uint)BitConverter.SingleToInt32Bits(value)));
        static void AddPoint3(List<uint> list, float x, float y, float z)
        {
            AddF32(list, x);
            AddF32(list, y);
            AddF32(list, z);
        }

        static void AddPoint2(List<uint> list, float x, float y)
        {
            AddF32(list, x);
            AddF32(list, y);
        }

        void AddGuard()
        {
            AddS32(data32, guard);
            data16.Add((ushort)guard);
            data8.Add((byte)guard);
            guard++;
        }

        AddS32(data32, 0); // numNodes
        AddS32(data32, 1); // numObjects
        AddS32(data32, 0); // numDecals
        AddS32(data32, 0); // numSubShapes
        AddS32(data32, 0); // numIFLs
        AddS32(data32, 0); // numNodeRotations
        AddS32(data32, 0); // numNodeTranslations
        AddS32(data32, 0); // numNodeUniformScales
        AddS32(data32, 0); // numNodeAlignedScales
        AddS32(data32, 0); // numNodeArbScales
        AddS32(data32, 0); // numGroundFrames
        AddS32(data32, 0); // numObjectStates
        AddS32(data32, 0); // numDecalStates
        AddS32(data32, 0); // numTriggers
        AddS32(data32, 1); // numDetails
        AddS32(data32, 1); // numMeshes
        AddS32(data32, 2); // numNames
        AddF32(data32, 1f); // smallestVisibleSize
        AddS32(data32, 0); // smallestVisibleDL
        AddGuard();

        AddF32(data32, 1f); // radius
        AddF32(data32, 1f); // tubeRadius
        AddPoint3(data32, 0.5f, 0.5f, 0f); // center
        AddPoint3(data32, 0f, 0f, 0f); // bounds.min
        AddPoint3(data32, 1f, 1f, 0f); // bounds.max
        AddGuard();

        AddGuard(); // nodes

        AddS32(data32, 0); // object name index -> "body"
        AddS32(data32, 1); // num meshes
        AddS32(data32, 0); // start mesh index
        AddS32(data32, -1); // node index
        AddS32(data32, -1); // sibling
        AddS32(data32, -1); // first decal
        AddGuard();

        AddGuard(); // decals
        AddGuard(); // ifl
        AddGuard(); // subshapes
        AddGuard(); // default rot/trans etc.
        AddGuard(); // node scales
        AddGuard(); // ground frames
        AddGuard(); // object states
        AddGuard(); // decal states
        AddGuard(); // triggers

        AddS32(data32, 1); // detail name index
        AddS32(data32, 0); // subshape
        AddS32(data32, 0); // object detail num
        AddF32(data32, 1f); // size
        AddF32(data32, 0f); // avg error
        AddF32(data32, 0f); // max error
        AddS32(data32, 1); // poly count
        if (version >= 26)
        {
            AddS32(data32, -1);
            AddS32(data32, -1);
            AddS32(data32, -1);
            AddS32(data32, 0);
            AddF32(data32, 0f);
            data8.Add(0);
        }
        AddGuard();

        AddS32(data32, 0); // mesh type (standard)
        AddGuard();

        int vertexCount = useStripPrimitive ? 4 : 3;
        AddS32(data32, 1); // numFrames
        AddS32(data32, 1); // numMatFrames
        AddS32(data32, -1); // parentMesh
        AddPoint3(data32, 0f, 0f, 0f); // bounds.min
        AddPoint3(data32, 1f, 1f, 0f); // bounds.max
        AddPoint3(data32, 0.5f, 0.5f, 0f); // center
        AddF32(data32, 1f); // radius
        AddS32(data32, vertexCount); // numVerts
        AddPoint3(data32, 0f, 0f, 0f);
        AddPoint3(data32, 1f, 0f, 0f);
        AddPoint3(data32, 0f, 1f, 0f);
        if (useStripPrimitive)
        {
            AddPoint3(data32, 1f, 1f, 0f);
        }

        AddS32(data32, vertexCount); // numTVerts
        AddPoint2(data32, 0f, 0f);
        AddPoint2(data32, 1f, 0f);
        AddPoint2(data32, 0f, 1f);
        if (useStripPrimitive)
        {
            AddPoint2(data32, 1f, 1f);
        }
        if (version >= 26)
        {
            AddS32(data32, 0); // numTVerts2
            AddS32(data32, 0); // numVColors
        }
        AddPoint3(data32, 0f, 0f, 1f);
        AddPoint3(data32, 0f, 0f, 1f);
        AddPoint3(data32, 0f, 0f, 1f);
        if (useStripPrimitive)
        {
            AddPoint3(data32, 0f, 0f, 1f);
        }
        data8.Add(0);
        data8.Add(0);
        data8.Add(0); // encoded norms
        if (useStripPrimitive)
        {
            data8.Add(0); // encoded norms
        }
        AddS32(data32, 1); // numPrimitives
        int primitiveCount = useStripPrimitive ? 4 : 3;
        if (version <= 24)
        {
            data16.Add(0); // primitive start
            data16.Add((ushort)primitiveCount); // primitive count
            AddS32(data32, useStripPrimitive ? 3 : 2); // primitive max index (v24)
        }
        else
        {
            AddS32(data32, 0); // primitive start
            AddS32(data32, primitiveCount); // primitive count
            AddS32(data32, useStripPrimitive ? unchecked((int)0x40000000u) : 0); // primitive material
        }
        AddS32(data32, primitiveCount); // numIndices
        if (version <= 24)
        {
            data16.Add(0);
            data16.Add(1);
            data16.Add(2);
            if (useStripPrimitive)
            {
                data16.Add(3);
            }
        }
        else
        {
            AddS32(data32, 0);
            AddS32(data32, 1);
            AddS32(data32, 2);
            if (useStripPrimitive)
            {
                AddS32(data32, 3);
            }
        }
        AddS32(data32, 0); // numMergeIndices
        AddS32(data32, vertexCount); // vertsPerFrame
        AddS32(data32, 0); // flags
        AddGuard();

        AddGuard(); // after meshes

        data8.AddRange(System.Text.Encoding.UTF8.GetBytes("body"));
        data8.Add(0);
        data8.AddRange(System.Text.Encoding.UTF8.GetBytes("detail"));
        data8.Add(0);
        AddGuard();

        AddF32(data32, 1f); // alphaIn
        AddF32(data32, 1f); // alphaOut

        while ((data16.Count * 2) % 4 != 0)
        {
            data16.Add(0);
        }

        while (data8.Count % 4 != 0)
        {
            data8.Add(0);
        }

        int start16 = data32.Count;
        int start8 = start16 + (data16.Count * 2) / 4;
        int sizeAll = start8 + data8.Count / 4;

        using var stream = new MemoryStream();
        using var writer = new BinaryWriter(stream);
        writer.Write(version);
        writer.Write((short)1);
        writer.Write(sizeAll);
        writer.Write(start16);
        writer.Write(start8);
        foreach (uint value in data32) writer.Write(value);
        foreach (ushort value in data16) writer.Write(value);
        writer.Write(data8.ToArray());

        writer.Write(0); // numSequences
        writer.Write((sbyte)1); // matStreamType
        writer.Write(0); // numMaterials

        writer.Flush();
        return stream.ToArray();
    }
}
