using System.Collections.Generic;
using System.IO;
using System.IO.Compression;
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

    [Fact]
    public void Load_UsesOnlyHighestDetailMeshPerObject()
    {
        string path = Path.Combine(Path.GetTempPath(), $"{Path.GetRandomFileName()}.dts");
        File.WriteAllBytes(
            path,
            BuildMinimalDts(
                version: 26,
                useStripPrimitive: false,
                meshOffsets: [Vector3.Zero, new Vector3(10f, 0f, 0f)]));

        try
        {
            ColladaMesh mesh = Assert.Single(DtsLoader.Load(path));

            Assert.Equal(new Vector3(0f, 0f, 0f), mesh.Vertices[0].Position);
            Assert.DoesNotContain(mesh.Vertices, vertex => vertex.Position.X >= 10f);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void Load_AppliesNodeTranslationToObjectMesh()
    {
        string path = Path.Combine(Path.GetTempPath(), $"{Path.GetRandomFileName()}.dts");
        File.WriteAllBytes(
            path,
            BuildMinimalDts(
                version: 26,
                useStripPrimitive: false,
                nodeTranslation: new Vector3(5f, 6f, 7f)));

        try
        {
            ColladaMesh mesh = Assert.Single(DtsLoader.Load(path));

            Assert.True(mesh.HasBakedTransform);
            Assert.Equal(new Vector3(5f, 6f, 7f), mesh.Vertices[0].Position);
            Assert.Equal(new Vector3(6f, 6f, 7f), mesh.Vertices[1].Position);
            Assert.Equal(new Vector3(5f, 7f, 7f), mesh.Vertices[2].Position);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void Load_AppliesNodeRotationToObjectMesh()
    {
        string path = Path.Combine(Path.GetTempPath(), $"{Path.GetRandomFileName()}.dts");
        File.WriteAllBytes(
            path,
            BuildMinimalDts(
                version: 26,
                useStripPrimitive: false,
                nodeRotation: Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI * 0.5f)));

        try
        {
            ColladaMesh mesh = Assert.Single(DtsLoader.Load(path));

            Assert.True(mesh.HasBakedTransform);
            Assert.True(Vector3.Distance(Vector3.Zero, mesh.Vertices[0].Position) < 0.001f);
            Assert.True(Vector3.Distance(new Vector3(0f, 1f, 0f), mesh.Vertices[1].Position) < 0.001f);
            Assert.True(Vector3.Distance(new Vector3(-1f, 0f, 0f), mesh.Vertices[2].Position) < 0.001f);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void Load_FgxDts_ContainsExpectedBodyMeshes()
    {
        string zipPath = Path.Combine(GetRepositoryRoot(), "LibreRally", "Resources", "BeamNG Vehicles", "keciles_fgx_sedan.zip");
        string tempPath = Path.Combine(Path.GetTempPath(), $"{Path.GetRandomFileName()}.dts");

        using (ZipArchive archive = ZipFile.OpenRead(zipPath))
        {
            ZipArchiveEntry entry = archive.GetEntry("vehicles/fgx_sedan/fgx.dts")
                ?? throw new FileNotFoundException("Could not find fgx.dts in the FGX vehicle archive.", zipPath);
            using Stream input = entry.Open();
            using FileStream output = File.Create(tempPath);
            input.CopyTo(output);
        }

        try
        {
            List<ColladaMesh> meshes = DtsLoader.Load(tempPath);

            Assert.NotEmpty(meshes);
            Assert.Contains(meshes, mesh => mesh.GeometryName.Contains("fgx_body", StringComparison.OrdinalIgnoreCase));
            Assert.Contains(meshes, mesh => mesh.GeometryName.Contains("fgx_interior", StringComparison.OrdinalIgnoreCase));
        }
        finally
        {
            File.Delete(tempPath);
        }
    }

    private static string GetRepositoryRoot()
    {
        DirectoryInfo? directory = new(AppContext.BaseDirectory);
        while (directory != null)
        {
            string candidate = Path.Combine(directory.FullName, "LibreRally.sln");
            if (File.Exists(candidate))
            {
                return directory.FullName;
            }

            directory = directory.Parent;
        }

        throw new DirectoryNotFoundException("Could not locate the repository root for DTS tests.");
    }

    private static byte[] BuildMinimalDts(
        short version,
        bool useStripPrimitive,
        Vector3? nodeTranslation = null,
        Quaternion? nodeRotation = null,
        IReadOnlyList<Vector3>? meshOffsets = null)
    {
        var data32 = new List<uint>();
        var data16 = new List<ushort>();
        var data8 = new List<byte>();
        int guard = 0;
        bool useNodeTransform = nodeTranslation.HasValue || nodeRotation.HasValue;
        var objectMeshOffsets = meshOffsets is { Count: > 0 }
            ? new List<Vector3>(meshOffsets)
            : [Vector3.Zero];
        int objectMeshCount = objectMeshOffsets.Count;
        int numSubShapes = objectMeshCount > 1 ? 1 : 0;
        int numDetails = objectMeshCount;
        int bodyNameIndex = 0;
        int firstDetailNameIndex = 1;
        int rootNameIndex = firstDetailNameIndex + numDetails;
        int numNames = 1 + numDetails + (useNodeTransform ? 1 : 0);

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

        static void AddQuat16(List<ushort> list, Quaternion rotation)
        {
            var normalized = rotation.LengthSquared() > 1e-8f
                ? Quaternion.Normalize(rotation)
                : Quaternion.Identity;

            list.Add(unchecked((ushort)(short)MathF.Round(normalized.X * 32767f)));
            list.Add(unchecked((ushort)(short)MathF.Round(normalized.Y * 32767f)));
            list.Add(unchecked((ushort)(short)MathF.Round(normalized.Z * 32767f)));
            list.Add(unchecked((ushort)(short)MathF.Round(normalized.W * 32767f)));
        }

        void AddGuard()
        {
            AddS32(data32, guard);
            data16.Add((ushort)guard);
            data8.Add((byte)guard);
            guard++;
        }

        void AddStandardMesh(Vector3 offset)
        {
            AddS32(data32, 0); // mesh type (standard)
            AddGuard();

            int vertexCount = useStripPrimitive ? 4 : 3;
            AddS32(data32, 1); // numFrames
            AddS32(data32, 1); // numMatFrames
            AddS32(data32, -1); // parentMesh
            AddPoint3(data32, offset.X, offset.Y, offset.Z); // bounds.min
            AddPoint3(data32, offset.X + 1f, offset.Y + 1f, offset.Z); // bounds.max
            AddPoint3(data32, offset.X + 0.5f, offset.Y + 0.5f, offset.Z); // center
            AddF32(data32, 1f); // radius
            AddS32(data32, vertexCount); // numVerts
            AddPoint3(data32, offset.X + 0f, offset.Y + 0f, offset.Z + 0f);
            AddPoint3(data32, offset.X + 1f, offset.Y + 0f, offset.Z + 0f);
            AddPoint3(data32, offset.X + 0f, offset.Y + 1f, offset.Z + 0f);
            if (useStripPrimitive)
            {
                AddPoint3(data32, offset.X + 1f, offset.Y + 1f, offset.Z + 0f);
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
            while ((data16.Count & 1) != 0)
            {
                data16.Add(0);
            }

            AddS32(data32, vertexCount); // vertsPerFrame
            AddS32(data32, 0); // flags
            AddGuard();
        }

        AddS32(data32, useNodeTransform ? 1 : 0); // numNodes
        AddS32(data32, 1); // numObjects
        AddS32(data32, 0); // numDecals
        AddS32(data32, numSubShapes); // numSubShapes
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
        AddS32(data32, numDetails); // numDetails
        AddS32(data32, objectMeshCount); // numMeshes
        AddS32(data32, numNames); // numNames
        AddF32(data32, 1f); // smallestVisibleSize
        AddS32(data32, 0); // smallestVisibleDL
        AddGuard();

        AddF32(data32, 1f); // radius
        AddF32(data32, 1f); // tubeRadius
        AddPoint3(data32, 0.5f, 0.5f, 0f); // center
        AddPoint3(data32, 0f, 0f, 0f); // bounds.min
        AddPoint3(data32, 1f, 1f, 0f); // bounds.max
        AddGuard();

        if (useNodeTransform)
        {
            AddS32(data32, rootNameIndex); // node name index -> "root"
            AddS32(data32, -1); // parent index
            AddS32(data32, -1); // first object
            AddS32(data32, -1); // first child
            AddS32(data32, -1); // next sibling
        }
        AddGuard(); // nodes

        AddS32(data32, bodyNameIndex); // object name index -> "body"
        AddS32(data32, objectMeshCount); // num meshes
        AddS32(data32, 0); // start mesh index
        AddS32(data32, useNodeTransform ? 0 : -1); // node index
        AddS32(data32, -1); // sibling
        AddS32(data32, -1); // first decal
        AddGuard();

        AddGuard(); // decals
        AddGuard(); // ifl
        if (numSubShapes > 0)
        {
            AddS32(data32, 0); // first node
            AddS32(data32, 0); // first object
            AddS32(data32, 0); // first decal (deprecated)
        }
        AddGuard(); // subshape first-node/object/decal
        if (numSubShapes > 0)
        {
            AddS32(data32, useNodeTransform ? 1 : 0); // num nodes
            AddS32(data32, 1); // num objects
            AddS32(data32, 0); // num decals (deprecated)
        }
        AddGuard(); // subshape counts
        if (useNodeTransform)
        {
            AddQuat16(data16, nodeRotation ?? Quaternion.Identity);
            Vector3 translation = nodeTranslation ?? Vector3.Zero;
            AddPoint3(data32, translation.X, translation.Y, translation.Z);
        }
        AddGuard(); // default rot/trans etc.
        AddGuard(); // node scales
        AddGuard(); // ground frames
        AddGuard(); // object states
        AddGuard(); // decal states
        AddGuard(); // triggers

        for (int detailIndex = 0; detailIndex < numDetails; detailIndex++)
        {
            AddS32(data32, firstDetailNameIndex + detailIndex); // detail name index
            AddS32(data32, 0); // subshape
            AddS32(data32, detailIndex); // object detail num
            AddF32(data32, numDetails - detailIndex); // size
            AddF32(data32, 0f); // avg error
            AddF32(data32, 0f); // max error
            AddS32(data32, 1); // poly count
            if (version >= 26)
            {
                AddS32(data32, 0);
                AddS32(data32, 0);
                AddS32(data32, 0);
                AddS32(data32, 0);
                AddF32(data32, 0f);
                AddS32(data32, 0);
            }
        }
        AddGuard();

        foreach (Vector3 meshOffset in objectMeshOffsets)
        {
            AddStandardMesh(meshOffset);
        }

        AddGuard(); // after meshes

        data8.AddRange(System.Text.Encoding.UTF8.GetBytes("body"));
        data8.Add(0);
        for (int detailIndex = 0; detailIndex < numDetails; detailIndex++)
        {
            data8.AddRange(System.Text.Encoding.UTF8.GetBytes($"detail{detailIndex}"));
            data8.Add(0);
        }
        if (useNodeTransform)
        {
            data8.AddRange(System.Text.Encoding.UTF8.GetBytes("root"));
            data8.Add(0);
        }
        AddGuard();

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
        writer.Write((byte)1); // MaterialList binary format version
        writer.Write(0u); // numMaterials

        writer.Flush();
        return stream.ToArray();
    }
}
