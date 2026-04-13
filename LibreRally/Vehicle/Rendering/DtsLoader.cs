using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;

namespace LibreRally.Vehicle.Rendering;

/// <summary>
/// Best-effort Torque DTS mesh loader.
/// Supports static geometry extraction for common DTS v24-v26 files.
/// </summary>
public static class DtsLoader
{
    private const uint PrimitiveMaterialMask = 0x0FFFFFFF;
    private const uint PrimitiveStripFlag = 0x40000000;

    public static List<ColladaMesh> Load(string dtsFilePath)
    {
        using var stream = File.OpenRead(dtsFilePath);
        using var reader = new BinaryReader(stream);

        short version = reader.ReadInt16();
        _ = reader.ReadInt16(); // exporter version

        int sizeAll = reader.ReadInt32();
        int start16 = reader.ReadInt32();
        int start8 = reader.ReadInt32();
        if (sizeAll <= 0 || start16 < 0 || start8 < start16)
        {
            throw new InvalidDataException("Invalid DTS buffer header.");
        }

        int data32Words = start16;
        int data16Words = (start8 - start16) * 2;
        int data8Bytes = (sizeAll - start8) * 4;
        if (data16Words < 0 || data8Bytes < 0)
        {
            throw new InvalidDataException("Invalid DTS interleaved buffer sizes.");
        }

        uint[] data32 = new uint[data32Words];
        for (int i = 0; i < data32.Length; i++)
        {
            data32[i] = reader.ReadUInt32();
        }

        ushort[] data16 = new ushort[data16Words];
        for (int i = 0; i < data16.Length; i++)
        {
            data16[i] = reader.ReadUInt16();
        }

        byte[] data8 = reader.ReadBytes(data8Bytes);
        if (data8.Length != data8Bytes)
        {
            throw new EndOfStreamException("Unexpected end of DTS file while reading data8 buffer.");
        }

        var buffer = new DtsDataBuffer(data32, data16, data8);

        int numSequences = reader.ReadInt32();
        for (int i = 0; i < numSequences; i++)
        {
            SkipSequence(reader);
        }

        _ = reader.ReadSByte(); // matStreamType
        int numMaterials = reader.ReadInt32();
        var materialNames = new List<string>(Math.Max(numMaterials, 0));
        for (int i = 0; i < numMaterials; i++)
        {
            int length = reader.ReadInt32();
            if (length < 0)
            {
                throw new InvalidDataException("Invalid DTS material name length.");
            }

            byte[] nameBytes = reader.ReadBytes(length);
            if (nameBytes.Length != length)
            {
                throw new EndOfStreamException("Unexpected end of DTS file while reading material name.");
            }

            materialNames.Add(System.Text.Encoding.UTF8.GetString(nameBytes));
        }

        SkipMaterialArrays(reader, version, numMaterials);

        return ParseMeshes(buffer, version, materialNames);
    }

    private static List<ColladaMesh> ParseMeshes(DtsDataBuffer buffer, short version, List<string> materialNames)
    {
        int numNodes = buffer.ReadS32();
        int numObjects = buffer.ReadS32();
        int numDecals = buffer.ReadS32();
        int numSubShapes = buffer.ReadS32();
        int numIfls = buffer.ReadS32();
        int numNodeRotations = buffer.ReadS32();
        int numNodeTranslations = buffer.ReadS32();
        int numNodeUniformScales = buffer.ReadS32();
        int numNodeAlignedScales = buffer.ReadS32();
        int numNodeArbScales = buffer.ReadS32();
        int numGroundFrames = buffer.ReadS32();
        int numObjectStates = buffer.ReadS32();
        int numDecalStates = buffer.ReadS32();
        int numTriggers = buffer.ReadS32();
        int numDetails = buffer.ReadS32();
        int numMeshes = buffer.ReadS32();
        int numNames = buffer.ReadS32();
        _ = buffer.ReadF32();
        _ = buffer.ReadS32();
        buffer.CheckGuard();

        _ = buffer.ReadF32();
        _ = buffer.ReadF32();
        _ = buffer.ReadPoint3();
        _ = buffer.ReadPoint3();
        _ = buffer.ReadPoint3();
        buffer.CheckGuard();

        for (int i = 0; i < numNodes; i++)
        {
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
        }
        buffer.CheckGuard();

        var objects = new List<DtsObject>(Math.Max(numObjects, 0));
        for (int i = 0; i < numObjects; i++)
        {
            objects.Add(new DtsObject(
                NameIndex: buffer.ReadS32(),
                NumMeshes: buffer.ReadS32(),
                StartMeshIndex: buffer.ReadS32()));
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
        }
        buffer.CheckGuard();

        for (int i = 0; i < numDecals; i++)
        {
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
        }
        buffer.CheckGuard();

        for (int i = 0; i < numIfls; i++)
        {
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
        }
        buffer.CheckGuard();

        for (int i = 0; i < numSubShapes; i++) _ = buffer.ReadS32();
        for (int i = 0; i < numSubShapes; i++) _ = buffer.ReadS32();
        for (int i = 0; i < numSubShapes; i++) _ = buffer.ReadS32();
        for (int i = 0; i < numSubShapes; i++) _ = buffer.ReadS32();
        buffer.CheckGuard();

        for (int i = 0; i < numNodes * 4; i++) _ = buffer.ReadS16();
        for (int i = 0; i < numNodes; i++) _ = buffer.ReadPoint3();
        for (int i = 0; i < numNodeRotations * 4; i++) _ = buffer.ReadS16();
        for (int i = 0; i < numNodeTranslations; i++) _ = buffer.ReadPoint3();
        buffer.CheckGuard();

        for (int i = 0; i < numNodeUniformScales; i++) _ = buffer.ReadF32();
        for (int i = 0; i < numNodeAlignedScales; i++) _ = buffer.ReadPoint3();
        for (int i = 0; i < numNodeArbScales; i++) _ = buffer.ReadPoint3();
        for (int i = 0; i < numNodeArbScales * 4; i++) _ = buffer.ReadS16();
        buffer.CheckGuard();

        for (int i = 0; i < numGroundFrames; i++) _ = buffer.ReadPoint3();
        for (int i = 0; i < numGroundFrames * 4; i++) _ = buffer.ReadS16();
        buffer.CheckGuard();

        for (int i = 0; i < numObjectStates; i++)
        {
            _ = buffer.ReadF32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
        }
        buffer.CheckGuard();

        for (int i = 0; i < numDecalStates; i++) _ = buffer.ReadS32();
        buffer.CheckGuard();

        for (int i = 0; i < numTriggers; i++)
        {
            _ = buffer.ReadU32();
            _ = buffer.ReadF32();
        }
        buffer.CheckGuard();

        for (int i = 0; i < numDetails; i++)
        {
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadF32();
            _ = buffer.ReadF32();
            _ = buffer.ReadF32();
            _ = buffer.ReadS32();
            if (version >= 26)
            {
                _ = buffer.ReadS32();
                _ = buffer.ReadS32();
                _ = buffer.ReadS32();
                _ = buffer.ReadS32();
                _ = buffer.ReadF32();
                _ = buffer.ReadU8();
            }
        }
        buffer.CheckGuard();

        var parsedMeshes = new List<ParsedMesh>(Math.Max(numMeshes, 0));
        for (int i = 0; i < numMeshes; i++)
        {
            parsedMeshes.Add(ParseSingleMesh(buffer, version));
        }
        buffer.CheckGuard();

        var names = new List<string>(Math.Max(numNames, 0));
        for (int i = 0; i < numNames; i++)
        {
            names.Add(buffer.ReadNullTerminatedString8());
        }
        buffer.CheckGuard();

        for (int i = 0; i < numDetails; i++) _ = buffer.ReadF32();
        for (int i = 0; i < numDetails; i++) _ = buffer.ReadF32();

        return BuildOutputMeshes(objects, parsedMeshes, names, materialNames);
    }

    private static ParsedMesh ParseSingleMesh(DtsDataBuffer buffer, short version)
    {
        uint meshType = buffer.ReadU32();
        buffer.CheckGuard();

        _ = buffer.ReadS32();
        _ = buffer.ReadS32();
        _ = buffer.ReadS32();

        _ = buffer.ReadPoint3();
        _ = buffer.ReadPoint3();
        _ = buffer.ReadPoint3();
        _ = buffer.ReadF32();

        int numVerts = buffer.ReadS32();
        Vector3[] verts = ReadPoint3Array(buffer, numVerts);

        int numTVerts = buffer.ReadS32();
        Vector2[] tverts = ReadPoint2Array(buffer, numTVerts);

        if (version >= 26)
        {
            int numTVerts2 = buffer.ReadS32();
            for (int i = 0; i < numTVerts2; i++)
            {
                _ = buffer.ReadPoint2();
            }

            int numVColors = buffer.ReadS32();
            for (int i = 0; i < numVColors; i++)
            {
                _ = buffer.ReadU8();
                _ = buffer.ReadU8();
                _ = buffer.ReadU8();
                _ = buffer.ReadU8();
            }
        }

        Vector3[] norms = ReadPoint3Array(buffer, numVerts);
        for (int i = 0; i < numVerts; i++) _ = buffer.ReadU8();

        int numPrimitives = buffer.ReadS32();
        var primitiveStarts = new int[Math.Max(numPrimitives, 0)];
        var primitiveCounts = new int[Math.Max(numPrimitives, 0)];
        var primitiveMaterials = new uint[Math.Max(numPrimitives, 0)];

        if (version <= 24)
        {
            for (int i = 0; i < numPrimitives; i++)
            {
                primitiveStarts[i] = buffer.ReadS16();
                primitiveCounts[i] = buffer.ReadS16();
            }
            for (int i = 0; i < numPrimitives; i++) _ = buffer.ReadU32();
        }
        else
        {
            for (int i = 0; i < numPrimitives; i++)
            {
                primitiveStarts[i] = buffer.ReadS32();
                primitiveCounts[i] = buffer.ReadS32();
                primitiveMaterials[i] = buffer.ReadU32();
            }
        }

        int numIndices = buffer.ReadS32();
        int[] indices = new int[Math.Max(numIndices, 0)];
        if (version <= 24)
        {
            for (int i = 0; i < numIndices; i++) indices[i] = buffer.ReadS16();
        }
        else
        {
            for (int i = 0; i < numIndices; i++) indices[i] = buffer.ReadS32();
        }

        int numMergeIndices = buffer.ReadS32();
        for (int i = 0; i < numMergeIndices; i++) _ = buffer.ReadS16();
        _ = buffer.ReadS32();
        _ = buffer.ReadU32();
        buffer.CheckGuard();

        int meshKind = (int)(meshType & 0xF);
        if (meshKind == 1)
        {
            SkipSkinMeshPayload(buffer);
        }
        else if (meshKind == 3)
        {
            SkipSortedMeshPayload(buffer);
        }

        return new ParsedMesh(verts, norms, tverts, primitiveStarts, primitiveCounts, primitiveMaterials, indices);
    }

    private static void SkipSkinMeshPayload(DtsDataBuffer buffer)
    {
        int numInitialVerts = buffer.ReadS32();
        for (int i = 0; i < numInitialVerts; i++) _ = buffer.ReadPoint3();
        for (int i = 0; i < numInitialVerts; i++) _ = buffer.ReadPoint3();
        for (int i = 0; i < numInitialVerts; i++) _ = buffer.ReadU8();

        int numInitialTransforms = buffer.ReadS32();
        for (int i = 0; i < numInitialTransforms * 16; i++) _ = buffer.ReadF32();

        int numVertIndices = buffer.ReadS32();
        for (int i = 0; i < numVertIndices; i++) _ = buffer.ReadS32();
        int numBoneIndices = buffer.ReadS32();
        for (int i = 0; i < numBoneIndices; i++) _ = buffer.ReadS32();
        int numWeights = buffer.ReadS32();
        for (int i = 0; i < numWeights; i++) _ = buffer.ReadF32();
        int numNodeIndices = buffer.ReadS32();
        for (int i = 0; i < numNodeIndices; i++) _ = buffer.ReadS32();
        buffer.CheckGuard();
    }

    private static void SkipSortedMeshPayload(DtsDataBuffer buffer)
    {
        int numClusters = buffer.ReadS32();
        for (int i = 0; i < numClusters; i++)
        {
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
            _ = buffer.ReadPoint3();
            _ = buffer.ReadF32();
            _ = buffer.ReadS32();
            _ = buffer.ReadS32();
        }

        int numStartClusters = buffer.ReadS32();
        for (int i = 0; i < numStartClusters; i++) _ = buffer.ReadS32();
        int numFirstVerts = buffer.ReadS32();
        for (int i = 0; i < numFirstVerts; i++) _ = buffer.ReadS32();
        int numNumVerts = buffer.ReadS32();
        for (int i = 0; i < numNumVerts; i++) _ = buffer.ReadS32();
        int numFirstTVerts = buffer.ReadS32();
        for (int i = 0; i < numFirstTVerts; i++) _ = buffer.ReadS32();
        _ = buffer.ReadS32();
        buffer.CheckGuard();
    }

    private static List<ColladaMesh> BuildOutputMeshes(
        List<DtsObject> objects,
        List<ParsedMesh> parsedMeshes,
        List<string> names,
        List<string> materialNames)
    {
        var result = new List<ColladaMesh>();

        foreach (DtsObject obj in objects)
        {
            string objectName = GetSafeName(names, obj.NameIndex, "dts_object");
            int start = Math.Max(obj.StartMeshIndex, 0);
            int endExclusive = Math.Min(start + Math.Max(obj.NumMeshes, 0), parsedMeshes.Count);
            for (int meshIndex = start; meshIndex < endExclusive; meshIndex++)
            {
                ParsedMesh mesh = parsedMeshes[meshIndex];
                for (int primIndex = 0; primIndex < mesh.PrimitiveStarts.Length; primIndex++)
                {
                    int primitiveStart = mesh.PrimitiveStarts[primIndex];
                    int primitiveCount = mesh.PrimitiveCounts[primIndex];
                    if (primitiveStart < 0 || primitiveCount < 3 || primitiveStart + primitiveCount > mesh.Indices.Length)
                    {
                        continue;
                    }

                    bool strip = (mesh.PrimitiveMaterials[primIndex] & PrimitiveStripFlag) != 0;
                    int matIndex = (int)(mesh.PrimitiveMaterials[primIndex] & PrimitiveMaterialMask);
                    string materialName = GetSafeName(materialNames, matIndex, "");

                    var outVerts = new List<ColladaVertex>();
                    var outIndices = new List<int>();
                    EmitPrimitiveTriangles(mesh, primitiveStart, primitiveCount, strip, outVerts, outIndices);
                    if (outVerts.Count == 0 || outIndices.Count == 0)
                    {
                        continue;
                    }

                    result.Add(new ColladaMesh
                    {
                        Name = $"{objectName}_{meshIndex}_{primIndex}",
                        GeometryName = objectName,
                        MaterialName = materialName,
                        SceneNodeName = objectName,
                        HasBakedTransform = false,
                        Vertices = outVerts,
                        Indices = outIndices,
                    });
                }
            }
        }

        if (result.Count == 0 && parsedMeshes.Count > 0)
        {
            var fallbackObject = new DtsObject(0, parsedMeshes.Count, 0);
            return BuildOutputMeshes(new List<DtsObject> { fallbackObject }, parsedMeshes, new List<string> { "dts_mesh" }, materialNames);
        }

        return result;
    }

    private static void EmitPrimitiveTriangles(
        ParsedMesh mesh,
        int start,
        int count,
        bool strip,
        List<ColladaVertex> outVerts,
        List<int> outIndices)
    {
        if (strip)
        {
            for (int i = 0; i + 2 < count; i++)
            {
                int a = mesh.Indices[start + i];
                int b = mesh.Indices[start + i + 1];
                int c = mesh.Indices[start + i + 2];
                if ((i & 1) == 1)
                {
                    (b, c) = (c, b);
                }

                EmitTriangle(mesh, a, b, c, outVerts, outIndices);
            }

            return;
        }

        for (int i = 0; i + 2 < count; i += 3)
        {
            int a = mesh.Indices[start + i];
            int b = mesh.Indices[start + i + 1];
            int c = mesh.Indices[start + i + 2];
            EmitTriangle(mesh, a, b, c, outVerts, outIndices);
        }
    }

    private static void EmitTriangle(
        ParsedMesh mesh,
        int a,
        int b,
        int c,
        List<ColladaVertex> outVerts,
        List<int> outIndices)
    {
        int baseIndex = outVerts.Count;
        outVerts.Add(ReadVertex(mesh, a));
        outVerts.Add(ReadVertex(mesh, b));
        outVerts.Add(ReadVertex(mesh, c));
        outIndices.Add(baseIndex);
        outIndices.Add(baseIndex + 1);
        outIndices.Add(baseIndex + 2);
    }

    private static ColladaVertex ReadVertex(ParsedMesh mesh, int index)
    {
        if (index < 0 || index >= mesh.Vertices.Length)
        {
            return new ColladaVertex(Vector3.Zero, Vector3.UnitY, Vector2.Zero);
        }

        Vector3 position = mesh.Vertices[index];
        Vector3 normal = index < mesh.Normals.Length ? mesh.Normals[index] : Vector3.UnitY;
        Vector2 uv = index < mesh.TVerts.Length ? mesh.TVerts[index] : Vector2.Zero;
        return new ColladaVertex(position, normal, uv);
    }

    private static string GetSafeName(IReadOnlyList<string> names, int index, string fallback)
    {
        if (index >= 0 && index < names.Count && !string.IsNullOrWhiteSpace(names[index]))
        {
            return names[index];
        }

        return fallback;
    }

    private static Vector3[] ReadPoint3Array(DtsDataBuffer buffer, int count)
    {
        if (count <= 0)
        {
            return Array.Empty<Vector3>();
        }

        var result = new Vector3[count];
        for (int i = 0; i < count; i++)
        {
            result[i] = buffer.ReadPoint3();
        }

        return result;
    }

    private static Vector2[] ReadPoint2Array(DtsDataBuffer buffer, int count)
    {
        if (count <= 0)
        {
            return Array.Empty<Vector2>();
        }

        var result = new Vector2[count];
        for (int i = 0; i < count; i++)
        {
            result[i] = buffer.ReadPoint2();
        }

        return result;
    }

    private static void SkipSequence(BinaryReader reader)
    {
        for (int i = 0; i < 15; i++) _ = reader.ReadInt32();
        for (int i = 0; i < 8; i++) SkipBitSet(reader);
    }

    private static void SkipBitSet(BinaryReader reader)
    {
        _ = reader.ReadInt32();
        int numWords = reader.ReadInt32();
        if (numWords < 0)
        {
            throw new InvalidDataException("Invalid DTS bitset length.");
        }

        reader.BaseStream.Seek(numWords * 4L, SeekOrigin.Current);
    }

    private static void SkipMaterialArrays(BinaryReader reader, short version, int numMaterials)
    {
        if (numMaterials <= 0)
        {
            return;
        }

        reader.BaseStream.Seek(numMaterials * 4L, SeekOrigin.Current); // matFlags
        reader.BaseStream.Seek(numMaterials * 4L, SeekOrigin.Current); // matReflectanceMaps
        reader.BaseStream.Seek(numMaterials * 4L, SeekOrigin.Current); // matBumpMaps
        reader.BaseStream.Seek(numMaterials * 4L, SeekOrigin.Current); // matDetailMaps
        if (version == 25)
        {
            reader.BaseStream.Seek(numMaterials * 4L, SeekOrigin.Current); // dummy
        }

        reader.BaseStream.Seek(numMaterials * 4L, SeekOrigin.Current); // matDetailScales
        reader.BaseStream.Seek(numMaterials * 4L, SeekOrigin.Current); // matReflectance
    }

    private readonly record struct DtsObject(int NameIndex, int NumMeshes, int StartMeshIndex);

    private sealed record ParsedMesh(
        Vector3[] Vertices,
        Vector3[] Normals,
        Vector2[] TVerts,
        int[] PrimitiveStarts,
        int[] PrimitiveCounts,
        uint[] PrimitiveMaterials,
        int[] Indices);

    private sealed class DtsDataBuffer
    {
        private readonly uint[] _data32;
        private readonly ushort[] _data16;
        private readonly byte[] _data8;
        private int _i32;
        private int _i16;
        private int _i8;
        private int _guard32;
        private short _guard16;
        private byte _guard8;

        public DtsDataBuffer(uint[] data32, ushort[] data16, byte[] data8)
        {
            _data32 = data32;
            _data16 = data16;
            _data8 = data8;
        }

        public int ReadS32() => unchecked((int)ReadU32());

        public uint ReadU32()
        {
            if (_i32 >= _data32.Length)
            {
                throw new EndOfStreamException("Unexpected end of DTS 32-bit buffer.");
            }

            return _data32[_i32++];
        }

        public short ReadS16()
        {
            if (_i16 >= _data16.Length)
            {
                throw new EndOfStreamException("Unexpected end of DTS 16-bit buffer.");
            }

            return unchecked((short)_data16[_i16++]);
        }

        public byte ReadU8()
        {
            if (_i8 >= _data8.Length)
            {
                throw new EndOfStreamException("Unexpected end of DTS 8-bit buffer.");
            }

            return _data8[_i8++];
        }

        public float ReadF32() => BitConverter.Int32BitsToSingle(ReadS32());

        public Vector3 ReadPoint3() => new(ReadF32(), ReadF32(), ReadF32());

        public Vector2 ReadPoint2() => new(ReadF32(), ReadF32());

        public string ReadNullTerminatedString8()
        {
            var chars = new List<byte>();
            while (true)
            {
                byte value = ReadU8();
                if (value == 0)
                {
                    break;
                }

                chars.Add(value);
            }

            return chars.Count == 0 ? string.Empty : System.Text.Encoding.UTF8.GetString(chars.ToArray());
        }

        public void CheckGuard()
        {
            int g32 = ReadS32();
            short g16 = ReadS16();
            byte g8 = ReadU8();
            if (g32 != _guard32 || g16 != _guard16 || g8 != _guard8)
            {
                throw new InvalidDataException("DTS interleaved buffer guard mismatch.");
            }

            _guard32++;
            _guard16++;
            _guard8++;
        }
    }
}
