using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;

namespace LibreRally.Vehicle.Rendering
{
	/// <summary>
	/// Best-effort Torque DTS mesh loader.
	/// Supports static geometry extraction for common DTS v24-v26 files.
	/// </summary>
	public static class DtsLoader
	{
		private const uint PrimitiveMaterialMask = 0x0FFFFFFF;
		private const uint PrimitiveStripFlag = 0x40000000;
		private const long MaxInterleavedBufferBytes = 128L * 1024L * 1024L;
		private const float Quat16MaxValue = 32767f;

		/// <summary>Loads static mesh geometry from a Torque DTS file.</summary>
		/// <param name="dtsFilePath">Path to the DTS file.</param>
		/// <returns>The extracted meshes converted into <see cref="ColladaMesh"/> objects.</returns>
		/// <exception cref="EndOfStreamException">Thrown when the DTS file ends before the required buffers are fully read.</exception>
		/// <exception cref="InvalidDataException">Thrown when the DTS file header or buffer layout is invalid.</exception>
		public static List<ColladaMesh> Load(string dtsFilePath)
		{
			using var stream = File.OpenRead(dtsFilePath);
			using var reader = new BinaryReader(stream);

			short version = reader.ReadInt16();
			_ = reader.ReadInt16(); // exporter version
			if (version is < 24 or > 26)
			{
				throw new InvalidDataException($"Unsupported DTS version {version}. Supported versions are 24 through 26.");
			}

			int sizeAll = reader.ReadInt32();
			int start16 = reader.ReadInt32();
			int start8 = reader.ReadInt32();
			if (sizeAll <= 0 || start16 < 0 || start8 < start16 || start16 > sizeAll || start8 > sizeAll)
			{
				throw new InvalidDataException("Invalid DTS buffer header.");
			}

			long interleavedBufferBytes = (long)sizeAll * sizeof(uint);
			long remainingBytes = stream.Length - stream.Position;
			if (interleavedBufferBytes > remainingBytes)
			{
				throw new InvalidDataException("Invalid DTS buffer header: interleaved buffer exceeds file length.");
			}

			if (interleavedBufferBytes > MaxInterleavedBufferBytes)
			{
				throw new InvalidDataException($"DTS interleaved buffer exceeds safety limit ({MaxInterleavedBufferBytes} bytes).");
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

			List<string> materialNames = ReadMaterialList(reader, version);

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

			var nodes = new List<DtsNode>(Math.Max(numNodes, 0));
			for (int i = 0; i < numNodes; i++)
			{
				nodes.Add(new DtsNode(
					NameIndex: buffer.ReadS32(),
					ParentIndex: buffer.ReadS32()));
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
					StartMeshIndex: buffer.ReadS32(),
					NodeIndex: buffer.ReadS32()));
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

			_ = ReadS32Array(buffer, numSubShapes);
			int[] subShapeFirstObjects = ReadS32Array(buffer, numSubShapes);
			_ = ReadS32Array(buffer, numSubShapes);
			buffer.CheckGuard();

			_ = ReadS32Array(buffer, numSubShapes);
			int[] subShapeNumObjects = ReadS32Array(buffer, numSubShapes);
			_ = ReadS32Array(buffer, numSubShapes);
			buffer.CheckGuard();

			Quaternion[] defaultRotations = ReadQuat16Array(buffer, numNodes);
			Vector3[] defaultTranslations = ReadPoint3Array(buffer, numNodes);
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

			var details = new List<DtsDetail>(Math.Max(numDetails, 0));
			for (int i = 0; i < numDetails; i++)
			{
				int nameIndex = buffer.ReadS32();
				int subShapeNum = buffer.ReadS32();
				int objectDetailNum = buffer.ReadS32();
				float size = buffer.ReadF32();
				float averageError = buffer.ReadF32();
				float maxError = buffer.ReadF32();
				int polyCount = buffer.ReadS32();
				details.Add(new DtsDetail(nameIndex, subShapeNum, objectDetailNum, size, averageError, maxError, polyCount));
				if (version >= 26)
				{
					_ = buffer.ReadS32();
					_ = buffer.ReadS32();
					_ = buffer.ReadU32();
					_ = buffer.ReadU32();
					_ = buffer.ReadF32();
					_ = buffer.ReadU32();
				}
			}
			buffer.CheckGuard();

			var parsedMeshes = new List<ParsedMesh>(Math.Max(numMeshes, 0));
			for (int i = 0; i < numMeshes; i++)
			{
				parsedMeshes.Add(ParseSingleMesh(buffer, version, parsedMeshes));
			}
			buffer.CheckGuard();

			var names = new List<string>(Math.Max(numNames, 0));
			for (int i = 0; i < numNames; i++)
			{
				names.Add(buffer.ReadNullTerminatedString8());
			}
			buffer.CheckGuard();

			return BuildOutputMeshes(
				objects,
				parsedMeshes,
				names,
				materialNames,
				nodes,
				defaultRotations,
				defaultTranslations,
				details,
				subShapeFirstObjects,
				subShapeNumObjects);
		}

		private static ParsedMesh ParseSingleMesh(DtsDataBuffer buffer, short version, IReadOnlyList<ParsedMesh> parsedMeshes)
		{
			uint meshType = buffer.ReadU32();
			buffer.CheckGuard();

			_ = buffer.ReadS32();
			_ = buffer.ReadS32();
			int parentMesh = buffer.ReadS32();

			_ = buffer.ReadPoint3();
			_ = buffer.ReadPoint3();
			_ = buffer.ReadPoint3();
			_ = buffer.ReadF32();

			int numVerts = buffer.ReadS32();
			Vector3[] verts = parentMesh >= 0
				? GetSharedArray(parsedMeshes, parentMesh, numVerts, static mesh => mesh.Vertices, "vertex")
				: ReadPoint3Array(buffer, numVerts);

			int numTVerts = buffer.ReadS32();
			Vector2[] tverts = parentMesh >= 0
				? GetSharedArray(parsedMeshes, parentMesh, numTVerts, static mesh => mesh.TVerts, "texture coordinate")
				: ReadPoint2Array(buffer, numTVerts);

			if (version >= 26)
			{
				int numTVerts2 = buffer.ReadS32();
				if (parentMesh < 0)
				{
					for (int i = 0; i < numTVerts2; i++)
					{
						_ = buffer.ReadPoint2();
					}
				}

				int numVColors = buffer.ReadS32();
				if (parentMesh < 0)
				{
					for (int i = 0; i < numVColors; i++)
					{
						_ = buffer.ReadU32();
					}
				}
			}

			Vector3[] norms = parentMesh >= 0
				? GetSharedArray(parsedMeshes, parentMesh, numVerts, static mesh => mesh.Normals, "normal")
				: ReadPoint3Array(buffer, numVerts);
			if (parentMesh < 0)
			{
				for (int i = 0; i < numVerts; i++) _ = buffer.ReadU8();
			}

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
			buffer.Align16To32();
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

		private static T[] GetSharedArray<T>(
			IReadOnlyList<ParsedMesh> parsedMeshes,
			int parentMeshIndex,
			int count,
			Func<ParsedMesh, T[]> selector,
			string arrayName)
		{
			if (count <= 0)
			{
				return [];
			}

			if ((uint)parentMeshIndex >= (uint)parsedMeshes.Count)
			{
				throw new InvalidDataException($"DTS mesh references invalid parent mesh {parentMeshIndex} for shared {arrayName} data.");
			}

			T[] parentData = selector(parsedMeshes[parentMeshIndex]);
			if (parentData.Length < count)
			{
				throw new InvalidDataException(
					$"DTS mesh references parent mesh {parentMeshIndex} with insufficient shared {arrayName} data ({parentData.Length} < {count}).");
			}

			if (parentData.Length == count)
			{
				return parentData;
			}

			var result = new T[count];
			Array.Copy(parentData, result, count);
			return result;
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
			List<string> materialNames,
			IReadOnlyList<DtsNode> nodes,
			IReadOnlyList<Quaternion> defaultRotations,
			IReadOnlyList<Vector3> defaultTranslations,
			IReadOnlyList<DtsDetail> details,
			IReadOnlyList<int> subShapeFirstObjects,
			IReadOnlyList<int> subShapeNumObjects)
		{
			var result = new List<ColladaMesh>();
			Matrix4x4[] nodeWorldTransforms = BuildNodeWorldTransforms(nodes, defaultRotations, defaultTranslations);
			int[] objectSubShapes = BuildObjectSubShapeIndices(objects.Count, subShapeFirstObjects, subShapeNumObjects);
			Dictionary<int, int> preferredDetails = SelectPreferredDetails(details);

			for (int objectIndex = 0; objectIndex < objects.Count; objectIndex++)
			{
				DtsObject obj = objects[objectIndex];
				string objectName = GetSafeName(names, obj.NameIndex, "dts_object");
				Matrix4x4 objectTransform = GetObjectTransform(nodeWorldTransforms, obj.NodeIndex);
				bool hasObjectTransform = !IsApproximatelyIdentity(objectTransform);
				int? meshIndex = SelectPreferredMeshIndex(objectIndex, obj, objectSubShapes, preferredDetails, parsedMeshes);
				if (!meshIndex.HasValue)
				{
					continue;
				}

				ParsedMesh mesh = parsedMeshes[meshIndex.Value];
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

					List<ColladaVertex> bakedVertices = hasObjectTransform
						? BakeVertices(outVerts, objectTransform)
						: outVerts;

					result.Add(new ColladaMesh
					{
						Name = $"{objectName}_{meshIndex.Value}_{primIndex}",
						GeometryName = objectName,
						MaterialName = materialName,
						SceneNodeName = objectName,
						HasBakedTransform = hasObjectTransform,
						Vertices = bakedVertices,
						Indices = outIndices,
					});
				}
			}

			if (result.Count == 0 && parsedMeshes.Count > 0)
			{
				var fallbackObject = new DtsObject(0, parsedMeshes.Count, 0);
				return BuildOutputMeshes(
					[fallbackObject],
					parsedMeshes,
					["dts_mesh"],
					materialNames,
					[],
					[],
					[],
					[],
					[],
					[]);
			}

			return result;
		}

		private static int[] BuildObjectSubShapeIndices(
			int objectCount,
			IReadOnlyList<int> subShapeFirstObjects,
			IReadOnlyList<int> subShapeNumObjects)
		{
			if (objectCount <= 0)
			{
				return [];
			}

			var result = new int[objectCount];
			Array.Fill(result, -1);

			int subShapeCount = Math.Min(subShapeFirstObjects.Count, subShapeNumObjects.Count);
			for (int subShapeIndex = 0; subShapeIndex < subShapeCount; subShapeIndex++)
			{
				int start = Math.Max(subShapeFirstObjects[subShapeIndex], 0);
				int endExclusive = Math.Min(start + Math.Max(subShapeNumObjects[subShapeIndex], 0), objectCount);
				for (int objectIndex = start; objectIndex < endExclusive; objectIndex++)
				{
					result[objectIndex] = subShapeIndex;
				}
			}

			return result;
		}

		private static Dictionary<int, int> SelectPreferredDetails(IReadOnlyList<DtsDetail> details)
		{
			var result = new Dictionary<int, int>();
			foreach (DtsDetail detail in details)
			{
				if (detail.SubShapeNum < 0 || detail.ObjectDetailNum < 0)
				{
					continue;
				}

				// Torque detail lists are ordered from highest to lowest visual detail,
				// so the first detail we see for a subshape is the best static mesh choice.
				result.TryAdd(detail.SubShapeNum, detail.ObjectDetailNum);
			}

			return result;
		}

		private static int? SelectPreferredMeshIndex(
			int objectIndex,
			DtsObject obj,
			IReadOnlyList<int> objectSubShapes,
			IReadOnlyDictionary<int, int> preferredDetails,
			IReadOnlyList<ParsedMesh> parsedMeshes)
		{
			if (obj.NumMeshes <= 0)
			{
				return null;
			}

			if ((uint)objectIndex < (uint)objectSubShapes.Count)
			{
				int subShapeIndex = objectSubShapes[objectIndex];
				if (subShapeIndex >= 0 &&
				    preferredDetails.TryGetValue(subShapeIndex, out int preferredDetail) &&
				    TryGetObjectMeshIndex(obj, preferredDetail, parsedMeshes, out int preferredMeshIndex))
				{
					return preferredMeshIndex;
				}
			}

			for (int detailIndex = 0; detailIndex < obj.NumMeshes; detailIndex++)
			{
				if (TryGetObjectMeshIndex(obj, detailIndex, parsedMeshes, out int fallbackMeshIndex))
				{
					return fallbackMeshIndex;
				}
			}

			return null;
		}

		private static bool TryGetObjectMeshIndex(
			DtsObject obj,
			int detailIndex,
			IReadOnlyList<ParsedMesh> parsedMeshes,
			out int meshIndex)
		{
			meshIndex = -1;
			if (detailIndex < 0 || detailIndex >= obj.NumMeshes)
			{
				return false;
			}

			int candidate = obj.StartMeshIndex + detailIndex;
			if ((uint)candidate >= (uint)parsedMeshes.Count)
			{
				return false;
			}

			ParsedMesh mesh = parsedMeshes[candidate];
			if (mesh.PrimitiveStarts.Length == 0 || mesh.Indices.Length == 0)
			{
				return false;
			}

			meshIndex = candidate;
			return true;
		}

		private static void EmitPrimitiveTriangles(
			ParsedMesh mesh,
			int start,
			int count,
			bool strip,
			List<ColladaVertex> outVerts,
			List<int> outIndices)
		{
			var vertexIndexMap = new Dictionary<int, int>();
			if (strip)
			{
				for (int i = 0; i + 2 < count; i++)
				{
					int a = mesh.Indices[start + i];
					int b = mesh.Indices[start + i + 1];
					int c = mesh.Indices[start + i + 2];
					if ((i & 1) == 1)
					{
						(a, b) = (b, a);
					}

					EmitTriangle(mesh, a, b, c, outVerts, outIndices, vertexIndexMap);
				}

				return;
			}

			for (int i = 0; i + 2 < count; i += 3)
			{
				int a = mesh.Indices[start + i];
				int b = mesh.Indices[start + i + 1];
				int c = mesh.Indices[start + i + 2];
				EmitTriangle(mesh, a, b, c, outVerts, outIndices, vertexIndexMap);
			}
		}

		private static void EmitTriangle(
			ParsedMesh mesh,
			int a,
			int b,
			int c,
			List<ColladaVertex> outVerts,
			List<int> outIndices,
			Dictionary<int, int> vertexIndexMap)
		{
			outIndices.Add(GetOrAddVertexIndex(mesh, a, outVerts, vertexIndexMap));
			outIndices.Add(GetOrAddVertexIndex(mesh, b, outVerts, vertexIndexMap));
			outIndices.Add(GetOrAddVertexIndex(mesh, c, outVerts, vertexIndexMap));
		}

		private static int GetOrAddVertexIndex(
			ParsedMesh mesh,
			int sourceIndex,
			List<ColladaVertex> outVerts,
			Dictionary<int, int> vertexIndexMap)
		{
			if (vertexIndexMap.TryGetValue(sourceIndex, out int existingIndex))
			{
				return existingIndex;
			}

			int nextIndex = outVerts.Count;
			outVerts.Add(ReadVertex(mesh, sourceIndex));
			vertexIndexMap[sourceIndex] = nextIndex;
			return nextIndex;
		}

		private static Matrix4x4[] BuildNodeWorldTransforms(
			IReadOnlyList<DtsNode> nodes,
			IReadOnlyList<Quaternion> defaultRotations,
			IReadOnlyList<Vector3> defaultTranslations)
		{
			var result = new Matrix4x4[nodes.Count];
			var resolved = new bool[nodes.Count];
			var resolving = new bool[nodes.Count];

			for (int i = 0; i < nodes.Count; i++)
			{
				ResolveNodeWorldTransform(i, nodes, defaultRotations, defaultTranslations, result, resolved, resolving);
			}

			return result;
		}

		private static Matrix4x4 ResolveNodeWorldTransform(
			int nodeIndex,
			IReadOnlyList<DtsNode> nodes,
			IReadOnlyList<Quaternion> defaultRotations,
			IReadOnlyList<Vector3> defaultTranslations,
			Matrix4x4[] result,
			bool[] resolved,
			bool[] resolving)
		{
			if (resolved[nodeIndex])
			{
				return result[nodeIndex];
			}

			if (resolving[nodeIndex])
			{
				throw new InvalidDataException("DTS node hierarchy contains a cycle.");
			}

			resolving[nodeIndex] = true;

			Quaternion rotation = nodeIndex < defaultRotations.Count
				? defaultRotations[nodeIndex]
				: Quaternion.Identity;
			if (rotation.LengthSquared() <= 1e-8f)
			{
				rotation = Quaternion.Identity;
			}
			else
			{
				rotation = Quaternion.Normalize(rotation);
			}

			Vector3 translation = nodeIndex < defaultTranslations.Count
				? defaultTranslations[nodeIndex]
				: Vector3.Zero;

			Matrix4x4 localTransform = Matrix4x4.CreateFromQuaternion(rotation);
			localTransform.M41 = translation.X;
			localTransform.M42 = translation.Y;
			localTransform.M43 = translation.Z;

			int parentIndex = nodes[nodeIndex].ParentIndex;
			result[nodeIndex] = parentIndex >= 0 && parentIndex < nodes.Count
				? Matrix4x4.Multiply(
					localTransform,
					ResolveNodeWorldTransform(
						parentIndex,
						nodes,
						defaultRotations,
						defaultTranslations,
						result,
						resolved,
						resolving))
				: localTransform;

			resolving[nodeIndex] = false;
			resolved[nodeIndex] = true;
			return result[nodeIndex];
		}

		private static Matrix4x4 GetObjectTransform(IReadOnlyList<Matrix4x4> nodeWorldTransforms, int nodeIndex)
		{
			if (nodeIndex < 0 || nodeIndex >= nodeWorldTransforms.Count)
			{
				return Matrix4x4.Identity;
			}

			return nodeWorldTransforms[nodeIndex];
		}

		private static List<ColladaVertex> BakeVertices(List<ColladaVertex> vertices, Matrix4x4 transform)
		{
			var normalTransform = transform;
			if (Matrix4x4.Invert(transform, out Matrix4x4 inverseTransform))
			{
				normalTransform = Matrix4x4.Transpose(inverseTransform);
			}

			var result = new List<ColladaVertex>(vertices.Count);
			foreach (ColladaVertex vertex in vertices)
			{
				Vector3 position = Vector3.Transform(vertex.Position, transform);
				Vector3 normal = Vector3.TransformNormal(vertex.Normal, normalTransform);
				if (normal.LengthSquared() > 1e-8f)
				{
					normal = Vector3.Normalize(normal);
				}
				else
				{
					normal = Vector3.UnitY;
				}

				result.Add(new ColladaVertex(position, normal, vertex.TexCoord));
			}

			return result;
		}

		private static bool IsApproximatelyIdentity(Matrix4x4 matrix)
		{
			const float Epsilon = 1e-4f;
			return MathF.Abs(matrix.M11 - 1f) < Epsilon &&
			       MathF.Abs(matrix.M12) < Epsilon &&
			       MathF.Abs(matrix.M13) < Epsilon &&
			       MathF.Abs(matrix.M14) < Epsilon &&
			       MathF.Abs(matrix.M21) < Epsilon &&
			       MathF.Abs(matrix.M22 - 1f) < Epsilon &&
			       MathF.Abs(matrix.M23) < Epsilon &&
			       MathF.Abs(matrix.M24) < Epsilon &&
			       MathF.Abs(matrix.M31) < Epsilon &&
			       MathF.Abs(matrix.M32) < Epsilon &&
			       MathF.Abs(matrix.M33 - 1f) < Epsilon &&
			       MathF.Abs(matrix.M34) < Epsilon &&
			       MathF.Abs(matrix.M41) < Epsilon &&
			       MathF.Abs(matrix.M42) < Epsilon &&
			       MathF.Abs(matrix.M43) < Epsilon &&
			       MathF.Abs(matrix.M44 - 1f) < Epsilon;
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
				return [];
			}

			var result = new Vector3[count];
			for (int i = 0; i < count; i++)
			{
				result[i] = buffer.ReadPoint3();
			}

			return result;
		}

		private static Quaternion[] ReadQuat16Array(DtsDataBuffer buffer, int count)
		{
			if (count <= 0)
			{
				return [];
			}

			var result = new Quaternion[count];
			for (int i = 0; i < count; i++)
			{
				result[i] = new Quaternion(
					buffer.ReadS16() / Quat16MaxValue,
					buffer.ReadS16() / Quat16MaxValue,
					buffer.ReadS16() / Quat16MaxValue,
					buffer.ReadS16() / Quat16MaxValue);
			}

			return result;
		}

		private static int[] ReadS32Array(DtsDataBuffer buffer, int count)
		{
			if (count <= 0)
			{
				return [];
			}

			var result = new int[count];
			for (int i = 0; i < count; i++)
			{
				result[i] = buffer.ReadS32();
			}

			return result;
		}

		private static Vector2[] ReadPoint2Array(DtsDataBuffer buffer, int count)
		{
			if (count <= 0)
			{
				return [];
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

		private static List<string> ReadMaterialList(BinaryReader reader, short version)
		{
			byte materialListVersion = reader.ReadByte();
			if (materialListVersion == 0)
			{
				return [];
			}

			if (materialListVersion != 1)
			{
				return ReadTextMaterialList(reader, materialListVersion);
			}

			uint materialCountValue = reader.ReadUInt32();
			if (materialCountValue > int.MaxValue)
			{
				throw new InvalidDataException("Invalid DTS material count.");
			}

			int materialCount = (int)materialCountValue;
			var materialNames = new List<string>(materialCount);
			for (int i = 0; i < materialCount; i++)
			{
				materialNames.Add(StripLegacyMaterialPath(ReadTorqueShortString(reader)));
			}

			SkipMaterialArrays(reader, version, materialCount);
			return materialNames;
		}

		private static string ReadTorqueShortString(BinaryReader reader)
		{
			int length = reader.ReadByte();
			byte[] bytes = reader.ReadBytes(length);
			if (bytes.Length != length)
			{
				throw new EndOfStreamException("Unexpected end of DTS file while reading material name.");
			}

			return System.Text.Encoding.UTF8.GetString(bytes);
		}

		private static List<string> ReadTextMaterialList(BinaryReader reader, byte firstByte)
		{
			var materialNames = new List<string>();
			var current = new List<byte> { firstByte };
			while (true)
			{
				int next = reader.BaseStream.ReadByte();
				if (next < 0)
				{
					if (current.Count > 0)
					{
						materialNames.Add(StripLegacyMaterialPath(System.Text.Encoding.UTF8.GetString(current.ToArray())));
					}

					return materialNames;
				}

				if (next is '\r' or '\n')
				{
					if (next == '\r' &&
					    reader.BaseStream.Position < reader.BaseStream.Length &&
					    reader.BaseStream.ReadByte() is not '\n')
					{
						reader.BaseStream.Seek(-1, SeekOrigin.Current);
					}

					if (current.Count == 0)
					{
						return materialNames;
					}

					materialNames.Add(StripLegacyMaterialPath(System.Text.Encoding.UTF8.GetString(current.ToArray())));
					current.Clear();
					continue;
				}

				current.Add((byte)next);
			}
		}

		private static string StripLegacyMaterialPath(string materialName)
		{
			if (string.IsNullOrEmpty(materialName))
			{
				return materialName;
			}

			int slashIndex = Math.Max(materialName.LastIndexOf('/'), materialName.LastIndexOf('\\'));
			return slashIndex >= 0 ? materialName[(slashIndex + 1)..] : materialName;
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

		private readonly record struct DtsNode(int NameIndex, int ParentIndex);
		private readonly record struct DtsObject(int NameIndex, int NumMeshes, int StartMeshIndex, int NodeIndex = -1);
		private readonly record struct DtsDetail(
			int NameIndex,
			int SubShapeNum,
			int ObjectDetailNum,
			float Size,
			float AverageError,
			float MaxError,
			int PolyCount);

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

			public void Align16To32()
			{
				if ((_i16 & 1) != 0)
				{
					_i16++;
					if (_i16 > _data16.Length)
					{
						throw new EndOfStreamException("Unexpected end of DTS 16-bit buffer while aligning to 32-bit boundary.");
					}
				}
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
}
