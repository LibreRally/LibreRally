using System;
using System.IO;
using LibreRally.Vehicle.Physics;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Definitions.Colliders;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using Stride.Rendering;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;

namespace LibreRally
{
	/// <summary>
	/// Builds the procedural ground plane and test-track surface sections used by the default scene.
	/// </summary>
	internal sealed class VehicleSpawnerTrackBuilder
	{
		private static readonly Logger Log = GlobalLogger.GetLogger("VehicleSpawnerTrackBuilder");
		private const float MinTrackUvScale = 0.25f;
		private const float TrackMeshBoundsHalfHeight = 0.02f;
		private const float TrackSurfaceLift = 0.01f;
		private const float BankedSectionRollAngleRadians = -0.35f;
		private const float InclineSectionPitchAngleRadians = 0.22f;
		private const float TrackSurfaceGlossiness = 0.2f;
		private const float TrackSurfaceMetalness = 0f;

		private TrackSurfaceMaterialLibrary? _trackSurfaceMaterialLibrary;

		/// <summary>
		/// Configures the root ground collider and attaches the procedural test-track segments.
		/// </summary>
		/// <param name="entity">The root scene entity that owns the ground setup.</param>
		/// <param name="game">The active Stride game instance.</param>
		public void ConfigureGround(Entity entity, Game game)
		{
			entity.Transform.Position = new Vector3(entity.Transform.Position.X, -0.5f, entity.Transform.Position.Z);
			entity.Add(new StaticComponent
			{
				FrictionCoefficient = 1.5f,
				Collider = new CompoundCollider
				{
					Colliders =
					{
						new BoxCollider { Size = new Vector3(500f, 1f, 500f) },
					},
				},
			});

			AddRoadTestSections(entity, game);
		}

		private void AddRoadTestSections(Entity parent, Game game)
		{
			try
			{
				var graphicsDevice = game.GraphicsDevice;
				_trackSurfaceMaterialLibrary ??= new TrackSurfaceMaterialLibrary(
					graphicsDevice,
					Path.Combine(AppContext.BaseDirectory, "Resources", "Track Materials"));
				var segments = new[]
				{
					new TrackSegmentDefinition("test_track_tarmac", new Vector3(0f, 0.5f, 0f), Quaternion.Identity, new Vector3(60f, 0.2f, 16f), SurfaceType.Tarmac, new Color4(0.21f, 0.21f, 0.22f), 7.5f, 1.35f),
					new TrackSegmentDefinition("test_track_gravel", new Vector3(-22f, 0.5f, 26f), Quaternion.Identity, new Vector3(28f, 0.25f, 14f), SurfaceType.Gravel, new Color4(0.47f, 0.40f, 0.29f), 5f, 1.05f),
					new TrackSegmentDefinition("test_track_snow", new Vector3(22f, 0.5f, 26f), Quaternion.Identity, new Vector3(28f, 0.25f, 14f), SurfaceType.Snow, new Color4(0.88f, 0.9f, 0.92f), 5f, 0.85f),
					new TrackSegmentDefinition("test_track_banked", new Vector3(38f, 1.8f, -6f), Quaternion.RotationZ(BankedSectionRollAngleRadians), new Vector3(20f, 0.25f, 28f), SurfaceType.Tarmac, new Color4(0.24f, 0.24f, 0.25f), 4f, 1.2f),
					new TrackSegmentDefinition("test_track_incline", new Vector3(0f, 1.7f, 40f), Quaternion.RotationX(InclineSectionPitchAngleRadians), new Vector3(12f, 0.25f, 42f), SurfaceType.Tarmac, new Color4(0.23f, 0.23f, 0.24f), 5.25f, 1.25f),
				};

				foreach (var segment in segments)
				{
					parent.AddChild(CreateTrackSegment(graphicsDevice, segment));
				}
			}
			catch (Exception ex)
			{
				Log.Warning($"Could not build test track sections: {ex}");
			}
		}

		private Entity CreateTrackSegment(GraphicsDevice graphicsDevice, in TrackSegmentDefinition segment)
		{
			var halfWidth = segment.ColliderSize.X * 0.5f;
			var halfLength = segment.ColliderSize.Z * 0.5f;
			var uvScale = MathF.Max(MinTrackUvScale, segment.UvScale);

			var vertices = new VertexPositionNormalTexture[]
			{
				new(new Vector3(-halfWidth, 0f, -halfLength), Vector3.UnitY, new Vector2(0f, 0f)),
				new(new Vector3(halfWidth, 0f, -halfLength), Vector3.UnitY, new Vector2(1f, 0f)),
				new(new Vector3(halfWidth, 0f, halfLength), Vector3.UnitY, new Vector2(1f, 1f)),
				new(new Vector3(-halfWidth, 0f, halfLength), Vector3.UnitY, new Vector2(0f, 1f)),
				new(new Vector3(-halfWidth, 0f, -halfLength), -Vector3.UnitY, new Vector2(0f, 0f)),
				new(new Vector3(halfWidth, 0f, -halfLength), -Vector3.UnitY, new Vector2(1f, 0f)),
				new(new Vector3(halfWidth, 0f, halfLength), -Vector3.UnitY, new Vector2(1f, 1f)),
				new(new Vector3(-halfWidth, 0f, halfLength), -Vector3.UnitY, new Vector2(0f, 1f)),
			};
			var indices = new[] { 0, 1, 2, 0, 2, 3, 4, 6, 5, 4, 7, 6 };

			var mesh = new Mesh
			{
				BoundingBox = new BoundingBox(
					new Vector3(-halfWidth, -TrackMeshBoundsHalfHeight, -halfLength),
					new Vector3(halfWidth, TrackMeshBoundsHalfHeight, halfLength)),
				Draw = new MeshDraw
				{
					PrimitiveType = PrimitiveType.TriangleList,
					VertexBuffers =
					[
						new VertexBufferBinding(
							Stride.Graphics.Buffer.Vertex.New(graphicsDevice, vertices),
							VertexPositionNormalTexture.Layout,
							vertices.Length),
					],
					IndexBuffer = new IndexBufferBinding(
						Stride.Graphics.Buffer.Index.New(graphicsDevice, indices),
						true,
						indices.Length),
					DrawCount = indices.Length,
				},
			};

			var material = _trackSurfaceMaterialLibrary?.CreateMaterial(
				               segment.SurfaceType,
				               uvScale,
				               segment.Albedo,
				               TrackSurfaceGlossiness,
				               TrackSurfaceMetalness)
			               ?? Material.New(graphicsDevice, new MaterialDescriptor
			               {
				               Attributes = new MaterialAttributes
				               {
					               Diffuse = new MaterialDiffuseMapFeature(new ComputeColor { Value = segment.Albedo }),
					               DiffuseModel = new MaterialDiffuseLambertModelFeature(),
					               MicroSurface = new MaterialGlossinessMapFeature(new ComputeFloat { Value = TrackSurfaceGlossiness }),
					               Specular = new MaterialMetalnessMapFeature(new ComputeFloat { Value = TrackSurfaceMetalness }),
				               },
			               });

			var trackEntity = new Entity(segment.Name);
			trackEntity.Transform.Position = segment.LocalPosition + new Vector3(0f, TrackSurfaceLift, 0f);
			trackEntity.Transform.Rotation = segment.LocalRotation;
			trackEntity.Add(new StaticComponent
			{
				FrictionCoefficient = segment.FrictionCoefficient,
				Collider = new CompoundCollider
				{
					Colliders =
					{
						new BoxCollider
						{
							Size = segment.ColliderSize,
							PositionLocal = new Vector3(0f, -segment.ColliderSize.Y * 0.5f, 0f),
						},
					},
				},
			});
			trackEntity.Add(new TrackSurfaceComponent { SurfaceType = segment.SurfaceType });
			trackEntity.Add(new ModelComponent { Model = new Model { mesh, material } });
			return trackEntity;
		}

		private readonly record struct TrackSegmentDefinition(
			string Name,
			Vector3 LocalPosition,
			Quaternion LocalRotation,
			Vector3 ColliderSize,
			SurfaceType SurfaceType,
			Color4 Albedo,
			float UvScale,
			float FrictionCoefficient);
	}
}
