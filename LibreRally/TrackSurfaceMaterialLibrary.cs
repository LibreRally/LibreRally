using System;
using System.Collections.Generic;
using System.IO;
using Stride.Core.Mathematics;
using Stride.Graphics;
using Stride.Rendering;
using Stride.Rendering.Materials;
using Stride.Rendering.Materials.ComputeColors;
using LibreRally.Vehicle.Physics;

namespace LibreRally
{
	internal sealed class TrackSurfaceMaterialLibrary
	{
		private readonly GraphicsDevice _graphicsDevice;
		private readonly string _materialsRoot;
		private readonly Dictionary<SurfaceType, TrackSurfaceTextureSet> _textureSets = [];

		public TrackSurfaceMaterialLibrary(GraphicsDevice graphicsDevice, string materialsRoot)
		{
			_graphicsDevice = graphicsDevice ?? throw new ArgumentNullException(nameof(graphicsDevice));
			_materialsRoot = materialsRoot ?? throw new ArgumentNullException(nameof(materialsRoot));
		}

		public Material CreateMaterial(SurfaceType surfaceType, float uvScale, Color4 fallbackColor, float fallbackGlossiness, float fallbackMetalness)
		{
			var textureSet = GetTextureSet(surfaceType);
			var tiledScale = new Vector2(MathF.Max(0.01f, uvScale), MathF.Max(0.01f, uvScale));

			IComputeColor diffuse = textureSet.ColorTexture != null
				? CreateColorMap(textureSet.ColorTexture, tiledScale)
				: new ComputeColor(fallbackColor);
			var microSurface = textureSet.RoughnessTexture != null
				? new MaterialGlossinessMapFeature(CreateScalarMap(textureSet.RoughnessTexture, tiledScale))
				{
					// AmbientCG supplies roughness; Stride expects glossiness for this feature.
					Invert = true,
				}
				: new MaterialGlossinessMapFeature(new ComputeFloat { Value = fallbackGlossiness });

			var descriptor = new MaterialDescriptor
			{
				Attributes = new MaterialAttributes
				{
					Diffuse = new MaterialDiffuseMapFeature(diffuse),
					DiffuseModel = new MaterialDiffuseLambertModelFeature(),
					// Procedural track quads currently provide position/normal/uv only, so keep the
					// shader path to texture-safe features until they have a tangent basis.
					MicroSurface = microSurface,
					Specular = new MaterialMetalnessMapFeature(new ComputeFloat { Value = fallbackMetalness }),
					SpecularModel = new MaterialSpecularMicrofacetModelFeature(),
				},
			};

			return Material.New(_graphicsDevice, descriptor);
		}

		private TrackSurfaceTextureSet GetTextureSet(SurfaceType surfaceType)
		{
			if (_textureSets.TryGetValue(surfaceType, out var cached))
			{
				return cached;
			}

			var descriptor = ResolveDescriptor(surfaceType);
			var folder = Path.Combine(_materialsRoot, descriptor.FolderName);
			var textureSet = new TrackSurfaceTextureSet(
				TryLoadTexture(folder, descriptor.ColorFileNames),
				TryLoadTexture(folder, descriptor.NormalFileNames),
				TryLoadTexture(folder, descriptor.RoughnessFileNames),
				TryLoadTexture(folder, descriptor.AmbientOcclusionFileNames),
				TryLoadTexture(folder, descriptor.DisplacementFileNames));

			_textureSets[surfaceType] = textureSet;
			return textureSet;
		}

		private static TrackSurfaceDescriptor ResolveDescriptor(SurfaceType surfaceType) => surfaceType switch
		{
			SurfaceType.Gravel => new TrackSurfaceDescriptor(
				"Ground079S",
				["Ground079S_1K-JPG_Color.jpg"],
				["Ground079S_1K-JPG_NormalDX.jpg"],
				["Ground079S_1K-JPG_Roughness.jpg"],
				["Ground079S_1K-JPG_AmbientOcclusion.jpg"],
				["Ground079S_1K-JPG_Displacement.jpg"]),
			SurfaceType.Snow => new TrackSurfaceDescriptor(
				"Snow010A",
				["Snow010A_1K-PNG_Color.png", "Snow010A_1K-JPG_Color.jpg"],
				["Snow010A_1K-PNG_NormalDX.png", "Snow010A_1K-JPG_NormalDX.jpg"],
				["Snow010A_1K-PNG_Roughness.png", "Snow010A_1K-JPG_Roughness.jpg"],
				["Snow010A_1K-PNG_AmbientOcclusion.png", "Snow010A_1K-JPG_AmbientOcclusion.jpg"],
				["Snow010A_1K-PNG_Displacement.png", "Snow010A_1K-JPG_Displacement.jpg"]),
			_ => new TrackSurfaceDescriptor(
				"Road012A",
				["Road012A_1K-JPG_Color.jpg"],
				["Road012A_1K-JPG_NormalDX.jpg"],
				["Road012A_1K-JPG_Roughness.jpg"],
				["Road012A_1K-JPG_AmbientOcclusion.jpg"],
				["Road012A_1K-JPG_Displacement.jpg"]),
		};

		private static ComputeTextureColor CreateColorMap(Texture texture, Vector2 scale) => new(texture)
		{
			Scale = scale,
			AddressModeU = TextureAddressMode.Wrap,
			AddressModeV = TextureAddressMode.Wrap,
		};

		private static ComputeTextureScalar CreateScalarMap(Texture texture, Vector2 scale) => new()
		{
			Texture = texture,
			Scale = scale,
			Channel = ColorChannel.R,
			AddressModeU = TextureAddressMode.Wrap,
			AddressModeV = TextureAddressMode.Wrap,
		};

		private Texture? TryLoadTexture(string folderPath, IReadOnlyList<string> candidateFileNames)
		{
			foreach (var fileName in candidateFileNames)
			{
				var absolutePath = Path.Combine(folderPath, fileName);
				var texture = TryLoadTexture(absolutePath);
				if (texture != null)
				{
					return texture;
				}
			}

			return null;
		}

		private Texture? TryLoadTexture(string absolutePath)
		{
			if (!File.Exists(absolutePath))
			{
				return null;
			}

			try
			{
				using var stream = File.OpenRead(absolutePath);
				using var image = Image.Load(stream);
				return Texture.New(_graphicsDevice, image);
			}
			catch (Exception ex)
			{
				System.Diagnostics.Debug.WriteLine($"Could not load track texture '{absolutePath}': {ex.Message}");
				return null;
			}
		}

		private readonly record struct TrackSurfaceDescriptor(
			string FolderName,
			IReadOnlyList<string> ColorFileNames,
			IReadOnlyList<string> NormalFileNames,
			IReadOnlyList<string> RoughnessFileNames,
			IReadOnlyList<string> AmbientOcclusionFileNames,
			IReadOnlyList<string> DisplacementFileNames);

		private sealed record TrackSurfaceTextureSet(
			Texture? ColorTexture,
			Texture? NormalTexture,
			Texture? RoughnessTexture,
			Texture? AmbientOcclusionTexture,
			Texture? DisplacementTexture);
	}
}
