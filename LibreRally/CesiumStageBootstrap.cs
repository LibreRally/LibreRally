using System;
using System.Collections.Generic;
using System.Globalization;
using Stride.Core.Mathematics;

namespace LibreRally
{
	/// <summary>
	/// Known environment variables that drive code-first startup scene selection.
	/// </summary>
	internal static class LibreRallyEnvironmentVariables
	{
		public const string SceneMode = "LIBRERALLY_SCENE_MODE";
		public const string CesiumStage = "LIBRERALLY_CESIUM_STAGE";
		public const string CesiumUrl = "LIBRERALLY_CESIUM_URL";
		public const string CesiumLocalFile = "LIBRERALLY_CESIUM_LOCAL_FILE";
		public const string CesiumIonAssetId = "LIBRERALLY_CESIUM_ION_ASSET_ID";
		public const string CesiumIonAccessToken = "LIBRERALLY_CESIUM_ION_ACCESS_TOKEN";
		public const string CesiumOriginLongitude = "LIBRERALLY_CESIUM_ORIGIN_LONGITUDE";
		public const string CesiumOriginLatitude = "LIBRERALLY_CESIUM_ORIGIN_LATITUDE";
		public const string CesiumOriginHeight = "LIBRERALLY_CESIUM_ORIGIN_HEIGHT";
		public const string CesiumMaximumScreenSpaceError = "LIBRERALLY_CESIUM_MAX_SCREEN_SPACE_ERROR";
		public const string CesiumMaximumTileLoads = "LIBRERALLY_CESIUM_MAX_TILE_LOADS";
		public const string CesiumMaximumCachedBytes = "LIBRERALLY_CESIUM_MAX_CACHED_BYTES";
	}

	/// <summary>
	/// Captures the fully resolved startup scene choice for the current runtime session.
	/// </summary>
	/// <param name="SceneMode">The high-level scene mode to boot.</param>
	/// <param name="CesiumStage">The resolved Cesium stage configuration used when <paramref name="SceneMode" /> is Cesium.</param>
	internal sealed record StartupSceneSelection(
		StartupSceneMode SceneMode,
		CesiumStageBootstrapConfiguration CesiumStage)
	{
		/// <summary>
		/// Resolves the startup scene selection from environment variables.
		/// </summary>
		/// <param name="getEnvironmentVariable">Environment variable accessor.</param>
		/// <returns>The resolved startup scene selection.</returns>
		public static StartupSceneSelection FromEnvironment(Func<string, string?> getEnvironmentVariable)
		{
			ArgumentNullException.ThrowIfNull(getEnvironmentVariable);
			return new StartupSceneSelection(
				CodeFirstSceneFactory.ResolveStartupSceneMode(getEnvironmentVariable(LibreRallyEnvironmentVariables.SceneMode)),
				CesiumStageBootstrapConfiguration.FromEnvironment(getEnvironmentVariable));
		}
	}

	/// <summary>
	/// Defines the camera setup used by a code-first scene bootstrap.
	/// </summary>
	/// <param name="Position">Initial camera position in local scene coordinates.</param>
	/// <param name="Rotation">Initial camera rotation.</param>
	/// <param name="KeyboardMovementSpeed">Free-fly keyboard movement speed.</param>
	/// <param name="TouchMovementSpeed">Touch movement speed.</param>
	/// <param name="SpeedFactor">Camera acceleration multiplier.</param>
	/// <param name="KeyboardRotationSpeed">Keyboard rotation speed.</param>
	/// <param name="MouseRotationSpeed">Mouse rotation speed.</param>
	/// <param name="TouchRotationSpeed">Touch rotation speed.</param>
	internal sealed record SceneCameraBootstrapConfiguration(
		Vector3 Position,
		Quaternion Rotation,
		Vector3? LookAtTarget,
		Vector3 KeyboardMovementSpeed,
		Vector3 TouchMovementSpeed,
		float SpeedFactor,
		Vector2 KeyboardRotationSpeed,
		Vector2 MouseRotationSpeed,
		Vector2 TouchRotationSpeed);

	/// <summary>
	/// Describes a code-first Cesium startup stage preset plus any environment overrides.
	/// </summary>
	/// <param name="StageKey">Stable key used to identify the stage preset.</param>
	/// <param name="DisplayName">Contributor-facing display name for the stage.</param>
	/// <param name="Description">Short description of what the stage loads.</param>
	/// <param name="DataSource">Tileset source type.</param>
	/// <param name="Url">Public or local tileset URL when <paramref name="DataSource" /> is Url.</param>
	/// <param name="LocalFilePath">Absolute or relative tileset.json path when <paramref name="DataSource" /> is LocalFile.</param>
	/// <param name="IonAssetId">Cesium ion asset ID when <paramref name="DataSource" /> is CesiumIon.</param>
	/// <param name="IonAccessToken">Cesium ion access token when <paramref name="DataSource" /> is CesiumIon.</param>
	/// <param name="OriginLongitudeDegrees">Longitude of the local georeference origin.</param>
	/// <param name="OriginLatitudeDegrees">Latitude of the local georeference origin.</param>
	/// <param name="OriginHeightMeters">Height of the local georeference origin in meters.</param>
	/// <param name="MaximumScreenSpaceError">Maximum allowed screen-space error.</param>
	/// <param name="MaximumSimultaneousTileLoads">Maximum simultaneous tile loads.</param>
	/// <param name="MaximumCachedBytes">Maximum cached tileset bytes retained by Cesium Native.</param>
	/// <param name="AutoFrameOnInitialTile">Whether the runtime may reposition the startup camera to frame the first committed tile.</param>
	/// <param name="Camera">Initial camera bootstrap settings.</param>
	/// <param name="SpawnLongitudeDegrees">Longitude of the gameplay spawn anchor.</param>
	/// <param name="SpawnLatitudeDegrees">Latitude of the gameplay spawn anchor.</param>
	/// <param name="SpawnHeightMeters">Height of the gameplay spawn anchor in meters.</param>
	/// <param name="SpawnEastUpNorthOffset">Spawn offset in the local east/up/north frame.</param>
	internal sealed record CesiumStageBootstrapConfiguration(
		string StageKey,
		string DisplayName,
		string Description,
		CesiumTilesetDataSource DataSource,
		string Url,
		string LocalFilePath,
		long IonAssetId,
		string IonAccessToken,
		double OriginLongitudeDegrees,
		double OriginLatitudeDegrees,
		double OriginHeightMeters,
		float MaximumScreenSpaceError,
		uint MaximumSimultaneousTileLoads,
		long MaximumCachedBytes,
		bool AutoFrameOnInitialTile,
		SceneCameraBootstrapConfiguration Camera,
		double SpawnLongitudeDegrees,
		double SpawnLatitudeDegrees,
		double SpawnHeightMeters,
		Vector3 SpawnEastUpNorthOffset)
	{
		private const string SampleTilesetUrl = "https://raw.githubusercontent.com/CesiumGS/3d-tiles-samples/main/1.0/TilesetWithRequestVolume/tileset.json";
		private const long DefaultMaximumCachedBytes = 268_435_456L;
		private const double MelbourneLongitudeDegrees = 144.9673305d;
		private const double MelbourneLatitudeDegrees = -37.8175087d;
		private const double MelbourneHeightMeters = 20d;

		/// <summary>
		/// Gets the root entity name used by the scene bootstrap.
		/// </summary>
		public string StageEntityName => $"{DisplayName} Stage";

		/// <summary>
		/// Gets the gameplay spawn-anchor entity name used by the scene bootstrap.
		/// </summary>
		public string SpawnAnchorEntityName => $"{StageKey}_spawn_anchor";

		/// <summary>
		/// Resolves a Cesium stage from the runtime environment.
		/// </summary>
		/// <param name="getEnvironmentVariable">Environment variable accessor.</param>
		/// <returns>The resolved stage configuration.</returns>
		public static CesiumStageBootstrapConfiguration FromEnvironment(Func<string, string?> getEnvironmentVariable)
		{
			ArgumentNullException.ThrowIfNull(getEnvironmentVariable);

			var preset = ResolvePreset(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumStage));
			var originLongitudeOverride = GetDouble(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumOriginLongitude));
			var originLatitudeOverride = GetDouble(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumOriginLatitude));
			var originHeightOverride = GetDouble(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumOriginHeight));
			var hasExplicitOriginOverride = originLongitudeOverride.HasValue || originLatitudeOverride.HasValue || originHeightOverride.HasValue;
			var configuration = preset with
			{
				Url = string.Equals(preset.StageKey, "url", StringComparison.OrdinalIgnoreCase)
					? GetNonEmpty(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumUrl)) ?? preset.Url
					: preset.Url,
				LocalFilePath = string.Equals(preset.StageKey, "file", StringComparison.OrdinalIgnoreCase)
					? GetNonEmpty(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumLocalFile)) ?? preset.LocalFilePath
					: preset.LocalFilePath,
				IonAssetId = string.Equals(preset.StageKey, "ion", StringComparison.OrdinalIgnoreCase)
					? GetInt64(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumIonAssetId)) ?? preset.IonAssetId
					: preset.IonAssetId,
				IonAccessToken = preset.DataSource == CesiumTilesetDataSource.CesiumIon
					? GetNonEmpty(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumIonAccessToken)) ?? preset.IonAccessToken
					: preset.IonAccessToken,
				OriginLongitudeDegrees = originLongitudeOverride ?? preset.OriginLongitudeDegrees,
				OriginLatitudeDegrees = originLatitudeOverride ?? preset.OriginLatitudeDegrees,
				OriginHeightMeters = originHeightOverride ?? preset.OriginHeightMeters,
				SpawnLongitudeDegrees = originLongitudeOverride ?? preset.SpawnLongitudeDegrees,
				SpawnLatitudeDegrees = originLatitudeOverride ?? preset.SpawnLatitudeDegrees,
				SpawnHeightMeters = originHeightOverride ?? preset.SpawnHeightMeters,
				MaximumScreenSpaceError = GetSingle(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumMaximumScreenSpaceError)) ?? preset.MaximumScreenSpaceError,
				MaximumSimultaneousTileLoads = GetUInt32(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumMaximumTileLoads)) ?? preset.MaximumSimultaneousTileLoads,
				MaximumCachedBytes = GetInt64(getEnvironmentVariable(LibreRallyEnvironmentVariables.CesiumMaximumCachedBytes)) ?? preset.MaximumCachedBytes,
				AutoFrameOnInitialTile = hasExplicitOriginOverride ? false : preset.AutoFrameOnInitialTile,
			};
			configuration.Validate();
			return configuration;
		}

		/// <summary>
		/// Builds the curated set of playable Cesium tracks for the pause-menu track selector.
		/// </summary>
		/// <param name="runtimeConfiguration">The runtime configuration providing shared token and origin overrides.</param>
		/// <returns>The curated track list plus any active custom stage that is not already represented.</returns>
		internal static IReadOnlyList<CesiumStageBootstrapConfiguration> BuildTrackSelectionStages(CesiumStageBootstrapConfiguration runtimeConfiguration)
		{
			ArgumentNullException.ThrowIfNull(runtimeConfiguration);

			var stages = new List<CesiumStageBootstrapConfiguration>
			{
				CreateMelbournePhotogrammetryStage().ApplySharedRuntimeOverrides(runtimeConfiguration),
				CreateGooglePhotorealistic3DTilesStage().ApplySharedRuntimeOverrides(runtimeConfiguration),
				CreateCesiumOsmBuildingsStage().ApplySharedRuntimeOverrides(runtimeConfiguration),
			};
			if (!stages.Exists(stage => MatchesTrackIdentity(stage, runtimeConfiguration))
			    && ShouldIncludeRuntimeStageInTrackList(runtimeConfiguration))
			{
				stages.Add(runtimeConfiguration);
			}

			return stages;
		}

		/// <summary>
		/// Determines whether two stage configurations refer to the same playable track.
		/// </summary>
		/// <param name="left">The left stage configuration.</param>
		/// <param name="right">The right stage configuration.</param>
		/// <returns><see langword="true" /> when the configurations refer to the same source track.</returns>
		internal static bool MatchesTrackIdentity(CesiumStageBootstrapConfiguration left, CesiumStageBootstrapConfiguration right)
		{
			ArgumentNullException.ThrowIfNull(left);
			ArgumentNullException.ThrowIfNull(right);

			if (string.Equals(left.StageKey, right.StageKey, StringComparison.OrdinalIgnoreCase))
			{
				return true;
			}

			if (left.DataSource != right.DataSource)
			{
				return false;
			}

			return left.DataSource switch
			{
				CesiumTilesetDataSource.CesiumIon => left.IonAssetId > 0 && left.IonAssetId == right.IonAssetId,
				CesiumTilesetDataSource.LocalFile => string.Equals(left.LocalFilePath, right.LocalFilePath, StringComparison.OrdinalIgnoreCase),
				_ => string.Equals(left.Url, right.Url, StringComparison.OrdinalIgnoreCase),
			};
		}

		/// <summary>
		/// Resolves a named stage preset.
		/// </summary>
		/// <param name="value">Preset selector.</param>
		/// <returns>The resolved stage preset.</returns>
		internal static CesiumStageBootstrapConfiguration ResolvePreset(string? value)
		{
			if (string.Equals(value, "melbourne", StringComparison.OrdinalIgnoreCase)
			    || string.Equals(value, "melbourne-photogrammetry", StringComparison.OrdinalIgnoreCase))
			{
				return CreateMelbournePhotogrammetryStage();
			}

			if (string.Equals(value, "google", StringComparison.OrdinalIgnoreCase)
			    || string.Equals(value, "google-photorealistic", StringComparison.OrdinalIgnoreCase)
			    || string.Equals(value, "google-photorealistic-3d-tiles", StringComparison.OrdinalIgnoreCase))
			{
				return CreateGooglePhotorealistic3DTilesStage();
			}

			if (string.Equals(value, "osm", StringComparison.OrdinalIgnoreCase)
			    || string.Equals(value, "osm-buildings", StringComparison.OrdinalIgnoreCase)
			    || string.Equals(value, "cesium-osm-buildings", StringComparison.OrdinalIgnoreCase))
			{
				return CreateCesiumOsmBuildingsStage();
			}

			if (string.Equals(value, "ion", StringComparison.OrdinalIgnoreCase))
			{
				return CreateIonStage();
			}

			if (string.Equals(value, "file", StringComparison.OrdinalIgnoreCase))
			{
				return CreateLocalFileStage();
			}

			if (string.Equals(value, "url", StringComparison.OrdinalIgnoreCase))
			{
				return CreateUrlStage();
			}

			return CreateSampleStage();
		}

		/// <summary>
		/// Creates the public sample stage used by default.
		/// </summary>
		/// <returns>The default sample stage configuration.</returns>
		internal static CesiumStageBootstrapConfiguration CreateSampleStage() =>
			new(
				StageKey: "sample",
				DisplayName: "Cesium Sample",
				Description: "Streams Cesium's public TilesetWithRequestVolume sample dataset.",
				DataSource: CesiumTilesetDataSource.Url,
				Url: SampleTilesetUrl,
				LocalFilePath: string.Empty,
				IonAssetId: 0,
				IonAccessToken: string.Empty,
				OriginLongitudeDegrees: -75.612094,
				OriginLatitudeDegrees: 40.042531,
				OriginHeightMeters: 20d,
				MaximumScreenSpaceError: 18f,
				MaximumSimultaneousTileLoads: 8,
				MaximumCachedBytes: DefaultMaximumCachedBytes,
				AutoFrameOnInitialTile: true,
				Camera: new SceneCameraBootstrapConfiguration(
					Position: new Vector3(0f, 55f, 160f),
					Rotation: Quaternion.Identity,
					LookAtTarget: Vector3.Zero,
					KeyboardMovementSpeed: new Vector3(30f),
					TouchMovementSpeed: new Vector3(4f, 4f, 1.2f),
					SpeedFactor: 6f,
					KeyboardRotationSpeed: new Vector2(2.2f),
					MouseRotationSpeed: new Vector2(0.35f, 0.35f),
					TouchRotationSpeed: new Vector2(0.5f, 0.35f)),
				SpawnLongitudeDegrees: -75.612094,
				SpawnLatitudeDegrees: 40.042531,
				SpawnHeightMeters: 20d,
				SpawnEastUpNorthOffset: new Vector3(0f, 2.5f, 0f));

		/// <summary>
		/// Validates contributor-provided runtime settings before Cesium scene startup proceeds.
		/// </summary>
		/// <exception cref="InvalidOperationException">Thrown when the Cesium stage configuration is invalid.</exception>
		internal void Validate()
		{
			if (OriginLongitudeDegrees is < -180d or > 180d)
			{
				throw new InvalidOperationException(
					$"{LibreRallyEnvironmentVariables.CesiumOriginLongitude} must be between -180 and 180 degrees. Current value: {OriginLongitudeDegrees.ToString(CultureInfo.InvariantCulture)}.");
			}

			if (OriginLatitudeDegrees is < -90d or > 90d)
			{
				throw new InvalidOperationException(
					$"{LibreRallyEnvironmentVariables.CesiumOriginLatitude} must be between -90 and 90 degrees. Current value: {OriginLatitudeDegrees.ToString(CultureInfo.InvariantCulture)}. " +
					$"If this looks like a longitude, check whether {LibreRallyEnvironmentVariables.CesiumOriginLongitude} and {LibreRallyEnvironmentVariables.CesiumOriginLatitude} are swapped.");
			}

			if (SpawnLongitudeDegrees is < -180d or > 180d)
			{
				throw new InvalidOperationException(
					$"Cesium spawn longitude must be between -180 and 180 degrees. Current value: {SpawnLongitudeDegrees.ToString(CultureInfo.InvariantCulture)}.");
			}

			if (SpawnLatitudeDegrees is < -90d or > 90d)
			{
				throw new InvalidOperationException(
					$"Cesium spawn latitude must be between -90 and 90 degrees. Current value: {SpawnLatitudeDegrees.ToString(CultureInfo.InvariantCulture)}.");
			}
		}

		private static CesiumStageBootstrapConfiguration CreateUrlStage()
		{
			var sample = CreateSampleStage();
			return sample with
			{
				StageKey = "url",
				DisplayName = "Cesium URL",
				Description = "Loads a contributor-supplied tileset.json URL via LIBRERALLY_CESIUM_URL.",
			};
		}

		private static CesiumStageBootstrapConfiguration CreateIonStage()
		{
			var sample = CreateSampleStage();
			return sample with
			{
				StageKey = "ion",
				DisplayName = "Cesium Ion",
				Description = "Loads a Cesium ion asset configured by LIBRERALLY_CESIUM_ION_ASSET_ID and LIBRERALLY_CESIUM_ION_ACCESS_TOKEN.",
				DataSource = CesiumTilesetDataSource.CesiumIon,
				Url = string.Empty,
			};
		}

		private CesiumStageBootstrapConfiguration ApplySharedRuntimeOverrides(CesiumStageBootstrapConfiguration runtimeConfiguration)
		{
			return this with
			{
				IonAccessToken = DataSource == CesiumTilesetDataSource.CesiumIon ? runtimeConfiguration.IonAccessToken : IonAccessToken,
				OriginLongitudeDegrees = runtimeConfiguration.OriginLongitudeDegrees,
				OriginLatitudeDegrees = runtimeConfiguration.OriginLatitudeDegrees,
				OriginHeightMeters = runtimeConfiguration.OriginHeightMeters,
				SpawnLongitudeDegrees = runtimeConfiguration.SpawnLongitudeDegrees,
				SpawnLatitudeDegrees = runtimeConfiguration.SpawnLatitudeDegrees,
				SpawnHeightMeters = runtimeConfiguration.SpawnHeightMeters,
				SpawnEastUpNorthOffset = runtimeConfiguration.SpawnEastUpNorthOffset,
				MaximumScreenSpaceError = runtimeConfiguration.MaximumScreenSpaceError,
				MaximumSimultaneousTileLoads = runtimeConfiguration.MaximumSimultaneousTileLoads,
				MaximumCachedBytes = runtimeConfiguration.MaximumCachedBytes,
				AutoFrameOnInitialTile = runtimeConfiguration.AutoFrameOnInitialTile,
			};
		}

		private static bool ShouldIncludeRuntimeStageInTrackList(CesiumStageBootstrapConfiguration runtimeConfiguration)
		{
			if (string.Equals(runtimeConfiguration.StageKey, "ion", StringComparison.OrdinalIgnoreCase))
			{
				return runtimeConfiguration.IonAssetId > 0;
			}

			if (string.Equals(runtimeConfiguration.StageKey, "url", StringComparison.OrdinalIgnoreCase))
			{
				return !string.IsNullOrWhiteSpace(runtimeConfiguration.Url);
			}

			return string.Equals(runtimeConfiguration.StageKey, "file", StringComparison.OrdinalIgnoreCase)
			       && !string.IsNullOrWhiteSpace(runtimeConfiguration.LocalFilePath);
		}

		private static CesiumStageBootstrapConfiguration CreateMelbournePhotogrammetryStage() =>
			CreateCuratedIonTrackStage(
				stageKey: "melbourne-photogrammetry",
				displayName: "Melbourne Photogrammetry",
				description: "Loads Cesium ion Melbourne Photogrammetry (asset 69380).",
				ionAssetId: 69380);

		private static CesiumStageBootstrapConfiguration CreateGooglePhotorealistic3DTilesStage() =>
			CreateCuratedIonTrackStage(
				stageKey: "google-photorealistic-3d-tiles",
				displayName: "Google Photorealistic 3D Tiles",
				description: "Loads Google Photorealistic 3D Tiles from Cesium ion (asset 2275207).",
				ionAssetId: 2275207);

		private static CesiumStageBootstrapConfiguration CreateCesiumOsmBuildingsStage() =>
			CreateCuratedIonTrackStage(
				stageKey: "cesium-osm-buildings",
				displayName: "Cesium OSM Buildings",
				description: "Loads Cesium OSM Buildings from Cesium ion (asset 96188).",
				ionAssetId: 96188);

		private static CesiumStageBootstrapConfiguration CreateCuratedIonTrackStage(
			string stageKey,
			string displayName,
			string description,
			long ionAssetId)
		{
			var ionStage = CreateIonStage();
			return ionStage with
			{
				StageKey = stageKey,
				DisplayName = displayName,
				Description = description,
				IonAssetId = ionAssetId,
				OriginLongitudeDegrees = MelbourneLongitudeDegrees,
				OriginLatitudeDegrees = MelbourneLatitudeDegrees,
				OriginHeightMeters = MelbourneHeightMeters,
				AutoFrameOnInitialTile = false,
				Camera = CreateCuratedTrackCamera(),
				SpawnLongitudeDegrees = MelbourneLongitudeDegrees,
				SpawnLatitudeDegrees = MelbourneLatitudeDegrees,
				SpawnHeightMeters = MelbourneHeightMeters,
				SpawnEastUpNorthOffset = new Vector3(0f, 2.5f, 0f),
			};
		}

		private static SceneCameraBootstrapConfiguration CreateCuratedTrackCamera() =>
			new(
				Position: new Vector3(0f, 96f, 28f),
				Rotation: Quaternion.Identity,
				LookAtTarget: Vector3.Zero,
				KeyboardMovementSpeed: new Vector3(30f),
				TouchMovementSpeed: new Vector3(4f, 4f, 1.2f),
				SpeedFactor: 6f,
				KeyboardRotationSpeed: new Vector2(2.2f),
				MouseRotationSpeed: new Vector2(0.35f, 0.35f),
				TouchRotationSpeed: new Vector2(0.5f, 0.35f));

		private static CesiumStageBootstrapConfiguration CreateLocalFileStage()
		{
			var sample = CreateSampleStage();
			return sample with
			{
				StageKey = "file",
				DisplayName = "Cesium Local File",
				Description = "Loads a local tileset.json configured by LIBRERALLY_CESIUM_LOCAL_FILE.",
				DataSource = CesiumTilesetDataSource.LocalFile,
				Url = string.Empty,
				LocalFilePath = string.Empty,
			};
		}

		private static string? GetNonEmpty(string? value) =>
			string.IsNullOrWhiteSpace(value) ? null : value.Trim();

		private static double? GetDouble(string? value) =>
			TryParse<double>(value, NumberStyles.Float | NumberStyles.AllowThousands, double.TryParse);

		private static float? GetSingle(string? value) =>
			TryParse<float>(value, NumberStyles.Float | NumberStyles.AllowThousands, float.TryParse);

		private static long? GetInt64(string? value) =>
			TryParse<long>(value, NumberStyles.Integer, long.TryParse);

		private static uint? GetUInt32(string? value) =>
			TryParse<uint>(value, NumberStyles.Integer, uint.TryParse);

		private delegate bool TryParseNumber<T>(string value, NumberStyles style, IFormatProvider? provider, out T result);

		private static T? TryParse<T>(string? value, NumberStyles style, TryParseNumber<T> parser)
			where T : struct
		{
			if (string.IsNullOrWhiteSpace(value))
			{
				return null;
			}

			return parser(value.Trim(), style, CultureInfo.InvariantCulture, out var parsed)
				? parsed
				: null;
		}
	}
}
