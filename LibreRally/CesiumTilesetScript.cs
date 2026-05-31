using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using Evergine.Bindings.CesiumNative;
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
	/// Identifies the Cesium dataset source used by <see cref="CesiumTilesetScript" />.
	/// </summary>
	public enum CesiumTilesetDataSource
	{
		/// <summary>
		/// Loads tiles from a public or local <c>tileset.json</c> URL.
		/// </summary>
		Url,

		/// <summary>
		/// Loads tiles from a local <c>tileset.json</c> file on disk.
		/// </summary>
		LocalFile,

		/// <summary>
		/// Loads tiles from a Cesium ion asset using an asset ID and access token.
		/// </summary>
		CesiumIon,
	}

	/// <summary>
	/// Streams a Cesium 3D Tiles dataset into a Stride scene using the Cesium Native .NET bindings.
	/// </summary>
	/// <remarks>
	/// This is intentionally code-first and scene-runtime driven so LibreRally can host Cesium content without
	/// relying on Unity editor extensions or plugin workflows.
	/// </remarks>
	[ComponentCategory("LibreRally")]
	public unsafe class CesiumTilesetScript : SyncScript
	{
		private static readonly Logger ScriptLog = GlobalLogger.GetLogger(nameof(CesiumTilesetScript));
		private static readonly RendererResourceCallbacksSet RendererCallbacks = new()
		{
			PrepareInLoadThread = PrepareInLoadThread,
			PrepareInMainThread = PrepareInMainThread,
			FreeResources = FreeResources,
		};
		private static readonly TilesetLoadErrorCallback LoadErrorCallback = OnTilesetLoadError;
		private static readonly RootTileAvailableCallback RootTileAvailableCallback = OnRootTileAvailable;
		private const string SampleTilesetUrl = "https://raw.githubusercontent.com/CesiumGS/3d-tiles-samples/main/1.0/TilesetWithRequestVolume/tileset.json";
		private const long DefaultMaximumCachedBytes = 268_435_456L;
		private const float MinimumPhysicsProxyExtent = 0.5f;
		private const float MaximumVisibleTileDistanceWithoutRecovery = 750f;
		private const int TileVisibilityGraceFrames = 12;
		private static int _viewDiagnosticsCount;
		private readonly object _entityGate = new();
		private readonly Queue<nint> _releaseQueue = new();
		private readonly HashSet<nint> _queuedTileReleases = new();
		private readonly Dictionary<nint, CesiumTileSceneNode> _tileEntities = new();
		private readonly HashSet<nint> _tilesToRenderThisFrame = new();
		private readonly Dictionary<CesiumMaterialCacheKey, Material> _materialCache = new();

		private GCHandle _selfHandle;
		private bool _initialized;
		private bool _disposed;
		private AsyncSystem _asyncSystem;
		private AssetAccessor _assetAccessor;
		private CreditSystem _creditSystem;
		private TilesetExternals _externals;
		private Tileset _tileset;
		private Ellipsoid _ellipsoid;
		private CesiumLocalGeoreference? _georeference;
		private CesiumGeoreferenceComponent? _georeferenceComponent;
		private int _appliedGeoreferenceRevision = int.MinValue;
		private CameraComponent? _camera;
		private Scene? _scene;
		private int _tileSequence;
		private int _initialCameraFramed;
		private int _lastLoggedLoadedTiles = -1;
		private int _lastLoggedRenderTiles = -1;
		private int _lastLoggedEntityCount = -1;
		private int _preparedLoadThreadTiles;
		private int _preparedMainThreadTiles;
		private int _visibilityFrameNumber;
		private int _lastLoggedPreparedLoadThreadTiles = -1;
		private int _lastLoggedPreparedMainThreadTiles = -1;

		/// <summary>
		/// Gets or sets the tileset source type.
		/// </summary>
		public CesiumTilesetDataSource DataSource { get; set; } = CesiumTilesetDataSource.Url;

		/// <summary>
		/// Gets or sets the source URL for a public or local <c>tileset.json</c>.
		/// </summary>
		public string Url { get; set; } = SampleTilesetUrl;

		/// <summary>
		/// Gets or sets the local tileset.json file path used when <see cref="DataSource" /> is <see cref="CesiumTilesetDataSource.LocalFile" />.
		/// </summary>
		public string LocalFilePath { get; set; } = string.Empty;

		/// <summary>
		/// Gets or sets the Cesium ion asset ID to load when <see cref="DataSource" /> is <see cref="CesiumTilesetDataSource.CesiumIon" />.
		/// </summary>
		public long IonAssetId { get; set; }

		/// <summary>
		/// Gets or sets the Cesium ion access token used when loading ion assets.
		/// </summary>
		public string IonAccessToken { get; set; } = string.Empty;

		/// <summary>
		/// Gets or sets the georeference longitude in degrees used when no <see cref="CesiumGeoreferenceComponent" /> is attached.
		/// </summary>
		public double OriginLongitudeDegrees { get; set; } = -75.612094;

		/// <summary>
		/// Gets or sets the georeference latitude in degrees used when no <see cref="CesiumGeoreferenceComponent" /> is attached.
		/// </summary>
		public double OriginLatitudeDegrees { get; set; } = 40.042531;

		/// <summary>
		/// Gets or sets the ellipsoid height in meters used when no <see cref="CesiumGeoreferenceComponent" /> is attached.
		/// </summary>
		public double OriginHeightMeters { get; set; } = 20d;

		/// <summary>
		/// Gets or sets a value indicating whether an attached or referenced georeference component should override the origin properties.
		/// </summary>
		public bool PreferEntityGeoreference { get; set; } = true;

		/// <summary>
		/// Gets or sets an explicit entity that owns the georeference component used by this tileset.
		/// </summary>
		public Entity? GeoreferenceEntity { get; set; }

		/// <summary>
		/// Gets or sets the maximum screen-space error passed to Cesium Native tile selection.
		/// </summary>
		public float MaximumScreenSpaceError { get; set; } = 24f;

		/// <summary>
		/// Gets or sets the maximum number of simultaneous tile loads.
		/// </summary>
		public uint MaximumSimultaneousTileLoads { get; set; } = 8;

		/// <summary>
		/// Gets or sets the maximum number of bytes Cesium Native may keep cached.
		/// </summary>
		public long MaximumCachedBytes { get; set; } = DefaultMaximumCachedBytes;

		/// <summary>
		/// Gets or sets the user agent sent by the Cesium asset accessor.
		/// </summary>
		public string UserAgent { get; set; } = "LibreRallyCesium/0.2";

		/// <summary>
		/// Gets or sets a value indicating whether the runtime may reposition the startup camera to frame the first committed tile.
		/// </summary>
		public bool AutoFrameOnInitialTile { get; set; } = true;

		/// <summary>
		/// Gets or sets a value indicating whether nearby streamed tiles create coarse static collision proxies.
		/// </summary>
		/// <remarks>
		/// This MVP path keeps physics local and cheap by using axis-aligned box colliders derived from tile bounds
		/// instead of attempting full-fidelity globe-scale terrain collision.
		/// </remarks>
		public bool EnablePhysicsProxyColliders { get; set; } = true;

		/// <summary>
		/// Gets or sets a value indicating whether nearby streamed tiles should use mesh-based colliders instead of coarse bounds.
		/// </summary>
		public bool UseAccuratePhysicsMeshes { get; set; } = true;

		/// <summary>
		/// Gets or sets the maximum local-space distance from the active camera at which tile physics proxies stay enabled.
		/// </summary>
		public float PhysicsProxyActivationDistance { get; set; } = 250f;

		/// <summary>
		/// Gets or sets the maximum number of per-tile box colliders retained before collapsing to one merged tile proxy.
		/// </summary>
		public int MaximumPhysicsProxyBoxesPerTile { get; set; } = 8;

		/// <summary>
		/// Gets or sets the friction coefficient applied to Cesium tile physics proxies.
		/// </summary>
		public float PhysicsFrictionCoefficient { get; set; } = 1.1f;

		/// <summary>
		/// Gets or sets the default tyre surface type assigned to Cesium collision meshes.
		/// </summary>
		public SurfaceType PhysicsSurfaceType { get; set; } = SurfaceType.Tarmac;

		/// <summary>
		/// Initializes the Cesium Native runtime and begins loading the configured tileset.
		/// </summary>
		/// <exception cref="InvalidOperationException">
		/// Thrown when the active scene has no camera or when the configured dataset source is invalid.
		/// </exception>
		public override void Start()
		{
			if (!OperatingSystem.IsWindows() && !OperatingSystem.IsMacOS())
			{
				ScriptLog.Warning("Cesium tileset loading is only configured for Windows and macOS runtimes.");
				return;
			}

			var configuration = ResolveRuntimeConfiguration();
			_scene = Entity.Scene;
			_camera = FindPrimaryCamera();
			if (_camera == null)
			{
				throw new InvalidOperationException("CesiumTilesetScript requires a camera in the active scene.");
			}

			_georeferenceComponent = ResolveGeoreferenceComponent();
			RefreshGeoreference(force: true);
			_selfHandle = GCHandle.Alloc(this);
			_ellipsoid = Ellipsoid.Wgs84();
			_asyncSystem = AsyncSystem.Create();
			_assetAccessor = AssetAccessor.Create(string.IsNullOrWhiteSpace(UserAgent) ? "LibreRallyCesium/0.2" : UserAgent.Trim());
			_creditSystem = CreditSystem.Create();
			_externals = TilesetExternals.Create(_asyncSystem, _assetAccessor, _creditSystem);
			_externals.SetRendererResourceCallbacks(RendererCallbacks, (void*)GCHandle.ToIntPtr(_selfHandle));
			var options = TilesetOptions.Create();
			options.MaximumScreenSpaceError = configuration.MaximumScreenSpaceError;
			options.MaximumSimultaneousTileLoads = configuration.MaximumSimultaneousTileLoads;
			options.MaximumCachedBytes = configuration.MaximumCachedBytes;
			options.EnableFrustumCulling = true;
			options.EnableFogCulling = false;
			options.EnableOcclusionCulling = true;
			options.PreloadAncestors = true;
			options.PreloadSiblings = true;
			options.SetLoadErrorCallback(LoadErrorCallback, (void*)GCHandle.ToIntPtr(_selfHandle));

			_tileset = configuration.DataSource switch
			{
				CesiumTilesetDataSource.CesiumIon => Tileset.CreateFromIon(_externals, configuration.IonAssetId, configuration.IonAccessToken, options, null),
				_ => Tileset.CreateFromUrl(_externals, configuration.SourceIdentifier, options),
			};
			_tileset.SetRootTileAvailableCallback(RootTileAvailableCallback, (void*)GCHandle.ToIntPtr(_selfHandle));
			options.Dispose();

			if (_tileset == Tileset.Null)
			{
				var lastError = CesiumNativeApi.GetLastError() ?? "Unknown Cesium Native error.";
				throw new InvalidOperationException($"Failed to create Cesium tileset: {lastError}");
			}

			_initialized = true;
			_disposed = false;
		}

		/// <summary>
		/// Advances Cesium Native background work, updates view-dependent selection, and applies tile removals.
		/// </summary>
		public override void Update()
		{
			if (!_initialized || _disposed)
			{
				return;
			}

			if (_camera == null)
			{
				_camera = FindPrimaryCamera();
				if (_camera == null)
				{
					return;
				}
			}

			RefreshGeoreference(force: false);
			if (_georeference == null)
			{
				return;
			}

			_creditSystem.StartNextFrame();
			_asyncSystem.DispatchMainThreadTasks();
			ReleaseQueuedTiles();
			var viewport = ResolveViewport();
			var cameraEntity = _camera.Entity;
			var cameraRotation = Matrix.RotationQuaternion(cameraEntity.Transform.Rotation);
			var cameraPosition = _georeference.ToEarthCenteredEarthFixed(cameraEntity.Transform.Position);
			var cameraDirection = _georeference.ToEarthCenteredEarthFixedDirection(cameraRotation.Forward);
			var cameraUp = _georeference.ToEarthCenteredEarthFixedDirection(cameraRotation.Up);
			var aspectRatio = ResolveAspectRatio(viewport);
			var verticalFieldOfViewRadians = MathUtil.DegreesToRadians(_camera.VerticalFieldOfView);
			var horizontalFieldOfViewRadians = 2d * Math.Atan(Math.Tan(verticalFieldOfViewRadians * 0.5d) * aspectRatio);
			var viewState = ViewState.CreatePerspective(
			ToCesiumVec3(cameraPosition),
			ToCesiumVec3(cameraDirection),
			ToCesiumVec3(cameraUp),
			new Vec2 { X = viewport.X, Y = viewport.Y },
			horizontalFieldOfViewRadians,
			verticalFieldOfViewRadians,
			_ellipsoid);
			var updateResult = _tileset.UpdateView(&viewState, 1, (float)Game.UpdateTime.Elapsed.TotalSeconds);
			ApplyTileVisibility(updateResult);
			UpdateTilePhysicsProxies(cameraEntity.Transform.Position);
		}

		/// <summary>
		/// Cancels the tileset runtime and releases native and graphics resources.
		/// </summary>
		public override void Cancel()
		{
			DisposeRuntimeState();
			base.Cancel();
		}

		internal CesiumTilesetRuntimeConfiguration ResolveRuntimeConfiguration() =>
		ResolveRuntimeConfiguration(
		DataSource,
		Url,
		LocalFilePath,
		IonAssetId,
		IonAccessToken,
		MaximumScreenSpaceError,
		MaximumSimultaneousTileLoads,
		MaximumCachedBytes);

		internal static CesiumTilesetRuntimeConfiguration ResolveRuntimeConfiguration(
		CesiumTilesetDataSource dataSource,
		string? url,
		string? localFilePath,
		long ionAssetId,
		string? ionAccessToken,
		float maximumScreenSpaceError,
		uint maximumSimultaneousTileLoads,
		long maximumCachedBytes)
		{
			if (float.IsNaN(maximumScreenSpaceError) || maximumScreenSpaceError <= 0f)
			{
				throw new InvalidOperationException("Cesium maximum screen-space error must be greater than zero.");
			}

			if (maximumSimultaneousTileLoads == 0)
			{
				throw new InvalidOperationException("Cesium maximum simultaneous tile loads must be greater than zero.");
			}

			if (maximumCachedBytes <= 0)
			{
				throw new InvalidOperationException("Cesium maximum cached bytes must be greater than zero.");
			}

			return dataSource switch
			{
				CesiumTilesetDataSource.CesiumIon when ionAssetId <= 0 => throw new InvalidOperationException("Cesium ion tilesets require a positive asset ID."),
				CesiumTilesetDataSource.CesiumIon when string.IsNullOrWhiteSpace(ionAccessToken) => throw new InvalidOperationException("Cesium ion tilesets require an access token."),
				CesiumTilesetDataSource.CesiumIon => new CesiumTilesetRuntimeConfiguration(
				dataSource,
				$"ion://{ionAssetId}",
				ionAssetId,
				ionAccessToken.Trim(),
				maximumScreenSpaceError,
				maximumSimultaneousTileLoads,
				maximumCachedBytes),
				CesiumTilesetDataSource.LocalFile => new CesiumTilesetRuntimeConfiguration(
				dataSource,
				ResolveLocalTilesetUri(localFilePath),
				0,
				string.Empty,
				maximumScreenSpaceError,
				maximumSimultaneousTileLoads,
				maximumCachedBytes),
				_ => new CesiumTilesetRuntimeConfiguration(
				dataSource,
				string.IsNullOrWhiteSpace(url) ? SampleTilesetUrl : url.Trim(),
				0,
				string.Empty,
				maximumScreenSpaceError,
				maximumSimultaneousTileLoads,
				maximumCachedBytes),
			};
		}

		private static string ResolveLocalTilesetUri(string? localFilePath)
		{
			if (string.IsNullOrWhiteSpace(localFilePath))
			{
				throw new InvalidOperationException("Cesium local-file tilesets require a tileset.json path.");
			}

			var absolutePath = Path.GetFullPath(localFilePath);
			return new Uri(absolutePath).AbsoluteUri;
		}

		private void RefreshGeoreference(bool force)
		{
			if (_georeferenceComponent != null)
			{
				if (force || _appliedGeoreferenceRevision != _georeferenceComponent.Revision || _georeference == null)
				{
					_georeference = _georeferenceComponent.ResolveGeoreference();
					_appliedGeoreferenceRevision = _georeferenceComponent.Revision;
				}

				return;
			}

			if (force || _georeference == null)
			{
				_georeference = new CesiumLocalGeoreference(OriginLongitudeDegrees, OriginLatitudeDegrees, OriginHeightMeters);
			}
		}

		private float ResolveAspectRatio(Vector2 viewport)
		{
			if (_camera != null && _camera.UseCustomAspectRatio && _camera.AspectRatio > 0f)
			{
				return _camera.AspectRatio;
			}

			return viewport.X / Math.Max(1f, viewport.Y);
		}

		private Vector2 ResolveViewport()
		{
			var graphicsDevice = ((Game)Game).GraphicsDevice;
			var width = Math.Max(1, graphicsDevice.Presenter.BackBuffer.Width);
			var height = Math.Max(1, graphicsDevice.Presenter.BackBuffer.Height);
			return new Vector2(width, height);
		}

		private CameraComponent? FindPrimaryCamera()
		{
			var scene = Entity.Scene;
			if (scene == null)
			{
				return null;
			}

			foreach (var entity in scene.Entities)
			{
				var camera = FindCameraRecursive(entity);
				if (camera != null)
				{
					return camera;
				}
			}

			return null;
		}

		private CesiumGeoreferenceComponent? ResolveGeoreferenceComponent()
		{
			if (!PreferEntityGeoreference)
			{
				return null;
			}

			if (GeoreferenceEntity != null)
			{
				return GeoreferenceEntity.Get<CesiumGeoreferenceComponent>();
			}

			var current = Entity;
			while (current != null)
			{
				var georeference = current.Get<CesiumGeoreferenceComponent>();
				if (georeference != null)
				{
					return georeference;
				}

				current = current.Transform.Parent?.Entity;
			}

			if (Entity.Scene == null)
			{
				return null;
			}

			foreach (var rootEntity in Entity.Scene.Entities)
			{
				var georeference = FindGeoreferenceRecursive(rootEntity);
				if (georeference != null)
				{
					return georeference;
				}
			}

			return null;
		}

		private static CesiumGeoreferenceComponent? FindGeoreferenceRecursive(Entity entity)
		{
			var georeference = entity.Get<CesiumGeoreferenceComponent>();
			if (georeference != null)
			{
				return georeference;
			}

			foreach (var child in entity.GetChildren())
			{
				var childGeoreference = FindGeoreferenceRecursive(child);
				if (childGeoreference != null)
				{
					return childGeoreference;
				}
			}

			return null;
		}

		private static CameraComponent? FindCameraRecursive(Entity entity)
		{
			var camera = entity.Get<CameraComponent>();
			if (camera != null)
			{
				return camera;
			}

			foreach (var child in entity.Transform.Children)
			{
				var childCamera = FindCameraRecursive(child.Entity);
				if (childCamera != null)
				{
					return childCamera;
				}
			}

			return null;
		}

		private void QueueTileRelease(nint handlePointer)
		{
			if (handlePointer == 0)
			{
				return;
			}

			lock (_entityGate)
			{
				if (_queuedTileReleases.Add(handlePointer))
				{
					_releaseQueue.Enqueue(handlePointer);
				}
			}
		}

		private void ReleaseQueuedTiles()
		{
			while (true)
			{
				nint handlePointer;
				lock (_entityGate)
				{
					if (_releaseQueue.Count == 0)
					{
						break;
					}

					handlePointer = _releaseQueue.Dequeue();
					_queuedTileReleases.Remove(handlePointer);
				}

				ReleaseTile(handlePointer);
			}
		}

		private void ReleaseTile(nint handlePointer)
		{
			if (_tileEntities.Remove(handlePointer, out var tileSceneNode))
			{
				RemovePhysicsProxy(tileSceneNode);
				if (tileSceneNode.Entity.Transform.Parent != null)
				{
					tileSceneNode.Entity.Transform.Parent = null;
				}

				var scene = tileSceneNode.Entity.Scene ?? _scene;
				if (scene != null && tileSceneNode.Entity.Scene == scene)
				{
					scene.Entities.Remove(tileSceneNode.Entity);
				}

				foreach (var ownedResource in tileSceneNode.OwnedResources)
				{
					ownedResource.Dispose();
				}
			}

			FreePreparedTileHandle(handlePointer);
		}

		private void DisposeRuntimeState()
		{
			if (_disposed)
			{
				return;
			}

			_disposed = true;
			_initialized = false;

			if (_externals != TilesetExternals.Null)
			{
				_externals.ClearRendererResourceCallbacks();
			}

			if (_tileset != Tileset.Null)
			{
				_tileset.Dispose();
				_tileset = Tileset.Null;
			}

			ReleaseQueuedTiles();
			ClearTileResources();
			DisposeMaterials();

			if (_externals != TilesetExternals.Null)
			{
				_externals.Dispose();
				_externals = TilesetExternals.Null;
			}

			if (_creditSystem != CreditSystem.Null)
			{
				_creditSystem.Dispose();
				_creditSystem = CreditSystem.Null;
			}

			if (_assetAccessor != AssetAccessor.Null)
			{
				_assetAccessor.Dispose();
				_assetAccessor = AssetAccessor.Null;
			}

			if (_asyncSystem != AsyncSystem.Null)
			{
				_asyncSystem.Dispose();
				_asyncSystem = AsyncSystem.Null;
			}

			if (_ellipsoid != Ellipsoid.Null)
			{
				_ellipsoid.Dispose();
				_ellipsoid = Ellipsoid.Null;
			}

			if (_selfHandle.IsAllocated)
			{
				_selfHandle.Free();
			}
		}

		private void ClearTileResources()
		{
			foreach (var entry in _tileEntities)
			{
				RemovePhysicsProxy(entry.Value);
				if (entry.Value.Entity.Transform.Parent != null)
				{
					entry.Value.Entity.Transform.Parent = null;
				}

				foreach (var ownedResource in entry.Value.OwnedResources)
				{
					ownedResource.Dispose();
				}

				FreePreparedTileHandle(entry.Key);
			}

			_tileEntities.Clear();
			_tilesToRenderThisFrame.Clear();
			_releaseQueue.Clear();
			_queuedTileReleases.Clear();
		}

		private void DisposeMaterials()
		{
			_materialCache.Clear();
		}

		private void UpdateTilePhysicsProxies(Vector3 cameraPosition)
		{
			lock (_entityGate)
			{
				foreach (var tileSceneNode in _tileEntities.Values)
				{
					if (!tileSceneNode.IsVisible)
					{
						RemovePhysicsProxy(tileSceneNode);
						continue;
					}

					var shouldEnable = EnablePhysicsProxyColliders
					&& ShouldEnablePhysicsProxy(cameraPosition, tileSceneNode.TileBounds, PhysicsProxyActivationDistance);

					if (shouldEnable)
					{
						EnsurePhysicsProxy(tileSceneNode);
					}
					else
					{
						RemovePhysicsProxy(tileSceneNode);
					}
				}
			}
		}

		private void ApplyTileVisibility(ViewUpdateResult updateResult)
		{
			lock (_entityGate)
			{
				_visibilityFrameNumber++;
				_tilesToRenderThisFrame.Clear();
				var currentVisibilityFrame = _visibilityFrameNumber;
				var cameraPosition = _camera?.Entity.Transform.Position ?? Vector3.Zero;
				var matchedEntities = 0;
				var missingEntities = 0;
				nint firstMissingHandle = 0;
				CesiumTileSceneNode? nearestMatchedTile = null;
				var nearestMatchedTileDistance = float.MaxValue;
				for (var index = 0; index < updateResult.TilesToRenderCount; index++)
				{
					var tile = updateResult.GetTileToRender(index);
					var handle = (nint)tile.RenderResources;
					if (handle != 0)
					{
						_tilesToRenderThisFrame.Add(handle);
						if (_tileEntities.TryGetValue(handle, out var tileSceneNode))
						{
							tileSceneNode.LastSelectedVisibilityFrame = currentVisibilityFrame;
							matchedEntities++;
							var center = (tileSceneNode.TileBounds.Minimum + tileSceneNode.TileBounds.Maximum) * 0.5f;
							var distance = Vector3.Distance(cameraPosition, center);
							if (nearestMatchedTile == null || distance < nearestMatchedTileDistance)
							{
								nearestMatchedTile = tileSceneNode;
								nearestMatchedTileDistance = distance;
							}
						}
						else
						{
							missingEntities++;
							if (firstMissingHandle == 0)
							{
								firstMissingHandle = handle;
							}
						}
					}
				}

				var loadedTiles = _tileset.NumberOfTilesLoaded;
				var shouldLog = Interlocked.Increment(ref _viewDiagnosticsCount) <= 20
					|| loadedTiles != _lastLoggedLoadedTiles
					|| updateResult.TilesToRenderCount != _lastLoggedRenderTiles
					|| _tileEntities.Count != _lastLoggedEntityCount
					|| _preparedLoadThreadTiles != _lastLoggedPreparedLoadThreadTiles
					|| _preparedMainThreadTiles != _lastLoggedPreparedMainThreadTiles;
				if (shouldLog)
				{
					_lastLoggedLoadedTiles = loadedTiles;
					_lastLoggedRenderTiles = updateResult.TilesToRenderCount;
					_lastLoggedEntityCount = _tileEntities.Count;
					_lastLoggedPreparedLoadThreadTiles = _preparedLoadThreadTiles;
					_lastLoggedPreparedMainThreadTiles = _preparedMainThreadTiles;
					var nearestMatchedBounds = "none";
					if (nearestMatchedTile != null)
					{
						var center = (nearestMatchedTile.TileBounds.Minimum + nearestMatchedTile.TileBounds.Maximum) * 0.5f;
						nearestMatchedBounds =
							$"center={center} distance={nearestMatchedTileDistance:0.##} min={nearestMatchedTile.TileBounds.Minimum} max={nearestMatchedTile.TileBounds.Maximum}";
					}
					ScriptLog.Info(
						$"[Cesium View] loaded={loadedTiles} render={updateResult.TilesToRenderCount} " +
						$"entities={_tileEntities.Count} preparedLoad={_preparedLoadThreadTiles} preparedMain={_preparedMainThreadTiles} " +
						$"matched={matchedEntities} missing={missingEntities} totalBytes={_tileset.TotalDataBytes} " +
						$"firstMissingHandle=0x{firstMissingHandle.ToString("X")} nearestMatchedBounds={nearestMatchedBounds}");
				}

				TryRecoverCameraOnDistantVisibleTile(nearestMatchedTile, nearestMatchedTileDistance);

				foreach (var entry in _tileEntities)
				{
					var isSelected = _tilesToRenderThisFrame.Contains(entry.Key);
					SetTileVisibility(entry.Value, isSelected || ShouldKeepTileVisible(entry.Value, currentVisibilityFrame));
				}
			}
		}

		private static bool ShouldKeepTileVisible(CesiumTileSceneNode tileSceneNode, int currentVisibilityFrame)
		{
			return currentVisibilityFrame - tileSceneNode.LastSelectedVisibilityFrame <= TileVisibilityGraceFrames;
		}

		private static void SetTileVisibility(CesiumTileSceneNode tileSceneNode, bool isVisible)
		{
			if (tileSceneNode.IsVisible == isVisible)
			{
				return;
			}

			foreach (var primitive in tileSceneNode.Primitives)
			{
				primitive.Model.Enabled = isVisible;
			}

			tileSceneNode.IsVisible = isVisible;
		}

		private void EnsurePhysicsProxy(CesiumTileSceneNode tileSceneNode)
		{
			if (tileSceneNode.PhysicsProxyEnabled)
			{
				return;
			}

			if (UseAccuratePhysicsMeshes && tileSceneNode.Primitives.Length > 0)
			{
				foreach (var primitive in tileSceneNode.Primitives)
				{
					if (primitive.PhysicsProxyEnabled)
					{
						continue;
					}

					primitive.Entity.Add(CreatePhysicsMeshComponent(primitive));
					primitive.PhysicsProxyEnabled = true;
				}
			}
			else
			{
				tileSceneNode.Entity.Add(CreatePhysicsProxyComponent(tileSceneNode));
			}
			tileSceneNode.PhysicsProxyEnabled = true;
		}

		private void RemovePhysicsProxy(CesiumTileSceneNode tileSceneNode)
		{
			if (!tileSceneNode.PhysicsProxyEnabled)
			{
				return;
			}

			var staticComponent = tileSceneNode.Entity.Get<StaticComponent>();
			if (staticComponent != null)
			{
				tileSceneNode.Entity.Remove(staticComponent);
			}

			foreach (var primitive in tileSceneNode.Primitives)
			{
				var primitiveStaticComponent = primitive.Entity.Get<StaticComponent>();
				if (primitiveStaticComponent == null)
				{
					continue;
				}

				primitive.Entity.Remove(primitiveStaticComponent);
				primitive.PhysicsProxyEnabled = false;
			}

			tileSceneNode.PhysicsProxyEnabled = false;
		}

		private StaticComponent CreatePhysicsMeshComponent(CesiumPrimitiveSceneNode primitiveSceneNode)
		{
			return new StaticComponent
			{
				FrictionCoefficient = ResolvePhysicsFrictionCoefficient(PhysicsFrictionCoefficient),
				Collider = new MeshCollider
				{
					Model = primitiveSceneNode.Model.Model!,
					Closed = false,
				},
			};
		}

		private StaticComponent CreatePhysicsProxyComponent(CesiumTileSceneNode tileSceneNode)
		{
			var collider = new CompoundCollider();
			foreach (var bounds in tileSceneNode.PhysicsBounds)
			{
				collider.Colliders.Add(new BoxCollider
				{
					Size = ResolvePhysicsProxySize(bounds),
					PositionLocal = (bounds.Minimum + bounds.Maximum) * 0.5f,
				});
			}

			return new StaticComponent
			{
				FrictionCoefficient = ResolvePhysicsFrictionCoefficient(PhysicsFrictionCoefficient),
				Collider = collider,
			};
		}

		private nint PrepareTileGeometry(GltfModel model, Mat4 tileTransform)
		{
			if (_georeference == null || model == GltfModel.Null || _disposed)
			{
				return 0;
			}

			var tileMatrix = CesiumMatrixUtilities.Multiply(
				CesiumMatrixUtilities.CopyMatrix(tileTransform),
				CesiumMatrixUtilities.YUpToZUp());
			var primitives = new List<CesiumPreparedPrimitive>();
			var sceneIndex = model.DefaultScene >= 0 ? model.DefaultScene : 0;
			var rootNodeCount = model.SceneGetNodeCount(sceneIndex);
			for (var i = 0; i < rootNodeCount; i++)
			{
				var nodeIndex = model.SceneGetNode(sceneIndex, i);
				AppendNodeGeometry(model, nodeIndex, tileMatrix, primitives);
			}

			if (primitives.Count == 0)
			{
				return 0;
			}

			Interlocked.Increment(ref _preparedLoadThreadTiles);
			var payload = new CesiumPreparedTile(
			Interlocked.Increment(ref _tileSequence),
			[.. primitives]);
			return GCHandle.ToIntPtr(GCHandle.Alloc(payload));
		}

		private void AppendNodeGeometry(
		GltfModel model,
		int nodeIndex,
		double[] parentTransform,
		List<CesiumPreparedPrimitive> primitives)
		{
			var nodeTransform = CesiumMatrixUtilities.ReadNodeTransform(model, nodeIndex);
			var worldTransform = CesiumMatrixUtilities.Multiply(parentTransform, nodeTransform);
			var meshIndex = model.NodeGetMesh(nodeIndex);
			if (meshIndex >= 0)
			{
				AppendMeshGeometry(model, meshIndex, worldTransform, primitives);
			}

			var childCount = model.NodeGetChildrenCount(nodeIndex);
			for (var i = 0; i < childCount; i++)
			{
				AppendNodeGeometry(model, model.NodeGetChild(nodeIndex, i), worldTransform, primitives);
			}
		}

		private void AppendMeshGeometry(
		GltfModel model,
		int meshIndex,
		double[] worldTransform,
		List<CesiumPreparedPrimitive> primitives)
		{
			if (_georeference == null)
			{
				return;
			}

			var primitiveCount = model.MeshGetPrimitiveCount(meshIndex);
			for (var primitiveIndex = 0; primitiveIndex < primitiveCount; primitiveIndex++)
			{
				if (model.PrimitiveGetMode(meshIndex, primitiveIndex) != 4)
				{
					continue;
				}

				var positionAccessorIndex = model.PrimitiveFindAttributeAccessorIndex(meshIndex, primitiveIndex, "POSITION");
				if (positionAccessorIndex < 0)
				{
					continue;
				}

				AccessorData positionAccessor;
				if (model.AccessorGetData(positionAccessorIndex, &positionAccessor) == 0)
				{
					continue;
				}

				var normalAccessorIndex = model.PrimitiveFindAttributeAccessorIndex(meshIndex, primitiveIndex, "NORMAL");
				AccessorData normalAccessor = default;
				var hasNormals = normalAccessorIndex >= 0 && model.AccessorGetData(normalAccessorIndex, &normalAccessor) != 0;
				var preparedMaterial = ResolvePreparedMaterial(model, model.PrimitiveGetMaterialIndex(meshIndex, primitiveIndex));
				AccessorData uvAccessor = default;
				var hasUv = false;
				if (preparedMaterial.BaseColorTexture != null)
				{
					var requestedTexCoordSet = preparedMaterial.BaseColorTexCoordSet;
					var requestedSemantic = ResolveTextureCoordinateSemantic(requestedTexCoordSet);
					var uvAccessorIndex = model.PrimitiveFindAttributeAccessorIndex(
						meshIndex,
						primitiveIndex,
						requestedSemantic);
					hasUv = uvAccessorIndex >= 0 && model.AccessorGetData(uvAccessorIndex, &uvAccessor) != 0;
					if (!hasUv && requestedTexCoordSet > 0)
					{
						uvAccessorIndex = model.PrimitiveFindAttributeAccessorIndex(meshIndex, primitiveIndex, ResolveTextureCoordinateSemantic(0));
						hasUv = uvAccessorIndex >= 0 && model.AccessorGetData(uvAccessorIndex, &uvAccessor) != 0;
						if (hasUv)
						{
							ScriptLog.Warning($"Cesium primitive requested TEXCOORD_{requestedTexCoordSet} for its base-color texture, but only TEXCOORD_0 was available. Falling back to TEXCOORD_0.");
						}
					}
				}
				var vertices = new List<VertexPositionNormalTexture>(positionAccessor.Count);
				for (var vertexIndex = 0; vertexIndex < positionAccessor.Count; vertexIndex++)
				{
					var sourcePosition = CesiumAccessorReader.ReadVector3(positionAccessor, vertexIndex);
					var transformedPosition = CesiumMatrixUtilities.TransformPoint(worldTransform, sourcePosition);
					var localPosition = _georeference.ToLocalPosition(transformedPosition);

					Vector3 localNormal;
					if (hasNormals)
					{
						var sourceNormal = CesiumAccessorReader.ReadVector3(normalAccessor, vertexIndex);
						var transformedNormal = CesiumMatrixUtilities.TransformDirection(worldTransform, sourceNormal);
						localNormal = _georeference.ToLocalDirection(transformedNormal);
						if (localNormal.LengthSquared() > 1e-6f)
						{
							localNormal.Normalize();
						}
						else
						{
							localNormal = Vector3.UnitY;
						}
					}
					else
					{
						localNormal = Vector3.UnitY;
					}

					var localUv = hasUv
					? CesiumAccessorReader.ReadVector2(uvAccessor, vertexIndex)
					: Vector2.Zero;

					vertices.Add(new VertexPositionNormalTexture(
					localPosition,
					localNormal,
					new Vector2(localUv.X, localUv.Y)));
				}

				if (vertices.Count == 0)
				{
					continue;
				}

				var indices = new List<int>();
				var indexAccessorIndex = model.PrimitiveGetIndicesAccessorIndex(meshIndex, primitiveIndex);
				AccessorData indexAccessor = default;
				if (indexAccessorIndex >= 0)
				{
					if (model.AccessorGetData(indexAccessorIndex, &indexAccessor) != 0)
					{
						for (var index = 0; index < indexAccessor.Count; index++)
						{
							indices.Add(CesiumAccessorReader.ReadIndex(indexAccessor, index));
						}
					}
				}
				else
				{
					for (var vertexIndex = 0; vertexIndex < vertices.Count; vertexIndex++)
					{
						indices.Add(vertexIndex);
					}
				}

				if (indices.Count == 0)
				{
					continue;
				}

				var expandedVertices = ExpandTriangleVertices(vertices, indices);
				if (expandedVertices.Length == 0)
				{
					continue;
				}

				if (!hasNormals)
				{
					PopulateTriangleNormals(expandedVertices);
				}

				primitives.Add(new CesiumPreparedPrimitive(
				expandedVertices,
				CreateSequentialIndices(expandedVertices.Length),
				ComputeBoundingBox(expandedVertices),
				preparedMaterial,
				!hasNormals));
			}
		}

		private static CesiumPreparedMaterial ResolvePreparedMaterial(GltfModel model, int materialIndex)
		{
			if (materialIndex < 0)
			{
				return CesiumPreparedMaterial.Default;
			}

			MaterialData materialData = default;
			if (model.MaterialGetData(materialIndex, &materialData) == 0)
			{
				return CesiumPreparedMaterial.Default;
			}

			return new CesiumPreparedMaterial(
				new CesiumMaterialKey(
					(float)materialData.BaseColorFactor[0],
				(float)materialData.BaseColorFactor[1],
				(float)materialData.BaseColorFactor[2],
				(float)materialData.BaseColorFactor[3],
				(float)materialData.MetallicFactor,
				(float)materialData.RoughnessFactor),
			TryResolveBaseColorTexture(model, materialData.BaseColorTexture),
			ResolveTextureCoordinateSet(materialData.BaseColorTexture),
			materialData.DoubleSided != 0);
		}

		private static int ResolveTextureCoordinateSet(TextureInfo textureInfo) =>
			Math.Max(0, textureInfo.TexCoord);

		internal static string ResolveTextureCoordinateSemantic(int texCoordSet) =>
			$"TEXCOORD_{Math.Max(0, texCoordSet)}";

		private static CesiumPreparedTexture? TryResolveBaseColorTexture(GltfModel model, TextureInfo textureInfo)
		{
			if (textureInfo.TextureIndex < 0)
			{
				return null;
			}

			var imageIndex = model.TextureGetSource(textureInfo.TextureIndex);
			if (imageIndex < 0)
			{
				return null;
			}

			ImageData imageData = default;
			if (model.ImageGetData(imageIndex, &imageData) == 0)
			{
				return null;
			}

			var preparedTexture = CesiumTextureDecoder.TryDecode(imageData, forceOpaqueAlpha: true);
			if (preparedTexture == null)
			{
				return null;
			}

			var samplerState = ResolveSamplerState(model, textureInfo.TextureIndex);

			return preparedTexture.Value with
			{
				AddressModeU = samplerState.AddressModeU,
				AddressModeV = samplerState.AddressModeV,
				Filter = samplerState.Filter,
			};
		}

		private static CesiumTextureSamplerState ResolveSamplerState(GltfModel model, int textureIndex)
		{
			var samplerIndex = model.TextureGetSampler(textureIndex);
			if (samplerIndex < 0)
			{
				return CesiumTextureSamplerState.Default;
			}

			SamplerData samplerData = default;
			if (model.SamplerGetData(samplerIndex, &samplerData) == 0)
			{
				return CesiumTextureSamplerState.Default;
			}

			return new CesiumTextureSamplerState(
				MapTextureAddressMode(samplerData.WrapS),
				MapTextureAddressMode(samplerData.WrapT),
				MapTextureFilter(samplerData.MinFilter, samplerData.MagFilter));
		}

		private static TextureAddressMode MapTextureAddressMode(int wrapMode) => wrapMode switch
		{
			33071 => TextureAddressMode.Clamp,
			33648 => TextureAddressMode.Mirror,
			10497 => TextureAddressMode.Wrap,
			_ => TextureAddressMode.Wrap,
		};

		private static TextureFilter MapTextureFilter(int minFilter, int magFilter)
		{
			var usesNearest = minFilter is 9728 or 9984 or 9986 || magFilter == 9728;
			var usesMipmaps = minFilter is 9984 or 9985 or 9986 or 9987;
			var usesLinear = minFilter is 9729 or 9985 or 9987 || magFilter == 9729 || minFilter == 0 || magFilter == 0;

			if (usesNearest && !usesLinear)
			{
				return usesMipmaps ? TextureFilter.MinMagPointMipLinear : TextureFilter.Point;
			}

			if (usesMipmaps)
			{
				return TextureFilter.MinMagLinearMipPoint;
			}

			return TextureFilter.Linear;
		}

		private CesiumTileSceneNode CreateTileEntity(CesiumPreparedTile preparedTile)
		{
			var graphicsDevice = ((Game)Game).GraphicsDevice;
			var ownedResources = new List<IDisposable>();
			var primitiveSceneNodes = new CesiumPrimitiveSceneNode[preparedTile.Primitives.Length];
			var primitiveBounds = new BoundingBox[preparedTile.Primitives.Length];
			var entity = new Entity($"CesiumTile_{preparedTile.TileId}");
			entity.Add(new TrackSurfaceComponent
			{
				SurfaceType = PhysicsSurfaceType,
				SurfaceFrictionCoefficient = PhysicsFrictionCoefficient,
			});
			for (var primitiveIndex = 0; primitiveIndex < preparedTile.Primitives.Length; primitiveIndex++)
			{
				var primitive = preparedTile.Primitives[primitiveIndex];
				var vertexBuffer = Stride.Graphics.Buffer.Vertex.New(graphicsDevice, primitive.Vertices);
				var indexBuffer = Stride.Graphics.Buffer.Index.New(graphicsDevice, primitive.Indices);
				ownedResources.Add(vertexBuffer);
				ownedResources.Add(indexBuffer);
				primitiveBounds[primitiveIndex] = primitive.BoundingBox;
				var mesh = new Mesh
				{
					BoundingBox = primitive.BoundingBox,
					Draw = new MeshDraw
					{
						PrimitiveType = PrimitiveType.TriangleList,
						VertexBuffers =
				[
				new VertexBufferBinding(vertexBuffer, VertexPositionNormalTexture.Layout, primitive.Vertices.Length),
],
						IndexBuffer = new IndexBufferBinding(indexBuffer, true, primitive.Indices.Length),
						DrawCount = primitive.Indices.Length,
					},
				};
				var primitiveEntity = new Entity($"CesiumTile_{preparedTile.TileId}_Primitive_{primitiveIndex}");
				var model = new Model
{
	mesh,
	GetOrCreateMaterial(graphicsDevice, primitive.Material, primitive.UseUnlitFallback, ownedResources),
};
				var modelComponent = new ModelComponent
				{
					Model = model,
					Enabled = false,
				};
				primitiveEntity.Add(modelComponent);
				primitiveEntity.Add(new TrackSurfaceComponent
				{
					SurfaceType = PhysicsSurfaceType,
					SurfaceFrictionCoefficient = PhysicsFrictionCoefficient,
				});
				entity.AddChild(primitiveEntity);
				primitiveSceneNodes[primitiveIndex] = new CesiumPrimitiveSceneNode(primitiveEntity, modelComponent, primitive.BoundingBox);
			}

			var tileSceneNode = new CesiumTileSceneNode(
				entity,
				primitiveSceneNodes,
				[.. ownedResources],
				CombineBounds(primitiveBounds),
				SelectPhysicsProxyBounds(primitiveBounds, MaximumPhysicsProxyBoxesPerTile));
			TryFrameCameraOnInitialTile(tileSceneNode.TileBounds);
			return tileSceneNode;
		}

		private void TryFrameCameraOnInitialTile(BoundingBox tileBounds)
		{
			if (!AutoFrameOnInitialTile || _camera?.Entity == null || Interlocked.Exchange(ref _initialCameraFramed, 1) != 0)
			{
				return;
			}

			FrameCameraOnTileBounds(tileBounds, "initial tile");
		}

		private void TryRecoverCameraOnDistantVisibleTile(CesiumTileSceneNode? nearestMatchedTile, float nearestMatchedTileDistance)
		{
			if (AutoFrameOnInitialTile
				|| nearestMatchedTile == null
				|| nearestMatchedTileDistance <= MaximumVisibleTileDistanceWithoutRecovery
				|| _camera?.Entity == null
				|| Interlocked.Exchange(ref _initialCameraFramed, 1) != 0)
			{
				return;
			}

			PointCameraAtTileBounds(nearestMatchedTile.TileBounds, "nearest visible tile");
		}

		private void FrameCameraOnTileBounds(BoundingBox tileBounds, string reason)
		{
			var size = tileBounds.Maximum - tileBounds.Minimum;
			var center = (tileBounds.Minimum + tileBounds.Maximum) * 0.5f;
			var horizontalExtent = Math.Max(Math.Max(size.X, size.Z) * 0.5f, 25f);
			var verticalExtent = Math.Max(size.Y * 0.5f, 15f);
			var diagonalExtent = Math.Max(size.Length() * 0.5f, 100f);
			var cameraOffset = new Vector3(
				diagonalExtent * 1.35f,
				Math.Max(verticalExtent * 3f, diagonalExtent * 0.45f) + 150f,
				diagonalExtent * 1.55f);
			var cameraPosition = center + cameraOffset;
			var forward = center - cameraPosition;
			_camera.Entity.Transform.Position = cameraPosition;
			_camera.Entity.Transform.Rotation = CesiumRotationUtilities.CreateCameraRotationFromUpAndForward(Vector3.UnitY, forward);
			ScriptLog.Info($"Framed Cesium camera on {reason} at {cameraPosition} looking at {center}.");
		}

		private void PointCameraAtTileBounds(BoundingBox tileBounds, string reason)
		{
			if (_camera?.Entity == null)
			{
				return;
			}

			var center = (tileBounds.Minimum + tileBounds.Maximum) * 0.5f;
			var cameraPosition = _camera.Entity.Transform.Position;
			var forward = center - cameraPosition;
			_camera.Entity.Transform.Rotation = CesiumRotationUtilities.CreateCameraRotationFromUpAndForward(Vector3.UnitY, forward);
			ScriptLog.Info($"Pointed Cesium camera at {reason} from {cameraPosition} toward {center}.");
		}

		private Material GetOrCreateMaterial(GraphicsDevice graphicsDevice, CesiumPreparedMaterial preparedMaterial, bool useUnlitFallback, List<IDisposable> ownedResources)
		{
			if (preparedMaterial.BaseColorTexture != null)
			{
				var texture = CreateStrideTexture(graphicsDevice, preparedMaterial.BaseColorTexture.Value);
				ownedResources.Add(texture);
				return CreateMaterial(graphicsDevice, texture, preparedMaterial.BaseColorTexture, preparedMaterial.MaterialKey, preparedMaterial.DoubleSided, useUnlitFallback);
			}

			var cacheKey = new CesiumMaterialCacheKey(preparedMaterial.MaterialKey, preparedMaterial.DoubleSided, useUnlitFallback);
			lock (_materialCache)
			{
				if (_materialCache.TryGetValue(cacheKey, out var material))
				{
					return material;
				}

				material = CreateMaterial(graphicsDevice, null, null, preparedMaterial.MaterialKey, preparedMaterial.DoubleSided, useUnlitFallback);
				_materialCache[cacheKey] = material;
				return material;
			}
		}

		private static Texture CreateStrideTexture(GraphicsDevice graphicsDevice, CesiumPreparedTexture texture) =>
			Texture.New2D(
				graphicsDevice,
				texture.Width,
				texture.Height,
				PixelFormat.R8G8B8A8_UNorm_SRgb,
				texture.PixelData,
				TextureFlags.ShaderResource,
				GraphicsResourceUsage.Immutable,
				TextureOptions.None);

		private static Material CreateMaterial(GraphicsDevice graphicsDevice, Texture? texture, CesiumPreparedTexture? preparedTexture, CesiumMaterialKey materialKey, bool doubleSided, bool useUnlitFallback)
		{
			IComputeColor diffuse = texture == null
				? new ComputeColor(new Color4(materialKey.R, materialKey.G, materialKey.B, materialKey.A))
				: CreateBaseColorCompute(texture, preparedTexture, materialKey);
			IComputeColor litDiffuse = useUnlitFallback
				? new ComputeColor(Color4.Black)
				: diffuse;

			return Material.New(graphicsDevice, new MaterialDescriptor
			{
				Attributes = new MaterialAttributes
				{
					CullMode = doubleSided ? CullMode.None : CullMode.Back,
					Diffuse = new MaterialDiffuseMapFeature(litDiffuse),
					DiffuseModel = new MaterialDiffuseLambertModelFeature(),
					Emissive = useUnlitFallback ? new MaterialEmissiveMapFeature(diffuse) : null,
				},
			});
		}

		private static IComputeColor CreateBaseColorCompute(Texture texture, CesiumPreparedTexture? preparedTexture, CesiumMaterialKey materialKey)
		{
			var baseTexture = new ComputeTextureColor(texture)
			{
				AddressModeU = preparedTexture?.AddressModeU ?? TextureAddressMode.Wrap,
				AddressModeV = preparedTexture?.AddressModeV ?? TextureAddressMode.Wrap,
				Filtering = preparedTexture?.Filter ?? TextureFilter.Linear,
			};
			var factor = new Color4(materialKey.R, materialKey.G, materialKey.B, materialKey.A);
			return IsApproximatelyWhite(factor)
				? baseTexture
				: new ComputeBinaryColor(baseTexture, new ComputeColor(factor), BinaryOperator.Multiply);
		}

		private static bool IsApproximatelyWhite(Color4 color) =>
			Math.Abs(color.R - 1f) < 1e-4f
			&& Math.Abs(color.G - 1f) < 1e-4f
			&& Math.Abs(color.B - 1f) < 1e-4f
			&& Math.Abs(color.A - 1f) < 1e-4f;

		private static BoundingBox ComputeBoundingBox(IReadOnlyList<VertexPositionNormalTexture> vertices)
		{
			if (vertices.Count == 0)
			{
				return new BoundingBox(Vector3.Zero, Vector3.Zero);
			}

			var min = vertices[0].Position;
			var max = vertices[0].Position;
			for (var i = 1; i < vertices.Count; i++)
			{
				min = Vector3.Min(min, vertices[i].Position);
				max = Vector3.Max(max, vertices[i].Position);
			}

			return new BoundingBox(min, max);
		}

		internal static VertexPositionNormalTexture[] ExpandTriangleVertices(
			IReadOnlyList<VertexPositionNormalTexture> vertices,
			IReadOnlyList<int> indices)
		{
			if (vertices.Count == 0 || indices.Count < 3)
			{
				return [];
			}

			var expanded = new List<VertexPositionNormalTexture>(indices.Count - (indices.Count % 3));
			for (var index = 0; index + 2 < indices.Count; index += 3)
			{
				var index0 = indices[index];
				var index1 = indices[index + 1];
				var index2 = indices[index + 2];
				if ((uint)index0 >= (uint)vertices.Count
					|| (uint)index1 >= (uint)vertices.Count
					|| (uint)index2 >= (uint)vertices.Count)
				{
					continue;
				}

				expanded.Add(vertices[index0]);
				expanded.Add(vertices[index2]);
				expanded.Add(vertices[index1]);
			}

			return [.. expanded];
		}

		internal static void PopulateTriangleNormals(VertexPositionNormalTexture[] vertices)
		{
			for (var index = 0; index + 2 < vertices.Length; index += 3)
			{
				var position0 = vertices[index].Position;
				var position1 = vertices[index + 1].Position;
				var position2 = vertices[index + 2].Position;
				var edge1 = position1 - position0;
				var edge2 = position2 - position0;
				var normal = Vector3.Cross(edge2, edge1);
				if (normal.LengthSquared() <= 1e-12f)
				{
					normal = Vector3.UnitY;
				}
				else
				{
					normal.Normalize();
				}

				vertices[index] = new VertexPositionNormalTexture(position0, normal, vertices[index].TextureCoordinate);
				vertices[index + 1] = new VertexPositionNormalTexture(position1, normal, vertices[index + 1].TextureCoordinate);
				vertices[index + 2] = new VertexPositionNormalTexture(position2, normal, vertices[index + 2].TextureCoordinate);
			}
		}

		internal static int[] CreateSequentialIndices(int count)
		{
			var indices = new int[count];
			for (var index = 0; index < count; index++)
			{
				indices[index] = index;
			}

			return indices;
		}

		internal static BoundingBox CombineBounds(IReadOnlyList<BoundingBox> bounds)
		{
			if (bounds.Count == 0)
			{
				return new BoundingBox(Vector3.Zero, Vector3.Zero);
			}

			var min = bounds[0].Minimum;
			var max = bounds[0].Maximum;
			for (var i = 1; i < bounds.Count; i++)
			{
				min = Vector3.Min(min, bounds[i].Minimum);
				max = Vector3.Max(max, bounds[i].Maximum);
			}

			return new BoundingBox(min, max);
		}

		internal static BoundingBox[] SelectPhysicsProxyBounds(IReadOnlyList<BoundingBox> primitiveBounds, int maximumBoxes)
		{
			if (primitiveBounds.Count == 0)
			{
				return [];
			}

			if (maximumBoxes <= 0 || primitiveBounds.Count > maximumBoxes)
			{
				return [CombineBounds(primitiveBounds)];
			}

			var selectedBounds = new BoundingBox[primitiveBounds.Count];
			for (var i = 0; i < primitiveBounds.Count; i++)
			{
				selectedBounds[i] = primitiveBounds[i];
			}

			return selectedBounds;
		}

		internal static bool ShouldEnablePhysicsProxy(Vector3 cameraPosition, BoundingBox tileBounds, float activationDistance)
		{
			if (float.IsNaN(activationDistance) || float.IsInfinity(activationDistance) || activationDistance <= 0f)
			{
				return false;
			}

			var closestPoint = Vector3.Clamp(cameraPosition, tileBounds.Minimum, tileBounds.Maximum);
			return Vector3.DistanceSquared(cameraPosition, closestPoint) <= activationDistance * activationDistance;
		}

		private static Vector3 ResolvePhysicsProxySize(BoundingBox bounds)
		{
			var size = bounds.Maximum - bounds.Minimum;
			size.X = MathF.Max(size.X, MinimumPhysicsProxyExtent);
			size.Y = MathF.Max(size.Y, MinimumPhysicsProxyExtent);
			size.Z = MathF.Max(size.Z, MinimumPhysicsProxyExtent);
			return size;
		}

		private static float ResolvePhysicsFrictionCoefficient(float frictionCoefficient)
		{
			if (float.IsNaN(frictionCoefficient) || float.IsInfinity(frictionCoefficient))
			{
				return 0f;
			}

			return Math.Max(0f, frictionCoefficient);
		}

		private static Vec3 ToCesiumVec3(CesiumDouble3 value) =>
		new()
		{
			X = value.X,
			Y = value.Y,
			Z = value.Z,
		};

		private static CesiumTilesetScript? TryResolveInstance(void* userData)
		{
			if (userData == null)
			{
				return null;
			}

			var handle = GCHandle.FromIntPtr((IntPtr)userData);
			return handle.IsAllocated && handle.Target is CesiumTilesetScript script && !script._disposed
			? script
			: null;
		}

		private static void FreePreparedTileHandle(nint handlePointer)
		{
			if (handlePointer == 0)
			{
				return;
			}

			var handle = GCHandle.FromIntPtr(handlePointer);
			if (handle.IsAllocated)
			{
				handle.Free();
			}
		}

		private static void* PrepareInLoadThread(void* userData, GltfModel model, Mat4 transform)
		{
			var script = TryResolveInstance(userData);
			return script == null ? null : (void*)script.PrepareTileGeometry(model, transform);
		}

		private static void* PrepareInMainThread(void* userData, Tile tile, void* pLoadData)
		{
			if (pLoadData == null)
			{
				return null;
			}

			var script = TryResolveInstance(userData);
			if (script == null || script._scene == null)
			{
				return pLoadData;
			}

			var handlePointer = (nint)pLoadData;
			var handle = GCHandle.FromIntPtr(handlePointer);
			if (!handle.IsAllocated || handle.Target is not CesiumPreparedTile preparedTile)
			{
				return pLoadData;
			}

			lock (script._entityGate)
			{
				if (!script._tileEntities.ContainsKey(handlePointer))
				{
					var tileSceneNode = script.CreateTileEntity(preparedTile);
					script.Entity.AddChild(tileSceneNode.Entity);
					script._tileEntities[handlePointer] = tileSceneNode;
					Interlocked.Increment(ref script._preparedMainThreadTiles);
				}
			}

			return pLoadData;
		}

		private static void FreeResources(void* userData, Tile tile, void* pLoadData, void* pMainThreadData)
		{
			var handlePointer = pMainThreadData != null
			? (nint)pMainThreadData
			: (nint)pLoadData;
			var script = TryResolveInstance(userData);
			if (script == null)
			{
				FreePreparedTileHandle(handlePointer);
				return;
			}

			script.QueueTileRelease(handlePointer);
		}

		private static void OnTilesetLoadError(void* userData, byte* message)
		{
			var script = TryResolveInstance(userData);
			if (script == null)
			{
				return;
			}

			var text = message == null ? "Unknown Cesium tileset load error." : Marshal.PtrToStringUTF8((IntPtr)message);
			ScriptLog.Error(text ?? "Unknown Cesium tileset load error.");
		}

		private static void OnRootTileAvailable(void* userData)
		{
			var script = TryResolveInstance(userData);
			if (script == null)
			{
				return;
			}

			ScriptLog.Info($"Cesium root tile became available for {script.ResolveRuntimeConfiguration().SourceIdentifier}.");
		}

		internal readonly record struct CesiumTilesetRuntimeConfiguration(
		CesiumTilesetDataSource DataSource,
		string SourceIdentifier,
		long IonAssetId,
		string IonAccessToken,
		double MaximumScreenSpaceError,
		uint MaximumSimultaneousTileLoads,
		long MaximumCachedBytes);

		private sealed record CesiumPreparedTile(
		int TileId,
		CesiumPreparedPrimitive[] Primitives);

		private sealed record CesiumPreparedPrimitive(
		VertexPositionNormalTexture[] Vertices,
		int[] Indices,
		BoundingBox BoundingBox,
			CesiumPreparedMaterial Material,
			bool UseUnlitFallback);

		private sealed class CesiumTileSceneNode(
		Entity entity,
		CesiumPrimitiveSceneNode[] primitives,
		IDisposable[] ownedResources,
		BoundingBox tileBounds,
		BoundingBox[] physicsBounds)
		{
			public Entity Entity { get; } = entity;
			public CesiumPrimitiveSceneNode[] Primitives { get; } = primitives;
			public IDisposable[] OwnedResources { get; } = ownedResources;
			public BoundingBox TileBounds { get; } = tileBounds;
			public BoundingBox[] PhysicsBounds { get; } = physicsBounds;
			public bool IsVisible { get; set; }
			public bool PhysicsProxyEnabled { get; set; }
			public int LastSelectedVisibilityFrame { get; set; } = int.MinValue / 2;
		}

		private sealed class CesiumPrimitiveSceneNode(Entity entity, ModelComponent model, BoundingBox bounds)
		{
			public Entity Entity { get; } = entity;
			public ModelComponent Model { get; } = model;
			public BoundingBox Bounds { get; } = bounds;
			public bool PhysicsProxyEnabled { get; set; }
		}

		private readonly record struct CesiumMaterialCacheKey(
			CesiumMaterialKey MaterialKey,
			bool DoubleSided,
			bool UseUnlitFallback);

		private readonly record struct CesiumPreparedMaterial(
			CesiumMaterialKey MaterialKey,
			CesiumPreparedTexture? BaseColorTexture,
			int BaseColorTexCoordSet,
			bool DoubleSided)
		{
			public static readonly CesiumPreparedMaterial Default = new(CesiumMaterialKey.Default, null, 0, false);
		}

		internal readonly record struct CesiumPreparedTexture(
			int Width,
			int Height,
			byte[] PixelData,
			TextureAddressMode AddressModeU,
			TextureAddressMode AddressModeV,
			TextureFilter Filter);

		private readonly record struct CesiumTextureSamplerState(
			TextureAddressMode AddressModeU,
			TextureAddressMode AddressModeV,
			TextureFilter Filter)
		{
			public static readonly CesiumTextureSamplerState Default = new(TextureAddressMode.Wrap, TextureAddressMode.Wrap, TextureFilter.Linear);
		}

		private readonly record struct CesiumMaterialKey(
			float R,
			float G,
			float B,
			float A,
			float Metallic,
			float Roughness)
		{
			public static readonly CesiumMaterialKey Default = new(0.82f, 0.84f, 0.87f, 1f, 0f, 0.85f);
		}
	}

	internal static unsafe class CesiumNativeBindings
	{
		[DllImport("CesiumNativeC", CallingConvention = CallingConvention.Cdecl, EntryPoint = "cesium_gltf_node_get_matrix")]
		public static extern int GltfNodeGetMatrix(GltfModel model, int nodeIndex, double* output);

		[DllImport("CesiumNativeC", CallingConvention = CallingConvention.Cdecl, EntryPoint = "cesium_gltf_node_get_translation")]
		public static extern void GltfNodeGetTranslation(GltfModel model, int nodeIndex, double* output);

		[DllImport("CesiumNativeC", CallingConvention = CallingConvention.Cdecl, EntryPoint = "cesium_gltf_node_get_rotation")]
		public static extern void GltfNodeGetRotation(GltfModel model, int nodeIndex, double* output);

		[DllImport("CesiumNativeC", CallingConvention = CallingConvention.Cdecl, EntryPoint = "cesium_gltf_node_get_scale")]
		public static extern void GltfNodeGetScale(GltfModel model, int nodeIndex, double* output);
	}

	internal static unsafe class CesiumMatrixUtilities
	{
		public static double[] Identity() =>
	[
	1d, 0d, 0d, 0d,
0d, 1d, 0d, 0d,
0d, 0d, 1d, 0d,
0d, 0d, 0d, 1d,
];

		public static double[] YUpToZUp() =>
	[
	1d, 0d, 0d, 0d,
0d, 0d, 1d, 0d,
0d, -1d, 0d, 0d,
0d, 0d, 0d, 1d,
];

		public static double[] CopyMatrix(Mat4 matrix)
		{
			var values = new double[16];
			for (var i = 0; i < 16; i++)
			{
				values[i] = matrix.M[i];
			}

			return values;
		}

		public static double[] ReadNodeTransform(GltfModel model, int nodeIndex)
		{
			double* matrix = stackalloc double[16];
			if (CesiumNativeBindings.GltfNodeGetMatrix(model, nodeIndex, matrix) != 0)
			{
				var values = new double[16];
				for (var i = 0; i < 16; i++)
				{
					values[i] = matrix[i];
				}

				return values;
			}

			double* translation = stackalloc double[3];
			translation[0] = 0d;
			translation[1] = 0d;
			translation[2] = 0d;
			double* rotation = stackalloc double[4];
			rotation[0] = 0d;
			rotation[1] = 0d;
			rotation[2] = 0d;
			rotation[3] = 1d;
			double* scale = stackalloc double[3];
			scale[0] = 1d;
			scale[1] = 1d;
			scale[2] = 1d;
			CesiumNativeBindings.GltfNodeGetTranslation(model, nodeIndex, translation);
			CesiumNativeBindings.GltfNodeGetRotation(model, nodeIndex, rotation);
			CesiumNativeBindings.GltfNodeGetScale(model, nodeIndex, scale);
			return ComposeTransform(
			translation[0],
			translation[1],
			translation[2],
			rotation[0],
			rotation[1],
			rotation[2],
			rotation[3],
			scale[0],
			scale[1],
			scale[2]);
		}

		public static double[] ComposeTransform(
		double translationX,
		double translationY,
		double translationZ,
		double rotationX,
		double rotationY,
		double rotationZ,
		double rotationW,
		double scaleX,
		double scaleY,
		double scaleZ)
		{
			var magnitudeSquared = (rotationX * rotationX) + (rotationY * rotationY) + (rotationZ * rotationZ) + (rotationW * rotationW);
			if (magnitudeSquared <= 1e-12d)
			{
				rotationX = 0d;
				rotationY = 0d;
				rotationZ = 0d;
				rotationW = 1d;
			}
			else if (Math.Abs(magnitudeSquared - 1d) > 1e-9d)
			{
				var inverseMagnitude = 1d / Math.Sqrt(magnitudeSquared);
				rotationX *= inverseMagnitude;
				rotationY *= inverseMagnitude;
				rotationZ *= inverseMagnitude;
				rotationW *= inverseMagnitude;
			}

			var xx = rotationX * rotationX;
			var yy = rotationY * rotationY;
			var zz = rotationZ * rotationZ;
			var xy = rotationX * rotationY;
			var xz = rotationX * rotationZ;
			var yz = rotationY * rotationZ;
			var wx = rotationW * rotationX;
			var wy = rotationW * rotationY;
			var wz = rotationW * rotationZ;
			var m00 = 1d - (2d * (yy + zz));
			var m01 = 2d * (xy - wz);
			var m02 = 2d * (xz + wy);
			var m10 = 2d * (xy + wz);
			var m11 = 1d - (2d * (xx + zz));
			var m12 = 2d * (yz - wx);
			var m20 = 2d * (xz - wy);
			var m21 = 2d * (yz + wx);
			var m22 = 1d - (2d * (xx + yy));
			return
			[
			m00 * scaleX, m10 * scaleX, m20 * scaleX, 0d,
m01 * scaleY, m11 * scaleY, m21 * scaleY, 0d,
m02 * scaleZ, m12 * scaleZ, m22 * scaleZ, 0d,
translationX, translationY, translationZ, 1d,
];
		}

		public static double[] Multiply(double[] left, double[] right)
		{
			var result = new double[16];
			for (var column = 0; column < 4; column++)
			{
				for (var row = 0; row < 4; row++)
				{
					double sum = 0d;
					for (var k = 0; k < 4; k++)
					{
						sum += left[(k * 4) + row] * right[(column * 4) + k];
					}

					result[(column * 4) + row] = sum;
				}
			}

			return result;
		}

		public static CesiumDouble3 TransformPoint(double[] matrix, CesiumDouble3 point)
		{
			var x = (matrix[0] * point.X) + (matrix[4] * point.Y) + (matrix[8] * point.Z) + matrix[12];
			var y = (matrix[1] * point.X) + (matrix[5] * point.Y) + (matrix[9] * point.Z) + matrix[13];
			var z = (matrix[2] * point.X) + (matrix[6] * point.Y) + (matrix[10] * point.Z) + matrix[14];
			var w = (matrix[3] * point.X) + (matrix[7] * point.Y) + (matrix[11] * point.Z) + matrix[15];
			if (Math.Abs(w) > 1e-9d && Math.Abs(w - 1d) > 1e-9d)
			{
				var inverseW = 1d / w;
				return new CesiumDouble3(x * inverseW, y * inverseW, z * inverseW);
			}

			return new CesiumDouble3(x, y, z);
		}

		public static CesiumDouble3 TransformDirection(double[] matrix, CesiumDouble3 direction)
		{
			var x = (matrix[0] * direction.X) + (matrix[4] * direction.Y) + (matrix[8] * direction.Z);
			var y = (matrix[1] * direction.X) + (matrix[5] * direction.Y) + (matrix[9] * direction.Z);
			var z = (matrix[2] * direction.X) + (matrix[6] * direction.Y) + (matrix[10] * direction.Z);
			return CesiumDouble3.Normalize(new CesiumDouble3(x, y, z));
		}
	}

	internal static unsafe class CesiumAccessorReader
	{
		private const int ComponentTypeByte = 5120;
		private const int ComponentTypeUnsignedByte = 5121;
		private const int ComponentTypeShort = 5122;
		private const int ComponentTypeUnsignedShort = 5123;
		private const int ComponentTypeUnsignedInt = 5125;
		private const int ComponentTypeFloat = 5126;

		public static CesiumDouble3 ReadVector3(AccessorData accessor, int index)
		{
			if (accessor.NumberOfComponents < 3)
			{
				throw new InvalidOperationException("The glTF accessor does not contain three components.");
			}

			var x = ReadComponent(accessor, index, 0);
			var y = ReadComponent(accessor, index, 1);
			var z = ReadComponent(accessor, index, 2);
			return new CesiumDouble3(x, y, z);
		}

		public static Vector2 ReadVector2(AccessorData accessor, int index)
		{
			if (accessor.NumberOfComponents < 2)
			{
				return Vector2.Zero;
			}

			var x = (float)ReadComponent(accessor, index, 0);
			var y = (float)ReadComponent(accessor, index, 1);

			// glTF 2.0 stores texture coordinates as either:
			// - Floats (ComponentType 5126) already in [0, 1] range
			// - Normalized integers (ComponentType 5121/5123/etc) that need denormalization
			// 
			// However, denormalization should ONLY be applied if the accessor.Normalized flag is true.
			// Since we don't have access to that flag in this structure, we use a heuristic:
			// If the values are already in [0, 1] range (floats or large integers like 65535),
			// don't denormalize. The main culprit: denormalizing float data causes massive corruption.
			//
			// For now: Only denormalize if values are small enough to suggest they're normalized ints.
			// Values > 1 suggest they're already in the [0, 65535] range and need denormalization.
			//
			// Check the first raw components to decide:
			var maxRawComponent = Math.Max(Math.Abs(x), Math.Abs(y));
			var shouldDenormalize = accessor.ComponentType switch
			{
				ComponentTypeFloat => false, // Floats never need denormalization
				ComponentTypeUnsignedByte => maxRawComponent > 1f, // If > 1, it's raw data needing ÷255
				ComponentTypeUnsignedShort => maxRawComponent > 1f, // If > 1, it's raw data needing ÷65535
				ComponentTypeByte => maxRawComponent > 1f,
				ComponentTypeShort => maxRawComponent > 1f,
				_ => false
			};

			if (shouldDenormalize)
			{
				switch (accessor.ComponentType)
				{
					case ComponentTypeUnsignedByte:
						x /= 255f;
						y /= 255f;
						break;
					case ComponentTypeUnsignedShort:
						x /= 65535f;
						y /= 65535f;
						break;
					case ComponentTypeByte:
						// Signed byte normalized range is [-1, 1], denormalize by dividing by 127
						x = x / 127f;
						y = y / 127f;
						break;
					case ComponentTypeShort:
						// Signed short normalized range is [-1, 1], denormalize by dividing by 32767
						x = x / 32767f;
						y = y / 32767f;
						break;
				}
			}

			return new Vector2(x, y);
		}

		public static int ReadIndex(AccessorData accessor, int index)
		{
			return accessor.ComponentType switch
			{
				ComponentTypeUnsignedByte => ReadAccessorPointer<byte>(accessor, index, 0),
				ComponentTypeUnsignedShort => ReadAccessorPointer<ushort>(accessor, index, 0),
				ComponentTypeUnsignedInt => checked((int)ReadAccessorPointer<uint>(accessor, index, 0)),
				ComponentTypeByte => ReadAccessorPointer<sbyte>(accessor, index, 0),
				ComponentTypeShort => ReadAccessorPointer<short>(accessor, index, 0),
				_ => throw new NotSupportedException($"Unsupported glTF index component type {accessor.ComponentType}."),
			};
		}

		private static double ReadComponent(AccessorData accessor, int index, int componentIndex)
		{
			return accessor.ComponentType switch
			{
				ComponentTypeFloat => ReadAccessorPointer<float>(accessor, index, componentIndex),
				ComponentTypeByte => ReadAccessorPointer<sbyte>(accessor, index, componentIndex),
				ComponentTypeUnsignedByte => ReadAccessorPointer<byte>(accessor, index, componentIndex),
				ComponentTypeShort => ReadAccessorPointer<short>(accessor, index, componentIndex),
				ComponentTypeUnsignedShort => ReadAccessorPointer<ushort>(accessor, index, componentIndex),
				ComponentTypeUnsignedInt => ReadAccessorPointer<uint>(accessor, index, componentIndex),
				_ => throw new NotSupportedException($"Unsupported glTF accessor component type {accessor.ComponentType}."),
			};
		}

		private static T ReadAccessorPointer<T>(AccessorData accessor, int index, int componentIndex)
		where T : unmanaged
		{
			var elementStride = accessor.Stride == 0
			? (nuint)(sizeof(T) * Math.Max(1, accessor.NumberOfComponents))
			: accessor.Stride;
			var byteOffset = (nuint)index * elementStride + (nuint)(componentIndex * sizeof(T));
			return *(T*)((byte*)accessor.Data + byteOffset);
		}
	}

	internal static unsafe class CesiumTextureDecoder
	{
		public static CesiumTilesetScript.CesiumPreparedTexture? TryDecode(ImageData imageData, bool forceOpaqueAlpha = false)
		{
			if (imageData.Width <= 0
				|| imageData.Height <= 0
				|| imageData.Channels <= 0
				|| imageData.BytesPerChannel <= 0
				|| imageData.PixelData == null)
			{
				return null;
			}

			var pixelDataSize = checked((int)imageData.PixelDataSize.ToUInt64());
			if (pixelDataSize == 0)
			{
				return null;
			}

			var source = new byte[pixelDataSize];
			Marshal.Copy((IntPtr)imageData.PixelData, source, 0, source.Length);
			var converted = ConvertToRgba8(source, imageData.Channels, imageData.BytesPerChannel);
			if (converted != null && forceOpaqueAlpha)
			{
				ForceOpaqueAlpha(converted);
			}
			return converted == null
				? null
				: new CesiumTilesetScript.CesiumPreparedTexture(
					imageData.Width,
					imageData.Height,
					converted,
					TextureAddressMode.Wrap,
					TextureAddressMode.Wrap,
					TextureFilter.Linear);
		}

		internal static byte[]? ConvertToRgba8(byte[] source, int channels, int bytesPerChannel)
		{
			if (source == null || source.Length == 0 || channels is < 1 or > 4 || bytesPerChannel is < 1 or > 2)
			{
				return null;
			}

			var sourcePixelStride = channels * bytesPerChannel;
			if (source.Length % sourcePixelStride != 0)
			{
				return null;
			}

			var pixelCount = source.Length / sourcePixelStride;
			var converted = new byte[pixelCount * 4];
			for (var pixelIndex = 0; pixelIndex < pixelCount; pixelIndex++)
			{
				var sourceOffset = pixelIndex * sourcePixelStride;
				var targetOffset = pixelIndex * 4;
				var component0 = ReadComponent(source, sourceOffset, bytesPerChannel);
				var component1 = channels >= 3 ? ReadComponent(source, sourceOffset + bytesPerChannel, bytesPerChannel) : component0;
				var component2 = channels >= 3 ? ReadComponent(source, sourceOffset + (2 * bytesPerChannel), bytesPerChannel) : component0;
				var component3 = channels == 2
					? ReadComponent(source, sourceOffset + bytesPerChannel, bytesPerChannel)
					: channels == 4
						? ReadComponent(source, sourceOffset + (3 * bytesPerChannel), bytesPerChannel)
						: (byte)255;

				converted[targetOffset] = component0;
				converted[targetOffset + 1] = component1;
				converted[targetOffset + 2] = component2;
				converted[targetOffset + 3] = component3;
			}

			return converted;
		}

		private static byte ReadComponent(byte[] source, int offset, int bytesPerChannel) =>
			bytesPerChannel == 1
				? source[offset]
				: (byte)(BinaryPrimitives.ReadUInt16LittleEndian(source.AsSpan(offset, 2)) / 257);

		private static void ForceOpaqueAlpha(byte[] pixels)
		{
			for (var offset = 3; offset < pixels.Length; offset += 4)
			{
				pixels[offset] = byte.MaxValue;
			}
		}
	}
}

