using System;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally
{
	/// <summary>
	/// Anchors an entity to an explicit globe location and optional tangent-frame offset.
	/// </summary>
	[DataContract]
	[ComponentCategory("LibreRally")]
	public sealed class CesiumGlobeAnchorComponent : EntityComponent
	{
		private double _longitudeDegrees;
		private double _latitudeDegrees;
		private double _heightMeters;
		private Vector3 _localOffset = Vector3.Zero;
		private Quaternion _rotationOffset = Quaternion.Identity;
		private bool _alignToEastUpNorth = true;
		private int _revision;

		/// <summary>
		/// Gets or sets the anchor longitude in degrees.
		/// </summary>
		public double LongitudeDegrees
		{
			get => _longitudeDegrees;
			set => SetDoubleField(ref _longitudeDegrees, value);
		}

		/// <summary>
		/// Gets or sets the anchor latitude in degrees.
		/// </summary>
		public double LatitudeDegrees
		{
			get => _latitudeDegrees;
			set => SetDoubleField(ref _latitudeDegrees, value);
		}

		/// <summary>
		/// Gets or sets the anchor height in meters above the ellipsoid.
		/// </summary>
		public double HeightMeters
		{
			get => _heightMeters;
			set => SetDoubleField(ref _heightMeters, value);
		}

		/// <summary>
		/// Gets or sets the local tangent-frame offset from the anchor position in east/up/north meters.
		/// </summary>
		public Vector3 LocalOffset
		{
			get => _localOffset;
			set => SetVectorField(ref _localOffset, value);
		}

		/// <summary>
		/// Gets or sets the local rotation offset applied after the east/up/north alignment.
		/// </summary>
		public Quaternion RotationOffset
		{
			get => _rotationOffset;
			set => SetQuaternionField(ref _rotationOffset, value);
		}

		/// <summary>
		/// Gets or sets a value indicating whether the entity should align to the anchor east/up/north frame.
		/// </summary>
		public bool AlignToEastUpNorth
		{
			get => _alignToEastUpNorth;
			set
			{
				if (_alignToEastUpNorth == value)
				{
					return;
				}

				_alignToEastUpNorth = value;
				MarkDirty();
			}
		}

		/// <summary>
		/// Gets the revision used by runtime scripts to detect anchor changes.
		/// </summary>
		public int Revision => _revision;

		/// <summary>
		/// Resolves the anchored local position using the provided georeference.
		/// </summary>
		/// <param name="georeference">The georeference that maps globe coordinates into the current local frame.</param>
		/// <returns>The resolved local position for the anchor.</returns>
		/// <exception cref="ArgumentNullException">
		/// Thrown when <paramref name="georeference" /> is <see langword="null" /> because the anchor cannot be projected without a globe-to-local frame.
		/// </exception>
		public Vector3 ResolveLocalPosition(CesiumGeoreferenceComponent georeference)
		{
			ArgumentNullException.ThrowIfNull(georeference);

			var position = georeference.ToLocalPosition(LongitudeDegrees, LatitudeDegrees, HeightMeters);
			if (LocalOffset == Vector3.Zero)
			{
				return position;
			}

			georeference.ResolveEastUpNorthAxes(LongitudeDegrees, LatitudeDegrees, out var east, out var up, out var north);
			return position + (east * LocalOffset.X) + (up * LocalOffset.Y) + (north * LocalOffset.Z);
		}

		/// <summary>
		/// Resolves the anchored local rotation using the provided georeference.
		/// </summary>
		/// <param name="georeference">The georeference that maps globe coordinates into the current local frame.</param>
		/// <returns>The resolved local rotation for the anchor.</returns>
		/// <exception cref="ArgumentNullException">
		/// Thrown when <paramref name="georeference" /> is <see langword="null" /> because the anchor orientation depends on the active globe-to-local frame.
		/// </exception>
		public Quaternion ResolveLocalRotation(CesiumGeoreferenceComponent georeference)
		{
			ArgumentNullException.ThrowIfNull(georeference);

			if (!AlignToEastUpNorth)
			{
				return Quaternion.Normalize(RotationOffset);
			}

			georeference.ResolveEastUpNorthAxes(LongitudeDegrees, LatitudeDegrees, out _, out var up, out var north);
			var tangentRotation = CesiumRotationUtilities.CreateRotationFromUpAndForward(up, north);
			return Quaternion.Normalize(tangentRotation * RotationOffset);
		}

		/// <summary>
		/// Applies the anchored transform to the provided entity.
		/// </summary>
		/// <param name="entity">The entity to reposition and rotate.</param>
		/// <param name="georeference">The georeference used to resolve the anchor.</param>
		/// <exception cref="ArgumentNullException">
		/// Thrown when <paramref name="entity" /> or <paramref name="georeference" /> is <see langword="null" /> because both are required to apply the resolved anchor transform.
		/// </exception>
		public void ApplyTo(Entity entity, CesiumGeoreferenceComponent georeference)
		{
			ArgumentNullException.ThrowIfNull(entity);
			ArgumentNullException.ThrowIfNull(georeference);

			entity.Transform.Position = ResolveLocalPosition(georeference);
			entity.Transform.Rotation = ResolveLocalRotation(georeference);
		}

		private void SetDoubleField(ref double field, double value)
		{
			if (Math.Abs(field - value) <= 1e-12d)
			{
				return;
			}

			field = value;
			MarkDirty();
		}

		private void SetVectorField(ref Vector3 field, Vector3 value)
		{
			if ((field - value).LengthSquared() <= 1e-12f)
			{
				return;
			}

			field = value;
			MarkDirty();
		}

		private void SetQuaternionField(ref Quaternion field, Quaternion value)
		{
			var delta = new Vector4(field.X - value.X, field.Y - value.Y, field.Z - value.Z, field.W - value.W);
			if (delta.LengthSquared() <= 1e-12f)
			{
				return;
			}

			field = value;
			MarkDirty();
		}

		private void MarkDirty()
		{
			unchecked
			{
				_revision++;
				if (_revision == int.MinValue)
				{
					_revision = 0;
				}
			}
		}
	}

	/// <summary>
	/// Applies a <see cref="CesiumGlobeAnchorComponent" /> to the owning entity at runtime.
	/// </summary>
	[ComponentCategory("LibreRally")]
	public sealed class CesiumGlobeAnchorScript : SyncScript
	{
		private CesiumGeoreferenceComponent? _georeference;
		private CesiumGlobeAnchorComponent? _anchor;
		private int _appliedGeoreferenceRevision = int.MinValue;
		private int _appliedAnchorRevision = int.MinValue;

		/// <summary>
		/// Gets or sets the entity that owns the georeference component.
		/// </summary>
		public Entity? GeoreferenceEntity { get; set; }

		/// <summary>
		/// Resolves required components and applies the initial anchored transform.
		/// </summary>
		/// <exception cref="InvalidOperationException">
		/// Thrown when the entity is missing a <see cref="CesiumGlobeAnchorComponent" /> or the scene has no <see cref="CesiumGeoreferenceComponent" />.
		/// </exception>
		public override void Start()
		{
			_anchor = Entity.Get<CesiumGlobeAnchorComponent>()
				?? throw new InvalidOperationException("CesiumGlobeAnchorScript requires a CesiumGlobeAnchorComponent on the same entity.");
			_georeference = ResolveGeoreference()
				?? throw new InvalidOperationException("CesiumGlobeAnchorScript could not find a CesiumGeoreferenceComponent in the scene.");
			ApplyAnchor(force: true);
		}

		/// <summary>
		/// Reapplies the anchored transform whenever the anchor or georeference changes.
		/// </summary>
		public override void Update()
		{
			ApplyAnchor(force: false);
		}

		private void ApplyAnchor(bool force)
		{
			if (_anchor == null || _georeference == null)
			{
				return;
			}

			if (!force &&
			    _appliedGeoreferenceRevision == _georeference.Revision &&
			    _appliedAnchorRevision == _anchor.Revision)
			{
				return;
			}

			_anchor.ApplyTo(Entity, _georeference);
			_appliedGeoreferenceRevision = _georeference.Revision;
			_appliedAnchorRevision = _anchor.Revision;
		}

		private CesiumGeoreferenceComponent? ResolveGeoreference()
		{
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
	}

	internal static class CesiumRotationUtilities
	{
		public static Quaternion CreateRotationFromUpAndForward(Vector3 up, Vector3 forward)
		{
			var normalizedUp = SafeNormalize(up, Vector3.UnitY);
			var normalizedForward = SafeNormalize(ProjectOnPlane(forward, normalizedUp), Vector3.UnitZ);
			var upRotation = CreateRotationBetween(Vector3.UnitY, normalizedUp, Vector3.UnitZ);
			var rotatedForward = SafeNormalize(ProjectOnPlane(Vector3.Transform(Vector3.UnitZ, upRotation), normalizedUp), Vector3.UnitZ);
			var forwardRotation = CreateSignedAxisRotation(rotatedForward, normalizedForward, normalizedUp);
			return Quaternion.Normalize(upRotation * forwardRotation);
		}

		public static Quaternion CreateCameraRotationFromUpAndForward(Vector3 up, Vector3 visibleForward)
		{
			var normalizedUp = SafeNormalize(up, Vector3.UnitY);
			if ((normalizedUp - Vector3.UnitY).LengthSquared() > 1e-6f)
			{
				return CreateRotationFromUpAndForward(normalizedUp, -visibleForward);
			}

			var lookDirection = SafeNormalize(visibleForward, -Vector3.UnitZ);
			var horizontal = MathF.Sqrt((lookDirection.X * lookDirection.X) + (lookDirection.Z * lookDirection.Z));
			var yaw = MathF.Atan2(-lookDirection.X, -lookDirection.Z);
			var pitch = MathF.Atan2(lookDirection.Y, horizontal);
			return Quaternion.RotationYawPitchRoll(yaw, pitch, 0f);
		}

		private static Quaternion CreateSignedAxisRotation(Vector3 from, Vector3 to, Vector3 axis)
		{
			var dot = Math.Clamp(Vector3.Dot(from, to), -1f, 1f);
			if (dot >= 1f - 1e-6f)
			{
				return Quaternion.Identity;
			}

			var angle = MathF.Acos(dot);
			var sign = Vector3.Dot(Vector3.Cross(from, to), axis) >= 0f ? 1f : -1f;
			return Quaternion.RotationAxis(axis, angle * sign);
		}

		private static Quaternion CreateRotationBetween(Vector3 from, Vector3 to, Vector3 fallbackAxis)
		{
			var axis = Vector3.Cross(from, to);
			if (axis.LengthSquared() < 1e-6f)
			{
				return Vector3.Dot(from, to) >= 0f
					? Quaternion.Identity
					: Quaternion.RotationAxis(fallbackAxis, MathF.PI);
			}

			var dot = Math.Clamp(Vector3.Dot(from, to), -1f, 1f);
			return Quaternion.RotationAxis(Vector3.Normalize(axis), MathF.Acos(dot));
		}

		private static Vector3 ProjectOnPlane(Vector3 value, Vector3 planeNormal)
		{
			return value - planeNormal * Vector3.Dot(value, planeNormal);
		}

		private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
		{
			var lengthSquared = value.LengthSquared();
			return lengthSquared < 1e-6f
				? fallback
				: value / MathF.Sqrt(lengthSquared);
		}
	}
}
