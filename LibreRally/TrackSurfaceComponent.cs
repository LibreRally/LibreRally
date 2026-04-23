using LibreRally.Vehicle.Physics;
using Stride.Core;
using Stride.Engine;

namespace LibreRally
{
	/// <summary>
	/// Tags a ground entity with the driving surface type used by the tyre model.
	/// </summary>
	[DataContract]
	[ComponentCategory("LibreRally")]
	public sealed class TrackSurfaceComponent : EntityComponent
	{
		/// <summary>
		/// Surface type sampled by wheel ground probes for this track section.
		/// </summary>
		public SurfaceType SurfaceType { get; set; } = SurfaceType.Tarmac;

		/// <summary>
		/// Override for the peak surface friction coefficient multiplier.
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceFrictionCoefficient { get; set; } = -1f;

		/// <summary>
		/// Override for the slip-stiffness scale applied to tyre force buildup.
		/// Lower values make the surface feel more compliant and less asphalt-like.
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceSlipStiffnessScale { get; set; } = -1f;

		/// <summary>
		/// Override for the relaxation-length scale that delays transient force buildup.
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceRelaxationLengthScale { get; set; } = -1f;

		/// <summary>
		/// Override for the longitudinal slip-tolerance scale.
		/// Higher values move peak traction to a larger slip ratio for loose surfaces.
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceSlipTolerance { get; set; } = -1f;

		/// <summary>
		/// Override for the rolling-resistance coefficient.
		/// Higher values model soft or energy-absorbing surfaces.
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float RollingResistanceCoefficient { get; set; } = -1f;

		/// <summary>
		/// Override for the surface deformation scale.
		/// Higher values model loose surfaces that deform under load and tolerate more slip.
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceDeformationScale { get; set; } = -1f;

		/// <summary>
		/// Override for the roughness/noise factor used for deterministic contact-patch grip variation.
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceNoiseFactor { get; set; } = -1f;

		/// <summary>
		/// Override for surface microtexture (adhesion-dominant fine texture).
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceMicrotexture { get; set; } = -1f;

		/// <summary>
		/// Override for surface macrotexture (coarse drainage and hysteresis texture).
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceMacrotexture { get; set; } = -1f;

		/// <summary>
		/// Override for standing water depth in metres.
		/// Negative values keep the default calibration for <see cref="SurfaceType"/>.
		/// </summary>
		public float SurfaceWaterDepth { get; set; } = -1f;

		/// <summary>
		/// Resolves the effective tyre-interaction properties for this surface section.
		/// </summary>
		/// <returns>The resolved surface properties, including any overrides.</returns>
		public SurfaceProperties ResolveSurfaceProperties()
		{
			var defaults = SurfaceProperties.ForType(SurfaceType);
			return new SurfaceProperties
			{
				FrictionCoefficient = SurfaceFrictionCoefficient >= 0f ? SurfaceFrictionCoefficient : defaults.FrictionCoefficient,
				Microtexture = SurfaceMicrotexture >= 0f ? SurfaceMicrotexture : defaults.Microtexture,
				Macrotexture = SurfaceMacrotexture >= 0f ? SurfaceMacrotexture : defaults.Macrotexture,
				WaterDepth = SurfaceWaterDepth >= 0f ? SurfaceWaterDepth : defaults.WaterDepth,
				RollingResistance = RollingResistanceCoefficient >= 0f ? RollingResistanceCoefficient : defaults.RollingResistance,
				SlipStiffnessScale = SurfaceSlipStiffnessScale >= 0f ? SurfaceSlipStiffnessScale : defaults.SlipStiffnessScale,
				RelaxationLengthScale = SurfaceRelaxationLengthScale >= 0f ? SurfaceRelaxationLengthScale : defaults.RelaxationLengthScale,
				PeakSlipRatioScale = SurfaceSlipTolerance >= 0f ? SurfaceSlipTolerance : defaults.PeakSlipRatioScale,
				DeformationFactor = SurfaceDeformationScale >= 0f ? SurfaceDeformationScale : defaults.DeformationFactor,
				NoiseFactor = SurfaceNoiseFactor >= 0f ? SurfaceNoiseFactor : defaults.NoiseFactor,
			};
		}
	}
}
