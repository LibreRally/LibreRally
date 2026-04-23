using System;

namespace LibreRally.Vehicle.Physics
{
	internal readonly record struct TyreMagicFormulaCoefficients(float B, float C, float D, float E);

	/// <summary>
	/// Pure-slip Magic Formula parameter set with lightweight load/camber evaluation.
	/// This keeps the main tyre update loop allocation-free while allowing the base B/C/E
	/// coefficients to be promoted into an explicit reusable object.
	/// </summary>
	internal sealed class TyreMagicFormulaSet
	{
		public float BaseB { get; set; }
		public float BaseC { get; set; }
		public float BaseE { get; set; }
		public float LoadStiffnessSensitivity { get; set; }
		public float CamberStiffnessSensitivity { get; set; }
		public float CamberCurvatureSensitivity { get; set; }
		public float SurfaceDeformationSensitivity { get; set; }

		public TyreMagicFormulaSet(float baseB, float baseC, float baseE)
		{
			BaseB = baseB;
			BaseC = baseC;
			BaseE = baseE;
		}

		public TyreMagicFormulaCoefficients Evaluate(
			float peakForce,
			float loadRatio,
			float camberAngle,
			float surfaceDeformation,
			float stiffnessScale = 1f)
		{
			var clampedLoadRatio = MathF.Max(loadRatio, 0.1f);
			var camberMagnitude = Math.Clamp(MathF.Abs(camberAngle), 0f, 0.6f);
			var deformation = Math.Clamp(surfaceDeformation, 0f, 1f);
			var b = BaseB
			        * MathF.Pow(clampedLoadRatio, LoadStiffnessSensitivity)
			        * MathF.Max(stiffnessScale, 0.1f)
			        * (1f + camberMagnitude * CamberStiffnessSensitivity)
			        * (1f - deformation * SurfaceDeformationSensitivity);
			var e = BaseE + camberMagnitude * CamberCurvatureSensitivity;
			return new TyreMagicFormulaCoefficients(
				b,
				BaseC,
				MathF.Max(peakForce, 0f),
				e);
		}
	}
}
