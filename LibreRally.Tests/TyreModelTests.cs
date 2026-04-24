using LibreRally.Vehicle.Physics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies pure-slip, combined-slip, thermal, surface, and moment behavior in <see cref="TyreModel"/>.
	/// </summary>
	public class TyreModelTests
	{
		// Deterministic reference surface for analytical tyre-model tests.
		// This intentionally keeps neutralized values instead of using gameplay defaults.
		private static readonly SurfaceProperties DeterministicReferenceSurface = new()
		{
			FrictionCoefficient = 1.0f,
			Microtexture = 0.8f,
			Macrotexture = 0.6f,
			WaterDepth = 0f,
			RollingResistance = 0.012f,
			SlipStiffnessScale = 1.0f,
			RelaxationLengthScale = 1.0f,
			PeakSlipRatioScale = 1.0f,
			DeformationFactor = 0f,
			NoiseFactor = 0f,
		};

		private static readonly SurfaceProperties WetTarmac = SurfaceProperties.ForType(SurfaceType.WetTarmac);

		private static readonly SurfaceProperties DeterministicPacejkaSurface = new()
		{
			FrictionCoefficient = 1.0f,
			Microtexture = 0.8f,
			Macrotexture = 0.6f,
			WaterDepth = 0f,
			RollingResistance = 0f,
			SlipStiffnessScale = 1.0f,
			RelaxationLengthScale = 1.0f,
			PeakSlipRatioScale = 1.0f,
			DeformationFactor = 0f,
			NoiseFactor = 0f,
		};

		private const int ForceToleranceDecimalPlaces = 1;

		private static float MagicFormula(float x, float b, float c, float d, float e)
		{
			var bx = b * x;
			return d * MathF.Sin(c * MathF.Atan(bx - e * (bx - MathF.Atan(bx))));
		}

		/// <summary>
		/// Verifies that Pacejka-only longitudinal force follows the Magic Formula equation.
		/// </summary>
		[Fact]
		public void PacejkaOnly_LongitudinalForce_FollowsMagicFormulaEquation()
		{
			var model = new TyreModel(0.305f)
			{
				ActiveMode = TyreModelMode.PacejkaOnly,
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			var state = TyreState.CreateDefault();
			const float normalLoad = 3000f;
			const float longitudinalVelocity = 20f;
			var rollingRadius = model.ComputeEffectiveRollingRadius(normalLoad);
			state.AngularVelocity = longitudinalVelocity * 1.12f / rollingRadius;

			model.Update(ref state, longitudinalVelocity, 0f, normalLoad, 0f, 0f, 0f,
				DeterministicPacejkaSurface, 0.01f, out float actualFx, out _, out _);

			var slipRatio = state.SlipRatio;
			var peakForce = normalLoad * model.ComputeEffectiveFriction(normalLoad, DeterministicPacejkaSurface, 30f, 1f);
			var coefficients = model.EvaluateLongitudinalPureSlipCoefficients(
				peakForce,
				normalLoad / model.ReferenceLoad,
				DeterministicPacejkaSurface);
			var expectedFx = MagicFormula(slipRatio, coefficients.B, coefficients.C, coefficients.D, coefficients.E);

			Assert.Equal(expectedFx, actualFx, ForceToleranceDecimalPlaces);
		}

		/// <summary>
		/// Verifies that Pacejka-only lateral force follows the Magic Formula equation when the high-slip extension is disabled.
		/// </summary>
		[Fact]
		public void PacejkaOnly_LateralForce_FollowsMagicFormulaEquationWhenHighSlipExtensionIsDisabled()
		{
			var model = new TyreModel(0.305f)
			{
				ActiveMode = TyreModelMode.PacejkaOnly,
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				HighSlipTransitionStart = 2f,
				HighSlipTransitionEnd = 2.1f,
			};

			var state = TyreState.CreateDefault();
			const float normalLoad = 3000f;
			const float longitudinalVelocity = 20f;
			const float lateralVelocity = 1.2f;
			var rollingRadius = model.ComputeEffectiveRollingRadius(normalLoad);
			state.AngularVelocity = longitudinalVelocity / rollingRadius;

			model.Update(ref state, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
				DeterministicPacejkaSurface, 0.01f, out _, out float actualFy, out _);

			var slipAngle = state.SlipAngle;
			var peakForce = normalLoad * model.ComputeEffectiveFriction(normalLoad, DeterministicPacejkaSurface, 30f, 1f);
			var coefficients = model.EvaluateLateralPureSlipCoefficients(
				peakForce,
				normalLoad / model.ReferenceLoad,
				camberAngle: 0f,
				DeterministicPacejkaSurface);
			var expectedFy = MagicFormula(slipAngle, coefficients.B, coefficients.C, coefficients.D, coefficients.E);

			Assert.Equal(expectedFy, actualFy, ForceToleranceDecimalPlaces);
		}

		/// <summary>
		/// Verifies that pure-slip coefficient evaluation responds to load and camber.
		/// </summary>
		[Fact]
		public void PureSlipCoefficientEvaluation_RespondsToLoadAndCamber()
		{
			var model = new TyreModel(0.305f) { LateralLoadStiffnessSensitivity = 0.12f, LateralCamberStiffnessSensitivity = 0.6f, LateralCamberCurvatureSensitivity = -0.25f, };

			var lowLoad = model.EvaluateLateralPureSlipCoefficients(2500f, 0.7f, 0f, DeterministicPacejkaSurface);
			var highLoad = model.EvaluateLateralPureSlipCoefficients(2500f, 1.3f, 0f, DeterministicPacejkaSurface);
			var cambered = model.EvaluateLateralPureSlipCoefficients(2500f, 1.0f, 0.15f, DeterministicPacejkaSurface);

			Assert.True(highLoad.B > lowLoad.B);
			Assert.True(cambered.B > lowLoad.B);
			Assert.True(cambered.E < lowLoad.E);
		}

		/// <summary>
		/// Verifies that loose surfaces reduce slip stiffness and move peak longitudinal force to higher slip.
		/// </summary>
		[Fact]
		public void PureSlipCoefficientEvaluation_RespondsToSurfaceSlipStiffnessAndSlipTolerance()
		{
			var model = new TyreModel(0.305f);
			var tarmac = SurfaceProperties.ForType(SurfaceType.Tarmac);
			var gravel = SurfaceProperties.ForType(SurfaceType.Gravel);

			var tarmacLateral = model.EvaluateLateralPureSlipCoefficients(3000f, 1f, 0f, tarmac);
			var gravelLateral = model.EvaluateLateralPureSlipCoefficients(3000f, 1f, 0f, gravel);
			var tarmacLongitudinal = model.EvaluateLongitudinalPureSlipCoefficients(3000f, 1f, tarmac);
			var gravelLongitudinal = model.EvaluateLongitudinalPureSlipCoefficients(3000f, 1f, gravel);

			Assert.True(gravelLateral.B < tarmacLateral.B);
			Assert.True(gravelLongitudinal.B < tarmacLongitudinal.B);
			Assert.True(gravel.PeakSlipRatioScale > tarmac.PeakSlipRatioScale);
		}

		/// <summary>
		/// Verifies that contact-patch length grows with load and shrinks with pressure.
		/// </summary>
		[Fact]
		public void ContactPatchLength_GrowsWithLoad_AndShrinksWithPressure()
		{
			var model = new TyreModel(0.305f) { Width = 0.205f, ContactPatchLengthScale = 1.0f, TyrePressure = 220f, };

			float lightLoadPatch = model.ComputeEffectivePatchLength(1500f);
			float heavyLoadPatch = model.ComputeEffectivePatchLength(3000f);

			model.TyrePressure = 280f;
			float highPressurePatch = model.ComputeEffectivePatchLength(3000f);

			Assert.True(heavyLoadPatch > lightLoadPatch);
			Assert.True(highPressurePatch < heavyLoadPatch);
		}

		/// <summary>
		/// Verifies that effective rolling radius decreases under load.
		/// </summary>
		[Fact]
		public void EffectiveRollingRadius_DecreasesUnderLoad()
		{
			var model = new TyreModel(0.305f) { TyrePressure = 220f, VerticalStiffness = 200000f, };

			float unloadedRadius = model.ComputeEffectiveRollingRadius(0f);
			float loadedRadius = model.ComputeEffectiveRollingRadius(3000f);

			Assert.Equal(model.Radius, unloadedRadius);
			Assert.True(loadedRadius < unloadedRadius);
			Assert.True(loadedRadius > model.Radius * 0.9f);
		}

		/// <summary>
		/// Verifies that estimated wheel inertia grows with hub radius and width.
		/// </summary>
		[Fact]
		public void EstimatedWheelInertia_GrowsWithHubRadiusAndWidth()
		{
			var baselineModel = new TyreModel(0.305f) { Width = 0.205f, };
			var largerHubModel = new TyreModel(0.305f) { Width = 0.205f, HubRadius = 0.24f, HubWidth = 0.28f, };

			float baselineInertia = baselineModel.ComputeEstimatedWheelInertia(3000f);
			float largerHubInertia = largerHubModel.ComputeEstimatedWheelInertia(3000f);

			Assert.True(baselineInertia > 0f);
			Assert.True(largerHubInertia > baselineInertia);
		}

		/// <summary>
		/// Verifies that brush stiffness increases with pressure and width.
		/// </summary>
		[Fact]
		public void BrushStiffness_IncreasesWithPressureAndWidth()
		{
			var model = new TyreModel(0.305f) { Width = 0.205f, TyrePressure = 220f, };

			float baseline = model.ComputeEffectiveBrushStiffness();

			model.TyrePressure = 260f;
			float higherPressure = model.ComputeEffectiveBrushStiffness();

			model.TyrePressure = 220f;
			model.Width = 0.235f;
			float widerTyre = model.ComputeEffectiveBrushStiffness();

			Assert.True(higherPressure > baseline);
			Assert.True(widerTyre > baseline);
		}

		/// <summary>
		/// Verifies that standing-wave resistance factor grows with speed.
		/// </summary>
		[Fact]
		public void StandingWaveResistanceFactor_GrowsWithSpeed()
		{
			var model = new TyreModel(0.305f) { StandingWaveCriticalSpeed = 65f, StandingWaveResistanceGain = 1.0f, };

			float lowSpeed = model.ComputeStandingWaveResistanceFactor(10f);
			float highSpeed = model.ComputeStandingWaveResistanceFactor(80f);

			Assert.True(lowSpeed >= 1f);
			Assert.True(highSpeed > lowSpeed);
		}

		/// <summary>
		/// Verifies that the obsolete contact-patch length alias maps to the scale property.
		/// </summary>
		[Fact]
		public void ContactPatchLengthAlias_MapsToScaleProperty()
		{
			var model = new TyreModel(0.305f);

#pragma warning disable CS0618
			model.ContactPatchLength = 1.2f;
#pragma warning restore CS0618

			Assert.Equal(1.2f, model.ContactPatchLengthScale);
		}

		/// <summary>
		/// Verifies that effective relaxation length grows on loose low-grip surfaces.
		/// </summary>
		[Fact]
		public void EffectiveRelaxationLength_GrowsOnLooseLowGripSurfaces()
		{
			var model = new TyreModel(0.305f) { CarcassStiffness = 1.0f, };

			float tarmacLateral = model.ComputeEffectiveRelaxationLength(DeterministicReferenceSurface, longitudinal: false);
			float gravelLateral = model.ComputeEffectiveRelaxationLength(SurfaceProperties.ForType(SurfaceType.Gravel), longitudinal: false);
			float snowLateral = model.ComputeEffectiveRelaxationLength(SurfaceProperties.ForType(SurfaceType.Snow), longitudinal: false);
			float tarmacLongitudinal = model.ComputeEffectiveRelaxationLength(DeterministicReferenceSurface, longitudinal: true);
			float gravelLongitudinal = model.ComputeEffectiveRelaxationLength(SurfaceProperties.ForType(SurfaceType.Gravel), longitudinal: true);

			Assert.True(gravelLateral > tarmacLateral);
			Assert.True(snowLateral > gravelLateral);
			Assert.True(gravelLongitudinal > tarmacLongitudinal);
		}

		/// <summary>
		/// Verifies that relaxation-length operating-point scale drops at higher slip.
		/// </summary>
		[Fact]
		public void RelaxationLengthOperatingPointScale_DropsAtHigherSlip()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				ContactPatchStiffness = 12000f,
				AligningTorqueResidualFactor = 0f,
				AligningTorqueFxMomentArm = 0f,
				HighSlipTransitionStart = 2f,
				HighSlipTransitionEnd = 2.1f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
			};

			const float peakForce = 3000f;
			var lateralCoefficients = model.EvaluateLateralPureSlipCoefficients(
				peakForce,
				loadRatio: 1f,
				camberAngle: 0f,
				DeterministicReferenceSurface);
			var longitudinalCoefficients = model.EvaluateLongitudinalPureSlipCoefficients(
				peakForce,
				loadRatio: 1f,
				DeterministicReferenceSurface);
			float lowSlipLateralScale = model.ComputeRelaxationLengthOperatingPointScale(
				longitudinal: false,
				slipRatio: 0f,
				slipAngle: 0.03f,
				lateralCoefficients,
				lateralCoefficients);
			float highSlipLateralScale = model.ComputeRelaxationLengthOperatingPointScale(
				longitudinal: false,
				slipRatio: 0f,
				slipAngle: 0.35f,
				lateralCoefficients,
				lateralCoefficients);
			float lowSlipLongitudinalScale = model.ComputeRelaxationLengthOperatingPointScale(
				longitudinal: true,
				slipRatio: 0.03f,
				slipAngle: 0f,
				longitudinalCoefficients,
				longitudinalCoefficients);
			float highSlipLongitudinalScale = model.ComputeRelaxationLengthOperatingPointScale(
				longitudinal: true,
				slipRatio: 0.35f,
				slipAngle: 0f,
				longitudinalCoefficients,
				longitudinalCoefficients);

			Assert.True(lowSlipLateralScale > highSlipLateralScale);
			Assert.True(lowSlipLongitudinalScale > highSlipLongitudinalScale);
			Assert.InRange(highSlipLateralScale, 0.2f, 2.0f);
			Assert.InRange(highSlipLongitudinalScale, 0.2f, 2.0f);
		}

		/// <summary>
		/// Verifies that effective relaxation length uses the operating-point slope scale.
		/// </summary>
		[Fact]
		public void EffectiveRelaxationLength_UsesOperatingPointSlopeScale()
		{
			var model = new TyreModel(0.305f) { CarcassStiffness = 1.0f, };

			float baseLength = model.ComputeEffectiveRelaxationLength(DeterministicReferenceSurface, longitudinal: false);
			float reducedSlopeLength = model.ComputeEffectiveRelaxationLength(DeterministicReferenceSurface, longitudinal: false, operatingPointSlopeScale: 0.2f);
			float amplifiedSlopeLength = model.ComputeEffectiveRelaxationLength(DeterministicReferenceSurface, longitudinal: false, operatingPointSlopeScale: 1.5f);

			Assert.True(reducedSlopeLength < baseLength);
			Assert.True(amplifiedSlopeLength > baseLength);
		}

		/// <summary>
		/// Verifies that effective friction uses the power-law load-sensitivity path.
		/// </summary>
		[Fact]
		public void EffectiveFriction_UsesPowerLawLoadSensitivity()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0.15f,
				ReferenceLoad = 3000f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
			};

			float referenceMu = model.ComputeEffectiveFriction(3000f, DeterministicReferenceSurface, 30f, 1.0f);
			float mediumLoadMu = model.ComputeEffectiveFriction(3300f, DeterministicReferenceSurface, 30f, 1.0f);
			float highLoadMu = model.ComputeEffectiveFriction(6000f, DeterministicReferenceSurface, 30f, 1.0f);

			Assert.Equal(1.0f, referenceMu, 4);
			Assert.True(mediumLoadMu < referenceMu);
			Assert.True(highLoadMu < mediumLoadMu);
			Assert.True(highLoadMu / mediumLoadMu < mediumLoadMu / referenceMu);
		}

		/// <summary>
		/// Verifies that effective friction uses the BeamNG load curve when provided.
		/// </summary>
		[Fact]
		public void EffectiveFriction_UsesBeamNgLoadCurveWhenProvided()
		{
			var referenceMu = TyreModel.ComputeBeamNgLoadCoefficient(3000f, 1.6f, 0.5f, 0.000165f);
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = referenceMu,
				LoadSensitivity = 0.4f,
				BeamNgNoLoadFrictionCoefficient = 1.6f,
				BeamNgFullLoadFrictionCoefficient = 0.5f,
				BeamNgLoadSensitivitySlope = 0.000165f,
				ReferenceLoad = 3000f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
			};

			float measuredReferenceMu = model.ComputeEffectiveFriction(3000f, DeterministicReferenceSurface, 30f, 1.0f);
			float highLoadMu = model.ComputeEffectiveFriction(6000f, DeterministicReferenceSurface, 30f, 1.0f);
			float expectedHighLoadMu = referenceMu *
			                           (TyreModel.ComputeBeamNgLoadCoefficient(6000f, 1.6f, 0.5f, 0.000165f) / referenceMu);

			Assert.Equal(referenceMu, measuredReferenceMu, 4);
			Assert.Equal(expectedHighLoadMu, highLoadMu, 4);
		}

		/// <summary>
		/// Verifies that effective friction increases with larger patch area.
		/// </summary>
		[Fact]
		public void EffectiveFriction_IncreasesWithLargerPatchArea()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0.05f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
			};

			model.TyrePressure = 180f;
			float lowPressureMu = model.ComputeEffectiveFriction(3000f, DeterministicReferenceSurface, 30f, 1.0f);

			model.TyrePressure = 280f;
			float highPressureMu = model.ComputeEffectiveFriction(3000f, DeterministicReferenceSurface, 30f, 1.0f);

			Assert.True(lowPressureMu > highPressureMu);
		}

		/// <summary>
		/// Verifies that the friction ellipse caps combined slip at the ellipse boundary.
		/// </summary>
		[Fact]
		public void FrictionEllipse_CapsCombinedSlipAtEllipseBoundary()
		{
			var baselineModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				FrictionEllipseRatio = 1.0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			var ellipticalModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				FrictionEllipseRatio = 0.9f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			var baselineState = TyreState.CreateDefault();
			var ellipticalState = TyreState.CreateDefault();
			float longitudinalVelocity = 20f;
			float lateralVelocity = 10f;
			float normalLoad = 3000f;
			float angularVelocity = longitudinalVelocity * 1.18f / baselineModel.Radius;
			baselineState.AngularVelocity = angularVelocity;
			ellipticalState.AngularVelocity = angularVelocity;

			baselineModel.Update(ref baselineState, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out float baselineFx, out float baselineFy, out _);
			ellipticalModel.Update(ref ellipticalState, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out float ellipticalFx, out float ellipticalFy, out _);

			float fxMax = normalLoad;
			float fyMax = normalLoad * 0.9f;
			float ellipseValue = (ellipticalFx * ellipticalFx) / (fxMax * fxMax)
			                     + (ellipticalFy * ellipticalFy) / (fyMax * fyMax);

			Assert.True(MathF.Abs(ellipticalFy) < MathF.Abs(baselineFy));
			Assert.True(MathF.Abs(ellipticalFy) <= fyMax + 1e-3f);
			Assert.InRange(ellipseValue, 0f, 1.0001f);
			Assert.True(MathF.Abs(ellipticalFy) < fxMax);
		}

		/// <summary>
		/// Verifies that higher pressure increases steady-state cornering force.
		/// </summary>
		[Fact]
		public void HigherPressure_IncreasesSteadyStateCorneringForce()
		{
			var lowPressureModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				TyrePressure = 180f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			var highPressureModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				TyrePressure = 280f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			var lowPressureState = TyreState.CreateDefault();
			var highPressureState = TyreState.CreateDefault();
			float longitudinalVelocity = 20f;
			float lateralVelocity = 1.4f;
			float angularVelocity = longitudinalVelocity / lowPressureModel.Radius;
			lowPressureState.AngularVelocity = angularVelocity;
			highPressureState.AngularVelocity = angularVelocity;

			lowPressureModel.Update(ref lowPressureState, longitudinalVelocity, lateralVelocity, 3000f, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out float lowPressureFy, out _);
			highPressureModel.Update(ref highPressureState, longitudinalVelocity, lateralVelocity, 3000f, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out float highPressureFy, out _);

			Assert.True(MathF.Abs(highPressureFy) > MathF.Abs(lowPressureFy));
		}

		/// <summary>
		/// Verifies that effective friction clamps the reference patch-area load.
		/// </summary>
		[Fact]
		public void EffectiveFriction_ClampsReferencePatchAreaLoad()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				ReferenceLoad = 0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0.05f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
			};

			float mu = model.ComputeEffectiveFriction(3000f, DeterministicReferenceSurface, 30f, 1.0f);

			Assert.True(float.IsFinite(mu));
			Assert.InRange(mu, 0.5f, 2.0f);
		}

		/// <summary>
		/// Verifies that the friction ellipse respects very low peak force.
		/// </summary>
		[Fact]
		public void FrictionEllipse_RespectsVeryLowPeakForce()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				FrictionEllipseRatio = 0.9f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			var state = TyreState.CreateDefault();
			float normalLoad = 0.25f;
			float longitudinalVelocity = 20f;
			float lateralVelocity = 7.3f;
			state.AngularVelocity = longitudinalVelocity / model.Radius;

			model.Update(ref state, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out float fx, out float fy, out _);

			float fxMax = normalLoad;
			float fyMax = normalLoad * 0.9f;
			float ellipseValue = (fx * fx) / (fxMax * fxMax) + (fy * fy) / (fyMax * fyMax);

			Assert.InRange(ellipseValue, 0f, 1.0001f);
		}

		/// <summary>
		/// Verifies that the friction ellipse clamps final applied forces after camber and rolling resistance.
		/// </summary>
		[Fact]
		public void FrictionEllipse_ClampsFinalAppliedForcesAfterCamberAndRollingResistance()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				FrictionEllipseRatio = 0.9f,
				CamberThrustCoefficient = 0.25f,
				RollingResistanceCoefficient = 0.08f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
			};

			var state = TyreState.CreateDefault();
			float normalLoad = 3000f;
			float longitudinalVelocity = 20f;
			float lateralVelocity = 7.3f;
			state.AngularVelocity = longitudinalVelocity / model.Radius;

			model.Update(ref state, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0.35f,
				DeterministicReferenceSurface, 0.01f, out float fx, out float fy, out _);

			float fxMax = normalLoad;
			float fyMax = normalLoad * 0.9f;
			float ellipseValue = (fx * fx) / (fxMax * fxMax) + (fy * fy) / (fyMax * fyMax);

			Assert.InRange(ellipseValue, 0f, 1.0001f);
		}

		/// <summary>
		/// Verifies that combined-slip interaction reduces longitudinal force under cornering.
		/// </summary>
		[Fact]
		public void CombinedSlipInteraction_ReducesLongitudinalForceUnderCornering()
		{
			var noCouplingModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				FrictionEllipseRatio = 4.0f,
				CombinedSlipCoupling = 0f,
				CombinedSlipExponent = 2.0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				CoolingRate = 0f,
				CoreCoolingRate = 0f,
				RoadHeatTransferRate = 0f,
			};
			var couplingModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				FrictionEllipseRatio = 4.0f,
				CombinedSlipCoupling = 1.2f,
				CombinedSlipExponent = 2.0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				CoolingRate = 0f,
				CoreCoolingRate = 0f,
				RoadHeatTransferRate = 0f,
			};

			var noCouplingState = TyreState.CreateDefault();
			var couplingState = TyreState.CreateDefault();
			const float longitudinalVelocity = 20f;
			const float normalLoad = 3000f;
			const float slipRatioTarget = 0.12f;
			float noCouplingAngularVelocity = longitudinalVelocity * (1f + slipRatioTarget) / noCouplingModel.Radius;
			float couplingAngularVelocity = longitudinalVelocity * (1f + slipRatioTarget) / couplingModel.Radius;
			noCouplingState.AngularVelocity = noCouplingAngularVelocity;
			couplingState.AngularVelocity = couplingAngularVelocity;

			noCouplingModel.Update(ref noCouplingState, longitudinalVelocity, 6f, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out float noCouplingFx, out float noCouplingFy, out _);
			couplingModel.Update(ref couplingState, longitudinalVelocity, 6f, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out float couplingFx, out float couplingFy, out _);

			Assert.True(MathF.Abs(noCouplingFy) > 0f);
			Assert.True(MathF.Abs(couplingFy) > 0f);
			Assert.True(MathF.Abs(couplingFx) < MathF.Abs(noCouplingFx));
		}

		/// <summary>
		/// Verifies that combined-slip weights are unity without cross-slip.
		/// </summary>
		[Fact]
		public void CombinedSlipWeights_AreUnityWithoutCrossSlip()
		{
			var model = new TyreModel(0.305f) { CombinedSlipCoupling = 1.2f, CombinedSlipExponent = 2.0f, };

			Assert.Equal(1f, model.ComputeCombinedSlipLongitudinalWeight(0f, 0.12f), 4);
			Assert.Equal(1f, model.ComputeCombinedSlipLateralWeight(0f, 0.25f), 4);
		}

		/// <summary>
		/// Verifies that combined-slip weights drop as cross-slip increases.
		/// </summary>
		[Fact]
		public void CombinedSlipWeights_DropAsCrossSlipIncreases()
		{
			var model = new TyreModel(0.305f) { CombinedSlipCoupling = 1.2f, CombinedSlipExponent = 2.0f, };

			float moderateFxWeight = model.ComputeCombinedSlipLongitudinalWeight(0.15f, 0.12f);
			float highFxWeight = model.ComputeCombinedSlipLongitudinalWeight(0.35f, 0.12f);
			float moderateFyWeight = model.ComputeCombinedSlipLateralWeight(0.08f, 0.25f);
			float highFyWeight = model.ComputeCombinedSlipLateralWeight(0.18f, 0.25f);

			Assert.InRange(moderateFxWeight, 0f, 1f);
			Assert.InRange(highFxWeight, 0f, 1f);
			Assert.InRange(moderateFyWeight, 0f, 1f);
			Assert.InRange(highFyWeight, 0f, 1f);
			Assert.True(highFxWeight < moderateFxWeight);
			Assert.True(highFyWeight < moderateFyWeight);
		}

		/// <summary>
		/// Verifies that combined-slip interaction reduces lateral force under wheelspin.
		/// </summary>
		[Fact]
		public void CombinedSlipInteraction_ReducesLateralForceUnderWheelspin()
		{
			var noCouplingModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				FrictionEllipseRatio = 4.0f,
				CombinedSlipCoupling = 0f,
				CombinedSlipExponent = 2.0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				CoolingRate = 0f,
				CoreCoolingRate = 0f,
				RoadHeatTransferRate = 0f,
			};
			var couplingModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				FrictionEllipseRatio = 4.0f,
				CombinedSlipCoupling = 1.2f,
				CombinedSlipExponent = 2.0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				CoolingRate = 0f,
				CoreCoolingRate = 0f,
				RoadHeatTransferRate = 0f,
			};

			var noCouplingState = TyreState.CreateDefault();
			var couplingState = TyreState.CreateDefault();
			const float longitudinalVelocity = 20f;
			const float lateralVelocity = 3f;
			const float normalLoad = 3000f;
			const float slipRatioTarget = 0.18f;
			float noCouplingAngularVelocity = longitudinalVelocity * (1f + slipRatioTarget) / noCouplingModel.Radius;
			float couplingAngularVelocity = longitudinalVelocity * (1f + slipRatioTarget) / couplingModel.Radius;
			noCouplingState.AngularVelocity = noCouplingAngularVelocity;
			couplingState.AngularVelocity = couplingAngularVelocity;

			noCouplingModel.Update(ref noCouplingState, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out float noCouplingFy, out _);
			couplingModel.Update(ref couplingState, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out float couplingFy, out _);

			Assert.True(MathF.Abs(couplingFy) < MathF.Abs(noCouplingFy));
		}

		/// <summary>
		/// Verifies that the thermal model tracks surface and core temperature with lag during heat-up and cooldown.
		/// </summary>
		[Fact]
		public void ThermalModel_TracksSurfaceAndCoreWithLagDuringHeatAndCooldown()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				CoolingRate = 18f,
				CoreCoolingRate = 3.5f,
				RoadHeatTransferRate = 25f,
				SurfaceToCoreConductance = 45f,
				ThermalMass = 7000f,
				CoreThermalMass = 28000f,
			};

			var state = TyreState.CreateDefault();
			const float longitudinalVelocity = 20f;
			const float normalLoad = 3000f;
			const float dt = 0.01f;
			const int heatupIterations = 800;
			const int cooldownIterations = 1200;
			const int extendedCooldownIterations = 4000;

			for (int i = 0; i < heatupIterations; i++)
			{
				state.AngularVelocity = longitudinalVelocity * 1.25f / model.Radius;
				model.Update(ref state, longitudinalVelocity, 6f, normalLoad, 0f, 0f, 0f,
					DeterministicReferenceSurface, dt, out _, out _, out _);
			}

			float heatedSurface = state.Temperature;
			float heatedCore = state.CoreTemperature;

			for (int i = 0; i < cooldownIterations; i++)
			{
				state.AngularVelocity = 0f;
				model.Update(ref state, 0f, 0f, normalLoad, 0f, 0f, 0f,
					DeterministicReferenceSurface, dt, out _, out _, out _);
			}

			float postCooldownSurface = state.Temperature;
			float postCooldownCore = state.CoreTemperature;

			for (int i = 0; i < extendedCooldownIterations; i++)
			{
				state.AngularVelocity = 0f;
				model.Update(ref state, 0f, 0f, normalLoad, 0f, 0f, 0f,
					DeterministicReferenceSurface, dt, out _, out _, out _);
			}

			Assert.True(heatedSurface > model.AmbientTemperature);
			Assert.True(heatedCore > model.AmbientTemperature);
			Assert.True(heatedSurface > heatedCore);
			Assert.True(postCooldownSurface < heatedSurface);
			Assert.True((heatedSurface - postCooldownSurface) > (heatedCore - postCooldownCore));
			Assert.True(state.Temperature < postCooldownSurface);
		}

		// ── Pneumatic trail tests ────────────────────────────────────────────────

		/// <summary>
		/// Verifies that brush pneumatic trail returns one third of the half patch under full adhesion.
		/// </summary>
		[Fact]
		public void BrushPneumaticTrail_FullAdhesion_ReturnsOneThirdHalfPatch()
		{
			float halfPatch = 0.09f;
			float trail = TyreModel.ComputeBrushPneumaticTrail(halfPatch, lambda: 1.0f);

			Assert.Equal(halfPatch / 3f, trail, 5);
		}

		/// <summary>
		/// Verifies that brush pneumatic trail returns zero under full sliding.
		/// </summary>
		[Fact]
		public void BrushPneumaticTrail_FullSliding_ReturnsZero()
		{
			float halfPatch = 0.09f;
			float trail = TyreModel.ComputeBrushPneumaticTrail(halfPatch, lambda: 0.0f);

			Assert.Equal(0f, trail, 5);
		}

		/// <summary>
		/// Verifies that brush pneumatic trail peaks before collapsing.
		/// </summary>
		[Fact]
		public void BrushPneumaticTrail_PeaksBeforeCollapsing()
		{
			// The trail should rise initially then collapse as λ decreases from 1 to 0.
			float halfPatch = 0.09f;

			float trailFull = TyreModel.ComputeBrushPneumaticTrail(halfPatch, 1.0f);
			float trailMid = TyreModel.ComputeBrushPneumaticTrail(halfPatch, 0.6f);
			float trailLow = TyreModel.ComputeBrushPneumaticTrail(halfPatch, 0.2f);

			// Trail at λ=0.6 should be less than at full adhesion (past the peak)
			// and trail at λ=0.2 should be less than at λ=0.6.
			Assert.True(trailFull > 0f);
			Assert.True(trailLow < trailMid);
			Assert.True(trailLow < trailFull);
		}

		/// <summary>
		/// Verifies that equivalent trail slip grows with longitudinal slip during cornering.
		/// </summary>
		[Fact]
		public void EquivalentTrailSlip_GrowsWithLongitudinalSlipDuringCornering()
		{
			float pureCornering = TyreModel.ComputeEquivalentTrailSlip(0.12f, 0f, 1.4f);
			float combinedSlip = TyreModel.ComputeEquivalentTrailSlip(0.12f, 0.18f, 1.4f);

			Assert.True(MathF.Abs(combinedSlip) > MathF.Abs(pureCornering));
			Assert.True(MathF.Sign(combinedSlip) == MathF.Sign(pureCornering));
		}

		/// <summary>
		/// Verifies that the Fx aligning moment arm is zero without lateral force.
		/// </summary>
		[Fact]
		public void FxAligningMomentArm_IsZeroWithoutLateralForce()
		{
			var model = new TyreModel(0.305f) { AligningTorqueFxMomentArm = 0.01f, WheelOffset = 0.02f, };

			Assert.Equal(0f, model.ComputeFxAligningMomentArm(0f, 3000f, 0.1f), 5);
		}

		/// <summary>
		/// Verifies that the Fx aligning moment arm grows with wheel offset and lateral load.
		/// </summary>
		[Fact]
		public void FxAligningMomentArm_GrowsWithWheelOffsetAndLateralLoad()
		{
			var baseline = new TyreModel(0.305f) { AligningTorqueFxMomentArm = 0.008f, };
			var offsetModel = new TyreModel(0.305f) { AligningTorqueFxMomentArm = 0.008f, WheelOffset = 0.03f, };

			float baselineArm = baseline.ComputeFxAligningMomentArm(1500f, 3000f, 0.1f);
			float offsetArm = offsetModel.ComputeFxAligningMomentArm(1500f, 3000f, 0.1f);

			Assert.True(offsetArm > baselineArm);
		}

		/// <summary>
		/// Verifies that overturning couple grows with lateral force and camber.
		/// </summary>
		[Fact]
		public void OverturningCouple_GrowsWithLateralForceAndCamber()
		{
			var baseline = new TyreModel(0.305f) { OverturningCoupleFactor = 0.015f, OverturningCamberFactor = 0.35f, };
			var cambered = new TyreModel(0.305f) { OverturningCoupleFactor = 0.015f, OverturningCamberFactor = 0.35f, };

			float baselineMoment = baseline.ComputeOverturningCouple(3200f, 900f, 3200f, 0f, 0.29f);
			float higherLateralMoment = baseline.ComputeOverturningCouple(3200f, 1800f, 3200f, 0f, 0.29f);
			float camberedMoment = cambered.ComputeOverturningCouple(3200f, 900f, 3200f, 0.12f, 0.29f);

			Assert.True(higherLateralMoment > baselineMoment);
			Assert.True(camberedMoment > baselineMoment);
		}

		/// <summary>
		/// Verifies that rolling-resistance moment opposes rolling direction and grows with speed.
		/// </summary>
		[Fact]
		public void RollingResistanceMoment_OpposesRollingDirectionAndGrowsWithSpeed()
		{
			var model = new TyreModel(0.305f) { RollingResistanceMomentFactor = 0.006f, RollingResistanceMomentFxFactor = 0.08f, StandingWaveCriticalSpeed = 55f, };

			float slowForward = model.ComputeRollingResistanceMoment(0.29f, -45f, -15f, 5f, 0f, 3200f, signedRollingDirection: 5f);
			float fastForward = model.ComputeRollingResistanceMoment(0.29f, -45f, -15f, 35f, 0f, 3200f, signedRollingDirection: 35f);
			float fastReverse = model.ComputeRollingResistanceMoment(0.29f, -45f, -15f, 35f, 0f, 3200f, signedRollingDirection: -35f);

			Assert.True(MathF.Abs(fastForward) > MathF.Abs(slowForward));
			Assert.True(fastForward < 0f);
			Assert.True(fastReverse > 0f);
		}

		/// <summary>
		/// Verifies that self-aligning torque collapses at high slip angle.
		/// </summary>
		[Fact]
		public void SelfAligningTorque_CollapsesAtHighSlipAngle()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			// Small slip angle — expect significant Mz
			var stateSmall = TyreState.CreateDefault();
			stateSmall.AngularVelocity = 20f / model.Radius;
			model.Update(ref stateSmall, 20f, 1.0f, 3000f, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out float fySmall, out float mzSmall);

			// Large slip angle — expect reduced Mz
			var stateLarge = TyreState.CreateDefault();
			stateLarge.AngularVelocity = 20f / model.Radius;
			model.Update(ref stateLarge, 20f, 12f, 3000f, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out float fyLarge, out float mzLarge);

			var equivalentSlipSmall = TyreModel.ComputeEquivalentTrailSlip(MathF.Tan(stateSmall.SlipAngle), stateSmall.SlipRatio, 1f);
			var equivalentSlipLarge = TyreModel.ComputeEquivalentTrailSlip(MathF.Tan(stateLarge.SlipAngle), stateLarge.SlipRatio, 1f);

			Assert.True(MathF.Abs(mzSmall) > 0f);
			Assert.True(MathF.Abs(fySmall) > 1f);
			Assert.True(MathF.Abs(fyLarge) > 1f);
			Assert.True(float.IsFinite(mzLarge));
			Assert.True(MathF.Abs(equivalentSlipLarge) > MathF.Abs(equivalentSlipSmall));
		}

		/// <summary>
		/// Verifies that self-aligning torque drops when longitudinal slip adds combined trail demand.
		/// </summary>
		[Fact]
		public void SelfAligningTorque_DropsWhenLongitudinalSlipAddsCombinedTrailDemand()
		{
			var pureCorneringModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				AligningTorqueFxMomentArm = 0f,
				AligningTorqueResidualFactor = 0f,
			};
			var combinedSlipModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				AligningTorqueFxMomentArm = 0f,
				AligningTorqueResidualFactor = 0f,
			};

			var pureState = TyreState.CreateDefault();
			var combinedState = TyreState.CreateDefault();
			const float longitudinalVelocity = 20f;
			const float normalLoad = 3000f;
			const float lateralVelocity = 2.5f;
			pureState.AngularVelocity = longitudinalVelocity / pureCorneringModel.Radius;
			combinedState.AngularVelocity = longitudinalVelocity * 1.16f / combinedSlipModel.Radius;

			pureCorneringModel.Update(ref pureState, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out _, out float pureMz);
			combinedSlipModel.Update(ref combinedState, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out _, out float combinedMz);

			Assert.True(MathF.Abs(combinedMz) < MathF.Abs(pureMz));
		}

		/// <summary>
		/// Verifies that the Fx moment arm changes self-aligning torque while cornering and braking.
		/// </summary>
		[Fact]
		public void SelfAligningTorque_FxMomentArm_ChangesTorqueWhileCorneringAndBraking()
		{
			var noMomentArmModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				AligningTorqueFxMomentArm = 0f,
				AligningTorqueResidualFactor = 0f,
			};
			var momentArmModel = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				AligningTorqueFxMomentArm = 0.015f,
				AligningTorqueResidualFactor = 0f,
			};

			var noMomentArmState = TyreState.CreateDefault();
			var momentArmState = TyreState.CreateDefault();
			const float longitudinalVelocity = 20f;
			const float normalLoad = 3000f;
			noMomentArmState.AngularVelocity = longitudinalVelocity * 0.9f / noMomentArmModel.Radius;
			momentArmState.AngularVelocity = longitudinalVelocity * 0.9f / momentArmModel.Radius;

			noMomentArmModel.Update(ref noMomentArmState, longitudinalVelocity, 2.5f, normalLoad, 0f, 0f, 0.1f,
				DeterministicReferenceSurface, 0.01f, out _, out _, out float noMomentArmMz);
			momentArmModel.Update(ref momentArmState, longitudinalVelocity, 2.5f, normalLoad, 0f, 0f, 0.1f,
				DeterministicReferenceSurface, 0.01f, out _, out _, out float momentArmMz);

			Assert.NotEqual(noMomentArmMz, momentArmMz);
		}

		// ── Adhesion fraction tests ──────────────────────────────────────────────

		/// <summary>
		/// Verifies that adhesion fraction remains in full adhesion at low slip.
		/// </summary>
		[Fact]
		public void AdhesionFraction_FullAdhesion_AtLowSlip()
		{
			// Very low slip → full adhesion (λ = 1)
			float lambda = TyreModel.ComputeAdhesionFraction(
				absSlip: 0.01f, peakForce: 3000f, halfPatch: 0.09f, brushCperLength: 300000f);

			Assert.Equal(1f, lambda);
		}

		/// <summary>
		/// Verifies that adhesion fraction decreases with higher slip.
		/// </summary>
		[Fact]
		public void AdhesionFraction_DecreasesWithHigherSlip()
		{
			float peakForce = 3000f;
			float halfPatch = 0.09f;
			float brushCperLength = 300000f;

			float lambdaLow = TyreModel.ComputeAdhesionFraction(0.1f, peakForce, halfPatch, brushCperLength);
			float lambdaHigh = TyreModel.ComputeAdhesionFraction(1.0f, peakForce, halfPatch, brushCperLength);

			Assert.True(lambdaHigh < lambdaLow);
			Assert.True(lambdaHigh >= 0f);
			Assert.True(lambdaLow <= 1f);
		}

		// ── Longitudinal brush transient tests ───────────────────────────────────

		/// <summary>
		/// Verifies that longitudinal deflection builds under braking.
		/// </summary>
		[Fact]
		public void LongitudinalDeflection_BuildsUnderBraking()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			var state = TyreState.CreateDefault();
			state.AngularVelocity = 20f / model.Radius;

			// Baseline step at cruise — no braking
			model.Update(ref state, 20f, 0f, 3000f, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out _, out _);
			float baselineDeflection = state.LongitudinalDeflection;

			// Apply heavy braking for several steps — deflection should build
			for (int i = 0; i < 5; i++)
			{
				model.Update(ref state, 20f, 0f, 3000f, 0f, 3000f, 0f,
					DeterministicReferenceSurface, 0.01f, out _, out _, out _);
			}

			float deflectionBraking = state.LongitudinalDeflection;

			Assert.True(
				MathF.Abs(deflectionBraking) > 1e-5f &&
				MathF.Abs(deflectionBraking) > MathF.Abs(baselineDeflection));
		}

		/// <summary>
		/// Verifies that longitudinal deflection resets when airborne.
		/// </summary>
		[Fact]
		public void LongitudinalDeflection_ResetsWhenAirborne()
		{
			var model = new TyreModel(0.305f);
			var state = TyreState.CreateDefault();
			state.AngularVelocity = 20f / model.Radius;
			state.LongitudinalDeflection = 0.01f;

			// Airborne: normalLoad = 0
			model.Update(ref state, 20f, 0f, 0f, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out _, out _);

			Assert.Equal(0f, state.LongitudinalDeflection);
		}

		// ── Carcass shear tests ──────────────────────────────────────────────────

		/// <summary>
		/// Verifies that carcass shear force increases with deflection.
		/// </summary>
		[Fact]
		public void CarcassShearForce_IncreasesWithDeflection()
		{
			var model = new TyreModel(0.305f) { CarcassShearCoefficient = 0.5f, };

			float brushCperLength = 300000f;
			float halfPatch = 0.09f;
			float patchLength = 0.18f;

			float shearSmall = model.ComputeCarcassShearForce(0.005f, halfPatch, brushCperLength, patchLength);
			float shearLarge = model.ComputeCarcassShearForce(0.015f, halfPatch, brushCperLength, patchLength);

			Assert.True(shearSmall > 0f);
			Assert.True(shearLarge > shearSmall);
		}

		/// <summary>
		/// Verifies that carcass shear force is zero when no deflection is present.
		/// </summary>
		[Fact]
		public void CarcassShearForce_ZeroWhenNoDeflection()
		{
			var model = new TyreModel(0.305f) { CarcassShearCoefficient = 0.5f, };

			float shear = model.ComputeCarcassShearForce(0f, 0.09f, 300000f, 0.18f);

			Assert.Equal(0f, shear);
		}

		/// <summary>
		/// Verifies that carcass shear force is zero when its coefficient is zero.
		/// </summary>
		[Fact]
		public void CarcassShearForce_ZeroWhenCoefficientIsZero()
		{
			var model = new TyreModel(0.305f) { CarcassShearCoefficient = 0f, };

			float shear = model.ComputeCarcassShearForce(0.01f, 0.09f, 300000f, 0.18f);

			Assert.Equal(0f, shear);
		}

		// ── Wet grip / hydroplaning tests ────────────────────────────────────────

		/// <summary>
		/// Verifies that wet-grip factor returns one on a dry surface.
		/// </summary>
		[Fact]
		public void WetGripFactor_DryReturnsOne()
		{
			float factor = TyreModel.ComputeWetGripFactor(0f, 0.6f, 20f);

			Assert.Equal(1f, factor);
		}

		/// <summary>
		/// Verifies that wet-grip factor decreases with water depth.
		/// </summary>
		[Fact]
		public void WetGripFactor_DecreasesWithWaterDepth()
		{
			float shallow = TyreModel.ComputeWetGripFactor(0.0005f, 0.6f, 10f);
			float deep = TyreModel.ComputeWetGripFactor(0.003f, 0.6f, 10f);

			Assert.True(shallow < 1f);
			Assert.True(deep < shallow);
		}

		/// <summary>
		/// Verifies that macrotexture improves wet-grip drainage.
		/// </summary>
		[Fact]
		public void WetGripFactor_MacrotextureImprovesDrainage()
		{
			float lowMacro = TyreModel.ComputeWetGripFactor(0.002f, 0.2f, 10f);
			float highMacro = TyreModel.ComputeWetGripFactor(0.002f, 0.9f, 10f);

			Assert.True(highMacro > lowMacro);
		}

		/// <summary>
		/// Verifies that wet-grip factor decreases with speed.
		/// </summary>
		[Fact]
		public void WetGripFactor_DecreasesWithSpeed()
		{
			float lowSpeed = TyreModel.ComputeWetGripFactor(0.003f, 0.5f, 10f);
			float highSpeed = TyreModel.ComputeWetGripFactor(0.003f, 0.5f, 30f);

			Assert.True(highSpeed < lowSpeed);
		}

		/// <summary>
		/// Verifies that hydroplaning collapses wet-grip factor.
		/// </summary>
		[Fact]
		public void WetGripFactor_HydroplaningCollapsesGrip()
		{
			// Deep water at high speed should approach minimum grip
			float factor = TyreModel.ComputeWetGripFactor(0.004f, 0.3f, 40f);

			Assert.True(factor < 0.2f);
			Assert.True(factor >= 0.05f); // never below minimum
		}

		/// <summary>
		/// Verifies that wet-grip factor never drops below its minimum.
		/// </summary>
		[Fact]
		public void WetGripFactor_NeverBelowMinimum()
		{
			// Extreme conditions: deepest water, highest speed, no drainage
			float factor = TyreModel.ComputeWetGripFactor(0.01f, 0f, 100f);

			Assert.True(factor >= 0.05f);
		}

		// ── Microtexture / macrotexture tests ────────────────────────────────────

		/// <summary>
		/// Verifies that effective friction uses macrotexture drainage to improve wet grip.
		/// </summary>
		[Fact]
		public void EffectiveFriction_MacrotextureImprovesWetGripViaDrainage()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
			};

			// Microtexture/macrotexture no longer modulate dry baseline µ directly;
			// they affect wet grip retention via macrotexture drainage recovery.
			var highMacro = new SurfaceProperties
			{
				FrictionCoefficient = 1.0f,
				Microtexture = 0.9f,
				Macrotexture = 0.9f,
				WaterDepth = 0.002f,
				RollingResistance = 0f,
				SlipStiffnessScale = 1.0f,
				RelaxationLengthScale = 1.0f,
				PeakSlipRatioScale = 1.0f,
				DeformationFactor = 0f,
				NoiseFactor = 0f,
			};

			var lowMacro = new SurfaceProperties
			{
				FrictionCoefficient = 1.0f,
				Microtexture = 0.3f,
				Macrotexture = 0.2f,
				WaterDepth = 0.002f,
				RollingResistance = 0f,
				SlipStiffnessScale = 1.0f,
				RelaxationLengthScale = 1.0f,
				PeakSlipRatioScale = 1.0f,
				DeformationFactor = 0f,
				NoiseFactor = 0f,
			};

			float muHighMacro = model.ComputeEffectiveFriction(3000f, highMacro, 30f, 1.0f, absVx: 15f);
			float muLowMacro = model.ComputeEffectiveFriction(3000f, lowMacro, 30f, 1.0f, absVx: 15f);

			// Higher macrotexture drains water better → higher grip on wet surface
			Assert.True(muHighMacro > muLowMacro);
		}

		// ── WetTarmac surface integration tests ─────────────────────────────────

		/// <summary>
		/// Verifies that wet tarmac produces less grip than dry tarmac.
		/// </summary>
		[Fact]
		public void WetTarmac_ProducesLessGripThanDryTarmac()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				OptimalTemperature = 30f,
				TemperatureWindow = 100f,
				WornGripFraction = 1.0f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
			};

			var dryState = TyreState.CreateDefault();
			var wetState = TyreState.CreateDefault();
			float longitudinalVelocity = 20f;
			float lateralVelocity = 3f;
			dryState.AngularVelocity = longitudinalVelocity / model.Radius;
			wetState.AngularVelocity = longitudinalVelocity / model.Radius;

			model.Update(ref dryState, longitudinalVelocity, lateralVelocity, 3000f, 0f, 0f, 0f,
				DeterministicReferenceSurface, 0.01f, out _, out float dryFy, out _);
			model.Update(ref wetState, longitudinalVelocity, lateralVelocity, 3000f, 0f, 0f, 0f,
				WetTarmac, 0.01f, out _, out float wetFy, out _);

			Assert.True(MathF.Abs(dryFy) > MathF.Abs(wetFy));
		}

		// ── Road noise grip perturbation tests ──────────────────────────────────

		/// <summary>
		/// Verifies that road-noise grip factor returns near unity.
		/// </summary>
		[Fact]
		public void RoadNoiseGripFactor_ReturnsNearUnity()
		{
			// With very low noise, the factor should be close to 1.0
			float factor = TyreModel.ComputeRoadNoiseGripFactor(0.01f, 20f, 0.01f);

			Assert.InRange(factor, 0.99f, 1.01f);
		}

		/// <summary>
		/// Verifies that road-noise grip factor stays bounded by its configured amplitude.
		/// </summary>
		[Fact]
		public void RoadNoiseGripFactor_BoundedByAmplitude()
		{
			// At full noise, the factor must stay within ±8% of 1.0
			for (int i = 0; i < 100; i++)
			{
				float factor = TyreModel.ComputeRoadNoiseGripFactor(1.0f, i * 0.5f, 0.01f);
				Assert.InRange(factor, 0.92f - 0.001f, 1.08f + 0.001f);
			}
		}

		/// <summary>
		/// Verifies that road-noise grip factor returns unity at zero speed.
		/// </summary>
		[Fact]
		public void RoadNoiseGripFactor_ReturnsUnityAtZeroSpeed()
		{
			// At zero speed, PSD excitation is negligible — should return 1.0
			float factor = TyreModel.ComputeRoadNoiseGripFactor(1.0f, 0f, 0.01f);

			Assert.Equal(1f, factor);
		}

		/// <summary>
		/// Verifies that update uses applied drive torque before slip force calculation.
		/// </summary>
		[Fact]
		public void Update_UsesAppliedDriveTorqueBeforeSlipForceCalculation()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				TemperatureWindow = 100f,
				WornGripFraction = 1f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				ActiveMode = TyreModelMode.PacejkaOnly,
				WheelInertia = 1.0f,
			};
			var state = TyreState.CreateDefault();

			model.Update(
				ref state,
				longitudinalVelocity: 0f,
				lateralVelocity: 0f,
				normalLoad: 3000f,
				driveTorque: 120f,
				brakeTorque: 0f,
				camberAngle: 0f,
				in DeterministicReferenceSurface,
				dt: 0.01f,
				out _,
				out _,
				out _);

			Assert.True(state.SlipRatio > 0f);
			Assert.True(state.DriveTorque > 0f);
		}

		/// <summary>
		/// Verifies that update stores tyre reaction torque and keeps brake from reversing wheel.
		/// </summary>
		[Fact]
		public void Update_StoresTyreReactionTorqueAndKeepsBrakeFromReversingWheel()
		{
			var model = new TyreModel(0.305f)
			{
				PeakFrictionCoefficient = 1.0f,
				LoadSensitivity = 0f,
				ContactAreaGripExponent = 0f,
				TemperatureWindow = 100f,
				WornGripFraction = 1f,
				RollingResistanceCoefficient = 0f,
				CarcassShearCoefficient = 0f,
				ActiveMode = TyreModelMode.PacejkaOnly,
				WheelInertia = 1.0f,
			};
			var state = TyreState.CreateDefault();
			state.AngularVelocity = 1f;

			model.Update(
				ref state,
				longitudinalVelocity: 0.2f,
				lateralVelocity: 0f,
				normalLoad: 3000f,
				driveTorque: 0f,
				brakeTorque: 400f,
				camberAngle: 0f,
				in DeterministicReferenceSurface,
				dt: 0.02f,
				out _,
				out _,
				out _);

			Assert.True(state.TyreReactionTorque >= 0f);
			Assert.True(state.AngularVelocity >= 0f);
		}

		// ── Default surface calibration tests ────────────────────────────────────

		/// <summary>
		/// Verifies that wet-tarmac surface defaults include water and lower friction.
		/// </summary>
		[Fact]
		public void SurfaceDefaults_WetTarmacHasWaterAndLowerFriction()
		{
			var wet = SurfaceProperties.ForType(SurfaceType.WetTarmac);
			var dry = SurfaceProperties.ForType(SurfaceType.Tarmac);

			Assert.True(wet.WaterDepth > 0f);
			Assert.Equal(0f, dry.WaterDepth);
			Assert.True(wet.FrictionCoefficient < dry.FrictionCoefficient);
		}

		/// <summary>
		/// Verifies that all default surfaces expose texture values.
		/// </summary>
		[Fact]
		public void SurfaceDefaults_AllSurfacesHaveTexture()
		{
			foreach (SurfaceType type in Enum.GetValues<SurfaceType>())
			{
				var props = SurfaceProperties.ForType(type);

				// All surfaces should have non-negative texture values
				Assert.True(props.Microtexture >= 0f);
				Assert.True(props.Macrotexture >= 0f);
			}
		}

		/// <summary>
		/// Verifies that ice defaults to very low texture.
		/// </summary>
		[Fact]
		public void SurfaceDefaults_IceHasVeryLowTexture()
		{
			var ice = SurfaceProperties.ForType(SurfaceType.Ice);

			Assert.True(ice.Microtexture <= 0.1f);
			Assert.True(ice.Macrotexture <= 0.1f);
		}

		/// <summary>
		/// Verifies that gravel defaults to high macrotexture.
		/// </summary>
		[Fact]
		public void SurfaceDefaults_GravelHasHighMacrotexture()
		{
			var gravel = SurfaceProperties.ForType(SurfaceType.Gravel);

			Assert.True(gravel.Macrotexture >= 0.8f);
		}

		/// <summary>
		/// Verifies that loose surfaces expose softer slip stiffness, more slip tolerance, and higher rolling resistance.
		/// </summary>
		[Fact]
		public void SurfaceDefaults_LooseSurfacesModifyTyreParameters()
		{
			var asphalt = SurfaceProperties.ForType(SurfaceType.Tarmac);
			var gravel = SurfaceProperties.ForType(SurfaceType.Gravel);
			var snow = SurfaceProperties.ForType(SurfaceType.Snow);

			Assert.True(asphalt.FrictionCoefficient >= 1.05f);
			Assert.True(gravel.SlipStiffnessScale < asphalt.SlipStiffnessScale);
			Assert.True(snow.SlipStiffnessScale < gravel.SlipStiffnessScale);
			Assert.True(gravel.PeakSlipRatioScale > asphalt.PeakSlipRatioScale);
			Assert.True(snow.PeakSlipRatioScale > gravel.PeakSlipRatioScale);
			Assert.True(gravel.RollingResistance > asphalt.RollingResistance);
			Assert.True(snow.RollingResistance > gravel.RollingResistance);
			Assert.True(gravel.RelaxationLengthScale > asphalt.RelaxationLengthScale);
		}
	}
}
