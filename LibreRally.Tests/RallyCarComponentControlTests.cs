using LibreRally.Vehicle;
using LibreRally.Vehicle.Physics;
using Stride.Core.Mathematics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies drivetrain, control-filtering, assist, and surface-VFX helpers in <see cref="RallyCarComponent"/>.
	/// </summary>
	public class RallyCarComponentControlTests
	{
		/// <summary>
		/// Verifies that shift RPM clamping caps excess wheelspin above road speed.
		/// </summary>
		[Fact]
		public void ClampShiftRpmForSlip_CapsExcessWheelspinAboveRoadSpeed()
		{
			float shiftRpm = RallyCarComponent.ClampShiftRpmForSlip(1500f, 6500f, 900f);

			Assert.Equal(2400f, shiftRpm);
		}

		/// <summary>
		/// Verifies that shift RPM clamping does not drop below road speed.
		/// </summary>
		[Fact]
		public void ClampShiftRpmForSlip_DoesNotDropBelowRoadSpeed()
		{
			float shiftRpm = RallyCarComponent.ClampShiftRpmForSlip(3200f, 1800f, 900f);

			Assert.Equal(3200f, shiftRpm);
		}

		/// <summary>
		/// Verifies that driven-wheel omega clamping preserves reverse direction while capping slip.
		/// </summary>
		[Fact]
		public void ClampDrivenWheelOmegaForSlip_PreservesReverseDirectionWhileCappingSlip()
		{
			float effectiveRatio = 4.5f;
			float roadWheelOmega = -2f;
			float drivenWheelOmega = -80f;

			float clampedOmega = RallyCarComponent.ClampDrivenWheelOmegaForSlip(
				roadWheelOmega,
				drivenWheelOmega,
				effectiveRatio,
				slipAllowanceRpm: 900f);

			float omegaToRpm = effectiveRatio * (60f / (2f * MathF.PI));
			float expectedOmegaMagnitude = ((MathF.Abs(roadWheelOmega) * omegaToRpm) + 900f) / omegaToRpm;

			Assert.Equal(-expectedOmegaMagnitude, clampedOmega, 3);
		}

		/// <summary>
		/// Verifies that driven-wheel omega resolution preserves grounded wheel direction.
		/// </summary>
		[Fact]
		public void ResolveDrivenWheelOmega_PreservesGroundedWheelDirection()
		{
			float resolvedOmega = RallyCarComponent.ResolveDrivenWheelOmega(
				fallbackOmega: -1f,
				forwardSpeed: -2f,
				sampledWheelOmegas: [-12f, -8f, 25f],
				sampledWheelGrounded: [true, true, false]);

			Assert.Equal(-10f, resolvedOmega, 3);
		}

		/// <summary>
		/// Verifies that driven-wheel omega resolution falls back to road omega when only airborne wheelspin exists at standstill.
		/// </summary>
		[Fact]
		public void ResolveDrivenWheelOmega_FallsBackToRoadOmega_WhenOnlyUngroundedWheelspinExistsAtStandstill()
		{
			float resolvedOmega = RallyCarComponent.ResolveDrivenWheelOmega(
				fallbackOmega: 0.25f,
				forwardSpeed: 0.1f,
				sampledWheelOmegas: [35f, 37f],
				sampledWheelGrounded: [false, false]);

			Assert.Equal(0.25f, resolvedOmega, 3);
		}

		/// <summary>
		/// Verifies that auto-clutch driven-wheel omega uses the slowest grounded driven wheel.
		/// </summary>
		[Fact]
		public void ResolveAutoClutchDrivenWheelOmega_UsesSlowestGroundedDrivenWheel()
		{
			float resolvedOmega = RallyCarComponent.ResolveAutoClutchDrivenWheelOmega(
				fallbackOmega: 0.25f,
				sampledWheelOmegas: [28f, 1.5f, 40f],
				sampledWheelGrounded: [true, true, false]);

			Assert.Equal(1.5f, resolvedOmega, 3);
		}

		/// <summary>
		/// Verifies that auto-clutch torque scaling does not punish a single grounded wheelspin outlier.
		/// </summary>
		[Fact]
		public void ComputeAutoClutchTorqueScale_DoesNotPunishSingleGroundedWheelspinOutlier()
		{
			float tractionWheelOmega = RallyCarComponent.ResolveAutoClutchDrivenWheelOmega(
				fallbackOmega: 0f,
				sampledWheelOmegas: [28f, 1f],
				sampledWheelGrounded: [true, true]);
			float slipClampedOmega = RallyCarComponent.ClampDrivenWheelOmegaForSlip(
				roadWheelOmega: 0f,
				drivenWheelOmega: tractionWheelOmega,
				effectiveRatio: 12.87f,
				slipAllowanceRpm: 1400f);
			float scale = RallyCarComponent.ComputeAutoClutchTorqueScale(
				drivenWheelOmega: tractionWheelOmega,
				slipClampedDrivenWheelOmega: slipClampedOmega,
				effectiveRatio: 12.87f,
				wheelspinWindowRpm: 1400f,
				minTorqueScale: 0.25f);

			Assert.Equal(1f, scale, 3);
		}

		/// <summary>
		/// Verifies that point velocity adds angular contribution at the wheel point.
		/// </summary>
		[Fact]
		public void ComputePointVelocity_AddsAngularContributionAtWheelPoint()
		{
			Vector3 pointVelocity = RallyCarComponent.ComputePointVelocity(
				linearVelocity: new Vector3(0f, 0f, 10f),
				angularVelocity: new Vector3(0f, 1f, 0f),
				pointOffset: new Vector3(0f, 0f, 2f));

			Assert.Equal(new Vector3(2f, 0f, 10f), pointVelocity);
		}

		/// <summary>
		/// Verifies that point velocity returns linear velocity when angular rate is zero.
		/// </summary>
		[Fact]
		public void ComputePointVelocity_ReturnsLinearVelocity_WhenAngularRateIsZero()
		{
			Vector3 pointVelocity = RallyCarComponent.ComputePointVelocity(
				linearVelocity: new Vector3(1f, -2f, 3f),
				angularVelocity: Vector3.Zero,
				pointOffset: new Vector3(4f, 5f, 6f));

			Assert.Equal(new Vector3(1f, -2f, 3f), pointVelocity);
		}

		/// <summary>
		/// Verifies that BeamNG camber precompression maps around unity in radians.
		/// </summary>
		[Fact]
		public void ConvertCamberPrecompressionToRadians_MapsBeamPrecompressionAroundUnity()
		{
			float camberRadians = RallyCarComponent.ConvertCamberPrecompressionToRadians(0.96f);

			Assert.Equal(-0.036f, camberRadians, 3);
		}

		/// <summary>
		/// Verifies that alignment camber angle adds compression gain to static camber.
		/// </summary>
		[Fact]
		public void ComputeAlignmentCamberAngle_AddsCompressionGainToStaticCamber()
		{
			float camberRadians = RallyCarComponent.ComputeAlignmentCamberAngle(
				staticCamberRadians: -0.03f,
				camberGainPerMeter: -0.35f,
				suspensionCompressionMeters: 0.10f);

			Assert.Equal(-0.065f, camberRadians, 3);
		}

		/// <summary>
		/// Verifies that alignment camber angle uses opposite signs across wheel sides.
		/// </summary>
		[Fact]
		public void ComputeAlignmentCamberAngle_UsesOppositeSignsAcrossWheelSides()
		{
			float alignmentCamber = RallyCarComponent.ComputeAlignmentCamberAngle(
				staticCamberRadians: -0.03f,
				camberGainPerMeter: -0.35f,
				suspensionCompressionMeters: 0.10f);

			float leftCamberRadians = alignmentCamber * RallyCarComponent.ComputeCamberSideSign(wheelSideDot: -0.6f);
			float rightCamberRadians = alignmentCamber * RallyCarComponent.ComputeCamberSideSign(wheelSideDot: 0.6f);

			Assert.Equal(-leftCamberRadians, rightCamberRadians, 3);
		}

		/// <summary>
		/// Verifies that auto-clutch torque scaling remains full when wheelspin matches the slip limit.
		/// </summary>
		[Fact]
		public void ComputeAutoClutchTorqueScale_RemainsFull_WhenWheelspinMatchesSlipLimit()
		{
			float scale = RallyCarComponent.ComputeAutoClutchTorqueScale(
				drivenWheelOmega: 12f,
				slipClampedDrivenWheelOmega: 12f,
				effectiveRatio: 4.5f,
				wheelspinWindowRpm: 1400f,
				minTorqueScale: 0.25f);

			Assert.Equal(1f, scale, 3);
		}

		/// <summary>
		/// Verifies that auto-clutch torque scaling reaches the floor when wheelspin greatly exceeds the slip limit.
		/// </summary>
		[Fact]
		public void ComputeAutoClutchTorqueScale_ReachesFloor_WhenWheelspinGreatlyExceedsSlipLimit()
		{
			float scale = RallyCarComponent.ComputeAutoClutchTorqueScale(
				drivenWheelOmega: 60f,
				slipClampedDrivenWheelOmega: 5f,
				effectiveRatio: 4.5f,
				wheelspinWindowRpm: 1400f,
				minTorqueScale: 0.25f);

			Assert.Equal(0.25f, scale, 3);
		}

		/// <summary>
		/// Verifies that traction-control torque scaling remains full when drive-wheel speed matches road speed.
		/// </summary>
		[Fact]
		public void ComputeTractionControlTorqueScale_RemainsFull_WhenDriveWheelMatchesRoadSpeed()
		{
			float scale = RallyCarComponent.ComputeTractionControlTorqueScale(
				drivenWheelSlipRatio: 0.15f,
				slipRatioTarget: 0.15f,
				slipRatioWindow: 0.10f,
				minTorqueScale: 0.08f);

			Assert.Equal(1f, scale, 3);
		}

		/// <summary>
		/// Verifies that traction-control torque scaling reaches the floor when a driven wheel runs far ahead.
		/// </summary>
		[Fact]
		public void ComputeTractionControlTorqueScale_ReachesFloor_WhenSingleDrivenWheelRunsFarAhead()
		{
			float scale = RallyCarComponent.ComputeTractionControlTorqueScale(
				drivenWheelSlipRatio: 0.30f,
				slipRatioTarget: 0.15f,
				slipRatioWindow: 0.10f,
				minTorqueScale: 0.08f);

			Assert.Equal(0.08f, scale, 3);
		}

		/// <summary>
		/// Verifies that ABS brake-torque scaling remains full when wheel slip is below target.
		/// </summary>
		[Fact]
		public void ComputeAbsBrakeTorqueScale_RemainsFull_WhenWheelSlipIsBelowTarget()
		{
			float scale = RallyCarComponent.ComputeAbsBrakeTorqueScale(
				slipRatio: -0.10f,
				rollingDirection: 1f,
				slipRatioTarget: 0.15f,
				slipRatioWindow: 0.10f,
				minBrakeScale: 0.18f);

			Assert.Equal(1f, scale, 3);
		}

		/// <summary>
		/// Verifies that ABS brake-torque scaling reaches the floor when a forward wheel approaches lockup.
		/// </summary>
		[Fact]
		public void ComputeAbsBrakeTorqueScale_ReachesFloor_WhenForwardWheelApproachesLockup()
		{
			float scale = RallyCarComponent.ComputeAbsBrakeTorqueScale(
				slipRatio: -0.30f,
				rollingDirection: 1f,
				slipRatioTarget: 0.15f,
				slipRatioWindow: 0.10f,
				minBrakeScale: 0.18f);

			Assert.Equal(0.18f, scale, 3);
		}

		/// <summary>
		/// Verifies that ABS brake-torque scaling uses travel direction for reverse braking.
		/// </summary>
		[Fact]
		public void ComputeAbsBrakeTorqueScale_UsesTravelDirectionForReverseBraking()
		{
			float scale = RallyCarComponent.ComputeAbsBrakeTorqueScale(
				slipRatio: 0.30f,
				rollingDirection: -1f,
				slipRatioTarget: 0.15f,
				slipRatioWindow: 0.10f,
				minBrakeScale: 0.18f);

			Assert.Equal(0.18f, scale, 3);
		}

		/// <summary>
		/// Verifies that ground-probe contact scale is full at nominal contact.
		/// </summary>
		[Fact]
		public void ComputeGroundProbeContactScale_IsFullAtNominalContact()
		{
			float contactScale = RallyCarComponent.ComputeGroundProbeContactScale(
				hitDistance: 0.66f,
				wheelRadius: 0.305f,
				probeMargin: 0.05f);

			Assert.Equal(1f, contactScale, 3);
		}

		/// <summary>
		/// Verifies that ground-probe contact scale fades out across the probe margin.
		/// </summary>
		[Fact]
		public void ComputeGroundProbeContactScale_FadesOutAcrossProbeMargin()
		{
			float contactScale = RallyCarComponent.ComputeGroundProbeContactScale(
				hitDistance: 0.71f,
				wheelRadius: 0.305f,
				probeMargin: 0.05f);

			Assert.Equal(0f, contactScale, 3);
		}

		/// <summary>
		/// Verifies that surface blending only activates when two hits are close enough to matter.
		/// </summary>
		[Fact]
		public void ComputeSurfaceBlendFactor_ActivatesOnlyInsideBlendWindow()
		{
			float blended = RallyCarComponent.ComputeSurfaceBlendFactor(
				primaryDistance: 0.66f,
				secondaryDistance: 0.69f,
				blendDistance: 0.08f);
			float isolated = RallyCarComponent.ComputeSurfaceBlendFactor(
				primaryDistance: 0.66f,
				secondaryDistance: 0.80f,
				blendDistance: 0.08f);

			Assert.InRange(blended, 0.01f, 0.5f);
			Assert.Equal(0f, isolated, 3);
		}

		/// <summary>
		/// Verifies that track-surface overrides flow into effective tyre-interaction properties.
		/// </summary>
		[Fact]
		public void TrackSurfaceComponent_ResolveSurfaceProperties_AppliesOverrides()
		{
			var surface = new TrackSurfaceComponent
			{
				SurfaceType = SurfaceType.Gravel,
				SurfaceFrictionCoefficient = 0.78f,
				SurfaceSlipStiffnessScale = 0.46f,
				SurfaceRelaxationLengthScale = 1.45f,
				SurfaceSlipTolerance = 2.6f,
				RollingResistanceCoefficient = 0.05f,
			};

			var resolved = surface.ResolveSurfaceProperties();

			Assert.Equal(0.78f, resolved.FrictionCoefficient, 3);
			Assert.Equal(0.46f, resolved.SlipStiffnessScale, 3);
			Assert.Equal(1.45f, resolved.RelaxationLengthScale, 3);
			Assert.Equal(2.6f, resolved.PeakSlipRatioScale, 3);
			Assert.Equal(0.05f, resolved.RollingResistance, 3);
		}

		/// <summary>
		/// Verifies that wheel-surface VFX intensity stays off for low-slip tarmac running.
		/// </summary>
		[Fact]
		public void ComputeWheelSurfaceVfxIntensity_StaysOffForLowTarmacSlip()
		{
			float intensity = RallyCarComponent.ComputeWheelSurfaceVfxIntensity(
				surfaceType: SurfaceType.Tarmac,
				slipRatio: 0.08f,
				slipAngleRadians: 0.03f,
				normalLoadScale: 1f,
				contactScale: 1f);

			Assert.Equal(0f, intensity, 3);
		}

		/// <summary>
		/// Verifies that wheel-surface VFX intensity activates for a tarmac burnout.
		/// </summary>
		[Fact]
		public void ComputeWheelSurfaceVfxIntensity_ActivatesForBurnoutOnTarmac()
		{
			float intensity = RallyCarComponent.ComputeWheelSurfaceVfxIntensity(
				surfaceType: SurfaceType.Tarmac,
				slipRatio: 0.95f,
				slipAngleRadians: 0.12f,
				normalLoadScale: 1f,
				contactScale: 1f);

			Assert.True(intensity > 0.9f);
		}

		/// <summary>
		/// Verifies that wheel-surface VFX intensity activates earlier on gravel.
		/// </summary>
		[Fact]
		public void ComputeWheelSurfaceVfxIntensity_ActivatesEarlierOnGravel()
		{
			float intensity = RallyCarComponent.ComputeWheelSurfaceVfxIntensity(
				surfaceType: SurfaceType.Gravel,
				slipRatio: 0.18f,
				slipAngleRadians: 0.04f,
				normalLoadScale: 1f,
				contactScale: 1f);

			Assert.True(intensity > 0.2f);
		}

		/// <summary>
		/// Verifies that wheel-surface VFX intensity is clamped to one for tarmac spawn scaling.
		/// </summary>
		[Fact]
		public void ComputeWheelSurfaceVfxIntensity_IsClampedToOneForSpawnScaling()
		{
			float intensity = RallyCarComponent.ComputeWheelSurfaceVfxIntensity(
				surfaceType: SurfaceType.Tarmac,
				slipRatio: 1.6f,
				slipAngleRadians: 0.6f,
				normalLoadScale: 3f,
				contactScale: 1f);

			Assert.Equal(1f, intensity, 3);
		}

		/// <summary>
		/// Verifies that wheel-surface VFX intensity is clamped to one for gravel spawn scaling.
		/// </summary>
		[Fact]
		public void ComputeWheelSurfaceVfxIntensity_IsClampedToOneForSpawnScalingOnGravel()
		{
			float intensity = RallyCarComponent.ComputeWheelSurfaceVfxIntensity(
				surfaceType: SurfaceType.Gravel,
				slipRatio: 1.6f,
				slipAngleRadians: 0.6f,
				normalLoadScale: 3f,
				contactScale: 1f);

			Assert.Equal(1f, intensity, 3);
		}

		/// <summary>
		/// Verifies that low-speed yaw assist uses drive direction for reverse launch.
		/// </summary>
		[Fact]
		public void ComputeLowSpeedYawAssistRate_UsesDriveDirectionForReverseLaunch()
		{
			float assist = RallyCarComponent.ComputeLowSpeedYawAssistRate(
				steerRack: 1f,
				forwardSpeed: 0f,
				driveInput: 1f,
				driveDirection: -1f,
				assistGain: 1f);

			Assert.True(assist < 0f);
		}

		/// <summary>
		/// Verifies that low-speed yaw assist fades out once tyre forces should dominate.
		/// </summary>
		[Fact]
		public void ComputeLowSpeedYawAssistRate_FadesOutOnceTyreForcesShouldDominate()
		{
			float assist = RallyCarComponent.ComputeLowSpeedYawAssistRate(
				steerRack: 1f,
				forwardSpeed: 6f,
				driveInput: 1f,
				driveDirection: 1f,
				assistGain: 1f);

			Assert.Equal(0f, assist);
		}

		/// <summary>
		/// Verifies that yaw-assist top-up does not damp existing yaw in the same direction.
		/// </summary>
		[Fact]
		public void ApplyYawAssistTopUp_DoesNotDampExistingYawInSameDirection()
		{
			float adjustedYaw = RallyCarComponent.ApplyYawAssistTopUp(
				currentYawRate: 0.35f,
				targetYawRate: 0.1f,
				maxDelta: 0.02f);

			Assert.Equal(0.35f, adjustedYaw);
		}

		/// <summary>
		/// Verifies that trigger deadzone rejects small trigger noise.
		/// </summary>
		[Fact]
		public void ApplyTriggerDeadzone_RejectsSmallTriggerNoise()
		{
			float filtered = RallyCarComponent.ApplyTriggerDeadzone(0.05f, 0.08f);

			Assert.Equal(0f, filtered);
		}

		/// <summary>
		/// Verifies that trigger deadzone rescales remaining trigger travel.
		/// </summary>
		[Fact]
		public void ApplyTriggerDeadzone_ReScalesRemainingTriggerTravel()
		{
			float filtered = RallyCarComponent.ApplyTriggerDeadzone(0.54f, 0.08f);

			Assert.Equal(0.5f, filtered, 3);
		}

		/// <summary>
		/// Verifies that signed-axis deadzone rejects small stick drift.
		/// </summary>
		[Fact]
		public void ApplySignedAxisDeadzone_RejectsSmallStickDrift()
		{
			float filtered = RallyCarComponent.ApplySignedAxisDeadzone(-0.09f, 0.12f);

			Assert.Equal(0f, filtered);
		}

		/// <summary>
		/// Verifies that signed-axis deadzone preserves sign after the deadzone.
		/// </summary>
		[Fact]
		public void ApplySignedAxisDeadzone_PreservesSignAfterDeadzone()
		{
			float filtered = RallyCarComponent.ApplySignedAxisDeadzone(-0.56f, 0.12f);

			Assert.Equal(-0.5f, filtered, 3);
		}

		/// <summary>
		/// Verifies that vehicle wake retention is <see langword="false" /> when the car is stopped and no input is present.
		/// </summary>
		[Fact]
		public void ShouldKeepVehicleAwake_IsFalse_WhenStoppedAndNoInput()
		{
			bool keepAwake = RallyCarComponent.ShouldKeepVehicleAwake(
				throttleInput: 0f,
				brakeInput: 0f,
				steerInput: 0f,
				handbrakeRequested: false,
				linearVelocity: Vector3.Zero,
				angularVelocity: Vector3.Zero);

			Assert.False(keepAwake);
		}

		/// <summary>
		/// Verifies that vehicle wake retention is <see langword="true" /> when driver input is present.
		/// </summary>
		[Fact]
		public void ShouldKeepVehicleAwake_IsTrue_WhenDriverAppliesInput()
		{
			bool keepAwake = RallyCarComponent.ShouldKeepVehicleAwake(
				throttleInput: 0.2f,
				brakeInput: 0f,
				steerInput: 0f,
				handbrakeRequested: false,
				linearVelocity: Vector3.Zero,
				angularVelocity: Vector3.Zero);

			Assert.True(keepAwake);
		}

		/// <summary>
		/// Verifies that standing gear selection enters reverse when braking at standstill.
		/// </summary>
		[Fact]
		public void ResolveStandingGearSelection_EntersReverse_WhenBrakingAtStandstill()
		{
			int selectedGear = RallyCarComponent.ResolveStandingGearSelection(
				currentGear: 1,
				speedKmh: 0.4f,
				forwardSpeed: 0f,
				throttleInput: 0f,
				brakeInput: 1f,
				handbrakeRequested: false,
				shiftCooldown: 0f,
				reverseEngageSpeedKmh: 1.5f);

			Assert.Equal(0, selectedGear);
		}

		/// <summary>
		/// Verifies that standing gear selection does not enter reverse while the vehicle is still rolling.
		/// </summary>
		[Fact]
		public void ResolveStandingGearSelection_DoesNotEnterReverse_WhileStillRolling()
		{
			int selectedGear = RallyCarComponent.ResolveStandingGearSelection(
				currentGear: 1,
				speedKmh: 6f,
				forwardSpeed: 1.7f,
				throttleInput: 0f,
				brakeInput: 1f,
				handbrakeRequested: false,
				shiftCooldown: 0f,
				reverseEngageSpeedKmh: 1.5f);

			Assert.Equal(1, selectedGear);
		}
	}
}
