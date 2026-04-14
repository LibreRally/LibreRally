using LibreRally.Vehicle;
using Stride.Core.Mathematics;

namespace LibreRally.Tests;

public class RallyCarComponentControlTests
{
    [Fact]
    public void ClampShiftRpmForSlip_CapsExcessWheelspinAboveRoadSpeed()
    {
        float shiftRpm = RallyCarComponent.ClampShiftRpmForSlip(1500f, 6500f, 900f);

        Assert.Equal(2400f, shiftRpm);
    }

    [Fact]
    public void ClampShiftRpmForSlip_DoesNotDropBelowRoadSpeed()
    {
        float shiftRpm = RallyCarComponent.ClampShiftRpmForSlip(3200f, 1800f, 900f);

        Assert.Equal(3200f, shiftRpm);
    }

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

    [Fact]
    public void ComputePointVelocity_AddsAngularContributionAtWheelPoint()
    {
        Vector3 pointVelocity = RallyCarComponent.ComputePointVelocity(
            linearVelocity: new Vector3(0f, 0f, 10f),
            angularVelocity: new Vector3(0f, 1f, 0f),
            pointOffset: new Vector3(0f, 0f, 2f));

        Assert.Equal(new Vector3(2f, 0f, 10f), pointVelocity);
    }

    [Fact]
    public void ComputePointVelocity_ReturnsLinearVelocity_WhenAngularRateIsZero()
    {
        Vector3 pointVelocity = RallyCarComponent.ComputePointVelocity(
            linearVelocity: new Vector3(1f, -2f, 3f),
            angularVelocity: Vector3.Zero,
            pointOffset: new Vector3(4f, 5f, 6f));

        Assert.Equal(new Vector3(1f, -2f, 3f), pointVelocity);
    }

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

    [Fact]
    public void ComputeTractionControlTorqueScale_RemainsFull_WhenDriveWheelMatchesRoadSpeed()
    {
        float scale = RallyCarComponent.ComputeTractionControlTorqueScale(
            maxDrivenWheelOmega: 8f,
            roadWheelOmega: 8f,
            effectiveRatio: 4.5f,
            wheelspinWindowRpm: 800f,
            minTorqueScale: 0.08f);

        Assert.Equal(1f, scale, 3);
    }

    [Fact]
    public void ComputeTractionControlTorqueScale_ReachesFloor_WhenSingleDrivenWheelRunsFarAhead()
    {
        float scale = RallyCarComponent.ComputeTractionControlTorqueScale(
            maxDrivenWheelOmega: 40f,
            roadWheelOmega: 2f,
            effectiveRatio: 4.5f,
            wheelspinWindowRpm: 800f,
            minTorqueScale: 0.08f);

        Assert.Equal(0.08f, scale, 3);
    }

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

    [Fact]
    public void ComputeGroundProbeContactScale_IsFullAtNominalContact()
    {
        float contactScale = RallyCarComponent.ComputeGroundProbeContactScale(
            hitDistance: 0.66f,
            wheelRadius: 0.305f,
            probeMargin: 0.05f);

        Assert.Equal(1f, contactScale, 3);
    }

    [Fact]
    public void ComputeGroundProbeContactScale_FadesOutAcrossProbeMargin()
    {
        float contactScale = RallyCarComponent.ComputeGroundProbeContactScale(
            hitDistance: 0.71f,
            wheelRadius: 0.305f,
            probeMargin: 0.05f);

        Assert.Equal(0f, contactScale, 3);
    }

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

    [Fact]
    public void ApplyYawAssistTopUp_DoesNotDampExistingYawInSameDirection()
    {
        float adjustedYaw = RallyCarComponent.ApplyYawAssistTopUp(
            currentYawRate: 0.35f,
            targetYawRate: 0.1f,
            maxDelta: 0.02f);

        Assert.Equal(0.35f, adjustedYaw);
    }

    [Fact]
    public void ApplyTriggerDeadzone_RejectsSmallTriggerNoise()
    {
        float filtered = RallyCarComponent.ApplyTriggerDeadzone(0.05f, 0.08f);

        Assert.Equal(0f, filtered);
    }

    [Fact]
    public void ApplyTriggerDeadzone_ReScalesRemainingTriggerTravel()
    {
        float filtered = RallyCarComponent.ApplyTriggerDeadzone(0.54f, 0.08f);

        Assert.Equal(0.5f, filtered, 3);
    }

    [Fact]
    public void ApplySignedAxisDeadzone_RejectsSmallStickDrift()
    {
        float filtered = RallyCarComponent.ApplySignedAxisDeadzone(-0.09f, 0.12f);

        Assert.Equal(0f, filtered);
    }

    [Fact]
    public void ApplySignedAxisDeadzone_PreservesSignAfterDeadzone()
    {
        float filtered = RallyCarComponent.ApplySignedAxisDeadzone(-0.56f, 0.12f);

        Assert.Equal(-0.5f, filtered, 3);
    }

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
