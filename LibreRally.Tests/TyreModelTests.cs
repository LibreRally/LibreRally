using LibreRally.Vehicle.Physics;

namespace LibreRally.Tests;

public class TyreModelTests
{
    private static readonly SurfaceProperties Tarmac = SurfaceProperties.ForType(SurfaceType.Tarmac);
    private static readonly SurfaceProperties WetTarmac = SurfaceProperties.ForType(SurfaceType.WetTarmac);

    [Fact]
    public void ContactPatchLength_GrowsWithLoad_AndShrinksWithPressure()
    {
        var model = new TyreModel(0.305f)
        {
            Width = 0.205f,
            ContactPatchLengthScale = 1.0f,
            TyrePressure = 220f,
        };

        float lightLoadPatch = model.ComputeEffectivePatchLength(1500f);
        float heavyLoadPatch = model.ComputeEffectivePatchLength(3000f);

        model.TyrePressure = 280f;
        float highPressurePatch = model.ComputeEffectivePatchLength(3000f);

        Assert.True(heavyLoadPatch > lightLoadPatch);
        Assert.True(highPressurePatch < heavyLoadPatch);
    }

    [Fact]
    public void EffectiveRollingRadius_DecreasesUnderLoad()
    {
        var model = new TyreModel(0.305f)
        {
            TyrePressure = 220f,
            VerticalStiffness = 200000f,
        };

        float unloadedRadius = model.ComputeEffectiveRollingRadius(0f);
        float loadedRadius = model.ComputeEffectiveRollingRadius(3000f);

        Assert.Equal(model.Radius, unloadedRadius);
        Assert.True(loadedRadius < unloadedRadius);
        Assert.True(loadedRadius > model.Radius * 0.9f);
    }

    [Fact]
    public void BrushStiffness_IncreasesWithPressureAndWidth()
    {
        var model = new TyreModel(0.305f)
        {
            Width = 0.205f,
            TyrePressure = 220f,
        };

        float baseline = model.ComputeEffectiveBrushStiffness();

        model.TyrePressure = 260f;
        float higherPressure = model.ComputeEffectiveBrushStiffness();

        model.TyrePressure = 220f;
        model.Width = 0.235f;
        float widerTyre = model.ComputeEffectiveBrushStiffness();

        Assert.True(higherPressure > baseline);
        Assert.True(widerTyre > baseline);
    }

    [Fact]
    public void StandingWaveResistanceFactor_GrowsWithSpeed()
    {
        var model = new TyreModel(0.305f)
        {
            StandingWaveCriticalSpeed = 65f,
            StandingWaveResistanceGain = 1.0f,
        };

        float lowSpeed = model.ComputeStandingWaveResistanceFactor(10f);
        float highSpeed = model.ComputeStandingWaveResistanceFactor(80f);

        Assert.True(lowSpeed >= 1f);
        Assert.True(highSpeed > lowSpeed);
    }

    [Fact]
    public void ContactPatchLengthAlias_MapsToScaleProperty()
    {
        var model = new TyreModel(0.305f);

#pragma warning disable CS0618
        model.ContactPatchLength = 1.2f;
#pragma warning restore CS0618

        Assert.Equal(1.2f, model.ContactPatchLengthScale);
    }

    [Fact]
    public void EffectiveRelaxationLength_GrowsOnLooseLowGripSurfaces()
    {
        var model = new TyreModel(0.305f)
        {
            CarcassStiffness = 1.0f,
        };

        float tarmacLateral = model.ComputeEffectiveRelaxationLength(Tarmac, longitudinal: false);
        float gravelLateral = model.ComputeEffectiveRelaxationLength(SurfaceProperties.ForType(SurfaceType.Gravel), longitudinal: false);
        float snowLateral = model.ComputeEffectiveRelaxationLength(SurfaceProperties.ForType(SurfaceType.Snow), longitudinal: false);
        float tarmacLongitudinal = model.ComputeEffectiveRelaxationLength(Tarmac, longitudinal: true);
        float gravelLongitudinal = model.ComputeEffectiveRelaxationLength(SurfaceProperties.ForType(SurfaceType.Gravel), longitudinal: true);

        Assert.True(gravelLateral > tarmacLateral);
        Assert.True(snowLateral > gravelLateral);
        Assert.True(gravelLongitudinal > tarmacLongitudinal);
    }

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

        float referenceMu = model.ComputeEffectiveFriction(3000f, Tarmac, 30f, 1.0f);
        float mediumLoadMu = model.ComputeEffectiveFriction(3300f, Tarmac, 30f, 1.0f);
        float highLoadMu = model.ComputeEffectiveFriction(6000f, Tarmac, 30f, 1.0f);

        Assert.Equal(1.0f, referenceMu, 4);
        Assert.True(mediumLoadMu < referenceMu);
        Assert.True(highLoadMu < mediumLoadMu);
        Assert.True(highLoadMu / mediumLoadMu < mediumLoadMu / referenceMu);
    }

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

        float measuredReferenceMu = model.ComputeEffectiveFriction(3000f, Tarmac, 30f, 1.0f);
        float highLoadMu = model.ComputeEffectiveFriction(6000f, Tarmac, 30f, 1.0f);
        float expectedHighLoadMu = referenceMu *
                                   (TyreModel.ComputeBeamNgLoadCoefficient(6000f, 1.6f, 0.5f, 0.000165f) / referenceMu);

        Assert.Equal(referenceMu, measuredReferenceMu, 4);
        Assert.Equal(expectedHighLoadMu, highLoadMu, 4);
    }

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
        float lowPressureMu = model.ComputeEffectiveFriction(3000f, Tarmac, 30f, 1.0f);

        model.TyrePressure = 280f;
        float highPressureMu = model.ComputeEffectiveFriction(3000f, Tarmac, 30f, 1.0f);

        Assert.True(lowPressureMu > highPressureMu);
    }

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
        float lateralVelocity = 7.3f;
        float normalLoad = 3000f;
        float angularVelocity = longitudinalVelocity / baselineModel.Radius;
        baselineState.AngularVelocity = angularVelocity;
        ellipticalState.AngularVelocity = angularVelocity;

        baselineModel.Update(ref baselineState, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
            Tarmac, 0.01f, out float baselineFx, out float baselineFy, out _);
        ellipticalModel.Update(ref ellipticalState, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
            Tarmac, 0.01f, out float ellipticalFx, out float ellipticalFy, out _);

        float fxMax = normalLoad;
        float fyMax = normalLoad * 0.9f;
        float ellipseValue = (ellipticalFx * ellipticalFx) / (fxMax * fxMax)
            + (ellipticalFy * ellipticalFy) / (fyMax * fyMax);

        Assert.True(MathF.Abs(ellipticalFy) < MathF.Abs(baselineFy));
        Assert.True(MathF.Abs(ellipticalFy) <= fyMax + 1e-3f);
        Assert.InRange(ellipseValue, 0.995f, 1.005f);
        Assert.True(MathF.Abs(ellipticalFy) < fxMax);
    }

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
            Tarmac, 0.01f, out _, out float lowPressureFy, out _);
        highPressureModel.Update(ref highPressureState, longitudinalVelocity, lateralVelocity, 3000f, 0f, 0f, 0f,
            Tarmac, 0.01f, out _, out float highPressureFy, out _);

        Assert.True(MathF.Abs(highPressureFy) > MathF.Abs(lowPressureFy));
    }

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

        float mu = model.ComputeEffectiveFriction(3000f, Tarmac, 30f, 1.0f);

        Assert.True(float.IsFinite(mu));
        Assert.InRange(mu, 0.5f, 2.0f);
    }

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
            Tarmac, 0.01f, out float fx, out float fy, out _);

        float fxMax = normalLoad;
        float fyMax = normalLoad * 0.9f;
        float ellipseValue = (fx * fx) / (fxMax * fxMax) + (fy * fy) / (fyMax * fyMax);

        Assert.InRange(ellipseValue, 0f, 1.0001f);
    }

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
            Tarmac, 0.01f, out float fx, out float fy, out _);

        float fxMax = normalLoad;
        float fyMax = normalLoad * 0.9f;
        float ellipseValue = (fx * fx) / (fxMax * fxMax) + (fy * fy) / (fyMax * fyMax);

        Assert.InRange(ellipseValue, 0.995f, 1.005f);
    }

    /// <summary>
    /// Verifies explicit combined-slip coupling: adding lateral slip reduces available longitudinal force.
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
            Tarmac, 0.01f, out float noCouplingFx, out float noCouplingFy, out _);
        couplingModel.Update(ref couplingState, longitudinalVelocity, 6f, normalLoad, 0f, 0f, 0f,
            Tarmac, 0.01f, out float couplingFx, out float couplingFy, out _);

        Assert.True(MathF.Abs(noCouplingFy) > 0f);
        Assert.True(MathF.Abs(couplingFy) > 0f);
        Assert.True(MathF.Abs(couplingFx) < MathF.Abs(noCouplingFx));
    }

    /// <summary>
    /// Verifies two-node tyre thermals: surface heats first, then core lags and remains warmer during cooldown.
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

        for (int i = 0; i < heatupIterations; i++)
        {
            state.AngularVelocity = longitudinalVelocity * 1.25f / model.Radius;
            model.Update(ref state, longitudinalVelocity, 6f, normalLoad, 0f, 0f, 0f,
                Tarmac, dt, out _, out _, out _);
        }

        float heatedSurface = state.Temperature;
        float heatedCore = state.CoreTemperature;

        for (int i = 0; i < cooldownIterations; i++)
        {
            state.AngularVelocity = longitudinalVelocity / model.Radius;
            model.Update(ref state, longitudinalVelocity, 0f, normalLoad, 0f, 0f, 0f,
                Tarmac, dt, out _, out _, out _);
        }

        Assert.True(heatedSurface > model.AmbientTemperature);
        Assert.True(heatedCore > model.AmbientTemperature);
        Assert.True(heatedSurface > heatedCore);
        Assert.True(state.Temperature < heatedSurface);
        Assert.True(state.CoreTemperature < heatedCore);
        Assert.True(state.CoreTemperature > state.Temperature);
    }

    // ── Pneumatic trail tests ────────────────────────────────────────────────

    [Fact]
    public void BrushPneumaticTrail_FullAdhesion_ReturnsOneThirdHalfPatch()
    {
        float halfPatch = 0.09f;
        float trail = TyreModel.ComputeBrushPneumaticTrail(halfPatch, lambda: 1.0f);

        Assert.Equal(halfPatch / 3f, trail, 5);
    }

    [Fact]
    public void BrushPneumaticTrail_FullSliding_ReturnsZero()
    {
        float halfPatch = 0.09f;
        float trail = TyreModel.ComputeBrushPneumaticTrail(halfPatch, lambda: 0.0f);

        Assert.Equal(0f, trail, 5);
    }

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
            Tarmac, 0.01f, out _, out _, out float mzSmall);

        // Large slip angle — expect reduced Mz
        var stateLarge = TyreState.CreateDefault();
        stateLarge.AngularVelocity = 20f / model.Radius;
        model.Update(ref stateLarge, 20f, 12f, 3000f, 0f, 0f, 0f,
            Tarmac, 0.01f, out _, out _, out float mzLarge);

        Assert.True(MathF.Abs(mzSmall) > 0f);
        // At high slip, the trail collapses so |Mz| per unit Fy should decrease
        Assert.True(MathF.Abs(mzLarge) < MathF.Abs(mzSmall) * 3f);
    }

    // ── Adhesion fraction tests ──────────────────────────────────────────────

    [Fact]
    public void AdhesionFraction_FullAdhesion_AtLowSlip()
    {
        // Very low slip → full adhesion (λ = 1)
        float lambda = TyreModel.ComputeAdhesionFraction(
            absSlip: 0.01f, peakForce: 3000f, halfPatch: 0.09f, brushCperLength: 300000f);

        Assert.Equal(1f, lambda);
    }

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
            Tarmac, 0.01f, out _, out _, out _);
        float baselineDeflection = state.LongitudinalDeflection;

        // Apply heavy braking for several steps — deflection should build
        for (int i = 0; i < 5; i++)
        {
            model.Update(ref state, 20f, 0f, 3000f, 0f, 3000f, 0f,
                Tarmac, 0.01f, out _, out _, out _);
        }

        float deflectionBraking = state.LongitudinalDeflection;

        Assert.True(
            MathF.Abs(deflectionBraking) > 1e-5f &&
            MathF.Abs(deflectionBraking) > MathF.Abs(baselineDeflection));
    }

    [Fact]
    public void LongitudinalDeflection_ResetsWhenAirborne()
    {
        var model = new TyreModel(0.305f);
        var state = TyreState.CreateDefault();
        state.AngularVelocity = 20f / model.Radius;
        state.LongitudinalDeflection = 0.01f;

        // Airborne: normalLoad = 0
        model.Update(ref state, 20f, 0f, 0f, 0f, 0f, 0f,
            Tarmac, 0.01f, out _, out _, out _);

        Assert.Equal(0f, state.LongitudinalDeflection);
    }

    // ── Carcass shear tests ──────────────────────────────────────────────────

    [Fact]
    public void CarcassShearForce_IncreasesWithDeflection()
    {
        var model = new TyreModel(0.305f)
        {
            CarcassShearCoefficient = 0.5f,
        };

        float brushCperLength = 300000f;
        float halfPatch = 0.09f;
        float patchLength = 0.18f;

        float shearSmall = model.ComputeCarcassShearForce(0.005f, halfPatch, brushCperLength, patchLength);
        float shearLarge = model.ComputeCarcassShearForce(0.015f, halfPatch, brushCperLength, patchLength);

        Assert.True(shearSmall > 0f);
        Assert.True(shearLarge > shearSmall);
    }

    [Fact]
    public void CarcassShearForce_ZeroWhenNoDeflection()
    {
        var model = new TyreModel(0.305f)
        {
            CarcassShearCoefficient = 0.5f,
        };

        float shear = model.ComputeCarcassShearForce(0f, 0.09f, 300000f, 0.18f);

        Assert.Equal(0f, shear);
    }

    [Fact]
    public void CarcassShearForce_ZeroWhenCoefficientIsZero()
    {
        var model = new TyreModel(0.305f)
        {
            CarcassShearCoefficient = 0f,
        };

        float shear = model.ComputeCarcassShearForce(0.01f, 0.09f, 300000f, 0.18f);

        Assert.Equal(0f, shear);
    }

    // ── Wet grip / hydroplaning tests ────────────────────────────────────────

    [Fact]
    public void WetGripFactor_DryReturnsOne()
    {
        float factor = TyreModel.ComputeWetGripFactor(0f, 0.6f, 20f);

        Assert.Equal(1f, factor);
    }

    [Fact]
    public void WetGripFactor_DecreasesWithWaterDepth()
    {
        float shallow = TyreModel.ComputeWetGripFactor(0.0005f, 0.6f, 10f);
        float deep = TyreModel.ComputeWetGripFactor(0.003f, 0.6f, 10f);

        Assert.True(shallow < 1f);
        Assert.True(deep < shallow);
    }

    [Fact]
    public void WetGripFactor_MacrotextureImprovesDrainage()
    {
        float lowMacro = TyreModel.ComputeWetGripFactor(0.002f, 0.2f, 10f);
        float highMacro = TyreModel.ComputeWetGripFactor(0.002f, 0.9f, 10f);

        Assert.True(highMacro > lowMacro);
    }

    [Fact]
    public void WetGripFactor_DecreasesWithSpeed()
    {
        float lowSpeed = TyreModel.ComputeWetGripFactor(0.003f, 0.5f, 10f);
        float highSpeed = TyreModel.ComputeWetGripFactor(0.003f, 0.5f, 30f);

        Assert.True(highSpeed < lowSpeed);
    }

    [Fact]
    public void WetGripFactor_HydroplaningCollapsesGrip()
    {
        // Deep water at high speed should approach minimum grip
        float factor = TyreModel.ComputeWetGripFactor(0.004f, 0.3f, 40f);

        Assert.True(factor < 0.2f);
        Assert.True(factor >= 0.05f); // never below minimum
    }

    [Fact]
    public void WetGripFactor_NeverBelowMinimum()
    {
        // Extreme conditions: deepest water, highest speed, no drainage
        float factor = TyreModel.ComputeWetGripFactor(0.01f, 0f, 100f);

        Assert.True(factor >= 0.05f);
    }

    // ── Microtexture / macrotexture tests ────────────────────────────────────

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
            NoiseFactor = 0f,
        };

        var lowMacro = new SurfaceProperties
        {
            FrictionCoefficient = 1.0f,
            Microtexture = 0.3f,
            Macrotexture = 0.2f,
            WaterDepth = 0.002f,
            NoiseFactor = 0f,
        };

        float muHighMacro = model.ComputeEffectiveFriction(3000f, highMacro, 30f, 1.0f, absVx: 15f);
        float muLowMacro = model.ComputeEffectiveFriction(3000f, lowMacro, 30f, 1.0f, absVx: 15f);

        // Higher macrotexture drains water better → higher grip on wet surface
        Assert.True(muHighMacro > muLowMacro);
    }

    // ── WetTarmac surface integration tests ─────────────────────────────────

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
            Tarmac, 0.01f, out _, out float dryFy, out _);
        model.Update(ref wetState, longitudinalVelocity, lateralVelocity, 3000f, 0f, 0f, 0f,
            WetTarmac, 0.01f, out _, out float wetFy, out _);

        Assert.True(MathF.Abs(dryFy) > MathF.Abs(wetFy));
    }

    // ── Road noise grip perturbation tests ──────────────────────────────────

    [Fact]
    public void RoadNoiseGripFactor_ReturnsNearUnity()
    {
        // With very low noise, the factor should be close to 1.0
        float factor = TyreModel.ComputeRoadNoiseGripFactor(0.01f, 20f, 0.01f);

        Assert.InRange(factor, 0.99f, 1.01f);
    }

    [Fact]
    public void RoadNoiseGripFactor_BoundedByAmplitude()
    {
        // At full noise, the factor must stay within ±8% of 1.0
        for (int i = 0; i < 100; i++)
        {
            float factor = TyreModel.ComputeRoadNoiseGripFactor(1.0f, (float)i * 0.5f, 0.01f);
            Assert.InRange(factor, 0.92f - 0.001f, 1.08f + 0.001f);
        }
    }

    [Fact]
    public void RoadNoiseGripFactor_ReturnsUnityAtZeroSpeed()
    {
        // At zero speed, PSD excitation is negligible — should return 1.0
        float factor = TyreModel.ComputeRoadNoiseGripFactor(1.0f, 0f, 0.01f);

        Assert.Equal(1f, factor);
    }

    // ── Default surface calibration tests ────────────────────────────────────

    [Fact]
    public void SurfaceDefaults_WetTarmacHasWaterAndLowerFriction()
    {
        var wet = SurfaceProperties.ForType(SurfaceType.WetTarmac);
        var dry = SurfaceProperties.ForType(SurfaceType.Tarmac);

        Assert.True(wet.WaterDepth > 0f);
        Assert.Equal(0f, dry.WaterDepth);
        Assert.True(wet.FrictionCoefficient < dry.FrictionCoefficient);
    }

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

    [Fact]
    public void SurfaceDefaults_IceHasVeryLowTexture()
    {
        var ice = SurfaceProperties.ForType(SurfaceType.Ice);

        Assert.True(ice.Microtexture <= 0.1f);
        Assert.True(ice.Macrotexture <= 0.1f);
    }

    [Fact]
    public void SurfaceDefaults_GravelHasHighMacrotexture()
    {
        var gravel = SurfaceProperties.ForType(SurfaceType.Gravel);

        Assert.True(gravel.Macrotexture >= 0.8f);
    }
}
