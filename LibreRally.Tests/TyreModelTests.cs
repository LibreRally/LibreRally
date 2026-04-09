using LibreRally.Vehicle.Physics;

namespace LibreRally.Tests;

public class TyreModelTests
{
    private static readonly SurfaceProperties Tarmac = SurfaceProperties.ForType(SurfaceType.Tarmac);

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

    // ── Pneumatic trail (brush model, The Contact Patch C2015) ──────────────

    [Fact]
    public void PneumaticTrailFactor_IsOneAtZeroSlip()
    {
        var model = new TyreModel(0.305f) { Width = 0.205f, TyrePressure = 220f };
        float brushStiffness = model.ComputeEffectiveBrushStiffness();
        float factor = model.ComputePneumaticTrailFactor(0f, 3000f, 1.0f, brushStiffness);
        Assert.Equal(1f, factor, 5);
    }

    [Fact]
    public void PneumaticTrailFactor_DecreasesMonotonicallyWithSlipAngle()
    {
        var model = new TyreModel(0.305f) { Width = 0.205f, TyrePressure = 220f };
        float brushStiffness = model.ComputeEffectiveBrushStiffness();
        const float normalLoad = 3000f;
        const float mu = 1.0f;

        float prev = 1f;
        foreach (float alpha in new[] { 0.05f, 0.10f, 0.20f, 0.35f, 0.60f, 0.90f, 1.10f })
        {
            float factor = model.ComputePneumaticTrailFactor(alpha, normalLoad, mu, brushStiffness);
            Assert.True(factor <= prev + 1e-5f,
                $"Trail factor should be non-increasing; got {factor} after {prev} at α={alpha} rad");
            prev = factor;
        }
    }

    [Fact]
    public void PneumaticTrailFactor_IsZeroAtFullSlide()
    {
        var model = new TyreModel(0.305f) { Width = 0.205f, TyrePressure = 220f };
        float brushStiffness = model.ComputeEffectiveBrushStiffness();
        // λ = 0 at extreme slip → factor should reach zero
        float factor = model.ComputePneumaticTrailFactor(1.2f, 3000f, 1.0f, brushStiffness);
        Assert.True(factor >= 0f);
        Assert.True(factor < 0.05f, $"Trail factor should be near zero at extreme slip; got {factor}");
    }

    [Fact]
    public void SelfAligningTorque_PeaksThenFallsWithSlipAngle()
    {
        // Article (C2015 Fig. 9): self-aligning moment rises to a peak then declines.
        // Verify the computed Mz follows this shape over a sweep of lateral velocities.
        var model = new TyreModel(0.305f)
        {
            PeakFrictionCoefficient = 1.0f,
            LoadSensitivity = 0f,
            ContactAreaGripExponent = 0f,
            OptimalTemperature = 30f,
            TemperatureWindow = 100f,
            WornGripFraction = 1.0f,
            RollingResistanceCoefficient = 0f,
        };

        const float normalLoad = 3000f;
        const float longitudinalVelocity = 20f;
        float maxMz = 0f;
        float lastMz = 0f;
        bool sawPeakThenFall = false;

        foreach (float lateralVelocity in new[] { 0.2f, 0.5f, 1.0f, 2.0f, 4.0f, 7.0f, 12.0f, 18.0f })
        {
            var state = TyreState.CreateDefault();
            state.AngularVelocity = longitudinalVelocity / model.Radius;
            model.Update(ref state, longitudinalVelocity, lateralVelocity, normalLoad, 0f, 0f, 0f,
                Tarmac, 0.01f, out _, out _, out float mz);

            float absMz = MathF.Abs(mz);
            if (absMz > maxMz)
                maxMz = absMz;
            else if (maxMz > 0f && absMz < maxMz * 0.8f)
                sawPeakThenFall = true;

            lastMz = absMz;
        }

        Assert.True(sawPeakThenFall,
            "Self-aligning torque should peak and then fall at high slip angles (article C2015 Fig. 9)");
    }

    [Fact]
    public void SelfAligningTorque_ScalesWithPneumaticTrailProperty()
    {
        var baseModel = new TyreModel(0.305f)
        {
            PeakFrictionCoefficient = 1.0f, LoadSensitivity = 0f,
            ContactAreaGripExponent = 0f, PneumaticTrail = 1.0f,
            OptimalTemperature = 30f, TemperatureWindow = 100f,
            WornGripFraction = 1.0f, RollingResistanceCoefficient = 0f,
        };
        var doubleModel = new TyreModel(0.305f)
        {
            PeakFrictionCoefficient = 1.0f, LoadSensitivity = 0f,
            ContactAreaGripExponent = 0f, PneumaticTrail = 2.0f,
            OptimalTemperature = 30f, TemperatureWindow = 100f,
            WornGripFraction = 1.0f, RollingResistanceCoefficient = 0f,
        };

        var state1 = TyreState.CreateDefault();
        var state2 = TyreState.CreateDefault();
        float lv = 20f, latV = 1.5f;
        state1.AngularVelocity = lv / baseModel.Radius;
        state2.AngularVelocity = lv / doubleModel.Radius;

        baseModel.Update(ref state1, lv, latV, 3000f, 0f, 0f, 0f, Tarmac, 0.01f, out _, out _, out float mz1);
        doubleModel.Update(ref state2, lv, latV, 3000f, 0f, 0f, 0f, Tarmac, 0.01f, out _, out _, out float mz2);

        Assert.True(MathF.Abs(mz2) > MathF.Abs(mz1),
            "PneumaticTrail=2.0 should produce larger self-aligning torque than 1.0");
        Assert.Equal(MathF.Abs(mz1), MathF.Abs(mz2) / 2f, 3);
    }
}
