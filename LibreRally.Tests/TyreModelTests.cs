using LibreRally.Vehicle.Physics;

namespace LibreRally.Tests;

public class TyreModelTests
{
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
}
