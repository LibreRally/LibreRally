using LibreRally.Vehicle.Physics;
using Xunit;

namespace LibreRally.Tests;

public class VehicleDynamicsSystemTests
{
    [Fact]
    public void ResolveGeometry_ComputesFrontBiasedStaticWheelLoadsFromChassisCom()
    {
        var geometry = VehicleDynamicsGeometryResolver.Resolve(
            new Stride.Core.Mathematics.Vector3(-0.78f, 0.35f, 1.30f),
            new Stride.Core.Mathematics.Vector3(0.78f, 0.35f, 1.30f),
            new Stride.Core.Mathematics.Vector3(-0.76f, 0.32f, -1.20f),
            new Stride.Core.Mathematics.Vector3(0.76f, 0.32f, -1.20f),
            new Stride.Core.Mathematics.Vector3(0f, 0.55f, 0.18f));

        Assert.Equal(2.50f, geometry.Wheelbase, 3);
        Assert.Equal(1.56f, geometry.FrontTrackWidth, 3);
        Assert.Equal(1.52f, geometry.RearTrackWidth, 3);
        Assert.True(geometry.FrontAxleLoadFraction > 0.5f);
        Assert.True(geometry.FrontStaticWheelLoad(1500f) > geometry.RearStaticWheelLoad(1500f));
    }

    [Fact]
    public void ComputeAdditionalSupportLoads_UsesOnlyDynamicLoadDelta()
    {
        var dynamics = new VehicleDynamicsSystem();
        var contactScales = new[] { 1f, 1f, 1f, 1f };
        var deltas = new float[VehicleDynamicsSystem.WheelCount];

        for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
        {
            dynamics.StaticNormalLoads[i] = 3000f;
            dynamics.CurrentNormalLoads[i] = 3000f;
            dynamics.WheelGrounded[i] = true;
        }

        dynamics.CurrentNormalLoads[VehicleDynamicsSystem.FL] = 3600f;
        dynamics.CurrentNormalLoads[VehicleDynamicsSystem.FR] = 2400f;

        dynamics.ComputeAdditionalSupportLoads(contactScales, deltas);

        Assert.Equal(600f, deltas[VehicleDynamicsSystem.FL], 3);
        Assert.Equal(-600f, deltas[VehicleDynamicsSystem.FR], 3);
        Assert.Equal(0f, deltas[VehicleDynamicsSystem.RL], 3);
        Assert.Equal(0f, deltas[VehicleDynamicsSystem.RR], 3);
    }

    [Fact]
    public void ComputeAdditionalSupportLoads_ScalesBaselineWithContactConfidence()
    {
        var dynamics = new VehicleDynamicsSystem();
        var deltas = new float[VehicleDynamicsSystem.WheelCount];
        var contactScales = new[] { 0.5f, 1f, 1f, 1f };

        for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
        {
            dynamics.StaticNormalLoads[i] = 3000f;
            dynamics.CurrentNormalLoads[i] = 3000f;
            dynamics.WheelGrounded[i] = true;
        }

        dynamics.CurrentNormalLoads[VehicleDynamicsSystem.FL] = 1500f;

        dynamics.ComputeAdditionalSupportLoads(contactScales, deltas);

        Assert.Equal(0f, deltas[VehicleDynamicsSystem.FL], 3);
        Assert.Equal(0f, deltas[VehicleDynamicsSystem.FR], 3);
        Assert.Equal(0f, deltas[VehicleDynamicsSystem.RL], 3);
        Assert.Equal(0f, deltas[VehicleDynamicsSystem.RR], 3);
    }
}
