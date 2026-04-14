using System.Collections.Generic;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using Xunit;

namespace LibreRally.Tests;

public sealed class DifferentialAndPowertrainTests
{
    [Fact]
    public void Resolve_MapsLsdPreloadAndCoastLockCoefficients()
    {
        var definition = new VehicleDefinition
        {
            PowertrainDevices = new List<JBeamPowertrainDevice>
            {
                new("engine", "mainEngine", "", 0, "", null, "", null, null, null),
                new("differential", "rearDiff", "mainEngine", 0, "", null, "lsd", 140f, 0.25f, 0.45f),
                new("wheelaxle", "wheelaxleRL", "rearDiff", 0, "RL", null, "", null, null, null),
                new("wheelaxle", "wheelaxleRR", "rearDiff", 0, "RR", null, "", null, null, null),
            },
        };

        var setup = VehiclePowertrainResolver.Resolve(definition);

        Assert.Equal(DifferentialType.LimitedSlip, setup.RearDiff.Type);
        Assert.Equal(0.25f, setup.RearDiff.LockingCoefficient, 3);
        Assert.Equal(0.45f, setup.RearDiff.CoastLockingCoefficient, 3);
        Assert.Equal(140f, setup.RearDiff.PreloadTorque, 3);
    }

    [Fact]
    public void SplitTorque_UsesPreloadAndHigherCoastLock()
    {
        var config = DifferentialConfig.CreateLimitedSlip(
            biasRatio: 10f,
            lockingCoeff: 0.1f,
            coastLockingCoeff: 0.8f,
            preloadTorque: 40f);

        DifferentialSolver.SplitTorque(in config, 200f, omegaLeft: 20f, omegaRight: 5f, out var driveLeft, out var driveRight);
        DifferentialSolver.SplitTorque(in config, -200f, omegaLeft: 20f, omegaRight: 5f, out var coastLeft, out var coastRight);

        var driveTransfer = (driveRight - driveLeft) * 0.5f;
        var coastTransfer = (coastRight - coastLeft) * 0.5f;

        Assert.True(driveTransfer > 40f);
        Assert.True(coastTransfer > driveTransfer);
    }
}
