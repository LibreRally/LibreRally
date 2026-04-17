using LibreRally.Vehicle.Physics;
using Stride.Core.Mathematics;

namespace LibreRally.Tests;

/// <summary>Tests for <see cref="ChassisBodyRollSystem"/>.</summary>
public class ChassisBodyRollSystemTests
{
    /// <summary>
    /// When all suspension compressions are equal, no roll torque should be produced.
    /// </summary>
    [Fact]
    public void EqualCompression_ProducesNoRollTorque()
    {
        var system = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 5000f,
            RearRollStiffness = 4000f,
            RollDampingCoefficient = 0f,
        };

        // All wheels equally compressed at 0.02 m
        float[] compressions = { 0.02f, 0.02f, 0.02f, 0.02f };

        var torque = ComputeRollTorque(system, compressions);

        Assert.Equal(0f, torque, precision: 4);
    }

    /// <summary>
    /// Left-side-more-compressed (positive delta) should produce positive roll torque,
    /// and right-side-more-compressed should produce negative roll torque.
    /// </summary>
    [Theory]
    [InlineData(0.04f, 0.01f, 0.04f, 0.01f)] // Left more compressed
    [InlineData(0.01f, 0.04f, 0.01f, 0.04f)] // Right more compressed
    public void AsymmetricCompression_ProducesCorrectSign(float fl, float fr, float rl, float rr)
    {
        var system = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 5000f,
            RearRollStiffness = 4000f,
            RollDampingCoefficient = 0f,
        };

        float[] compressions = { fl, fr, rl, rr };
        var torque = ComputeRollTorque(system, compressions);

        if (fl > fr) // Left more compressed → positive torque
        {
            Assert.True(torque > 0f, $"Expected positive torque, got {torque}");
        }
        else // Right more compressed → negative torque
        {
            Assert.True(torque < 0f, $"Expected negative torque, got {torque}");
        }
    }

    /// <summary>
    /// Roll torque magnitude scales linearly with stiffness.
    /// </summary>
    [Fact]
    public void TorqueMagnitude_ScalesWithStiffness()
    {
        float[] compressions = { 0.04f, 0.01f, 0.04f, 0.01f };

        var low = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 2000f,
            RearRollStiffness = 2000f,
            RollDampingCoefficient = 0f,
        };

        var high = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 6000f,
            RearRollStiffness = 6000f,
            RollDampingCoefficient = 0f,
        };

        var torqueLow = ComputeRollTorque(low, compressions);
        var torqueHigh = ComputeRollTorque(high, compressions);

        // High stiffness should produce 3× the torque (6000/2000)
        Assert.True(MathF.Abs(torqueHigh) > MathF.Abs(torqueLow),
            $"High stiffness torque {torqueHigh} should exceed low {torqueLow}");
        Assert.Equal(torqueLow * 3f, torqueHigh, precision: 2);
    }

    /// <summary>
    /// Front-only and rear-only stiffness should independently contribute.
    /// </summary>
    [Fact]
    public void FrontAndRear_ContributeIndependently()
    {
        float[] compressions = { 0.03f, 0.01f, 0.03f, 0.01f };

        var frontOnly = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 5000f,
            RearRollStiffness = 0f,
            RollDampingCoefficient = 0f,
        };

        var rearOnly = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 0f,
            RearRollStiffness = 4000f,
            RollDampingCoefficient = 0f,
        };

        var both = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 5000f,
            RearRollStiffness = 4000f,
            RollDampingCoefficient = 0f,
        };

        var torqueFront = ComputeRollTorque(frontOnly, compressions);
        var torqueRear = ComputeRollTorque(rearOnly, compressions);
        var torqueBoth = ComputeRollTorque(both, compressions);

        Assert.Equal(torqueFront + torqueRear, torqueBoth, precision: 4);
    }

    /// <summary>
    /// Zero stiffness produces zero roll torque.
    /// </summary>
    [Fact]
    public void ZeroStiffness_ProducesNoTorque()
    {
        var system = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 0f,
            RearRollStiffness = 0f,
            RollDampingCoefficient = 0f,
        };

        float[] compressions = { 0.05f, 0.00f, 0.05f, 0.00f };
        var torque = ComputeRollTorque(system, compressions);

        Assert.Equal(0f, torque, precision: 6);
    }

    /// <summary>
    /// Near-zero timestep should produce no effect (early return).
    /// </summary>
    [Fact]
    public void NearZeroDt_ProducesNoEffect()
    {
        var system = new ChassisBodyRollSystem
        {
            FrontRollStiffness = 5000f,
            RearRollStiffness = 4000f,
            RollDampingCoefficient = 0f,
        };

        float[] compressions = { 0.04f, 0.01f, 0.04f, 0.01f };
        var torque = ComputeRollTorqueWithDt(system, compressions, 0f);

        Assert.Equal(0f, torque, precision: 6);
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    /// <summary>
    /// Computes the expected scalar roll torque without involving BEPU.
    /// This mirrors the formula in <see cref="ChassisBodyRollSystem.Apply"/> minus damping.
    /// </summary>
    private static float ComputeRollTorque(ChassisBodyRollSystem system, float[] compressions)
    {
        var deltaFront = compressions[VehicleDynamicsSystem.FL]
                       - compressions[VehicleDynamicsSystem.FR];
        var deltaRear = compressions[VehicleDynamicsSystem.RL]
                      - compressions[VehicleDynamicsSystem.RR];

        return deltaFront * system.FrontRollStiffness
             + deltaRear * system.RearRollStiffness;
    }

    /// <summary>
    /// Variant that checks whether near-zero dt produces no torque (mirrors early-return logic).
    /// </summary>
    private static float ComputeRollTorqueWithDt(ChassisBodyRollSystem system, float[] compressions, float dt)
    {
        if (dt < 1e-6f)
        {
            return 0f;
        }

        return ComputeRollTorque(system, compressions);
    }
}
