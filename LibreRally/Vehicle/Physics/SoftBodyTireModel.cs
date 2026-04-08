using System;
using Stride.Core;

namespace LibreRally.Vehicle.Physics;

[DataContract]
public sealed class SoftBodyTireModel
{
    private const float ReferenceWheelLoad = 3000f;
    private const float DegreesPerRadian = 180f / MathF.PI;

    public SoftBodyTireModel(float wheelRadius)
    {
        WheelRadius = MathF.Max(0.1f, wheelRadius);
        MaxDeflection = WheelRadius * 0.12f;
    }

    public float WheelRadius { get; }
    public float PeakSlipAngleDegrees { get; set; } = 12f;
    public float PeakFrictionCoefficient { get; set; } = 0.67f;
    public float SlidingFrictionCoefficient { get; set; } = 0.42f;
    public float MaxSlipAngleDegrees { get; set; } = 30f;
    public float ContactPatchStiffness { get; set; } = 65000f;
    public float ContactPatchDamping { get; set; } = 4500f;
    public float RelaxationLength { get; set; } = 1.35f;
    public float MaxDeflection { get; set; }

    public float LateralDeflection { get; private set; }
    public float LastSlipAngleDegrees { get; private set; }
    public float LastLateralForce { get; private set; }

    public float EvaluateLateralForce(float lateralVelocity, float longitudinalVelocity, float normalLoad, float dt, float lowSpeedGrip)
    {
        if (dt < 1e-4f || normalLoad <= 0f)
        {
            ResetTransientState();
            return 0f;
        }

        float absLongitudinalVelocity = MathF.Abs(longitudinalVelocity);
        float slipAngle = MathF.Atan2(lateralVelocity, MathF.Max(absLongitudinalVelocity, 0.5f));
        float slipAngleDegrees = MathF.Abs(slipAngle) * DegreesPerRadian;
        LastSlipAngleDegrees = slipAngleDegrees;

        float previousDeflection = LateralDeflection;
        float relaxationSpeed = MathF.Max(absLongitudinalVelocity, 2f);
        float deflectionRate = lateralVelocity - (LateralDeflection * relaxationSpeed / MathF.Max(RelaxationLength, 0.05f));
        LateralDeflection += deflectionRate * dt;

        float loadScale = Math.Clamp(normalLoad / MathF.Max(ReferenceWheelLoad, 1f), 0.8f, 1.25f);
        LateralDeflection = Math.Clamp(LateralDeflection, -MaxDeflection * loadScale, MaxDeflection * loadScale);

        float deflectionVelocity = (LateralDeflection - previousDeflection) / dt;
        float softBodyForce = (-ContactPatchStiffness * LateralDeflection) - (ContactPatchDamping * deflectionVelocity);
        float gripEnvelope = EvaluateGripEnvelope(slipAngleDegrees) * normalLoad;
        float lowSpeedForce = -lateralVelocity * normalLoad * MathF.Max(0f, lowSpeedGrip) * 0.15f;

        float lateralForce = absLongitudinalVelocity < 1f
            ? lowSpeedForce
            : softBodyForce;

        LastLateralForce = Math.Clamp(lateralForce, -gripEnvelope, gripEnvelope);
        return LastLateralForce;
    }

    public void ResetTransientState()
    {
        LateralDeflection = 0f;
        LastSlipAngleDegrees = 0f;
        LastLateralForce = 0f;
    }

    private float EvaluateGripEnvelope(float slipAngleDegrees)
    {
        float peakSlip = Math.Clamp(PeakSlipAngleDegrees, 0.01f, MathF.Max(MaxSlipAngleDegrees, 0.01f));
        float maxSlip = MathF.Max(MaxSlipAngleDegrees, peakSlip);
        float limitedSlip = Math.Clamp(slipAngleDegrees, 0f, maxSlip);
        if (limitedSlip <= peakSlip)
            return PeakFrictionCoefficient * (limitedSlip / peakSlip);

        float tailRange = MathF.Max(maxSlip - peakSlip, 0.01f);
        float tailT = Math.Clamp((limitedSlip - peakSlip) / tailRange, 0f, 1f);
        return PeakFrictionCoefficient + (SlidingFrictionCoefficient - PeakFrictionCoefficient) * tailT;
    }
}
