using System;
using Stride.Core.Mathematics;

namespace LibreRally.Vehicle.Physics;

internal readonly record struct VehicleDynamicsGeometry(
    float Wheelbase,
    float FrontTrackWidth,
    float RearTrackWidth,
    float AverageTrackWidth,
    float CgToFrontAxle,
    float CgToRearAxle)
{
    public float FrontAxleLoadFraction => Wheelbase > 1e-3f ? Math.Clamp(CgToRearAxle / Wheelbase, 0.05f, 0.95f) : 0.5f;
    public float RearAxleLoadFraction => Wheelbase > 1e-3f ? Math.Clamp(CgToFrontAxle / Wheelbase, 0.05f, 0.95f) : 0.5f;

    public float FrontStaticWheelLoad(float vehicleMassKg) => MathF.Max(0f, vehicleMassKg) * 9.81f * FrontAxleLoadFraction * 0.5f;
    public float RearStaticWheelLoad(float vehicleMassKg) => MathF.Max(0f, vehicleMassKg) * 9.81f * RearAxleLoadFraction * 0.5f;
}

internal static class VehicleDynamicsGeometryResolver
{
    private const float DefaultWheelbase = 2.55f;
    private const float DefaultTrackWidth = 1.50f;

    public static VehicleDynamicsGeometry Resolve(
        Vector3 wheelFl,
        Vector3 wheelFr,
        Vector3 wheelRl,
        Vector3 wheelRr,
        Vector3 centerOfMass)
    {
        var frontAxleCenter = (wheelFl + wheelFr) * 0.5f;
        var rearAxleCenter = (wheelRl + wheelRr) * 0.5f;
        var rearToFront = frontAxleCenter - rearAxleCenter;
        var wheelbase = MathF.Abs(rearToFront.Z);
        if (wheelbase <= 0.5f)
        {
            wheelbase = DefaultWheelbase;
        }

        var frontTrack = MathF.Abs(wheelFl.X - wheelFr.X);
        if (frontTrack <= 0.5f)
        {
            frontTrack = DefaultTrackWidth;
        }

        var rearTrack = MathF.Abs(wheelRl.X - wheelRr.X);
        if (rearTrack <= 0.5f)
        {
            rearTrack = frontTrack > 0.5f ? frontTrack : DefaultTrackWidth;
        }

        var axleVectorLengthSquared = rearToFront.LengthSquared();
        float rearToCgDistance;
        if (axleVectorLengthSquared > 1e-4f)
        {
            var projection = Vector3.Dot(centerOfMass - rearAxleCenter, rearToFront) / axleVectorLengthSquared;
            rearToCgDistance = Math.Clamp(projection, 0.05f, 0.95f) * wheelbase;
        }
        else
        {
            rearToCgDistance = wheelbase * 0.5f;
        }

        var cgToFrontDistance = Math.Max(0.05f, wheelbase - rearToCgDistance);
        return new VehicleDynamicsGeometry(
            Wheelbase: wheelbase,
            FrontTrackWidth: frontTrack,
            RearTrackWidth: rearTrack,
            AverageTrackWidth: (frontTrack + rearTrack) * 0.5f,
            CgToFrontAxle: cgToFrontDistance,
            CgToRearAxle: Math.Max(0.05f, rearToCgDistance));
    }
}
