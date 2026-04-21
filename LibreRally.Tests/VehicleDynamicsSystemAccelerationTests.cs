using LibreRally.Vehicle.Physics;
using Stride.Core.Mathematics;

namespace LibreRally.Tests;

public class VehicleDynamicsSystemAccelerationTests
{
    [Fact]
    public void EstimatePlanarAccelerationFromNetForce_ProjectsAlongChassisAxes()
    {
        var netForce = new Vector3(1200f, 0f, 2400f);
        var forward = Vector3.UnitZ;
        var right = Vector3.UnitX;

        var acceleration = VehicleDynamicsSystem.EstimatePlanarAccelerationFromNetForce(
            netForce,
            forward,
            right,
            vehicleMass: 1200f);

        Assert.Equal(2f, acceleration.X, 4);
        Assert.Equal(1f, acceleration.Y, 4);
    }

    [Fact]
    public void EstimatePlanarAccelerationFromNetForce_HandlesRotatedAxes()
    {
        var forward = Vector3.Normalize(new Vector3(1f, 0f, 1f));
        var right = Vector3.Normalize(new Vector3(1f, 0f, -1f));
        var netForce = forward * 3000f + right * 1500f;

        var acceleration = VehicleDynamicsSystem.EstimatePlanarAccelerationFromNetForce(
            netForce,
            forward,
            right,
            vehicleMass: 1500f);

        Assert.Equal(2f, acceleration.X, 4);
        Assert.Equal(1f, acceleration.Y, 4);
    }

    [Fact]
    public void EstimatePlanarAccelerationFromNetForce_NearZeroMassReturnsZero()
    {
        var acceleration = VehicleDynamicsSystem.EstimatePlanarAccelerationFromNetForce(
            new Vector3(1000f, 0f, 1000f),
            Vector3.UnitZ,
            Vector3.UnitX,
            vehicleMass: 0f);

        Assert.Equal(Vector2.Zero, acceleration);
    }

    [Fact]
    public void EstimateWorldAccelerationFromNetForce_NearZeroMassReturnsZero()
    {
        var acceleration = VehicleDynamicsSystem.EstimateWorldAccelerationFromNetForce(
            new Vector3(1000f, 0f, 1000f),
            vehicleMass: 0f);

        Assert.Equal(Vector3.Zero, acceleration);
    }

    [Fact]
    public void PlanarProjection_ReprojectsSameWorldAccelerationAcrossFrameRotation()
    {
        var netForce = new Vector3(1000f, 0f, 0f);
        var worldAcceleration = VehicleDynamicsSystem.EstimateWorldAccelerationFromNetForce(
            netForce,
            vehicleMass: 1000f);

        var step1 = VehicleDynamicsSystem.EstimatePlanarAccelerationFromNetForce(
            netForce,
            Vector3.UnitZ,
            Vector3.UnitX,
            vehicleMass: 1000f);

        var step2Forward = Vector3.UnitX;
        var step2Right = -Vector3.UnitZ;
        var step2 = new Vector2(
            Vector3.Dot(worldAcceleration, step2Forward),
            Vector3.Dot(worldAcceleration, step2Right));

        Assert.Equal(0f, step1.X, 4);
        Assert.Equal(1f, step1.Y, 4);
        Assert.Equal(1f, step2.X, 4);
        Assert.Equal(0f, step2.Y, 4);
    }
}
