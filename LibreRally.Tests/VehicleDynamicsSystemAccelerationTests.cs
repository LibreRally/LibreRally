using LibreRally.Vehicle.Physics;
using Stride.Core.Mathematics;

namespace LibreRally.Tests
{
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

		[Fact]
		public void EffectivePitchTransferHeight_AppliesAntiDiveAndAntiSquat()
		{
			var brakingHeight = VehicleDynamicsSystem.ComputeEffectivePitchTransferHeight(
				cgHeight: 0.5f,
				longitudinalAccel: -4f,
				antiDiveFactor: 0.2f,
				antiSquatFactor: 0.4f);
			var accelerationHeight = VehicleDynamicsSystem.ComputeEffectivePitchTransferHeight(
				cgHeight: 0.5f,
				longitudinalAccel: 4f,
				antiDiveFactor: 0.2f,
				antiSquatFactor: 0.4f);

			Assert.Equal(0.4f, brakingHeight, 4);
			Assert.Equal(0.3f, accelerationHeight, 4);
		}

		[Fact]
		public void AxleLateralTransfer_DecreasesAsRollCenterRises()
		{
			var lowRollCenterTransfer = VehicleDynamicsSystem.ComputeAxleLateralTransfer(
				vehicleMass: 1200f,
				axleLoadFraction: 0.55f,
				lateralAccel: 5f,
				cgHeight: 0.5f,
				rollCenterHeight: 0.08f,
				trackWidth: 1.5f);
			var highRollCenterTransfer = VehicleDynamicsSystem.ComputeAxleLateralTransfer(
				vehicleMass: 1200f,
				axleLoadFraction: 0.55f,
				lateralAccel: 5f,
				cgHeight: 0.5f,
				rollCenterHeight: 0.22f,
				trackWidth: 1.5f);

			Assert.True(highRollCenterTransfer < lowRollCenterTransfer);
		}

		[Fact]
		public void ComputeWheelMomentImpulseWorld_ProjectsLocalMomentsOntoWheelAxes()
		{
			var wheelForward = Vector3.UnitZ;
			var wheelRight = Vector3.UnitX;

			var impulse = VehicleDynamicsSystem.ComputeWheelMomentImpulseWorld(
				wheelForward,
				wheelRight,
				overturningCouple: 12f,
				rollingResistanceMoment: -8f,
				dt: 0.02f);

			Assert.Equal(-0.16f, impulse.X, 4);
			Assert.Equal(0f, impulse.Y, 4);
			Assert.Equal(0.24f, impulse.Z, 4);
		}

		[Fact]
		public void ComputeWheelMomentImpulseWorld_FollowsRotatedWheelBasis()
		{
			var wheelForward = Vector3.Normalize(new Vector3(1f, 0f, 1f));
			var wheelRight = Vector3.Normalize(new Vector3(1f, 0f, -1f));

			var impulse = VehicleDynamicsSystem.ComputeWheelMomentImpulseWorld(
				wheelForward,
				wheelRight,
				overturningCouple: 10f,
				rollingResistanceMoment: 6f,
				dt: 0.05f);

			Assert.Equal(0.5656854f, impulse.X, 4);
			Assert.Equal(0f, impulse.Y, 4);
			Assert.Equal(0.14142136f, impulse.Z, 4);
		}
	}
}
