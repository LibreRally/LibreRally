using LibreRally.Vehicle.Physics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the vehicle physics builder behavior.
	/// </summary>
	public class VehiclePhysicsBuilderTests
	{
		/// <summary>
		/// Verifies that compute suspension travel keeps generous landing headroom for typical rally loads.
		/// </summary>
		[Fact]
		public void ComputeSuspensionTravel_KeepsGenerousLandingHeadroomForTypicalRallyLoads()
		{
			const float staticNormalLoad = 2800f;

			var frontBump = VehiclePhysicsBuilder.ComputeBumpTravel(staticNormalLoad, 60000f);
			var frontRebound = VehiclePhysicsBuilder.ComputeReboundTravel(staticNormalLoad, 60000f);
			var rearBump = VehiclePhysicsBuilder.ComputeBumpTravel(staticNormalLoad, 50000f);
			var rearRebound = VehiclePhysicsBuilder.ComputeReboundTravel(staticNormalLoad, 50000f);

			Assert.Equal(0.18f, frontBump, 3);
			Assert.Equal(0.16f, frontRebound, 3);
			Assert.Equal(0.18f, rearBump, 3);
			Assert.Equal(0.16f, rearRebound, 3);
		}

		/// <summary>
		/// Verifies that compute suspension travel scales from static sag before hitting clamps.
		/// </summary>
		[Fact]
		public void ComputeSuspensionTravel_ScalesFromStaticSagBeforeHittingClamps()
		{
			const float staticNormalLoad = 1200f;

			var bumpTravel = VehiclePhysicsBuilder.ComputeBumpTravel(staticNormalLoad, 60000f);
			var reboundTravel = VehiclePhysicsBuilder.ComputeReboundTravel(staticNormalLoad, 60000f);

			Assert.Equal(0.12f, bumpTravel, 3);
			Assert.Equal(0.08f, reboundTravel, 3);
		}
	}
}
