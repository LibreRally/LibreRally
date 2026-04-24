using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the differential and powertrain behavior.
	/// </summary>
	public sealed class DifferentialAndPowertrainTests
	{
		/// <summary>
		/// Verifies that resolve maps lsd preload and coast lock coefficients.
		/// </summary>
		[Fact]
		public void Resolve_MapsLsdPreloadAndCoastLockCoefficients()
		{
			var definition = new VehicleDefinition
			{
				PowertrainDevices = new List<JBeamPowertrainDevice>
				{
					new("engine", "mainEngine", "", 0, "", null, "", null, null, null), new("differential", "rearDiff", "mainEngine", 0, "", null, "lsd", 140f, 0.25f, 0.45f), new("wheelaxle", "wheelaxleRL", "rearDiff", 0, "RL", null, "", null, null, null), new("wheelaxle", "wheelaxleRR", "rearDiff", 0, "RR", null, "", null, null, null),
				},
			};

			var setup = VehiclePowertrainResolver.Resolve(definition);

			Assert.Equal(DifferentialType.LimitedSlip, setup.RearDiff.Type);
			Assert.Equal(0.25f, setup.RearDiff.LockingCoefficient, 3);
			Assert.Equal(0.45f, setup.RearDiff.CoastLockingCoefficient, 3);
			Assert.Equal(140f, setup.RearDiff.PreloadTorque, 3);
		}

		/// <summary>
		/// Verifies that split torque uses preload and higher coast lock.
		/// </summary>
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

		/// <summary>
		/// Verifies that split torque with coast lock set to zero does not fallback to drive lock.
		/// </summary>
		[Fact]
		public void SplitTorque_WithCoastLockSetToZero_DoesNotFallbackToDriveLock()
		{
			var config = DifferentialConfig.CreateLimitedSlip(
				biasRatio: 10f,
				lockingCoeff: 0.9f,
				coastLockingCoeff: 0f,
				preloadTorque: 0f);

			DifferentialSolver.SplitTorque(in config, -300f, omegaLeft: 20f, omegaRight: 5f, out var coastLeft, out var coastRight);

			var coastTransfer = (coastRight - coastLeft) * 0.5f;
			Assert.Equal(0f, coastTransfer, 4);
		}

		/// <summary>
		/// Verifies that split torque with preload ramps continuously at small delta omega.
		/// </summary>
		[Fact]
		public void SplitTorque_WithPreload_RampsContinuouslyAtSmallDeltaOmega()
		{
			var config = DifferentialConfig.CreateLimitedSlip(
				biasRatio: 10f,
				lockingCoeff: 0f,
				coastLockingCoeff: 0f,
				preloadTorque: 120f);

			DifferentialSolver.SplitTorque(in config, 0f, omegaLeft: 1.001f, omegaRight: 1f, out var smallDeltaLeft, out var smallDeltaRight);
			DifferentialSolver.SplitTorque(in config, 0f, omegaLeft: 1.02f, omegaRight: 1f, out var largerDeltaLeft, out var largerDeltaRight);

			var nearZeroTransfer = (smallDeltaRight - smallDeltaLeft) * 0.5f;
			var largerDeltaTransfer = (largerDeltaRight - largerDeltaLeft) * 0.5f;

			Assert.True(nearZeroTransfer > 0f);
			Assert.True(largerDeltaTransfer > nearZeroTransfer);
			Assert.True(largerDeltaTransfer < 120f);
		}

		/// <summary>
		/// Verifies that split torque open differential limits both outputs to least traction wheel.
		/// </summary>
		[Fact]
		public void SplitTorque_OpenDifferential_LimitsBothOutputsToLeastTractionWheel()
		{
			var config = DifferentialConfig.CreateOpen();

			DifferentialSolver.SplitTorque(
				in config,
				300f,
				omegaLeft: 12f,
				omegaRight: 18f,
				tractionLimitLeft: 140f,
				tractionLimitRight: 60f,
				out var torqueLeft,
				out var torqueRight);

			Assert.Equal(60f, torqueLeft, 3);
			Assert.Equal(60f, torqueRight, 3);
		}

		/// <summary>
		/// Verifies that split torque limited slip clamps bias and conserves delivered torque.
		/// </summary>
		[Fact]
		public void SplitTorque_LimitedSlip_ClampsBiasAndConservesDeliveredTorque()
		{
			var config = DifferentialConfig.CreateLimitedSlip(
				biasRatio: 3f,
				lockingCoeff: 0.85f,
				coastLockingCoeff: 0.4f,
				preloadTorque: 20f);

			DifferentialSolver.SplitTorque(
				in config,
				400f,
				omegaLeft: 24f,
				omegaRight: 8f,
				tractionLimitLeft: 70f,
				tractionLimitRight: 400f,
				out var torqueLeft,
				out var torqueRight);

			Assert.Equal(280f, torqueLeft + torqueRight, 3);
			Assert.True(torqueLeft >= 0f);
			Assert.True(torqueRight >= 0f);
			Assert.True(torqueRight / torqueLeft <= 3.0001f);
		}

		/// <summary>
		/// Verifies that resolve final drive uses selected final drive part ratio when intermediate devices stay at unit ratio.
		/// </summary>
		[Fact]
		public void ResolveFinalDrive_UsesSelectedFinalDrivePartRatioWhenIntermediateDevicesStayAtUnitRatio()
		{
			var definition = new VehicleDefinition
			{
				PowertrainDevices = new List<JBeamPowertrainDevice>
				{
					new("engine", "mainEngine", "", 0, "", null, "", null, null, null), new("differential", "rearDiff", "mainEngine", 0, "", 1f, "open", null, null, null), new("wheelaxle", "wheelaxleRL", "rearDiff", 0, "RL", null, "", null, null, null), new("wheelaxle", "wheelaxleRR", "rearDiff", 0, "RR", null, "", null, null, null),
				},
				PartGearRatios = new List<AssembledPartGearRatio> { new("fgx_finaldrive_R_391", "fgx_finaldrive_R", 3.91f), },
			};

			var setup = VehiclePowertrainResolver.Resolve(definition);

			Assert.False(setup.DriveFrontAxle);
			Assert.True(setup.DriveRearAxle);
			Assert.Equal(3.91f, setup.FinalDrive, 3);
		}
	}
}
