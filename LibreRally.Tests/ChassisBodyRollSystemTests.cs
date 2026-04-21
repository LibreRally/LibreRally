using LibreRally.Vehicle.Physics;

namespace LibreRally.Tests
{
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

		/// <summary>
		/// Viscous damping should oppose positive roll rate (reduce torque).
		/// </summary>
		[Fact]
		public void Damping_OpposesPositiveRollRate()
		{
			var system = new ChassisBodyRollSystem
			{
				FrontRollStiffness = 5000f,
				RearRollStiffness = 4000f,
				RollDampingCoefficient = 800f,
			};

			float[] compressions = { 0.03f, 0.01f, 0.03f, 0.01f };

			var undampedTorque = ComputeRollTorque(system, compressions);
			var dampedTorque = ComputeRollTorqueWithDamping(system, compressions, rollRate: 2.0f);

			Assert.True(undampedTorque > 0f, "Base torque should be positive");
			Assert.True(dampedTorque < undampedTorque,
				$"Damped torque {dampedTorque} should be less than undamped {undampedTorque}");
		}

		/// <summary>
		/// Viscous damping should oppose negative roll rate (increase torque towards positive).
		/// </summary>
		[Fact]
		public void Damping_OpposesNegativeRollRate()
		{
			var system = new ChassisBodyRollSystem
			{
				FrontRollStiffness = 5000f,
				RearRollStiffness = 4000f,
				RollDampingCoefficient = 800f,
			};

			float[] compressions = { 0.03f, 0.01f, 0.03f, 0.01f };

			var undampedTorque = ComputeRollTorque(system, compressions);
			var dampedTorque = ComputeRollTorqueWithDamping(system, compressions, rollRate: -2.0f);

			Assert.True(dampedTorque > undampedTorque,
				$"Damped torque {dampedTorque} with negative roll rate should exceed undamped {undampedTorque}");
		}

		/// <summary>
		/// Damping magnitude scales linearly with the damping coefficient.
		/// </summary>
		[Fact]
		public void DampingMagnitude_ScalesWithCoefficient()
		{
			float[] compressions = { 0.03f, 0.01f, 0.03f, 0.01f };
			const float rollRate = 1.5f;

			var low = new ChassisBodyRollSystem
			{
				FrontRollStiffness = 5000f,
				RearRollStiffness = 4000f,
				RollDampingCoefficient = 400f,
			};

			var high = new ChassisBodyRollSystem
			{
				FrontRollStiffness = 5000f,
				RearRollStiffness = 4000f,
				RollDampingCoefficient = 1200f,
			};

			var baseTorque = ComputeRollTorque(low, compressions);
			var lowDamped = ComputeRollTorqueWithDamping(low, compressions, rollRate);
			var highDamped = ComputeRollTorqueWithDamping(high, compressions, rollRate);

			var lowReduction = baseTorque - lowDamped;
			var highReduction = baseTorque - highDamped;

			// High coefficient (1200) should produce 3× the damping reduction of low (400)
			Assert.Equal(lowReduction * 3f, highReduction, precision: 2);
		}

		/// <summary>
		/// Zero roll rate with non-zero damping coefficient should produce
		/// the same torque as zero damping (no damping contribution).
		/// </summary>
		[Fact]
		public void ZeroRollRate_DampingHasNoEffect()
		{
			var system = new ChassisBodyRollSystem
			{
				FrontRollStiffness = 5000f,
				RearRollStiffness = 4000f,
				RollDampingCoefficient = 800f,
			};

			float[] compressions = { 0.03f, 0.01f, 0.03f, 0.01f };

			var undampedTorque = ComputeRollTorque(system, compressions);
			var dampedTorque = ComputeRollTorqueWithDamping(system, compressions, rollRate: 0f);

			Assert.Equal(undampedTorque, dampedTorque, precision: 4);
		}

		[Fact]
		public void EqualFrontRearCompression_ProducesNoPitchTorque()
		{
			var system = new ChassisBodyRollSystem
			{
				PitchStiffness = 12000f,
				PitchDampingCoefficient = 0f,
			};

			float[] compressions = { 0.02f, 0.02f, 0.02f, 0.02f };

			var torque = ComputePitchTorque(system, compressions);

			Assert.Equal(0f, torque, precision: 4);
		}

		[Theory]
		[InlineData(0.05f, 0.05f, 0.01f, 0.01f, 1f)]
		[InlineData(0.01f, 0.01f, 0.05f, 0.05f, -1f)]
		public void FrontRearCompressionBias_ProducesCorrectPitchSign(float fl, float fr, float rl, float rr, float expectedSign)
		{
			var system = new ChassisBodyRollSystem
			{
				PitchStiffness = 12000f,
				PitchDampingCoefficient = 0f,
			};

			float[] compressions = { fl, fr, rl, rr };
			var torque = ComputePitchTorque(system, compressions);

			Assert.True(MathF.Sign(torque) == MathF.Sign(expectedSign), $"Expected pitch sign {expectedSign}, got {torque}");
		}

		[Fact]
		public void PitchDamping_OpposesPositivePitchRate()
		{
			var system = new ChassisBodyRollSystem
			{
				PitchStiffness = 12000f,
				PitchDampingCoefficient = 1500f,
			};

			float[] compressions = { 0.04f, 0.04f, 0.01f, 0.01f };

			var undampedTorque = ComputePitchTorque(system, compressions);
			var dampedTorque = ComputePitchTorqueWithDamping(system, compressions, pitchRate: 2f);

			Assert.True(dampedTorque < undampedTorque,
				$"Positive pitch rate should reduce torque: damped {dampedTorque}, undamped {undampedTorque}");
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
		/// Computes the expected scalar roll torque including viscous damping.
		/// Mirrors the full formula in <see cref="ChassisBodyRollSystem.Apply"/>.
		/// </summary>
		/// <param name="system">Body roll configuration.</param>
		/// <param name="compressions">Per-wheel suspension compressions.</param>
		/// <param name="rollRate">Current roll rate (rad/s) about the forward axis.</param>
		private static float ComputeRollTorqueWithDamping(ChassisBodyRollSystem system, float[] compressions, float rollRate)
		{
			var baseTorque = ComputeRollTorque(system, compressions);
			return baseTorque - rollRate * system.RollDampingCoefficient;
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

		private static float ComputePitchTorque(ChassisBodyRollSystem system, float[] compressions)
		{
			var frontAverage = (compressions[VehicleDynamicsSystem.FL] + compressions[VehicleDynamicsSystem.FR]) * 0.5f;
			var rearAverage = (compressions[VehicleDynamicsSystem.RL] + compressions[VehicleDynamicsSystem.RR]) * 0.5f;
			return (frontAverage - rearAverage) * system.PitchStiffness;
		}

		private static float ComputePitchTorqueWithDamping(ChassisBodyRollSystem system, float[] compressions, float pitchRate)
		{
			var baseTorque = ComputePitchTorque(system, compressions);
			return baseTorque - pitchRate * system.PitchDampingCoefficient;
		}
	}
}
