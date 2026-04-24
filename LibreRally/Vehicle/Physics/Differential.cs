using System;

namespace LibreRally.Vehicle.Physics
{
	/// <summary>
	/// Differential type enumeration.
	/// Each type distributes engine torque differently between the two output shafts.
	/// Reference: Milliken &amp; Milliken, "Race Car Vehicle Dynamics", §6.4.
	/// </summary>
	public enum DifferentialType : byte
	{
		/// <summary>
		/// Open differential: equal torque split, allows unlimited speed difference.
		/// The faster-spinning wheel limits available traction — poor for rally.
		/// </summary>
		Open,

		/// <summary>
		/// Limited-slip differential: transfers torque based on wheel speed difference
		/// up to a bias ratio. Improves traction on mixed-grip surfaces.
		/// </summary>
		LimitedSlip,

		/// <summary>
		/// Locking differential: both wheels rotate at the same speed.
		/// Maximum traction on loose surfaces but reduces steering agility.
		/// </summary>
		Locking,
	}

	/// <summary>
	/// Configuration for a single differential unit.
	/// Used for front, rear, or center differentials in an AWD drivetrain.
	/// </summary>
	public struct DifferentialConfig
	{
		/// <summary>Type of differential behaviour.</summary>
		public DifferentialType Type;

		/// <summary>
		/// Bias ratio for limited-slip differentials (typically 1.5–4.0).
		/// Defines the maximum torque ratio between the two output shafts:
		///   T_slow / T_fast ≤ biasRatio.
		/// A value of 1.0 = open; higher = more locking tendency.
		/// Reference: Milliken, RCVD §6.4.2.
		/// </summary>
		public float BiasRatio;

		/// <summary>
		/// Locking coefficient for clutch-type LSDs (0.0–1.0).
		/// Controls how aggressively the LSD locks under acceleration.
		/// 0 = pure open behaviour, 1 = immediate full lock.
		/// </summary>
		public float LockingCoefficient;

		/// <summary>
		/// Locking coefficient used while coasting/engine braking (input torque &lt; 0).
		/// If zero, <see cref="LockingCoefficient"/> is used for both directions.
		/// </summary>
		public float CoastLockingCoefficient;

		/// <summary>
		/// Base clutch preload torque for limited-slip differentials (N·m).
		/// Applied as baseline locking before torque-proportional lock is added.
		/// </summary>
		public float PreloadTorque;

		/// <summary>Creates a default open differential.</summary>
		/// <returns>An open differential configuration.</returns>
		public static DifferentialConfig CreateOpen() => new()
		{
			Type = DifferentialType.Open,
			BiasRatio = 1.0f,
			LockingCoefficient = 0f,
			CoastLockingCoefficient = 0f,
			PreloadTorque = 0f,
		};

		/// <summary>Creates a default limited-slip differential with typical rally bias.</summary>
		/// <param name="biasRatio">Maximum torque bias ratio between the two outputs.</param>
		/// <param name="lockingCoeff">Locking coefficient under power.</param>
		/// <param name="coastLockingCoeff">Locking coefficient while coasting or engine braking.</param>
		/// <param name="preloadTorque">Preload torque applied before torque biasing.</param>
		/// <returns>A limited-slip differential configuration.</returns>
		public static DifferentialConfig CreateLimitedSlip(float biasRatio = 2.5f, float lockingCoeff = 0.3f, float coastLockingCoeff = 0f, float preloadTorque = 0f) => new()
		{
			Type = DifferentialType.LimitedSlip,
			BiasRatio = biasRatio,
			LockingCoefficient = lockingCoeff,
			CoastLockingCoefficient = coastLockingCoeff,
			PreloadTorque = preloadTorque,
		};

		/// <summary>Creates a locking differential.</summary>
		/// <returns>A fully locking differential configuration.</returns>
		public static DifferentialConfig CreateLocking() => new()
		{
			Type = DifferentialType.Locking,
			BiasRatio = float.MaxValue,
			LockingCoefficient = 1.0f,
			CoastLockingCoefficient = 1.0f,
			PreloadTorque = 0f,
		};
	}

	/// <summary>
	/// Stateless differential torque-split calculations.
	///
	/// Torque path: engine → clutch → gearbox → center diff → front/rear diff → wheels.
	///
	/// Each differential splits its input torque to two outputs based on
	/// wheel speed difference and the configured <see cref="DifferentialType"/>.
	///
	/// All methods are static and allocation-free for performance with 50+ vehicles.
	///
	/// Reference: Milliken &amp; Milliken, "Race Car Vehicle Dynamics", Chapter 6.
	/// </summary>
	public static class DifferentialSolver
	{
		private const float MinimumTorqueMagnitude = 1e-4f;
		private const float TractionEpsilon = 1e-3f;
		private const float LockingReferenceOmega = 5f;

		/// <summary>
		/// Splits <paramref name="inputTorque"/> into two output torques based on the
		/// differential configuration and the angular velocities of the two output shafts.
		/// </summary>
		/// <param name="config">Differential configuration.</param>
		/// <param name="inputTorque">Total torque entering the differential (N·m).</param>
		/// <param name="omegaLeft">Angular velocity of the left output shaft (rad/s).</param>
		/// <param name="omegaRight">Angular velocity of the right output shaft (rad/s).</param>
		/// <param name="torqueLeft">Output: torque delivered to the left shaft (N·m).</param>
		/// <param name="torqueRight">Output: torque delivered to the right shaft (N·m).</param>
		public static void SplitTorque(
			in DifferentialConfig config,
			float inputTorque,
			float omegaLeft,
			float omegaRight,
			out float torqueLeft,
			out float torqueRight)
		{
			SplitTorque(
				in config,
				inputTorque,
				omegaLeft,
				omegaRight,
				float.PositiveInfinity,
				float.PositiveInfinity,
				out torqueLeft,
				out torqueRight);
		}

		/// <summary>
		/// Splits <paramref name="inputTorque"/> into two output torques while respecting
		/// the available traction capacity of each output shaft.
		/// </summary>
		/// <param name="config">Differential configuration.</param>
		/// <param name="inputTorque">Total torque entering the differential (N·m).</param>
		/// <param name="omegaLeft">Angular velocity of the left output shaft (rad/s).</param>
		/// <param name="omegaRight">Angular velocity of the right output shaft (rad/s).</param>
		/// <param name="tractionLimitLeft">Maximum transmissible torque magnitude on the left output (N·m).</param>
		/// <param name="tractionLimitRight">Maximum transmissible torque magnitude on the right output (N·m).</param>
		/// <param name="torqueLeft">Output: torque delivered to the left shaft (N·m).</param>
		/// <param name="torqueRight">Output: torque delivered to the right shaft (N·m).</param>
		public static void SplitTorque(
			in DifferentialConfig config,
			float inputTorque,
			float omegaLeft,
			float omegaRight,
			float tractionLimitLeft,
			float tractionLimitRight,
			out float torqueLeft,
			out float torqueRight)
		{
			var canApplyPreloadAtZeroInput = config.Type == DifferentialType.LimitedSlip &&
			                                config.PreloadTorque > MinimumTorqueMagnitude;
			if (MathF.Abs(inputTorque) <= MinimumTorqueMagnitude && !canApplyPreloadAtZeroInput)
			{
				torqueLeft = 0f;
				torqueRight = 0f;
				return;
			}

			tractionLimitLeft = SanitizeTractionLimit(tractionLimitLeft);
			tractionLimitRight = SanitizeTractionLimit(tractionLimitRight);

			switch (config.Type)
			{
				case DifferentialType.Open:
					SplitOpen(inputTorque, tractionLimitLeft, tractionLimitRight, out torqueLeft, out torqueRight);
					break;

				case DifferentialType.LimitedSlip:
					SplitLimitedSlip(config, inputTorque, omegaLeft, omegaRight, tractionLimitLeft, tractionLimitRight,
						out torqueLeft, out torqueRight);
					break;

				case DifferentialType.Locking:
					SplitLocking(inputTorque, tractionLimitLeft, tractionLimitRight, out torqueLeft, out torqueRight);
					break;

				default:
					SplitOpen(inputTorque, tractionLimitLeft, tractionLimitRight, out torqueLeft, out torqueRight);
					break;
			}
		}

		internal static float ComputeMaximumDeliveredTorque(
			in DifferentialConfig config,
			float omegaLeft,
			float omegaRight,
			float tractionLimitLeft,
			float tractionLimitRight)
		{
			tractionLimitLeft = SanitizeTractionLimit(tractionLimitLeft);
			tractionLimitRight = SanitizeTractionLimit(tractionLimitRight);

			return config.Type switch
			{
				DifferentialType.Open or DifferentialType.Locking
					=> 2f * MathF.Min(tractionLimitLeft, tractionLimitRight),
				DifferentialType.LimitedSlip
					=> ComputeLimitedSlipDeliveredCapacity(
						MathF.Max(1f, config.BiasRatio),
						SelectPreferredOutput(omegaLeft, omegaRight, tractionLimitLeft, tractionLimitRight),
						tractionLimitLeft,
						tractionLimitRight),
				_
					=> 2f * MathF.Min(tractionLimitLeft, tractionLimitRight),
			};
		}

		/// <summary>
		/// Open differential: equal torque split (50/50).
		/// Each shaft receives half the input torque regardless of speed difference.
		/// This is the simplest and least effective for rally use.
		/// </summary>
		private static void SplitOpen(
			float inputTorque,
			float tractionLimitLeft,
			float tractionLimitRight,
			out float torqueLeft,
			out float torqueRight)
		{
			var deliveredTorque = MathF.Min(
				MathF.Abs(inputTorque),
				2f * MathF.Min(tractionLimitLeft, tractionLimitRight));
			var signedHalfTorque = MathF.CopySign(deliveredTorque * 0.5f, inputTorque);
			torqueLeft = signedHalfTorque;
			torqueRight = signedHalfTorque;
		}

		/// <summary>
		/// Limited-slip differential: transfers additional torque to the slower wheel
		/// based on speed difference and bias ratio.
		///
		/// <para>Model: clutch-type LSD with preload. Locking torque uses a smooth speed-delta
		/// ramp via tanh and combines preload plus torque-proportional locking:
		///   T_lock = (T_preload + lockCoeff(mode) × |T_input|) × tanh(|Δω| / ω_ref)
		/// where mode uses drive lock for positive or zero input torque and coast lock for negative input torque.
		/// The locked torque is transferred from the faster to the slower wheel,
		/// clamped so the torque ratio does not exceed the bias ratio.</para>
		///
		/// Reference: Milliken, RCVD §6.4.2, clutch-type LSD model.
		/// </summary>
		private static void SplitLimitedSlip(
			in DifferentialConfig config,
			float inputTorque,
			float omegaLeft,
			float omegaRight,
			float tractionLimitLeft,
			float tractionLimitRight,
			out float torqueLeft,
			out float torqueRight)
		{
			var preferredLeft = SelectPreferredOutput(omegaLeft, omegaRight, tractionLimitLeft, tractionLimitRight);
			var preferredLimit = preferredLeft ? tractionLimitLeft : tractionLimitRight;
			var otherLimit = preferredLeft ? tractionLimitRight : tractionLimitLeft;
			var biasRatio = MathF.Max(1f, config.BiasRatio);
			var deliveredMagnitude = MathF.Min(
				MathF.Abs(inputTorque),
				ComputeLimitedSlipDeliveredCapacity(biasRatio, preferredLeft, tractionLimitLeft, tractionLimitRight));
			var deltaOmega = omegaLeft - omegaRight;
			var preloadTorque = MathF.Max(0f, config.PreloadTorque);
			var lockCoeff = inputTorque < 0f
				? config.CoastLockingCoefficient
				: config.LockingCoefficient;

			var lockScale = MathF.Tanh(MathF.Abs(deltaOmega) / LockingReferenceOmega);
			var dynamicLockingTorque = MathF.Max(0f, lockCoeff) * deliveredMagnitude;
			var lockingTorque = (preloadTorque + dynamicLockingTorque) * lockScale;
			if (deliveredMagnitude <= MinimumTorqueMagnitude)
			{
				if (lockingTorque <= MinimumTorqueMagnitude)
				{
					torqueLeft = 0f;
					torqueRight = 0f;
					return;
				}

				var preloadTransfer = MathF.Min(lockingTorque, MathF.Min(preferredLimit, otherLimit));
				AssignSignedTransfer(preferredLeft, 0f, preloadTransfer, out torqueLeft, out torqueRight);
				return;
			}

			var signedHalf = MathF.CopySign(deliveredMagnitude * 0.5f, inputTorque);
			var halfMagnitude = deliveredMagnitude * 0.5f;

			var minimumTransfer = MathF.Max(0f, halfMagnitude - otherLimit);
			var maxTransferByPreferredCapacity = MathF.Max(0f, preferredLimit - halfMagnitude);
			var maxTransferByBiasRatio = halfMagnitude * (biasRatio - 1f) / (biasRatio + 1f);
			var maximumTransfer = MathF.Min(
				halfMagnitude,
				MathF.Min(maxTransferByPreferredCapacity, maxTransferByBiasRatio));
			var transfer = maximumTransfer >= minimumTransfer
				? Math.Clamp(lockingTorque, minimumTransfer, maximumTransfer)
				: minimumTransfer;
			AssignSignedTransfer(preferredLeft, signedHalf, transfer, out torqueLeft, out torqueRight);
		}

		/// <summary>
		/// Locking differential: equal speed, equal torque.
		/// Both output shafts forced to rotate at the same speed (rigid coupling).
		/// For torque purposes this is equivalent to a 50/50 split since
		/// the wheel speed equalisation is handled by the constraint system.
		/// </summary>
		private static void SplitLocking(
			float inputTorque,
			float tractionLimitLeft,
			float tractionLimitRight,
			out float torqueLeft,
			out float torqueRight)
		{
			SplitOpen(inputTorque, tractionLimitLeft, tractionLimitRight, out torqueLeft, out torqueRight);
		}

		private static bool SelectPreferredOutput(
			float omegaLeft,
			float omegaRight,
			float tractionLimitLeft,
			float tractionLimitRight)
		{
			if (tractionLimitLeft > tractionLimitRight + TractionEpsilon)
			{
				return true;
			}

			if (tractionLimitRight > tractionLimitLeft + TractionEpsilon)
			{
				return false;
			}

			return omegaLeft < omegaRight;
		}

		private static float ComputeLimitedSlipDeliveredCapacity(
			float biasRatio,
			bool preferredLeft,
			float tractionLimitLeft,
			float tractionLimitRight)
		{
			var preferredLimit = preferredLeft ? tractionLimitLeft : tractionLimitRight;
			var otherLimit = preferredLeft ? tractionLimitRight : tractionLimitLeft;
			return otherLimit + MathF.Min(preferredLimit, otherLimit * biasRatio);
		}

		private static void AssignSignedTransfer(
			bool preferredLeft,
			float signedHalfTorque,
			float transfer,
			out float torqueLeft,
			out float torqueRight)
		{
			if (preferredLeft)
			{
				torqueLeft = signedHalfTorque + transfer;
				torqueRight = signedHalfTorque - transfer;
				return;
			}

			torqueLeft = signedHalfTorque - transfer;
			torqueRight = signedHalfTorque + transfer;
		}

		private static float SanitizeTractionLimit(float tractionLimit)
		{
			if (!float.IsFinite(tractionLimit))
			{
				return float.PositiveInfinity;
			}

			return MathF.Max(0f, tractionLimit);
		}
	}
}
