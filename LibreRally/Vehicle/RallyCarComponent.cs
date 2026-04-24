using System;
using System.Collections.Generic;
using System.Linq;
using LibreRally.Vehicle.Physics;
using Stride.BepuPhysics;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;
using Stride.Particles;
using Stride.Particles.Components;
using Stride.Particles.Initializers;
using Stride.Particles.Materials;
using Stride.Particles.Modules;
using Stride.Particles.ShapeBuilders;
using Stride.Particles.Spawners;

namespace LibreRally.Vehicle
{
	/// <summary>
	/// Core component for rally vehicle simulation, handling engine, transmission, steering, and tyre dynamics.
	/// </summary>
	[ComponentCategory("LibreRally")]
	public class RallyCarComponent : SyncScript
	{
		private const float GroundProbeMargin = 0.05f;
		private const float SurfaceBlendDistance = 0.08f;
		private const float MinimumWheelContactPointScale = 0.01f;
		private const float GamePadTriggerDeadzone = 0.08f;
		private const float GamePadSteerDeadzone = 0.12f;
		private const float SleepLinearSpeedThreshold = 0.10f;
		private const float SleepAngularSpeedThreshold = 0.10f;
		// BeamNG alignment variables represent a beam precompression ratio around 1.0.
		// The camber vars in shipped JBeam data typically span about ±0.06 around unity, so
		// mapping that range with a 0.9 rad/unit scale yields a practical static-camber window
		// of roughly ±3 degrees before additional runtime clamping.
		private const float CamberRadiansPerPrecompressionUnit = 0.9f;
		private const float MaxStaticCamberFromAlignmentRadians = 0.2f;
		private const float MaxDynamicAlignmentCamberRadians = 0.35f;
		// Engine thermal simulation constants
		private const float MaxLoadTempIncreaseC = 30f;
		private const float AirSpeedCoolingMultiplier = 0.1f;
		private const float CoolingCoefScale = 0.001f;
		private const float BaseBurnRateLitersPerSecond = 0.015f;
		private const float DefaultBurnEfficiency = 0.3f;
		private const float TurboSpoolRatePerSecond = 3f;
		private const float WheelSurfaceVfxRiseRate = 12f;
		private const float WheelSurfaceVfxFallRate = 4f;
		private const float WheelSurfaceVfxMaxSpawnRate = 90f;
		private const float WheelSurfaceVfxSlipAngleWeight = 1.1f;
		private const float WheelSurfaceVfxMinSlipRampRange = 1e-4f;
		private const float TarmacVfxSlipRampStart = 0.22f;
		private const float TarmacVfxSlipRampFull = 0.90f;
		private const float GravelVfxSlipRampStart = 0.06f;
		private const float GravelVfxSlipRampFull = 0.40f;
		private const float SnowVfxSlipRampStart = 0.04f;
		private const float SnowVfxSlipRampFull = 0.28f;
		private const float WheelSurfaceVfxIntensityClampMax = 1f;
		private const float WheelSurfaceVfxMaxLoadScale = 1.25f;
		private const float WheelSurfaceVfxContactPatchRadiusScale = 0.82f;
		private const float WheelSurfaceVfxMinVerticalOffset = 0.12f;
		private const float WheelSurfaceVfxPositionMinX = -0.10f;
		private const float WheelSurfaceVfxPositionMinY = -0.02f;
		private const float WheelSurfaceVfxPositionMinZ = -0.10f;
		private const float WheelSurfaceVfxPositionMaxX = 0.10f;
		private const float WheelSurfaceVfxPositionMaxY = 0.08f;
		private const float WheelSurfaceVfxPositionMaxZ = 0.10f;
		private const float WheelSurfaceVfxVelocityMinX = -0.8f;
		private const float WheelSurfaceVfxVelocityMinY = 0.2f;
		private const float WheelSurfaceVfxVelocityMinZ = -0.8f;
		private const float WheelSurfaceVfxVelocityMaxX = 0.8f;
		private const float WheelSurfaceVfxVelocityMaxY = 1.3f;
		private const float WheelSurfaceVfxVelocityMaxZ = 0.8f;
		private const float WheelSurfaceVfxSizeMin = 0.18f;
		private const float WheelSurfaceVfxSizeMax = 0.42f;
		private const float GravelVfxRed = 0.74f;
		private const float GravelVfxGreen = 0.62f;
		private const float GravelVfxBlue = 0.47f;
		private const float GravelVfxBaseAlpha = 0.20f;
		private const float GravelVfxIntensityAlphaScale = 0.45f;
		private const float TarmacVfxRed = 0.42f;
		private const float TarmacVfxGreen = 0.42f;
		private const float TarmacVfxBlue = 0.42f;
		private const float TarmacVfxBaseAlpha = 0.16f;
		private const float TarmacVfxIntensityAlphaScale = 0.38f;
		private const float SnowVfxRed = 0.92f;
		private const float SnowVfxGreen = 0.94f;
		private const float SnowVfxBlue = 0.98f;
		private const float SnowVfxBaseAlpha = 0.24f;
		private const float SnowVfxIntensityAlphaScale = 0.46f;
		private static readonly Color4 TransparentColor = new(0f, 0f, 0f, 0f);
		/// <summary>Fraction of heat generated at idle (no throttle).</summary>
		private const float IdleHeatFraction = 0.15f;
		/// <summary>Oil viscosity coefficient: pressure drops by this fraction per °C above reference.</summary>
		private const float OilViscosityTempCoef = 0.003f;
		/// <summary>Oil reference temperature (°C) above which pressure starts to drop.</summary>
		private const float OilViscosityRefTempC = 80f;

		/// <summary>Gets or sets the entity representing the car's main chassis body.</summary>
		public Entity CarBody { get; set; } = new();

		/// <summary>Gets or sets the list of entities representing all wheels on the vehicle.</summary>
		public List<Entity> Wheels { get; set; } = [];

		/// <summary>Gets or sets the list of entities representing wheels that can be steered.</summary>
		public List<Entity> SteerWheels { get; set; } = [];

		/// <summary>Gets or sets the list of entities representing wheels that are driven by the engine.</summary>
		public List<Entity> DriveWheels { get; set; } = [];

		/// <summary>Gets or sets the list of entities representing wheels that have brakes applied.</summary>
		public List<Entity> BreakWheels { get; set; } = [];

		/// <summary>
		/// Vehicle dynamics system computing tyre forces, load transfer, differentials,
		/// and anti-roll bar effects. Created during <see cref="Start"/> and updated each frame.
		/// </summary>
		public VehicleDynamicsSystem? Dynamics { get; set; }

		/// <summary>Visual suspension link rig driven from the rigid-body wheel/chassis state.</summary>
		internal Rendering.SuspensionVisualKinematicsRig? SuspensionVisualRig { get; set; }

		/// <summary>Wheel radius used to convert speed to spin rate (m).</summary>
		public float WheelRadius { get; set; } = 0.305f;

		/// <summary>Brake motor force — how hard wheels resist rotation when braking.</summary>
		public float BrakeMotorForce { get; set; } = 10000f;

		/// <summary>Extra rear brake force multiplier for the handbrake so it can lock the rear axle.</summary>
		public float HandbrakeForceMultiplier { get; set; } = 4f;

		/// <summary>Maximum steering wheel lock angle (radians). ~0.52 rad ≈ 30°.</summary>
		public float MaxSteerAngle { get; set; } = 0.52f;

		/// <summary>Angular rate at which front wheels steer (rad/s).</summary>
		public float SteerRate { get; set; } = 3f;

		/// <summary>
		/// Minimal parking-speed yaw helper. It only tops up initial rotation until tyre forces
		/// and suspension load transfer take over.
		/// </summary>
		public float ChassisYawAssist { get; set; } = 1.0f;

		/// <summary>Static front-axle camber angle (rad), sourced from JBeam alignment settings.</summary>
		public float FrontStaticCamberRadians { get; set; }

		/// <summary>Static rear-axle camber angle (rad), sourced from JBeam alignment settings.</summary>
		public float RearStaticCamberRadians { get; set; }

		/// <summary>Front camber gain in bump/rebound (rad/m). Negative values add negative camber in bump.</summary>
		public float FrontCamberGainPerMeter { get; set; } = -0.35f;

		/// <summary>Rear camber gain in bump/rebound (rad/m). Negative values add negative camber in bump.</summary>
		public float RearCamberGainPerMeter { get; set; } = -0.25f;

		/// <summary>Legacy grip tuning value, now used as the soft-body tyre model's low-speed damping gain.</summary>
		public float LateralGrip { get; set; } = 6f;

		// ── Drivetrain ───────────────────────────────────────────────────────────

		/// <summary>Forward gear ratios indexed 1–N (index 0 = reverse).</summary>
		public float[] GearRatios { get; set; } = [3.25f, 3.64f, 2.38f, 1.76f, 1.35f, 1.06f, 0.84f];

		/// <summary>Final drive (differential) ratio.</summary>
		public float FinalDrive { get; set; } = 4.55f;

		/// <summary>RPM axis of the engine torque curve.</summary>
		public float[] TorqueCurveRpm { get; set; } =
			[0,  500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500];

		/// <summary>Torque (N·m at crank) corresponding to each RPM in <see cref="TorqueCurveRpm"/>.</summary>
		public float[] TorqueCurveNm { get; set; } =
			[0,   62,  105,  142,  176,  198,  210,  216,  221,  222,  221,  218,  210,  199,  186,  168];

		/// <summary>Engine + flywheel rotational inertia (kg·m²). Governs rev-response speed.</summary>
		public float EngineInertia { get; set; } = 0.25f;

		/// <summary>Constant engine friction torque (N·m).</summary>
		public float EngineFriction { get; set; } = 11.5f;

		/// <summary>Friction term proportional to engine angular velocity (N·m per rad/s).</summary>
		public float EngineDynamicFriction { get; set; } = 0.024f;

		/// <summary>Engine braking torque applied when throttle is lifted (N·m).</summary>
		public float EngineBrakeTorque { get; set; } = 38f;

		/// <summary>Max engine RPM (redline).</summary>
		public float MaxRpm { get; set; } = 7500f;

		/// <summary>Idle RPM (minimum engine speed).</summary>
		public float IdleRpm { get; set; } = 900f;

		/// <summary>Auto-clutch launch hold RPM used to let the engine build torque from a stop.</summary>
		public float AutoClutchLaunchRpm { get; set; } = 4500f;

		/// <summary>
		/// Equivalent engine-RPM window of excess driven-wheel spin over which the automatic clutch
		/// reduces transmitted torque to stop a full-throttle launch becoming a sustained burnout.
		/// </summary>
		public float AutoClutchWheelspinWindowRpm { get; set; } = 1400f;

		/// <summary>
		/// Minimum fraction of engine torque the automatic clutch still transmits while limiting
		/// excessive grounded wheelspin.
		/// </summary>
		public float AutoClutchMinTorqueScale { get; set; } = 0.25f;

		/// <summary>Enables engine-side traction control based on driven-wheel wheelspin.</summary>
		public bool TractionControlEnabled { get; set; } = true;

		/// <summary>Driven-wheel slip ratio target sourced from the active BeamNG traction-control definition.</summary>
		public float TractionControlSlipRatioTarget { get; set; } = 0.15f;

		/// <summary>Slip-ratio window above the TCS target over which torque reduction reaches the minimum scale.</summary>
		public float TractionControlSlipRatioWindow { get; set; } = 0.10f;

		/// <summary>TCS stays inactive right at crawl speed where slip-ratio estimates are numerically noisy.</summary>
		public float TractionControlMinimumSpeedKmh { get; set; } = 5f;

		/// <summary>How quickly TCS removes torque once excess driven-wheel slip is detected (1/s).</summary>
		public float TractionControlApplyRate { get; set; } = 16f;

		/// <summary>How quickly TCS restores torque after wheel slip recovers (1/s).</summary>
		public float TractionControlReleaseRate { get; set; } = 8f;

		/// <summary>Minimum fraction of engine torque that TCS still allows through when fully active.</summary>
		public float TractionControlMinTorqueScale { get; set; } = 0.08f;

		/// <summary>Driven-wheel slip ratio above which the HUD traction warning lamp turns on.</summary>
		public float TractionLossSlipThreshold { get; set; } = 0.18f;

		/// <summary>Whether the active vehicle definition equips ABS.</summary>
		public bool AbsEnabled { get; set; }

		/// <summary>Allows UI systems to temporarily block player input without pausing physics.</summary>
		public bool PlayerInputEnabled { get; set; } = true;

		/// <summary>Target braking slip-ratio magnitude taken from the active JBeam brake definition.</summary>
		public float AbsSlipRatioTarget { get; set; } = 0.15f;

		/// <summary>Slip-ratio window above the ABS target over which brake torque ramps down to the minimum scale.</summary>
		public float AbsSlipRatioWindow { get; set; } = 0.10f;

		/// <summary>Minimum fraction of service-brake torque ABS still allows when a wheel is close to locking.</summary>
		public float AbsMinBrakeTorqueScale { get; set; } = 0.18f;

		/// <summary>ABS stays inactive at parking speeds to avoid chatter right at a stop.</summary>
		public float AbsMinimumSpeedKmh { get; set; } = 4f;

		/// <summary>How quickly ABS removes brake torque once lock-up is detected (1/s).</summary>
		public float AbsApplyRate { get; set; } = 24f;

		/// <summary>How quickly ABS restores brake torque after wheel speed recovers (1/s).</summary>
		public float AbsReleaseRate { get; set; } = 12f;

		/// <summary>RPM to upshift at (auto-transmission).</summary>
		public float ShiftUpRpm { get; set; } = 6500f;

		/// <summary>RPM to downshift at (auto-transmission).</summary>
		public float ShiftDownRpm { get; set; } = 2200f;

		/// <summary>Maximum extra RPM that driven-wheel slip may contribute above road-speed RPM for shifting and torque coupling.</summary>
		public float ShiftSlipAllowanceRpm { get; set; } = 900f;

		/// <summary>Vehicle speed below which holding brake can select reverse gear.</summary>
		public float ReverseEngageSpeedKmh { get; set; } = 1.5f;

		// ── Engine thermal / oil / fuel / turbo ──────────────────────────────────

		/// <summary>Fuel tank capacity (litres). Zero when no tank is defined.</summary>
		public float FuelCapacityLiters { get; set; }

		/// <summary>Initial fuel load (litres), resolved from JBeam startingFuelCapacity.</summary>
		public float StartingFuelLiters { get; set; }

		/// <summary>Engine oil volume (litres) from JBeam. Used for thermal mass calculation.</summary>
		public float OilVolumeLiters { get; set; }

		/// <summary>Engine block temperature at which damage begins (°C).</summary>
		public float EngineBlockTempDamageThreshold { get; set; } = 180f;

		/// <summary>Thermostat target temperature (°C) — the engine's normal operating temperature.</summary>
		public float AirRegulatorTemperature { get; set; } = 85f;

		/// <summary>Air cooling efficiency coefficient from JBeam.</summary>
		public float EngineBlockAirCoolingEfficiency { get; set; }

		/// <summary>Burn efficiency throttle axis (0–1). Pair with <see cref="BurnEfficiencyValues"/>.</summary>
		public float[] BurnEfficiencyThrottle { get; set; } = [];

		/// <summary>Burn efficiency values (0–1) corresponding to each throttle point.</summary>
		public float[] BurnEfficiencyValues { get; set; } = [];

		/// <summary>Whether this engine has a turbocharger.</summary>
		public bool HasTurbo { get; set; }

		/// <summary>Maximum turbo boost pressure in PSI (from wastegate setting).</summary>
		public float TurboMaxBoostPsi { get; set; }

		// ── Read-only telemetry ──────────────────────────────────────────────────
		/// <summary>Current vehicle speed in km/h.</summary>
		public float SpeedKmh { get; private set; }

		/// <summary>Forward velocity in m/s.</summary>
		public float ForwardSpeedMs { get; private set; }

		/// <summary>Lateral velocity in m/s (slip speed).</summary>
		public float LateralSpeedMs { get; private set; }

		/// <summary>Yaw rate in radians per second.</summary>
		public float YawRateRad { get; private set; }

		/// <summary>Current engine RPM.</summary>
		public float EngineRpm { get; private set; }

		/// <summary>Filtered throttle input (0–1).</summary>
		public float ThrottleInput { get; private set; }

		/// <summary>Filtered combined brake input (0–1).</summary>
		public float BrakeInput { get; private set; }

		/// <summary>Combined drive demand (throttle - brake).</summary>
		public float DriveInput { get; private set; }

		/// <summary>Filtered service brake input (0–1).</summary>
		public float ServiceBrakeInput { get; private set; }

		/// <summary>Filtered steering input (-1 to 1).</summary>
		public float SteeringInput { get; private set; }

		/// <summary>Current steering rack position (radians).</summary>
		public float SteeringRack { get; private set; }

		/// <summary>Effective driveline RPM before final drive.</summary>
		public float DrivelineRpm { get; private set; }

		/// <summary>Whether the handbrake is currently engaged.</summary>
		public bool HandbrakeEngaged { get; private set; }

		/// <summary>Whether significant wheel slip is detected on any wheel.</summary>
		public bool TractionLossDetected { get; private set; }

		/// <summary>Whether traction control is actively reducing torque.</summary>
		public bool TractionControlActive { get; private set; }

		/// <summary>Whether ABS is actively reducing brake pressure.</summary>
		public bool AbsActive { get; private set; }

		/// <summary>Current maximum slip ratio across all driven wheels.</summary>
		public float DrivenWheelSlipRatio { get; private set; }

		/// <summary>Current torque scale factor applied by traction control (0–1).</summary>
		public float TractionControlTorqueScale { get; private set; } = 1f;

		/// <summary>Currently selected gear (0 = reverse, 1 = first, etc.).</summary>
		public int CurrentGear { get; private set; } = 1;

		/// <summary>Total drive ratio from engine to wheel (including gear and final drive).</summary>
		public float DriveRatio => EffectiveRatio;

		// ── Engine thermal / oil / fuel / turbo runtime state ────────────────────
		/// <summary>Current engine block temperature (°C). Settles at the air-regulator setpoint.</summary>
		public float EngineTemp { get; private set; }

		/// <summary>Remaining fuel (litres).</summary>
		public float FuelLiters { get; private set; }

		/// <summary>Current oil pressure (bar). Correlates with RPM and oil volume.</summary>
		public float OilPressure { get; private set; }

		/// <summary>Current oil temperature (°C). Lags slightly behind engine block temp.</summary>
		public float OilTemp { get; private set; }

		/// <summary>Current turbo boost pressure (bar). Zero when no turbo is present.</summary>
		public float TurboBoostBar { get; private set; }

		private float EffectiveRatio => GearRatios[Math.Clamp(CurrentGear, 0, GearRatios.Length - 1)] * FinalDrive;
		private float _shiftCooldown;

		// Simulated engine RPM — the engine drives the drivetrain, not the other way around.
		private float _engineRpm;
		// Thermal / oil / fuel / turbo simulation state
		private float _engineTemp;
		private float _oilTemp;
		private float _fuelLiters;
		private float _turboBoostBar;

		// ── Cached arrays for VehicleDynamicsSystem (avoid per-frame allocations) ─
		private readonly Vector3[] _wheelPositions = new Vector3[VehicleDynamicsSystem.WheelCount];
		private readonly Vector3[] _wheelContactPoints = new Vector3[VehicleDynamicsSystem.WheelCount];
		private readonly Vector3[] _suspensionAttachmentPoints = new Vector3[VehicleDynamicsSystem.WheelCount];
		private readonly Vector3[] _suspensionAxes = new Vector3[VehicleDynamicsSystem.WheelCount];
		private readonly Vector3[] _wheelVelocities = new Vector3[VehicleDynamicsSystem.WheelCount];
		private readonly Matrix[] _wheelOrientations = new Matrix[VehicleDynamicsSystem.WheelCount];
		private readonly float[] _wheelContactScales = new float[VehicleDynamicsSystem.WheelCount];
		private readonly bool[] _wheelGrounded = new bool[VehicleDynamicsSystem.WheelCount];
		private readonly float[] _suspensionCompressions = new float[VehicleDynamicsSystem.WheelCount];
		private readonly float[] _brakeTorques = new float[VehicleDynamicsSystem.WheelCount];
		private readonly float[] _camberAngles = new float[VehicleDynamicsSystem.WheelCount];
		private readonly float[] _absBrakeTorqueScales = [1f, 1f, 1f, 1f];
		private readonly Entity?[] _wheelSurfaceVfxEntities = new Entity?[VehicleDynamicsSystem.WheelCount];
		private readonly ParticleSystemComponent?[] _wheelSurfaceVfxComponents = new ParticleSystemComponent?[VehicleDynamicsSystem.WheelCount];
		private readonly SpawnerPerSecond?[] _wheelSurfaceVfxSpawners = new SpawnerPerSecond?[VehicleDynamicsSystem.WheelCount];
		private readonly float[] _wheelSurfaceVfxIntensity = new float[VehicleDynamicsSystem.WheelCount];
		private bool _wheelSurfaceVfxInitialized;

		/// <summary>
		/// Initializes runtime drivetrain, thermal, and wheel-surface state for the loaded rally car.
		/// </summary>
		public override void Start()
		{
			_engineRpm = IdleRpm;
			_engineTemp = AirRegulatorTemperature > 0f ? AirRegulatorTemperature * 0.5f : 40f;
			_oilTemp = _engineTemp;
			var startingFuelLiters = StartingFuelLiters >= 0f
				? MathUtil.Clamp(StartingFuelLiters, 0f, FuelCapacityLiters)
				: FuelCapacityLiters;

			_fuelLiters = startingFuelLiters;
			_turboBoostBar = 0f;

			EngineTemp = _engineTemp;
			OilTemp = _oilTemp;
			FuelLiters = _fuelLiters;
			TurboBoostBar = _turboBoostBar;
			OilPressure = 0f;

			// Create default dynamics system if none was injected by VehicleLoader
			Dynamics ??= new VehicleDynamicsSystem();
			InitializeWheelSurfaceVfx();
		}

		/// <summary>
		/// Advances the rally car simulation, input handling, and telemetry for the current frame.
		/// </summary>
		public override void Update()
		{
			var dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;
			if (dt <= 0f || dt > 0.1f)
			{
				dt = 0.016f;
			}

			if (!Game.IsActive)
			{
				ClearLiveInputs();
				SetVehicleBodiesAwake(false);
				return;
			}

			// ── Input ────────────────────────────────────────────────────────────
			var pad = PlayerInputEnabled ? Input.GamePads.FirstOrDefault() : null;
			float throttle = 0f, brake = 0f, steer = 0f;
			var handbrakeRequested = false;

			if (PlayerInputEnabled)
			{
				if (pad != null)
				{
					throttle  = ApplyTriggerDeadzone(pad.State.RightTrigger, GamePadTriggerDeadzone);
					brake     = ApplyTriggerDeadzone(pad.State.LeftTrigger, GamePadTriggerDeadzone);
					steer     = ApplySignedAxisDeadzone(pad.State.LeftThumb.X, GamePadSteerDeadzone);
					handbrakeRequested = pad.IsButtonDown(GamePadButton.A);
				}

				if (Input.IsKeyDown(Keys.Up)    || Input.IsKeyDown(Keys.W))
				{
					throttle  = MathF.Max(throttle, 1f);
				}

				if (Input.IsKeyDown(Keys.Down)  || Input.IsKeyDown(Keys.S))
				{
					brake     = MathF.Max(brake,    1f);
				}

				if (Input.IsKeyDown(Keys.Left)  || Input.IsKeyDown(Keys.A))
				{
					steer     = MathF.Min(steer,   -1f);
				}

				if (Input.IsKeyDown(Keys.Right) || Input.IsKeyDown(Keys.D))
				{
					steer     = MathF.Max(steer,    1f);
				}

				if (Input.IsKeyDown(Keys.Space))
				{
					handbrakeRequested = true;
				}
			}

			// ── Chassis physics ──────────────────────────────────────────────────
			var chassisBody = CarBody.Get<BodyComponent>();
			if (chassisBody == null)
			{
				return;
			}

			var chassisTransform = CarBody.Transform;
			chassisTransform.UpdateWorldMatrix();

			var noseDir = chassisTransform.WorldMatrix.Backward;  // local +Z = nose
			var rightDir = chassisTransform.WorldMatrix.Right;
			var vel = chassisBody.LinearVelocity;
			var forwardSpeed = Vector3.Dot(vel, noseDir);
			var lateralSpeed = Vector3.Dot(vel, rightDir);

			SpeedKmh     = MathF.Abs(forwardSpeed) * 3.6f;
			ForwardSpeedMs = forwardSpeed;
			LateralSpeedMs = lateralSpeed;
			YawRateRad = chassisBody.AngularVelocity.Y;
			ThrottleInput = throttle;
			BrakeInput    = brake;
			SteeringInput = steer;

			// ── Auto transmission ────────────────────────────────────────────────
			// Use slip-clamped driven-shaft RPM for shift decisions and engine coupling.
			// The sampled wheel omega remains signed so reverse and engine braking keep
			// the true shaft direction, but raw low-speed wheelspin can only add a
			// limited RPM above road speed.
			var effectiveRatio = EffectiveRatio;
			_shiftCooldown = MathF.Max(0f, _shiftCooldown - dt);

			var standingGear = ResolveStandingGearSelection(
				CurrentGear,
				SpeedKmh,
				forwardSpeed,
				throttle,
				brake,
				handbrakeRequested,
				_shiftCooldown,
				ReverseEngageSpeedKmh);
			if (standingGear != CurrentGear)
			{
				var changedDirection = standingGear == 0 || CurrentGear == 0;
				CurrentGear    = standingGear;
				effectiveRatio = EffectiveRatio;
				_shiftCooldown = changedDirection ? 0.25f : 0.4f;
			}

			var isReverseGear = CurrentGear == 0;
			var driveDirection = isReverseGear ? -1f : 1f;
			var driveInput = isReverseGear ? brake : throttle;
			var serviceBrakeInput = isReverseGear ? throttle : brake;
			var isBraking   = serviceBrakeInput > 0.05f;
			var isHandbrake = handbrakeRequested;
			DriveInput = driveInput;
			ServiceBrakeInput = serviceBrakeInput;
			HandbrakeEngaged = isHandbrake;
			SetVehicleBodiesAwake(ShouldKeepVehicleAwake(
				throttle,
				brake,
				steer,
				handbrakeRequested,
				chassisBody.LinearVelocity,
				chassisBody.AngularVelocity));

			var safeWheelRadius = MathF.Max(WheelRadius, 0.05f);
			var roadWheelOmega = forwardSpeed / safeWheelRadius;
			var wheelToEngineRpm = effectiveRatio * (60f / (2f * MathF.PI));
			var drivenWheelOmega = MeasureDrivenWheelOmega(forwardSpeed);
			MeasureDrivenWheelTractionState(
				forwardSpeed,
				out var autoClutchDrivenWheelOmega,
				out var drivenWheelSlipRatio);
			DrivenWheelSlipRatio = drivenWheelSlipRatio;
			TractionLossDetected = driveInput > 0.05f && SpeedKmh >= 1f && drivenWheelSlipRatio >= TractionLossSlipThreshold;
			var tractionControlCanIntervene = driveInput > 0.05f
			                                  && TractionControlEnabled
			                                  && !isHandbrake
			                                  && SpeedKmh >= TractionControlMinimumSpeedKmh;
			var tractionControlTargetScale = tractionControlCanIntervene
				? ComputeTractionControlTorqueScale(
					drivenWheelSlipRatio,
					TractionControlSlipRatioTarget,
					TractionControlSlipRatioWindow,
					TractionControlMinTorqueScale)
				: 1f;
			var tractionControlRate = tractionControlTargetScale < TractionControlTorqueScale
				? TractionControlApplyRate
				: TractionControlReleaseRate;
			var tractionControlScale = AdvanceControllerScale(
				TractionControlTorqueScale,
				tractionControlTargetScale,
				tractionControlRate,
				dt);
			TractionControlTorqueScale = tractionControlScale;
			TractionControlActive = tractionControlCanIntervene && tractionControlScale < 0.98f;
			var slipClampedDrivenWheelOmega = ClampDrivenWheelOmegaForSlip(
				roadWheelOmega,
				drivenWheelOmega,
				effectiveRatio,
				ShiftSlipAllowanceRpm);
			var shiftRpm = Math.Clamp(MathF.Abs(slipClampedDrivenWheelOmega) * wheelToEngineRpm, IdleRpm, MaxRpm);

			var numForwardGears = GearRatios.Length - 1;
			if (CurrentGear > 0)
			{
				if (_shiftCooldown <= 0f && !isBraking && !isHandbrake && driveInput > 0.05f)
				{
					if (shiftRpm >= ShiftUpRpm && CurrentGear < numForwardGears)
					{
						CurrentGear++;
						_shiftCooldown = 0.4f;
						effectiveRatio = EffectiveRatio;
					}
					else if (shiftRpm <= ShiftDownRpm && CurrentGear > 1)
					{
						CurrentGear--;
						_shiftCooldown = 0.4f;
						effectiveRatio = EffectiveRatio;
					}
				}
				if (_shiftCooldown <= 0f && isBraking && CurrentGear > 1)
				{
					if (shiftRpm < ShiftDownRpm)
					{
						CurrentGear--;
						_shiftCooldown = 0.4f;
						effectiveRatio = EffectiveRatio;
					}
				}
			}

			wheelToEngineRpm = effectiveRatio * (60f / (2f * MathF.PI));
			slipClampedDrivenWheelOmega = ClampDrivenWheelOmegaForSlip(
				roadWheelOmega,
				drivenWheelOmega,
				effectiveRatio,
				ShiftSlipAllowanceRpm);
			var drivelineRpm = Math.Clamp(MathF.Abs(slipClampedDrivenWheelOmega) * wheelToEngineRpm, IdleRpm, MaxRpm);
			DrivelineRpm = drivelineRpm;

			var gearLabel = isReverseGear ? "R" : CurrentGear.ToString();
			var padInfo = pad != null
				? $"Pad | T:{throttle:F2} B:{brake:F2} S:{steer:F2} G:{gearLabel}{(isHandbrake ? " HB" : string.Empty)}"
				: $"No pad ({Input.GamePads.Count}) — use arrows";
			DebugText.Print($"{padInfo} | {SpeedKmh:F0} km/h", new Int2(10, 30));

			// ── Engine simulation ─────────────────────────────────────────────────
			// Two separate concerns:
			//
			// 1. Wheel force (physics): derived from actual wheel/car speed → torque curve.
			//    This is always well-behaved — no free-revving, no stall, no redline cutoff.
			//
			// 2. RPM display: a state variable that revs up with throttle and is pulled
			//    toward the drivetrain demand as the car accelerates. This gives a
			//    realistic rev needle without affecting the driving physics.

			// ─ Torque for wheels (based on actual drivetrain state) ──────────────
			// Launch from a dead stop needs clutch slip; otherwise a locked wheel-speed
			// model is stuck at idle torque forever. Hold the engine in the launch band
			// until wheel speed catches up, then hand over to the real driveline RPM.
			var launchRpm   = IdleRpm + (AutoClutchLaunchRpm - IdleRpm) * driveInput;
			var torqueRpm   = driveInput > 0.05f ? MathF.Max(drivelineRpm, launchRpm) : drivelineRpm;
			// Base the launch wheelspin limiter on the slowest grounded driven wheel so
			// one unloaded open-diff wheel does not choke torque to the whole axle.
			var autoClutchSlipClampedOmega = ClampDrivenWheelOmegaForSlip(
				roadWheelOmega,
				autoClutchDrivenWheelOmega,
				effectiveRatio,
				AutoClutchWheelspinWindowRpm);
			var crankTorqueScale = driveInput > 0.05f
				? ComputeAutoClutchTorqueScale(
					autoClutchDrivenWheelOmega,
					autoClutchSlipClampedOmega,
					effectiveRatio,
					AutoClutchWheelspinWindowRpm,
					AutoClutchMinTorqueScale) * tractionControlScale
				: 1f;
			var crankTorque = InterpolateTorqueCurve(torqueRpm) * driveInput * crankTorqueScale;
			var applyEngineBraking = driveInput < 0.05f && MathF.Abs(forwardSpeed) > 0.25f;
			var engBrake    = applyEngineBraking ? EngineBrakeTorque : 0f;
			var demandRpm   = drivelineRpm;

			// ─ RPM display (state variable for the gauge) ───────────────────────
			var engineOmega  = _engineRpm * (2f * MathF.PI / 60f);
			var frictionLoss = EngineFriction + EngineDynamicFriction * engineOmega;

			// Rev up with throttle; coupling pulls toward actual drivetrain demand
			var displayCrank = InterpolateTorqueCurve(_engineRpm) * driveInput;
			if (_engineRpm >= MaxRpm)
			{
				displayCrank = 0f;
			}

			var netDisplay   = displayCrank - frictionLoss - engBrake;
			_engineRpm += (netDisplay / EngineInertia) * dt * (60f / (2f * MathF.PI));

			// Coupling: pull display RPM toward the actual driveline demand
			var displayTargetRpm = driveInput > 0.05f ? torqueRpm : demandRpm;
			var clutchCoupling = driveInput > 0.05f ? 7f : 5f;
			_engineRpm += (displayTargetRpm - _engineRpm) * clutchCoupling * dt;

			// Allow clutch flare on launch, but stop the display RPM from free-revving
			// thousands above the actual driveline once the car is rolling.
			var slipBlend = Math.Clamp(SpeedKmh / 35f, 0f, 1f);
			var clutchSlipAllowance = driveInput > 0.05f
				? MathUtil.Lerp(AutoClutchLaunchRpm - IdleRpm, 250f, slipBlend)
				: 150f;
			var maxCoupledRpm = demandRpm + clutchSlipAllowance;
			_engineRpm = MathF.Min(_engineRpm, MathF.Max(IdleRpm, maxCoupledRpm));

			_engineRpm = Math.Clamp(_engineRpm, IdleRpm, MaxRpm + 300f);
			EngineRpm  = _engineRpm;

			// ── Engine thermal / oil / fuel / turbo simulation ────────────────────
			UpdateEngineThermals(dt, driveInput, forwardSpeed);

			// ── Steering motors (front wheels) ────────────────────────────────────
			const float SteerServoGain = 12f;
			const float SteerVelocityLimitMultiplier = 2f;
			var steerTargetAngle = steer * MaxSteerAngle;
			var totalActualSteerAngle = 0f;
			var steerWheelCount = 0;

			foreach (var wheel in SteerWheels)
			{
				var ws = wheel.Get<WheelSettings>();
				if (ws?.SteerMotor == null)
				{
					continue;
				}

				ws.CurrentSteerAngle += Math.Clamp(steerTargetAngle - ws.CurrentSteerAngle, -SteerRate * dt, SteerRate * dt);

				wheel.Transform.UpdateWorldMatrix();
				var actualSteerAngle = MeasureSteerAngle(chassisTransform.WorldMatrix, wheel.Transform.WorldMatrix);
				var error            = ws.CurrentSteerAngle - actualSteerAngle;
				ws.SteerMotor.TargetVelocity = Math.Clamp(
					error * SteerServoGain,
					-SteerRate * SteerVelocityLimitMultiplier,
					SteerRate * SteerVelocityLimitMultiplier);

				totalActualSteerAngle += actualSteerAngle;
				steerWheelCount++;
			}

			// ── Wheel motor commands ──────────────────────────────────────────────
			// When VehicleDynamicsSystem is active, it applies tyre forces (Fx, Fy) directly
			// as impulses to the chassis. Disable BEPU wheel drive/brake motors to avoid
			// double-counting longitudinal forces (the dynamics system is authoritative).
			// Target = redline speed so motor always pushes forward; MaxForce limits torque.
			var   numDrive           = Math.Max(1, DriveWheels.Count);
			var wheelTargetOmega   = -(MaxRpm * (2f * MathF.PI / 60f) / effectiveRatio) * driveDirection;
			var availableWheelForce = MathF.Max(0f, crankTorque) * effectiveRatio / WheelRadius / numDrive;

			foreach (var wheel in DriveWheels)
			{
				var ws = wheel.Get<WheelSettings>();
				if (ws?.DriveMotor == null)
				{
					continue;
				}

				// When dynamics system is active, zero out motor forces to prevent
				// BEPU and VehicleDynamicsSystem from both generating tyre forces.
				if (Dynamics != null)
				{
					ws.DriveMotor.TargetVelocityLocalA = Vector3.Zero;
					ws.DriveMotor.MotorMaximumForce    = 0f;
					ws.DriveMotor.MotorDamping         = 0f;
					continue;
				}

				wheel.Transform.UpdateWorldMatrix();
				var driveAxisLocalA = MeasureWheelAxleAxisLocalA(chassisTransform.WorldMatrix, wheel.Transform.WorldMatrix);
				var rearWheel = IsRearWheel(wheel, ws);

				if (isHandbrake && rearWheel)
				{
					ws.DriveMotor.TargetVelocityLocalA = Vector3.Zero;
					ws.DriveMotor.MotorMaximumForce    = BrakeMotorForce * HandbrakeForceMultiplier;
					ws.DriveMotor.MotorDamping         = 500f;
				}
				else if (isBraking)
				{
					ws.DriveMotor.TargetVelocityLocalA = Vector3.Zero;
					ws.DriveMotor.MotorMaximumForce    = BrakeMotorForce * serviceBrakeInput;
					ws.DriveMotor.MotorDamping         = 500f;
				}
				else if (driveInput > 0.05f)
				{
					ws.DriveMotor.TargetVelocityLocalA = driveAxisLocalA * wheelTargetOmega;
					ws.DriveMotor.MotorMaximumForce    = availableWheelForce;
					ws.DriveMotor.MotorDamping         = 500f;
				}
				else
				{
					// Coasting / engine braking
					ws.DriveMotor.TargetVelocityLocalA = Vector3.Zero;
					ws.DriveMotor.MotorMaximumForce    = engBrake * effectiveRatio / WheelRadius / numDrive;
					ws.DriveMotor.MotorDamping         = 500f;
				}
			}

			var steerRack = steerWheelCount > 0 && MaxSteerAngle > 0.0001f
				? Math.Clamp(totalActualSteerAngle / (steerWheelCount * MaxSteerAngle), -1f, 1f)
				: steer;
			SteeringRack = steerRack;

			// ── Vehicle dynamics system ──────────────────────────────────────────
			// Computes tyre forces (slip-based), load transfer, anti-roll bars,
			// differential torque split, and applies impulses to BEPU bodies.
			var dynamics = Dynamics;
			if (dynamics != null)
			{
				// Gather per-wheel data into cached arrays (zero allocation)
				var wheelTorqueAtCrank = MathF.Max(0f, crankTorque) * effectiveRatio;
				GatherWheelData(chassisBody, chassisTransform.WorldMatrix);

				// Compute brake torque for dynamics system
				var serviceBrakeTorque = BrakeMotorForce * WheelRadius * serviceBrakeInput;
				var handbrakeTorque = BrakeMotorForce * WheelRadius * HandbrakeForceMultiplier;
				var absCanIntervene = isBraking
				                      && AbsEnabled
				                      && !isHandbrake
				                      && SpeedKmh >= AbsMinimumSpeedKmh;
				AbsActive = false;
				for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
				{
					var wheelServiceBrakeTorque = isBraking ? serviceBrakeTorque : 0f;
					if (absCanIntervene)
					{
						var wheelState = dynamics.WheelStates[i];
						var rollingDirection = ResolveRollingDirection(forwardSpeed, wheelState.AngularVelocity);
						var targetBrakeScale = dynamics.WheelGrounded[i]
							? ComputeAbsBrakeTorqueScale(
								wheelState.SlipRatio,
								rollingDirection,
								AbsSlipRatioTarget,
								AbsSlipRatioWindow,
								AbsMinBrakeTorqueScale)
							: 1f;
						var responseRate = targetBrakeScale < _absBrakeTorqueScales[i] ? AbsApplyRate : AbsReleaseRate;
						_absBrakeTorqueScales[i] = AdvanceControllerScale(_absBrakeTorqueScales[i], targetBrakeScale, responseRate, dt);
						wheelServiceBrakeTorque *= _absBrakeTorqueScales[i];
						AbsActive |= _absBrakeTorqueScales[i] < 0.98f;
					}
					else
					{
						_absBrakeTorqueScales[i] = 1f;
					}

					_brakeTorques[i] = wheelServiceBrakeTorque;
					if (isHandbrake && i >= VehicleDynamicsSystem.RL)
					{
						_brakeTorques[i] += handbrakeTorque;
					}
				}

				var chassisWorld = chassisTransform.WorldMatrix;
				var drivenDir = slipClampedDrivenWheelOmega > 0.1f ? 1f : (slipClampedDrivenWheelOmega < -0.1f ? -1f : 0f);
				var drivetrainTorque = driveInput > 0.05f
					? wheelTorqueAtCrank * driveDirection
					: applyEngineBraking
						? -drivenDir * engBrake * effectiveRatio
						: 0f;

				dynamics.Update(
					chassisBody,
					in chassisWorld,
					_wheelContactPoints,
					_suspensionAttachmentPoints,
					_suspensionAxes,
					_wheelVelocities,
					_wheelOrientations,
					_wheelContactScales,
					_wheelGrounded,
					_suspensionCompressions,
					drivetrainTorque,
					_brakeTorques,
					_camberAngles,
					dt);

				// Self-aligning torque belongs in steering feedback, not chassis yaw.
				// Until the game exposes a real steering/FBB path, keep it as telemetry only.
			}

			var frontAxleGrounded = Dynamics == null
			                        || Dynamics.WheelGrounded[VehicleDynamicsSystem.FL]
			                        || Dynamics.WheelGrounded[VehicleDynamicsSystem.FR];
			if (Dynamics == null && frontAxleGrounded)
			{
				var lowSpeedYawAssist = ComputeLowSpeedYawAssistRate(
					steerRack,
					forwardSpeed,
					driveInput,
					driveDirection,
					ChassisYawAssist);

				if (MathF.Abs(lowSpeedYawAssist) > 1e-4f)
				{
					var assistAv = chassisBody.AngularVelocity;
					assistAv.Y = ApplyYawAssistTopUp(assistAv.Y, lowSpeedYawAssist, 1.25f * dt);
					chassisBody.AngularVelocity = assistAv;
				}
			}

			UpdateWheelSurfaceVfx(dt, chassisTransform.WorldMatrix);
			SuspensionVisualRig?.UpdateVisuals();
		}

		/// <summary>
		/// Gathers per-wheel position, velocity, orientation, and ground contact data
		/// into cached arrays for the <see cref="VehicleDynamicsSystem"/>.
		/// </summary>
		private void GatherWheelData(BodyComponent chassisBody, Matrix chassisWorld)
		{
			var dynamics = Dynamics;
			var chassisRightAxis = SafeNormalize(chassisWorld.Right, Vector3.UnitX);
			var fallbackUp = SafeNormalize(chassisWorld.Up, Vector3.UnitY);
			var fallbackLongitudinal = SafeNormalize(chassisWorld.Backward, Vector3.UnitZ);

			for (var i = 0; i < Wheels.Count && i < VehicleDynamicsSystem.WheelCount; i++)
			{
				var wheel = Wheels[i];
				var wheelBody = wheel.Get<BodyComponent>();
				var wheelSettings = wheel.Get<WheelSettings>();

				if (wheelBody == null || wheelSettings == null)
				{
					_wheelContactPoints[i] = Vector3.Zero;
					_suspensionAttachmentPoints[i] = Vector3.Zero;
					_suspensionAxes[i] = fallbackUp;
					_wheelContactScales[i] = 0f;
					_wheelGrounded[i] = false;
					_suspensionCompressions[i] = 0f;
					_camberAngles[i] = 0f;
					if (dynamics != null && i < dynamics.WheelSurfaces.Length)
					{
						dynamics.WheelSurfaces[i] = SurfaceProperties.ForType(SurfaceType.Tarmac);
					}
					if (wheelSettings != null)
					{
						wheelSettings.CurrentSurface = SurfaceType.Tarmac;
						wheelSettings.CurrentSurfaceProperties = SurfaceProperties.ForType(SurfaceType.Tarmac);
					}
					continue;
				}

				wheel.Transform.UpdateWorldMatrix();
				var wheelWorld = wheel.Transform.WorldMatrix;

				// Wheel coordinate frame (excluding spin rotation)
				var wheelRight = SafeNormalize(wheelWorld.Right, chassisRightAxis);
				var nonSpinUp = ProjectOnPlane(fallbackUp, wheelRight);
				var wheelUp = SafeNormalize(nonSpinUp, fallbackUp);
				var wheelFwd = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), fallbackLongitudinal);
				var suspensionAxis = TransformDirection(chassisWorld, wheelSettings.SuspensionLocalAxis, fallbackUp);

				_wheelPositions[i] = wheelWorld.TranslationVector;
				_suspensionAttachmentPoints[i] = Vector3.TransformCoordinate(wheelSettings.SuspensionLocalOffsetA, chassisWorld);
				_suspensionAxes[i] = suspensionAxis;

				var wheelPointOffset = _wheelPositions[i] - chassisWorld.TranslationVector;
				_wheelVelocities[i] = ComputePointVelocity(
					chassisBody.LinearVelocity,
					chassisBody.AngularVelocity,
					wheelPointOffset);

				// Build orientation matrix from non-spinning axes
				_wheelOrientations[i] = new Matrix
				{
					Row1 = new Vector4(wheelRight, 0),
					Row2 = new Vector4(wheelUp, 0),
					Row3 = new Vector4(wheelFwd, 0),
					Row4 = new Vector4(0, 0, 0, 1),
				};

				// Measure suspension travel directly from the current chassis↔wheel relative offset
				// on the same axis/anchors used by the BEPU suspension constraints.
				_suspensionCompressions[i] = MeasureSuspensionCompression(chassisWorld, wheelWorld, wheelSettings, suspensionAxis);

				// Ground probe for contact detection. Use a soft contact scale so the tyre model
				// fades load out near contact loss instead of flipping between full static load and zero.
				var contactScale = ProbeWheelContactScale(chassisBody, wheelBody, wheelSettings, _wheelPositions[i], wheelUp, out _wheelContactPoints[i]);
				_wheelContactScales[i] = contactScale;
				_wheelGrounded[i] = contactScale > 0.05f;
				var dynamicsIndex = wheelSettings.DynamicsIndex >= 0 ? wheelSettings.DynamicsIndex : i;
				if (dynamics != null &&
				    dynamicsIndex < dynamics.WheelSurfaces.Length)
				{
					dynamics.WheelSurfaces[dynamicsIndex] = wheelSettings.CurrentSurfaceProperties;
				}

				var isFrontAxle = !IsRearWheel(wheel, wheelSettings);
				var staticCamber = isFrontAxle ? FrontStaticCamberRadians : RearStaticCamberRadians;
				var camberGain = isFrontAxle ? FrontCamberGainPerMeter : RearCamberGainPerMeter;
				var alignmentCamber = ComputeAlignmentCamberAngle(staticCamber, camberGain, _suspensionCompressions[i]);
				var camberSideSign = ComputeCamberSideSign(Vector3.Dot(wheelPointOffset, chassisRightAxis));
				_camberAngles[i] = alignmentCamber * camberSideSign;
			}
		}

		/// <summary>
		/// Linearly interpolates the engine torque curve at the given RPM.
		/// Falls back to the sunburst 2.0T peak-torque value when no curve is configured.
		/// </summary>
		private float InterpolateTorqueCurve(float rpm)
		{
			if (TorqueCurveRpm == null || TorqueCurveRpm.Length < 2 ||
			    TorqueCurveNm  == null || TorqueCurveNm.Length < TorqueCurveRpm.Length)
			{
				return 222f;  // sunburst peak as fallback
			}

			rpm = Math.Clamp(rpm, TorqueCurveRpm[0], TorqueCurveRpm[^1]);
			for (var i = 1; i < TorqueCurveRpm.Length; i++)
			{
				if (rpm <= TorqueCurveRpm[i])
				{
					var t = (rpm - TorqueCurveRpm[i - 1]) / (TorqueCurveRpm[i] - TorqueCurveRpm[i - 1]);
					return TorqueCurveNm[i - 1] + t * (TorqueCurveNm[i] - TorqueCurveNm[i - 1]);
				}
			}
			return TorqueCurveNm[^1];
		}

		/// <summary>
		/// Lightweight per-frame engine thermal, fuel consumption, oil, and turbo simulation.
		/// Produces gauge-plausible values from JBeam engine/fuel-tank data without full
		/// thermo-fluid modelling.
		/// </summary>
		private void UpdateEngineThermals(float dt, float throttle, float forwardSpeed)
		{
			// ── Engine temperature ──────────────────────────────────────────────
			// Heat generated is proportional to RPM and throttle (proxy for combustion power).
			var rpmFraction = MaxRpm > 0f ? _engineRpm / MaxRpm : 0f;
			var heatInput = (IdleHeatFraction + (1f - IdleHeatFraction) * throttle) * rpmFraction;

			// Cooling: air cooling (speed-dependent) plus the thermostat regulation.
			var targetTemp = AirRegulatorTemperature > 0f ? AirRegulatorTemperature : 85f;
			var airSpeed = MathF.Abs(forwardSpeed);
			var coolingCoef = EngineBlockAirCoolingEfficiency > 0f ? EngineBlockAirCoolingEfficiency : 10f;
			var airCooling = (1f + airSpeed * AirSpeedCoolingMultiplier) * coolingCoef * CoolingCoefScale;

			// Simple first-order approach toward target ± offset from load
			var heatTarget = targetTemp + heatInput * MaxLoadTempIncreaseC;
			_engineTemp += (heatTarget - _engineTemp) * Math.Min(airCooling * dt, 1f);
			_engineTemp = Math.Clamp(_engineTemp, 0f, EngineBlockTempDamageThreshold + 50f);
			EngineTemp = _engineTemp;

			// ── Oil temperature ─────────────────────────────────────────────────
			// Oil temperature follows engine temp with a thermal lag.
			var oilTarget = _engineTemp - 5f; // Oil runs slightly cooler than block
			_oilTemp += (oilTarget - _oilTemp) * Math.Min(0.3f * dt, 1f);
			_oilTemp = Math.Clamp(_oilTemp, 0f, EngineBlockTempDamageThreshold + 20f);
			OilTemp = _oilTemp;

			// ── Oil pressure ────────────────────────────────────────────────────
			// Approximation: proportional to RPM, reduced when oil is hot.
			// Typical range is 1–5 bar. With zero oil volume the gauge stays at zero.
			if (OilVolumeLiters > 0f)
			{
				var basePressure = 1f + 4f * rpmFraction; // 1 bar idle → 5 bar at redline
				var tempFactor = 1f - OilViscosityTempCoef * MathF.Max(0f, _oilTemp - OilViscosityRefTempC);
				OilPressure = MathF.Max(0f, basePressure * Math.Clamp(tempFactor, 0.3f, 1f));
			}
			else
			{
				OilPressure = 0f;
			}

			// ── Fuel consumption ────────────────────────────────────────────────
			if (FuelCapacityLiters > 0f && _fuelLiters > 0f)
			{
				// Burn efficiency interpolation (throttle → efficiency)
				var efficiency = InterpolateBurnEfficiency(throttle);
				// Litres per second: simplified volumetric model.
				// At full throttle and max RPM the engine burns roughly 0.01–0.03 L/s for
				// a small 1.5 L engine; scale with displacement proxy (torque × rpm).
				var powerFraction = throttle * rpmFraction;
				var burnRate = BaseBurnRateLitersPerSecond * powerFraction;
				// Higher efficiency → less fuel for the same power
				if (efficiency > 0f)
				{
					burnRate /= (efficiency / DefaultBurnEfficiency);
				}

				_fuelLiters -= burnRate * dt;
				_fuelLiters = MathF.Max(0f, _fuelLiters);
			}

			FuelLiters = _fuelLiters;

			// ── Turbo boost ─────────────────────────────────────────────────────
			if (HasTurbo && TurboMaxBoostPsi > 0f)
			{
				// Simplified spool model: boost proportional to RPM × throttle with lag.
				var targetBoostPsi = TurboMaxBoostPsi * throttle * rpmFraction;
				_turboBoostBar += (targetBoostPsi * PsiToBar - _turboBoostBar) * Math.Min(TurboSpoolRatePerSecond * dt, 1f);
				_turboBoostBar = MathF.Max(0f, _turboBoostBar);
			}
			else
			{
				_turboBoostBar = 0f;
			}

			TurboBoostBar = _turboBoostBar;
		}

		/// <summary>PSI to bar conversion factor.</summary>
		public const float PsiToBar = 0.0689476f;

		/// <summary>
		/// Interpolates the burn efficiency curve at the given throttle fraction.
		/// Returns a default of 0.3 when no curve is defined.
		/// </summary>
		private float InterpolateBurnEfficiency(float throttle)
		{
			if (BurnEfficiencyThrottle == null || BurnEfficiencyThrottle.Length < 2 ||
			    BurnEfficiencyValues  == null || BurnEfficiencyValues.Length < BurnEfficiencyThrottle.Length)
			{
				return DefaultBurnEfficiency;
			}

			throttle = Math.Clamp(throttle, BurnEfficiencyThrottle[0], BurnEfficiencyThrottle[^1]);
			for (var i = 1; i < BurnEfficiencyThrottle.Length; i++)
			{
				if (throttle <= BurnEfficiencyThrottle[i])
				{
					var t = (throttle - BurnEfficiencyThrottle[i - 1]) /
					        (BurnEfficiencyThrottle[i] - BurnEfficiencyThrottle[i - 1]);
					return BurnEfficiencyValues[i - 1] + t * (BurnEfficiencyValues[i] - BurnEfficiencyValues[i - 1]);
				}
			}

			return BurnEfficiencyValues[^1];
		}

		private static bool IsRearWheel(Entity wheel, WheelSettings? wheelSettings)
		{
			var dynamicsIndex = wheelSettings?.DynamicsIndex ?? -1;
			if ((uint)dynamicsIndex < VehicleDynamicsSystem.WheelCount)
			{
				return dynamicsIndex >= VehicleDynamicsSystem.RL;
			}

			return wheel.Name.EndsWith("_RL", StringComparison.OrdinalIgnoreCase)
			       || wheel.Name.EndsWith("_RR", StringComparison.OrdinalIgnoreCase);
		}

		internal static float ClampShiftRpmForSlip(float roadRpm, float drivelineRpm, float slipAllowanceRpm)
		{
			var baseRpm = MathF.Max(roadRpm, 0f);
			var maxAllowedRpm = baseRpm + MathF.Max(slipAllowanceRpm, 0f);
			return Math.Clamp(drivelineRpm, baseRpm, maxAllowedRpm);
		}

		internal static float ClampDrivenWheelOmegaForSlip(
			float roadWheelOmega,
			float drivenWheelOmega,
			float effectiveRatio,
			float slipAllowanceRpm)
		{
			var safeRatio = MathF.Max(MathF.Abs(effectiveRatio), 1e-3f);
			var omegaToRpm = safeRatio * (60f / (2f * MathF.PI));
			var clampedRpm = ClampShiftRpmForSlip(
				MathF.Abs(roadWheelOmega) * omegaToRpm,
				MathF.Abs(drivenWheelOmega) * omegaToRpm,
				slipAllowanceRpm);

			var direction = MathF.Abs(drivenWheelOmega) > 0.1f
				? MathF.Sign(drivenWheelOmega)
				: MathF.Abs(roadWheelOmega) > 0.1f
					? MathF.Sign(roadWheelOmega)
					: 0f;

			return direction == 0f ? 0f : direction * (clampedRpm / omegaToRpm);
		}

		internal static float ResolveDrivenWheelOmega(
			float fallbackOmega,
			float forwardSpeed,
			ReadOnlySpan<float> sampledWheelOmegas,
			ReadOnlySpan<bool> sampledWheelGrounded)
		{
			var sampleCount = Math.Min(sampledWheelOmegas.Length, sampledWheelGrounded.Length);
			var groundedOmegaSum = 0f;
			var groundedCount = 0;
			var sampledOmegaSum = 0f;
			var sampledCount = 0;

			for (var i = 0; i < sampleCount; i++)
			{
				var wheelOmega = sampledWheelOmegas[i];
				sampledOmegaSum += wheelOmega;
				sampledCount++;

				if (!sampledWheelGrounded[i])
				{
					continue;
				}

				groundedOmegaSum += wheelOmega;
				groundedCount++;
			}

			if (groundedCount > 0)
			{
				return groundedOmegaSum / groundedCount;
			}

			if (sampledCount > 0 && MathF.Abs(forwardSpeed) > 0.5f)
			{
				return sampledOmegaSum / sampledCount;
			}

			return fallbackOmega;
		}

		internal static float ResolveAutoClutchDrivenWheelOmega(
			float fallbackOmega,
			ReadOnlySpan<float> sampledWheelOmegas,
			ReadOnlySpan<bool> sampledWheelGrounded)
		{
			var sampleCount = Math.Min(sampledWheelOmegas.Length, sampledWheelGrounded.Length);
			var groundedCount = 0;
			var selectedOmega = fallbackOmega;
			var selectedAbsOmega = float.PositiveInfinity;

			for (var i = 0; i < sampleCount; i++)
			{
				if (!sampledWheelGrounded[i])
				{
					continue;
				}

				var wheelOmega = sampledWheelOmegas[i];
				var absOmega = MathF.Abs(wheelOmega);
				if (absOmega < selectedAbsOmega)
				{
					selectedAbsOmega = absOmega;
					selectedOmega = wheelOmega;
				}

				groundedCount++;
			}

			return groundedCount > 0 ? selectedOmega : fallbackOmega;
		}

		internal static float ComputeLowSpeedYawAssistRate(
			float steerRack,
			float forwardSpeed,
			float driveInput,
			float driveDirection,
			float assistGain)
		{
			if (assistGain <= 0f || MathF.Abs(steerRack) < 0.01f)
			{
				return 0f;
			}

			var speedMs = MathF.Abs(forwardSpeed);
			var travelDirection = speedMs > 0.35f
				? MathF.Sign(forwardSpeed)
				: driveInput > 0.05f
					? MathF.Sign(driveDirection)
					: 0f;

			if (travelDirection == 0f)
			{
				return 0f;
			}

			const float AssistFadeInSpeed = 0.75f;
			const float AssistLaunchSpeedBias = 0.2f;
			const float AssistFadeOutSpeed = 4f;
			const float AssistPeakYawRate = 0.25f;

			var fadeIn = Math.Clamp((speedMs + driveInput * AssistLaunchSpeedBias) / AssistFadeInSpeed, 0f, 1f);
			var fadeOut = Math.Clamp(1f - (speedMs / AssistFadeOutSpeed), 0f, 1f);
			var assistBlend = fadeIn * fadeOut * fadeOut;

			return steerRack * travelDirection * assistGain * AssistPeakYawRate * assistBlend;
		}

		internal static float ApplyYawAssistTopUp(float currentYawRate, float targetYawRate, float maxDelta)
		{
			if (MathF.Abs(targetYawRate) < 1e-4f || maxDelta <= 0f)
			{
				return currentYawRate;
			}

			var shortfall = targetYawRate - currentYawRate;
			if (MathF.Abs(shortfall) < 1e-4f || MathF.Sign(shortfall) != MathF.Sign(targetYawRate))
			{
				return currentYawRate;
			}

			return currentYawRate + Math.Clamp(shortfall, -maxDelta, maxDelta);
		}

		private void ClearLiveInputs()
		{
			ThrottleInput = 0f;
			BrakeInput = 0f;
			DriveInput = 0f;
			ServiceBrakeInput = 0f;
			SteeringInput = 0f;
			SteeringRack = 0f;
			HandbrakeEngaged = false;
			TractionLossDetected = false;
			TractionControlActive = false;
			AbsActive = false;
			DrivenWheelSlipRatio = 0f;
			TractionControlTorqueScale = 1f;
			Array.Fill(_absBrakeTorqueScales, 1f);
		}

		private void SetVehicleBodiesAwake(bool awake)
		{
			var chassisBody = CarBody.Get<BodyComponent>();
			if (chassisBody != null)
			{
				chassisBody.Awake = awake;
			}

			foreach (var wheelBody in Wheels.Select(wheel => wheel.Get<BodyComponent>()))
			{
				if (wheelBody != null)
				{
					wheelBody.Awake = awake;
				}
			}
		}

		internal static int ResolveStandingGearSelection(
			int currentGear,
			float speedKmh,
			float forwardSpeed,
			float throttleInput,
			float brakeInput,
			bool handbrakeRequested,
			float shiftCooldown,
			float reverseEngageSpeedKmh)
		{
			if (shiftCooldown > 0f || handbrakeRequested)
			{
				return currentGear;
			}

			var reverseEngageSpeedMs = MathF.Max(reverseEngageSpeedKmh / 3.6f, 0.1f);
			var nearStandstill = speedKmh < reverseEngageSpeedKmh && MathF.Abs(forwardSpeed) < reverseEngageSpeedMs;
			var wantsForward = throttleInput > 0.05f;
			var wantsReverse = brakeInput > 0.05f && throttleInput <= 0.05f;

			if (currentGear == 0)
			{
				return wantsForward && nearStandstill ? 1 : 0;
			}

			if (wantsReverse && nearStandstill)
			{
				return 0;
			}

			// Ensure we don't inappropriately force a downshift to 1 while doing a high-RPM burnout
			if (nearStandstill && currentGear > 1 && !wantsForward)
			{
				return 1;
			}

			return currentGear;
		}

		internal static float ApplyTriggerDeadzone(float value, float deadzone)
		{
			var clampedDeadzone = Math.Clamp(deadzone, 0f, 0.99f);
			var clampedValue = Math.Clamp(value, 0f, 1f);
			if (clampedValue <= clampedDeadzone)
			{
				return 0f;
			}

			return (clampedValue - clampedDeadzone) / (1f - clampedDeadzone);
		}

		internal static float ApplySignedAxisDeadzone(float value, float deadzone)
		{
			var magnitude = ApplyTriggerDeadzone(MathF.Abs(value), deadzone);
			return magnitude == 0f ? 0f : MathF.CopySign(magnitude, value);
		}

		internal static bool ShouldKeepVehicleAwake(
			float throttleInput,
			float brakeInput,
			float steerInput,
			bool handbrakeRequested,
			Vector3 linearVelocity,
			Vector3 angularVelocity)
		{
			if (throttleInput > 0.02f || brakeInput > 0.02f || MathF.Abs(steerInput) > 0.02f || handbrakeRequested)
			{
				return true;
			}

			return linearVelocity.LengthSquared() > SleepLinearSpeedThreshold * SleepLinearSpeedThreshold
			       || angularVelocity.LengthSquared() > SleepAngularSpeedThreshold * SleepAngularSpeedThreshold;
		}

		private float MeasureDrivenWheelOmega(float forwardSpeed)
		{
			var safeWheelRadius = MathF.Max(WheelRadius, 0.05f);
			var fallbackOmega = forwardSpeed / safeWheelRadius;
			if (Dynamics == null || DriveWheels.Count == 0)
			{
				return fallbackOmega;
			}

			Span<float> sampledWheelOmegas = stackalloc float[VehicleDynamicsSystem.WheelCount];
			Span<bool> sampledWheelGrounded = stackalloc bool[VehicleDynamicsSystem.WheelCount];
			var sampledCount = 0;

			foreach (var wheel in DriveWheels)
			{
				var ws = wheel.Get<WheelSettings>();
				var dynamicsIndex = ws?.DynamicsIndex ?? -1;
				if ((uint)dynamicsIndex >= VehicleDynamicsSystem.WheelCount)
				{
					continue;
				}

				sampledWheelOmegas[sampledCount] = Dynamics.WheelStates[dynamicsIndex].AngularVelocity;
				sampledWheelGrounded[sampledCount] = Dynamics.WheelGrounded[dynamicsIndex];
				sampledCount++;

				if (sampledCount >= VehicleDynamicsSystem.WheelCount)
				{
					break;
				}
			}

			return ResolveDrivenWheelOmega(
				fallbackOmega,
				forwardSpeed,
				sampledWheelOmegas[..sampledCount],
				sampledWheelGrounded[..sampledCount]);
		}

		private void MeasureDrivenWheelTractionState(
			float forwardSpeed,
			out float autoClutchDrivenWheelOmega,
			out float maxDrivenWheelSlipRatio)
		{
			maxDrivenWheelSlipRatio = 0f;
			var safeWheelRadius = MathF.Max(WheelRadius, 0.05f);
			var fallbackOmega = forwardSpeed / safeWheelRadius;
			autoClutchDrivenWheelOmega = fallbackOmega;
			if (Dynamics == null || DriveWheels.Count == 0)
			{
				return;
			}

			Span<float> sampledWheelOmegas = stackalloc float[VehicleDynamicsSystem.WheelCount];
			Span<bool> sampledWheelGrounded = stackalloc bool[VehicleDynamicsSystem.WheelCount];
			var sampledCount = 0;

			foreach (var ws in DriveWheels.Select(wheel => wheel.Get<WheelSettings>()))
			{
				var dynamicsIndex = ws?.DynamicsIndex ?? -1;
				if ((uint)dynamicsIndex >= VehicleDynamicsSystem.WheelCount)
				{
					continue;
				}

				sampledWheelOmegas[sampledCount] = Dynamics.WheelStates[dynamicsIndex].AngularVelocity;
				sampledWheelGrounded[sampledCount] = Dynamics.WheelGrounded[dynamicsIndex];
				sampledCount++;

				if (!Dynamics.WheelGrounded[dynamicsIndex])
				{
					continue;
				}

				maxDrivenWheelSlipRatio = MathF.Max(maxDrivenWheelSlipRatio, MathF.Abs(Dynamics.WheelStates[dynamicsIndex].SlipRatio));

				if (sampledCount >= VehicleDynamicsSystem.WheelCount)
				{
					break;
				}
			}

			autoClutchDrivenWheelOmega = ResolveAutoClutchDrivenWheelOmega(
				fallbackOmega,
				sampledWheelOmegas[..sampledCount],
				sampledWheelGrounded[..sampledCount]);
		}

		internal static Vector3 ComputePointVelocity(Vector3 linearVelocity, Vector3 angularVelocity, Vector3 pointOffset)
		{
			return linearVelocity + Vector3.Cross(angularVelocity, pointOffset);
		}

		internal static float ConvertCamberPrecompressionToRadians(float precompression)
		{
			if (!float.IsFinite(precompression))
			{
				return 0f;
			}

			return Math.Clamp(
				(precompression - 1f) * CamberRadiansPerPrecompressionUnit,
				-MaxStaticCamberFromAlignmentRadians,
				MaxStaticCamberFromAlignmentRadians);
		}

		internal static float ComputeAlignmentCamberAngle(float staticCamberRadians, float camberGainPerMeter, float suspensionCompressionMeters)
		{
			if (!float.IsFinite(staticCamberRadians) || !float.IsFinite(camberGainPerMeter) || !float.IsFinite(suspensionCompressionMeters))
			{
				return 0f;
			}

			return Math.Clamp(
				staticCamberRadians + camberGainPerMeter * suspensionCompressionMeters,
				-MaxDynamicAlignmentCamberRadians,
				MaxDynamicAlignmentCamberRadians);
		}

		internal static float ComputeCamberSideSign(float wheelSideDot)
		{
			if (!float.IsFinite(wheelSideDot))
			{
				return 1f;
			}

			return wheelSideDot < 0f ? -1f : 1f;
		}

		internal static float ComputeAutoClutchTorqueScale(
			float drivenWheelOmega,
			float slipClampedDrivenWheelOmega,
			float effectiveRatio,
			float wheelspinWindowRpm,
			float minTorqueScale)
		{
			var clampedMinScale = Math.Clamp(minTorqueScale, 0f, 1f);
			var safeRatio = MathF.Max(MathF.Abs(effectiveRatio), 1e-3f);
			var safeWindowRpm = MathF.Max(wheelspinWindowRpm, 1f);
			var omegaToRpm = safeRatio * (60f / (2f * MathF.PI));
			var excessWheelOmega = MathF.Max(0f, MathF.Abs(drivenWheelOmega) - MathF.Abs(slipClampedDrivenWheelOmega));
			var excessSlipRpm = excessWheelOmega * omegaToRpm;
			var slipBlend = Math.Clamp(excessSlipRpm / safeWindowRpm, 0f, 1f);
			return 1f + (clampedMinScale - 1f) * slipBlend;
		}

		internal static float ComputeTractionControlTorqueScale(
			float drivenWheelSlipRatio,
			float slipRatioTarget,
			float slipRatioWindow,
			float minTorqueScale)
		{
			var target = Math.Clamp(slipRatioTarget, 0.02f, 0.5f);
			var window = MathF.Max(slipRatioWindow, 0.02f);
			var clampedMinScale = Math.Clamp(minTorqueScale, 0f, 1f);
			var excessSlip = MathF.Max(0f, MathF.Abs(drivenWheelSlipRatio) - target);
			var slipBlend = Math.Clamp(excessSlip / window, 0f, 1f);
			return 1f + (clampedMinScale - 1f) * slipBlend;
		}

		internal static float ComputeAbsBrakeTorqueScale(
			float slipRatio,
			float rollingDirection,
			float slipRatioTarget,
			float slipRatioWindow,
			float minBrakeScale)
		{
			if (MathF.Abs(rollingDirection) < 0.5f)
			{
				return 1f;
			}

			var target = Math.Clamp(slipRatioTarget, 0.02f, 0.5f);
			var window = MathF.Max(slipRatioWindow, 0.02f);
			var clampedMinScale = Math.Clamp(minBrakeScale, 0f, 1f);
			var brakingSlip = MathF.Max(0f, -slipRatio * MathF.Sign(rollingDirection));
			var excessSlip = MathF.Max(0f, brakingSlip - target);
			var slipBlend = Math.Clamp(excessSlip / window, 0f, 1f);
			return 1f + (clampedMinScale - 1f) * slipBlend;
		}

		internal static float ResolveRollingDirection(float forwardSpeed, float wheelAngularVelocity)
		{
			if (MathF.Abs(forwardSpeed) > 0.5f)
			{
				return MathF.Sign(forwardSpeed);
			}

			if (MathF.Abs(wheelAngularVelocity) > 0.5f)
			{
				return MathF.Sign(wheelAngularVelocity);
			}

			return 0f;
		}

		private static float AdvanceControllerScale(float currentScale, float targetScale, float responseRate, float dt)
		{
			if (dt <= 0f)
			{
				return targetScale;
			}

			var blend = Math.Clamp(responseRate * dt, 0f, 1f);
			return currentScale + (targetScale - currentScale) * blend;
		}

		private static float MeasureSteerAngle(Matrix chassisWorld, Matrix wheelWorld)
		{
			var up = chassisWorld.Up;
			var upLengthSq = up.LengthSquared();
			if (upLengthSq < 1e-6f)
			{
				return 0f;
			}

			up /= MathF.Sqrt(upLengthSq);

			var chassisRight = ProjectOnPlane(chassisWorld.Right, up);
			var wheelRight = ProjectOnPlane(wheelWorld.Right, up);
			var chassisRightLengthSq = chassisRight.LengthSquared();
			var wheelRightLengthSq = wheelRight.LengthSquared();
			if (chassisRightLengthSq < 1e-6f || wheelRightLengthSq < 1e-6f)
			{
				return 0f;
			}

			chassisRight /= MathF.Sqrt(chassisRightLengthSq);
			wheelRight /= MathF.Sqrt(wheelRightLengthSq);

			var sin = Vector3.Dot(Vector3.Cross(chassisRight, wheelRight), up);
			var cos = Math.Clamp(Vector3.Dot(chassisRight, wheelRight), -1f, 1f);

			// Positive steer angle corresponds to right lock, matching the steering servo target.
			return -MathF.Atan2(sin, cos);
		}

		private static Vector3 MeasureWheelAxleAxisLocalA(Matrix bodyAWorld, Matrix wheelWorld)
		{
			var wheelAxleWorld = wheelWorld.Right;
			var axleLengthSq = wheelAxleWorld.LengthSquared();
			if (axleLengthSq < 1e-6f)
			{
				return Vector3.UnitX;
			}

			wheelAxleWorld /= MathF.Sqrt(axleLengthSq);

			Vector3 axisLocalA = new(
				Vector3.Dot(wheelAxleWorld, bodyAWorld.Right),
				Vector3.Dot(wheelAxleWorld, bodyAWorld.Up),
				Vector3.Dot(wheelAxleWorld, bodyAWorld.Backward));

			var axisLengthSq = axisLocalA.LengthSquared();
			if (axisLengthSq < 1e-6f)
			{
				return Vector3.UnitX;
			}

			return axisLocalA / MathF.Sqrt(axisLengthSq);
		}

		private static float ProbeWheelContactScale(
			BodyComponent chassisBody,
			BodyComponent wheelBody,
			WheelSettings wheelSettings,
			Vector3 wheelPosition,
			Vector3 wheelUp,
			out Vector3 contactPoint)
		{
			var simulation = wheelBody.Simulation;
			var tyreModel = wheelSettings.TyreModel;
			if (simulation == null || tyreModel == null)
			{
				contactPoint = wheelPosition - wheelUp * 0.35f;
				return 0f;
			}

			var staticNormalLoad = Math.Max(wheelSettings.StaticNormalLoad, 0f);
			if (staticNormalLoad <= 0f)
			{
				contactPoint = wheelPosition;
				return 0f;
			}

			var wheelRadius = Math.Max(tyreModel.Radius, 0.1f);
			contactPoint = wheelPosition - wheelUp * wheelRadius;
			var rayOrigin = wheelPosition + wheelUp * (wheelRadius + GroundProbeMargin);
			var rayDirection = -wheelUp;
			var rayLength = wheelRadius * 2f + GroundProbeMargin * 2f;

			wheelSettings.GroundProbeHits.Clear();
			simulation.RayCastPenetrating(in rayOrigin, in rayDirection, rayLength, wheelSettings.GroundProbeHits);

			var primaryDistance = float.PositiveInfinity;
			var secondaryDistance = float.PositiveInfinity;
			var primarySurfaceType = SurfaceType.Tarmac;
			var primarySurfaceProperties = SurfaceProperties.ForType(SurfaceType.Tarmac);
			var secondarySurfaceProperties = primarySurfaceProperties;
			foreach (var hit in wheelSettings.GroundProbeHits)
			{
				if (hit.Collidable == null || hit.Collidable == wheelBody || hit.Collidable == chassisBody)
				{
					continue;
				}

				ResolveSurfaceProperties(hit.Collidable, out var hitSurfaceType, out var hitSurfaceProperties);
				if (hit.Distance < primaryDistance)
				{
					secondaryDistance = primaryDistance;
					secondarySurfaceProperties = primarySurfaceProperties;
					primaryDistance = hit.Distance;
					primarySurfaceType = hitSurfaceType;
					primarySurfaceProperties = hitSurfaceProperties;
					contactPoint = rayOrigin + rayDirection * hit.Distance;
				}
				else if (hit.Distance < secondaryDistance)
				{
					secondaryDistance = hit.Distance;
					secondarySurfaceProperties = hitSurfaceProperties;
				}
			}

			if (!float.IsFinite(primaryDistance))
			{
				wheelSettings.CurrentSurface = SurfaceType.Tarmac;
				wheelSettings.CurrentSurfaceProperties = SurfaceProperties.ForType(SurfaceType.Tarmac);
				return 0f;
			}

			var blendFactor = ComputeSurfaceBlendFactor(primaryDistance, secondaryDistance, SurfaceBlendDistance);
			wheelSettings.CurrentSurface = primarySurfaceType;
			wheelSettings.CurrentSurfaceProperties = blendFactor > 0f
				? SurfaceProperties.Lerp(primarySurfaceProperties, secondarySurfaceProperties, blendFactor)
				: primarySurfaceProperties;
			return ComputeGroundProbeContactScale(primaryDistance, wheelRadius, GroundProbeMargin);
		}

		private static void ResolveSurfaceProperties(
			object collidable,
			out SurfaceType surfaceType,
			out SurfaceProperties surfaceProperties)
		{
			if (collidable is EntityComponent entityComponent)
			{
				var surfaceTag = entityComponent.Entity.Get<TrackSurfaceComponent>();
				if (surfaceTag != null)
				{
					surfaceType = surfaceTag.SurfaceType;
					surfaceProperties = surfaceTag.ResolveSurfaceProperties();
					return;
				}
			}

			surfaceType = SurfaceType.Tarmac;
			surfaceProperties = SurfaceProperties.ForType(surfaceType);
		}

		internal static float ComputeSurfaceBlendFactor(float primaryDistance, float secondaryDistance, float blendDistance)
		{
			if (!float.IsFinite(primaryDistance) ||
			    !float.IsFinite(secondaryDistance) ||
			    secondaryDistance <= primaryDistance)
			{
				return 0f;
			}

			var safeBlendDistance = MathF.Max(blendDistance, 1e-4f);
			var distanceDelta = secondaryDistance - primaryDistance;
			if (distanceDelta >= safeBlendDistance)
			{
				return 0f;
			}

			var primaryWeight = 1f / MathF.Max(primaryDistance, 1e-3f);
			var secondaryWeight = 1f / MathF.Max(secondaryDistance, 1e-3f);
			var inverseDistanceBlend = secondaryWeight / MathF.Max(primaryWeight + secondaryWeight, 1e-6f);
			var transitionWeight = 1f - distanceDelta / safeBlendDistance;
			return Math.Clamp(inverseDistanceBlend * transitionWeight, 0f, 0.5f);
		}

		internal static float ComputeGroundProbeContactScale(float hitDistance, float wheelRadius, float probeMargin)
		{
			var safeRadius = MathF.Max(wheelRadius, 0.1f);
			var safeMargin = MathF.Max(probeMargin, 0.01f);
			var nominalContactDistance = safeRadius * 2f + safeMargin;
			var maxContactDistance = nominalContactDistance + safeMargin;
			var normalizedDistance = Math.Clamp((hitDistance - nominalContactDistance) / (maxContactDistance - nominalContactDistance), 0f, 1f);
			return 1f - normalizedDistance;
		}

		internal static float ComputeWheelSurfaceVfxIntensity(
			SurfaceType surfaceType,
			float slipRatio,
			float slipAngleRadians,
			float normalLoadScale,
			float contactScale)
		{
			if (contactScale <= 0f || normalLoadScale <= 0f)
			{
				return 0f;
			}

			var absSlipSignal = MathF.Max(MathF.Abs(slipRatio), MathF.Abs(slipAngleRadians) * WheelSurfaceVfxSlipAngleWeight);
			float surfaceIntensity = surfaceType switch
			{
				SurfaceType.Tarmac or SurfaceType.WetTarmac => ComputeWheelSurfaceVfxSlipRamp(absSlipSignal, start: TarmacVfxSlipRampStart, full: TarmacVfxSlipRampFull),
				SurfaceType.Gravel => ComputeWheelSurfaceVfxSlipRamp(absSlipSignal, start: GravelVfxSlipRampStart, full: GravelVfxSlipRampFull),
				SurfaceType.Snow => ComputeWheelSurfaceVfxSlipRamp(absSlipSignal, start: SnowVfxSlipRampStart, full: SnowVfxSlipRampFull),
				_ => 0f,
			};

			var clampedLoad = Math.Clamp(normalLoadScale, 0f, WheelSurfaceVfxMaxLoadScale);
			var clampedContact = Math.Clamp(contactScale, 0f, 1f);
			return Math.Clamp(surfaceIntensity * clampedLoad * clampedContact, 0f, WheelSurfaceVfxIntensityClampMax);
		}

		private static float ComputeWheelSurfaceVfxSlipRamp(float slipSignal, float start, float full)
		{
			var safeRange = MathF.Max(full - start, WheelSurfaceVfxMinSlipRampRange);
			return Math.Clamp((slipSignal - start) / safeRange, 0f, 1f);
		}

		private void InitializeWheelSurfaceVfx()
		{
			if (_wheelSurfaceVfxInitialized || CarBody == null)
			{
				return;
			}

			for (var i = 0; i < Wheels.Count && i < VehicleDynamicsSystem.WheelCount; i++)
			{
				var wheelSurfaceVfxEntity = new Entity($"WheelSurfaceVfx_{i}");
				var wheelSurfaceVfxSystem = CreateWheelSurfaceParticleSystem(out var wheelSurfaceVfxSpawner);
				wheelSurfaceVfxEntity.Add(wheelSurfaceVfxSystem);
				CarBody.AddChild(wheelSurfaceVfxEntity);
				_wheelSurfaceVfxEntities[i] = wheelSurfaceVfxEntity;
				_wheelSurfaceVfxComponents[i] = wheelSurfaceVfxSystem;
				_wheelSurfaceVfxSpawners[i] = wheelSurfaceVfxSpawner;
			}

			_wheelSurfaceVfxInitialized = true;
		}

		private static ParticleSystemComponent CreateWheelSurfaceParticleSystem(out SpawnerPerSecond spawner)
		{
			var particleSystemComponent = new ParticleSystemComponent();
			var particleSystem = particleSystemComponent.ParticleSystem;
			var emitter = new ParticleEmitter
			{
				EmitterName = "WheelSurfaceVfx",
				MaxParticlesOverride = 180,
				ParticleLifetime = new Vector2(0.35f, 0.95f),
				SimulationSpace = EmitterSimulationSpace.World,
				ShapeBuilder = new ShapeBuilderBillboard(),
				Material = new ParticleMaterialComputeColor(),
			};

			spawner = new SpawnerPerSecond { SpawnCount = 0f };
			emitter.Spawners.Add(spawner);
			emitter.Initializers.Add(new InitialPositionSeed
			{
				PositionMin = new Vector3(WheelSurfaceVfxPositionMinX, WheelSurfaceVfxPositionMinY, WheelSurfaceVfxPositionMinZ),
				PositionMax = new Vector3(WheelSurfaceVfxPositionMaxX, WheelSurfaceVfxPositionMaxY, WheelSurfaceVfxPositionMaxZ),
			});
			emitter.Initializers.Add(new InitialVelocitySeed
			{
				VelocityMin = new Vector3(WheelSurfaceVfxVelocityMinX, WheelSurfaceVfxVelocityMinY, WheelSurfaceVfxVelocityMinZ),
				VelocityMax = new Vector3(WheelSurfaceVfxVelocityMaxX, WheelSurfaceVfxVelocityMaxY, WheelSurfaceVfxVelocityMaxZ),
			});
			emitter.Initializers.Add(new InitialSizeSeed
			{
				RandomSize = new Vector2(WheelSurfaceVfxSizeMin, WheelSurfaceVfxSizeMax),
			});
			emitter.Updaters.Add(new UpdaterSpeedToDirection());
			emitter.Updaters.Add(new UpdaterGravity
			{
				GravitationalAcceleration = new Vector3(0f, -0.25f, 0f),
			});
			particleSystem.Emitters.Add(emitter);
			particleSystemComponent.Speed = 1f;
			particleSystemComponent.Color = TransparentColor;
			return particleSystemComponent;
		}

		private void UpdateWheelSurfaceVfx(float dt, in Matrix chassisWorld)
		{
			if (!_wheelSurfaceVfxInitialized)
			{
				return;
			}

			var dynamics = Dynamics;
			var inverseChassisWorld = Matrix.Invert(chassisWorld);

			for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
			{
				var wheelSurfaceVfxEntity = _wheelSurfaceVfxEntities[i];
				var wheelSurfaceVfxComponent = _wheelSurfaceVfxComponents[i];
				var wheelSurfaceVfxSpawner = _wheelSurfaceVfxSpawners[i];
				if (wheelSurfaceVfxEntity == null || wheelSurfaceVfxComponent == null || wheelSurfaceVfxSpawner == null)
				{
					continue;
				}

				var targetIntensity = 0f;
				var wheelSurface = SurfaceType.Tarmac;
				if (dynamics != null && i < Wheels.Count)
				{
					var wheel = Wheels[i];
					var wheelSettings = wheel.Get<WheelSettings>();
					wheelSurface = wheelSettings?.CurrentSurface ?? SurfaceType.Tarmac;
					var dynamicsIndex = wheelSettings?.DynamicsIndex >= 0 ? wheelSettings.DynamicsIndex : i;
					if (IsValidWheelDynamicsIndex(dynamicsIndex, dynamics) &&
					    dynamics.WheelGrounded[dynamicsIndex])
					{
						var wheelState = dynamics.WheelStates[dynamicsIndex];
						var staticLoad = MathF.Max(dynamics.StaticNormalLoads[dynamicsIndex], 1f);
						var normalLoadScale = dynamics.CurrentNormalLoads[dynamicsIndex] / staticLoad;
						targetIntensity = ComputeWheelSurfaceVfxIntensity(
							wheelSurface,
							wheelState.SlipRatio,
							wheelState.SlipAngle,
							normalLoadScale,
							_wheelContactScales[i]);
					}

					var tyreRadius = wheelSettings?.TyreModel?.Radius ?? WheelRadius;
					var wheelSurfacePosition = ComputeWheelSurfaceVfxPosition(i, tyreRadius);
					wheelSurfaceVfxEntity.Transform.Position = Vector3.TransformCoordinate(wheelSurfacePosition, inverseChassisWorld);
				}

				var responseRate = targetIntensity > _wheelSurfaceVfxIntensity[i]
					? WheelSurfaceVfxRiseRate
					: WheelSurfaceVfxFallRate;
				_wheelSurfaceVfxIntensity[i] = AdvanceControllerScale(_wheelSurfaceVfxIntensity[i], targetIntensity, responseRate, dt);
				wheelSurfaceVfxSpawner.SpawnCount = _wheelSurfaceVfxIntensity[i] * WheelSurfaceVfxMaxSpawnRate;
				wheelSurfaceVfxComponent.Color = ResolveWheelSurfaceVfxColor(wheelSurface, _wheelSurfaceVfxIntensity[i]);
			}
		}

		private static Color4 ResolveWheelSurfaceVfxColor(SurfaceType surfaceType, float intensity)
		{
			var clampedIntensity = Math.Clamp(intensity, 0f, 1f);
			if (clampedIntensity <= 0f)
			{
				return TransparentColor;
			}

			return surfaceType switch
			{
				SurfaceType.Gravel => new Color4(GravelVfxRed, GravelVfxGreen, GravelVfxBlue, GravelVfxBaseAlpha + clampedIntensity * GravelVfxIntensityAlphaScale),
				SurfaceType.Tarmac or SurfaceType.WetTarmac => new Color4(TarmacVfxRed, TarmacVfxGreen, TarmacVfxBlue, TarmacVfxBaseAlpha + clampedIntensity * TarmacVfxIntensityAlphaScale),
				SurfaceType.Snow => new Color4(SnowVfxRed, SnowVfxGreen, SnowVfxBlue, SnowVfxBaseAlpha + clampedIntensity * SnowVfxIntensityAlphaScale),
				_ => TransparentColor,
			};
		}

		private static bool IsValidWheelDynamicsIndex(int dynamicsIndex, VehicleDynamicsSystem dynamics)
		{
			return dynamicsIndex >= 0 &&
			       dynamicsIndex < dynamics.WheelStates.Length &&
			       dynamicsIndex < dynamics.CurrentNormalLoads.Length &&
			       dynamicsIndex < dynamics.StaticNormalLoads.Length &&
			       dynamicsIndex < dynamics.WheelGrounded.Length;
		}

		private Vector3 ComputeWheelSurfaceVfxPosition(int wheelIndex, float tyreRadius)
		{
			if (_wheelContactScales[wheelIndex] > MinimumWheelContactPointScale)
			{
				return _wheelContactPoints[wheelIndex];
			}

			var wheelUp = _wheelOrientations[wheelIndex].Up;
			var verticalOffset = MathF.Max(tyreRadius * WheelSurfaceVfxContactPatchRadiusScale, WheelSurfaceVfxMinVerticalOffset);
			return _wheelPositions[wheelIndex] - wheelUp * verticalOffset;
		}

		private static float MeasureSuspensionCompression(
			in Matrix chassisWorld,
			in Matrix wheelWorld,
			WheelSettings wheelSettings,
			Vector3 suspensionAxisWorld)
		{
			var suspensionAnchorA = Vector3.TransformCoordinate(wheelSettings.SuspensionLocalOffsetA, chassisWorld);
			var suspensionAnchorB = Vector3.TransformCoordinate(wheelSettings.SuspensionLocalOffsetB, wheelWorld);
			var axisOffset = Vector3.Dot(suspensionAnchorB - suspensionAnchorA, suspensionAxisWorld);

			// Positive compression means the wheel has moved toward the chassis along the suspension axis.
			// This matches the sign convention used by spring, damper, anti-roll, and telemetry code.
			return axisOffset - wheelSettings.SuspensionTargetOffset;
		}

		private static Vector3 ProjectOnPlane(Vector3 vector, Vector3 planeNormal)
		{
			return vector - planeNormal * Vector3.Dot(vector, planeNormal);
		}

		private static Vector3 TransformDirection(in Matrix world, Vector3 localDirection, Vector3 fallback)
		{
			var transformed =
				world.Right * localDirection.X +
				world.Up * localDirection.Y +
				world.Backward * localDirection.Z;

			return SafeNormalize(transformed, fallback);
		}

		private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
		{
			var lengthSquared = value.LengthSquared();
			if (lengthSquared < 1e-6f)
			{
				return fallback;
			}

			return value / MathF.Sqrt(lengthSquared);
		}
	}
}
