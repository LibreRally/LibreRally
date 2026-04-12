using System;
using System.Collections.Generic;
using System.Linq;
using LibreRally.Vehicle.Physics;
using Stride.BepuPhysics;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;

namespace LibreRally.Vehicle;

[ComponentCategory("LibreRally")]
public class RallyCarComponent : SyncScript
{
    private const float GroundProbeMargin = 0.05f;
    private const float GamePadTriggerDeadzone = 0.08f;
    private const float GamePadSteerDeadzone = 0.12f;
    private const float SleepLinearSpeedThreshold = 0.10f;
    private const float SleepAngularSpeedThreshold = 0.10f;

    public Entity CarBody { get; set; } = new();
    public List<Entity> Wheels { get; set; } = new();
    public List<Entity> SteerWheels { get; set; } = new();
    public List<Entity> DriveWheels { get; set; } = new();
    public List<Entity> BreakWheels { get; set; } = new();

    /// <summary>
    /// Vehicle dynamics system computing tyre forces, load transfer, differentials,
    /// and anti-roll bar effects. Created during <see cref="Start"/> and updated each frame.
    /// </summary>
    public VehicleDynamicsSystem? Dynamics { get; set; }

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

    /// <summary>Legacy grip tuning value, now used as the soft-body tyre model's low-speed damping gain.</summary>
    public float LateralGrip { get; set; } = 6f;

    // ── Drivetrain ───────────────────────────────────────────────────────────

    /// <summary>Forward gear ratios indexed 1–N (index 0 = reverse).</summary>
    public float[] GearRatios { get; set; } = { 3.25f, 3.64f, 2.38f, 1.76f, 1.35f, 1.06f, 0.84f };

    /// <summary>Final drive (differential) ratio.</summary>
    public float FinalDrive { get; set; } = 4.55f;

    /// <summary>RPM axis of the engine torque curve.</summary>
    public float[] TorqueCurveRpm { get; set; } =
        {    0,  500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000, 7500 };

    /// <summary>Torque (N·m at crank) corresponding to each RPM in <see cref="TorqueCurveRpm"/>.</summary>
    public float[] TorqueCurveNm { get; set; } =
        {    0,   62,  105,  142,  176,  198,  210,  216,  221,  222,  221,  218,  210,  199,  186,  168 };

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

    /// <summary>Engine-RPM-equivalent driven-wheel excess spin window before TCS reaches minimum torque.</summary>
    public float TractionControlWheelspinWindowRpm { get; set; } = 800f;

    /// <summary>Minimum fraction of engine torque that TCS still allows through when fully active.</summary>
    public float TractionControlMinTorqueScale { get; set; } = 0.08f;

    /// <summary>Driven-wheel slip ratio above which the HUD traction warning lamp turns on.</summary>
    public float TractionLossSlipThreshold { get; set; } = 0.18f;

    /// <summary>RPM to upshift at (auto-transmission).</summary>
    public float ShiftUpRpm { get; set; } = 6500f;

    /// <summary>RPM to downshift at (auto-transmission).</summary>
    public float ShiftDownRpm { get; set; } = 2200f;

    /// <summary>Maximum extra RPM that driven-wheel slip may contribute above road-speed RPM for shifting and torque coupling.</summary>
    public float ShiftSlipAllowanceRpm { get; set; } = 900f;

    /// <summary>Vehicle speed below which holding brake can select reverse gear.</summary>
    public float ReverseEngageSpeedKmh { get; set; } = 1.5f;

    // ── Read-only telemetry ──────────────────────────────────────────────────
    public float SpeedKmh { get; private set; }
    public float ForwardSpeedMs { get; private set; }
    public float LateralSpeedMs { get; private set; }
    public float YawRateRad { get; private set; }
    public float EngineRpm { get; private set; }
    public float ThrottleInput { get; private set; }
    public float BrakeInput { get; private set; }
    public float DriveInput { get; private set; }
    public float ServiceBrakeInput { get; private set; }
    public float SteeringInput { get; private set; }
    public float SteeringRack { get; private set; }
    public float DrivelineRpm { get; private set; }
    public bool HandbrakeEngaged { get; private set; }
    public bool TractionLossDetected { get; private set; }
    public bool TractionControlActive { get; private set; }
    public float DrivenWheelSlipRatio { get; private set; }
    public float TractionControlTorqueScale { get; private set; } = 1f;
    public int CurrentGear { get; private set; } = 1;

    public float DriveRatio => EffectiveRatio;

    private float EffectiveRatio => GearRatios[Math.Clamp(CurrentGear, 0, GearRatios.Length - 1)] * FinalDrive;
    private float _shiftCooldown;

    // Simulated engine RPM — the engine drives the drivetrain, not the other way around.
    private float _engineRpm;

    // ── Cached arrays for VehicleDynamicsSystem (avoid per-frame allocations) ─
    private readonly Vector3[] _wheelPositions = new Vector3[VehicleDynamicsSystem.WheelCount];
    private readonly Vector3[] _wheelVelocities = new Vector3[VehicleDynamicsSystem.WheelCount];
    private readonly Matrix[] _wheelOrientations = new Matrix[VehicleDynamicsSystem.WheelCount];
    private readonly float[] _wheelContactScales = new float[VehicleDynamicsSystem.WheelCount];
    private readonly bool[] _wheelGrounded = new bool[VehicleDynamicsSystem.WheelCount];
    private readonly float[] _suspensionCompressions = new float[VehicleDynamicsSystem.WheelCount];
    private readonly float[] _brakeTorques = new float[VehicleDynamicsSystem.WheelCount];
    private readonly float[] _camberAngles = new float[VehicleDynamicsSystem.WheelCount];

    public override void Start()
    {
        _engineRpm = IdleRpm;

        // Create default dynamics system if none was injected by VehicleLoader
        Dynamics ??= new VehicleDynamicsSystem();
    }

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
        var pad = Input.GamePads.FirstOrDefault();

        float throttle = 0f, brake = 0f, steer = 0f;
        var handbrakeRequested = false;

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
        MeasureDrivenWheelTractionState(out var maxDrivenWheelOmega, out var drivenWheelSlipRatio);
        DrivenWheelSlipRatio = drivenWheelSlipRatio;
        TractionLossDetected = driveInput > 0.05f && drivenWheelSlipRatio >= TractionLossSlipThreshold;
        var tractionControlScale = driveInput > 0.05f && TractionControlEnabled && !isHandbrake
            ? ComputeTractionControlTorqueScale(
                maxDrivenWheelOmega,
                roadWheelOmega,
                effectiveRatio,
                TractionControlWheelspinWindowRpm,
                TractionControlMinTorqueScale)
            : 1f;
        TractionControlTorqueScale = tractionControlScale;
        TractionControlActive = TractionControlEnabled && tractionControlScale < 0.98f;
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
        var crankTorqueScale = driveInput > 0.05f
            ? ComputeAutoClutchTorqueScale(
                drivenWheelOmega,
                slipClampedDrivenWheelOmega,
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
        if (Dynamics != null)
        {
            // Gather per-wheel data into cached arrays (zero allocation)
            var wheelTorqueAtCrank = MathF.Max(0f, crankTorque) * effectiveRatio;
            GatherWheelData(chassisBody, chassisTransform.WorldMatrix);

            // Compute brake torque for dynamics system
            var serviceBrakeTorque = BrakeMotorForce * WheelRadius * serviceBrakeInput;
            var handbrakeTorque = BrakeMotorForce * WheelRadius * HandbrakeForceMultiplier;
            for (var i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
            {
                _brakeTorques[i] = isBraking ? serviceBrakeTorque : 0f;
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

            Dynamics.Update(
                chassisBody,
                in chassisWorld,
                _wheelPositions,
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
    }

    /// <summary>
    /// Gathers per-wheel position, velocity, orientation, and ground contact data
    /// into cached arrays for the <see cref="VehicleDynamicsSystem"/>.
    /// </summary>
    private void GatherWheelData(BodyComponent chassisBody, Matrix chassisWorld)
    {
        var fallbackRight = SafeNormalize(chassisWorld.Right, Vector3.UnitX);
        var fallbackUp = SafeNormalize(chassisWorld.Up, Vector3.UnitY);
        var fallbackLongitudinal = SafeNormalize(chassisWorld.Backward, Vector3.UnitZ);

        for (var i = 0; i < Wheels.Count && i < VehicleDynamicsSystem.WheelCount; i++)
        {
            var wheel = Wheels[i];
            var wheelBody = wheel.Get<BodyComponent>();
            var wheelSettings = wheel.Get<WheelSettings>();

            if (wheelBody == null || wheelSettings == null)
            {
                _wheelContactScales[i] = 0f;
                _wheelGrounded[i] = false;
                _suspensionCompressions[i] = 0f;
                _camberAngles[i] = 0f;
                continue;
            }

            wheel.Transform.UpdateWorldMatrix();
            var wheelWorld = wheel.Transform.WorldMatrix;

            // Wheel coordinate frame (excluding spin rotation)
            var wheelRight = SafeNormalize(wheelWorld.Right, fallbackRight);
            var nonSpinUp = ProjectOnPlane(fallbackUp, wheelRight);
            var wheelUp = SafeNormalize(nonSpinUp, fallbackUp);
            var wheelFwd = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), fallbackLongitudinal);

            _wheelPositions[i] = wheelWorld.TranslationVector;

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
            _suspensionCompressions[i] = MeasureSuspensionCompression(chassisWorld, wheelWorld, wheelSettings, fallbackUp);

            // Ground probe for contact detection. Use a soft contact scale so the tyre model
            // fades load out near contact loss instead of flipping between full static load and zero.
            var contactScale = ProbeWheelContactScale(chassisBody, wheelBody, wheelSettings, _wheelPositions[i], wheelUp);
            _wheelContactScales[i] = contactScale;
            _wheelGrounded[i] = contactScale > 0.05f;

            // Camber is currently zero (vertical wheels) — could be extended from constraint geometry
            _camberAngles[i] = 0f;
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
        DrivenWheelSlipRatio = 0f;
        TractionControlTorqueScale = 1f;
    }

    private void SetVehicleBodiesAwake(bool awake)
    {
        var chassisBody = CarBody.Get<BodyComponent>();
        if (chassisBody != null)
        {
            chassisBody.Awake = awake;
        }

        foreach (var wheel in Wheels)
        {
            var wheelBody = wheel.Get<BodyComponent>();
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

    private void MeasureDrivenWheelTractionState(out float maxDrivenWheelOmega, out float maxDrivenWheelSlipRatio)
    {
        maxDrivenWheelOmega = 0f;
        maxDrivenWheelSlipRatio = 0f;
        if (Dynamics == null || DriveWheels.Count == 0)
        {
            return;
        }

        foreach (var wheel in DriveWheels)
        {
            var ws = wheel.Get<WheelSettings>();
            var dynamicsIndex = ws?.DynamicsIndex ?? -1;
            if ((uint)dynamicsIndex >= VehicleDynamicsSystem.WheelCount || !Dynamics.WheelGrounded[dynamicsIndex])
            {
                continue;
            }

            maxDrivenWheelOmega = MathF.Max(maxDrivenWheelOmega, MathF.Abs(Dynamics.WheelStates[dynamicsIndex].AngularVelocity));
            maxDrivenWheelSlipRatio = MathF.Max(maxDrivenWheelSlipRatio, MathF.Abs(Dynamics.WheelStates[dynamicsIndex].SlipRatio));
        }
    }

    internal static Vector3 ComputePointVelocity(Vector3 linearVelocity, Vector3 angularVelocity, Vector3 pointOffset)
    {
        return linearVelocity + Vector3.Cross(angularVelocity, pointOffset);
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
        float maxDrivenWheelOmega,
        float roadWheelOmega,
        float effectiveRatio,
        float wheelspinWindowRpm,
        float minTorqueScale)
    {
        var clampedMinScale = Math.Clamp(minTorqueScale, 0f, 1f);
        var safeRatio = MathF.Max(MathF.Abs(effectiveRatio), 1e-3f);
        var safeWindowRpm = MathF.Max(wheelspinWindowRpm, 1f);
        var omegaToRpm = safeRatio * (60f / (2f * MathF.PI));
        var excessSlipRpm = MathF.Max(0f, (MathF.Abs(maxDrivenWheelOmega) - MathF.Abs(roadWheelOmega)) * omegaToRpm);
        var slipBlend = Math.Clamp(excessSlipRpm / safeWindowRpm, 0f, 1f);
        return 1f + (clampedMinScale - 1f) * slipBlend;
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
        Vector3 wheelUp)
    {
        var simulation = wheelBody.Simulation;
        var tyreModel = wheelSettings.TyreModel;
        if (simulation == null || tyreModel == null)
        {
	        return 0f;
        }

        var staticNormalLoad = Math.Max(wheelSettings.StaticNormalLoad, 0f);
        if (staticNormalLoad <= 0f)
        {
	        return 0f;
        }

        var wheelRadius = Math.Max(tyreModel.Radius, 0.1f);
        var rayOrigin = wheelPosition + wheelUp * (wheelRadius + GroundProbeMargin);
        var rayDirection = -wheelUp;
        var rayLength = wheelRadius * 2f + GroundProbeMargin * 2f;

        wheelSettings.GroundProbeHits.Clear();
        simulation.RayCastPenetrating(in rayOrigin, in rayDirection, rayLength, wheelSettings.GroundProbeHits, CollisionMask.Everything);

        var bestDistance = float.PositiveInfinity;
        foreach (var hit in wheelSettings.GroundProbeHits)
        {
            if (hit.Collidable == null || hit.Collidable == wheelBody || hit.Collidable == chassisBody)
            {
	            continue;
            }

            bestDistance = MathF.Min(bestDistance, hit.Distance);
        }

        if (!float.IsFinite(bestDistance))
        {
	        return 0f;
        }

        return ComputeGroundProbeContactScale(bestDistance, wheelRadius, GroundProbeMargin);
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

    private static float MeasureSuspensionCompression(
        in Matrix chassisWorld,
        in Matrix wheelWorld,
        WheelSettings wheelSettings,
        Vector3 fallbackAxis)
    {
        var suspensionAxisWorld = TransformDirection(chassisWorld, wheelSettings.SuspensionLocalAxis, fallbackAxis);
        var suspensionAnchorA = Vector3.TransformCoordinate(wheelSettings.SuspensionLocalOffsetA, chassisWorld);
        var suspensionAnchorB = Vector3.TransformCoordinate(wheelSettings.SuspensionLocalOffsetB, wheelWorld);
        var axisOffset = Vector3.Dot(suspensionAnchorB - suspensionAnchorA, suspensionAxisWorld);
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
