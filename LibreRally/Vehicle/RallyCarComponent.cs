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

    /// <summary>
    /// Scale factor for self-aligning torque feedback to steering.
    /// SAT values are in N·m; this converts to a subtle angular velocity effect
    /// suitable for chassis-level force feedback without destabilising physics.
    /// </summary>
    private const float SelfAligningTorqueScale = 0.001f;

    /// <summary>
    /// Default suspension compression estimate (m) when full constraint data is unavailable.
    /// Represents typical static sag under quarter-car weight.
    /// </summary>
    private const float DefaultSuspensionCompression = 0.02f;

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

    /// <summary>Direct chassis yaw assist — keeps steering snappy even without full tire physics.</summary>
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

    /// <summary>RPM to upshift at (auto-transmission).</summary>
    public float ShiftUpRpm { get; set; } = 6500f;

    /// <summary>RPM to downshift at (auto-transmission).</summary>
    public float ShiftDownRpm { get; set; } = 2200f;

    /// <summary>Vehicle speed below which holding brake can select reverse gear.</summary>
    public float ReverseEngageSpeedKmh { get; set; } = 1.5f;

    // ── Read-only telemetry ──────────────────────────────────────────────────
    public float SpeedKmh { get; private set; }
    public float EngineRpm { get; private set; }
    public float ThrottleInput { get; private set; }
    public float BrakeInput { get; private set; }
    public bool HandbrakeEngaged { get; private set; }
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
        float dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;
        if (dt <= 0f || dt > 0.1f) dt = 0.016f;

        // ── Input ────────────────────────────────────────────────────────────
        var pad = Input.GamePads.FirstOrDefault();

        float throttle = 0f, brake = 0f, steer = 0f;
        bool handbrakeRequested = false;

        if (pad != null)
        {
            throttle  = pad.State.RightTrigger;
            brake     = pad.State.LeftTrigger;
            steer     = pad.State.LeftThumb.X;
            handbrakeRequested = pad.IsButtonDown(GamePadButton.A);
        }

        if (Input.IsKeyDown(Keys.Up)    || Input.IsKeyDown(Keys.W)) throttle  = MathF.Max(throttle, 1f);
        if (Input.IsKeyDown(Keys.Down)  || Input.IsKeyDown(Keys.S)) brake     = MathF.Max(brake,    1f);
        if (Input.IsKeyDown(Keys.Left)  || Input.IsKeyDown(Keys.A)) steer     = MathF.Min(steer,   -1f);
        if (Input.IsKeyDown(Keys.Right) || Input.IsKeyDown(Keys.D)) steer     = MathF.Max(steer,    1f);
        if (Input.IsKeyDown(Keys.Space))                             handbrakeRequested = true;

        // ── Chassis physics ──────────────────────────────────────────────────
        var chassisBody = CarBody.Get<BodyComponent>();
        if (chassisBody == null) return;

        chassisBody.Awake = true;

        var chassisTransform = CarBody.Transform;
        chassisTransform.UpdateWorldMatrix();

        var noseDir = chassisTransform.WorldMatrix.Backward;  // local +Z = nose
        var vel = chassisBody.LinearVelocity;
        float forwardSpeed = Vector3.Dot(vel, noseDir);

        SpeedKmh     = MathF.Abs(forwardSpeed) * 3.6f;
        ThrottleInput = throttle;
        BrakeInput    = brake;

        // ── Auto transmission ────────────────────────────────────────────────
        // Use driven-wheel RPM for shift decisions — NOT the cosmetic display RPM.
        // This keeps the gearbox tied to the actual driveline (including wheelspin)
        // without reintroducing the old free-rev-to-6th-at-standstill bug.
        float effectiveRatio = EffectiveRatio;
        _shiftCooldown = MathF.Max(0f, _shiftCooldown - dt);

        int standingGear = ResolveStandingGearSelection(
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
            bool changedDirection = standingGear == 0 || CurrentGear == 0;
            CurrentGear    = standingGear;
            effectiveRatio = EffectiveRatio;
            _shiftCooldown = changedDirection ? 0.25f : 0.4f;
        }

        bool isReverseGear = CurrentGear == 0;
        float driveDirection = isReverseGear ? -1f : 1f;
        float driveInput = isReverseGear ? brake : throttle;
        float serviceBrakeInput = isReverseGear ? throttle : brake;
        bool isBraking   = serviceBrakeInput > 0.05f;
        bool isHandbrake = handbrakeRequested;
        HandbrakeEngaged = isHandbrake;

        float drivenWheelOmega = MeasureDrivenWheelOmegaForShift(forwardSpeed);
        float drivelineRpm = drivenWheelOmega * effectiveRatio * (60f / (2f * MathF.PI));
        drivelineRpm = Math.Clamp(drivelineRpm, IdleRpm, MaxRpm);

        int numForwardGears = GearRatios.Length - 1;
        if (CurrentGear > 0)
        {
            if (_shiftCooldown <= 0f && !isBraking && !isHandbrake && driveInput > 0.05f)
            {
                if (drivelineRpm >= ShiftUpRpm && CurrentGear < numForwardGears)
                {
                    CurrentGear++;
                    _shiftCooldown = 0.4f;
                    effectiveRatio = EffectiveRatio;
                }
                else if (drivelineRpm <= ShiftDownRpm && CurrentGear > 1)
                {
                    CurrentGear--;
                    _shiftCooldown = 0.4f;
                    effectiveRatio = EffectiveRatio;
                }
            }
            if (_shiftCooldown <= 0f && isBraking && CurrentGear > 1)
            {
                if (drivelineRpm < ShiftDownRpm)
                {
                    CurrentGear--;
                    _shiftCooldown = 0.4f;
                    effectiveRatio = EffectiveRatio;
                }
            }
        }

        drivelineRpm = Math.Clamp(drivenWheelOmega * effectiveRatio * (60f / (2f * MathF.PI)), IdleRpm, MaxRpm);

        string gearLabel = isReverseGear ? "R" : CurrentGear.ToString();
        string padInfo = pad != null
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
        float launchRpm   = IdleRpm + (AutoClutchLaunchRpm - IdleRpm) * driveInput;
        float torqueRpm   = driveInput > 0.05f ? MathF.Max(drivelineRpm, launchRpm) : drivelineRpm;
        float crankTorque = InterpolateTorqueCurve(torqueRpm) * driveInput;
        bool applyEngineBraking = driveInput < 0.05f && MathF.Abs(forwardSpeed) > 0.25f;
        float engBrake    = applyEngineBraking ? EngineBrakeTorque : 0f;
        float demandRpm   = drivelineRpm;

        // ─ RPM display (state variable for the gauge) ───────────────────────
        float engineOmega  = _engineRpm * (2f * MathF.PI / 60f);
        float frictionLoss = EngineFriction + EngineDynamicFriction * engineOmega;

        // Rev up with throttle; coupling pulls toward actual drivetrain demand
        float displayCrank = InterpolateTorqueCurve(_engineRpm) * driveInput;
        if (_engineRpm >= MaxRpm) displayCrank = 0f;
        float netDisplay   = displayCrank - frictionLoss - engBrake;
        _engineRpm += (netDisplay / EngineInertia) * dt * (60f / (2f * MathF.PI));

        // Coupling: pull display RPM toward the actual driveline demand
        float displayTargetRpm = driveInput > 0.05f ? torqueRpm : demandRpm;
        float clutchCoupling = driveInput > 0.05f ? 7f : 5f;
        _engineRpm += (displayTargetRpm - _engineRpm) * clutchCoupling * dt;

        // Allow clutch flare on launch, but stop the display RPM from free-revving
        // thousands above the actual driveline once the car is rolling.
        float slipBlend = Math.Clamp(SpeedKmh / 35f, 0f, 1f);
        float clutchSlipAllowance = driveInput > 0.05f
            ? MathUtil.Lerp(AutoClutchLaunchRpm - IdleRpm, 250f, slipBlend)
            : 150f;
        float maxCoupledRpm = demandRpm + clutchSlipAllowance;
        _engineRpm = MathF.Min(_engineRpm, MathF.Max(IdleRpm, maxCoupledRpm));

        _engineRpm = Math.Clamp(_engineRpm, IdleRpm, MaxRpm + 300f);
        EngineRpm  = _engineRpm;

        // ── Steering motors (front wheels) ────────────────────────────────────
        const float SteerServoGain = 12f;
        const float SteerVelocityLimitMultiplier = 2f;
        float steerTargetAngle = steer * MaxSteerAngle;
        float totalActualSteerAngle = 0f;
        int steerWheelCount = 0;

        foreach (var wheel in SteerWheels)
        {
            var ws = wheel.Get<WheelSettings>();
            if (ws?.SteerMotor == null) continue;

            ws.CurrentSteerAngle += Math.Clamp(steerTargetAngle - ws.CurrentSteerAngle, -SteerRate * dt, SteerRate * dt);

            wheel.Transform.UpdateWorldMatrix();
            float actualSteerAngle = MeasureSteerAngle(chassisTransform.WorldMatrix, wheel.Transform.WorldMatrix);
            float error            = ws.CurrentSteerAngle - actualSteerAngle;
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
        int   numDrive           = Math.Max(1, DriveWheels.Count);
        float wheelTargetOmega   = -(MaxRpm * (2f * MathF.PI / 60f) / effectiveRatio) * driveDirection;
        float availableWheelForce = MathF.Max(0f, crankTorque) * effectiveRatio / WheelRadius / numDrive;

        foreach (var wheel in DriveWheels)
        {
            var wb = wheel.Get<BodyComponent>();
            if (wb != null) wb.Awake = true;
            var ws = wheel.Get<WheelSettings>();
            if (ws?.DriveMotor == null) continue;

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
            Vector3 driveAxisLocalA = MeasureWheelAxleAxisLocalA(chassisTransform.WorldMatrix, wheel.Transform.WorldMatrix);
            bool rearWheel = IsRearWheel(wheel, ws);

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

        // ── Chassis yaw assist ────────────────────────────────────────────────
        float speedMs   = MathF.Abs(forwardSpeed);
        float steerAuth = 0.4f + 0.6f * MathF.Min(1f, speedMs / 6f);
        float steerRack = steerWheelCount > 0 && MaxSteerAngle > 0.0001f
            ? Math.Clamp(totalActualSteerAngle / (steerWheelCount * MaxSteerAngle), -1f, 1f)
            : steer;
        float targetYaw = steerRack * ChassisYawAssist * steerAuth;

        var av = chassisBody.AngularVelocity;
        av.Y = av.Y + (targetYaw - av.Y) * MathF.Min(1f, 6f * dt);

        // Anti-roll bar / chassis stiffness: damp pitch (X) and roll (Z) angular velocity
        av.X *= (1f - MathF.Min(1f, 10f * dt));  // pitch (wheelie / endo)
        av.Z *= (1f - MathF.Min(1f, 12f * dt));  // roll  (tipping)

        chassisBody.AngularVelocity = av;

        // ── Vehicle dynamics system ──────────────────────────────────────────
        // Computes tyre forces (slip-based), load transfer, anti-roll bars,
        // differential torque split, and applies impulses to BEPU bodies.
        if (Dynamics != null)
        {
            // Gather per-wheel data into cached arrays (zero allocation)
            float wheelTorqueAtCrank = MathF.Max(0f, crankTorque) * effectiveRatio;
            GatherWheelData(chassisBody, chassisTransform.WorldMatrix);

            // Compute brake torque for dynamics system
            float serviceBrakeTorque = BrakeMotorForce * WheelRadius * serviceBrakeInput;
            float handbrakeTorque = BrakeMotorForce * WheelRadius * HandbrakeForceMultiplier;
            for (int i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
            {
                _brakeTorques[i] = isBraking ? serviceBrakeTorque : 0f;
                if (isHandbrake && i >= VehicleDynamicsSystem.RL)
                    _brakeTorques[i] += handbrakeTorque;
            }

            Matrix chassisWorld = chassisTransform.WorldMatrix;
            float drivetrainTorque = driveInput > 0.05f
                ? wheelTorqueAtCrank * driveDirection
                : applyEngineBraking
                    ? -MathF.Sign(forwardSpeed) * engBrake * effectiveRatio
                    : 0f;

            Dynamics.Update(
                chassisBody,
                in chassisWorld,
                _wheelPositions,
                _wheelVelocities,
                _wheelOrientations,
                _wheelGrounded,
                _suspensionCompressions,
                drivetrainTorque,
                _brakeTorques,
                _camberAngles,
                dt);

            // Feed self-aligning torque back to steering (opposes steering input)
            float sat = Dynamics.GetTotalSelfAligningTorque();
            if (MathF.Abs(sat) > 0.1f)
            {
                var satAv = chassisBody.AngularVelocity;
                satAv.Y += sat * dt * SelfAligningTorqueScale;
                chassisBody.AngularVelocity = satAv;
            }
        }
    }

    /// <summary>
    /// Gathers per-wheel position, velocity, orientation, and ground contact data
    /// into cached arrays for the <see cref="VehicleDynamicsSystem"/>.
    /// </summary>
    private void GatherWheelData(BodyComponent chassisBody, Matrix chassisWorld)
    {
        Vector3 fallbackRight = SafeNormalize(chassisWorld.Right, Vector3.UnitX);
        Vector3 fallbackUp = SafeNormalize(chassisWorld.Up, Vector3.UnitY);
        Vector3 fallbackLongitudinal = SafeNormalize(chassisWorld.Backward, Vector3.UnitZ);

        for (int i = 0; i < Wheels.Count && i < VehicleDynamicsSystem.WheelCount; i++)
        {
            var wheel = Wheels[i];
            var wheelBody = wheel.Get<BodyComponent>();
            var wheelSettings = wheel.Get<WheelSettings>();

            if (wheelBody == null || wheelSettings == null)
            {
                _wheelGrounded[i] = false;
                continue;
            }

            wheel.Transform.UpdateWorldMatrix();
            Matrix wheelWorld = wheel.Transform.WorldMatrix;

            // Wheel coordinate frame (excluding spin rotation)
            Vector3 wheelRight = SafeNormalize(wheelWorld.Right, fallbackRight);
            Vector3 nonSpinUp = ProjectOnPlane(fallbackUp, wheelRight);
            Vector3 wheelUp = SafeNormalize(nonSpinUp, fallbackUp);
            Vector3 wheelFwd = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), fallbackLongitudinal);

            _wheelPositions[i] = wheelWorld.TranslationVector;
            _wheelVelocities[i] = wheelBody.LinearVelocity;

            // Build orientation matrix from non-spinning axes
            _wheelOrientations[i] = new Matrix
            {
                Row1 = new Vector4(wheelRight, 0),
                Row2 = new Vector4(wheelUp, 0),
                Row3 = new Vector4(wheelFwd, 0),
                Row4 = new Vector4(0, 0, 0, 1),
            };

            // Ground probe for contact detection
            float normalLoad = ProbeWheelNormalLoad(chassisBody, wheelBody, wheelSettings, _wheelPositions[i], wheelUp);
            _wheelGrounded[i] = normalLoad > 0f;

            // Suspension compression: estimated from distance to ground vs rest length
            // This is a simplified approximation — full suspension travel would require
            // reading the LinearAxisServo constraint position.
            _suspensionCompressions[i] = _wheelGrounded[i] ? DefaultSuspensionCompression : 0f;

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
            return 222f;  // sunburst peak as fallback

        rpm = Math.Clamp(rpm, TorqueCurveRpm[0], TorqueCurveRpm[^1]);
        for (int i = 1; i < TorqueCurveRpm.Length; i++)
        {
            if (rpm <= TorqueCurveRpm[i])
            {
                float t = (rpm - TorqueCurveRpm[i - 1]) / (TorqueCurveRpm[i] - TorqueCurveRpm[i - 1]);
                return TorqueCurveNm[i - 1] + t * (TorqueCurveNm[i] - TorqueCurveNm[i - 1]);
            }
        }
        return TorqueCurveNm[^1];
    }

    private static bool IsRearWheel(Entity wheel, WheelSettings? wheelSettings)
    {
        int dynamicsIndex = wheelSettings?.DynamicsIndex ?? -1;
        if ((uint)dynamicsIndex < VehicleDynamicsSystem.WheelCount)
            return dynamicsIndex >= VehicleDynamicsSystem.RL;

        return wheel.Name.EndsWith("_RL", StringComparison.OrdinalIgnoreCase)
            || wheel.Name.EndsWith("_RR", StringComparison.OrdinalIgnoreCase);
    }

    private static int ResolveStandingGearSelection(
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
            return currentGear;

        float reverseEngageSpeedMs = MathF.Max(reverseEngageSpeedKmh / 3.6f, 0.1f);
        bool nearStandstill = speedKmh < reverseEngageSpeedKmh && MathF.Abs(forwardSpeed) < reverseEngageSpeedMs;
        bool wantsForward = throttleInput > 0.05f;
        bool wantsReverse = brakeInput > 0.05f && throttleInput <= 0.05f;

        if (currentGear == 0)
            return wantsForward && nearStandstill ? 1 : 0;

        if (wantsReverse && nearStandstill)
            return 0;

        if (speedKmh < 2f && currentGear > 1)
            return 1;

        return currentGear;
    }

    private float MeasureDrivenWheelOmegaForShift(float forwardSpeed)
    {
        float safeWheelRadius = MathF.Max(WheelRadius, 0.05f);
        float fallbackOmega = MathF.Abs(forwardSpeed) / safeWheelRadius;
        if (Dynamics == null || DriveWheels.Count == 0)
            return fallbackOmega;

        float groundedOmegaSum = 0f;
        int groundedCount = 0;
        float sampledOmegaSum = 0f;
        int sampledCount = 0;

        foreach (var wheel in DriveWheels)
        {
            var ws = wheel.Get<WheelSettings>();
            int dynamicsIndex = ws?.DynamicsIndex ?? -1;
            if ((uint)dynamicsIndex >= VehicleDynamicsSystem.WheelCount)
                continue;

            float wheelOmega = MathF.Abs(Dynamics.WheelStates[dynamicsIndex].AngularVelocity);
            sampledOmegaSum += wheelOmega;
            sampledCount++;

            if (!Dynamics.WheelGrounded[dynamicsIndex])
                continue;

            groundedOmegaSum += wheelOmega;
            groundedCount++;
        }

        if (groundedCount > 0)
            return groundedOmegaSum / groundedCount;

        if (sampledCount > 0 && MathF.Abs(forwardSpeed) > 0.5f)
            return sampledOmegaSum / sampledCount;

        return fallbackOmega;
    }

    private static float MeasureSteerAngle(Matrix chassisWorld, Matrix wheelWorld)
    {
        Vector3 up = chassisWorld.Up;
        float upLengthSq = up.LengthSquared();
        if (upLengthSq < 1e-6f)
            return 0f;

        up /= MathF.Sqrt(upLengthSq);

        Vector3 chassisRight = ProjectOnPlane(chassisWorld.Right, up);
        Vector3 wheelRight = ProjectOnPlane(wheelWorld.Right, up);
        float chassisRightLengthSq = chassisRight.LengthSquared();
        float wheelRightLengthSq = wheelRight.LengthSquared();
        if (chassisRightLengthSq < 1e-6f || wheelRightLengthSq < 1e-6f)
            return 0f;

        chassisRight /= MathF.Sqrt(chassisRightLengthSq);
        wheelRight /= MathF.Sqrt(wheelRightLengthSq);

        float sin = Vector3.Dot(Vector3.Cross(chassisRight, wheelRight), up);
        float cos = Math.Clamp(Vector3.Dot(chassisRight, wheelRight), -1f, 1f);

        // Positive steer angle corresponds to right lock, matching the steering servo target.
        return -MathF.Atan2(sin, cos);
    }

    private static Vector3 MeasureWheelAxleAxisLocalA(Matrix bodyAWorld, Matrix wheelWorld)
    {
        Vector3 wheelAxleWorld = wheelWorld.Right;
        float axleLengthSq = wheelAxleWorld.LengthSquared();
        if (axleLengthSq < 1e-6f)
            return Vector3.UnitX;

        wheelAxleWorld /= MathF.Sqrt(axleLengthSq);

        Vector3 axisLocalA = new(
            Vector3.Dot(wheelAxleWorld, bodyAWorld.Right),
            Vector3.Dot(wheelAxleWorld, bodyAWorld.Up),
            Vector3.Dot(wheelAxleWorld, bodyAWorld.Backward));

        float axisLengthSq = axisLocalA.LengthSquared();
        if (axisLengthSq < 1e-6f)
            return Vector3.UnitX;

        return axisLocalA / MathF.Sqrt(axisLengthSq);
    }

    private static float ProbeWheelNormalLoad(
        BodyComponent chassisBody,
        BodyComponent wheelBody,
        WheelSettings wheelSettings,
        Vector3 wheelPosition,
        Vector3 wheelUp)
    {
        var simulation = wheelBody.Simulation;
        var tyreModel = wheelSettings.TyreModel;
        if (simulation == null || tyreModel == null)
            return 0f;

        float staticNormalLoad = Math.Max(wheelSettings.StaticNormalLoad, 0f);
        if (staticNormalLoad <= 0f)
            return 0f;

        float wheelRadius = Math.Max(tyreModel.Radius, 0.1f);
        Vector3 rayOrigin = wheelPosition + wheelUp * (wheelRadius + GroundProbeMargin);
        Vector3 rayDirection = -wheelUp;
        float rayLength = wheelRadius * 2f + GroundProbeMargin * 2f;

        wheelSettings.GroundProbeHits.Clear();
        simulation.RayCastPenetrating(ref rayOrigin, ref rayDirection, rayLength, wheelSettings.GroundProbeHits, CollisionMask.Everything);

        foreach (var hit in wheelSettings.GroundProbeHits)
        {
            if (hit.Collidable == null || hit.Collidable == wheelBody || hit.Collidable == chassisBody)
                continue;

            return staticNormalLoad;
        }

        return 0f;
    }

    private static Vector3 ProjectOnPlane(Vector3 vector, Vector3 planeNormal)
    {
        return vector - planeNormal * Vector3.Dot(vector, planeNormal);
    }

    private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
    {
        float lengthSquared = value.LengthSquared();
        if (lengthSquared < 1e-6f)
            return fallback;

        return value / MathF.Sqrt(lengthSquared);
    }
}
