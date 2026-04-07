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
    public Entity CarBody { get; set; } = new();
    public List<Entity> Wheels { get; set; } = new();
    public List<Entity> SteerWheels { get; set; } = new();
    public List<Entity> DriveWheels { get; set; } = new();
    public List<Entity> BreakWheels { get; set; } = new();

    /// <summary>Wheel radius used to convert speed to spin rate (m).</summary>
    public float WheelRadius { get; set; } = 0.305f;

    /// <summary>Brake motor force — how hard wheels resist rotation when braking.</summary>
    public float BrakeMotorForce { get; set; } = 10000f;

    /// <summary>Maximum steering wheel lock angle (radians). ~0.52 rad ≈ 30°.</summary>
    public float MaxSteerAngle { get; set; } = 0.52f;

    /// <summary>Angular rate at which front wheels steer (rad/s).</summary>
    public float SteerRate { get; set; } = 3f;

    /// <summary>Direct chassis yaw assist — keeps steering snappy even without full tire physics.</summary>
    public float ChassisYawAssist { get; set; } = 1.0f;

    /// <summary>Lateral grip: fraction of sideways velocity removed per second.</summary>
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

    // ── Read-only telemetry ──────────────────────────────────────────────────
    public float SpeedKmh { get; private set; }
    public float EngineRpm { get; private set; }
    public float ThrottleInput { get; private set; }
    public float BrakeInput { get; private set; }
    public int CurrentGear { get; private set; } = 1;

    public float DriveRatio => EffectiveRatio;

    private float EffectiveRatio => GearRatios[Math.Clamp(CurrentGear, 0, GearRatios.Length - 1)] * FinalDrive;
    private float _shiftCooldown;

    // Simulated engine RPM — the engine drives the drivetrain, not the other way around.
    private float _engineRpm;

    public override void Start()
    {
        _engineRpm = IdleRpm;
    }

    public override void Update()
    {
        float dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;
        if (dt <= 0f || dt > 0.1f) dt = 0.016f;

        // ── Input ────────────────────────────────────────────────────────────
        var pad = Input.GamePads.FirstOrDefault();

        float throttle = 0f, brake = 0f, steer = 0f;
        bool handbrake = false;

        if (pad != null)
        {
            throttle  = pad.State.RightTrigger;
            brake     = pad.State.LeftTrigger;
            steer     = pad.State.LeftThumb.X;
            handbrake = pad.IsButtonDown(GamePadButton.A);
        }

        if (Input.IsKeyDown(Keys.Up)    || Input.IsKeyDown(Keys.W)) throttle  = MathF.Max(throttle, 1f);
        if (Input.IsKeyDown(Keys.Down)  || Input.IsKeyDown(Keys.S)) brake     = MathF.Max(brake,    1f);
        if (Input.IsKeyDown(Keys.Left)  || Input.IsKeyDown(Keys.A)) steer     = MathF.Min(steer,   -1f);
        if (Input.IsKeyDown(Keys.Right) || Input.IsKeyDown(Keys.D)) steer     = MathF.Max(steer,    1f);
        if (Input.IsKeyDown(Keys.Space))                             handbrake = true;

        string padInfo = pad != null
            ? $"Pad | T:{throttle:F2} B:{brake:F2} S:{steer:F2} G:{CurrentGear}"
            : $"No pad ({Input.GamePads.Count}) — use arrows";
        DebugText.Print($"{padInfo} | {SpeedKmh:F0} km/h", new Int2(10, 30));

        bool isBraking   = brake > 0.05f;
        bool isHandbrake = handbrake;

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
        // Use speed-derived RPM for shift decisions — NOT the cosmetic display RPM.
        // The display RPM free-revs at standstill, which would shift the car to 6th
        // gear before it even moves.
        float effectiveRatio = EffectiveRatio;
        _shiftCooldown = MathF.Max(0f, _shiftCooldown - dt);

        float wheelOmegaForShift = MathF.Abs(forwardSpeed) / WheelRadius;
        float speedRpm = wheelOmegaForShift * effectiveRatio * (60f / (2f * MathF.PI));
        speedRpm = Math.Clamp(speedRpm, IdleRpm, MaxRpm);

        // Force gear 1 when nearly stationary
        if (SpeedKmh < 2f && CurrentGear > 1 && _shiftCooldown <= 0f)
        {
            CurrentGear    = 1;
            effectiveRatio = EffectiveRatio;
            _shiftCooldown = 0.4f;
        }

        int numForwardGears = GearRatios.Length - 1;
        if (_shiftCooldown <= 0f && !isBraking && !isHandbrake && throttle > 0.05f)
        {
            if (speedRpm >= ShiftUpRpm && CurrentGear < numForwardGears)
            {
                CurrentGear++;
                _shiftCooldown = 0.4f;
                effectiveRatio = EffectiveRatio;
            }
            else if (speedRpm <= ShiftDownRpm && CurrentGear > 1)
            {
                CurrentGear--;
                _shiftCooldown = 0.4f;
                effectiveRatio = EffectiveRatio;
            }
        }
        if (_shiftCooldown <= 0f && isBraking && CurrentGear > 1)
        {
            if (speedRpm < ShiftDownRpm)
            {
                CurrentGear--;
                _shiftCooldown = 0.4f;
                effectiveRatio = EffectiveRatio;
            }
        }

        speedRpm = Math.Clamp(wheelOmegaForShift * effectiveRatio * (60f / (2f * MathF.PI)), IdleRpm, MaxRpm);

        // ── Engine simulation ─────────────────────────────────────────────────
        // Two separate concerns:
        //
        // 1. Wheel force (physics): derived from actual wheel/car speed → torque curve.
        //    This is always well-behaved — no free-revving, no stall, no redline cutoff.
        //
        // 2. RPM display / shifting: a state variable that revs up with throttle and
        //    is pulled toward the drivetrain demand as the car accelerates.
        //    This gives a realistic rev needle without affecting the driving physics.

        // ─ Torque for wheels (based on actual drivetrain state) ──────────────
        // Launch from a dead stop needs clutch slip; otherwise a locked wheel-speed
        // model is stuck at idle torque forever. Hold the engine in the launch band
        // until wheel speed catches up, then hand over to the real speed-derived RPM.
        float launchRpm   = IdleRpm + (AutoClutchLaunchRpm - IdleRpm) * throttle;
        float torqueRpm   = throttle > 0.05f ? MathF.Max(speedRpm, launchRpm) : speedRpm;
        float crankTorque = InterpolateTorqueCurve(torqueRpm) * throttle;
        float engBrake    = (throttle < 0.05f) ? EngineBrakeTorque : 0f;
        float demandRpm   = speedRpm;

        // ─ RPM display (state variable, for gauge and auto-shift) ───────────
        float engineOmega  = _engineRpm * (2f * MathF.PI / 60f);
        float frictionLoss = EngineFriction + EngineDynamicFriction * engineOmega;

        // Rev up with throttle; coupling pulls toward actual drivetrain demand
        float displayCrank = InterpolateTorqueCurve(_engineRpm) * throttle;
        if (_engineRpm >= MaxRpm) displayCrank = 0f;
        float netDisplay   = displayCrank - frictionLoss - engBrake;
        _engineRpm += (netDisplay / EngineInertia) * dt * (60f / (2f * MathF.PI));

        // Coupling: pull display RPM toward drivetrain demand when moving
        float displayTargetRpm = throttle > 0.05f ? torqueRpm : demandRpm;
        _engineRpm += (displayTargetRpm - _engineRpm) * 5f * dt;

        _engineRpm = Math.Clamp(_engineRpm, IdleRpm, MaxRpm + 300f);
        EngineRpm  = _engineRpm;

        // ── Wheel motor commands ──────────────────────────────────────────────
        // Target = redline speed so motor always pushes forward; MaxForce limits torque.
        int   numDrive            = Math.Max(1, DriveWheels.Count);
        float wheelTargetOmega     = -(MaxRpm * (2f * MathF.PI / 60f) / effectiveRatio);
        float availableWheelForce  = MathF.Max(0f, crankTorque) * effectiveRatio / WheelRadius / numDrive;

        foreach (var wheel in DriveWheels)
        {
            var wb = wheel.Get<BodyComponent>();
            if (wb != null) wb.Awake = true;
            var ws = wheel.Get<WheelSettings>();
            if (ws?.DriveMotor == null) continue;

            if (isBraking || isHandbrake)
            {
                ws.DriveMotor.TargetVelocity    = 0f;
                ws.DriveMotor.MotorMaximumForce = isHandbrake ? BrakeMotorForce * 2f : BrakeMotorForce;
                ws.DriveMotor.MotorDamping      = 500f;
            }
            else if (throttle > 0.05f)
            {
                ws.DriveMotor.TargetVelocity    = wheelTargetOmega;
                ws.DriveMotor.MotorMaximumForce = availableWheelForce;
                ws.DriveMotor.MotorDamping      = 500f;
            }
            else
            {
                // Coasting / engine braking
                ws.DriveMotor.TargetVelocity    = 0f;
                ws.DriveMotor.MotorMaximumForce = engBrake * effectiveRatio / WheelRadius / numDrive;
                ws.DriveMotor.MotorDamping      = 500f;
            }
        }

        // ── Steering motors (front wheels) ────────────────────────────────────
        foreach (var wheel in SteerWheels)
        {
            var ws = wheel.Get<WheelSettings>();
            if (ws?.SteerMotor == null) continue;

            ws.CurrentSteerAngle += -steer * SteerRate * dt;
            ws.CurrentSteerAngle  = Math.Clamp(ws.CurrentSteerAngle, -MaxSteerAngle, MaxSteerAngle);

            float targetAngle = -steer * MaxSteerAngle;
            float error       = targetAngle - ws.CurrentSteerAngle;
            ws.SteerMotor.TargetVelocity = Math.Clamp(error * 8f, -SteerRate, SteerRate);
        }

        // ── Chassis yaw assist ────────────────────────────────────────────────
        float speedMs    = MathF.Abs(forwardSpeed);
        float steerAuth  = 0.4f + 0.6f * MathF.Min(1f, speedMs / 6f);
        float targetYaw  = -steer * ChassisYawAssist * steerAuth;

        var av = chassisBody.AngularVelocity;
        av.Y = av.Y + (targetYaw - av.Y) * MathF.Min(1f, 6f * dt);

        // Anti-roll bar / chassis stiffness: damp pitch (X) and roll (Z) angular velocity
        av.X *= (1f - MathF.Min(1f, 10f * dt));  // pitch (wheelie / endo)
        av.Z *= (1f - MathF.Min(1f, 12f * dt));  // roll  (tipping)

        chassisBody.AngularVelocity = av;

        // ── Lateral grip ──────────────────────────────────────────────────────
        {
            var right = chassisTransform.WorldMatrix.Right;
            float lateralSpeed = Vector3.Dot(vel, right);
            chassisBody.LinearVelocity -= right * (lateralSpeed * MathF.Min(1f, LateralGrip * dt));
        }
    }

    /// <summary>
    /// Linearly interpolates the engine torque curve at the given RPM.
    /// Falls back to <see cref="PeakTorqueNm"/> (set to zero here, overridden by loader) when curve is absent.
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
}

