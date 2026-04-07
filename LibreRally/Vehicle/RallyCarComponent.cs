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

    /// <summary>Forward gear ratios indexed 1–N (index 0 = reverse).</summary>
    public float[] GearRatios { get; set; } = { 3.25f, 3.64f, 2.38f, 1.76f, 1.35f, 1.06f, 0.84f };

    /// <summary>Final drive (differential) ratio.</summary>
    public float FinalDrive { get; set; } = 4.55f;

    /// <summary>Peak engine torque at the crank (N·m).</summary>
    public float PeakTorqueNm { get; set; } = 222f;

    /// <summary>Max engine RPM (redline).</summary>
    public float MaxRpm { get; set; } = 7500f;

    /// <summary>Idle RPM (minimum engine speed).</summary>
    public float IdleRpm { get; set; } = 900f;

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

    // Only kept for backward compatibility (HUD reads it); computed from current gear
    public float DriveRatio => EffectiveRatio;

    private float EffectiveRatio => GearRatios[Math.Clamp(CurrentGear, 0, GearRatios.Length - 1)] * FinalDrive;
    private float _shiftCooldown;

    public override void Start() { }

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

        // Car nose is at Stride +Z (BeamNG -Y → Stride +Z = WorldMatrix.Backward)
        // Forward = +Z = Backward in Stride's convention
        var noseDir = chassisTransform.WorldMatrix.Backward;
        var vel = chassisBody.LinearVelocity;
        float forwardSpeed = Vector3.Dot(vel, noseDir);

        SpeedKmh = MathF.Abs(forwardSpeed) * 3.6f;
        ThrottleInput = throttle;
        BrakeInput    = brake;

        // ── Auto transmission ────────────────────────────────────────────────
        // Compute engine RPM from current wheel speed and active gear ratio.
        float wheelOmega   = MathF.Abs(forwardSpeed) / WheelRadius;  // rad/s
        float effectiveRatio = EffectiveRatio;
        EngineRpm = Math.Clamp(wheelOmega * effectiveRatio * 60f / (2f * MathF.PI), IdleRpm, MaxRpm);

        _shiftCooldown = MathF.Max(0f, _shiftCooldown - dt);

        int numForwardGears = GearRatios.Length - 1; // index 0 = reverse
        if (_shiftCooldown <= 0f && !isBraking && !isHandbrake && throttle > 0.05f)
        {
            if (EngineRpm >= ShiftUpRpm && CurrentGear < numForwardGears)
            {
                CurrentGear++;
                _shiftCooldown = 0.4f;
                effectiveRatio = EffectiveRatio;
            }
            else if (EngineRpm <= ShiftDownRpm && CurrentGear > 1)
            {
                CurrentGear--;
                _shiftCooldown = 0.4f;
                effectiveRatio = EffectiveRatio;
            }
        }
        // Down-shift when braking hard
        if (_shiftCooldown <= 0f && isBraking && CurrentGear > 1)
        {
            float brakingRpm = wheelOmega * EffectiveRatio * 60f / (2f * MathF.PI);
            if (brakingRpm < ShiftDownRpm)
            {
                CurrentGear--;
                _shiftCooldown = 0.4f;
                effectiveRatio = EffectiveRatio;
            }
        }

        // ── Per-gear motor parameters ────────────────────────────────────────
        float maxWheelOmega  = MaxRpm / effectiveRatio * (2f * MathF.PI / 60f);
        float targetOmega    = -(throttle * maxWheelOmega);
        // Divide engine torque evenly across all driven wheels so total wheel force
        // = PeakTorqueNm × effectiveRatio regardless of how many motors are used.
        int numDrive = Math.Max(1, DriveWheels.Count);
        float gearMotorForce = PeakTorqueNm * effectiveRatio / WheelRadius / numDrive;

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
                ws.DriveMotor.TargetVelocity    = targetOmega;
                ws.DriveMotor.MotorMaximumForce = gearMotorForce;
                ws.DriveMotor.MotorDamping      = 500f;
            }
            else
            {
                // Coasting: let wheels free-spin, no drive torque
                ws.DriveMotor.TargetVelocity    = 0f;
                ws.DriveMotor.MotorMaximumForce = 0f;
                ws.DriveMotor.MotorDamping      = 500f;
            }
        }

        // ── Steering motors (front wheels, relative to chassis) ───────────────
        // TargetVelocity drives the wheel to yaw around chassis-Y.
        // We track the software steering angle and stop driving when at the lock limit.
        foreach (var wheel in SteerWheels)
        {
            var ws = wheel.Get<WheelSettings>();
            if (ws?.SteerMotor == null) continue;

            // Integrate tracked angle (clamped to ±MaxSteerAngle); negate steer to match physics direction
            float prevAngle = ws.CurrentSteerAngle;
            ws.CurrentSteerAngle += -steer * SteerRate * dt;
            ws.CurrentSteerAngle  = Math.Clamp(ws.CurrentSteerAngle, -MaxSteerAngle, MaxSteerAngle);

            // Drive at a rate proportional to the remaining distance to target angle
            float targetAngle = -steer * MaxSteerAngle;
            float error       = targetAngle - ws.CurrentSteerAngle;
            ws.SteerMotor.TargetVelocity = Math.Clamp(error * 8f, -SteerRate, SteerRate);
        }

        // ── Chassis yaw assist (blended with motor steering) ─────────────────
        // Positive steer input = right thumb = should turn right.
        // However, positive AngularVelocity.Y in Stride/BEPU actually rotates the nose
        // toward -X (left), so we negate to get the correct physical direction.
        float speedMs = MathF.Abs(forwardSpeed);
        float steerAuth  = 0.4f + 0.6f * MathF.Min(1f, speedMs / 6f);
        float targetYaw  = -steer * ChassisYawAssist * steerAuth;

        var av = chassisBody.AngularVelocity;
        av.Y = av.Y + (targetYaw - av.Y) * MathF.Min(1f, 6f * dt);

        // ── Anti-roll and anti-wheelie damping ────────────────────────────────
        // Car local axes: +Z = nose, +Y = up, +X = right.
        // Pitch (nose up/down, wheelies/endos): AngularVelocity.X
        // Roll  (sideways tilt):               AngularVelocity.Z
        // Damp these strongly to simulate chassis rigidity and anti-roll bars.
        float rollDamp  = MathF.Min(1f, 12f * dt);
        float pitchDamp = MathF.Min(1f, 10f * dt);
        av.X *= (1f - pitchDamp);
        av.Z *= (1f - rollDamp);

        chassisBody.AngularVelocity = av;

        // ── Lateral grip: bleed sideways velocity to simulate tyre grip ───────
        {
            var right = chassisTransform.WorldMatrix.Right;
            float lateralSpeed = Vector3.Dot(vel, right);
            chassisBody.LinearVelocity -= right * (lateralSpeed * MathF.Min(1f, LateralGrip * dt));
        }
    }
}
