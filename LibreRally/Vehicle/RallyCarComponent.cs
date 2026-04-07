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

    /// <summary>Maximum forward speed in m/s (~180 km/h).</summary>
    public float MaxSpeedMs { get; set; } = 50f;

    /// <summary>Wheel radius used to convert speed to spin rate (m).</summary>
    public float WheelRadius { get; set; } = 0.305f;

    /// <summary>Maximum drive motor torque (N·m). Higher = more torque / acceleration.</summary>
    public float DriveMotorForce { get; set; } = 8000f;

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

    public float SpeedKmh { get; private set; }

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
            ? $"Pad | T:{throttle:F2} B:{brake:F2} S:{steer:F2}"
            : $"No pad ({Input.GamePads.Count}) — use arrows";
        DebugText.Print($"{padInfo} | {SpeedKmh:F0} km/h", new Int2(10, 30));

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

        // ── Drive wheels (motor-based spin → friction → propulsion) ──────────
        // Nose is at +Z. Positive rotation around chassis +X moves wheel bottom in -Z,
        // which pushes the car in +Z (forward toward nose). BUT BEPU convention flips this:
        // positive TargetVelocity actually drives the car in -Z (backward). Negate to go forward.
        float targetOmega = -(throttle * MaxSpeedMs / WheelRadius);
        bool isBraking = brake > 0.05f;
        bool isHandbrake = handbrake;

        foreach (var wheel in DriveWheels)
        {
            wheel.Get<BodyComponent>()!.Awake = true;
            var ws = wheel.Get<WheelSettings>();
            if (ws?.DriveMotor == null) continue;

            if (isBraking || isHandbrake)
            {
                ws.DriveMotor.TargetVelocity = 0f;
                ws.DriveMotor.MotorMaximumForce = isHandbrake ? BrakeMotorForce * 2f : BrakeMotorForce;
            }
            else if (throttle > 0.05f)
            {
                ws.DriveMotor.TargetVelocity   = targetOmega;
                ws.DriveMotor.MotorMaximumForce = DriveMotorForce;
            }
            else
            {
                // Coasting: let wheels free-spin, no drive torque
                ws.DriveMotor.TargetVelocity    = 0f;
                ws.DriveMotor.MotorMaximumForce = 0f;
            }
        }

        // ── Steering motors (front wheels, relative to chassis) ───────────────
        // TargetVelocity drives the wheel to yaw around chassis-Y.
        // We track the software steering angle and stop driving when at the lock limit.
        foreach (var wheel in SteerWheels)
        {
            var ws = wheel.Get<WheelSettings>();
            if (ws?.SteerMotor == null) continue;

            // Integrate tracked angle (clamped to ±MaxSteerAngle)
            float prevAngle = ws.CurrentSteerAngle;
            ws.CurrentSteerAngle += steer * SteerRate * dt;
            ws.CurrentSteerAngle  = Math.Clamp(ws.CurrentSteerAngle, -MaxSteerAngle, MaxSteerAngle);

            // Drive at a rate proportional to the remaining distance to target angle
            float targetAngle = steer * MaxSteerAngle;
            float error       = targetAngle - ws.CurrentSteerAngle;
            ws.SteerMotor.TargetVelocity = Math.Clamp(error * 8f, -SteerRate, SteerRate);
        }

        // ── Chassis yaw assist (blended with motor steering) ─────────────────
        // Positive steer input = turn right. Positive Y angular velocity (right-hand rule) 
        // rotates nose from +Z toward +X = rightward. Matches: steer>0 → turn right.
        float speedMs = MathF.Abs(forwardSpeed);
        float steerAuth  = 0.4f + 0.6f * MathF.Min(1f, speedMs / 6f);
        float targetYaw  = steer * ChassisYawAssist * steerAuth;

        var av = chassisBody.AngularVelocity;
        av.Y = av.Y + (targetYaw - av.Y) * MathF.Min(1f, 6f * dt);
        chassisBody.AngularVelocity = av;

        // ── Lateral grip: bleed sideways velocity to simulate tyre grip ───────
        {
            var right = chassisTransform.WorldMatrix.Right;
            float lateralSpeed = Vector3.Dot(vel, right);
            chassisBody.LinearVelocity -= right * (lateralSpeed * MathF.Min(1f, LateralGrip * dt));
        }
    }
}
