using System;
using System.Linq;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Constraints;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;

namespace LibreRally.Vehicle;

/// <summary>
/// Driving script for a vehicle built by <see cref="VehicleLoader"/>.
/// Attach to the vehicle root entity.
///
/// Controls:
///   W / S       — throttle / brake
///   A / D       — steer left / steer right
///   Space       — handbrake
/// </summary>
public class VehicleScript : SyncScript
{
    // ──────────────────────────────────────────────────────────────────────────
    // Tuning parameters (editable in Stride Editor)
    // ──────────────────────────────────────────────────────────────────────────

    public float EngineTorque { get; set; } = 2500f;
    public float BrakeTorque { get; set; } = 4000f;
    public float MaxSteerAngle { get; set; } = 0.45f;   // radians (~26°)
    public float SteerSpeed { get; set; } = 2.5f;
    public float SteerReturnSpeed { get; set; } = 4f;
    public float MaxSpeedMs { get; set; } = 70f;         // ~250 km/h

    // ──────────────────────────────────────────────────────────────────────────
    // Internal state
    // ──────────────────────────────────────────────────────────────────────────

    /// <summary>Set by <see cref="VehicleLoader"/> after assembly.</summary>
    public VehicleDefinition? Definition { get; set; }

    private BodyComponent? _chassis;
    private BodyComponent? _wheelFL, _wheelFR, _wheelRL, _wheelRR;

    private float _steerAngle;

    public override void Start()
    {
        // Find child entities by name
        foreach (var child in Entity.GetChildren())
        {
            switch (child.Name)
            {
                case "chassis":  _chassis  = child.Get<BodyComponent>(); break;
                case "wheel_FL": _wheelFL  = child.Get<BodyComponent>(); break;
                case "wheel_FR": _wheelFR  = child.Get<BodyComponent>(); break;
                case "wheel_RL": _wheelRL  = child.Get<BodyComponent>(); break;
                case "wheel_RR": _wheelRR  = child.Get<BodyComponent>(); break;
            }
        }
    }

    public override void Update()
    {
        if (_chassis == null) return;

        float dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;
        if (dt <= 0f) return;

        ProcessDriving(dt);
    }

    private void ProcessDriving(float dt)
    {
        // ── Input ──────────────────────────────────────────────────────────
        float throttle = 0f, brake = 0f, steerInput = 0f;
        bool handbrake = false;

        if (Input.IsKeyDown(Keys.W)) throttle = 1f;
        if (Input.IsKeyDown(Keys.S)) brake = 1f;
        if (Input.IsKeyDown(Keys.A)) steerInput = -1f;
        if (Input.IsKeyDown(Keys.D)) steerInput = 1f;
        if (Input.IsKeyDown(Keys.Space)) handbrake = true;

        // ── Steering ──────────────────────────────────────────────────────
        if (Math.Abs(steerInput) > 0.01f)
            _steerAngle = MathUtil.Clamp(
                _steerAngle + steerInput * SteerSpeed * dt,
                -MaxSteerAngle, MaxSteerAngle);
        else
            _steerAngle = MathUtil.Lerp(_steerAngle, 0f, SteerReturnSpeed * dt);

        // ── Chassis velocity ───────────────────────────────────────────────
        var vel = _chassis!.LinearVelocity;
        float speed = vel.Length();

        // Forward in Stride chassis space is -Z
        var chassisForward = Vector3.Transform(-Vector3.UnitZ, _chassis.Entity.Transform.Rotation);

        // ── Throttle ──────────────────────────────────────────────────────
        if (throttle > 0.01f && speed < MaxSpeedMs)
        {
            var impulse = chassisForward * (EngineTorque * throttle * dt);
            _chassis.ApplyImpulse(impulse, Vector3.Zero);
        }

        // ── Braking ───────────────────────────────────────────────────────
        if ((brake > 0.01f || handbrake) && speed > 0.01f)
        {
            float multiplier = handbrake ? 1.5f : brake;
            var brakeImpulse = -vel * (BrakeTorque * multiplier * dt);
            _chassis.ApplyImpulse(brakeImpulse, Vector3.Zero);
        }

        // ── Anti-roll (keep upright) ───────────────────────────────────────
        ApplyAntiRoll(dt);
    }

    private void ApplyAntiRoll(float dt)
    {
        if (_chassis == null) return;

        var up = Vector3.Transform(Vector3.UnitY, _chassis.Entity.Transform.Rotation);
        var worldUp = Vector3.UnitY;
        var rollCorrection = Vector3.Cross(up, worldUp);
        _chassis.AngularVelocity += rollCorrection * 8f * dt;
    }
}
