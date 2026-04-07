using System;
using LibreRally.Race;
using LibreRally.Vehicle;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally.HUD;

[ComponentCategory("LibreRally")]
public class RallyHud : SyncScript
{
    public RallyCarComponent? Car { get; set; }
    public RaceTimer? Timer { get; set; }
    public RaceController? RaceController { get; set; }

    // Gauge layout constants — tuned for 1280×720
    private const float GaugeRx   = 95f;   // horizontal radius (px)
    private const float GaugeRy   = 58f;   // vertical radius (compensates for char aspect ~8×16px)
    private const float StartDeg  = 225f;  // 7:30 clock position = 0 km/h
    private const float SweepDeg  = -270f; // 270° clockwise sweep to 4:30 = max km/h
    private const float MaxSpeed  = 200f;
    private const int   NumTicks  = 10;    // ticks at 0,20,40,...,200 km/h

    public override void Start() { }

    public override void Update()
    {
        if (Car == null) return;

        var gd = ((Stride.Engine.Game)Game).GraphicsDevice;
        int sw = gd.Presenter?.BackBuffer?.Width  ?? 1280;
        int sh = gd.Presenter?.BackBuffer?.Height ?? 720;

        // ── Speedometer arc (bottom-right) ───────────────────────────────────
        int cx = sw - 150;
        int cy = sh - 140;

        DrawSpeedGauge(cx, cy, Car.SpeedKmh);

        // ── RPM bar ──────────────────────────────────────────────────────────
        int barY = cy + 72;
        int barX = cx - 70;
        DrawRpmBar(barX, barY, Car.EngineRpm, Car.MaxRpm);

        // ── Input indicators (small, bottom of gauge) ─────────────────────
        int infoY = cy + 88;
        DrawInputLine(barX, infoY, Car.ThrottleInput, Car.BrakeInput);

        // ── Timer / race state ────────────────────────────────────────────
        if (Timer != null)
            DebugText.Print($"Time: {Timer.GetTimeString()}", new Int2(16, 16));

        if (RaceController != null && !RaceController.IsRacing && RaceController.CountdownRemaining > 0)
            DebugText.Print($"GO IN: {RaceController.CountdownBeats}", new Int2(16, 36));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Gauge drawing helpers
    // ─────────────────────────────────────────────────────────────────────────

    private void DrawSpeedGauge(int cx, int cy, float speedKmh)
    {
        // Draw arc ticks — filled (*) for the swept portion, empty (.) for the rest
        for (int i = 0; i <= NumTicks; i++)
        {
            float fraction  = (float)i / NumTicks;
            float tickSpeed = fraction * MaxSpeed;
            float angleDeg  = StartDeg + SweepDeg * fraction;
            float angleRad  = angleDeg * MathF.PI / 180f;

            int tx = (int)(cx + MathF.Cos(angleRad) * GaugeRx);
            int ty = (int)(cy - MathF.Sin(angleRad) * GaugeRy);

            // Major ticks every 40 km/h get a number label offset outward
            bool major = (i % 4 == 0);
            bool swept = speedKmh >= tickSpeed - 0.5f;

            DebugText.Print(swept ? (major ? "O" : "*") : (major ? "o" : "."), new Int2(tx, ty));

            if (major)
            {
                // Tick speed label — nudge radially outward 18px
                int lx = (int)(cx + MathF.Cos(angleRad) * (GaugeRx + 18));
                int ly = (int)(cy - MathF.Sin(angleRad) * (GaugeRy + 12));
                DebugText.Print($"{(int)tickSpeed}", new Int2(lx - 8, ly - 6));
            }
        }

        // Compute needle tip position (slightly inside arc)
        {
            float fraction = Math.Clamp(speedKmh / MaxSpeed, 0f, 1f);
            float angleDeg = StartDeg + SweepDeg * fraction;
            float angleRad = angleDeg * MathF.PI / 180f;
            int nx = (int)(cx + MathF.Cos(angleRad) * (GaugeRx * 0.65f));
            int ny = (int)(cy - MathF.Sin(angleRad) * (GaugeRy * 0.65f));
            DebugText.Print("+", new Int2(nx, ny));  // needle tip
        }

        // Digital speed readout at center
        string speedStr = $"{speedKmh:F0}";
        DebugText.Print(speedStr, new Int2(cx - speedStr.Length * 4, cy - 8));
        DebugText.Print("km/h", new Int2(cx - 14, cy + 8));
    }

    private void DrawRpmBar(int x, int y, float rpm, float maxRpm)
    {
        const int Blocks = 16;
        int filled = (int)Math.Clamp(rpm / maxRpm * Blocks, 0, Blocks);

        // Last 2 blocks = redline zone
        string bar = "RPM [";
        for (int i = 0; i < Blocks; i++)
            bar += (i < filled) ? (i >= Blocks - 2 ? "!" : "#") : " ";
        bar += $"] {rpm:F0}";
        DebugText.Print(bar, new Int2(x, y));
    }

    private void DrawInputLine(int x, int y, float throttle, float brake)
    {
        int t = (int)(throttle * 8);
        int b = (int)(brake    * 8);
        DebugText.Print($"T[{new string('>', t),8}] B[{new string('<', b),8}]", new Int2(x, y));
    }
}
