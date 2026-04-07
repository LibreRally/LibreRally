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

    public override void Start() { }

    public override void Update()
    {
        int x = 16;
        int y = 16;

        if (Car != null)
        {
            float speed = Car.SpeedKmh;
            float rpm   = Car.EngineRpm;
            float maxRpm = Car.MaxRpm;

            // ── Speed display ─────────────────────────────────────────────────
            DebugText.Print($"SPEED  {speed,6:F0} km/h", new Int2(x, y)); y += 20;

            // Speed bar (0–200 km/h = 20 blocks)
            int speedBlocks = (int)Math.Clamp(speed / 10f, 0, 20);
            string speedBar = "[" + new string('|', speedBlocks) + new string(' ', 20 - speedBlocks) + "]";
            DebugText.Print(speedBar, new Int2(x, y)); y += 22;

            // ── RPM display ───────────────────────────────────────────────────
            DebugText.Print($"RPM  {rpm,6:F0} / {maxRpm:F0}", new Int2(x, y)); y += 20;

            // RPM bar (0–maxRpm = 20 blocks; last 2 = redline)
            int rpmBlocks = (int)Math.Clamp(rpm / maxRpm * 20f, 0, 20);
            string rpmBar = "[" + new string('|', Math.Min(rpmBlocks, 18))
                              + (rpmBlocks >= 18 ? new string('!', rpmBlocks - 18) : "")
                              + new string(' ', 20 - rpmBlocks) + "]";
            DebugText.Print(rpmBar, new Int2(x, y)); y += 22;

            // ── Throttle / brake ──────────────────────────────────────────────
            int tBlocks = (int)(Car.ThrottleInput * 10);
            int bBlocks = (int)(Car.BrakeInput    * 10);
            DebugText.Print($"T [{new string('#', tBlocks),10}]  B [{new string('#', bBlocks),10}]",
                new Int2(x, y)); y += 20;

            y += 6;
        }

        if (Timer != null)
        {
            DebugText.Print($"Time: {Timer.GetTimeString()}", new Int2(x, y)); y += 20;
        }

        if (RaceController != null && !RaceController.IsRacing && RaceController.CountdownRemaining > 0)
        {
            DebugText.Print($"GO IN: {RaceController.CountdownBeats}", new Int2(x, y));
        }
    }
}
