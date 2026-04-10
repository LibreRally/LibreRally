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
        if (Timer != null)
        {
	        DebugText.Print($"Time: {Timer.GetTimeString()}", new Int2(16, 16));
        }

        if (RaceController != null && !RaceController.IsRacing && RaceController.CountdownRemaining > 0)
        {
	        DebugText.Print($"GO IN: {RaceController.CountdownBeats}", new Int2(16, 36));
        }

        if (Car == null)
        {
	        return;
        }

        // Digital overlay aligned to the dual-dial gauge drawn by SpeedoGauge.
        var gd = ((Stride.Engine.Game)Game).GraphicsDevice;
        var sw = gd.Presenter?.BackBuffer?.Width  ?? 1280;
        var sh = gd.Presenter?.BackBuffer?.Height ?? 720;
        var speedCx = sw - 255;
        var rpmCx   = sw - 110;
        var cy      = sh - 132;
        var midCx   = (speedCx + rpmCx) / 2;

        var speedStr = $"{Car.SpeedKmh:F0}";
        DebugText.Print(speedStr, new Int2(speedCx - speedStr.Length * 4, cy - 8));
        DebugText.Print("km/h", new Int2(speedCx - 14, cy + 10));

        var rpmStr = $"{Car.EngineRpm / 1000f:F1}";
        DebugText.Print(rpmStr, new Int2(rpmCx - rpmStr.Length * 4, cy - 8));
        DebugText.Print("x1000", new Int2(rpmCx - 18, cy + 10));

        // Redline flash warning
        if (Car.EngineRpm > Car.MaxRpm * 0.92f)
        {
	        DebugText.Print("SHIFT!", new Int2(rpmCx - 18, cy - 30));
        }
    }
}
