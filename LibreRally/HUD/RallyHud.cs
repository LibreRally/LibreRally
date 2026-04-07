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
            DebugText.Print($"Time: {Timer.GetTimeString()}", new Int2(16, 16));

        if (RaceController != null && !RaceController.IsRacing && RaceController.CountdownRemaining > 0)
            DebugText.Print($"GO IN: {RaceController.CountdownBeats}", new Int2(16, 36));

        if (Car == null) return;

        // Digital overlay centred inside the graphical gauge (bottom-right area)
        var gd = ((Stride.Engine.Game)Game).GraphicsDevice;
        int sw = gd.Presenter?.BackBuffer?.Width  ?? 1280;
        int sh = gd.Presenter?.BackBuffer?.Height ?? 720;
        int cx = sw - 155;
        int cy = sh - 155;

        string speedStr = $"{Car.SpeedKmh:F0}";
        DebugText.Print(speedStr, new Int2(cx - speedStr.Length * 4, cy - 10));
        DebugText.Print("km/h", new Int2(cx - 14, cy + 8));
        DebugText.Print($"G{Car.CurrentGear}", new Int2(cx - 36, cy + 44));
        DebugText.Print($"{Car.EngineRpm:F0}", new Int2(cx - 20, cy + 102));

        // Redline flash warning
        if (Car.EngineRpm > Car.MaxRpm * 0.92f)
            DebugText.Print("SHIFT!", new Int2(cx - 18, cy - 30));
    }
}
