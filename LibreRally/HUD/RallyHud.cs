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
        int y = 20;
        int x = 20;

        if (Car != null)
        {
            DebugText.Print($"Speed: {Car.SpeedKmh:F0} km/h", new Int2(x, y)); y += 20;
        }

        if (Timer != null)
        {
            DebugText.Print($"Time: {Timer.GetTimeString()}", new Int2(x, y)); y += 20;
        }

        if (RaceController != null)
        {
            if (!RaceController.IsRacing && RaceController.CountdownRemaining > 0)
                DebugText.Print($"GO IN: {RaceController.CountdownBeats}", new Int2(x, y));
        }
    }
}
