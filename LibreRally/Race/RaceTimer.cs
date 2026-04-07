using System;
using Stride.Engine;

namespace LibreRally.Race;

[ComponentCategory("LibreRally")]
public class RaceTimer : SyncScript
{
    private double _elapsed;
    private bool _running;

    public bool IsRunning => _running;
    public double ElapsedSeconds => _elapsed;

    public void StartTimer() { _running = true; }
    public void StopTimer() { _running = false; }
    public void ResetTimer() { _running = false; _elapsed = 0; }

    public string GetTimeString()
    {
        int totalCs = (int)(_elapsed * 100);
        int cs = totalCs % 100;
        int totalSec = totalCs / 100;
        int secs = totalSec % 60;
        int mins = totalSec / 60;
        return $"{mins:D2}:{secs:D2}.{cs:D2}";
    }

    public override void Start() { }

    public override void Update()
    {
        if (_running)
            _elapsed += Game.UpdateTime.Elapsed.TotalSeconds;
    }
}
