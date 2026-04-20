using System;
using Stride.Engine;

namespace LibreRally.Race;

/// <summary>
/// A timer component used to track race duration.
/// </summary>
[ComponentCategory("LibreRally")]
public class RaceTimer : SyncScript
{
    private double _elapsed;
    private bool _running;

    /// <summary>
    /// Gets a value indicating whether the timer is currently running.
    /// </summary>
    public bool IsRunning => _running;

    /// <summary>
    /// Gets the elapsed time in seconds.
    /// </summary>
    public double ElapsedSeconds => _elapsed;

    /// <summary>
    /// Starts the timer.
    /// </summary>
    public void StartTimer() { _running = true; }

    /// <summary>
    /// Stops the timer.
    /// </summary>
    public void StopTimer() { _running = false; }

    /// <summary>
    /// Resets the timer to zero and stops it.
    /// </summary>
    public void ResetTimer() { _running = false; _elapsed = 0; }

    /// <summary>
    /// Gets the formatted elapsed time as a string (MM:SS.cs).
    /// </summary>
    /// <returns>A string representing the elapsed time.</returns>
    public string GetTimeString()
    {
        var totalCs = (int)(_elapsed * 100);
        var cs = totalCs % 100;
        var totalSec = totalCs / 100;
        var secs = totalSec % 60;
        var mins = totalSec / 60;
        return $"{mins:D2}:{secs:D2}.{cs:D2}";
    }

    /// <inheritdoc/>
    public override void Start() { }

    /// <inheritdoc/>
    public override void Update()
    {
        if (_running)
        {
	        _elapsed += Game.UpdateTime.Elapsed.TotalSeconds;
        }
    }
}
