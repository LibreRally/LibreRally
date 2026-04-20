using System;
using System.Collections.Generic;
using Stride.Engine;

namespace LibreRally.Race;

/// <summary>
/// Controls the flow of a race, including countdown, timing, and checkpoints.
/// </summary>
[ComponentCategory("LibreRally")]
public class RaceController : SyncScript
{
    /// <summary>
    /// Gets or sets the associated race timer.
    /// </summary>
    public RaceTimer? Timer { get; set; }

    /// <summary>
    /// Gets or sets the list of checkpoints for the race.
    /// </summary>
    public List<CheckpointTrigger> Checkpoints { get; set; } = new();

    /// <summary>
    /// Gets or sets the duration of the pre-race countdown in seconds.
    /// </summary>
    public float CountdownDuration { get; set; } = 5f;

    private enum State { Countdown, Racing, Finished }
    private State _state = State.Countdown;
    private float _countdown;
    private int _nextCheckpoint;

    /// <summary>
    /// Gets the split times recorded at each checkpoint.
    /// </summary>
    public List<double> SplitTimes { get; } = new();

    /// <summary>
    /// Gets a value indicating whether the race is currently in progress.
    /// </summary>
    public bool IsRacing => _state == State.Racing;

    /// <summary>
    /// Gets the remaining countdown time in seconds.
    /// </summary>
    public float CountdownRemaining => MathF.Max(0f, _countdown);

    /// <summary>
    /// Gets the rounded number of countdown beats remaining.
    /// </summary>
    public int CountdownBeats => (int)MathF.Ceiling(_countdown);

    /// <summary>
    /// Occurs when the race starts after the countdown finishes.
    /// </summary>
    public event Action? RaceStarted;

    /// <summary>
    /// Occurs when the race is finished after all checkpoints have been passed.
    /// </summary>
    public event Action? RaceFinished;

    /// <summary>
    /// Starts the race countdown and subscribes to checkpoint events.
    /// </summary>
    public override void Start()
    {
        _countdown = CountdownDuration;
        foreach (var cp in Checkpoints)
            cp.Triggered += OnCheckpointTriggered;
    }

    /// <summary>
    /// Advances the race countdown and monitors checkpoint completion.
    /// </summary>
    public override void Update()
    {
        var dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;

        if (_state == State.Countdown)
        {
            _countdown -= dt;
            if (_countdown <= 0f)
            {
                _state = State.Racing;
                Timer?.StartTimer();
                RaceStarted?.Invoke();
            }
        }
    }

    private void OnCheckpointTriggered(int index)
    {
        if (_state != State.Racing)
        {
	        return;
        }

        if (index != _nextCheckpoint)
        {
	        return;
        }

        SplitTimes.Add(Timer?.ElapsedSeconds ?? 0);
        _nextCheckpoint++;

        if (_nextCheckpoint >= Checkpoints.Count)
        {
            _state = State.Finished;
            Timer?.StopTimer();
            RaceFinished?.Invoke();
        }
    }
}
