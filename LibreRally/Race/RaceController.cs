using System;
using System.Collections.Generic;
using Stride.Engine;

namespace LibreRally.Race;

[ComponentCategory("LibreRally")]
public class RaceController : SyncScript
{
    public RaceTimer? Timer { get; set; }
    public List<CheckpointTrigger> Checkpoints { get; set; } = new();
    public float CountdownDuration { get; set; } = 5f;

    private enum State { Countdown, Racing, Finished }
    private State _state = State.Countdown;
    private float _countdown;
    private int _nextCheckpoint;
    public List<double> SplitTimes { get; } = new();

    public bool IsRacing => _state == State.Racing;
    public float CountdownRemaining => MathF.Max(0f, _countdown);
    public int CountdownBeats => (int)MathF.Ceiling(_countdown);

    public event Action? RaceStarted;
    public event Action? RaceFinished;

    public override void Start()
    {
        _countdown = CountdownDuration;
        foreach (var cp in Checkpoints)
            cp.Triggered += OnCheckpointTriggered;
    }

    public override void Update()
    {
        float dt = (float)Game.UpdateTime.Elapsed.TotalSeconds;

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
        if (_state != State.Racing) return;
        if (index != _nextCheckpoint) return;

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
