using Stride.Engine;

namespace LibreRally.Race
{
	/// <summary>
	/// A timer component used to track race duration.
	/// </summary>
	[ComponentCategory("LibreRally")]
	public class RaceTimer : SyncScript
	{
		/// <summary>
		/// Gets a value indicating whether the timer is currently running.
		/// </summary>
		public bool IsRunning { get; private set; }

		/// <summary>
		/// Gets the elapsed time in seconds.
		/// </summary>
		public double ElapsedSeconds { get; private set; }

		/// <summary>
		/// Starts the timer.
		/// </summary>
		public void StartTimer() { IsRunning = true; }

		/// <summary>
		/// Stops the timer.
		/// </summary>
		public void StopTimer() { IsRunning = false; }

		/// <summary>
		/// Resets the timer to zero and stops it.
		/// </summary>
		public void ResetTimer() { IsRunning = false; ElapsedSeconds = 0; }

		/// <summary>
		/// Gets the formatted elapsed time as a string (MM:SS.cs).
		/// </summary>
		/// <returns>A string representing the elapsed time.</returns>
		public string GetTimeString()
		{
			var totalCs = (int)(ElapsedSeconds * 100);
			var cs = totalCs % 100;
			var totalSec = totalCs / 100;
			var secs = totalSec % 60;
			var mins = totalSec / 60;
			return $"{mins:D2}:{secs:D2}.{cs:D2}";
		}

		/// <summary>
		/// Initialises the timer component before the race begins.
		/// </summary>
		public override void Start() { }

		/// <summary>
		/// Advances the elapsed race time while the timer is running.
		/// </summary>
		public override void Update()
		{
			if (IsRunning)
			{
				ElapsedSeconds += Game.UpdateTime.Elapsed.TotalSeconds;
			}
		}
	}
}
