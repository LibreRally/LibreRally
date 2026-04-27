using System;

namespace LibreRally.Vehicle
{
	/// <summary>
	/// Represents a vehicle loading progress update, carrying a stage name and 0–1 progress value.
	/// </summary>
	public readonly struct VehicleLoadProgress
	{
		/// <summary>
		/// Gets the human-readable loading stage description.
		/// </summary>
		public string Stage { get; }

		/// <summary>
		/// Gets the loading progress as a fraction between 0 and 1.
		/// </summary>
		public float Progress { get; }

		/// <summary>
		/// Initializes a new instance of the <see cref="VehicleLoadProgress"/> struct with an explicit progress value.
		/// </summary>
		/// <param name="stage">The human-readable stage name.</param>
		/// <param name="progress">Progress fraction between 0 and 1.</param>
		public VehicleLoadProgress(string stage, float progress)
		{
			Stage = stage;
			Progress = Math.Clamp(progress, 0f, 1f);
		}

		/// <summary>
		/// Initializes a new instance of the <see cref="VehicleLoadProgress"/> struct from a completed/total count.
		/// </summary>
		/// <param name="stage">The human-readable stage name.</param>
		/// <param name="completed">The number of completed items.</param>
		/// <param name="total">The total number of items.</param>
		public VehicleLoadProgress(string stage, int completed, int total)
		{
			Stage = stage;
			Progress = total > 0 ? Math.Clamp((float)completed / total, 0f, 1f) : 0f;
		}
	}
}
