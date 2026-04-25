using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using LibreRally.Vehicle;

namespace LibreRally.HUD
{
	/// <summary>
	/// Classifies the driver's verdict for one A/B tuning comparison prompt.
	/// </summary>
	public enum TuningSessionVerdict
	{
		/// <summary>The candidate setup felt better than the baseline for the current prompt.</summary>
		Better,

		/// <summary>The candidate setup felt worse than the baseline for the current prompt.</summary>
		Worse,

		/// <summary>The candidate setup felt meaningfully the same as the baseline for the current prompt.</summary>
		Same,

		/// <summary>The current prompt needs another run before a fair verdict can be made.</summary>
		NeedsMoreTime,
	}

	/// <summary>
	/// One structured prompt in a local A/B tuning session.
	/// </summary>
	/// <param name="Id">Stable prompt identifier.</param>
	/// <param name="Title">Short prompt title shown in the UI.</param>
	/// <param name="DrivingScript">Concrete on-road instructions for the test.</param>
	/// <param name="ComparisonQuestion">Question the driver should answer after the run.</param>
	/// <param name="TelemetryFocus">Which telemetry values matter most for the prompt.</param>
	public sealed record TuningSessionPrompt(
		string Id,
		string Title,
		string DrivingScript,
		string ComparisonQuestion,
		string TelemetryFocus);

	/// <summary>
	/// One recorded verdict in a tuning session.
	/// </summary>
	/// <param name="PromptId">Prompt identifier that this response answers.</param>
	/// <param name="Verdict">Driver verdict for the prompt.</param>
	/// <param name="Note">Optional short freeform note from the driver.</param>
	public sealed record TuningSessionResponse(
		string PromptId,
		TuningSessionVerdict Verdict,
		string Note);

	/// <summary>
	/// Local, structured A/B driving-session model used to compare baseline and candidate variants without an MCP server.
	/// </summary>
	public sealed class TuningSessionModel
	{
		private readonly Dictionary<string, TuningSessionResponse> _responses = new(StringComparer.OrdinalIgnoreCase);

		private TuningSessionModel(
			string vehicleName,
			string statusText,
			string baselineLabel,
			string candidateLabel,
			string telemetrySummary,
			IReadOnlyList<TuningSessionPrompt> prompts)
		{
			VehicleName = vehicleName;
			StatusText = statusText;
			BaselineLabel = baselineLabel;
			CandidateLabel = candidateLabel;
			TelemetrySummary = telemetrySummary;
			Prompts = prompts;
		}

		/// <summary>Gets the vehicle name shown for the session.</summary>
		public string VehicleName { get; }

		/// <summary>Gets the short status line describing the active comparison.</summary>
		public string StatusText { get; }

		/// <summary>Gets the baseline variant label.</summary>
		public string BaselineLabel { get; }

		/// <summary>Gets the candidate variant label.</summary>
		public string CandidateLabel { get; }

		/// <summary>Gets the troubleshooting or telemetry summary carried into the session.</summary>
		public string TelemetrySummary { get; }

		/// <summary>Gets the ordered list of structured prompts for the session.</summary>
		public IReadOnlyList<TuningSessionPrompt> Prompts { get; }

		/// <summary>Gets the current prompt index.</summary>
		public int CurrentPromptIndex { get; private set; }

		/// <summary>Gets the currently selected prompt.</summary>
		public TuningSessionPrompt CurrentPrompt => Prompts[Math.Clamp(CurrentPromptIndex, 0, Math.Max(Prompts.Count - 1, 0))];

		/// <summary>Gets the count of prompts that already have responses.</summary>
		public int CompletedPromptCount => _responses.Count;

		/// <summary>Gets the count of prompts still awaiting a recorded verdict.</summary>
		public int PendingPromptCount => Prompts.Count - CompletedPromptCount;

		/// <summary>Gets the recorded responses keyed by prompt identifier.</summary>
		public IReadOnlyDictionary<string, TuningSessionResponse> Responses => _responses;

		/// <summary>
		/// Creates a structured A/B tuning session for local better-or-worse driving comparisons.
		/// </summary>
		/// <param name="vehicleName">Display name of the active vehicle.</param>
		/// <param name="statusText">Short status line shown in the session header.</param>
		/// <param name="baselineLabel">Label for the baseline run or preset.</param>
		/// <param name="candidateLabel">Label for the candidate run or preset.</param>
		/// <param name="telemetrySummary">Optional short troubleshooting or telemetry summary shown alongside the prompts.</param>
		/// <returns>A new <see cref="TuningSessionModel"/>.</returns>
		public static TuningSessionModel Create(
			string vehicleName,
			string statusText,
			string baselineLabel,
			string candidateLabel,
			string? telemetrySummary = null)
		{
			var prompts = new List<TuningSessionPrompt>
			{
				new(
					"launch-grip",
					"Launch grip",
					"Use the same flat tarmac section. From a complete stop, apply full throttle in first gear and hold a straight wheel until 80 km/h or the braking marker.",
					$"Compared with {baselineLabel}, is {candidateLabel} better, worse, or the same for initial bite and wheelspin control?",
					"Watch launch speed, driven-wheel slip ratio, wheel omega, and per-wheel Fx."),
				new(
					"corner-entry",
					"Corner entry rotation",
					"Approach the same medium-speed corner at the same marker speed. Lift or trail brake with the same steering input and note how quickly the nose rotates into the apex.",
					$"Compared with {baselineLabel}, is {candidateLabel} better, worse, or the same for entry confidence and rotation?",
					"Watch front/rear load transfer, slip angle build-up, and braking stability."),
				new(
					"mid-corner-balance",
					"Mid-corner balance",
					"Hold a steady throttle through the middle of the same corner. Keep your line constant and judge whether the car settles or needs extra steering correction.",
					$"Compared with {baselineLabel}, is {candidateLabel} better, worse, or the same for steady-state balance?",
					"Watch lateral force balance, normal load split, and tyre-model response."),
				new(
					"power-down-traction",
					"Power-down traction",
					"At the same corner exit point, go to power in one clean application and hold it. Judge whether the car hooks up, spins the inside tyre, or bogs.",
					$"Compared with {baselineLabel}, is {candidateLabel} better, worse, or the same for exit traction and acceleration?",
					"Watch driven-wheel slip ratio, longitudinal force, RPM rise, and acceleration."),
			};

			return new TuningSessionModel(
				vehicleName,
				statusText,
				baselineLabel,
				candidateLabel,
				string.IsNullOrWhiteSpace(telemetrySummary) ? "No troubleshooting report attached." : telemetrySummary,
				prompts);
		}

		internal static TuningSessionModel CreateFromTroubleshootingReport(
			string vehicleName,
			string statusText,
			string baselineLabel,
			string candidateLabel,
			VehicleTroubleshootingReport report)
		{
			return Create(
				vehicleName,
				statusText,
				baselineLabel,
				candidateLabel,
				BuildTelemetrySummary(report));
		}

		/// <summary>
		/// Records or replaces the response for the current prompt.
		/// </summary>
		/// <param name="verdict">Driver verdict for the current prompt.</param>
		/// <param name="note">Optional short note from the driver.</param>
		public void RecordCurrentPromptResponse(TuningSessionVerdict verdict, string? note = null)
		{
			var prompt = CurrentPrompt;
			_responses[prompt.Id] = new TuningSessionResponse(prompt.Id, verdict, note?.Trim() ?? string.Empty);
		}

		/// <summary>
		/// Advances to the next prompt when possible.
		/// </summary>
		/// <returns><see langword="true"/> when the current prompt index changed; otherwise <see langword="false"/>.</returns>
		public bool MoveNextPrompt()
		{
			if (CurrentPromptIndex >= Prompts.Count - 1)
			{
				return false;
			}

			CurrentPromptIndex++;
			return true;
		}

		/// <summary>
		/// Moves back to the previous prompt when possible.
		/// </summary>
		/// <returns>
		/// <see langword="true"/> when the current prompt index changed to an earlier prompt;
		/// otherwise, <see langword="false"/>.
		/// </returns>
		public bool MovePreviousPrompt()
		{
			if (CurrentPromptIndex <= 0)
			{
				return false;
			}

			CurrentPromptIndex--;
			return true;
		}

		/// <summary>
		/// Builds a compact summary of the session responses for later review or persistence.
		/// </summary>
		/// <returns>Readable multi-line summary text.</returns>
		public string CreateSessionSummary()
		{
			var builder = new StringBuilder();
			builder.AppendLine($"{VehicleName}: {BaselineLabel} vs {CandidateLabel}");
			builder.AppendLine(StatusText);
			builder.AppendLine($"Telemetry: {TelemetrySummary}");
			foreach (var prompt in Prompts)
			{
				if (_responses.TryGetValue(prompt.Id, out var response))
				{
					builder.AppendLine($"{prompt.Title}: {response.Verdict}{FormatNote(response.Note)}");
				}
				else
				{
					builder.AppendLine($"{prompt.Title}: Pending");
				}
			}

			return builder.ToString();
		}

		private static string BuildTelemetrySummary(VehicleTroubleshootingReport report)
		{
			var observations = report.Observations.Take(2).ToArray();
			if (observations.Length == 0)
			{
				return "Troubleshooting report attached.";
			}

			return string.Join(" ", observations);
		}

		private static string FormatNote(string note)
		{
			return string.IsNullOrWhiteSpace(note) ? string.Empty : $" ({note})";
		}
	}
}
